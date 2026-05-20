#ifdef USE_I2C
#ifdef USE_BMV080
/*********************************************************************************************\
 * BMV080 - Bosch particulate matter sensor (PM1, PM2.5, PM10) via I2C
 * Requires precompiled Bosch SDK libraries in lib/lib_i2c/BMV080/src/esp32/:
 *   lib_bmv080.a
 *   lib_postProcessor.a
\*********************************************************************************************/

#define XSNS_125             125
#define XI2C_95              95
#define BMV080_ADDR          0x54

#include <bmv080.h>
#include <bmv080_defs.h>

// --- Sercom Wrapper ---
struct BmvI2cSercom { uint8_t addr; uint8_t bus; };
static BmvI2cSercom g_sercom;

// --- State ---
struct BMV080_State {
  void*   handle = nullptr;
  uint8_t bus    = 0;
  bool    ready  = false;

  struct {
    float   integration_time = 10.0f;   // s
    uint8_t algo_id          = 4;       // 2=Fast,3=Balanced,4=HighPrecision
    bool    powered          = true;
  } cfg;

  uint32_t last_data_ms      = 0;
  uint32_t error_count       = 0;

  struct {
    uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
    bool  obstructed = false;
    bool  out_of_range = false;
    bool  valid = false;
  } last;
};
static BMV080_State BMV;

static inline bool BMV080_Active(void) {
  return BMV.ready && BMV.handle && BMV.cfg.powered;
}

static const char* BMV_AlgoName(uint8_t id) {
  switch (id) {
    case 2: return "FastResponse";
    case 3: return "Balanced";
    case 4: return "HighPrecision";
    default: return "Unknown";
  }
}

// --- I2C Bridge (static functions for Bosch API callbacks) ---
static int8_t BMV080_I2cRead(void *sercom_handle, uint16_t header, uint16_t *payload, uint16_t payload_length) {
  auto *sc = reinterpret_cast<BmvI2cSercom*>(sercom_handle);
  uint16_t hdr = (uint16_t)(header << 1);
  uint8_t  hb  = (uint8_t)(hdr >> 8);
  uint8_t  lb  = (uint8_t)(hdr & 0xFF);
  if (I2cWriteBuffer(sc->addr, hb, &lb, 1, sc->bus)) return -1;
  uint16_t nbytes = payload_length * 2;
  if (nbytes > 512) return -2;
  uint8_t raw[512];
  if (I2cReadBuffer(sc->addr, -1, raw, nbytes, sc->bus)) return -3;
  for (uint16_t i = 0; i < payload_length; i++) {
    payload[i] = ((uint16_t)raw[2*i] << 8) | raw[2*i + 1];
  }
  return 0;
}

static int8_t BMV080_I2cWrite(void *sercom_handle, uint16_t header, const uint16_t *payload, uint16_t payload_length) {
  auto *sc = reinterpret_cast<BmvI2cSercom*>(sercom_handle);
  uint16_t hdr = (uint16_t)(header << 1);
  uint8_t  hb  = (uint8_t)(hdr >> 8);
  uint8_t  lb  = (uint8_t)(hdr & 0xFF);
  uint16_t nbytes = 1 + payload_length * 2;
  if (nbytes > 255) return -1;
  uint8_t buf[255];
  buf[0] = lb;
  for (uint16_t i = 0; i < payload_length; i++) {
    buf[1 + 2*i]     = (uint8_t)(payload[i] >> 8);
    buf[1 + 2*i + 1] = (uint8_t)(payload[i] & 0xFF);
  }
  return I2cWriteBuffer(sc->addr, hb, buf, nbytes, sc->bus) ? -2 : 0;
}

static int8_t BMV080_I2cDelay(uint32_t ms) { delay(ms); return 0; }

static bmv080_callback_data_ready_t BMV080_DataReady = [](bmv080_output_t out, void*) {
  BMV.last.pm1          = (uint16_t)out.pm1_mass_concentration;
  BMV.last.pm25         = (uint16_t)out.pm2_5_mass_concentration;
  BMV.last.pm10         = (uint16_t)out.pm10_mass_concentration;
  BMV.last.obstructed   = out.is_obstructed;
  BMV.last.out_of_range = out.is_outside_measurement_range;
  BMV.last.valid        = true;
  BMV.last_data_ms      = millis();
};

// --- Bosch parameter helpers ---
static bool BMV080_SetAlgo(uint8_t algo_id) {
  if (!BMV.ready || !BMV.handle) return false;
  bmv080_measurement_algorithm_t algo;
  if      (algo_id == 2) algo = E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE;
  else if (algo_id == 3) algo = E_BMV080_MEASUREMENT_ALGORITHM_BALANCED;
  else if (algo_id == 4) algo = E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION;
  else return false;
  if (bmv080_set_parameter(BMV.handle, "measurement_algorithm", &algo) == E_BMV080_OK) {
    BMV.cfg.algo_id = algo_id;
    return true;
  }
  return false;
}

static bool BMV080_SetIntegrationTime(float seconds) {
  if (!BMV.ready || !BMV.handle) return false;
  if (seconds < 1.0f)  seconds = 1.0f;
  if (seconds > 60.0f) seconds = 60.0f;
  if (bmv080_set_parameter(BMV.handle, "integration_time", &seconds) == E_BMV080_OK) {
    BMV.cfg.integration_time = seconds;
    return true;
  }
  return false;
}

static void BMV080_ApplyConfig(void) {
  BMV080_SetAlgo(BMV.cfg.algo_id);
  BMV080_SetIntegrationTime(BMV.cfg.integration_time);
  float it = 0.0f;
  if (bmv080_get_parameter(BMV.handle, "integration_time", &it) == E_BMV080_OK && it > 0.0f) {
    BMV.cfg.integration_time = it;
  }
}

// --- Open / Close (shared init+poweron logic) ---
static bool BMV080_Open(void) {
  g_sercom.addr = BMV080_ADDR;
  g_sercom.bus  = BMV.bus;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    bmv080_status_code_t rc = bmv080_open(&BMV.handle, (bmv080_sercom_handle_t)&g_sercom,
                                          BMV080_I2cRead, BMV080_I2cWrite, BMV080_I2cDelay);
    if (rc != E_BMV080_OK) {
      AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: open failed (%d) attempt %d"), rc, attempt);
      delay(50); continue;
    }
    (void)bmv080_reset(BMV.handle);
    BMV080_ApplyConfig();
    rc = bmv080_start_continuous_measurement(BMV.handle);
    if (rc == E_BMV080_OK) {
      BMV.ready = true; BMV.cfg.powered = true; BMV.last.valid = false;
      BMV.last_data_ms = millis();
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: started, algo=%s, IntTime=%.0fs"),
             BMV_AlgoName(BMV.cfg.algo_id), BMV.cfg.integration_time);
      return true;
    }
    AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: start failed (%d) attempt %d"), rc, attempt);
    bmv080_close(&BMV.handle); BMV.handle = nullptr; delay(50);
  }
  return false;
}

static void BMV080_Close(void) {
  if (BMV.handle) {
    bmv080_stop_measurement(BMV.handle);
    bmv080_close(&BMV.handle);
    BMV.handle = nullptr;
  }
  BMV.ready = false;
}

// --- Power Control ---
static bool BMV080_PowerOff(void) {
  BMV080_Close();
  BMV.cfg.powered = false;
  AddLog(LOG_LEVEL_INFO, PSTR("BMV080: powered off"));
  return true;
}

static bool BMV080_PowerOn(void) {
  if (BMV080_Active()) return true;
  return BMV080_Open();
}

// --- Recovery ---
static void BMV080_Recovery(void) {
  if (!BMV.cfg.powered) return;
  if (millis() - BMV.last_data_ms <= 3000) return;
  AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: no data for %ums, recovering"), millis() - BMV.last_data_ms);
  BMV080_Close();
  BMV080_Open();
}

// --- Serve / Poll (250ms needed for bmv080_serve_interrupt) ---
static void BMV080_Every250ms(void) {
  if (!BMV080_Active()) return;
  bmv080_status_code_t rc = bmv080_serve_interrupt(BMV.handle, BMV080_DataReady, nullptr);
  if (rc != E_BMV080_OK) {
    BMV.error_count++;
    if (BMV.error_count <= 5 || (BMV.error_count % 50) == 0) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("BMV080: serve error %d (count=%u)"), rc, BMV.error_count);
    }
    // Soft reset on persistent HW read errors
    if (rc == 105 && BMV.error_count >= 10) {
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: HW_READ errors, soft reset"));
      if (bmv080_reset(BMV.handle) == E_BMV080_OK) {
        BMV080_ApplyConfig();
        if (bmv080_start_continuous_measurement(BMV.handle) == E_BMV080_OK) {
          BMV.last_data_ms = millis();
          BMV.error_count = 0;
          return;
        }
      }
      // Soft reset failed, do full recovery
      AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: soft reset failed, full recovery"));
      BMV080_Close();
      BMV080_Open();
      BMV.error_count = 0;
    }
  } else {
    BMV.error_count = 0;
  }
  BMV080_Recovery();
}

// --- JSON / Web ---
static void BMV080_Show(bool json) {
  if (!BMV080_Active() || !BMV.last.valid) return;
  if (json) {
    ResponseAppend_P(PSTR(",\"BMV080\":{\"PM1\":%u,\"PM2.5\":%u,\"PM10\":%u,"
                          "\"Obstruct\":%u,\"OutOfRange\":%u}"),
                     BMV.last.pm1, BMV.last.pm25, BMV.last.pm10,
                     BMV.last.obstructed, BMV.last.out_of_range);
  }
#ifdef USE_WEBSERVER
  else {
    WSContentSend_P(PSTR("{s}BMV080 Algo{m}%s{e}"), BMV_AlgoName(BMV.cfg.algo_id));
    WSContentSend_P(PSTR("{s}BMV080 IntTime{m}%us{e}"), (uint16_t)BMV.cfg.integration_time);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "1",   BMV.last.pm1);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "2.5", BMV.last.pm25);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "10",  BMV.last.pm10);
    WSContentSend_P(PSTR("{s}BMV080 Obstruct{m}%s{e}"), BMV.last.obstructed ? PSTR("true") : PSTR("false"));
    WSContentSend_P(PSTR("{s}BMV080 OutOfRange{m}%s{e}"), BMV.last.out_of_range ? PSTR("true") : PSTR("false"));
  }
#endif
}

// --- Command ---
static bool BMV080_SelectMode(uint16_t mode) {
  if (mode > 999) {
    float sec = (float)(mode / 1000);
    if (sec < 1.0f)  sec = 1.0f;
    if (sec > 60.0f) sec = 60.0f;
    if (!BMV080_Active()) {
      BMV.cfg.integration_time = sec;
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Preset IntTime to %.0fs (apply on init)"), sec);
      return true;
    }
    return BMV080_SetIntegrationTime(sec);
  }

  switch (mode) {
    case 0: return BMV080_PowerOff();
    case 1: return BMV080_PowerOn();
    case 2: case 3: case 4:
      if (!BMV080_Active()) { BMV.cfg.algo_id = (uint8_t)mode; return true; }
      return BMV080_SetAlgo((uint8_t)mode);
    case 10:
      if (!BMV080_Active()) return false;
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Soft reset"));
      if (bmv080_reset(BMV.handle) == E_BMV080_OK) {
        BMV080_ApplyConfig();
        if (bmv080_start_continuous_measurement(BMV.handle) == E_BMV080_OK) {
          BMV.last_data_ms = millis();
          return true;
        }
      }
      return false;
    case 11:
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Hard reinit"));
      BMV080_Close();
      return BMV080_Open();
    default: return false;
  }
}

static bool BMV080Cmd(void) {
  if (XdrvMailbox.data_len > 0 && XdrvMailbox.data) {
    if ((XdrvMailbox.data[0] == '?' && XdrvMailbox.data[1] == '\0') ||
        !strcasecmp_P(XdrvMailbox.data, PSTR("help"))) {
      Response_P(PSTR("{\"BMV080_help\":{"
                      "\"0\":\"Power Off\",\"1\":\"Power On\","
                      "\"2\":\"Fast Response\",\"3\":\"Balanced\",\"4\":\"High Precision\","
                      "\">999\":\"Set Integration Time (sec=value/1000)\","
                      "\"10\":\"Soft Reset\",\"11\":\"Hard Reinit\"}}"));
      return true;
    }
    uint16_t mode = (uint16_t)atoi(XdrvMailbox.data);
    bool ok = BMV080_SelectMode(mode);
    Response_P(PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.command, XdrvMailbox.data);
    return ok;
  }
  Response_P(PSTR("{\"BMV080\":{\"Power\":%s,\"Mode\":\"Continuous\",\"Algo\":\"%s\",\"IntTime\":%u}}"),
             BMV.cfg.powered ? PSTR("true") : PSTR("false"),
             BMV_AlgoName(BMV.cfg.algo_id),
             (uint16_t)BMV.cfg.integration_time);
  return true;
}

// --- Detect ---
static void BMV080_Detect(void) {
  for (uint8_t bus = 0; bus < 2; bus++) {
    if (!I2cSetDevice(BMV080_ADDR, bus)) continue;
    BMV.bus = bus;
    if (BMV080_Open()) { I2cSetActiveFound(BMV080_ADDR, "BMV080", bus); break; }
  }
}

// --- Dispatcher ---
bool Xsns125(uint32_t function) {
  if (!I2cEnabled(XI2C_95)) return false;
  switch (function) {
    case FUNC_INIT:               BMV080_Detect();        break;
    case FUNC_EVERY_250_MSECOND:  BMV080_Every250ms();    break;
    case FUNC_JSON_APPEND:        BMV080_Show(true);      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:         BMV080_Show(false);     break;
#endif
    case FUNC_COMMAND_SENSOR:
      if (XSNS_125 == XdrvMailbox.index) return BMV080Cmd();
      break;
  }
  return false;
}

#endif // USE_BMV080
#endif // USE_I2C
