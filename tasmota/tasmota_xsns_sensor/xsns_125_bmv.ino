#ifdef USE_I2C
#ifdef USE_BMV080

#define XSNS_125             125
#define XI2C_95              95
#define BMV080_ADDR          0x54
#define BMV080_SERVE_PERIOD  1

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
  uint32_t recover_attempts  = 0;

  struct {
    uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
    bool  obstructed = false;
    bool  out_of_range = false;
    float runtime_s = 0;
    bool  valid = false;
  } last;
};
static BMV080_State BMV;

// --- Utils (keine Bosch-Typen in Signaturen!) ---
static const char* BMV_AlgoNameId(uint8_t id) {
  switch (id) {
    case 2: return "FastResponse";
    case 3: return "Balanced";
    case 4: return "HighPrecision";
    default: return "Unknown";
  }
}

// --- I2C Bridge ---
class BMV080_Driver {
public:
  static int8_t Read(void *sercom_handle, uint16_t header, uint16_t *payload, uint16_t payload_length) {
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

  static int8_t Write(void *sercom_handle, uint16_t header, const uint16_t *payload, uint16_t payload_length) {
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

  static int8_t Delay(uint32_t ms) { delay(ms); return 0; }

  static void DataReady(bmv080_output_t out, void*) {
    BMV.last.runtime_s    = out.runtime_in_sec;
    BMV.last.pm1          = (uint16_t)out.pm1_mass_concentration;
    BMV.last.pm25         = (uint16_t)out.pm2_5_mass_concentration;
    BMV.last.pm10         = (uint16_t)out.pm10_mass_concentration;
    BMV.last.obstructed   = out.is_obstructed;
    BMV.last.out_of_range = out.is_outside_measurement_range;
    BMV.last.valid        = true;
    BMV.last_data_ms      = millis();
    //AddLog(LOG_LEVEL_INFO, PSTR("BMV080: t=%.1fs PM1=%.2f PM2.5=%.2f PM10=%.2f obstruct=%u out_of_range=%u"),
    //  BMV.last.runtime_s, BMV.last.pm1, BMV.last.pm25, BMV.last.pm10,
    //  BMV.last.obstructed, BMV.last.out_of_range);
  }
};

// --- Bosch-Parameter setzen/lesen (intern mappen) ---
static bool BMV080_SetAlgoOnce_(uint8_t algo_id) {
  if (!BMV.ready || !BMV.handle) return false;
  bmv080_measurement_algorithm_t algo = E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION;
  if      (algo_id == 2) algo = E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE;
  else if (algo_id == 3) algo = E_BMV080_MEASUREMENT_ALGORITHM_BALANCED;
  else if (algo_id == 4) algo = E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION;
  else return false;
  bmv080_status_code_t rc = bmv080_set_parameter(BMV.handle, "measurement_algorithm", &algo);
  if (rc == E_BMV080_OK) { BMV.cfg.algo_id = algo_id; return true; }
  return false;
}

static bool BMV080_ApplyIntegrationTime(float seconds) {
  if (!BMV.ready || !BMV.handle) return false;
  if (seconds < 1.0f)  seconds = 1.0f;
  if (seconds > 60.0f) seconds = 60.0f;
  float val = seconds;
  bmv080_status_code_t rc = bmv080_set_parameter(BMV.handle, "integration_time", &val);
  if (rc == E_BMV080_OK) { BMV.cfg.integration_time = seconds; return true; }
  return false;
}

static bool BMV080_ReadIntegrationTime(void) {
  if (!BMV.ready || !BMV.handle) return false;
  float it = 0.0f;
  bmv080_status_code_t rc = bmv080_get_parameter(BMV.handle, "integration_time", &it);
  if (rc == E_BMV080_OK && it > 0.0f) { BMV.cfg.integration_time = it; return true; }
  return false;
}

static void BMV080_ApplyConfigOnce(void) {
  (void)BMV080_SetAlgoOnce_(BMV.cfg.algo_id);
  (void)BMV080_ApplyIntegrationTime(BMV.cfg.integration_time);
}

// --- Power Control ---
static bool BMV080_PowerOff(void) {
  if (!BMV.ready || !BMV.handle) { BMV.cfg.powered = false; return true; }
  bmv080_status_code_t rc = bmv080_stop_measurement(BMV.handle);
  bmv080_close(&BMV.handle);
  BMV.handle = nullptr;
  BMV.ready  = false;
  BMV.cfg.powered = false;
  AddLog(LOG_LEVEL_INFO, PSTR("BMV080: powered off"));
  return (rc == E_BMV080_OK);
}

static bool BMV080_PowerOn(void) {
  if (BMV.ready && BMV.cfg.powered) return true;
  g_sercom.addr = BMV080_ADDR;
  g_sercom.bus  = BMV.bus;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    bmv080_status_code_t rc = bmv080_open(&BMV.handle, (bmv080_sercom_handle_t)&g_sercom,
                                          BMV080_Driver::Read, BMV080_Driver::Write, BMV080_Driver::Delay);
    if (rc != E_BMV080_OK) { AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: open failed (%d) (power on #%d)"), rc, attempt); delay(50); continue; }
    (void)bmv080_reset(BMV.handle);
    BMV080_ApplyConfigOnce();
    rc = bmv080_start_continuous_measurement(BMV.handle);
    if (rc == E_BMV080_OK) {
      BMV.ready = true; BMV.cfg.powered = true; BMV.last.valid = false; BMV.last_data_ms = millis();
      (void)BMV080_ReadIntegrationTime();
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: power on, algo=%s, IntTime=%.0fs"),
             BMV_AlgoNameId(BMV.cfg.algo_id), BMV.cfg.integration_time);
      return true;
    }
    AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: start failed (%d) (power on #%d)"), rc, attempt);
    bmv080_close(&BMV.handle); BMV.handle = nullptr; delay(50);
  }
  return false;
}

// --- Init / Detect / Recovery ---
static bool BMV080_Init(uint8_t addr, uint8_t bus) {
  g_sercom.addr = addr; g_sercom.bus  = bus;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    bmv080_status_code_t rc = bmv080_open(&BMV.handle, (bmv080_sercom_handle_t)&g_sercom,
                                          BMV080_Driver::Read, BMV080_Driver::Write, BMV080_Driver::Delay);
    if (rc != E_BMV080_OK) { AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: open failed (%d) (attempt %d)"), rc, attempt); delay(50); continue; }
    (void)bmv080_reset(BMV.handle);
    BMV080_ApplyConfigOnce();
    rc = bmv080_start_continuous_measurement(BMV.handle);
    if (rc == E_BMV080_OK) {
      BMV.bus = bus; BMV.ready = true; BMV.cfg.powered = true; BMV.last.valid = false; BMV.last_data_ms = millis();
      (void)BMV080_ReadIntegrationTime();
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Continuous, Algo=%s, IntTime=%.0fs"),
             BMV_AlgoNameId(BMV.cfg.algo_id), BMV.cfg.integration_time);
      return true;
    }
    AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: start failed (%d) (attempt %d)"), rc, attempt);
    bmv080_close(&BMV.handle); BMV.handle = nullptr; delay(50);
  }
  return false;
}

static void BMV080_Recovery(void) {
  if (!BMV.cfg.powered) return;
  uint32_t now = millis();
  uint32_t no_data_ms = now - BMV.last_data_ms;
  if (no_data_ms > 3000) {
    AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: no data %ums, errors=%u → recover"), no_data_ms, BMV.error_count);
    AddLog(LOG_LEVEL_INFO,  PSTR("BMV080: soft recover (reset+restart)"));
    if (bmv080_reset(BMV.handle) == E_BMV080_OK) {
      BMV080_ApplyConfigOnce();
      if (bmv080_start_continuous_measurement(BMV.handle) == E_BMV080_OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("BMV080: soft recover OK"));
        BMV.last_data_ms = now; return;
      }
    }
    AddLog(LOG_LEVEL_INFO, PSTR("BMV080: soft recover failed, doing hard recover"));
    bmv080_close(&BMV.handle); BMV.handle = nullptr;
    if (BMV080_Init(BMV080_ADDR, BMV.bus)) {
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: hard recover OK (attempts=%u)"), ++BMV.recover_attempts);
    } else {
      AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: hard recover FAILED (attempts=%u)"), ++BMV.recover_attempts);
    }
  }
}


// --- Serve / Poll (alle ~250ms) ---
static void BMV080_Every250ms(void) {
  if (!BMV.ready || !BMV.handle || !BMV.cfg.powered) return;
  bmv080_status_code_t rc = bmv080_serve_interrupt(BMV.handle, BMV080_Driver::DataReady, nullptr);
  if (rc != E_BMV080_OK) {
    BMV.error_count++;
    AddLog(LOG_LEVEL_DEBUG, PSTR("BMV080: serve_interrupt error %d (count=%u)"), rc, BMV.error_count);
  }
  BMV080_Recovery();
}

// --- JSON / Web ---
static void BMV080_Show(bool json) {
  if (!BMV.ready || !BMV.cfg.powered || !BMV.last.valid) return;
  if (json) {
    ResponseAppend_P(PSTR(",\"BMV080\":{\"PM1\":%u,\"PM2_5\":%u,\"PM10\":%u,"
                          "\"Obstruct\":%u,\"OutOfRange\":%u}"),
                     BMV.last.pm1, BMV.last.pm25, BMV.last.pm10,
                     BMV.last.obstructed, BMV.last.out_of_range);
  }
#ifdef USE_WEBSERVER
  else {
    WSContentSend_P(PSTR("<tr><th>BMV080</th><td>Power</td><td>%s</td></tr>"),
      BMV.cfg.powered ? PSTR("true") : PSTR("false"));
    WSContentSend_P(PSTR("<tr><th>BMV080</th><td>Algo</td><td>%s</td></tr>"),
      BMV_AlgoNameId(BMV.cfg.algo_id));
    WSContentSend_P(PSTR("<tr><th>BMV080</th><td>IntTime</td><td>%us</td></tr>"),
      (uint16_t)BMV.cfg.integration_time);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "1",   BMV.last.pm1);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "2.5", BMV.last.pm25);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "10",  BMV.last.pm10);
    WSContentSend_P(PSTR("<tr><th>BMV080</th><td>Obstruct</td><td>%s</td></tr>"),
      BMV.last.obstructed ? PSTR("true") : PSTR("false"));
    WSContentSend_P(PSTR("<tr><th>BMV080</th><td>OutOfRange</td><td>%s</td></tr>"),
      BMV.last.out_of_range ? PSTR("true") : PSTR("false"));
  }
#endif
}

// --- Command ---
static bool BMV080_SelectMode(uint16_t mode) {
  if (mode > 999) {
    float sec = (float)(mode / 1000);
    if (sec < 1.0f)  sec = 1.0f;
    if (sec > 60.0f) sec = 60.0f;
    if (!BMV.ready || !BMV.handle || !BMV.cfg.powered) {
      BMV.cfg.integration_time = sec;
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Preset IntTime to %.0fs (apply on init)"), sec);
      return true;
    }
    bool ok = BMV080_ApplyIntegrationTime(sec);
    AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Set IntTime to %.0fs %s"), sec, ok ? PSTR("OK") : PSTR("ERR"));
    return ok;
  }

  switch (mode) {
    case 0: return BMV080_PowerOff();
    case 1: return BMV080_PowerOn();
    case 2: case 3: case 4:
      if (!BMV.ready || !BMV.handle || !BMV.cfg.powered) { BMV.cfg.algo_id = (uint8_t)mode; return true; }
      return BMV080_SetAlgoOnce_((uint8_t)mode);
    case 10:
      if (!BMV.ready || !BMV.handle || !BMV.cfg.powered) return false;
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Soft reset"));
      if (bmv080_reset(BMV.handle) == E_BMV080_OK) {
        BMV080_ApplyConfigOnce();
        if (bmv080_start_continuous_measurement(BMV.handle) == E_BMV080_OK) { BMV.last_data_ms = millis(); return true; }
      }
      return false;
    case 11:
      AddLog(LOG_LEVEL_INFO, PSTR("BMV080: Hard reinit"));
      if (BMV.ready && BMV.handle) { bmv080_close(&BMV.handle); BMV.handle = nullptr; BMV.ready = false; }
      return BMV080_Init(BMV080_ADDR, BMV.bus);
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
  if (BMV.ready && BMV.handle) { (void)BMV080_ReadIntegrationTime(); }
  Response_P(PSTR("{\"BMV080\":{\"Power\":%s,\"Mode\":\"Continuous\",\"Algo\":\"%s\",\"IntTime\":%u}}"),
             BMV.cfg.powered ? PSTR("true") : PSTR("false"),
             BMV_AlgoNameId(BMV.cfg.algo_id),
             (uint16_t)BMV.cfg.integration_time);
  return true;
}

// --- Detect ---
static void BMV080_Detect(void) {
  for (uint8_t bus = 0; bus < 2; bus++) {
    if (!I2cSetDevice(BMV080_ADDR, bus)) continue;
    if (BMV080_Init(BMV080_ADDR, bus)) { I2cSetActiveFound(BMV080_ADDR, "BMV080", bus); break; }
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
