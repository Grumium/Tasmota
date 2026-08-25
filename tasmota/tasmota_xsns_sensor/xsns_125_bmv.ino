/*
  xsns_125_bmv.ino - Bosch BMV080 particulate matter sensor support for Tasmota

  Copyright (C) 2026  Jan-David Förster

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_BMV080
/*********************************************************************************************\
 * BMV080 - Bosch particulate matter sensor (PM1, PM2.5, PM10) via I2C
 *
 * Needs the precompiled Bosch SDK for the target arch in
 * lib/lib_i2c/BMV080/src/<arch>/ : lib_bmv080.a, lib_postProcessor.a
 *
 * Two traps, both found the hard way:
 *  - bmv080_serve_interrupt() needs ~12 kB of stack, far more than Tasmota's
 *    loop task has, so all SDK calls run in a worker task (BMV080_Task).
 *  - The I2C callbacks must return bmv080_status_code_t values; negative
 *    error codes make the SDK reject the chip id with status 107.
 *
 * Commands (comma separated arguments, [] = optional). Idle until started:
 *   Sensor125                 status
 *   Sensor125 0               stop
 *   Sensor125 1[,int]         continuous, integration time 1..60 s
 *   Sensor125 2[,on[,period]] duty cycling, ON time and period 3..900 s
 *   Sensor125 3|4|5           algorithm Fast|Balanced|HighPrecision,
 *                             only while continuous is running
 *   Sensor125 9               reopen
 *
 * Duty cycling saves power by sleeping between windows: one sample per period
 * instead of one per second, and the algorithm is fixed to FastResponse
 * (Bosch datasheet). The SDK requires period >= integration time + 2 s; a
 * command breaking that gets adjusted (explicit period wins) and logged.
\*********************************************************************************************/

#define XSNS_125             125
#define XI2C_111            111        // See I2CDEVICES.md
#define BMV080_ADDR          0x54
// An obstructed sensor makes the SDK drain its FIFO in blocks of up to ~368
// bytes, well past the 128 byte Wire default, so enlarge the buffer at open().
#define BMV080_WIRE_BUF      512
// Measured peak of bmv080_serve_interrupt(): 12376 bytes. 12 kB tripped the
// stack canary; do not lower without re-measuring the high water mark.
#define BMV080_TASK_STACK    24576

#include <bmv080.h>
#include <bmv080_defs.h>

// 1 = start measuring at boot instead of waiting for "Sensor125 1"/"2".
#ifndef BMV080_AUTOSTART
#define BMV080_AUTOSTART 0
#endif

// The SDK stores this pointer and hands it back to the callbacks, so the object
// must be word aligned and must outlive the handle.
struct __attribute__((aligned(4))) BmvI2cSercom { uint32_t addr; uint32_t bus; };
static BmvI2cSercom g_sercom;

struct BMV080_State {
  void*   handle = nullptr;
  uint8_t bus    = 0;
  bool    ready    = false;
  bool    detected = false;   // I2C address answered at boot

  struct {
    float    integration_time = 10.0f;  // s; in duty cycling this is the ON time
    uint16_t duty_period      = 30;     // s; full cycle = ON time + sleep
    uint8_t  algo_id          = 4;      // driver ids: 2=Fast,3=Balanced,4=HighPrecision
                                        // (mapped to SDK enum 1/2/3 in BMV080_ApplyConfig)
    bool     duty_cycling     = false;  // false = continuous
    bool     powered          = true;
  } cfg;

  uint32_t last_data_ms      = 0;
  uint32_t error_count       = 0;
  uint16_t warn_count        = 0;
  uint32_t fifo_drain        = 0;   // extra serve calls spent draining the FIFO
  size_t   wire_buf          = 0;   // actual Wire buffer size in bytes
  uint8_t  ioerr = 0;               // rate limit for I2C error logs
  uint8_t  recover_tries = 0;
  TaskHandle_t task = nullptr;
  volatile bool req_open = false, req_close = false;

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

/*********************************************************************************************\
 * I2C bridge - callbacks handed to the Bosch SDK
\*********************************************************************************************/

// Rate limited I2C error log: the first few failures are the informative ones.
static void BMV080_IoErr(const char *what, uint16_t header, uint32_t detail) {
  if (BMV.ioerr >= 8) return;
  BMV.ioerr++;
  AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: %s hdr=%04X (%u)"), what, header, detail);
}

// Address phase, shared by read and write. The wire format matches the DFRobot
// integration: header << 1, high byte first, then a STOP.
static void BMV080_SendAddr(TwoWire &w, uint16_t header) {
  uint16_t hdr = (uint16_t)(header << 1);
  w.beginTransmission((uint8_t)g_sercom.addr);
  w.write((uint8_t)(hdr >> 8));
  w.write((uint8_t)(hdr & 0xFF));
}

static int8_t BMV080_I2cRead(void *sercom_handle, uint16_t header, uint16_t *payload, uint16_t payload_length) {
  if (sercom_handle != &g_sercom || !payload) return E_BMV080_ERROR_NULLPTR;
  if (!payload_length) return E_BMV080_ERROR_HW_READ;

  TwoWire& w = I2cGetWire(g_sercom.bus);
  if (&w == nullptr) return E_BMV080_ERROR_HW_READ;

  BMV080_SendAddr(w, header);
  uint8_t erc = w.endTransmission(true);
  if (erc) { BMV080_IoErr(PSTR("RD addr NAK"), header, erc); return E_BMV080_ERROR_HW_READ; }

  // Chunked because an obstructed sensor makes the SDK ask for far more than
  // one transaction holds. Big endian on the wire: high byte first.
  size_t   cap   = BMV.wire_buf ? BMV.wire_buf : 128;   // before setBufferSize
  uint16_t max_w = (uint16_t)((cap > 254 ? 254 : cap) / 2);
  for (uint16_t done = 0; done < payload_length; ) {
    uint16_t n = payload_length - done;
    if (n > max_w) { n = max_w; }
    if (w.requestFrom((uint16_t)g_sercom.addr, (uint8_t)(n * 2), true) != (size_t)(n * 2)) {
      BMV080_IoErr(PSTR("RD short"), header, payload_length);
      return E_BMV080_ERROR_HW_READ;
    }
    while (n--) {
      uint8_t hi = w.read();
      payload[done++] = ((uint16_t)hi << 8) | (uint8_t)w.read();
    }
  }
  return E_BMV080_OK;
}

static int8_t BMV080_I2cWrite(void *sercom_handle, uint16_t header, const uint16_t *payload, uint16_t payload_length) {
  if (sercom_handle != &g_sercom || !payload) return E_BMV080_ERROR_NULLPTR;

  // A write cannot be split without losing the register context, so it has to
  // fit one transaction.
  uint32_t bytes = (uint32_t)payload_length * 2 + 2;
  size_t   cap   = BMV.wire_buf ? BMV.wire_buf : 128;
  if (bytes > cap) { BMV080_IoErr(PSTR("WR too long"), header, bytes); return E_BMV080_ERROR_HW_WRITE; }

  TwoWire& w = I2cGetWire(g_sercom.bus);
  if (&w == nullptr) return E_BMV080_ERROR_HW_WRITE;

  BMV080_SendAddr(w, header);
  for (uint16_t i = 0; i < payload_length; i++) {
    w.write((uint8_t)(payload[i] >> 8));         // big endian, see BMV080_I2cRead
    w.write((uint8_t)(payload[i] & 0xFF));
  }
  uint8_t erc = w.endTransmission();
  if (erc) { BMV080_IoErr(PSTR("WR failed"), header, erc); return E_BMV080_ERROR_HW_WRITE; }
  return E_BMV080_OK;
}

static int8_t BMV080_I2cDelay(uint32_t ms) { delay(ms); return 0; }

// Duty cycling lets the SDK time the sleep phases itself, so it needs a tick.
static uint32_t BMV080_Tick(void) { return millis(); }

// The SDK reports mass concentrations as float, but the sensor only resolves
// whole ug/m3 -- measured over many samples, every value was an exact integer
// (runtime_in_sec by contrast does carry fractions). The uint16 cast is
// therefore lossless.
static bmv080_callback_data_ready_t BMV080_DataReady = [](bmv080_output_t out, void*) {
  BMV.last.pm1          = (uint16_t)out.pm1_mass_concentration;
  BMV.last.pm25         = (uint16_t)out.pm2_5_mass_concentration;
  BMV.last.pm10         = (uint16_t)out.pm10_mass_concentration;
  if (out.is_obstructed != BMV.last.obstructed) {
    AddLog(LOG_LEVEL_INFO, PSTR("BMV080: obstruction %s"),
           out.is_obstructed ? PSTR("detected") : PSTR("cleared"));
  }
  BMV.last.obstructed   = out.is_obstructed;
  BMV.last.out_of_range = out.is_outside_measurement_range;
  BMV.last.valid        = true;
  BMV.last_data_ms      = millis();
};

// Parameters must be applied while the handle is open and before
// start_continuous_measurement (see bmv080.h).
static void BMV080_ApplyConfig(void) {
  if (!BMV.ready || !BMV.handle) return;

  // Duty cycling supports only the fast response algorithm -- stated both in
  // the parameter table of bmv080.h and in the Bosch datasheet. Force it here
  // so the reported algorithm matches what the sensor actually runs.
  bmv080_measurement_algorithm_t algo;
  if (BMV.cfg.duty_cycling) {
    algo = E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE;
  } else {
    switch (BMV.cfg.algo_id) {                   // driver id -> SDK enum
      case 2:  algo = E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE;  break;
      case 3:  algo = E_BMV080_MEASUREMENT_ALGORITHM_BALANCED;       break;
      default: algo = E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION; break;
    }
  }
  bmv080_set_parameter(BMV.handle, "measurement_algorithm", &algo);

  float it = BMV.cfg.integration_time;
  bmv080_set_parameter(BMV.handle, "integration_time", &it);
  if (bmv080_get_parameter(BMV.handle, "integration_time", &it) == E_BMV080_OK && it > 0.0f) {
    BMV.cfg.integration_time = it;                // what the sensor accepted
  }

  if (BMV.cfg.duty_cycling) {
    uint16_t dp = BMV.cfg.duty_period;
    bmv080_set_parameter(BMV.handle, "duty_cycling_period", &dp);
    if (bmv080_get_parameter(BMV.handle, "duty_cycling_period", &dp) == E_BMV080_OK && dp > 0) {
      BMV.cfg.duty_period = dp;
    }
  }
}

/********************************************************************************************/

static bool BMV080_Open(void) {
  g_sercom.addr = BMV080_ADDR;
  g_sercom.bus  = BMV.bus;
  BMV.ioerr = 0;

  // Room for the large FIFO reads an obstructed sensor triggers.
  TwoWire& w = I2cGetWire(BMV.bus);
  if (&w != nullptr) { BMV.wire_buf = w.setBufferSize(BMV080_WIRE_BUF); }

  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    bmv080_status_code_t rc = bmv080_open(&BMV.handle, (bmv080_sercom_handle_t)&g_sercom,
                                          BMV080_I2cRead, BMV080_I2cWrite, BMV080_I2cDelay);
    if (E_BMV080_OK == rc) {
      bmv080_reset(BMV.handle);
      BMV.ready = true;                          // ApplyConfig checks this
      BMV080_ApplyConfig();                      // must precede start (bmv080.h)
      rc = BMV.cfg.duty_cycling
             ? bmv080_start_duty_cycling_measurement(BMV.handle, BMV080_Tick,
                                                     E_BMV080_DUTY_CYCLING_MODE_0)
             : bmv080_start_continuous_measurement(BMV.handle);
      if (E_BMV080_OK == rc) {
        BMV.cfg.powered = true;
        BMV.last.valid = false;
        BMV.last_data_ms = millis();
        BMV.recover_tries = 0;
        if (BMV.cfg.duty_cycling) {
          AddLog(LOG_LEVEL_INFO, PSTR("BMV080: started duty cycling, %us of %us"),
                 (uint16_t)BMV.cfg.integration_time, BMV.cfg.duty_period);
        } else {
          AddLog(LOG_LEVEL_INFO, PSTR("BMV080: started continuous, %us, algo %s"),
                 (uint16_t)BMV.cfg.integration_time, BMV_AlgoName(BMV.cfg.algo_id));
        }
        return true;
      }
      BMV.ready = false;
    }
    AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: start failed (%d) attempt %d"), rc, attempt);
    if (BMV.handle) { bmv080_close(&BMV.handle); BMV.handle = nullptr; }
    delay(50);
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

static void BMV080_StartTask(void);              // defined below, owns all SDK calls

static bool BMV080_PowerOff(void) {
  if (BMV.task) { BMV.req_close = true; }        // let the task do the SDK call
  else { BMV080_Close(); BMV.cfg.powered = false; }
  AddLog(LOG_LEVEL_INFO, PSTR("BMV080: powered off"));
  return true;
}

static bool BMV080_PowerOn(void) {
  if (BMV080_Active()) return true;
  BMV.cfg.powered = true;                        // Detect may have parked it idle
  BMV.recover_tries = 0;
  BMV080_StartTask();
  BMV.req_open = true;                           // the task owns all SDK calls
  return (BMV.task != nullptr);
}

static void BMV080_Recovery(void) {
  if (!BMV.cfg.powered) return;
  // Continuous mode delivers a sample every second; duty cycling only once per
  // period, so the dead-sensor timeout has to follow the configured rhythm.
  uint32_t timeout = BMV.cfg.duty_cycling
                       ? ((uint32_t)BMV.cfg.duty_period * 1000) + 5000
                       : 3000;
  if (millis() - BMV.last_data_ms <= timeout) return;
  // Give up after a few attempts instead of hammering the bus forever.
  if (BMV.recover_tries >= 3) {
    if (BMV.recover_tries == 3) {
      BMV.recover_tries++;
      AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: recovery gave up, staying idle"));
      BMV080_Close();
      BMV.cfg.powered = false;
    }
    return;
  }
  BMV.recover_tries++;
  AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: no data for %ums, recovering"), millis() - BMV.last_data_ms);
  BMV080_Close();
  BMV080_Open();
}

// The Bosch SDK needs ~4.5 kB of stack per bmv080_serve_interrupt() call, which
// overflows Tasmota's 8 kB loop task (measured: 4448 -> 48 bytes free). All SDK
// calls therefore run in a dedicated task with its own generous stack.
// How to treat a status code from bmv080_serve_interrupt().
// Codes 1..4 and 208/209 are warnings, 100+ are errors (see bmv080_defs.h).
// Plain integers in the signature: the Arduino preprocessor hoists function
// prototypes to the top of the combined sketch, where neither a custom enum
// nor bmv080_status_code_t is known yet. int32_t works for both.
#define BMV_SERVE_OK       0   // nothing to do
#define BMV_SERVE_DRAIN    1   // FIFO overflowing: serve again, do not reset
#define BMV_SERVE_BLOCKED  2   // sensor alive but optically blocked: report only
#define BMV_SERVE_BENIGN   3   // other warning: note it, keep going
#define BMV_SERVE_FAULT    4   // real error: counts towards a reset

static uint8_t BMV080_Classify(int32_t rc) {
  switch (rc) {
    case E_BMV080_OK:
      return BMV_SERVE_OK;
    // More events than one serve call can drain -- the expected result of an
    // obstructed field of view. Draining fixes it, resetting does not.
    case E_BMV080_WARNING_FIFO_EVENTS_OVERFLOW:
    case E_BMV080_WARNING_FIFO_SW_BUFFER_SIZE:
    case E_BMV080_WARNING_FIFO_HW_BUFFER_SIZE:
    case E_BMV080_ERROR_FIFO_EVENTS_COUNT_SATURATED:
      return BMV_SERVE_DRAIN;
    // Documented hint for these is "remove potential obstructions".
    case E_BMV080_ERROR_DC_CANCEL_RANGE:
    case E_BMV080_ERROR_DC_ESTIM_RANGE:
    case E_BMV080_ERROR_LPWR_RANGE:
      return BMV_SERVE_BLOCKED;
    default:
      return (rc >= 1 && rc <= 4) ? BMV_SERVE_BENIGN : BMV_SERVE_FAULT;
  }
}

// Rate limited logging for the non-fatal cases.
static bool BMV080_NoteWarning(void) {
  if (BMV.warn_count < 250) { BMV.warn_count++; }
  return (BMV.warn_count == 1 || (BMV.warn_count % 100) == 0);
}

static void BMV080_ServeOnce(void) {
  bmv080_status_code_t rc = bmv080_serve_interrupt(BMV.handle, BMV080_DataReady, nullptr);

  switch (BMV080_Classify(rc)) {
    case BMV_SERVE_OK:
      BMV.error_count = 0;
      BMV.warn_count  = 0;
      break;

    case BMV_SERVE_DRAIN:
      if (BMV080_NoteWarning()) {
        AddLog(LOG_LEVEL_INFO, PSTR("BMV080: FIFO pressure (%d), draining%s"), rc,
               BMV.last.obstructed ? PSTR(" - obstruction reported") : PSTR(""));
      }
      // Catch up right away, bounded so the task is never hogged.
      for (uint8_t i = 0; i < 8; i++) {
        BMV.fifo_drain++;
        if (bmv080_serve_interrupt(BMV.handle, BMV080_DataReady, nullptr) == E_BMV080_OK) break;
      }
      BMV.last_data_ms = millis();               // not a dead sensor
      return;                                    // nothing is broken, skip Recovery

    case BMV_SERVE_BLOCKED:
      if (BMV080_NoteWarning()) {
        AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: optical error %d - check for obstruction"), rc);
      }
      BMV.last_data_ms = millis();               // alive, just blocked
      return;

    case BMV_SERVE_BENIGN:
      if (BMV080_NoteWarning()) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("BMV080: serve warning %d (count=%u)"), rc, BMV.warn_count);
      }
      break;

    case BMV_SERVE_FAULT:
      BMV.error_count++;
      if (BMV.error_count <= 5 || (BMV.error_count % 50) == 0) {
        AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: serve error %d (count=%u)"), rc, BMV.error_count);
      }
      if (rc == E_BMV080_ERROR_HW_READ && BMV.error_count >= 10) {
        AddLog(LOG_LEVEL_INFO, PSTR("BMV080: HW_READ errors, soft reset"));
        BMV.error_count = 0;
        if (bmv080_reset(BMV.handle) == E_BMV080_OK) {
          BMV080_ApplyConfig();
          if (bmv080_start_continuous_measurement(BMV.handle) == E_BMV080_OK) {
            BMV.last_data_ms = millis();
            return;
          }
        }
        AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: soft reset failed, full recovery"));
        BMV080_Close();
        BMV080_Open();
      }
      break;
  }
  BMV080_Recovery();
}

static void BMV080_Task(void *arg) {
  for (;;) {
    // Close first, then open, so a reinit request (both flags set) works and
    // the close does not clear the powered flag the open is about to need.
    if (BMV.req_close) {
      BMV.req_close = false;
      BMV080_Close();
      if (!BMV.req_open) { BMV.cfg.powered = false; }
    }
    if (BMV.req_open) { BMV.req_open = false; BMV080_Open(); }
    if (BMV080_Active()) { BMV080_ServeOnce(); }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

static void BMV080_StartTask(void) {
  if (BMV.task) return;
  // Measured peak usage of bmv080_serve_interrupt(), via the task's own
  // uxTaskGetStackHighWaterMark():
  //   steady state                    11068 bytes
  //   after replaying a 45 s backlog  12376 bytes (12200 of 24576 left free)
  // The peak does not grow with the backlog length, so 24 kB is ~50% headroom.
  // 12 kB tripped the stack canary outright -- do not lower this without
  // re-measuring.
  xTaskCreatePinnedToCore(BMV080_Task, "bmv080", BMV080_TASK_STACK, nullptr, 1, &BMV.task, 1);
  if (!BMV.task) { AddLog(LOG_LEVEL_ERROR, PSTR("BMV080: task create failed")); }
}

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
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "1",   BMV.last.pm1);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "2.5", BMV.last.pm25);
    WSContentSend_PD(HTTP_SNS_ENVIRONMENTAL_CONCENTRATION, "BMV080", "10",  BMV.last.pm10);
    // Only worth a row when something is off; algo/interval live in "Sensor125".
    if (BMV.last.obstructed)   { WSContentSend_P(PSTR("{s}BMV080{m}obstructed{e}")); }
    if (BMV.last.out_of_range) { WSContentSend_P(PSTR("{s}BMV080{m}out of range{e}")); }
  }
#endif
}

/*********************************************************************************************\
 * Command Sensor125
\*********************************************************************************************/

// Clamp integration time and duty period against each other: the SDK requires
// the period to exceed the integration time by at least 2 s.
static void BMV080_FixTiming(bool period_wins) {
  if (BMV.cfg.duty_period >= (uint16_t)BMV.cfg.integration_time + 2) return;
  if (period_wins) {
    BMV.cfg.integration_time = (float)(BMV.cfg.duty_period - 2);
  } else {
    BMV.cfg.duty_period = (uint16_t)BMV.cfg.integration_time + 2;
  }
  AddLog(LOG_LEVEL_INFO, PSTR("BMV080: timing adjusted to IntTime=%us Period=%us"),
         (uint16_t)BMV.cfg.integration_time, BMV.cfg.duty_period);
}

// Apply a config change: nothing to do while idle (it is picked up on the next
// open), otherwise let the worker task reopen the sensor with the new setting.
static bool BMV080_Reconfigure(void) {
  if (!BMV080_Active()) return true;
  BMV.req_close = true;
  BMV.req_open  = true;
  return true;
}

static void BMV080_SetIntegration(float sec) {
  if (sec < 1.0f)  sec = 1.0f;
  if (sec > 60.0f) sec = 60.0f;
  BMV.cfg.integration_time = sec;
}

static void BMV080_SetPeriod(uint16_t sec) {
  if (sec < 3)   sec = 3;
  if (sec > 900) sec = 900;
  BMV.cfg.duty_period = sec;
}

static bool BMV080Cmd(void) {
  // Tasmota reports data_len 0 for a bare "?", so check the payload itself.
  if (XdrvMailbox.data && XdrvMailbox.data[0]) {
    if ((XdrvMailbox.data[0] == '?' && XdrvMailbox.data[1] == '\0') ||
        !strcasecmp_P(XdrvMailbox.data, PSTR("help"))) {
      Response_P(PSTR("{\"BMV080_help\":{"
                      "\"0\":\"Off\","
                      "\"1[,int]\":\"Continuous, opt. integration time in s\","
                      "\"2[,int,period]\":\"Duty cycling, opt. ON time and period in s\","
                      "\"3|4|5\":\"Algorithm Fast|Balanced|HighPrecision (continuous only)\","
                      "\"9\":\"Reinit\"}}"));
      return true;
    }

    char arg[XdrvMailbox.data_len + 8];
    int  argc = ArgC();
    uint16_t mode = (uint16_t)atoi(ArgV(arg, 1));

    switch (mode) {
      case 0:
        return BMV080_PowerOff();

      case 1:                                    // continuous [,integration]
        BMV.cfg.duty_cycling = false;
        if (argc > 1) { BMV080_SetIntegration(CharToFloat(ArgV(arg, 2))); }
        break;

      case 2:                                    // duty cycling [,on [,period]]
        BMV.cfg.duty_cycling = true;
        if (argc > 1) { BMV080_SetIntegration(CharToFloat(ArgV(arg, 2))); }
        if (argc > 2) { BMV080_SetPeriod((uint16_t)atoi(ArgV(arg, 3))); }
        BMV080_FixTiming(argc > 2);              // an explicit period wins
        break;

      case 3: case 4: case 5:                    // algorithm: driver ids 2..4
        // The algorithm only means anything for a running continuous
        // measurement: duty cycling is fixed to fast response (datasheet), and
        // while idle there is nothing to apply it to.
        if (!BMV080_Active() || BMV.cfg.duty_cycling) {
          AddLog(LOG_LEVEL_INFO,
                 PSTR("BMV080: algorithm needs a running continuous measurement"
                      " - start it with 'Sensor125 1'"));
          return false;
        }
        BMV.cfg.algo_id = (uint8_t)(mode - 1);
        BMV080_Reconfigure();
        Response_P(PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.command, XdrvMailbox.data);
        return true;

      case 9:                                    // reinit
        if (!BMV.task) return false;
        BMV.req_close = true;
        BMV.req_open  = true;
        Response_P(PSTR("{\"%s\":\"Reinit\"}"), XdrvMailbox.command);
        return true;

      default:
        return false;
    }

    // Only modes 1 and 2 start the sensor; they reach this point.
    bool ok = BMV080_Active() ? BMV080_Reconfigure() : BMV080_PowerOn();
    Response_P(PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.command, XdrvMailbox.data);
    return ok;
  }

  // In duty cycling the SDK forces FastResponse, so report what is really used.
  Response_P(PSTR("{\"BMV080\":{\"Power\":%s,\"Mode\":\"%s\",\"Algo\":\"%s\",\"IntTime\":%u,"
                  "\"Period\":%u,\"Obstruct\":%u,\"Errors\":%u,\"Warnings\":%u,\"FifoDrains\":%u}}"),
             BMV.cfg.powered ? PSTR("true") : PSTR("false"),
             BMV.cfg.duty_cycling ? PSTR("DutyCycling") : PSTR("Continuous"),
             BMV.cfg.duty_cycling ? PSTR("FastResponse") : BMV_AlgoName(BMV.cfg.algo_id),
             (uint16_t)BMV.cfg.integration_time,
             BMV.cfg.duty_cycling ? BMV.cfg.duty_period : 0,
             BMV.last.obstructed, BMV.error_count, BMV.warn_count, BMV.fifo_drain);
  return true;
}

/********************************************************************************************/

static void BMV080_Detect(void) {
  for (uint8_t bus = 0; bus < 2; bus++) {
    if (!I2cSetDevice(BMV080_ADDR, bus)) continue;
    BMV.bus = bus;
    I2cSetActiveFound(BMV080_ADDR, "BMV080", bus);
    BMV.detected = true;
#if BMV080_AUTOSTART
    // Never call the SDK from the loop task: it needs ~11 kB of stack.
    // BMV080_PowerOn starts the worker task and lets it do the open.
    BMV080_PowerOn();
#else
    BMV.cfg.powered = false;                     // wait for an explicit start
    AddLog(LOG_LEVEL_INFO,
           PSTR("BMV080: found on bus%d, idle - 'Sensor125 1' or 'Sensor125 2' to start"),
           bus +1);
#endif
    return;
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns125(uint32_t function) {
  if (!I2cEnabled(XI2C_111)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    BMV080_Detect();
  }
  else if (BMV.detected) {
    switch (function) {
      case FUNC_JSON_APPEND:
        BMV080_Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        BMV080_Show(0);
        break;
#endif  // USE_WEBSERVER
      case FUNC_COMMAND_SENSOR:
        if (XSNS_125 == XdrvMailbox.index) { result = BMV080Cmd(); }
        break;
    }
  }
  return result;
}

#endif  // USE_BMV080
#endif  // USE_I2C
