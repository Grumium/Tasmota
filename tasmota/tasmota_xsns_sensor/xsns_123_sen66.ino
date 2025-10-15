/*
  xsns_123_sen66.ino - SEN66 gas and air quality sensor support for Tasmota

  Copyright (C) 2022  Tyeth Gundry

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
#ifdef USE_SEN5X
  #undef USE_SEN5X
#endif
#ifdef USE_SEN66
/*********************************************************************************************\
 * SEN66 - Gas (VOC - Volatile Organic Compounds / NOx - Nitrous Oxides) and Particulates (PM)
 *
 * Source: Sensirion SEN66 Driver + Example, and Tasmota Driver 98 by Jean-Pierre Deschamps
 * Adaption for TASMOTA: Tyeth Gundry
 *
 * I2C Address: 0x69
\*********************************************************************************************/

#define XSNS_123 123
#define XI2C_76 76 // See I2CDEVICES.md

#define SEN66_CO2_INVALID 0xFFFF  // Invalid CO2 value during calibration

#define D_PRFX_SEN66 "Sen66"
#define D_CMND_TEMP_OFFSET "TempOffset"
#define D_CMND_HUMIDITY_OFFSET "HumidityOffset"
#define D_CMND_CO2_CALIBRATE "CO2Calibrate"
#define D_CMND_CO2_AUTOCAL "CO2AutoCal"

const char kSen66Commands[] PROGMEM = D_PRFX_SEN66 "|" D_CMND_TEMP_OFFSET "|" D_CMND_HUMIDITY_OFFSET "|" D_CMND_CO2_CALIBRATE "|" D_CMND_CO2_AUTOCAL ;

void CmndSen66TempOffset(void); 
void CmndSen66HumidityOffset(void); 
void CmndSen66CO2Calibrate(void);
void CmndSen66CO2AutoCal(void);

void (* const Sen66Command[])(void) PROGMEM = {
  &CmndSen66TempOffset, &CmndSen66HumidityOffset, &CmndSen66CO2Calibrate, &CmndSen66CO2AutoCal
};

typedef enum {
  START_CONTINUOUS_MEASUREMENT = 0x21,
  STOP_MEASUREMENT = 0x104,
  GET_DATA_READY = 0x202,
  READ_MEASURED_VALUES_AS_INTEGERS = 0x300,
  READ_NUMBER_CONCENTRATION_VALUES_AS_INTEGERS = 0x316,
  READ_MEASURED_RAW_VALUES = 0x405,
  START_FAN_CLEANING = 0x5607,
  SET_TEMPERATURE_OFFSET_PARAMETERS = 0x60b2,
  SET_VOC_ALGORITHM_TUNING_PARAMETERS = 0x60d0,
  GET_VOC_ALGORITHM_TUNING_PARAMETERS_ = 0x60d0,
  SET_NOX_ALGORITHM_TUNING_PARAMETERS = 0x60e1,
  GET_NOX_ALGORITHM_TUNING_PARAMETERS = 0x60e1,
  SET_TEMPERATURE_ACCELERATION_PARAMETERS = 0x6100,
  SET_VOC_ALGORITHM_STATE = 0x6181,
  GET_VOC_ALGORITHM_STATE = 0x6181,
  PERFORM_FORCED_CO2_RECALIBRATION = 0x6707,
  SET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION = 0x6711,
  GET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION = 0x6711,
  SET_AMBIENT_PRESSURE = 0x6720,
  GET_AMBIENT_PRESSURE = 0x6720,
  SET_SENSOR_ALTITUDE = 0x6736,
  GET_SENSOR_ALTITUDE = 0x6736,
  ACTIVATE_SHT_HEATER = 0x6765,
  GET_PRODUCT_NAME = 0xd014,
  GET_SERIAL_NUMBER = 0xd033,
  READ_DEVICE_STATUS = 0xd206,
  READ_AND_CLEAR_DEVICE_STATUS = 0xd210,
  DEVICE_RESET_CMD_ID = 0xd304,
} SEN66Cmds_t;

#define SEN66_ADDRESS 0x6b
#define SEN66_PASSIVE_MODE_INTERVAL 10

#include <SensirionI2cSen66.h>
#include <Wire.h>
SensirionI2cSen66 *sen66 = nullptr;
static bool g_sen66_valid = false;

struct SEN66DATA_s {
  uint16_t numberConcentrationPm0p5;
  uint16_t numberConcentrationPm1p0;
  uint16_t numberConcentrationPm2p5;
  uint16_t numberConcentrationPm4p0;
  uint16_t numberConcentrationPm10p0;
  uint16_t massConcentrationPm1p0;
  uint16_t massConcentrationPm2p5;
  uint16_t massConcentrationPm4p0;
  uint16_t massConcentrationPm10p0;
  int16_t ambientHumidity;
  int16_t ambientTemperature;
  int16_t vocIndex;
  int16_t noxIndex;
  uint16_t co2;
  float temp_offset = 0.0f;
  int8_t humidity_offset = 0;
  float rh_temp_correction = 0.0f;
  bool calibrating = false;
  uint32_t calibration_end_time = 0;  // millis() when calibration should end
  bool post_calibration_validation = false;  // validate readings after calibration
  uint32_t co2_suppress_until = 0;  // millis() when to stop suppressing CO2 in JSON
  uint32_t co2_suppress_min_until = 0;  // minimum time CO2 suppression must remain active
  bool co2_suppress_active = false;  // actively suppressing CO2 publication
} *SEN66DATA = nullptr;

// Efficient saturation vapor pressure calculation using polynomial approximation
// error negligible compared to Magnus-Tetens
float es_approx(float T_c) {
  float T = T_c;
  float T2 = T * T;
  float T3 = T2 * T;
  return 5.60658f + 0.64705f * T_c - 0.00326f * T2 + 0.00074f * T3;
}

// Corrected relative humidity calculation
float rh_corrected(float RH_meas, float T_meas_C, float deltaT_correction) {
  if (RH_meas <= 0.0f || deltaT_correction == 0.0f) return RH_meas;
  
  float T_true = T_meas_C + deltaT_correction;
  float ratio = es_approx(T_meas_C) / es_approx(T_true);
  float RH_corr = RH_meas * ratio;
  
  // Clamp to valid range
  if (RH_corr < 0.0f) return 0.0f;
  if (RH_corr > 100.0f) return 100.0f;
  return RH_corr;
}

/********************************************************************************************/

void sen66_Init(void) {
  int usingI2cBus = 0;
#ifdef ESP32
  if (!I2cSetDevice(SEN66_ADDRESS, 0)) {
    DEBUG_SENSOR_LOG(PSTR("Sensirion SEN66 not found, i2c bus 0"));
    if (TasmotaGlobal.i2c_enabled[1] ) {
      if(!I2cSetDevice(SEN66_ADDRESS, 1)) {
        DEBUG_SENSOR_LOG(PSTR("Sensirion SEN66 not found, i2c bus 1"));
        return;
      }
      usingI2cBus = 1;
    } else {
      return;
    }
  }
#else
  if (!I2cSetDevice(SEN66_ADDRESS)) {
    DEBUG_SENSOR_LOG(PSTR("Sensirion SEN66 not found, i2c bus 0"));
    return;
  }
#endif

  sen66 = new SensirionI2cSen66();
  if (1 == usingI2cBus) {
#if defined(ESP32) && defined(USE_I2C_BUS2)
    sen66->begin(Wire1, SEN66_ADDRESS);
#else
    sen66->begin(Wire, SEN66_ADDRESS);
#endif
  }
  else {
    sen66->begin(Wire, SEN66_ADDRESS);
  }
  
  if (!Settings->flag6.sen5x_passive_mode) {
    int error_stop = sen66->deviceReset();
    if (error_stop != 0) {
      DEBUG_SENSOR_LOG(PSTR("Sensirion SEN66 failed to reset device (I2C Bus %d)"), usingI2cBus);
      return;
    }
    // Wait 1 second for sensors to start recording + 100ms for reset command
    delay(1100);
    int error_start = sen66->startContinuousMeasurement();
    if (error_start != 0) {
      DEBUG_SENSOR_LOG(PSTR("Sensirion SEN66 failed to start measurement (I2C Bus %d)"), usingI2cBus);
      return;
    }
  }

  SEN66DATA = (SEN66DATA_s *)calloc(1, sizeof(struct SEN66DATA_s));
  g_sen66_valid = false;  
  I2cSetActiveFound(SEN66_ADDRESS, "SEN66", usingI2cBus);
}

void CmndSen66TempOffset(void) {
  if (SEN66DATA) {
    if (XdrvMailbox.data_len) {
      float val = CharToFloat(XdrvMailbox.data);
      if (val > -20.0f && val < 20.0f) {
        SEN66DATA->temp_offset = val;
      }
    }
    ResponseCmndFloat(SEN66DATA->temp_offset, 2);
  }
}

void CmndSen66HumidityOffset(void) {
  if (SEN66DATA) {
    if (XdrvMailbox.data_len) {
      int val = atoi(XdrvMailbox.data);
      if (val >= -50 && val <= 50) {
        SEN66DATA->humidity_offset = val;
      }
    }
    ResponseCmndNumber(SEN66DATA->humidity_offset);
  }
}

void CmndSen66CO2Calibrate(void) {
  if (sen66 && SEN66DATA) {
    uint16_t targetCo2 = 400;  // Default to 400 ppm
    if (XdrvMailbox.data_len) {
      targetCo2 = atoi(XdrvMailbox.data);
      if (targetCo2 < 300 || targetCo2 > 2000) {
        ResponseCmndChar("InvalidRange");
        return;
      }
    }
    
    AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Starting CO2 calibration at %d ppm"), targetCo2);
    
    // Skip updates during calibration and validate first readings afterward
    SEN66DATA->calibrating = true;
    SEN66DATA->calibration_end_time = millis() + 15000;  // 15 seconds protection after restart
    SEN66DATA->post_calibration_validation = false;  // Will be enabled when period ends
    // Start CO2 suppression for up to 15 seconds, minimum 8 seconds
    uint32_t now = millis();
    SEN66DATA->co2_suppress_until = now + 15000;
    SEN66DATA->co2_suppress_min_until = now + 8000;  // Minimum 8 seconds
    SEN66DATA->co2_suppress_active = true;
    AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 suppression started - until %lu (min %lu)"), SEN66DATA->co2_suppress_until, SEN66DATA->co2_suppress_min_until);
    
    // Stop measurement first
    uint16_t error = sen66->stopMeasurement();
    if (error) {
      AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to stop measurement, error %d"), error);
      ResponseCmndChar("StopError");
      return;
    }
    
    // Wait 600ms after stopping measurement (datasheet requirement)
    delay(600);
    
    // Perform calibration
    uint16_t correction = 0;
    error = sen66->performForcedCo2Recalibration(targetCo2, correction);
    if (error) {
      AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: CO2 calibration failed with error %d"), error);
      // Try to restart measurement even if calibration failed
      sen66->startContinuousMeasurement();
      ResponseCmndChar("CalibrationError");
      return;
    }
    
    // Wait for calibration to complete (datasheet: ~500ms)
    delay(500);
    
    // Restart measurement
    error = sen66->startContinuousMeasurement();
    if (error) {
      AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to restart measurement, error %d"), error);
      ResponseCmndChar("RestartError");
      return;
    }
    
    AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 calibration completed at %d ppm, correction: %d ppm"), targetCo2, correction);
    ResponseCmndNumber(correction);
  } else {
    ResponseCmndChar("NotReady");
  }
}

void CmndSen66CO2AutoCal(void) {
  if (sen66 && SEN66DATA) {
    uint16_t error;
    
    // If parameter provided, need to enter idle mode to change setting
    if (XdrvMailbox.data_len) {
      // Skip updates during config change and validate first readings afterward
      SEN66DATA->calibrating = true;
      SEN66DATA->calibration_end_time = millis() + 10000;  // 10 seconds protection after restart
      SEN66DATA->post_calibration_validation = false;  // Will be enabled when period ends
      // Start CO2 suppression for up to 10 seconds, minimum 6 seconds
      uint32_t now = millis();
      SEN66DATA->co2_suppress_until = now + 10000;
      SEN66DATA->co2_suppress_min_until = now + 6000;  // Minimum 6 seconds
      SEN66DATA->co2_suppress_active = true;
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 suppression started (AutoCal) - until %lu (min %lu)"), SEN66DATA->co2_suppress_until, SEN66DATA->co2_suppress_min_until);
      
      // Stop measurement to enter idle mode (required for CO2 auto-cal setting)
      error = sen66->stopMeasurement();
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to stop measurement for CO2 auto-cal setting, error %d"), error);
        ResponseCmndChar("StopError");
        return;
      }
      
      // Brief wait for idle mode transition (minimal delay)
      delay(50);
      
      bool enable = (atoi(XdrvMailbox.data) != 0);
      error = sen66->setCo2SensorAutomaticSelfCalibration(enable ? 1 : 0);
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Setting CO2 auto-calibration setting failed with error %d"), error);
        // Restart measurement even if setting failed
        sen66->startContinuousMeasurement();
        ResponseCmndChar("SetError");
        return;
      }
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 auto-calibration setting %s"), enable ? "enabled" : "disabled");
      
      uint8_t padding = 0;
      bool enabled = false;
      error = sen66->getCo2SensorAutomaticSelfCalibration(padding, enabled);
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Reading CO2 auto-calibration setting failed with error %d"), error);
        // Restart measurement even if reading failed
        sen66->startContinuousMeasurement();
        ResponseCmndChar("ReadError");
        return;
      }
      
      // Restart measurement
      error = sen66->startContinuousMeasurement();
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to restart measurement after CO2 auto-cal setting, error %d"), error);
        ResponseCmndChar("RestartError");
        return;
      }
      
      ResponseCmndNumber(enabled ? 1 : 0);
    } else {
      // No parameter - just read current status (also requires idle mode)
      SEN66DATA->calibrating = true;
      SEN66DATA->calibration_end_time = millis() + 8000;  // 8 seconds protection after restart
      SEN66DATA->post_calibration_validation = false;  // Will be enabled when period ends
      // Start CO2 suppression for up to 8 seconds, minimum 5 seconds
      uint32_t now = millis();
      SEN66DATA->co2_suppress_until = now + 8000;
      SEN66DATA->co2_suppress_min_until = now + 5000;  // Minimum 5 seconds
      SEN66DATA->co2_suppress_active = true;
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 suppression started (AutoCal) - until %lu (min %lu)"), SEN66DATA->co2_suppress_until, SEN66DATA->co2_suppress_min_until);
      
      error = sen66->stopMeasurement();
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to stop measurement for CO2 auto-cal status read, error %d"), error);
        ResponseCmndChar("StopError");
        return;
      }
      
      delay(50);  // Brief wait for idle mode transition (minimal delay)
      
      uint8_t padding = 0;
      bool enabled = false;
      error = sen66->getCo2SensorAutomaticSelfCalibration(padding, enabled);
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Reading CO2 auto-calibration setting status failed with error %d"), error);
        sen66->startContinuousMeasurement();
        ResponseCmndChar("ReadError");
        return;
      }
      
      // Restart measurement
      error = sen66->startContinuousMeasurement();
      if (error) {
        AddLog(LOG_LEVEL_ERROR, PSTR("SEN66: Failed to restart measurement after CO2 auto-cal status read, error %d"), error);
        ResponseCmndChar("RestartError");
        return;
      }
      
      ResponseCmndNumber(enabled ? 1 : 0);
    }
  } else {
    ResponseCmndChar("NotReady");
  }
}

void SEN66Update(void) {  // Perform every second to ensure proper operation of the baseline compensation algorithm
  if (!SEN66DATA) return;
  
  uint32_t now = millis();
  
  // Check if CO2 suppression period should end (only after minimum time)
  if (SEN66DATA->co2_suppress_active) {
    if (now > SEN66DATA->co2_suppress_until || 
        (now > SEN66DATA->co2_suppress_min_until && SEN66DATA->co2 >= 300 && SEN66DATA->co2 <= 5000)) {
      SEN66DATA->co2_suppress_active = false;
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: CO2 suppression period ended"));
    }
  }
  
  // Check if protection period has ended (applies to both calibration and config changes)
  if (SEN66DATA->calibrating && now > SEN66DATA->calibration_end_time) {
    SEN66DATA->calibrating = false;
    SEN66DATA->post_calibration_validation = true;  // Enable validation for next readings
    AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Protection period ended, validating next readings"));
  }
  
  // Skip reading during protection period to avoid publishing invalid values
  if (SEN66DATA->calibrating) {
    DEBUG_SENSOR_LOG(PSTR("SEN66: Skipping update during protection period"));
    return;
  }

  uint16_t error;
  //uint8_t padding;
  //bool dataReady;
  char errorMessage[256];
  DEBUG_SENSOR_LOG(PSTR("Running readMeasuredValues for SEN66..."));
  //rror = sen66->getDataReady(padding, dataReady);
  //AddLog(LOG_LEVEL_INFO, PSTR("Data Ready %d"), dataReady);
  error = sen66->readNumberConcentrationValuesAsIntegers(
    SEN66DATA->numberConcentrationPm0p5, SEN66DATA->numberConcentrationPm1p0, SEN66DATA->numberConcentrationPm2p5,
    SEN66DATA->numberConcentrationPm4p0, SEN66DATA->numberConcentrationPm10p0);
  error = sen66->readMeasuredValuesAsIntegers(
      SEN66DATA->massConcentrationPm1p0, SEN66DATA->massConcentrationPm2p5, SEN66DATA->massConcentrationPm4p0,
      SEN66DATA->massConcentrationPm10p0, SEN66DATA->ambientHumidity, SEN66DATA->ambientTemperature, SEN66DATA->vocIndex,
      SEN66DATA->noxIndex, SEN66DATA->co2);

  if (error) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("SEN66: Failed to retrieve readings"));
#ifdef DEBUG_TASMOTA_SENSOR
    DEBUG_SENSOR_LOG(PSTR("Error trying to execute readMeasuredValues():"));
    errorToString(error, errorMessage, 256);
    DEBUG_SENSOR_LOG(errorMessage);
#endif
    g_sen66_valid = false; 
    return;
  }

  // Always validate CO2 values - reject implausible readings anytime
  if (SEN66DATA->co2 == 0xFFFF || SEN66DATA->co2 > 5000 || (SEN66DATA->co2 < 300 && SEN66DATA->co2 > 0)) {  
    // Reject: invalid (65535), too high (>5000ppm), or too low (1-299ppm)
    AddLog(LOG_LEVEL_DEBUG, PSTR("SEN66: Rejecting implausible CO2 value: %u ppm"), SEN66DATA->co2);
    g_sen66_valid = false;  // Mark as invalid - don't publish
    return;
  }
  
  // Additional validation after operations - re-enter protection for first invalid readings
  if (SEN66DATA->post_calibration_validation && 
      (SEN66DATA->co2 == 0xFFFF || SEN66DATA->co2 > 5000 || (SEN66DATA->co2 < 300 && SEN66DATA->co2 > 0))) {  
    // Re-enter protection mode and try again
    SEN66DATA->calibrating = true;
    SEN66DATA->calibration_end_time = millis() + 1000;  // Retry in 1 second
    g_sen66_valid = false;
    return;
  }
  
  // First plausible value received - disable validation, but respect minimum suppression time
  if (SEN66DATA->post_calibration_validation && SEN66DATA->co2 >= 300 && SEN66DATA->co2 <= 5000) {
    SEN66DATA->post_calibration_validation = false;
    // Only stop suppression if minimum time has passed
    if (millis() > SEN66DATA->co2_suppress_min_until) {
      SEN66DATA->co2_suppress_active = false;
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: First plausible CO2 value: %u ppm - suppression ended"), SEN66DATA->co2);
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("SEN66: First plausible CO2 value: %u ppm - suppression continues (min time)"), SEN66DATA->co2);
    }
  }
  
  g_sen66_valid = true;                  // Only set valid if we reach this point
  
  // Log CO2 value only after successful validation
  //if (!isnan(SEN66DATA->co2)) {
  //  AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Current CO2: %u ppm"), SEN66DATA->co2);
  //} else {
  //  AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Current CO2: nan"));
  //}
}

void SEN66Show(bool json) {
  if (!SEN66DATA) return;
  
  // Don't show anything during calibration or if data is invalid
  if (!g_sen66_valid && !SEN66DATA->calibrating) {
    return;
  }
  char types[10];
  strcpy_P(types, PSTR("SEN66"));

  float temperature = NAN;
  float humidity = NAN;
  float abs_humidity = NAN;
  float dew = NAN;
  float npm0_5 = SEN66DATA->numberConcentrationPm0p5/10.0f;
  float npm1 = (SEN66DATA->numberConcentrationPm1p0 - SEN66DATA->numberConcentrationPm0p5)/10.0f;
  float npm2_5 = (SEN66DATA->numberConcentrationPm2p5 - SEN66DATA->numberConcentrationPm1p0)/10.0f;
  float npm4 = (SEN66DATA->numberConcentrationPm4p0 - SEN66DATA->numberConcentrationPm2p5)/10.0f;
  float npm10 = (SEN66DATA->numberConcentrationPm10p0 - SEN66DATA->numberConcentrationPm4p0)/10.0f;
  float pm1 = SEN66DATA->massConcentrationPm1p0/10.0f;
  float pm2_5 = SEN66DATA->massConcentrationPm2p5/10.0f;
  float pm4 = SEN66DATA->massConcentrationPm4p0/10.0f;
  float pm10 = SEN66DATA->massConcentrationPm10p0/10.0f;
  int voc = SEN66DATA->vocIndex / 10;
  int nox = SEN66DATA->noxIndex / 10;
  char str_humidity[33];
  char str_dewpoint[33];
  //AddLog(LOG_LEVEL_INFO, PSTR("voc %f"), voc);
  bool ahum_available = (!isnan(SEN66DATA->ambientTemperature) && !isnan(SEN66DATA->ambientHumidity) && (SEN66DATA->ambientHumidity > 0));
  if (ahum_available) {
    temperature = (SEN66DATA->ambientTemperature / 200.0f);
    humidity = ConvertHumidity(SEN66DATA->ambientHumidity/100.0f);
    dew =      CalcTempHumToDew(temperature,humidity);
    abs_humidity = CalcTempHumToAbsHum(temperature, humidity);
    temperature = ConvertTemp(temperature + SEN66DATA->temp_offset);
    dtostrfd(humidity, Settings->flag2.humidity_resolution, str_humidity);
    dtostrfd(dew, Settings->flag2.temperature_resolution, str_dewpoint);
  }

  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":{\"PM1\":%1_f,\"PM2_5\":%1_f,\"PM4\":%1_f,\"PM10\":%1_f,"),
      types,
      &pm1, &pm2_5, &pm4, &pm10);
    ResponseAppend_P(PSTR("\"PN0-0_5\":%1_f,\"PN0_5-1\":%1_f,\"PN1-2_5\":%1_f,\"PN2_5-4\":%1_f,\"PN4-10\":%1_f,"),
      &npm0_5, &npm1, &npm2_5, &npm4, &npm10);
    // Show CO2 as 0 when suppressed or during protection period, normal value when not suppressed
    if (!isnan(SEN66DATA->co2)) {
      if (!SEN66DATA->co2_suppress_active && !SEN66DATA->calibrating) {
        ResponseAppend_P(PSTR("\"CO2\":%u,"), SEN66DATA->co2);
      } else {
        ResponseAppend_P(PSTR("\"CO2\":0,"));
        if (SEN66DATA->calibrating) {
          AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Showing CO2 as 0 in JSON output during protection period (actual: %u)"), SEN66DATA->co2);
        } else {
          AddLog(LOG_LEVEL_INFO, PSTR("SEN66: Showing CO2 as 0 in JSON output during suppression (actual: %u)"), SEN66DATA->co2);
        }
      }
    }
    if (!isnan(SEN66DATA->noxIndex)) {
      ResponseAppend_P(PSTR("\"NOx\":%d,"), nox);
    }
    if (!isnan(SEN66DATA->vocIndex)) {
      ResponseAppend_P(PSTR("\"VOC\":%d,"), voc);
    }
    if (ahum_available) {
      ResponseAppend_P(PSTR("\"" D_JSON_TEMPERATURE "\":%2_f"), &temperature);
      ResponseAppend_P(PSTR(",\"" D_JSON_HUMIDITY "\":%1_f"), &humidity);
      ResponseAppend_P(PSTR(",\"" D_JSON_DEWPOINT "\":%2_f"), &dew);
      ResponseAppend_P(PSTR(",\"" D_JSON_AHUM "\":%4_f"), &abs_humidity);
    }
    ResponseJsonEnd();
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "1", &pm1);
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "2.5", &pm2_5);
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "4", &pm4);
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "10", &pm10);
    WSContentSend_PD(HTTP_SNS_F_PARTICLE_NUMBER_CONCENTRATION, types, "0","0.5", &npm0_5);
    WSContentSend_PD(HTTP_SNS_F_PARTICLE_NUMBER_CONCENTRATION, types, "0.5", "1", &npm1);
    WSContentSend_PD(HTTP_SNS_F_PARTICLE_NUMBER_CONCENTRATION, types, "1","2.5", &npm2_5);
    WSContentSend_PD(HTTP_SNS_F_PARTICLE_NUMBER_CONCENTRATION, types, "2.5","4", &npm4);
    WSContentSend_PD(HTTP_SNS_F_PARTICLE_NUMBER_CONCENTRATION, types, "4","10", &npm10);
    // Show CO2 as 0 in web when suppressed or during protection period, normal value when not suppressed
    if (!isnan(SEN66DATA->co2)) {
      if (!SEN66DATA->co2_suppress_active && !SEN66DATA->calibrating) {
        WSContentSend_PD(HTTP_SNS_CO2, types, SEN66DATA->co2);
      } else {
        WSContentSend_PD(HTTP_SNS_CO2, types, 0);
        if (SEN66DATA->calibrating) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("SEN66: Showing CO2 as 0 in web output during protection period (actual: %u)"), SEN66DATA->co2);
        } else {
          AddLog(LOG_LEVEL_DEBUG, PSTR("SEN66: Showing CO2 as 0 in web output during suppression (actual: %u)"), SEN66DATA->co2);
        }
      }
    }
    if (!isnan(SEN66DATA->noxIndex)) {
      WSContentSend_PD(HTTP_SNS_NOX, types, nox);
    }
    if (!isnan(SEN66DATA->vocIndex)) {
      WSContentSend_PD(HTTP_SNS_VOC, types, voc);
    }
    if (ahum_available) {
      WSContentSend_Temp(types, temperature);
      WSContentSend_PD(HTTP_SNS_HUM, types, str_humidity);
      WSContentSend_PD(HTTP_SNS_DEW, types, str_dewpoint, TempUnit());
      WSContentSend_PD(HTTP_SNS_F_ABS_HUM, types, 4, &abs_humidity);
    }
#endif
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns123(uint32_t function) {
  if (!I2cEnabled(XI2C_76)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    sen66_Init();
  }
  else if (SEN66DATA != nullptr) {
    switch (function) {
    case FUNC_EVERY_SECOND:
      if (Settings->flag6.sen5x_passive_mode) {
        if (TasmotaGlobal.uptime % SEN66_PASSIVE_MODE_INTERVAL == 0) {
          SEN66Update();
        }
      }
      else {
        SEN66Update();
      }
      break;
    case FUNC_JSON_APPEND:
      SEN66Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      SEN66Show(0);
      break;
#endif // USE_WEBSERVER
    case FUNC_COMMAND:
      result = DecodeCommand(kSen66Commands, Sen66Command);
      break;
    }
  }
  return result;
}

#endif // USE_SEN66
#endif // USE_I2C
