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
//#define XI2C_76 76 // See I2CDEVICES.md

#define D_PRFX_SEN66 "Sen66"
#define D_CMND_TEMP_OFFSET "TempOffset"

const char kSen66Commands[] PROGMEM = D_PRFX_SEN66 "|" D_CMND_TEMP_OFFSET ;

void CmndSen66TempOffset(void); 

void (* const Sen66Command[])(void) PROGMEM = {
  &CmndSen66TempOffset
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
} *SEN66DATA = nullptr;

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

void SEN66Update(void) {  // Perform every second to ensure proper operation of the baseline compensation algorithm
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

  g_sen66_valid = true;                  // NEU: Daten gültig
}

void SEN66Show(bool json) {
  if (!g_sen66_valid) {                  // NEU: Bei Fehler nichts anzeigen/anhängen
    return;
  }
  char types[10];
  strcpy_P(types, PSTR("SEN66"));

  float temperature = NAN;
  float humidity = NAN;
  float abs_humidity = NAN;
  float temp_raw = NAN;
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
    temp_raw = (SEN66DATA->ambientTemperature / 200.0f);
    temperature = ConvertTemp(temp_raw + SEN66DATA->temp_offset);
    humidity = ConvertHumidity(SEN66DATA->ambientHumidity/100.0f);
    dew =      CalcTempHumToDew(temp_raw,humidity);
    abs_humidity = CalcTempHumToAbsHum(temp_raw, humidity);
    dtostrfd(humidity, Settings->flag2.humidity_resolution, str_humidity);
    dtostrfd(dew, Settings->flag2.temperature_resolution, str_dewpoint);
  }

  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":{\"PM1\":%1_f,\"PM2_5\":%1_f,\"PM4\":%1_f,\"PM10\":%1_f,"),
      types,
      &pm1, &pm2_5, &pm4, &pm10);
    ResponseAppend_P(PSTR("\"PN0-0_5\":%1_f,\"PN0_5-1\":%1_f,\"PN1-2_5\":%1_f,\"PN2_5-4\":%1_f,\"PN4-10\":%1_f,"),
      &npm0_5, &npm1, &npm2_5, &npm4, &npm10);
    if (!isnan(SEN66DATA->co2)) {
        ResponseAppend_P(PSTR("\"CO2\":%u,"), SEN66DATA->co2);
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
    if (!isnan(SEN66DATA->co2)) {
        WSContentSend_PD(HTTP_SNS_CO2, types, SEN66DATA->co2);
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
  //if (!I2cEnabled(XI2C_76)) { return false; }

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
