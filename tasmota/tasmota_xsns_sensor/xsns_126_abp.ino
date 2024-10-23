/*
  xsns_126_abp.ino - Honeywell Basic ABP Series gauge pressure sensor support for Tasmota
  The gauge pressure sensor measures pressure relative to the local atmospheric pressure.

  Adapted from amsx915 driver by Bastian Urschel
  Copyright (C) 2024 Jan-David Förster

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
#ifdef USE_ABP
/*********************************************************************************************\
 * ABP - Pressure and Temperature
 * I2C Address: 0x28
\*********************************************************************************************/

#define XSNS_126              126
#define XI2C_95               95  // See I2CDEVICES.md

#ifndef ABP_ADDR
  #define ABP_ADDR            0x28
#endif

//#define ABP_EVERYNSECONDS     5
#define ABP_DEVICE_NAME		    "ABP"

#define ABP_PMIN_DEFAULT           0
#define ABP_PMAX_DEFAULT           6895

#ifndef USE_UFILESYS
#warning "ABP pressure settings cannot be saved persistent due to missing filesystem"
#endif

/********************************************************************************************/

typedef struct ABPdata_s {
  uint32_t  file_crc32;    // To detect file changes
  uint16_t  file_version;  // To detect driver function changes
  int16_t   pmin;
  int16_t   pmax;
  float pressure  = NAN;
  float temperature = NAN;
  uint8_t meas_valid = 0;
  //uint8_t cnt;
} ABPdata_t;
ABPdata_t *ABP = nullptr;

const uint16_t ABP_VERSION = 0x0100;       // Latest sensor version (See settings deltas below)

bool ABPCommand() {
  int32_t vals[2];
  ParseParameters(2,(uint32_t *)vals);
  if(XdrvMailbox.data_len >= 3 && XdrvMailbox.data_len < 13) {
    if (vals[0] >= -4200 && vals[0] < 11000) {
      if (vals[1] > -4200 && vals[1] <= 11000) {
        ABP->pmin = (int16_t)vals[0]; // save pmin of sensor
        ABP->pmax = (int16_t)vals[1];  // same with pmax
        ABPSettingsSave();
        Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_126, "pressure range set");
        return true;
      }
    }
  }
  else if(XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"" ABP_DEVICE_NAME "\": {\"pmin\":%i,\"pmax\":%i}}"), ABP->pmin, ABP->pmax);
    return true;
  }
  Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_126, "invalid pressure range [-4200..11000 hPa]");
  return false;
}

void ABPDetect(void) {
    if (!I2cActive(ABP_ADDR)) {
      Wire.requestFrom(ABP_ADDR, 4);
      if(Wire.available() == 4) {
        I2cSetActiveFound(ABP_ADDR, ABP_DEVICE_NAME);        
        ABP = (ABPdata_t *)calloc(1, sizeof(ABPdata_t));
        if (!ABP) {
          AddLog(LOG_LEVEL_ERROR, PSTR(ABP_DEVICE_NAME ":@%02X Memory error!"), ABP_ADDR);
        }
        else {
          ABPSettingsLoad(0); // load config
        }
      return;
      }
    }
}


void ABPUpdate(void) {
  //if(ABP->cnt++ == ABP_EVERYNSECONDS) { // try to read sensor every n seconds
    //ABP->cnt = 0;
    if (!ABPReadData()) {
      AddLogMissed("ABP", ABP->meas_valid);
    }
  //}
}

bool ABPReadData(void) {
  if (ABP->meas_valid) { ABP->meas_valid--; }
  Wire.requestFrom(ABP_ADDR, (uint8_t)4); // Request 4 bytes
  if(Wire.available() != 4) { return false; }
  uint8_t buffer[4];
  for (uint32_t i = 0; i < 4; i++) {
    buffer[i] = Wire.read();
  }
  float press = ((((buffer[0] & 0x3F) << 8 | buffer[1]) & 0x3FFF) - 1638.0f)*((ABP->pmax) - (ABP->pmin)) / 13107 + (ABP->pmin);
  int32_t temp_10  = ((((buffer[2] << 8 | buffer[3]) >> 5) * 2000) >> 11) - 500;
  ABP->pressure = ConvertPressure(press);
  ABP->temperature = ConvertTemp(temp_10/10.0f);
  
  if (isnan(ABP->temperature) || isnan(ABP->pressure)) { return false; }

  ABP->meas_valid = SENSOR_MAX_MISS;
  return true;
}

void ABPShow(bool json) {
  if(ABP->meas_valid) {
    if (json) {
      ResponseAppend_P(PSTR(",\"" ABP_DEVICE_NAME "\":{\"" D_JSON_TEMPERATURE "\":%1_f,\"" D_JSON_PRESSURE "\":%1_f}"), &ABP->temperature, &ABP->pressure);
#ifdef USE_WEBSERVER
      } else {
        char str_pressure[9];
        dtostrfd(ABP->pressure, 1, str_pressure);
        WSContentSend_PD(HTTP_SNS_PRESSURE, ABP_DEVICE_NAME, str_pressure, PressureUnit().c_str());
        WSContentSend_Temp(ABP_DEVICE_NAME, ABP->temperature);
#endif  // USE_WEBSERVER
      }
  }
}

/*********************************************************************************************\
 * Driver Settings load and save
\*********************************************************************************************/

void ABPSettingsLoad(bool erase) {
  // Called from FUNC_PRE_INIT (erase = 0) once at restart
  memset(ABP, 0x00, sizeof(ABPdata_t));
  ABP->file_version = ABP_VERSION;
  ABP->pmax = ABP_PMAX_DEFAULT;
  ABP->pmin = ABP_PMIN_DEFAULT;

  // *** End Init default values ***

#ifndef USE_UFILESYS
  AddLog(LOG_LEVEL_INFO, PSTR("CFG: " ABP_DEVICE_NAME " defaults as file system not enabled"));
#else
  // Try to load sensor config file
  char filename[20];
  snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_SENSOR), XSNS_126);

  if (erase) {
    TfsDeleteFile(filename);  // Use defaults
  }
  else if (TfsLoadFile(filename, (uint8_t*)ABP, sizeof(ABPdata_t))) {
    if (ABP->file_version != ABP_VERSION) {      // Fix version dependent changes
      // Set current version and save settings
      ABP->file_version = ABP_VERSION;
      ABPSettingsSave();
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: " ABP_DEVICE_NAME " config loaded from file"));
  }
  else {
    // File system not ready: No flash space reserved for file system
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: " ABP_DEVICE_NAME " use defaults as file system not ready or file not found"));
  }
#endif  // USE_UFILESYS
}

void ABPSettingsSave(void) {
  // Called from FUNC_SAVE_SETTINGS every SaveData second and at restart
#ifdef USE_UFILESYS
  uint32_t crc32 = GetCfgCrc32((uint8_t*)ABP +4, sizeof(ABPdata_t) -4);  // Skip crc32
  if (crc32 != ABP->file_crc32) {
    // Try to save sensor config file
    ABP->file_crc32 = crc32;

    char filename[20];
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_SENSOR), XSNS_126);

    if (TfsSaveFile(filename, (const uint8_t*)ABP, sizeof(ABPdata_t))) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: " ABP_DEVICE_NAME " Settings saved to file"));
    } else {
      // File system not ready: No flash space reserved for file system
      AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: ERROR " ABP_DEVICE_NAME " file system not ready or unable to save file"));
    }
  }
#endif  // USE_UFILESYS
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns126(uint32_t function) {
  if (!I2cEnabled(XI2C_95)) { return false; }

  bool result = false;

  if (function == FUNC_INIT) {
    ABPDetect();
  }
  if(ABP) {
    switch(function) {
      case FUNC_EVERY_SECOND:
        ABPUpdate();
        break;
      case FUNC_COMMAND_SENSOR:
        if(XSNS_126 == XdrvMailbox.index) {
          result = ABPCommand();
        }
        break;
      case FUNC_JSON_APPEND:
        ABPShow(1);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        ABPShow(0);
        break;
  #endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_ABP
#endif  // USE_I2C
