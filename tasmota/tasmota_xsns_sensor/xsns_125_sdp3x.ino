/*
  xsns_125_sdp3x.ino - Sensirion Digital Differential Pressure Sensors

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
#ifdef USE_SDP3X
/*********************************************************************************************\
 * Sensirion I2C Digital Differential Pressure Sensor
 *
 * This driver supports the following sensors:

 * - SDP3x series: SDP31, SDP32, SDP33,  (addresses: A: 0x21, B: 0x22, C: 0x23)
\*********************************************************************************************/

#define XSNS_125            125
#define XI2C_94             94         // See I2CDEVICES.md

#define SDP3X_TYPES         3          // SDP31, SDP32 and SDP33
#define SDP3X_ADDRESSES     3          // 0x21, 0x22, 0x23

enum SDP3X_Types { SDP3X_TYPE_31, SDP3X_TYPE_32, SDP3X_TYPE_33 };
const char kSdp3xTypes[] PROGMEM = "SDP31|SDP32|SDP33";

uint8_t sdp3x_addresses[] = { 0x21, 0x22, 0x23 };


uint8_t sdp3x_count = 0;
struct SDP3XSTRUCT {
  float   press = NAN;
  float   temp = NAN;
  uint8_t valid = 0;
  uint8_t type;        // Sensor type
  uint8_t address;     // I2C bus address
  uint8_t bus;
  char types[6];  // Sensor type name and address, e.g. "SDP3X"
} sdp3x_sensors[SDP3X_ADDRESSES];

uint8_t Sdp3xComputeCrc(uint8_t data[], uint8_t len) {
  // Compute CRC as per datasheet
  uint8_t crc = 0xFF;

  for (uint8_t x = 0; x < len; x++) {
    crc ^= data[x];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool Sdp3xRead(uint32_t sensor) {

  if (sdp3x_sensors[sensor].valid) { sdp3x_sensors[sensor].valid--; }
  TwoWire& myWire = I2cGetWire(sdp3x_sensors[sensor].bus);
  if (&myWire == nullptr) { return false; }   // No valid I2c bus
  uint32_t type = sdp3x_sensors[sensor].type;
  uint8_t data[9];
  myWire.requestFrom(sdp3x_sensors[sensor].address, (uint8_t)9); // Request 9 bytes of data
  for (uint32_t i = 0; i < 9; i++) {
    data[i] = myWire.read();
    //AddLog(LOG_LEVEL_ERROR, PSTR("%i"), data[i]);
    if (((i+1) % 3) == 0) {
      if (Sdp3xComputeCrc(&data[i-2], 2) != data[i]) { return false; }
      //AddLog(LOG_LEVEL_ERROR, PSTR("CRC"));
    }
  };

  int16_t p_raw =  (int16_t)((data[0] << 8) | data[1]);
  int16_t t_raw  = (int16_t)((data[3] << 8) | data[4]);
  int16_t scaling_factor = (int16_t)((data[6] << 8) | data[7]);

  sdp3x_sensors[sensor].temp = ConvertTemp(t_raw / 200.0f);
  sdp3x_sensors[sensor].press = ConvertPressure(p_raw / (float)scaling_factor);
  if (isnan(sdp3x_sensors[sensor].temp) || isnan(sdp3x_sensors[sensor].press)) { return false; }
    sdp3x_sensors[sensor].valid = SENSOR_MAX_MISS;
  return true;
}

/********************************************************************************************/

void Sdp3xDetect(void) {
  for (uint32_t bus = 0; bus < 2; bus++) {
    for (uint32_t i = 0; i < SDP3X_ADDRESSES; i++) {
        if (!I2cSetDevice(sdp3x_addresses[i], bus)) { continue; }
        uint32_t id = Sdp3xIDandStartup(sdp3x_addresses[i], bus);
        if(id) {
            sdp3x_sensors[sdp3x_count].type = id - 1;  
            sdp3x_sensors[sdp3x_count].address = sdp3x_addresses[i];
            sdp3x_sensors[sdp3x_count].bus = bus;
            GetTextIndexed(sdp3x_sensors[sdp3x_count].types, sizeof(sdp3x_sensors[sdp3x_count].types), sdp3x_sensors[sdp3x_count].type, kSdp3xTypes);
            I2cSetActiveFound(sdp3x_sensors[sdp3x_count].address, sdp3x_sensors[sdp3x_count].types, sdp3x_sensors[sdp3x_count].bus);
            sdp3x_count++;
            if (SDP3X_ADDRESSES == sdp3x_count) {
                return;
            }
        } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("SDP Sensor startup failed on bus %i, %02X"), bus, sdp3x_addresses[i]);
        }
    }
  }
}
    
uint8_t Sdp3xIDandStartup(uint8_t address, uint8_t bus) {
      TwoWire& myWire = I2cGetWire(bus);
      myWire.beginTransmission(address); // stop continuous measurement
      myWire.write(0x3F); 
      myWire.write(0xF9); 
      if (myWire.endTransmission()) { return 0; }
      delay(2);
      myWire.beginTransmission(address); // read identifier
      myWire.write(0x36); 
      myWire.write(0x7C); 
      if (myWire.endTransmission()) { return 0; }
      myWire.beginTransmission(address); // read identifier
      myWire.write(0xE1); 
      myWire.write(0x02); 
      if (myWire.endTransmission()) { return 0; }
      delay(2);
      uint8_t id = 0;
      myWire.requestFrom(address, (uint8_t)18);
      for (uint32_t i = 0; i < 18; i++) {
        if (i == 4){ // only byte 4 is relevant
            id = myWire.read(); 
            continue;
        }
        myWire.read();             
      };
      delay(2);
      myWire.beginTransmission(address); // start continuous mass flow measurement with T compensation and averaging
      myWire.write(0x36);       
      myWire.write(0x03);     
      if (!myWire.endTransmission()) { 
        return id; 
      }
      return 0;
}


void Sdp3xUpdate(void) {
    for (uint32_t idx = 0; idx < sdp3x_count; idx++) {
      if (!Sdp3xRead(idx)) {
        AddLogMissed(sdp3x_sensors[idx].types, sdp3x_sensors[idx].valid);
      }
  }
}

void Sdp3xShow(bool json) {
  char types[11];

  for (uint32_t idx = 0; idx < sdp3x_count; idx++) {
    if (sdp3x_sensors[idx].valid) {
      strlcpy(types, sdp3x_sensors[idx].types, sizeof(types));
      if (sdp3x_count > 1) {
        snprintf_P(types, sizeof(types), PSTR("%s%c%02X"), types, IndexSeparator(), sdp3x_sensors[idx].address);  // "SDP3X-0xXX"  
#ifdef ESP32
        if (TasmotaGlobal.i2c_enabled_2) {
          for (uint32_t i = 1; i < sdp3x_count; i++) {
            if (sdp3x_sensors[0].bus != sdp3x_sensors[i].bus) {
              snprintf_P(types, sizeof(types), PSTR("%s%c%d"), types, IndexSeparator(), sdp3x_sensors[idx].bus + 1); // "SDP3X-0xXX-X"  
              break;
            }
          }
        }
#endif
      }  
      if (json) {
        ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_TEMPERATURE "\":%2_f,\"" D_JSON_PRESSURE "\":%3_f}"), &types, &sdp3x_sensors[idx].temp, &sdp3x_sensors[idx].press);
#ifdef USE_WEBSERVER
      } else {
        char str_pressure[9];
        dtostrfd(sdp3x_sensors[idx].press, 3, str_pressure);
        WSContentSend_PD(HTTP_SNS_PRESSURE, types, str_pressure, "Pa");
        WSContentSend_Temp(types, sdp3x_sensors[idx].temp);
#endif  // USE_WEBSERVER
      }
  }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns125(uint32_t function) {
  if (!I2cEnabled(XI2C_94)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    Sdp3xDetect();
  }
  else if (sdp3x_count) {
    switch (function) {
      case FUNC_EVERY_SECOND:
        Sdp3xUpdate();
        break;
      case FUNC_JSON_APPEND:
        Sdp3xShow(1);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Sdp3xShow(0);
        break;
  #endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_SDP3X
#endif  // USE_I2C
