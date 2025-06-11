/*
  xsns_124_alphasense_opc.ino - Alphasense Ltd. Optical Particle Counter support for Tasmota

  Copyright (C) 2025 Jan-David Förster

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


#ifdef USE_SPI
#ifdef USE_OPC
#define XSNS_124                  124
#define D_CMND_OPC "OPC"

#pragma pack(push, 1)

typedef enum {
  OPC_WRITE_LOFF,
  OPC_WRITE_LON,
  OPC_WRITE_FOFF,
  OPC_WRITE_FON,
  OPC_READ_SN,
  OPC_READ_FW,
  OPC_READ_STATUS,
  OPC_READ_HIST,
  OPC_READ_PM,
  OPC_READ_CONFIG,
  OPC_READ_INFO,
  OPC_STATUS
} OPC_CommandCodes_t;

typedef enum {
  OPC_TYPE_N2       = 1 << 0,
  OPC_TYPE_N3       = 1 << 1,
  OPC_TYPE_R1       = 1 << 2,
  OPC_TYPE_R2       = 1 << 3,
  OPC_INTERVALOVERR = 1 << 4,
  OPC_LONOFF        = 1 << 5,
  OPC_FONOFF        = 1 << 6,
  OPC_PMHIST        = 1 << 7
} OPC_mode_bit_field_t;

typedef struct {
  uint16_t a, b, c, d, e, f, g, h, i, j, k, l, m, n, o;
  uint8_t mtof_a, mtof_c, mtof_e, mtof_g;
  float flow;
  uint16_t temp, humi;
  float period;
  uint8_t rej_cnt_gl, rej_cnt_ln;
  float pm_a, pm_b, pm_c;
} hist_n2data_t;

typedef struct {
  uint16_t a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x;
  uint8_t mtof_a, mtof_c, mtof_e, mtof_g;
  uint16_t period, flow, temp, humi;
  float pm_a, pm_b, pm_c;
  uint16_t rej_cnt_gli, rej_cnt_lon, rej_cnt_rat, rej_cnt_oor, fan_rev_cnt, las_status;
} hist_n3data_t;

hist_n2data_t *hist_n2 = nullptr;
hist_n3data_t *hist_n3 = nullptr;

typedef struct {
  void **ptr;
  size_t size;
} OPC_AllocInfo_t;

// Lookup-Tabelle zur Allokation

OPC_AllocInfo_t OPC_AllocTable[4] = {
  { (void **)&hist_n2, sizeof(hist_n2data_t) },
  { (void **)&hist_n3, sizeof(hist_n3data_t) },
  { NULL, 0 }, // { (void **)&hist_r1, sizeof(hist_r1data_t) },
  { NULL, 0 }  // (void **)&hist_r2, sizeof(hist_r2data_t) }
};

//const char kOPC_Codes[] PROGMEM = "OPC_STATUS|OPC_RESET|OPC_READ_PM|OPC_READ_HIST|OPC_READ_POWER|OPC_READ_FW|OPC_READ_SN|OPC_READ_INFO|OPC_WRITE_POWER";
void OPCReadDataToStruct(OPC_CommandCodes_t cmd);
void OPCWriteControl(OPC_CommandCodes_t cmd);

struct OPC_Defaults {
  uint8_t reg;      // Register-Adresse
  uint8_t val[4];   // Längen für OPC_TYPE_N2, OPC_TYPE_N3, OPC_TYPE_R1, OPC_TYPE_R2
};

#pragma pack(pop)

const OPC_Defaults OPCCommand[] PROGMEM = {
  // { reg, {OPC_TYPE_N2, OPC_TYPE_N3, OPC_TYPE_R1, OPC_TYPE_R2}
  { 0x03, { 0x03, 0x06, 0x00, 0x00 }},  // OPC_WRITE_LOFF
  { 0x03, { 0x02, 0x07, 0x00, 0x00 }},  // OPC_WRITE_LON
  { 0x03, { 0x05, 0x02, 0x00, 0x03 }},  // OPC_WRITE_FOFF
  { 0x03, { 0x04, 0x03, 0x03, 0x00 }},  // OPC_WRITE_FON
  { 0x10, {60, 60, 60, 60} },  // OPC_READ_SN
  { 0x12, {4, 4, 4, 4} },  // OPC_READ_FW
  { 0x13, {4, 6, 0, 0} },   // OPC_READ_STATUS
  { 0x30, {64, 86, 64, 64} },  // OPC_READ_HIST
  { 0x32, {12, 14, 14, 14} },  // OPC_READ_PM
  { 0x3C, {0, 168, 0, 0 }  },  // OPC_READ_CONFIG
  { 0x3F, {60, 60, 60, 60} },  // OPC_READ_INFO
  { 0xCF, {1, 1, 1, 1} }  // OPC_STATUS 
};

//void* OPCReadTarget[sizeof(OPCCommand)];  
void* OPCReadTarget[sizeof(OPCCommand) / sizeof(OPCCommand[0])];

#ifdef USE_WEBSERVER
#define WEB_HANDLE_OPC "s124"
const char HTTP_BTN_OPC[] PROGMEM = 
"<p>"
"<button onclick='sendOPCCommand()'>Toggle " D_CMND_OPC "</button>"
"</p>"
"<script>"
"function sendOPCCommand() {"
"  fetch('" WEB_HANDLE_OPC "', { method: 'POST' });"
"}"
"</script>";
#endif // USE_WEBSERVER

#define OPC_SLOW_INTERVAL 4
#define OPC_DEFAULT_INTERVAL 2
//#define OPC_TYPES         4          // OPC-N2, OPC-N3, OPC-R1 and OPC-R2

const char S_JSON_OPC_COMMAND_NVALUE[] PROGMEM = "{\"" D_CMND_OPC "%s\":%d}";

const char kOPC_Types[] PROGMEM = "OPC-N2|OPC-N3|OPC-R1|OPC-R2";

struct OPC_T {
  //bool chk_status;
  //bool init;
  uint8_t mode, setmode;     //
  uint8_t interval    = OPC_DEFAULT_INTERVAL ;
  uint8_t setinterval = OPC_DEFAULT_INTERVAL ;
  //uint8_t power;
  char types[20];
  //unsigned char data[61];

  //struct POWER_T {
  //  uint8_t state_fan, state_laser, dac_fan, dac_laser, switch_laser, toggle_gain; 
  //};

  //struct STATUS_T {
  //  uint8_t fan_on, laser_on, fan_dac, laser_dac, laser_sw, gain;
  //};

  struct DATA_T {
  //  POWER_T power;
    struct PM_T {
      float pm_a, pm_b, pm_c;
    } pm;
    float temp, humi, abs_humi;
    char flowstr[FLOATSZ];
    //STATUS_T status;
    unsigned char sn[60];
    unsigned char info[60];
    bool available;
  } data;
} OPC;


bool OPCAvailable(uint8_t cmd) {
    return (SPI.transfer(cmd) == 0xF3);
}

void OPCPreInit(void) {
  if (PinUsed(GPIO_OPC_CS) && (SPI_MOSI_MISO == TasmotaGlobal.spi_enabled)) {
    pinMode(Pin(GPIO_OPC_CS), OUTPUT);
    digitalWrite(Pin(GPIO_OPC_CS), 1);
    OPC.setmode = 0x10;
    OPC.mode    = 0;
    //OPCReadTarget[OPC_READ_INFO]   = &OPC.msg.info;
    //OPC.chk_status = false;
    //OPC.mode &= ~OPC_INIT; // init to false
    //OPC.available = false;
  }
}

void OPCAllocateMem(uint32_t type) {

  void **ptr = OPC_AllocTable[type].ptr;
  size_t size = OPC_AllocTable[type].size;

  if (ptr && *ptr) { 
      free(*ptr);
      *ptr = nullptr;
      AddLog(LOG_LEVEL_INFO, PSTR("Free: %u"), size);
  }
  
  OPCReadTarget[OPC_READ_HIST] = *ptr = calloc(1, size);
  if (!*ptr) {
      AddLog(LOG_LEVEL_INFO, PSTR("Memory allocation failed %u"), size);
      return;
  } else {   AddLog(LOG_LEVEL_INFO, PSTR("Alloc: %u"), size);}

}

bool OPCInit(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("Type search %s"), OPC.types);
  //OPC.mode &= ~0x0F; // delete OPCs
  if (!OPCHandleData(OPCCommand[OPC_READ_INFO].reg, 0xF3, OPCCommand[OPC_READ_INFO].val[0], OPC.data.info)) {
  //  OPC.interval = 5;
    return false;
  }
  //if (OPC.msg.histogram_p) {
  //  free(OPC.msg.histogram_p);
  //  OPC.msg.histogram_p = NULL;
  //}

  for (uint32_t i = 0; i < 4; i++) {
    GetTextIndexed(OPC.types, sizeof(OPC.types), i, kOPC_Types);
      //str = GetTextIndexed(OPC.types, sizeof(OPC.types), OPC.type, kOPC_Types);
    AddLog(LOG_LEVEL_INFO, PSTR("Type search %s"), OPC.types);
    if (strstr((const char*)OPC.data.info, OPC.types) != NULL) {
      AddLog(LOG_LEVEL_INFO, PSTR("%s: Found and initialized %s"), D_CMND_OPC, OPC.types);
      AddLog(LOG_LEVEL_INFO, PSTR("%s: %s"), D_CMND_OPC, OPC.data.info);
      OPC.mode |= (1 << i); // Set Bit i
      AddLog(LOG_LEVEL_INFO, PSTR("%s: mode is: %i  and lowest bit is %i"), D_CMND_OPC, i, (__builtin_ctz(OPC.mode & 0x0F)));

      //uint8_t len = OPCCommand[OPC_READ_HIST].val[i];
      //OPCAllocateMem(i);  // Speicher für Histogramm-Struktur allokieren

      OPCReadTarget[OPC_READ_PM]     = &OPC.data.pm;
      //if (*OPC_AllocTable[i].ptr == nullptr) {
      //  AddLog(LOG_LEVEL_ERROR, PSTR("OPC: Memory allocation failed for type %u"), i);
      //  return false;
      //}
      return ((OPC.mode & 0x0F) != 0);
    }   
  }
  return false;
}

void OPCLoop(void) {
  if (PinUsed(GPIO_OPC_CS) && (SPI_MOSI_MISO == TasmotaGlobal.spi_enabled)) {
    uint8_t diff = (OPC.setmode ^ (OPC.mode & 0xF0));//
    //AddLog(LOG_LEVEL_INFO, PSTR("OPC: Masken 0x%02X 0x%02X 0x%02X "), OPC.setmode, OPC.mode, diff);
    if (diff & OPC_INTERVALOVERR) {  
      if (OPC.setmode & OPC_INTERVALOVERR) { OPC.interval = OPC_SLOW_INTERVAL; OPC.mode |= OPC_INTERVALOVERR;} else { OPC.interval = OPC.setinterval;}
      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Intervall auf %d gesetzt"), OPC.interval);
    }
    if (diff & 0x60) { // keep only bits 5 and 6
      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Änderungen erkannt: 0x%02X"), diff);

      if (diff & OPC_FONOFF) {
        OPCWriteControl((OPC.setmode & OPC_FONOFF) ? OPC_WRITE_FON : OPC_WRITE_FOFF);
        OPC.mode ^= OPC_FONOFF;
        return;
      } else if (diff & OPC_LONOFF) {
        OPCWriteControl((OPC.setmode & OPC_LONOFF) ? OPC_WRITE_LON : OPC_WRITE_LOFF);
        OPC.mode ^= OPC_LONOFF;
        return;
      }   
    }

    if ((TasmotaGlobal.uptime % OPC.interval) == 0) {  
      if ((OPC.mode & 0x0F) == 0) { //if no OPC, do the init!
        AddLog(LOG_LEVEL_INFO, PSTR("%s: Searching sensor..."), D_CMND_OPC);
        //AddLog(LOG_LEVEL_INFO, PSTR("OPC mode after init: 0x%02X"), OPC.mode);
        if (OPCInit()) {OPC.setmode = OPC.mode &= ~0x10;}
        return;
      }
    // READ INTERVAL
      if (diff & OPC_PMHIST) {
        OPC.mode ^= OPC_PMHIST;
        if ((OPC.mode & OPC_PMHIST)) {
          //OPCAllocateMem((OPC.mode ? __builtin_ctz(OPC.mode) : -1));
          OPCAllocateMem(__builtin_ctz(OPC.mode & 0x0F));
        }
      }
      if ((OPC.mode & 0x60) == 0x60) { //fan on and laser on?
        if (!(OPC.mode & OPC_PMHIST)) {
          //AddLog(LOG_LEVEL_INFO, PSTR("Read PM: 0x%02X"), OPC.mode);
          OPCReadDataToStruct(OPC_READ_PM);
        } else {
          //AddLog(LOG_LEVEL_INFO, PSTR("Read Hist: 0x%02X"), OPC.mode);
          OPCReadDataToStruct(OPC_READ_HIST);
        }  
      }
    } 
  }
}

void OPCReadDataToStruct(OPC_CommandCodes_t cmd) {
  if (cmd < OPC_READ_SN || cmd > OPC_STATUS) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid read command %d"), D_CMND_OPC, cmd);
    return;
  }
  uint8_t reg, len;
  reg = OPCCommand[cmd].reg; //   memcpy_P(&reg, &OPCCommand[cmd].reg, sizeof(uint8_t)); 
  len = OPCCommand[cmd].val[__builtin_ctz(OPC.mode & 0x0F)]; // memcpy_P(&len, &OPCCommand[cmd].val[OPC.type], sizeof(uint8_t));
  if (len == 0) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid read command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }
  void* target = OPCReadTarget[cmd];
  if (!target) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid target for command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }


  //AddLog(LOG_LEVEL_INFO, PSTR("%s: reg %d (len %u)"), D_CMND_OPC, reg, len);

/*   if (len == 0 || OPCReadTarget[cmd] == NULL) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: No valid target for command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }

  if (!target) {
    AddLog(LOG_LEVEL_ERROR, PSTR("OPCReadDataToStruct: Target pointer is NULL! cmd=%d"), cmd);
    return;
  } */


  //char opc_code_name[20];
  //GetTextIndexed(opc_code_name, sizeof(opc_code_name), cmd, kOPC_Codes);
  //AddLog(LOG_LEVEL_INFO, PSTR("%s: Try to fill %s ."), D_CMND_OPC, opc_code_name);

  uint8_t data[len];
  if (OPCHandleData(reg, 0xF3, len, data)) {
    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("%s: handle reg %u len %u"), D_CMND_OPC, reg, len);
    if (cmd == OPC_READ_PM || cmd == OPC_READ_HIST) {  
      if (!(OPC.mode & OPC_TYPE_N2)) {
        if (!CheckCRC(data, len)) { AddLog(LOG_LEVEL_INFO, PSTR("CRC check failed.")); return; } // CRC check failed.
        memcpy(target, data, len-2);
        OPC.data.available = true;
        return;
      }
      memcpy(target, data, len);
        //uint8_t *ptr = (uint8_t*)target;
        //for (size_t i = 0; i < len; i++) {
        //  AddLog(LOG_LEVEL_INFO, PSTR("Byte[%u] = 0x%02X\n"), i, ptr[i]);
        //}
    }

    //AddLog(LOG_LEVEL_INFO, PSTR("%s: Read len is %u and struct %u."), D_CMND_OPC, len, sizeof(HIST_N3_T));
  }
}

bool CheckCRC(unsigned char data[], size_t length) {
  // OPC-N3 and R1
  if (OPC.mode & OPC_TYPE_N2 || data == NULL || length < 2){ return false; }
  uint16_t crc = 0xFFFF ;
  for (size_t i = 0; i < length - 2; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
        crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return (crc == ((data[length-1] << 8) | data[length-2]));
}


void OPCSpiEnable(void) {
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1));  // Set up SPI at 400 kHz, MSB first, Capture at rising edge
  digitalWrite(Pin(GPIO_OPC_CS), 0);
}

void OPCSpiDisable(void) {
  digitalWrite(Pin(GPIO_OPC_CS), 1);
  SPI.endTransaction();     
}

void HandleOPCAction(void) {
  if (!HttpCheckPriviledgedAccess()) { return; }
  char command[12];
  snprintf_P(command, sizeof(command), PSTR("SENSOR124 %d"), !(OPC.mode & OPC_FONOFF));
  AddLog(LOG_LEVEL_INFO, PSTR("HandleOPCAction: Executing command: %s"), command);
  ExecuteWebCommand(command);
}


bool OPCHandleData(unsigned char reg, unsigned char cmd, uint8_t count, unsigned char *bytes) {
	if (count == 0) {
		return false;
	}
  for (int attempt = 0; attempt < 11; attempt++){ // Try a few times to read from register.
    if (attempt >= 10) {
      AddLog(LOG_LEVEL_INFO, PSTR("%s: Unable to read from register 0x%02x."), D_CMND_OPC, reg);
      if (OPC.mode & OPC_INTERVALOVERR) {OPC.mode &= ~0x0F;} // This is already the second time the communication fails. So do a fresh init.
      OPC.setmode |= OPC_INTERVALOVERR;
      return false;
    }
    OPCSpiEnable();
    //AddLog(LOG_LEVEL_INFO, PSTR("%s: Read reg 0x%02x."), D_CMND_OPC, reg);
    if (OPCAvailable(reg)) { break; }// First read always yields 0x31, false, so try again.
    
    OPCSpiDisable();
    delay(20);
  }
  delayMicroseconds(20);
  //AddLog(LOG_LEVEL_INFO, PSTR("%s: Send cmd 0x%02x."), D_CMND_OPC, cmd);
  while (count--) {
    *bytes++ = SPI.transfer(cmd);
  }
  OPCSpiDisable();
  //OPC.mode &= ~OPC_COMFAIL;
  //OPC.valid = SENSOR_MAX_MISS;
  return true;
} // End OPCHandleData()

void OPCWriteControl(OPC_CommandCodes_t cmd) {
  //if (cmd != OPC_WRITE_ON && cmd != OPC_WRITE_OFF && cmd != OPC_WRITE_LOFF && cmd != OPC_WRITE_LON) {
  //  AddLog(LOG_LEVEL_INFO, PSTR("%s: Invalid write command %d"), D_CMND_OPC, cmd);
  //  return;
  //}

  uint8_t type, reg, command;
  type    = (__builtin_ctz(OPC.mode & 0x0F));
  reg     = OPCCommand[cmd].reg;           //memcpy_P(&reg, &OPCCommand[cmd].reg, sizeof(uint8_t));          // Read register from PROGMEM
  command = OPCCommand[cmd].val[type]; //memcpy_P(&command, &OPCCommand[cmd].val[OPC.type], sizeof(uint8_t));  // Read value based on OPC.type
  AddLog(LOG_LEVEL_INFO, PSTR("%s: cmd =%u"), D_CMND_OPC, cmd);
  AddLog(LOG_LEVEL_INFO, PSTR("%s: Writing cmd 0x%02X to register 0x%02X, OPC type=%u"), D_CMND_OPC, command, reg, type);

  unsigned char ret;
  OPCHandleData(reg, command, 1, &ret);
  AddLog(LOG_LEVEL_INFO, PSTR("%s: returned 0x%02X"), D_CMND_OPC, ret);

} 

bool OPCCmd(void)
{
  if (XdrvMailbox.data_len > 0) {
    OPCSelectMode(XdrvMailbox.payload);
    Response_P(S_JSON_OPC_COMMAND_NVALUE, XdrvMailbox.command, XdrvMailbox.payload);
  }
  return true;
}

void OPCSelectMode(uint16_t mode)
{
  AddLog(LOG_LEVEL_INFO, PSTR("OPC: set mode to %u"),mode);

  if (mode > 999) {
    OPC.interval = mode / 1000;
    AddLog(LOG_LEVEL_INFO, PSTR("OPC: set interval to %d seconds"), OPC.interval);
    return;  // Keine weitere Modusverarbeitung, wenn ein Intervall gesetzt wurde
  }

  switch(mode){
    case 0:
      OPC.setmode &= ~(OPC_FONOFF | OPC_LONOFF); // Alle Modi ausschalten
      break;//OPCWriteControl(OPC_WRITE_FOFF);
      //delay(20);
      //OPCWriteControl(OPC_WRITE_LOFF);
      break;
    case 1:
      OPC.setmode |= (OPC_FONOFF | OPC_LONOFF); //OPCWriteControl(OPC_WRITE_LOFF);
      break;
    case 2:
      OPC.setmode &= ~OPC_PMHIST;
      break;
    case 3:
      OPC.setmode |= OPC_PMHIST;
      break;
    //case 4:
    //  OPC.mode |= OPC_PMHIST;
    //  break;
    //case 5:
    //  OPC.mode |= OPC_PMHIST;
    //  break;
  }
  //MqttPublishTeleperiodSensor();
}

void OPCShow(bool json) {

  if ((!(OPC.mode & 0x0F))            // kein OPC-Typ gefunden
      || (OPC.mode & OPC_INTERVALOVERR)
      || (!(OPC.mode & OPC_FONOFF))
      || (!(OPC.mode & OPC_LONOFF))) {
    return;
  }

  char types[11];
  strlcpy(types, OPC.types, sizeof(types));


  //float pm1 = 0.0f;
  //float pm2_5 = 0.0f;
  //float pm10 = 0.0f;


  uint32_t active = (__builtin_ctz(OPC.mode & 0x0F));  // OPC-Typ
  //AddLog(LOG_LEVEL_INFO, PSTR("OPC: type %u"), active);
  void *hist_ptr = *OPC_AllocTable[active].ptr;  // Lookup-Tabelle
  if ((OPC.mode & OPC_PMHIST) && hist_ptr && OPC.data.available) {
    OPC.data.available = false;
    if ((1 << active) == OPC_TYPE_N2) {
      hist_n2data_t *n2 = (hist_n2data_t *)hist_ptr;
      OPC.data.pm.pm_a   = n2->pm_a;
      OPC.data.pm.pm_b = n2->pm_b;
      OPC.data.pm.pm_b  = n2->pm_c;
      AddLog(LOG_LEVEL_INFO, PSTR("N2: PM1=%f, PM2.5=%f, PM10=%f"), 
      n2->pm_a, n2->pm_b, n2->pm_c);
    } else if ((1 << active) == OPC_TYPE_N3) {
      hist_n3data_t *n3 = (hist_n3data_t *)hist_ptr;
      OPC.data.pm.pm_a   = n3->pm_a;
      OPC.data.pm.pm_b = n3->pm_b;
      OPC.data.pm.pm_c  = n3->pm_c;

      uint32_t h_10 = ((uint32_t)n3->humi * 1000U) >> 16;
      int32_t t_10 = -450 + (((uint32_t)n3->temp * 1750U) >> 16);
      OPC.data.humi = ConvertHumidity(h_10/10.0f);
      OPC.data.temp = ConvertTemp(t_10/10.0f);
      OPC.data.abs_humi = CalcTempHumToAbsHum(OPC.data.temp, OPC.data.humi);
      //OPC.data.flow = (n3->flow * 0.0006f);
      dtostrfd((n3->flow * 0.0006f), 3, OPC.data.flowstr);

      //AddLog(LOG_LEVEL_INFO, PSTR("N3: PM1=%f, PM2.5=%f, PM10=%f, Temp=%f, Humi=%f, Flow rate=%f"), 
      //        n3->pm_a, n3->pm_b, n3->pm_c, OPC.data.temp, OPC.data.humi, OPC.data.flow);
      DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR(
        "N3 Data: "
        "a=%u, b=%u, c=%u, d=%u, e=%u, f=%u, g=%u, h=%u, i=%u, j=%u, "
        "k=%u, l=%u, m=%u, n=%u, o=%u, p=%u, q=%u, r=%u, s=%u, t=%u, "
        "u=%u, v=%u, w=%u, x=%u, "
        "mtof_a=%u, mtof_c=%u, mtof_e=%u, mtof_g=%u, "
        "period=%u, flow=%s, temp=%f, humi=%f, "
        "pm_a=%f, pm_b=%f, pm_c=%f, "
        "rej_cnt_gli=%u, rej_cnt_lon=%u, rej_cnt_rat=%u, rej_cnt_oor=%u, "
        "fan_rev_cnt=%u, las_status=%u"
      ),
        n3->a, n3->b, n3->c, n3->d, n3->e, n3->f, n3->g, n3->h, n3->i, n3->j,
        n3->k, n3->l, n3->m, n3->n, n3->o, n3->p, n3->q, n3->r, n3->s, n3->t,
        n3->u, n3->v, n3->w, n3->x,
        n3->mtof_a, n3->mtof_c, n3->mtof_e, n3->mtof_g,
        n3->period, OPC.data.flowstr, OPC.data.temp, OPC.data.humi,
        n3->pm_a, n3->pm_b, n3->pm_c,
        n3->rej_cnt_gli, n3->rej_cnt_lon, n3->rej_cnt_rat, n3->rej_cnt_oor,
        n3->fan_rev_cnt, n3->las_status
      );

    }
    
  } else if (!(OPC.mode & OPC_PMHIST) && hist_ptr && OPC.data.available) { 
    // -> Normaler PM-Modus (OPC.msg.pm)
    //pm1   = OPC.data.pm.pm_a;
    //pm2_5 = OPC.data.pm.pm_b;
    //pm10  = OPC.data.pm.pm_c;
    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("N3: PM1=%f, PM2.5=%f, PM10=%f"), 
    OPC.data.pm.pm_a, OPC.data.pm.pm_b, OPC.data.pm.pm_c);
  }
  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":{\"PM1\":%1_f,\"PM2.5\":%1_f,\"PM10\":%1_f,"),
    types,
    &OPC.data.pm.pm_a, &OPC.data.pm.pm_b, &OPC.data.pm.pm_c);
    if (OPC.mode & OPC_PMHIST) {
      ResponseAppendTHD(OPC.data.temp, OPC.data.humi);
      ResponseAppend_P(PSTR(",\"" D_JSON_AHUM "\":%4_f"), &OPC.data.abs_humi);
    }
    ResponseJsonEnd();
#ifdef USE_WEBSERVER
  } else {

  if ((OPC.mode & OPC_PMHIST) && ((1 << active) == OPC_TYPE_N3)) {

    WSContentSend_THD(types, OPC.data.temp, OPC.data.humi);
    WSContentSend_PD(HTTP_SNS_F_ABS_HUM, types, 4, &OPC.data.abs_humi);
    WSContentSend_PD(HTTP_SNS_LPM, types, OPC.data.flowstr);
    
  }

  WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "1",   &OPC.data.pm.pm_a);
  WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "2.5", &OPC.data.pm.pm_b);
  WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, types, "10",  &OPC.data.pm.pm_c);

#endif
  }


}

bool Xsns124(uint32_t function) {
  bool result = false;
  if (FUNC_INIT == function) {
    OPCPreInit();
  } else {

    switch (function) {
      case FUNC_EVERY_SECOND:
        OPCLoop();
        break;

      case FUNC_COMMAND_SENSOR:
        if (XSNS_124 == XdrvMailbox.index) {
          result = OPCCmd();
        }
        break;
      case FUNC_JSON_APPEND:
        OPCShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        OPCShow(0);
        break;
      case FUNC_WEB_ADD_MAIN_BUTTON:
        WSContentSend_P(HTTP_BTN_OPC);
        break;
      case FUNC_WEB_ADD_HANDLER:
        WebServer_on(PSTR("/" WEB_HANDLE_OPC), HandleOPCAction);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_OPC
#endif  // USE_SPI