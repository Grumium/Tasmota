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

#define FORMAT_PM_DIA(val_um100, buffer)                        \
  do {                                                          \
    if ((val_um100) % 100 == 0) {                               \
      /* Format-String aus Flash, nicht RAM */                  \
      snprintf_P((buffer), sizeof(buffer), PSTR("%u"),         \
                 (val_um100) / 100);                            \
    } else {                                                    \
      dtostrfd((val_um100) / 100.0f, 1, (buffer));              \
    }                                                           \
  } while (0)

#pragma pack(push, 1)

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
  uint16_t bins[24];
  uint8_t mtof_a, mtof_c, mtof_e, mtof_g;
  uint16_t period, flow, temp, humi;
  float pm_a, pm_b, pm_c;
  uint16_t rej_cnt_gli, rej_cnt_lon, rej_cnt_rat, rej_cnt_oor, fan_rev_cnt, las_status;
} hist_n3data_t;

hist_n2data_t *hist_n2 = nullptr;
hist_n3data_t *hist_n3 = nullptr;


typedef enum {
  OPC_WRITE_LOFF,
  OPC_WRITE_LON,
  OPC_WRITE_FOFF,
  OPC_WRITE_FON,
  OPC_WRITE_LGAIN,
  OPC_WRITE_HGAIN,
  OPC_READ_SN,
  OPC_READ_FW,
  OPC_READ_STATUS,
  OPC_READ_HIST,
  OPC_READ_PM,
  OPC_READ_CONFIG,
  OPC_READ_INFO,
  OPC_CHK_STATUS,
  OPC_DO_RESET
} OPC_CommandCodes_t;

typedef enum {
  OPC_TYPE_N2       = 1 << 0, //1
  OPC_TYPE_N3       = 1 << 1, //2
  OPC_TYPE_R1       = 1 << 2, //4
  OPC_TYPE_R2       = 1 << 3, //8
  OPC_OVERRIDE      = 1 << 4, //16
  OPC_LONOFF        = 1 << 5, //32
  OPC_FONOFF        = 1 << 6, //64
  OPC_PMHIST        = 1 << 7, //128
  OPC_CONFIG        = 1 << 8, //256 
  OPC_STATUS        = 1 << 9,  //512
  OPC_LGAIN        = 1 << 10,  //1024
  OPC_HGAIN        = 1 << 11,  //2048
  OPC_RESET        = 1 << 12  //4096
} OPC_mode_bit_field_t;

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

//const char kOPC_Codes[] PROGMEM = "OPC_CHK_STATUS|OPC_RESET|OPC_READ_PM|OPC_READ_HIST|OPC_READ_POWER|OPC_READ_FW|OPC_READ_SN|OPC_READ_INFO|OPC_WRITE_POWER";
void OPCReadDataToStruct(OPC_CommandCodes_t cmd); // Todo: Change to boolean
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
  { 0x03, { 0x00, 0x08, 0x00, 0x00 }},  // OPC_WRITE_LGAIN
  { 0x03, { 0x00, 0x09, 0x00, 0x00 }},  // OPC_WRITE_HGAIN
  
  { 0x10, {60, 60, 60, 60} },  // OPC_READ_SN
  { 0x12, {4, 4, 4, 4} },  // OPC_READ_FW
  { 0x13, {4, 6, 0, 0} },   // OPC_READ_STATUS
  { 0x30, {64, 86, 64, 64} },  // OPC_READ_HIST
  { 0x32, {12, 14, 14, 14} },  // OPC_READ_PM
  { 0x3C, {0, 168, 0, 0 }  },  // OPC_READ_CONFIG
  { 0x3F, {60, 60, 60, 60} },  // OPC_READ_INFO
  { 0xCF, {1, 1, 1, 1} },  // OPC_CHK_STATUS 
  { 0x06, {1, 1, 1, 1} }  // OPC_DO_RESET
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

const char kBinsZeroString[] PROGMEM = 
  "0,0,0,0,0,0,0,0,"
  "0,0,0,0,0,0,0,0,"
  "0,0,0,0,0,0,0,0";

static const uint16_t binsZero[24] = { 0 };
static const uint8_t kMaxSpiRetries = 10; 

struct OPC_T {
  //bool chk_status;
  //bool init;
  uint16_t mode, setmode;     //
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
  struct CONFIG_T {
    // 16-bit ADC Bin boundaries (BB0 - BB24)
    uint16_t bin_boundaries_adc[25];         // [0] - [24]
    // 16-bit Bin boundaries in µm * 100 (BBD0 - BBD24)
    uint16_t bin_boundaries_um100[25];       // [0] - [24]
    // 16-bit Bin weighting factors (BW0 - BW23)
    uint16_t bin_weightings[24];   // [0] - [23]
    // PM measurement boundaries in µm * 100
    uint16_t pm_dia_a;  // M_A
    uint16_t pm_dia_b;  // M_B
    uint16_t pm_dia_c;  // M_C
    // weitere 16-bit Konfigurationswerte
    uint16_t max_tof;                      // MaxTOF
    uint16_t am_sampling_interval_count;  // AMSamplingIntervalCount
    uint16_t am_idle_interval_count;      // AMIdleIntervalCount
    uint16_t am_max_data_arrays;          // AMMaxDataArraysInFile
    // 8-bit Konfigurationsflags
    uint8_t am_only_save_pm;              // AMOnlySavePMData
    uint8_t am_fan_on_idle;               // AMFanOnInIdle
    uint8_t am_laser_on_idle;             // AMLaserOnInIdle
    uint8_t tof_to_sfr_factor;            // TOF to SFR factor
    uint8_t pvp;                          // Particle Validation Period
    uint8_t bin_weighting_index;         // BinWeightingIndex
    bool news;
  } config;
  struct STATUS_T {
    uint8_t fan_on, laser_on, fan_dac, laser_dac, laser_sw, gain;
  } status;
  struct DATA_T {


    struct PM_T {
      float a, b, c;
      char dia_a[FLOATSZ], dia_b[FLOATSZ], dia_c[FLOATSZ];
    } pm;

    float temp, humi, abs_humi, flow, period;
    char concstr[200];

    unsigned char sn[60];
    unsigned char info[60];
    bool available;
    bool isFirst;
  } data;
} OPC;

const char* OPCReplaceDotWithUnderscore(const char* input) {
  static char buffer[FLOATSZ];
  strlcpy(buffer, input, sizeof(buffer));

  char *dot = strchr(buffer, '.');
  if (dot) {
    if (dot[1] == '0' && dot[2] == '\0') {
      *dot = '\0';
    } else {
      *dot = '_';
    }
  }

  return buffer;
}


bool OPCAvailable(uint8_t cmd) {
    return (SPI.transfer(cmd) == 0xF3);
}

void OPCPreInit(void) {
  if (PinUsed(GPIO_OPC_CS) && (SPI_MOSI_MISO == TasmotaGlobal.spi_enabled)) {
    pinMode(Pin(GPIO_OPC_CS), OUTPUT);
    digitalWrite(Pin(GPIO_OPC_CS), 1);
    OPC.config.news = false;
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
      DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("Free: %u"), size);
  }
  
  OPCReadTarget[OPC_READ_HIST] = *ptr = calloc(1, size);
  if (!*ptr) {
      AddLog(LOG_LEVEL_ERROR, PSTR("Memory allocation failed %u"), size);
      return;
  } else {   DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("Alloc: %u"), size);}

}

bool OPCInit(void) {
  //GetTextIndexed(OPC.types, sizeof(OPC.types), 5, kOPC_Types);
  //OPC.mode &= ~0x0F; // delete OPCs
  uint8_t regInfo = pgm_read_byte(&OPCCommand[OPC_READ_INFO].reg);
  uint8_t lenInfo = pgm_read_byte(&OPCCommand[OPC_READ_INFO].val[0]);
  if (!OPCHandleData(regInfo, 0xF3, lenInfo, OPC.data.info)) {
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
      //AddLog(LOG_LEVEL_INFO, PSTR("%s: mode is: %i  and lowest bit is %i"), D_CMND_OPC, i, (__builtin_ctz(OPC.mode & 0x0F)));

      //uint8_t len = OPCCommand[OPC_READ_HIST].val[i];
      //OPCAllocateMem(i);  // Speicher für Histogramm-Struktur allokieren

      OPCReadTarget[OPC_READ_PM]     = &OPC.data.pm;
      OPCReadTarget[OPC_READ_CONFIG]     = &OPC.config;
      OPCReadTarget[OPC_READ_STATUS]     = &OPC.status;
      //if (*OPC_AllocTable[i].ptr == nullptr) {
      //  AddLog(LOG_LEVEL_ERROR, PSTR("OPC: Memory allocation failed for type %u"), i);
      //  return false;
      //}
      return ((OPC.mode & 0x0F) != 0);
    }   
  }
  return false;
}


static void OPCProcessMeasurements(void) {
  if (!OPC.data.available) { return; }
  if (OPC.mode & OPC_PMHIST) {
    const uint32_t active = __builtin_ctz(OPC.mode & 0x0F);
    void *hist_ptr        = *OPC_AllocTable[active].ptr;
    if (!hist_ptr) { return; }
    // ── Histogram mode ───────────────────────────────────────────────────────
    if ((1U << active) == OPC_TYPE_N2) {
      auto *n2 = static_cast<hist_n2data_t *>(hist_ptr);
      OPC.data.pm.a = n2->pm_a;
      OPC.data.pm.b = n2->pm_b;
      OPC.data.pm.c = n2->pm_c;

    } else if ((1U << active) == OPC_TYPE_N3) {
      auto *n3 = static_cast<hist_n3data_t *>(hist_ptr);

      // PM values
      OPC.data.pm.a = n3->pm_a;
      OPC.data.pm.b = n3->pm_b;
      OPC.data.pm.c = n3->pm_c;

      // Histogram concentrations
      if (n3->flow == 0 || n3->period == 0 ||
          !memcmp(n3->bins, binsZero, sizeof(n3->bins))) {
        strlcpy(OPC.data.concstr, kBinsZeroString, sizeof(OPC.data.concstr));
        OPC.data.flow = 0.0f;
      } else {
        OPC.data.concstr[0] = '\0';
        const float norm = 1.0f / (n3->period * n3->flow * 0.01f);
        for (uint8_t i = 0; i < 24; ++i) {
          const float conc = n3->bins[i] * norm;
          char        tmp[16];
          dtostrfd(conc, 3, tmp);
          strlcat(OPC.data.concstr, tmp, sizeof(OPC.data.concstr));
          if (i < 23) strlcat(OPC.data.concstr, PSTR(","), sizeof(OPC.data.concstr));
        }
        OPC.data.flow = n3->flow * 0.0006f;             // l/min
      }

      // Environmental data
      OPC.data.period = n3->period / 100.0f;            // s

      const uint32_t h_10 = ((uint32_t)n3->humi * 1000U) >> 16;
      const int32_t  t_10 = -450 + (((uint32_t)n3->temp * 1750U) >> 16);

      OPC.data.humi     = ConvertHumidity(h_10 / 10.0f);
      OPC.data.temp     = ConvertTemp(t_10  / 10.0f);
      OPC.data.abs_humi = CalcTempHumToAbsHum(OPC.data.temp, OPC.data.humi);
    }

  }

  OPC.data.isFirst     = true;   // derived data ready for display
  OPC.data.available = false;  // consume raw buffer
}

void OPCLoop(void) {
  if (PinUsed(GPIO_OPC_CS) && (SPI_MOSI_MISO == TasmotaGlobal.spi_enabled)) {
    uint16_t diff = (OPC.setmode ^ (OPC.mode & 0xF0)); //ignore the lowest four bits, these are for the sensor types

    AddLog(LOG_LEVEL_INFO, PSTR("OPC: setmode=0x%03X, mode=0x%03X, diff=0x%03X"), 
           OPC.setmode, OPC.mode, diff);

    if (diff & OPC_OVERRIDE) {  
      if (OPC.setmode & OPC_OVERRIDE) { OPC.interval = OPC_SLOW_INTERVAL; OPC.mode |= OPC_OVERRIDE;} else { OPC.interval = OPC.setinterval; OPC.mode ^= OPC_OVERRIDE;}
      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Period set to default: %d sec"), OPC.interval);
    }

    if (diff & OPC_CONFIG) {
      if (OPC.setmode & OPC_CONFIG) {
        OPC.setmode ^= OPC_CONFIG; 
        DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: READ CONFIG TO STRUCT"));
        OPCReadDataToStruct(OPC_READ_CONFIG); // if boolean, then successful change mode.
        OPC.mode ^= OPC_CONFIG; 
        FORMAT_PM_DIA(OPC.config.pm_dia_a, OPC.data.pm.dia_a);
        FORMAT_PM_DIA(OPC.config.pm_dia_b, OPC.data.pm.dia_b);
        FORMAT_PM_DIA(OPC.config.pm_dia_c, OPC.data.pm.dia_c);
        MqttPublishSensor();
      }
    }
    if (diff & OPC_STATUS) {
      if (OPC.setmode & OPC_STATUS) {
        OPC.setmode ^= OPC_STATUS; 
        DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: READ STATUS TO STRUCT"));
        OPCReadDataToStruct(OPC_READ_STATUS); // if boolean, then successful change mode.
        OPC.mode ^= OPC_STATUS; 
        MqttPublishSensor();
      }
    }

    //if (diff & 0x960) { 
    //  DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: Changes detected: 0x%03X"), diff);

    if (diff & OPC_FONOFF) {
      OPCWriteControl((OPC.setmode & OPC_FONOFF) ? OPC_WRITE_FON : OPC_WRITE_FOFF);
      OPC.mode ^= OPC_FONOFF;
      return;
    } else if (diff & OPC_LONOFF) {
      OPCWriteControl((OPC.setmode & OPC_LONOFF) ? OPC_WRITE_LON : OPC_WRITE_LOFF);
      OPC.mode ^= OPC_LONOFF;
      return;
    } else if (diff & OPC_LGAIN) {
      OPCWriteControl(OPC_WRITE_LGAIN);
      OPC.setmode &= ~OPC_LGAIN;
      OPC.setmode ^= OPC_STATUS;
      return;
    } else if (diff & OPC_HGAIN) {
      OPCWriteControl(OPC_WRITE_HGAIN);
      OPC.setmode &= ~OPC_HGAIN;
      OPC.setmode ^= OPC_STATUS;
      return;
    } else if (diff & OPC_RESET) {
      unsigned char ret;
      OPCHandleData(0x06, 0xF3, 1, &ret);
      AddLog(LOG_LEVEL_INFO, PSTR("%s: returned 0x%02X"), D_CMND_OPC, ret);
      OPC.setmode ^= OPC_RESET;
      OPC.mode &= ~0x0F;
      return;
    }   
    //}

    if ((TasmotaGlobal.uptime % OPC.interval) == 0) {  
      if ((OPC.mode & 0x0F) == 0) { //if no OPC, do the init!
        AddLog(LOG_LEVEL_INFO, PSTR("%s: Searching sensor..."), D_CMND_OPC);
        //AddLog(LOG_LEVEL_INFO, PSTR("OPC mode after init: 0x%02X"), OPC.mode);
        if (OPCInit()) {
          OPC.setmode = OPC.mode &= ~0x10;
          OPC.setmode ^= OPC_CONFIG;
          OPC.setmode ^= OPC_STATUS;
        }
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
    OPCProcessMeasurements();
    if ((OPC.data.isFirst))  {
      MqttPublishSensor();
    }
  }
}

void OPCReadDataToStruct(OPC_CommandCodes_t cmd) {
  if (cmd < OPC_READ_SN || cmd > OPC_CHK_STATUS) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid read command %d"), D_CMND_OPC, cmd);
    return;
  }
  uint8_t reg, len;
  reg = pgm_read_byte(&OPCCommand[cmd].reg); //   memcpy_P(&reg, &OPCCommand[cmd].reg, sizeof(uint8_t)); 
  len = pgm_read_byte(&OPCCommand[cmd].val[__builtin_ctz(OPC.mode & 0x0F)]); // memcpy_P(&len, &OPCCommand[cmd].val[OPC.type], sizeof(uint8_t));
  if (len == 0) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid read command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }
  void* target = OPCReadTarget[cmd];
  if (!target) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid target for command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }
  uint8_t data[len];
  if (OPCHandleData(reg, 0xF3, len, data)) {
    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("%s: handle reg %u len %u"), D_CMND_OPC, reg, len);
    if (cmd == OPC_READ_PM || cmd == OPC_READ_HIST ) {  
      if (!(OPC.mode & OPC_TYPE_N2)) {
        if (!CheckCRC(data, len)) { AddLog(LOG_LEVEL_ERROR, PSTR("CRC check failed.")); return; } // CRC check failed.
        memcpy(target, data, len-2);
        OPC.data.available = true;
        return;
      }
      memcpy(target, data, len);
    }
    if (cmd == OPC_READ_CONFIG || cmd == OPC_READ_STATUS) {  
      memcpy(target, data, len);
      //OPC.data.available = true;
      return;
    }
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
  DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("HandleOPCAction: Executing command: %s"), command);
  ExecuteWebCommand(command);
}

class SpiGuard {
public:
  explicit SpiGuard(uint32_t clk_hz = 400'000UL) {
    SPI.beginTransaction(SPISettings(clk_hz, MSBFIRST, SPI_MODE1));
    digitalWrite(Pin(GPIO_OPC_CS), LOW);     // CS aktiv (0)
  }
  ~SpiGuard() {
    digitalWrite(Pin(GPIO_OPC_CS), HIGH);    // CS inaktiv (1)
    SPI.endTransaction();
  }
  // Non-copyable – vermeidet versehentliches Kopieren
  SpiGuard(const SpiGuard&)            = delete;
  SpiGuard& operator=(const SpiGuard&) = delete;
};

bool OPCHandleData(uint8_t reg, uint8_t cmd, uint8_t count, uint8_t* bytes)
{
  if (!count) { return false; }

  /* ---------- Verfügbarkeit prüfen ---------- */
  bool ok = false;
  for (uint8_t attempt = 0; attempt < kMaxSpiRetries; ++attempt) {
    {                           // Scope 1 → SpiGuard zerstört sich am Block-Ende
      SpiGuard spi;             // → beginTransaction + CS low
      if (OPCAvailable(reg)) {  // Gerät antwortet mit 0xF3 ?
        ok = true;
        break;
      }
    }                           // CS high + endTransaction hier automatisch
    delay(20);
  }
  if (!ok) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Unable to read reg 0x%02X"), D_CMND_OPC, reg);
    OPC.setmode |= OPC_OVERRIDE;        // Re-Init erzwingen
    return false;
  }

  /* ---------- Daten lesen ---------- */
  {
    SpiGuard spi;               // Scope 2 – eigentlicher Transfer
    delayMicroseconds(20);
    while (count--) {
      *bytes++ = SPI.transfer(cmd);
    }
  }                             // CS high, endTransaction

  return true;
}

void OPCWriteControl(OPC_CommandCodes_t cmd) {
  //if (cmd != OPC_WRITE_ON && cmd != OPC_WRITE_OFF && cmd != OPC_WRITE_LOFF && cmd != OPC_WRITE_LON) {
  //  AddLog(LOG_LEVEL_INFO, PSTR("%s: Invalid write command %d"), D_CMND_OPC, cmd);
  //  return;
  //}

  uint8_t type, reg, command;
  type    = (__builtin_ctz(OPC.mode & 0x0F));
  reg     = pgm_read_byte(&OPCCommand[cmd].reg);           //memcpy_P(&reg, &OPCCommand[cmd].reg, sizeof(uint8_t));          // Read register from PROGMEM
  command = pgm_read_byte(&OPCCommand[cmd].val[type]); //memcpy_P(&command, &OPCCommand[cmd].val[OPC.type], sizeof(uint8_t));  // Read value based on OPC.type
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
  DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: set mode to %u"),mode);

  if (mode > 999) {
    OPC.interval = mode / 1000;
    AddLog(LOG_LEVEL_INFO, PSTR("OPC: Set period to %d sec"), OPC.interval);
    return;  // Keine weitere Modusverarbeitung, wenn ein Intervall gesetzt wurde
  }

  switch(mode){
    case 0:
      OPC.setmode &= ~(OPC_FONOFF | OPC_LONOFF); // Alle Modi ausschalten
      break;//OPCWriteControl(OPC_WRITE_FOFF);
      //delay(20);
      //OPCWriteControl(OPC_WRITE_LOFF);
    case 1:
      OPC.setmode |= (OPC_FONOFF | OPC_LONOFF); 
      break;
    case 2:
      OPC.setmode &= ~OPC_FONOFF; 
      break;
    case 3:
      OPC.setmode |= OPC_FONOFF; 
      break;
    case 4:
      OPC.setmode &= ~OPC_PMHIST;
      break;
    case 5:
      OPC.setmode |= OPC_PMHIST;
      break;
    case 6:
      OPC.setmode |= OPC_CONFIG;
      break;
    case 7:
      OPC.setmode |= OPC_STATUS;
      break;
    case 8:
      OPC.setmode |= OPC_LGAIN;
      break;
    case 9:
      OPC.setmode |= OPC_HGAIN;
      break;
    case 10:
      OPC.setmode |= OPC_RESET;
      break;
    //case 5:
    //  OPC.mode |= OPC_PMHIST;
    //  break;
  }
}

void OPCShow(bool json) { // Wird so oft aufgerufen wie read_sensors()
  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":{\"Mode\":%i"), D_CMND_OPC, OPC.mode);
    ResponseAppend_P(PSTR(",\"First_Publish\":%d"), OPC.data.isFirst);
    if ((OPC.mode & 0x0F) && (OPC.mode & OPC_STATUS))
    {
        ResponseAppend_P(PSTR(",\"Status\":{"));
        ResponseAppend_P(PSTR("\"fan_on\":%u,"), OPC.status.fan_on);
        ResponseAppend_P(PSTR("\"laserdac_on\":%u,"), OPC.status.laser_on);
        ResponseAppend_P(PSTR("\"fandac_val\":%u,"), OPC.status.fan_dac);
        ResponseAppend_P(PSTR("\"laserdac_val\":%u,"), OPC.status.laser_dac);
        ResponseAppend_P(PSTR("\"laser_switch\":%u,"), OPC.status.laser_sw);
        ResponseAppend_P(PSTR("\"gain_setting\":%u}"), OPC.status.gain);
      if (OPC.mode & OPC_STATUS) {
          AddLog(LOG_LEVEL_INFO, PSTR("OPC: RESET STATUS FLAG"));
          OPC.mode ^= OPC_STATUS; 
        }
    }
    if ((OPC.mode & 0x0F) && (OPC.mode & OPC_CONFIG))
    {
        ResponseAppend_P(PSTR(",\"Config\":{"));

        // 1. bin_boundaries_adc[25]
        ResponseAppend_P(PSTR("\"bin_boundaries_adc\":["));
        for (int i = 0; i < 25; i++) {
          ResponseAppend_P(PSTR("%u"), OPC.config.bin_boundaries_adc[i]);
          if (i < 24) ResponseAppend_P(PSTR(","));
        }
        ResponseAppend_P(PSTR("],"));

        // 2. bin_boundaries_um100[25]
        ResponseAppend_P(PSTR("\"bin_boundaries_um100\":["));
        for (int i = 0; i < 25; i++) {
          ResponseAppend_P(PSTR("%u"), OPC.config.bin_boundaries_um100[i]);
          if (i < 24) ResponseAppend_P(PSTR(","));
        }
        ResponseAppend_P(PSTR("],"));

        // 3. bin_weightings[24]
        ResponseAppend_P(PSTR("\"bin_weightings\":["));
        for (int i = 0; i < 24; i++) {
          ResponseAppend_P(PSTR("%u"), OPC.config.bin_weightings[i]);
          if (i < 23) ResponseAppend_P(PSTR(","));
        }
        ResponseAppend_P(PSTR("],"));

        // 4. einzelne PM-Grenzen
        ResponseAppend_P(PSTR("\"pm_dia_a\":%u,"), OPC.config.pm_dia_a);
        ResponseAppend_P(PSTR("\"pm_dia_b\":%u,"), OPC.config.pm_dia_b);
        ResponseAppend_P(PSTR("\"pm_dia_c\":%u,"), OPC.config.pm_dia_c);

        // 5. weitere Skalierungs- und Flags-Werte
        ResponseAppend_P(PSTR("\"max_tof\":%u,"), OPC.config.max_tof);
        ResponseAppend_P(PSTR("\"am_sampling_interval_count\":%u,"), OPC.config.am_sampling_interval_count);
        ResponseAppend_P(PSTR("\"am_idle_interval_count\":%u,"), OPC.config.am_idle_interval_count);
        ResponseAppend_P(PSTR("\"am_max_data_arrays\":%u,"), OPC.config.am_max_data_arrays);
        ResponseAppend_P(PSTR("\"am_only_save_pm\":%u,"), OPC.config.am_only_save_pm);
        ResponseAppend_P(PSTR("\"am_fan_on_idle\":%u,"), OPC.config.am_fan_on_idle);
        ResponseAppend_P(PSTR("\"am_laser_on_idle\":%u,"), OPC.config.am_laser_on_idle);
        ResponseAppend_P(PSTR("\"tof_to_sfr_factor\":%u,"), OPC.config.tof_to_sfr_factor);
        ResponseAppend_P(PSTR("\"pvp\":%u,"), OPC.config.pvp);
        ResponseAppend_P(PSTR("\"bin_weighting_index\":%u}"), OPC.config.bin_weighting_index);

        // Ende des Config-Objects
        //ResponseAppend_P(PSTR("}"));
        if (OPC.mode & OPC_CONFIG) {
          DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: RESET CONFIG FLAG"));
          OPC.mode ^= OPC_CONFIG; 
        }
        
    }


  } else {
#ifdef USE_WEBSERVER
    WSContentSend_P(PSTR("{s}%s Mode{m}0x%02X{e}"), D_CMND_OPC, OPC.mode);
    WSContentSend_P(PSTR("{s}%s Gain{m}%u{e}"), D_CMND_OPC, OPC.status.gain);
#endif
  }
  DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC.data.isFirst= %d"), OPC.data.isFirst);

  if (OPC.data.isFirst) {
    OPC.data.isFirst = false;  // consume – avoids duplicate display until next packet
  }


  if ((!(OPC.mode & 0x0F))            // no OPC detected
      || (OPC.mode & OPC_OVERRIDE)
      || (!(OPC.mode & OPC_FONOFF))
      || (!(OPC.mode & OPC_LONOFF)))
      {
      if (json) { 
        //AddLog(LOG_LEVEL_INFO, PSTR("OPC: Look no further"));
        ResponseJsonEnd(); 
      }
      return;
  }
      // Zurücksetzen, damit die Config nicht bei jeder Show erscheint
      //OPC.config.news = false;
  
  char types[11];
  strlcpy(types, OPC.types, sizeof(types));

  //float pm1 = 0.0f;
  //float pm2_5 = 0.0f;
  //float pm10 = 0.0f;

/*   char conc_str_array[200] = {0};
  uint32_t active = (__builtin_ctz(OPC.mode & 0x0F));  // OPC-Typ
  //AddLog(LOG_LEVEL_INFO, PSTR("OPC: mode %u"), OPC.mode);
  void *hist_ptr = *OPC_AllocTable[active].ptr;  // Lookup-Tabelle
  if ((OPC.mode & OPC_PMHIST) && hist_ptr && OPC.data.available) {
    OPC.data.available = false;
    if ((1 << active) == OPC_TYPE_N2) {
      hist_n2data_t *n2 = (hist_n2data_t *)hist_ptr;
      OPC.data.pm.a   = n2->pm_a;
      OPC.data.pm.b = n2->pm_b;
      OPC.data.pm.c  = n2->pm_c;
      AddLog(LOG_LEVEL_INFO, PSTR("N2: PM1=%f, PM2.5=%f, PM10=%f"), 
      n2->pm_a, n2->pm_b, n2->pm_c);
    } else if ((1 << active) == OPC_TYPE_N3) {
      hist_n3data_t *n3 = (hist_n3data_t *)hist_ptr;
      OPC.data.pm.a  = n3->pm_a;
      OPC.data.pm.b  = n3->pm_b;
      OPC.data.pm.c  = n3->pm_c;

      if (n3->flow == 0 || n3->period == 0 || (0 == memcmp(n3->bins, binsZero, sizeof(n3->bins)))) {
        strlcpy(OPC.data.concstr, kBinsZeroString, sizeof(OPC.data.concstr));
        OPC.data.flow = 0.0;
      } else {
        char conc_str_array[200] = {0};
        float norm = 1.0f / (n3->period * n3->flow * 0.01f);
        for (int i = 0; i < 24; i++) {
          float conc = n3->bins[i] * norm;
          char tmp[16];
          dtostrfd(conc, 2, tmp);
          strlcat(conc_str_array, tmp, sizeof(conc_str_array));
          if (i < 23) strlcat(conc_str_array, PSTR(","), sizeof(conc_str_array));
        }
        strlcpy(OPC.data.concstr, conc_str_array, sizeof(OPC.data.concstr));
        OPC.data.flow = n3->flow * 0.0006f;
      }
      OPC.data.period = n3->period / 100.0f;
      uint32_t h_10 = ((uint32_t)n3->humi * 1000U) >> 16;
      int32_t t_10 = -450 + (((uint32_t)n3->temp * 1750U) >> 16);
      OPC.data.humi = ConvertHumidity(h_10 / 10.0f);
      OPC.data.temp = ConvertTemp(t_10 / 10.0f);
      OPC.data.abs_humi = CalcTempHumToAbsHum(OPC.data.temp, OPC.data.humi); */
      //AddLog(LOG_LEVEL_INFO, PSTR("N3: PM1=%f, PM2.5=%f, PM10=%f, Temp=%f, Humi=%f, Flow rate=%f, Period=%f"), 
      //        n3->pm_a, n3->pm_b, n3->pm_c, OPC.data.temp, OPC.data.humi, OPC.data.flow, OPC.data.period);
      /*       
      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Bin boundaries (ADC):"));
      for (int i = 0; i < 25; i++) {
        AddLog(LOG_LEVEL_INFO, PSTR("  ADC[%02d] = %u"), i, OPC.config.bin_boundaries_adc[i]);
      }

      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Bin boundaries (um * 100):"));
      for (int i = 0; i < 25; i++) {
        AddLog(LOG_LEVEL_INFO, PSTR("  UM100[%02d] = %u"), i, OPC.config.bin_boundaries_um100[i]);
      }

      AddLog(LOG_LEVEL_INFO, PSTR("OPC: Bin weightings:"));
      for (int i = 0; i < 24; i++) {
        AddLog(LOG_LEVEL_INFO, PSTR("  Weight[%02d] = %u"), i, OPC.config.bin_weightings[i]);
      } */
    



      //for (int i = 0; i < 24; i++) {
      //  AddLog(LOG_LEVEL_INFO, PSTR("Bin[%02d] = %u"), i, n3->bins[i]);
      //}
/*       DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR(
        "mtof_a=%u, mtof_c=%u, mtof_e=%u, mtof_g=%u, "
        "period=%f, flow=%f, temp=%f, humi=%f, "
        "pm_a=%f, pm_b=%f, pm_c=%f, "
        "rej_cnt_gli=%u, rej_cnt_lon=%u, rej_cnt_rat=%u, rej_cnt_oor=%u, "
        "fan_rev_cnt=%u, las_status=%u"
      ),
        n3->mtof_a, n3->mtof_c, n3->mtof_e, n3->mtof_g,
        OPC.data.period, OPC.data.flow, OPC.data.temp, OPC.data.humi,
        n3->pm_a, n3->pm_b, n3->pm_c,
        n3->rej_cnt_gli, n3->rej_cnt_lon, n3->rej_cnt_rat, n3->rej_cnt_oor,
        n3->fan_rev_cnt, n3->las_status
      ); */
    
  if (json) {
    ResponseAppend_P(PSTR(",\"Gain\":%u"), OPC.status.gain);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_a), &OPC.data.pm.a);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_b), &OPC.data.pm.b);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_c), &OPC.data.pm.c);

    if (OPC.mode & OPC_PMHIST) {
      ResponseAppend_P(PSTR(",\"Histogram\":[%s]"),  OPC.data.concstr);
      ResponseAppend_P(PSTR(",\"Flow\":%3_f"),       &OPC.data.flow);
      ResponseAppend_P(PSTR(",\"Period\":%2_f"),     &OPC.data.period);
      ResponseAppend_P(PSTR(","));
      ResponseAppendTHD(OPC.data.temp, OPC.data.humi);
      ResponseAppend_P(PSTR(",\"" D_JSON_AHUM "\":%4_f"), &OPC.data.abs_humi);
    }
    ResponseJsonEnd();
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, OPC.data.pm.dia_a, &OPC.data.pm.a);
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, OPC.data.pm.dia_b, &OPC.data.pm.b);
    WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, OPC.data.pm.dia_c, &OPC.data.pm.c);

    if ((OPC.mode & OPC_PMHIST) && ((1U << __builtin_ctz(OPC.mode & 0x0F)) == OPC_TYPE_N3)) {
      WSContentSend_PD(PSTR("{s}%s " D_FLOW_RATE "{m}%3_f " D_UNIT_LITER_PER_MINUTE "{e}"), D_CMND_OPC, &OPC.data.flow);
      WSContentSend_PD(PSTR("{s}%s " D_JSON_PERIOD "{m}%3_f " D_UNIT_SECOND "{e}"),            D_CMND_OPC, &OPC.data.period);
      WSContentSend_THD(D_CMND_OPC, OPC.data.temp, OPC.data.humi);
      WSContentSend_PD(HTTP_SNS_F_ABS_HUM, D_CMND_OPC, 4, &OPC.data.abs_humi);
    }
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