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
#define OPC_MAX_SENSORS           MAX_OPC  // Max 4 OPC sensors

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

#ifdef USE_WEBSERVER
#define WEB_HANDLE_OPC "s124"
#endif // USE_WEBSERVER

#define OPC_SLOW_INTERVAL 4
#define OPC_DEFAULT_INTERVAL 2

const char S_JSON_OPC_COMMAND_NVALUE[] PROGMEM = "{\"" D_CMND_OPC "%s\":%d}";

const char kOPC_Types[] PROGMEM = "OPC-N2|OPC-N3|OPC-R1|OPC-R2";

const char kBinsZeroString[] PROGMEM = 
  "0,0,0,0,0,0,0,0,"
  "0,0,0,0,0,0,0,0,"
  "0,0,0,0,0,0,0,0";

#ifdef USE_WEBSERVER
const char kOpcEmptyCell[] PROGMEM = "<td></td><td style='text-align:right'>-</td>";
#endif

static const uint8_t kMaxSpiRetries = 10; 

struct OPC_T {
  uint16_t mode, setmode;
  uint8_t interval    = OPC_DEFAULT_INTERVAL;
  uint8_t setinterval = OPC_DEFAULT_INTERVAL;
  int8_t  cs_pin      = -1;
  char types[8];
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
    char concstr[144];

    bool available;
    bool isFirst;
  } data;
  hist_n3data_t *hist_n3 = nullptr;
};

OPC_T opc[OPC_MAX_SENSORS];
uint8_t opc_count = 0;
uint8_t opc_idx   = 0;
#define OPC opc[opc_idx]

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
  if (SPI_MOSI_MISO != TasmotaGlobal.spi_enabled) { return; }
  for (uint32_t i = 0; i < OPC_MAX_SENSORS; i++) {
    if (PinUsed(GPIO_OPC_CS, i)) {
      opc_idx = opc_count;
      OPC.cs_pin  = Pin(GPIO_OPC_CS, i);
      OPC.setmode = 0x10;
      OPC.mode    = 0;
      pinMode(OPC.cs_pin, OUTPUT);
      digitalWrite(OPC.cs_pin, HIGH);
      opc_count++;
    }
  }
  if (opc_count > 0) {
#ifdef ESP32
    SPI.begin(Pin(GPIO_SPI_CLK), Pin(GPIO_SPI_MISO), Pin(GPIO_SPI_MOSI), -1);
#else
    SPI.begin();
#endif
    AddLog(LOG_LEVEL_INFO, PSTR("OPC: %d sensor(s) configured on SPI bus"), opc_count);
  }
  opc_idx = 0;
}

void OPCAllocateMem(uint32_t type) {
  void  **ptr;
  size_t  size;
  switch (type) {
    case 1: ptr = (void**)&OPC.hist_n3; size = sizeof(hist_n3data_t); break;
    default: return;
  }
  if (ptr && *ptr) {
    free(*ptr);
    *ptr = nullptr;
    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("Free: %u"), size);
  }
  *ptr = calloc(1, size);
  if (!*ptr) {
    AddLog(LOG_LEVEL_ERROR, PSTR("Memory allocation failed %u"), size);
    return;
  } else { DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("Alloc: %u"), size); }
}

bool OPCInit(void) {
  uint8_t regInfo = pgm_read_byte(&OPCCommand[OPC_READ_INFO].reg);
  uint8_t lenInfo = pgm_read_byte(&OPCCommand[OPC_READ_INFO].val[0]);
  unsigned char info[60];
  if (!OPCHandleData(regInfo, 0xF3, lenInfo, info)) {
    return false;
  }

  for (uint32_t i = 0; i < 4; i++) {
    GetTextIndexed(OPC.types, sizeof(OPC.types), i, kOPC_Types);
    if (strstr((const char*)info, OPC.types) != NULL) {
      AddLog(LOG_LEVEL_INFO, PSTR("%s: OPC-%d found %s"), D_CMND_OPC, opc_idx + 1, OPC.types);
      OPC.mode |= (1 << i);
      return true;
    }   
  }
  return false;
}


static void OPCProcessMeasurements(void) {
  if (!OPC.data.available) { return; }
  if (OPC.mode & OPC_PMHIST) {
    if (!(OPC.mode & OPC_TYPE_N3)) { return; }  // Only N3 histogram supported
    if (!OPC.hist_n3) { return; }
    {
      auto *n3 = static_cast<hist_n3data_t *>((void*)OPC.hist_n3);

    
      uint8_t period_int;
      if (OPC.status.gain == 3 || OPC.status.gain == 2) {
        period_int = (uint8_t)((n3->period + 25U) / 50U);
      } else {
        period_int = (uint8_t)((n3->period + 50U) / 100U);
      }
      if (period_int != OPC.interval) {
        return;
      }
      OPC.data.period = n3->period * 0.01f; 

      OPC.data.pm.a = n3->pm_a;
      OPC.data.pm.b = n3->pm_b;
      OPC.data.pm.c = n3->pm_c;

      // Check for zero bins
      bool bins_zero = true;
      for (uint8_t bi = 0; bi < 24; bi++) { if (n3->bins[bi]) { bins_zero = false; break; } }
      if (n3->flow == 0 || n3->period == 0 || bins_zero) {
        strlcpy(OPC.data.concstr, kBinsZeroString, sizeof(OPC.data.concstr));
        OPC.data.flow = 0.0f;
      } else {
        OPC.data.concstr[0] = '\0';
        const float norm = 10000.0f / ((uint32_t)n3->period * n3->flow);
        uint8_t bin_count = (OPC.status.gain == 0 || OPC.status.gain == 1) ? 12 : 24;
        for (uint8_t i = 0; i < bin_count; ++i) {
          const float conc = n3->bins[i] * norm;
          char        tmp[16];
          dtostrfd(conc, 2, tmp);
          strlcat(OPC.data.concstr, tmp, sizeof(OPC.data.concstr));
          if (i < (bin_count - 1)) strlcat(OPC.data.concstr, PSTR(","), sizeof(OPC.data.concstr));
        }
        OPC.data.flow = n3->flow * 0.0006f;             // l/min
      }

      // Environental data


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
  if (SPI_MOSI_MISO != TasmotaGlobal.spi_enabled) { return; }
  for (opc_idx = 0; opc_idx < opc_count; opc_idx++) {
    uint16_t diff = (OPC.setmode ^ OPC.mode); //ignore the lowest four bits, these are for the sensor types

    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: setmode=0x%03X, mode=0x%03X, diff=0x%03X"), 
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
        continue;
      }
    }
    if (diff & OPC_STATUS) {
      if (OPC.setmode & OPC_STATUS) {
        OPC.setmode ^= OPC_STATUS; 
        DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC: READ STATUS TO STRUCT"));
        OPCReadDataToStruct(OPC_READ_STATUS); // if boolean, then successful change mode.
        OPC.mode ^= OPC_STATUS; 
        MqttPublishSensor();
        continue;
      }
    }

    if (diff & OPC_FONOFF) {
      OPCWriteControl((OPC.setmode & OPC_FONOFF) ? OPC_WRITE_FON : OPC_WRITE_FOFF);
      OPC.mode ^= OPC_FONOFF;
      continue;
    } else if (diff & OPC_LONOFF) {
      OPCWriteControl((OPC.setmode & OPC_LONOFF) ? OPC_WRITE_LON : OPC_WRITE_LOFF);
      OPC.mode ^= OPC_LONOFF;
      continue;
    } else if (diff & OPC_LGAIN) {
      OPCWriteControl(OPC_WRITE_LGAIN);
      OPC.setmode &= ~OPC_LGAIN;
      OPC.setmode ^= OPC_STATUS;
      continue;
    } else if (diff & OPC_HGAIN) {
      OPCWriteControl(OPC_WRITE_HGAIN);
      OPC.setmode &= ~OPC_HGAIN;
      OPC.setmode ^= OPC_STATUS;
      continue;
    } else if (diff & OPC_RESET) {
      unsigned char ret;
      OPCHandleData(0x06, 0xF3, 1, &ret);
      AddLog(LOG_LEVEL_INFO, PSTR("%s: returned 0x%02X"), D_CMND_OPC, ret);
      OPC.setmode ^= OPC_RESET;
      OPC.mode &= ~0x0F;
      continue;
    }

    if ((TasmotaGlobal.uptime % OPC.interval) == 0) {  
      if ((OPC.mode & 0x0F) == 0) { //if no OPC, do the init!
        AddLog(LOG_LEVEL_INFO, PSTR("%s: Searching sensor..."), D_CMND_OPC);
        if (OPCInit()) {
          OPC.setmode = OPC.mode &= ~0x10;
          OPC.setmode ^= OPC_CONFIG;
          OPC.setmode ^= OPC_STATUS;
        }
        continue;
      }
      // READ INTERVAL
      if (diff & OPC_PMHIST) {
        OPC.mode ^= OPC_PMHIST;
        if ((OPC.mode & OPC_PMHIST)) {
          OPCAllocateMem(__builtin_ctz(OPC.mode & 0x0F));
        }
      }
      if ((OPC.mode & 0x60) == 0x60) { //fan on and laser on?
        if (!(OPC.mode & OPC_PMHIST)) {
          OPCReadDataToStruct(OPC_READ_PM);
        } else {
          OPCReadDataToStruct(OPC_READ_HIST);
        }  
      } 
      OPCProcessMeasurements(); 
      if ((OPC.data.isFirst))  {
        MqttPublishSensor();
      }
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
  void* target = nullptr;
  switch (cmd) {
    case OPC_READ_PM:     target = &OPC.data.pm;    break;
    case OPC_READ_CONFIG: target = &OPC.config;     break;
    case OPC_READ_STATUS: target = &OPC.status;     break;
    case OPC_READ_HIST: {
      target = (OPC.mode & OPC_TYPE_N3) ? (void*)OPC.hist_n3 : nullptr;
      break;
    }
    default: break;
  }
  if (!target) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s: Invalid target for command %d (len=%u)"), D_CMND_OPC, cmd, len);
    return;
  }
  uint8_t data[len];
  if (OPCHandleData(reg, 0xF3, len, data)) {
    DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("%s: handle reg %u len %u"), D_CMND_OPC, reg, len);
    if (cmd == OPC_READ_PM || cmd == OPC_READ_HIST ) {  
      if (!CheckCRC(data, len)) { AddLog(LOG_LEVEL_ERROR, PSTR("CRC check failed.")); return; }
      memcpy(target, data, len-2);
      OPC.data.available = true;
      return;
    }
    if (cmd == OPC_READ_CONFIG || cmd == OPC_READ_STATUS) {  
      memcpy(target, data, len);
      return;
    }
  }
}

bool CheckCRC(unsigned char data[], size_t length) {
  if (data == NULL || length < 2) { return false; }
  uint16_t crc = 0xFFFF ;
  for (size_t i = 0; i < length - 2; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
        crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return (crc == ((data[length-1] << 8) | data[length-2]));
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
    digitalWrite(OPC.cs_pin, LOW);           // CS aktiv (0)
  }
  ~SpiGuard() {
    digitalWrite(OPC.cs_pin, HIGH);          // CS inaktiv (1)
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
  uint8_t type = __builtin_ctz(OPC.mode & 0x0F);
  uint8_t reg  = pgm_read_byte(&OPCCommand[cmd].reg);
  uint8_t val  = pgm_read_byte(&OPCCommand[cmd].val[type]);
  AddLog(LOG_LEVEL_INFO, PSTR("%s: Writing 0x%02X to reg 0x%02X (type=%u)"), D_CMND_OPC, val, reg, type);
  unsigned char ret;
  OPCHandleData(reg, val, 1, &ret);
  AddLog(LOG_LEVEL_INFO, PSTR("%s: returned 0x%02X"), D_CMND_OPC, ret);
}

bool OPCCmd(void)
{
  if (XdrvMailbox.data_len > 0) {
    int16_t command = XdrvMailbox.payload;
    uint8_t target  = 0;  // 0 = default (OPC-1)

    // "SENSOR124 2 5" → target=2, command=5  |  "SENSOR124 5" → target=0, command=5
    char *space = strchr(XdrvMailbox.data, ' ');
    if (space) {
      target  = atoi(XdrvMailbox.data);   // 1-based OPC index
      command = atoi(space + 1);
    }

    if (target > 0 && target <= opc_count) {
      opc_idx = target - 1;
      OPCSelectMode(command);
    } else {
      opc_idx = 0;
      OPCSelectMode(command);
    }
    opc_idx = 0;
    Response_P(S_JSON_OPC_COMMAND_NVALUE, XdrvMailbox.command, command);
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
      OPC.setmode &= ~(OPC_FONOFF | OPC_LONOFF);
      break;
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
  }
}

void OPCShow(bool json) { // Wird so oft aufgerufen wie read_sensors()
  for (opc_idx = 0; opc_idx < opc_count; opc_idx++) {
  // JSON-Key: "OPC" bei 1 Sensor, "OPC-1"/"OPC-2" bei mehreren
  char opc_key[8];
  if (opc_count > 1) {
    snprintf_P(opc_key, sizeof(opc_key), PSTR("%s%c%d"), D_CMND_OPC, IndexSeparator(), opc_idx + 1);
  } else {
    strlcpy(opc_key, D_CMND_OPC, sizeof(opc_key));
  }
  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":{\"Mode\":%i"), opc_key, OPC.mode);
    ResponseAppend_P(PSTR(",\"First_Publish\":%d"), OPC.data.isFirst);
    if ((OPC.mode & 0x0F) && (OPC.mode & OPC_STATUS))
    {
        ResponseAppend_P(PSTR(",\"Status\":{\"fan_on\":%u,\"laserdac_on\":%u,\"fandac_val\":%u,\"laserdac_val\":%u,\"laser_switch\":%u,\"gain_setting\":%u}"),
          OPC.status.fan_on, OPC.status.laser_on, OPC.status.fan_dac,
          OPC.status.laser_dac, OPC.status.laser_sw, OPC.status.gain);
        OPC.mode ^= OPC_STATUS;
    }
    if ((OPC.mode & 0x0F) && (OPC.mode & OPC_CONFIG))
    {
        ResponseAppend_P(PSTR(",\"Config\":{"));

        auto appendU16Array = [](const char *key, const uint16_t *arr, uint8_t cnt) {
          ResponseAppend_P(PSTR("\"%s\":["), key);
          for (uint8_t i = 0; i < cnt; i++) {
            ResponseAppend_P((i < cnt - 1) ? PSTR("%u,") : PSTR("%u"), arr[i]);
          }
          ResponseAppend_P(PSTR("],"));
        };
        appendU16Array("bin_boundaries_adc",   OPC.config.bin_boundaries_adc,  25);
        appendU16Array("bin_boundaries_um100",  OPC.config.bin_boundaries_um100, 25);
        appendU16Array("bin_weightings",        OPC.config.bin_weightings,       24);

        ResponseAppend_P(PSTR(
          "\"pm_dia_a\":%u,\"pm_dia_b\":%u,\"pm_dia_c\":%u,"
          "\"max_tof\":%u,\"am_sampling_interval_count\":%u,"
          "\"am_idle_interval_count\":%u,\"am_max_data_arrays\":%u,"
          "\"am_only_save_pm\":%u,\"am_fan_on_idle\":%u,"
          "\"am_laser_on_idle\":%u,\"tof_to_sfr_factor\":%u,"
          "\"pvp\":%u,\"bin_weighting_index\":%u}"),
          OPC.config.pm_dia_a, OPC.config.pm_dia_b, OPC.config.pm_dia_c,
          OPC.config.max_tof, OPC.config.am_sampling_interval_count,
          OPC.config.am_idle_interval_count, OPC.config.am_max_data_arrays,
          OPC.config.am_only_save_pm, OPC.config.am_fan_on_idle,
          OPC.config.am_laser_on_idle, OPC.config.tof_to_sfr_factor,
          OPC.config.pvp, OPC.config.bin_weighting_index);

        OPC.mode ^= OPC_CONFIG;
    }

  }
  DEBUG_SENSOR_LOG(LOG_LEVEL_INFO, PSTR("OPC.data.isFirst= %d"), OPC.data.isFirst);

  if (OPC.data.isFirst) {
    OPC.data.isFirst = false;  // consume – avoids duplicate display until next packet
  }

  if ((!(OPC.mode & 0x0F))
      || (OPC.mode & OPC_OVERRIDE)
      || (!(OPC.mode & OPC_FONOFF))
      || (!(OPC.mode & OPC_LONOFF)))
      {
      if (json) { 
        ResponseJsonEnd(); 
      }
      continue;
  }

  if (json) {
    ResponseAppend_P(PSTR(",\"Gain\":%u"), OPC.status.gain);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_a), &OPC.data.pm.a);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_b), &OPC.data.pm.b);
    ResponseAppend_P(PSTR(",\"PM%s\":%1_f"), OPCReplaceDotWithUnderscore(OPC.data.pm.dia_c), &OPC.data.pm.c);

    if (OPC.mode & OPC_PMHIST) {
      ResponseAppend_P(PSTR(",\"Hist\":[%s]"),  OPC.data.concstr);
      ResponseAppend_P(PSTR(",\"Flow\":%3_f"),       &OPC.data.flow);
      ResponseAppend_P(PSTR(",\"Period\":%2_f"),     &OPC.data.period);
      ResponseAppend_P(PSTR(","));
      ResponseAppendTHD(OPC.data.temp, OPC.data.humi);
      ResponseAppend_P(PSTR(",\"" D_JSON_AHUM "\":%4_f"), &OPC.data.abs_humi);
    }
    ResponseJsonEnd();
  }
  } // for opc_idx

#ifdef USE_WEBSERVER
  if (!json) {
    #define OPC_AL "<td style='text-align:right'>"
    bool rdy[OPC_MAX_SENSORS];
    bool any_ready = false, any_n3h = false;
    for (uint8_t i = 0; i < opc_count; i++) {
      rdy[i] = (opc[i].mode & 0x0F) && !(opc[i].mode & OPC_OVERRIDE)
            && (opc[i].mode & OPC_FONOFF) && (opc[i].mode & OPC_LONOFF);
      if (rdy[i]) any_ready = true;
      if (rdy[i] && (opc[i].mode & OPC_PMHIST)
          && ((1U << __builtin_ctz(opc[i].mode & 0x0F)) == OPC_TYPE_N3))
        any_n3h = true;
    }

    if (opc_count < 2) {
      // Single sensor: standard vertical layout
      WSContentSend_P(PSTR("{s}%s Mode{m}0x%02X{e}"), D_CMND_OPC, opc[0].mode);
      if (rdy[0]) {
        WSContentSend_P(PSTR("{s}%s Gain{m}%u{e}"), D_CMND_OPC, opc[0].status.gain);
        WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, opc[0].data.pm.dia_a, &opc[0].data.pm.a);
        WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, opc[0].data.pm.dia_b, &opc[0].data.pm.b);
        WSContentSend_PD(HTTP_SNS_F_ENVIRONMENTAL_CONCENTRATION, D_CMND_OPC, opc[0].data.pm.dia_c, &opc[0].data.pm.c);
        if (any_n3h) {
          WSContentSend_PD(PSTR("{s}%s " D_FLOW_RATE "{m}%3_f " D_UNIT_LITER_PER_MINUTE "{e}"), D_CMND_OPC, &opc[0].data.flow);
          WSContentSend_PD(PSTR("{s}%s " D_JSON_PERIOD "{m}%2_f " D_UNIT_SECOND "{e}"), D_CMND_OPC, &opc[0].data.period);
          WSContentSend_THD(D_CMND_OPC, opc[0].data.temp, opc[0].data.humi);
          WSContentSend_PD(HTTP_SNS_F_ABS_HUM, D_CMND_OPC, 4, &opc[0].data.abs_humi);
        }
      }
    } else {
      // Multi-sensor: transposed table (rows=metrics, columns=sensors)
      bool n3h[OPC_MAX_SENSORS];
      for (uint8_t i = 0; i < opc_count; i++) {
        n3h[i] = rdy[i] && (opc[i].mode & OPC_PMHIST)
              && ((1U << __builtin_ctz(opc[i].mode & 0x0F)) == OPC_TYPE_N3);
      }

      // Header
      WSContentSend_P(PSTR("{s}" D_CMND_OPC "</th>"));
      for (uint8_t i = 0; i < opc_count; i++) {
        WSContentSend_P(PSTR("<td></td>" OPC_AL "%s%c%d</td>"),
                        D_CMND_OPC, IndexSeparator(), i + 1);
      }
      WSContentSend_P(PSTR("{e}"));

      // Mode
      WSContentSend_P(PSTR("{s}Mode</th>"));
      for (uint8_t i = 0; i < opc_count; i++) {
        WSContentSend_P(PSTR("<td></td>" OPC_AL "0x%02X</td>"), opc[i].mode);
      }
      WSContentSend_P(PSTR("{e}"));

      if (any_ready) {
        // Gain
        WSContentSend_P(PSTR("{s}Gain</th>"));
        for (uint8_t i = 0; i < opc_count; i++) {
          if (rdy[i]) WSContentSend_P(PSTR("<td></td>" OPC_AL "%u</td>"), opc[i].status.gain);
          else        WSContentSend_P(kOpcEmptyCell);
        }
        WSContentSend_P(PSTR("{e}"));

        // PM row labels from first ready sensor
        const char *lbl_a = "A", *lbl_b = "B", *lbl_c = "C";
        for (uint8_t i = 0; i < opc_count; i++) {
          if (rdy[i]) {
            lbl_a = opc[i].data.pm.dia_a;
            lbl_b = opc[i].data.pm.dia_b;
            lbl_c = opc[i].data.pm.dia_c;
            break;
          }
        }

        // PM A
        WSContentSend_P(PSTR("{s}PM%s</th>"), lbl_a);
        for (uint8_t i = 0; i < opc_count; i++) {
          if (rdy[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%1_f " D_UNIT_MICROGRAM_PER_CUBIC_METER "</td>"), &opc[i].data.pm.a);
          else        WSContentSend_P(kOpcEmptyCell);
        }
        WSContentSend_P(PSTR("{e}"));

        // PM B
        WSContentSend_P(PSTR("{s}PM%s</th>"), lbl_b);
        for (uint8_t i = 0; i < opc_count; i++) {
          if (rdy[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%1_f " D_UNIT_MICROGRAM_PER_CUBIC_METER "</td>"), &opc[i].data.pm.b);
          else        WSContentSend_P(kOpcEmptyCell);
        }
        WSContentSend_P(PSTR("{e}"));

        // PM C
        WSContentSend_P(PSTR("{s}PM%s</th>"), lbl_c);
        for (uint8_t i = 0; i < opc_count; i++) {
          if (rdy[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%1_f " D_UNIT_MICROGRAM_PER_CUBIC_METER "</td>"), &opc[i].data.pm.c);
          else        WSContentSend_P(kOpcEmptyCell);
        }
        WSContentSend_P(PSTR("{e}"));

        if (any_n3h) {
          // Flow
          WSContentSend_P(PSTR("{s}" D_FLOW_RATE "</th>"));
          for (uint8_t i = 0; i < opc_count; i++) {
            if (n3h[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%3_f " D_UNIT_LITER_PER_MINUTE "</td>"), &opc[i].data.flow);
            else        WSContentSend_P(kOpcEmptyCell);
          }
          WSContentSend_P(PSTR("{e}"));

          // Period
          WSContentSend_P(PSTR("{s}" D_JSON_PERIOD "</th>"));
          for (uint8_t i = 0; i < opc_count; i++) {
            if (n3h[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%2_f " D_UNIT_SECOND "</td>"), &opc[i].data.period);
            else        WSContentSend_P(kOpcEmptyCell);
          }
          WSContentSend_P(PSTR("{e}"));

          // Temperature
          WSContentSend_P(PSTR("{s}" D_TEMPERATURE "</th>"));
          for (uint8_t i = 0; i < opc_count; i++) {
            if (n3h[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%*_f °%c</td>"),
                          Settings->flag2.temperature_resolution, &opc[i].data.temp, TempUnit());
            else        WSContentSend_P(kOpcEmptyCell);
          }
          WSContentSend_P(PSTR("{e}"));

          // Humidity
          WSContentSend_P(PSTR("{s}" D_HUMIDITY "</th>"));
          for (uint8_t i = 0; i < opc_count; i++) {
            if (n3h[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%*_f%%</td>"),
                          Settings->flag2.humidity_resolution, &opc[i].data.humi);
            else        WSContentSend_P(kOpcEmptyCell);
          }
          WSContentSend_P(PSTR("{e}"));

          // Absolute humidity
          WSContentSend_P(PSTR("{s}" D_ABSOLUTE_HUMIDITY "</th>"));
          for (uint8_t i = 0; i < opc_count; i++) {
            if (n3h[i]) WSContentSend_PD(PSTR("<td></td>" OPC_AL "%4_f " D_UNIT_GRAM_PER_CUBIC_METER "</td>"), &opc[i].data.abs_humi);
            else        WSContentSend_P(kOpcEmptyCell);
          }
          WSContentSend_P(PSTR("{e}"));
        }
      }
    }
  }
#endif
}

bool Xsns124(uint32_t function) {
  bool result = false;
  if (FUNC_PRE_INIT == function) {
    OPCPreInit();
  } else if (FUNC_INIT == function) {
    // OPC pin already initialized in FUNC_PRE_INIT
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