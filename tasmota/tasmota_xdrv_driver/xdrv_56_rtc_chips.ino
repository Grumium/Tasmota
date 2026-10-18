/*
  xsns_56_rtc_chips.ino - RTC chip support for Tasmota

  SPDX-FileCopyrightText: 2022 Theo Arends
  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: GPL-3.0-only
*/

#ifdef USE_I2C
#ifdef USE_RTC_CHIPS
/*********************************************************************************************\
 * RTC chip support
 * 
 * #define USE_RV3028
 *   RV-3028-C7 at I2C address 0x52
 *   Used in MSB Master G1
 * #define USE_DS3231
 *   DS1307 and DS3231 at I2C address 0x68
 *   Used by Ulanzi TC001
 * #define USE_BM8563
 *   BM8563 at I2C address 0x51
 *   Used by M5Stack and IOTTIMER (v3)
 * #define USE_PCF85063
 *   PCF85063 at I2C address 0x51
 *   Used by Waveshare ESP32-S3-POE-ETH-8DI-8RO
 * #define USE_PCF85363
 *   PCF85363 at I2C address 0x51
 *   Used by Shelly 3EM
 * #define USE_RX8010
 *   RX8010 at I2C address 0x32
 *   Used by IOTTIMER (v1 and v2)
 * #define USE_RX8030
 *   RX8010 at I2C address 0x32
 *   Used by #23855 
 * #define USE_RX8025
 *   RX8025 at I2C address 0x32
 *   Used by MSB Master G2
 * #define RTC_NTP_SERVER
 *   NTP server from the chip's time, phase from upstream NTP (enables USE_NTP_SERVER)
 * #define USE_RTC_CHIP_HOLDOVER
 *   Chip phase to about 0.1 ms and its rate record, for a restart without GPS or network
 *   Default with USE_GPS, DISABLE_RTC_CHIP_HOLDOVER drops it
\*********************************************************************************************/

#define XDRV_56             56

#ifdef USE_NTP_SERVER
#define RTC_PHASE_SYNC_SECS  3600              // Check the chip phase every hour, s
#define RTC_PHASE_MAX_WINDOW 250000            // Max us between two reads around the step
#define RTC_RELEASE_MAX_RELOADS 3              // Max reloads before releasing anyway
#define RTC_CHIP_PHASE_MAX_MS  150             // Max chip phase error before the chip is set again, ms
#endif  // USE_NTP_SERVER

struct {
  uint32_t (* ReadTime)(void);
  void (* SetTime)(uint32_t);
#ifdef USE_RTC_CHIP_HOLDOVER
  // Optional: set the time with the chip stopped, start it at the second (sets the phase)
  void (* SetTimeHeld)(uint32_t);
  void (* ReleaseTime)(void);
  bool (* AuxWrite)(uint32_t value);
  bool (* AuxRead)(uint32_t* value);
  uint32_t release_flip_us;         // us from start to the first second step: BM8563 1 s, PCF85063 0.5 s
  uint8_t seconds_reg;              // Seconds register, 0 = no phase machinery
  uint8_t drift_reg;                // Register for the ESP drift, 0 = none
  bool stopped;                     // Found stopped at boot: a set was cut off, the time is lost
#endif  // USE_RTC_CHIP_HOLDOVER
  int32_t (* MemRead)(uint8_t *, uint32_t);
  int32_t (* MemWrite)(uint8_t *, uint32_t);
  void (* ShowSensor)(bool);
#ifdef USE_NTP_SERVER
  uint32_t phase_time;              // Chip second waiting for its step, 0 = idle
  uint32_t phase_seen;              // micros() of the last read of that second
  uint8_t phase_polls;              // Reads waiting for the step
  uint8_t phase_tries;              // Retries after a read gap too wide
  uint32_t phase_audit;             // Uptime of the last phase check
#ifdef USE_RTC_CHIP_HOLDOVER
  bool release_pending;             // Time written, chip stopped, waiting for the second
  uint32_t release_armed_ms;        // millis() when the chip was stopped
  uint32_t release_utc;             // Second written to the chip
  uint8_t release_reloads;          // Reloads while stopped
  bool phase_coarse;                // Our phase came from a coarse chip read
  int32_t phase_limit;              // Max phase error at the next check, ms, 0 = default
  bool set_pending;                 // Set the chip at the next second
  bool fix_seconds;                 // Only the seconds are wrong
  bool net_reseat;                  // Set from upstream NTP
  uint8_t armed_stratum;            // Clock stratum when the set was requested
#endif  // USE_RTC_CHIP_HOLDOVER
#endif  // USE_NTP_SERVER
  bool detected;
  int8_t mem_size = -1;
  uint8_t address;
  uint8_t bus;
  char name[10];
} RtcChip;
#ifdef USE_RTC_CHIP_HOLDOVER
// Record in the four alarm registers (minute, hour, day, weekday: 7 + 6 + 6 + 3 bits). The alarm
// disable bit stays set in each. BM8563 and PCF85063A have the same layout
bool RtcAuxWriteAt(uint8_t addr, uint8_t base, uint32_t v) {
  uint8_t alarm[4] = { (uint8_t)(0x80 | (v & 0x7F)),        (uint8_t)(0x80 | ((v >> 7) & 0x3F)),
                       (uint8_t)(0x80 | ((v >> 13) & 0x3F)), (uint8_t)(0x80 | ((v >> 19) & 0x07)) };
  for (uint32_t i = 0; i < 4; i++) {
    if (!I2cWrite8(addr, base + i, alarm[i], RtcChip.bus)) { return false; }
  }
  return true;
}

// false if the clock was lost (seconds bit 7, VL / OS) or an alarm is in use
bool RtcAuxReadAt(uint8_t addr, uint8_t base, uint8_t seconds_reg, uint32_t* v) {
  uint8_t r[4], sec;
  if (!I2cValidRead8(&sec, addr, seconds_reg, RtcChip.bus) || (sec & 0x80)) { return false; }
  for (uint32_t i = 0; i < 4; i++) {
    if (!I2cValidRead8(&r[i], addr, base + i, RtcChip.bus)) { return false; }
    if (!(r[i] & 0x80)) { return false; }                         // Alarm in use
  }
  *v = (r[0] & 0x7F) | ((uint32_t)(r[1] & 0x3F) << 7) | ((uint32_t)(r[2] & 0x3F) << 13) |
       ((uint32_t)(r[3] & 0x07) << 19);
  return true;
}

// Record in three free bytes (RAM, or unused timer and alarm registers)
bool RtcAuxWriteRegs(const uint8_t* reg, uint32_t v) {
  for (uint32_t i = 0; i < 3; i++) {
    if (!I2cWrite8(RtcChip.address, reg[i], (v >> (8 * i)) & 0xFF, RtcChip.bus)) { return false; }
  }
  return true;
}

bool RtcAuxReadRegs(const uint8_t* reg, uint32_t* v) {
  uint8_t b;
  *v = 0;
  for (uint32_t i = 0; i < 3; i++) {
    if (!I2cValidRead8(&b, RtcChip.address, reg[i], RtcChip.bus)) { return false; }
    *v |= (uint32_t)b << (8 * i);
  }
  return true;
}

uint8_t RtcChipReadSeconds(void) {
  uint8_t sec;
  if (!I2cValidRead8(&sec, RtcChip.address, RtcChip.seconds_reg, RtcChip.bus)) { return 0xFF; }
  return sec & 0x7F;                                              // Bit 7 is VL / OS on some chips
}

// DS3231, RV-3028: writing the seconds register restarts the second
void RtcChipRewriteSeconds(void) {
  I2cWrite8(RtcChip.address, RtcChip.seconds_reg, Dec2Bcd(RtcChip.release_utc % 60), RtcChip.bus);
}
#endif  // USE_RTC_CHIP_HOLDOVER

/*********************************************************************************************\
 * RV-3028-C7 RTC Controller
 *
 * I2C Address: 0x52
\*********************************************************************************************/
#ifdef USE_RV3028

#define XI2C_94             94      // See I2CDEVICES.md

#define RV3028_ADDR  0x52           // I2C address of RV-3028-C7

// RV-3028-C7 Register Addresses
#define RV3028_SECONDS      0x00
#define RV3028_MINUTES      0x01
#define RV3028_HOURS        0x02
#define RV3028_WEEKDAY      0x03
#define RV3028_DATE         0x04
#define RV3028_MONTH        0x05
#define RV3028_YEAR         0x06
#define RV3028_STATUS       0x0E
#define RV3028_CONTROL1     0x0F
#define RV3028_CONTROL2     0x10

// Status register bits
#define RV3028_PORF         0       // Power-on Reset flag (bit 0 in STATUS register)

/*-------------------------------------------------------------------------------------------*\
 * Init register to activate BSM from VBACKUP (Direct Switching Mode)
\*-------------------------------------------------------------------------------------------*/
void RV3028_EnableDSM(void) {
  uint8_t current_eeprom;

  I2cWrite8(RtcChip.address, 0x25, 0x37, RtcChip.bus);  // EEADDR = 0x37
  I2cWrite8(RtcChip.address, 0x27, 0x22, RtcChip.bus);  // EECMD = 0x22 (EEPROM Read)
  delay(3);  

  current_eeprom = I2cRead8(RtcChip.address, 0x26, RtcChip.bus);  // EEDATA actual data

  if (current_eeprom != 0x14) {
    I2cWrite8(RtcChip.address, 0x25, 0x37, RtcChip.bus);  // EEADDR = 0x37
    I2cWrite8(RtcChip.address, 0x26, 0x14, RtcChip.bus);  // EEDATA = 0x14 (FEDE=1, BSM=01 DSM mode)
    I2cWrite8(RtcChip.address, 0x27, 0x21, RtcChip.bus);  // EECMD = 0x21 (EEPROM Write)
    delay(25);  
    AddLog(LOG_LEVEL_INFO, PSTR("RV3028: EEPROM 0x37 updated to DSM mode."));
  } else {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RV3028: EEPROM 0x37 already set to DSM mode."));
  }
}

/*-------------------------------------------------------------------------------------------*\
 * Read time from RV-3028-C7 and return the epoch time (seconds since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
uint32_t RV3028ReadTime(void) {

  uint8_t status = I2cRead8(RtcChip.address, RV3028_STATUS, RtcChip.bus);
  
  // Skontroluj PORF bit (bit 0 registra STATUS)
  if (status & _BV(RV3028_PORF)) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RV3028: PORF detected, RTC time invalid"));
    return 0;  // Invalid RTC time data
  }

  TIME_T tm;
  tm.second       = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_SECONDS, RtcChip.bus) & 0x7F);
  tm.minute       = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_MINUTES, RtcChip.bus) & 0x7F);
  tm.hour         = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_HOURS, RtcChip.bus) & 0x3F);       // 24h mode (12_24 bit = 0)
  tm.day_of_week  = I2cRead8(RtcChip.address, RV3028_WEEKDAY, RtcChip.bus) & 0x07;              // 0..6 (3-bit weekday counter)
  tm.day_of_month = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_DATE, RtcChip.bus) & 0x3F);
  tm.month        = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_MONTH, RtcChip.bus) & 0x1F);
  uint8_t year    = Bcd2Dec(I2cRead8(RtcChip.address, RV3028_YEAR, RtcChip.bus));
  // RV-3028-C7 holds year 00-99 (representing 2000-2099).
  // MakeTime requires tm.year as years since 1970.
  tm.year = year + 30;   // (e.g., 23 -> 53 for year 2023)
  return MakeTime(tm);
}

/*-------------------------------------------------------------------------------------------*\
 * Set RV-3028-C7 time using the given epoch time (seconds since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
void RV3028SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);
  I2cWrite8(RtcChip.address, RV3028_SECONDS, Dec2Bcd(tm.second), RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_MINUTES, Dec2Bcd(tm.minute), RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_HOURS,   Dec2Bcd(tm.hour),   RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_WEEKDAY, tm.day_of_week,     RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_DATE,    Dec2Bcd(tm.day_of_month), RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_MONTH,   Dec2Bcd(tm.month),  RtcChip.bus);
  // Convert years since 1970 to RTC register value (00..99)
  uint8_t true_year = (tm.year < 30) ? (tm.year + 70) : (tm.year - 30);
  I2cWrite8(RtcChip.address, RV3028_YEAR, Dec2Bcd(true_year), RtcChip.bus);
  // Clear the power-on reset flag (PORF) in the status register
  uint8_t status = I2cRead8(RtcChip.address, RV3028_STATUS, RtcChip.bus);
  I2cWrite8(RtcChip.address, RV3028_STATUS, status & ~_BV(RV3028_PORF), RtcChip.bus);

  // Enable LSM mode (VBACKUP)
  RV3028_EnableDSM();
}
#ifdef USE_RTC_CHIP_HOLDOVER
static const uint8_t kRv3028Aux[3] = { 0x1F, 0x20, 0x0A };       // User RAM 1 and 2, Timer Value 0

bool RV3028AuxWrite(uint32_t v) { return RtcAuxWriteRegs(kRv3028Aux, v); }
bool RV3028AuxRead(uint32_t* v) { return RtcAuxReadRegs(kRv3028Aux, v); }
#endif  // USE_RTC_CHIP_HOLDOVER

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void RV3028Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_94)) {
    RtcChip.address = RV3028_ADDR;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
      if (I2cValidRead(RtcChip.address, RV3028_STATUS, 1, RtcChip.bus)) {
        uint8_t status = I2cRead8(RtcChip.address, RV3028_STATUS, RtcChip.bus);
        if (status & _BV(RV3028_PORF)) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("RV3028: PORF detected at init, RTC time invalid"));
        }
        RtcChip.detected = 1;
        strcpy_P(RtcChip.name, PSTR("RV3028"));
        RtcChip.ReadTime = &RV3028ReadTime;
        RtcChip.SetTime  = &RV3028SetTime;
#ifdef USE_RTC_CHIP_HOLDOVER
        RtcChip.SetTimeHeld = &RV3028SetTime;
        RtcChip.ReleaseTime = &RtcChipRewriteSeconds;
        RtcChip.release_flip_us = 1000000;
        RtcChip.seconds_reg = RV3028_SECONDS;
        RtcChip.AuxWrite = &RV3028AuxWrite;
        RtcChip.AuxRead = &RV3028AuxRead;
        RtcChip.drift_reg = 0x07;                // Minutes alarm, alarm unused
#endif  // USE_RTC_CHIP_HOLDOVER
        RtcChip.mem_size = 2; // RAM 2 byte
        break;
      }
    }
  }
}
#endif  // USE_RV3028

/*********************************************************************************************\
 * DS1307 and DS3231
 *
 * I2C Address: 0x68
\*********************************************************************************************/
#ifdef USE_DS3231

#define XI2C_26             26      // See I2CDEVICES.md

#define DS3231_ADDRESS      0x68    // DS3231 I2C Address

// DS3231 Register Addresses
#define DS3231_SECONDS      0x00
#define DS3231_MINUTES      0x01
#define DS3231_HOURS        0x02
#define DS3231_DAY          0x03
#define DS3231_DATE         0x04
#define DS3231_MONTH        0x05
#define DS3231_YEAR         0x06
#define DS3231_CONTROL      0x0E
#define DS3231_STATUS       0x0F
#define DS3231_TEMP_MSB     0x11
#define DS3231_TEMP_LSB     0x12

// Control register bits
#define DS3231_OSF          7
#define DS3231_EOSC         7
#define DS3231_BBSQW        6
#define DS3231_CONV         5
#define DS3231_RS2          4
#define DS3231_RS1          3
#define DS3231_INTCN        2

//Other
#define DS3231_HR1224       6       // Hours register 12 or 24 hour mode (24 hour mode==0)
#define DS3231_CENTURY      7       // Century bit in Month register
#define DS3231_DYDT         6       // Day/Date flag bit in alarm Day/Date registers

/*-------------------------------------------------------------------------------------------*\
 * Read time from DS3231 and return the epoch time (second since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
uint32_t DS3231ReadTime(void) {
  TIME_T tm;
  tm.second = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_SECONDS, RtcChip.bus));
  tm.minute = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_MINUTES, RtcChip.bus));
  tm.hour = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_HOURS, RtcChip.bus) & ~_BV(DS3231_HR1224)); // 24h mode
  tm.day_of_week = I2cRead8(RtcChip.address, DS3231_DAY, RtcChip.bus);
  tm.day_of_month = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_DATE, RtcChip.bus));
  tm.month = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_MONTH, RtcChip.bus) & ~_BV(DS3231_CENTURY));  // Don't use the Century bit
  // MakeTime requires tm.year as number of years since 1970, 
  // However DS3231 is supposed to hold the true year but before this PR it was written tm.year directly
  // Assuming we read ... means ...
  //   00..21   = 1970..1990 written before PR (to support a RTC written with 1970) => don't apply correction
  //   22..51   = 2022..2051 written after PR => apply +30 years correction
  //   52..99   = 2022..2069 written before PR => don't apply correction
  uint8_t year = Bcd2Dec(I2cRead8(RtcChip.address, DS3231_YEAR, RtcChip.bus));
  tm.year = ((year <= 21) || (year >= 52)) ? year : (year + 30);
  return MakeTime(tm);
}
/*-------------------------------------------------------------------------------------------*\
 * Read temperature from DS3231 internal sensor, return as float
\*-------------------------------------------------------------------------------------------*/
#ifdef DS3231_ENABLE_TEMP
float DS3231ReadTemp(void) {
  int16_t temp_reg = I2cReadS16(RtcChip.address, DS3231_TEMP_MSB, RtcChip.bus) >> 6;
  float temp = temp_reg * 0.25;
  //AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("RTC: DS3231 temp_reg=%d"), temp_reg);
  return temp;
}
#endif // #ifdef DS3231_ENABLE_TEMP

/*-------------------------------------------------------------------------------------------*\
 * Show temperature from DS3231 internal sensor, Web or SENSOR
\*-------------------------------------------------------------------------------------------*/
#ifdef DS3231_ENABLE_TEMP
void D3231ShowSensor(bool json) {
    float f_temperature = ConvertTemp(DS3231ReadTemp());

    if (json) {
        ResponseAppend_P(PSTR(",\"DS3231\":{\"" D_JSON_TEMPERATURE "\":%*_f}"), Settings->flag2.temperature_resolution, &f_temperature);
#ifdef USE_DOMOTICZ
        if (0 == TasmotaGlobal.tele_period) {
          DomoticzFloatSensor(DZ_TEMP, f_temperature);
        }
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
        if (0 == TasmotaGlobal.tele_period) {
          KnxSensor(KNX_TEMPERATURE, f_temperature);
        }
#endif  // USE_KNX
    } 
#ifdef USE_WEBSERVER
    else {
        WSContentSend_Temp("DS3231", f_temperature);
    }
#endif // #ifdef USE_WEBSERVER
}
#endif // #ifdef DS3231_ENABLE_TEMP

/*-------------------------------------------------------------------------------------------*\
 * Get time as TIME_T and set the DS3231 time to this value
\*-------------------------------------------------------------------------------------------*/
void DS3231SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);
  I2cWrite8(RtcChip.address, DS3231_SECONDS, Dec2Bcd(tm.second), RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_MINUTES, Dec2Bcd(tm.minute), RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_HOURS, Dec2Bcd(tm.hour), RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_DAY, tm.day_of_week, RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_DATE, Dec2Bcd(tm.day_of_month), RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_MONTH, Dec2Bcd(tm.month), RtcChip.bus);
  uint8_t true_year = (tm.year < 30) ? (tm.year + 70) : (tm.year - 30);
  I2cWrite8(RtcChip.address, DS3231_YEAR, Dec2Bcd(true_year), RtcChip.bus);
  I2cWrite8(RtcChip.address, DS3231_STATUS, I2cRead8(RtcChip.address, DS3231_STATUS, RtcChip.bus) & ~_BV(DS3231_OSF), RtcChip.bus);
}
#ifdef USE_RTC_CHIP_HOLDOVER
bool DS3231AuxWrite(uint32_t v) { return RtcAuxWriteAt(RtcChip.address, 0x07, v); }    // Alarm 1
bool DS3231AuxRead(uint32_t* v) { return RtcAuxReadAt(RtcChip.address, 0x07, DS3231_SECONDS, v); }
#endif  // USE_RTC_CHIP_HOLDOVER

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void DS3231Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_26)) {
    RtcChip.address = DS3231_ADDRESS;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
      if (I2cValidRead(RtcChip.address, DS3231_STATUS, 1, RtcChip.bus)) {
        RtcChip.detected = 1;
        strcpy_P(RtcChip.name, PSTR("DS3231"));
        RtcChip.ReadTime = &DS3231ReadTime;
        RtcChip.SetTime = &DS3231SetTime;
#ifdef USE_RTC_CHIP_HOLDOVER
        RtcChip.SetTimeHeld = &DS3231SetTime;
        RtcChip.ReleaseTime = &RtcChipRewriteSeconds;
        RtcChip.release_flip_us = 1000000;
        RtcChip.seconds_reg = DS3231_SECONDS;
        RtcChip.AuxWrite = &DS3231AuxWrite;
        RtcChip.AuxRead = &DS3231AuxRead;
        RtcChip.drift_reg = 0x0B;                // Alarm 2 minutes
#endif  // USE_RTC_CHIP_HOLDOVER
#ifdef DS3231_ENABLE_TEMP
        RtcChip.ShowSensor = &D3231ShowSensor;
#endif
        RtcChip.mem_size = -1;
        break; 
      }
    }
  }
}
#endif  // USE_DS3231

/*********************************************************************************************\
 * PCF85063 support
 *
 * I2C Address: 0x51
\*********************************************************************************************/
#ifdef USE_PCF85063

#define XI2C_92             92       // Unique ID for I2C device search

#define PCF85063_ADDRESS    0x51     // PCF85063 I2C Address

#define PCF85063_REG_CTRL1      0x00
#define PCF85063_REG_CTRL2      0x01
#define PCF85063_REG_OFFSET     0x02
#define PCF85063_REG_SECONDS    0x04
#define PCF85063_REG_MINUTES    0x05
#define PCF85063_REG_HOURS      0x06
#define PCF85063_REG_DAYS       0x07
#define PCF85063_REG_WEEKDAYS   0x08
#define PCF85063_REG_MONTHS     0x09
#define PCF85063_REG_YEARS      0x0A

/*-------------------------------------------------------------------------------------------*\
 * Read time and return the epoch time (second since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
uint32_t Pcf85063ReadTime(void) {
  TwoWire& myWire = I2cGetWire(RtcChip.bus);
  myWire.beginTransmission(RtcChip.address);
  myWire.write(PCF85063_REG_SECONDS);
  myWire.endTransmission(false);   // false -> repeated start
  myWire.requestFrom((uint8_t)RtcChip.address, (uint8_t)7);

  uint8_t sec   = myWire.read(); // 0x04
  uint8_t min   = myWire.read(); // 0x05
  uint8_t hour  = myWire.read(); // 0x06
  uint8_t day   = myWire.read(); // 0x07
  uint8_t wday  = myWire.read(); // 0x08
  uint8_t month = myWire.read(); // 0x09
  uint8_t year  = myWire.read(); // 0x0A

  TIME_T tm;
  tm.second       = Bcd2Dec(sec  & 0x7F); 
  tm.minute       = Bcd2Dec(min  & 0x7F);
  tm.hour         = Bcd2Dec(hour & 0x3F);
  tm.day_of_month = Bcd2Dec(day  & 0x3F);
  tm.day_of_week  = wday & 0x07; 
  tm.month        = Bcd2Dec(month & 0x1F) -1; 
  uint8_t y = Bcd2Dec(year);
  tm.year = (y + 30);
  return MakeTime(tm);
}

/*-------------------------------------------------------------------------------------------*\
 * Get time as TIME_T and set time to this value
\*-------------------------------------------------------------------------------------------*/
void Pcf85063SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);

  uint8_t year = (tm.year -30); 
  if (year > 99) { year = 99; } 

  uint8_t bcd_sec   = Dec2Bcd(tm.second);
  uint8_t bcd_min   = Dec2Bcd(tm.minute);
  uint8_t bcd_hour  = Dec2Bcd(tm.hour);
  uint8_t bcd_day   = Dec2Bcd(tm.day_of_month);
  uint8_t bcd_wday  = tm.day_of_week & 0x07;
  uint8_t bcd_month = Dec2Bcd(tm.month +1);
  uint8_t bcd_year  = Dec2Bcd(year);

  TwoWire& myWire = I2cGetWire(RtcChip.bus);
  myWire.beginTransmission(RtcChip.address);
  myWire.write(PCF85063_REG_SECONDS);
  myWire.write(bcd_sec);
  myWire.write(bcd_min);
  myWire.write(bcd_hour);
  myWire.write(bcd_day);
  myWire.write(bcd_wday);
  myWire.write(bcd_month);
  myWire.write(bcd_year);
  myWire.endTransmission();
}
#ifdef USE_RTC_CHIP_HOLDOVER
#define PCF85063_CTRL1_STOP     0x20                // Bit 5, stops the clock
#define PCF85063_CTRL1_SR       0x10                // Bit 4, software reset
#define PCF85063_REG_ALARM_MIN  0x0C                // PCF85063A only

uint8_t pcf85063_ctrl1 = 0;                         // Control_1 at start (keeps CAP_SEL)

void Pcf85063SetTimeHeld(uint32_t epoch_time) {
  I2cWrite8(RtcChip.address, PCF85063_REG_CTRL1, pcf85063_ctrl1 | PCF85063_CTRL1_STOP, RtcChip.bus);
  Pcf85063SetTime(epoch_time);
}

void Pcf85063Release(void) {
  I2cWrite8(RtcChip.address, PCF85063_REG_CTRL1, pcf85063_ctrl1, RtcChip.bus);   // Start the clock
}

bool Pcf85063AuxWrite(uint32_t v) {
  return RtcAuxWriteAt(RtcChip.address, PCF85063_REG_ALARM_MIN, v);
}

bool Pcf85063AuxRead(uint32_t* v) {
  return RtcAuxReadAt(RtcChip.address, PCF85063_REG_ALARM_MIN, PCF85063_REG_SECONDS, v);
}

// PCF85063A has alarm registers, PCF85063TP has none above 0x0A (reads 0x80). Write a test value
// and read it back, the alarm stays disabled
bool Pcf85063HasAlarms(void) {
  uint8_t back = 0;
  if (!I2cWrite8(RtcChip.address, 0x0B, 0x80 | 0x55, RtcChip.bus)) { return false; }
  bool kept = I2cValidRead8(&back, RtcChip.address, 0x0B, RtcChip.bus) && ((0x80 | 0x55) == back);
  I2cWrite8(RtcChip.address, 0x0B, 0x80, RtcChip.bus);
  return kept;
}
#endif  // USE_RTC_CHIP_HOLDOVER

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void Pcf85063Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_92)) {
    RtcChip.address = PCF85063_ADDRESS;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      // Vyskúšame, či vieme prečítať nejaký register
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
      // Skúsime napr. prečítať PCF85063_REG_CTRL1
      if (I2cValidRead(RtcChip.address, PCF85063_REG_CTRL1, 1, RtcChip.bus)) { 
        RtcChip.detected = 1;
        strcpy_P(RtcChip.name, PSTR("PCF85063"));
        RtcChip.ReadTime = &Pcf85063ReadTime;
        RtcChip.SetTime  = &Pcf85063SetTime;
#ifdef USE_RTC_CHIP_HOLDOVER
        I2cValidRead8(&pcf85063_ctrl1, RtcChip.address, PCF85063_REG_CTRL1, RtcChip.bus);
        RtcChip.stopped = (pcf85063_ctrl1 & PCF85063_CTRL1_STOP);
        pcf85063_ctrl1 &= ~(PCF85063_CTRL1_STOP | PCF85063_CTRL1_SR);   // Keep CAP_SEL and 12_24
        RtcChip.release_flip_us = 500000;          // Steps 0.5 s after the start
        RtcChip.SetTimeHeld = &Pcf85063SetTimeHeld;
        RtcChip.ReleaseTime = &Pcf85063Release;
        RtcChip.seconds_reg = PCF85063_REG_SECONDS;
        if (Pcf85063HasAlarms()) {                   // PCF85063A
          RtcChip.AuxWrite = &Pcf85063AuxWrite;
          RtcChip.AuxRead = &Pcf85063AuxRead;
          RtcChip.drift_reg = 0x03;                // RAM_byte
        }
        AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: PCF85063%s, control 0x%02X"), RtcChip.AuxRead ? PSTR("A") : PSTR("TP"),
          pcf85063_ctrl1);
        Pcf85063Release();
#endif  // USE_RTC_CHIP_HOLDOVER
        RtcChip.mem_size = -1;    // Nemá extra user RAM, ak by si nepotreboval
        // Ak by si chcel implementovať MemRead/MemWrite, doplň RtcChip.MemRead a RtcChip.MemWrite
        break;
      }
    }
  }
}
#endif // USE_PCF85063

/*********************************************************************************************\
 * BM8563 - Real Time Clock
 *
 * I2C Address: 0x51 (Fixed in library as BM8563_ADRESS)
\*********************************************************************************************/
#ifdef USE_BM8563

#define XI2C_59             59       // See I2CDEVICES.md

#include "BM8563.h"
BM8563 Bm8563Rtc;

uint32_t BM8563GetUtc(void) {
  RTC_TimeTypeDef RTCtime;
  // 1. read has errors ???
  Bm8563Rtc.GetTime(&RTCtime);
//   core2_globs.Rtc.GetTime(&RTCtime);
  RTC_DateTypeDef RTCdate;
  Bm8563Rtc.GetDate(&RTCdate);
  TIME_T tm;
  tm.second =  RTCtime.Seconds;
  tm.minute = RTCtime.Minutes;
  tm.hour = RTCtime.Hours;
  tm.day_of_week = RTCdate.WeekDay;
  tm.day_of_month = RTCdate.Date;
  tm.month = RTCdate.Month;
  tm.year = RTCdate.Year - 1970;
  return MakeTime(tm);
}

void BM8563SetUtc(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);
  RTC_TimeTypeDef RTCtime;
  RTCtime.Hours = tm.hour;
  RTCtime.Minutes = tm.minute;
  RTCtime.Seconds = tm.second;
  Bm8563Rtc.SetTime(&RTCtime);
  RTC_DateTypeDef RTCdate;
  RTCdate.WeekDay = tm.day_of_week;
  RTCdate.Month = tm.month;
  RTCdate.Date = tm.day_of_month;
  RTCdate.Year = tm.year + 1970;
  Bm8563Rtc.SetDate(&RTCdate);
}
#ifdef USE_RTC_CHIP_HOLDOVER

#define BM8563_REG_CTRL1   0x00
#define BM8563_CTRL1_STOP  0x20                                   // bit 5

void BM8563SetUtcHeld(uint32_t epoch_time) {
  I2cWrite8(BM8563_ADRESS, BM8563_REG_CTRL1, BM8563_CTRL1_STOP, RtcChip.bus);   // Stop the clock
  BM8563SetUtc(epoch_time);
}

void BM8563Release(void) {
  I2cWrite8(BM8563_ADRESS, BM8563_REG_CTRL1, 0x00, RtcChip.bus);  // Start the clock
}

#define BM8563_REG_SECONDS 0x02                                   // Bit 7 is VL

#define BM8563_REG_ALARM_MIN  0x09

bool BM8563AuxWrite(uint32_t v) {
  return RtcAuxWriteAt(BM8563_ADRESS, BM8563_REG_ALARM_MIN, v);
}

bool BM8563AuxRead(uint32_t* v) {
  return RtcAuxReadAt(BM8563_ADRESS, BM8563_REG_ALARM_MIN, BM8563_REG_SECONDS, v);
}
#endif  // USE_RTC_CHIP_HOLDOVER

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void BM8563Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_59)) {
    RtcChip.address = BM8563_ADRESS;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
#ifdef USE_RTC_CHIP_HOLDOVER
      uint8_t ctrl1 = 0;
      RtcChip.stopped = I2cValidRead8(&ctrl1, BM8563_ADRESS, BM8563_REG_CTRL1, RtcChip.bus) &&
                        (ctrl1 & BM8563_CTRL1_STOP);              // Read before begin() clears it
#endif  // USE_RTC_CHIP_HOLDOVER
      Bm8563Rtc.begin(&I2cGetWire(RtcChip.bus));
      RtcChip.detected = 1;
      strcpy_P(RtcChip.name, PSTR("BM8563"));
      RtcChip.ReadTime = &BM8563GetUtc;
      RtcChip.SetTime = &BM8563SetUtc;
#ifdef USE_RTC_CHIP_HOLDOVER
      RtcChip.release_flip_us = 1000000;
      RtcChip.SetTimeHeld = &BM8563SetUtcHeld;
      RtcChip.ReleaseTime = &BM8563Release;
      RtcChip.seconds_reg = BM8563_REG_SECONDS;
      RtcChip.AuxWrite = &BM8563AuxWrite;
      RtcChip.AuxRead = &BM8563AuxRead;
      RtcChip.drift_reg = 0x0F;                  // Timer value, used while the timer is off
      BM8563Release();
#endif  // USE_RTC_CHIP_HOLDOVER
      RtcChip.mem_size = -1;
      break;
    }
  }
}
#endif  // USE_BM8563

/*********************************************************************************************\
 * PCF85363 support
 *
 * I2C Address: 0x51
\*********************************************************************************************/
#ifdef USE_PCF85363

#define XI2C_66             66      // See I2CDEVICES.md

#define PCF85363_ADDRESS    0x51    // PCF85363 I2C Address

/*-------------------------------------------------------------------------------------------*\
 * Read time and return the epoch time (second since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
uint32_t Pcf85363ReadTime(void) {
  TwoWire& myWire = I2cGetWire(RtcChip.bus);
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x00);
  myWire.endTransmission();

  uint8_t buffer[8];
  myWire.requestFrom(RtcChip.address, (uint8_t)8);
  for (uint32_t i = 0; i < 8; i++) { buffer[i] = myWire.read(); }
  myWire.endTransmission();

  TIME_T tm;
  tm.second = Bcd2Dec(buffer[1] & 0x7F);
  tm.minute = Bcd2Dec(buffer[2] & 0x7F);
  tm.hour = Bcd2Dec(buffer[3]);
  tm.day_of_month = Bcd2Dec(buffer[4]);
  tm.day_of_week = buffer[5];
  tm.month = Bcd2Dec(buffer[6]);
  tm.year = 30 + Bcd2Dec(buffer[7]);  // Offset from 1970. So 2022 - 1970 = 52
  return MakeTime(tm);
}

/*-------------------------------------------------------------------------------------------*\
 * Get time as TIME_T and set time to this value
\*-------------------------------------------------------------------------------------------*/
void Pcf85363SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);

  uint8_t buffer[8];
  buffer[0] = 0x00;                  // 100th_seconds (not used)
  buffer[1] = Dec2Bcd(tm.second);
  buffer[2] = Dec2Bcd(tm.minute);
  buffer[3] = Dec2Bcd(tm.hour);
  buffer[4] = Dec2Bcd(tm.day_of_month);
  buffer[5] = tm.day_of_week;
  buffer[6] = Dec2Bcd(tm.month);
  buffer[7] = Dec2Bcd(tm.year -30);  // Offset from 1970

  TwoWire& myWire = I2cGetWire(RtcChip.bus);
/*
  // Handbook page 13
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x2E);
  myWire.write(0x01);                  // Set stop
  myWire.write(0xA4);                  // Clear prescaler
  for (uint32_t i = 0; i < 8; i++) { myWire.write(buffer[i]); }
  myWire.endTransmission();
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x2E);
  myWire.write(0x00);                  // Set start
  myWire.endTransmission();
*/
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x00);
  for (uint32_t i = 0; i < 8; i++) { myWire.write(buffer[i]); }
  myWire.endTransmission();
}
#ifdef USE_RTC_CHIP_HOLDOVER
void Pcf85363SetTimeHeld(uint32_t epoch_time) {
  I2cWrite8(RtcChip.address, 0x2E, 0x01, RtcChip.bus);             // Stop the clock
  I2cWrite8(RtcChip.address, 0x2F, 0xA4, RtcChip.bus);             // Clear the prescaler
  Pcf85363SetTime(epoch_time);
}

void Pcf85363Release(void) {
  I2cWrite8(RtcChip.address, 0x2E, 0x00, RtcChip.bus);             // Start the clock
}

static const uint8_t kPcf85363Aux[3] = { 0x7D, 0x7E, 0x7F };       // Last three RAM bytes

bool Pcf85363AuxWrite(uint32_t v) { return RtcAuxWriteRegs(kPcf85363Aux, v); }
bool Pcf85363AuxRead(uint32_t* v) { return RtcAuxReadRegs(kPcf85363Aux, v); }
#endif  // USE_RTC_CHIP_HOLDOVER

/*-------------------------------------------------------------------------------------------*\
 * Dump all registers
\*-------------------------------------------------------------------------------------------*/
/*
void Pcf85363Dump(void) {
  uint8_t buffer[64];

  TwoWire& myWire = I2cGetWire(RtcChip.bus);
  // 0x00 to 0x2F
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x00);
  myWire.endTransmission();
  myWire.requestFrom(RtcChip.address, (uint8_t)48);
  for (uint32_t i = 0; i < 48; i++) {
    buffer[i] = myWire.read();
  }
  myWire.endTransmission();
  AddLog(LOG_LEVEL_DEBUG, PSTR("P85: Read 0x00: %48_H"), buffer);

  // 0x40 to 0x7F
  myWire.beginTransmission(RtcChip.address);
  myWire.write(0x40);
  myWire.endTransmission();
  myWire.requestFrom(RtcChip.address, (uint8_t)64);
  for (uint32_t i = 0; i < 64; i++) {
    buffer[i] = myWire.read();
  }
  myWire.endTransmission();
  AddLog(LOG_LEVEL_DEBUG, PSTR("P85: Read 0x40: %64_H"), buffer);
}
*/

/*-------------------------------------------------------------------------------------------*\
 * Memory block functions
\*-------------------------------------------------------------------------------------------*/
int32_t Pcf8563MemRead(uint8_t *buffer, uint32_t size) {
  return I2cReadBuffer(RtcChip.address, 0x40, buffer, size, RtcChip.bus);
}

int32_t Pcf8563MemWrite(uint8_t *buffer, uint32_t size) {
  return I2cWriteBuffer(RtcChip.address, 0x40, (uint8_t *)buffer, size, RtcChip.bus);
}

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void Pcf85363Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_66)) {
    RtcChip.address = PCF85363_ADDRESS;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
      RtcChip.detected = 1;
      strcpy_P(RtcChip.name, PSTR("PCF85363"));
      RtcChip.ReadTime = &Pcf85363ReadTime;
      RtcChip.SetTime = &Pcf85363SetTime;
      RtcChip.mem_size = 64;
#ifdef USE_RTC_CHIP_HOLDOVER
      RtcChip.SetTimeHeld = &Pcf85363SetTimeHeld;
      RtcChip.ReleaseTime = &Pcf85363Release;
      RtcChip.release_flip_us = 500000;          // As the PCF85063, corrected by the first placement
      RtcChip.seconds_reg = 0x01;
      RtcChip.AuxWrite = &Pcf85363AuxWrite;
      RtcChip.AuxRead = &Pcf85363AuxRead;
      RtcChip.drift_reg = 0x2C;                  // RAM_byte
      RtcChip.mem_size = 61;                     // The last three RAM bytes hold the record
#endif  // USE_RTC_CHIP_HOLDOVER
      RtcChip.MemRead = &Pcf8563MemRead;
      RtcChip.MemWrite = &Pcf8563MemWrite;
      break;
    }
  }
}
#endif // USE_PCF85363

/*********************************************************************************************\
 * RX8010 and RX8030 - Real Time Clock
 * based on linux/rtc-rx8010.c
 *
 * I2C Address: 0x32
\*********************************************************************************************/
#if defined(USE_RX8010) || defined(USE_RX8030)

#define XI2C_90             90       // See I2CDEVICES.md

#define RX8010_ADDRESS      0x32     // RX8010 I2C Address

// RX8010 Register Addresses
#define RX8010_REG_SEC		  0x10
#define RX8010_REG_MIN		  0x11
#define RX8010_REG_HOUR		  0x12
#define RX8010_REG_WDAY		  0x13
#define RX8010_REG_MDAY		  0x14
#define RX8010_REG_MONTH	  0x15
#define RX8010_REG_YEAR		  0x16
#ifdef USE_RX8030
#define RX80x0_REG_CTRL     0x1E
#endif
#ifdef USE_RX8010
#undef RX80x0_REG_CTRL
#define RX80x0_REG_CTRL		  0x1F
#endif

// Control Register (1Fh) bit positions
#define RX8010_BIT_CTRL_STOP	6

/*-------------------------------------------------------------------------------------------*\
 * Read time from RX8010 and return the epoch time (second since 1-1-1970 00:00)
\*-------------------------------------------------------------------------------------------*/
uint32_t Rx8010ReadTime(void) {
  TIME_T tm;

  uint8_t data[7];
  I2cReadBuffer(RtcChip.address, RX8010_REG_SEC, data, 7, RtcChip.bus);
  tm.second = Bcd2Dec(data[0] & 0x7F);
  tm.minute = Bcd2Dec(data[1] & 0x7F);
  tm.hour = Bcd2Dec(data[2] & 0x3F);    // Assumes 24hr clock
  tm.day_of_month = Bcd2Dec(data[3] & 0x3F);
  tm.month = Bcd2Dec(data[4] & 0x3F) -1;
  tm.year = Bcd2Dec(data[5]);
	if (tm.year < 70) { tm.year += 100; }
  tm.day_of_week = Bcd2Dec(data[6] & 0x7F);
  return MakeTime(tm);
}

/*-------------------------------------------------------------------------------------------*\
 * Get time as TIME_T and set the RX8010 time to this value
\*-------------------------------------------------------------------------------------------*/
void Rx8010SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);
	// Set STOP bit before changing clock/calendar
  I2cWrite8(RtcChip.address, RX80x0_REG_CTRL, I2cRead8(RtcChip.address, RX80x0_REG_CTRL, RtcChip.bus) | _BV(RX8010_BIT_CTRL_STOP), RtcChip.bus);
  uint8_t data[7];
  data[0] = Dec2Bcd(tm.second);
  data[1] = Dec2Bcd(tm.minute);
  data[2] = Dec2Bcd(tm.hour);
  data[3] = Dec2Bcd(tm.day_of_month);
  data[4] = Dec2Bcd(tm.month +1);
  data[5] = Dec2Bcd(tm.year % 100);
  data[6] = Dec2Bcd(tm.day_of_week);
  I2cWriteBuffer(RtcChip.address, RX8010_REG_SEC, data, 7, RtcChip.bus);
	// Clear STOP bit after changing clock/calendar
  I2cWrite8(RtcChip.address, RX80x0_REG_CTRL, I2cRead8(RtcChip.address, RX80x0_REG_CTRL, RtcChip.bus) & ~_BV(RX8010_BIT_CTRL_STOP), RtcChip.bus);
}

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void Rx8010Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_90)) {
    RtcChip.address = RX8010_ADDRESS;
    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {
      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) { continue; }
      if (I2cValidRead(RtcChip.address, RX80x0_REG_CTRL, 1, RtcChip.bus)) {
        RtcChip.detected = 1;
#ifdef USE_RX8030
        strcpy_P(RtcChip.name, PSTR("RX8030"));
#else
        strcpy_P(RtcChip.name, PSTR("RX8010"));
#endif
        RtcChip.ReadTime = &Rx8010ReadTime;
        RtcChip.SetTime = &Rx8010SetTime;
        RtcChip.mem_size = -1;
        break;
      }
    }
  }
}
#endif  // USE_RX8010
/*********************************************************************************************\
 * RX8025T - Real Time Clock
 *
 * I2C Address: 0x32
\*********************************************************************************************/
#if defined(USE_RX8025)

#define XI2C_96             96       // See I2CDEVICES.md
#define RX8025_ADDRESS      0x32

// RX8025T Register Addresses (per datasheet table 0..F)
#define RX8025_REG_SEC      0x00
#define RX8025_REG_MIN      0x01
#define RX8025_REG_HOUR     0x02
#define RX8025_REG_WEEK     0x03     // bitfield 6..0 (Sun..Sat)
#define RX8025_REG_MDAY     0x04
#define RX8025_REG_MONTH    0x05     // 01..12
#define RX8025_REG_YEAR     0x06     // 00..99 (2000..2099)

#define RX8025_REG_EXT      0x0D
#define RX8025_REG_FLAG     0x0E
#define RX8025_REG_CTRL     0x0F

// FLAG bits (per table: ... VLF VDET)
#define RX8025_FLAG_VLF     1
#define RX8025_FLAG_VDET    0

// CTRL RESET bit0 = stop status (datasheet)
#define RX8025_BIT_CTRL_RESET  0

// WEEK bitfield <-> Tasmota day_of_week (1..7, Sunday=1)
static uint8_t Rx8025WeekToWday(uint8_t week) {
  week &= 0x7F;                       // ignore bit7
  for (uint8_t i = 0; i < 7; i++) {
    if (week & (1U << i)) { return (uint8_t)(i + 1); }  // Sun=1
  }
  return 1;
}

static uint8_t Rx8025WdayToWeek(uint8_t wday) {
  if (wday < 1 || wday > 7) { wday = 1; }
  return (uint8_t)(1U << (wday - 1)); // one-hot bits 0..6
}

static void Rx8025LogRaw(const char *tag, uint8_t bus, uint8_t addr,
                         const uint8_t data[7], uint8_t ctrl, uint8_t flag, uint8_t ext) {
  AddLog(LOG_LEVEL_DEBUG,
         PSTR("RTC: RX8025T %s bus=%d addr=0x%02X RAW[00..06]=%02X %02X %02X %02X %02X %02X %02X  CTRL=%02X FLAG=%02X EXT=%02X"),
         tag, bus, addr,
         data[0], data[1], data[2], data[3], data[4], data[5], data[6],
         ctrl, flag, ext);
}

static void Rx8025PreInitIfNeeded(void) {
  uint8_t ctrl = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  uint8_t flag = I2cRead8(RtcChip.address, RX8025_REG_FLAG, RtcChip.bus);
  uint8_t ext  = I2cRead8(RtcChip.address, RX8025_REG_EXT,  RtcChip.bus);

  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T PRE-INIT CTRL=%02X FLAG=%02X EXT=%02X"), ctrl, flag, ext);

  // If VLF or VDET is set, datasheet says initialize registers before use.
  if (flag & (_BV(RX8025_FLAG_VLF) | _BV(RX8025_FLAG_VDET))) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T VLF/VDET set -> clearing EXT/FLAG (arduino-style)"));

    // Keep CTRL upper bits, force reserved bits 2..1 = 0
    uint8_t ctrl_base = ctrl & 0xF8;

    // Enter stop status (RESET=1)
    I2cWrite8(RtcChip.address, RX8025_REG_CTRL, ctrl_base | _BV(RX8025_BIT_CTRL_RESET), RtcChip.bus);

    // Clear EXT and FLAG (common minimal init used by reference libs)
    I2cWrite8(RtcChip.address, RX8025_REG_EXT,  0x00, RtcChip.bus);
    I2cWrite8(RtcChip.address, RX8025_REG_FLAG, 0x00, RtcChip.bus);

    // Exit stop status (RESET=0)
    I2cWrite8(RtcChip.address, RX8025_REG_CTRL, ctrl_base, RtcChip.bus);

    ctrl = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
    flag = I2cRead8(RtcChip.address, RX8025_REG_FLAG, RtcChip.bus);
    ext  = I2cRead8(RtcChip.address, RX8025_REG_EXT,  RtcChip.bus);
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T POST-INIT CTRL=%02X FLAG=%02X EXT=%02X"), ctrl, flag, ext);
  }
}

/*-------------------------------------------------------------------------------------------*\
 * Read time from RX8025T and return epoch time
\*-------------------------------------------------------------------------------------------*/
uint32_t Rx8025ReadTime(void) {
  TIME_T tm;

  uint8_t data[7];
  I2cReadBuffer(RtcChip.address, RX8025_REG_SEC, data, 7, RtcChip.bus);

  uint8_t ctrl = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  uint8_t flag = I2cRead8(RtcChip.address, RX8025_REG_FLAG, RtcChip.bus);
  uint8_t ext  = I2cRead8(RtcChip.address, RX8025_REG_EXT,  RtcChip.bus);

  Rx8025LogRaw("READ", RtcChip.bus, RtcChip.address, data, ctrl, flag, ext);

  tm.second = Bcd2Dec(data[0] & 0x7F);
  tm.minute = Bcd2Dec(data[1] & 0x7F);
  tm.hour   = Bcd2Dec(data[2] & 0x3F);

  tm.day_of_week  = Rx8025WeekToWday(data[3]);
  tm.day_of_month = Bcd2Dec(data[4] & 0x3F);

  // RX8025: MONTH is 01..12 (no -1)
  tm.month = Bcd2Dec(data[5] & 0x1F);

  // RX8025: YEAR is 00..99 for 2000..2099  => yOff = (2000-1970)+y2k = 30+y2k
  uint8_t y2k = Bcd2Dec(data[6]);
  tm.year = (uint8_t)(30 + y2k);

  uint32_t epoch = MakeTime(tm);

  AddLog(LOG_LEVEL_DEBUG,
         PSTR("RTC: RX8025T DECODE y2k=%u -> yOff=%u (abs=%u) m=%u d=%u w=%u %02u:%02u:%02u -> epoch=%u"),
         y2k, tm.year, (uint16_t)(1970 + tm.year),
         tm.month, tm.day_of_month, tm.day_of_week,
         tm.hour, tm.minute, tm.second,
         epoch);

  return epoch;
}

/*-------------------------------------------------------------------------------------------*\
 * Set RX8025T time from epoch
\*-------------------------------------------------------------------------------------------*/
void Rx8025SetTime(uint32_t epoch_time) {
  TIME_T tm;
  BreakTime(epoch_time, tm);

  uint16_t abs_year = (uint16_t)(1970 + tm.year);
  uint8_t y2k = 0;
  if (abs_year < 2000) { y2k = 0; }
  else if (abs_year > 2099) { y2k = 99; }
  else { y2k = (uint8_t)(abs_year - 2000); }

  AddLog(LOG_LEVEL_DEBUG,
         PSTR("RTC: RX8025T SET epoch=%u -> yOff=%u (abs=%u) m=%u d=%u w=%u %02u:%02u:%02u"),
         epoch_time, tm.year, abs_year,
         tm.month, tm.day_of_month, tm.day_of_week,
         tm.hour, tm.minute, tm.second);

  // CTRL: clear reserved bits 2..1, preserve others
  uint8_t ctrl0 = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  uint8_t ctrl_base = ctrl0 & 0xF8;

  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T CTRL before=%02X masked=%02X (set RESET bit0=stop)"), ctrl0, ctrl_base);

  // Enter stop status (RESET=1)
  I2cWrite8(RtcChip.address, RX8025_REG_CTRL, ctrl_base | _BV(RX8025_BIT_CTRL_RESET), RtcChip.bus);

  uint8_t ctrl1 = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T CTRL after STOP(read)=%02X"), ctrl1);

  uint8_t data[7];
  data[0] = Dec2Bcd(tm.second);
  data[1] = Dec2Bcd(tm.minute);
  data[2] = Dec2Bcd(tm.hour);
  data[3] = Rx8025WdayToWeek(tm.day_of_week);
  data[4] = Dec2Bcd(tm.day_of_month);

  // RX8025: MONTH is 01..12 (no +1)
  data[5] = Dec2Bcd(tm.month);

  data[6] = Dec2Bcd(y2k);

  AddLog(LOG_LEVEL_DEBUG,
         PSTR("RTC: RX8025T WRITE y2k=%u RAW=%02X %02X %02X %02X %02X %02X %02X"),
         y2k, data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

  I2cWriteBuffer(RtcChip.address, RX8025_REG_SEC, data, 7, RtcChip.bus);

  // Optional readback for verification
  uint8_t rb[7];
  I2cReadBuffer(RtcChip.address, RX8025_REG_SEC, rb, 7, RtcChip.bus);
  uint8_t ctrl_rb = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  uint8_t flag_rb = I2cRead8(RtcChip.address, RX8025_REG_FLAG, RtcChip.bus);
  uint8_t ext_rb  = I2cRead8(RtcChip.address, RX8025_REG_EXT,  RtcChip.bus);
  Rx8025LogRaw("READBACK", RtcChip.bus, RtcChip.address, rb, ctrl_rb, flag_rb, ext_rb);

  // Exit stop status (RESET=0)
  I2cWrite8(RtcChip.address, RX8025_REG_CTRL, ctrl_base, RtcChip.bus);
  uint8_t ctrl2 = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T CTRL after CLEAR(read)=%02X"), ctrl2);

  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s re-synced (" D_UTC_TIME ") %s"), RtcChip.name, GetDateAndTime(DT_UTC).c_str());
}

/*-------------------------------------------------------------------------------------------*\
 * Detection
\*-------------------------------------------------------------------------------------------*/
void Rx8025Detected(void) {
  if (!RtcChip.detected && I2cEnabled(XI2C_96)) {
    RtcChip.address = RX8025_ADDRESS;

    for (RtcChip.bus = 0; RtcChip.bus < MAX_I2C; RtcChip.bus++) {

      AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T DETECT try bus=%d addr=0x%02X"), RtcChip.bus, RtcChip.address);

      if (!I2cSetDevice(RtcChip.address, RtcChip.bus)) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T DETECT bus=%d -> I2cSetDevice FAIL"), RtcChip.bus);
        continue;
      }

      // Basic presence check: CTRL must be readable
      if (!I2cValidRead(RtcChip.address, RX8025_REG_CTRL, 1, RtcChip.bus)) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: RX8025T DETECT bus=%d -> I2cValidRead(CTRL) FAIL"), RtcChip.bus);
        continue;
      }

      // If VLF/VDET set, clear them before using registers (datasheet requirement)
      Rx8025PreInitIfNeeded();

      // Debug snapshot of time regs at detect
      uint8_t data[7];
      I2cReadBuffer(RtcChip.address, RX8025_REG_SEC, data, 7, RtcChip.bus);
      uint8_t ctrl = I2cRead8(RtcChip.address, RX8025_REG_CTRL, RtcChip.bus);
      uint8_t flag = I2cRead8(RtcChip.address, RX8025_REG_FLAG, RtcChip.bus);
      uint8_t ext  = I2cRead8(RtcChip.address, RX8025_REG_EXT,  RtcChip.bus);
      Rx8025LogRaw("DETECTED", RtcChip.bus, RtcChip.address, data, ctrl, flag, ext);

      RtcChip.detected = 1;
      strcpy_P(RtcChip.name, PSTR("RX8025T"));
      RtcChip.ReadTime = &Rx8025ReadTime;
      RtcChip.SetTime  = &Rx8025SetTime;
      RtcChip.mem_size = -1;
      break;
    }
  }
}
#endif  // USE_RX8025

/*********************************************************************************************\
 * RTC Detect and time set
\*********************************************************************************************/

void RtcChipDetect(void) {
  RtcChip.detected = 0;
  RtcChip.bus = 0;

#ifdef USE_RV3028
  RV3028Detected();
#endif  // USE_RV3028
#ifdef USE_DS3231
  DS3231Detected();
#endif  // USE_DS3231
#ifdef USE_BM8563
  BM8563Detected();
#endif  // USE_BM8563
#ifdef USE_PCF85363
  Pcf85363Detected();
#endif // USE_PCF85363
#if defined(USE_RX8010) || defined(USE_RX8030)
  Rx8010Detected();
#endif  // USE_RX8010
#ifdef USE_PCF85063
  Pcf85063Detected();
#endif  // USE_PCF85063
#ifdef USE_RX8025
  Rx8025Detected();
#endif  // USE_RX8025

  if (!RtcChip.detected) { return; }

  I2cSetActiveFound(RtcChip.address, RtcChip.name, RtcChip.bus);

  if (Rtc.utc_time < START_VALID_TIME) {                          // Not sync with NTP/GPS (time not valid), so read time
    uint32_t time = RtcChip.ReadTime();                           // Read UTC TIME
#ifdef USE_RTC_CHIP_HOLDOVER
    if (RtcChip.stopped) {                                        // Power lost between stop and start
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s found stopped, time lost"), RtcChip.name);
      time = 0;
    }
#endif  // USE_RTC_CHIP_HOLDOVER
    if (time > START_VALID_TIME) {
#ifdef USE_NTP_SERVER
#ifdef USE_RTC_CHIP_HOLDOVER
      RtcAuxLoad(time);
      RtcChipDriftLoad();
      int32_t lag = RtcChipLagUs(time);
      if (lag >= 1000000) { time += lag / 1000000; }
#endif  // USE_RTC_CHIP_HOLDOVER
      RtcSync(RtcChip.name, time, RTC_STRATUM_CHIP, RTC_PHASE_COLD, RTC_PHASE_SYNC_SECS);
#else
      Rtc.utc_time = time;
      RtcSync(RtcChip.name);
#endif  // USE_NTP_SERVER
    }
  }
}
#ifdef USE_NTP_SERVER
/*********************************************************************************************\
 * RTC chip phase: read it, and set it from our clock
\*********************************************************************************************/

bool RtcChipPresent(void) {
  return RtcChip.detected;
}

uint8_t RtcChipStratum(void) {
  uint8_t writer = RtcSettings.chip_writer_stratum;
  uint8_t earned = (writer > 0 && writer < RTC_STRATUM_NONE) ? (writer + 1) : RTC_STRATUM_CHIP;
  return (earned > RTC_STRATUM_CHIP) ? earned : RTC_STRATUM_CHIP;
}

void RtcChipSyncPhaseStart(void) {
  RtcChip.phase_time = RtcChip.ReadTime();
  RtcChip.phase_seen = micros();
  RtcChip.phase_polls = 0;
}

// Every 100 ms: watch the seconds register for its step when the clock needs the chip phase.
// Then use it, or start the fine catch
void RtcChipSyncPhasePoll(void) {
  if (!RtcChip.ReadTime) { return; }
#ifdef USE_RTC_CHIP_HOLDOVER
  if (RtcChip.set_pending || RtcChip.release_pending) {
    RtcChip.phase_time = 0;
    return;
  }
#endif  // USE_RTC_CHIP_HOLDOVER
  if (!RtcChip.phase_time) {                                      // Idle, check needed?
    bool unheld = (RTC_STRATUM_NONE == Rtc.stratum);
    bool cold = (RTC_PHASE_COLD == Rtc.phase_source);
#ifdef USE_RTC_CHIP_HOLDOVER
    bool unplaced = !RtcChipPlacedKnown() && ((TasmotaGlobal.uptime - RtcChip.phase_audit) >= 60) &&
                    RtcPhasePlaceable();
    bool audit = (Rtc.stratum < RTC_STRATUM_CHIP) &&
                 ((0 == RtcChip.phase_audit) || unplaced ||
                  ((TasmotaGlobal.uptime - RtcChip.phase_audit) >= RTC_PHASE_SYNC_SECS));
    bool place = RtcChipPlaceDue();
#else
    bool audit = false;
    bool place = false;
#endif  // USE_RTC_CHIP_HOLDOVER
    bool stale = ((TasmotaGlobal.uptime - Rtc.last_sync) >= RTC_PHASE_SYNC_SECS) &&
                 ((TasmotaGlobal.uptime - RtcChip.phase_audit) >= RTC_PHASE_SYNC_SECS);
    bool want = (unheld || cold || audit || place || stale);
    bool allowed = (audit || place || RtcBoundaryAllowed(RTC_STRATUM_CHIP, RTC_PHASE_SYNC_SECS));
    if ((Rtc.utc_time > START_VALID_TIME) && want && allowed) {
      if (audit || stale) { RtcChip.phase_audit = TasmotaGlobal.uptime; }
      RtcChip.phase_tries = 0;
      RtcChipSyncPhaseStart();
    }
    return;
  }

  uint32_t before = micros();
  uint32_t time = RtcChip.ReadTime();
  uint32_t after = micros();
  uint32_t after_ms = millis();
  if (time == RtcChip.phase_time) {
    if (RtcChip.phase_polls++ > 15) {                             // Over a second
      RtcChip.phase_time = 0;                                     // Chip stopped
      AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s not ticking"), RtcChip.name);
    }
    RtcChip.phase_seen = after;                                   // Same second
    return;
  }

  RtcChip.phase_time = 0;
  if (time < START_VALID_TIME) { return; }

  uint32_t span = before - RtcChip.phase_seen;
  uint32_t elapsed = (after - before) + (span / 2);
  if (elapsed > RTC_PHASE_MAX_WINDOW) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s phase window %u us, too wide (tries=%d before_re-arm)"),
      RtcChip.name, elapsed, RtcChip.phase_tries);
    if (RtcChip.phase_tries++ < 5) {
      RtcChipSyncPhaseStart();
    }
    return;
  }
#ifdef USE_RTC_CHIP_HOLDOVER
  bool fine = (0 != RtcChip.seconds_reg);
  if (fine) { RtcFineStart(false); }
  if (!fine && (Rtc.stratum < RTC_STRATUM_CHIP) && (RtcPhaseGood() || RtcPhaseNetwork())) {
    int32_t chip_ms = (int32_t)(elapsed / 1000);                  // ms since the chip second started
    int32_t ours_ms = (int32_t)RtcMillis();                       // ms since our second started
    int32_t skew = ours_ms - chip_ms;
    if (skew >  500) { skew -= 1000; }
    if (skew < -500) { skew += 1000; }
    int32_t limit = RtcChip.phase_limit ? RtcChip.phase_limit : RTC_CHIP_PHASE_MAX_MS;
    bool reseat = (abs(skew) > limit);
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s phase skew %d ms (ours %d, chip %d, limit %d)%s"),
      RtcChip.name, skew, ours_ms, chip_ms, limit, reseat ? PSTR(", re-seating") : PSTR(""));
    if (reseat) {
      RtcChipArmWrite();
      if (!RtcPhaseGood()) { RtcChip.net_reseat = true; }
    }
    RtcChip.phase_limit = 0;      // Default for the next check
  }
#endif  // USE_RTC_CHIP_HOLDOVER
  if ((Rtc.stratum >= RTC_STRATUM_CHIP) && RtcChipCrystalCarries()) {
    RtcChip.phase_tries = 0;
    return;
  }
  uint32_t b_ms = after_ms - (elapsed / 1000), b_us = after - elapsed;
#ifdef USE_RTC_CHIP_HOLDOVER
  if (!RtcChipFlipSecond(b_us, &time, &b_ms, &b_us)) {             // The caught step beats this read
    RtcChipCorrect(&time, &b_us, &b_ms);                         // Correct by the chip rate
  }
#endif  // USE_RTC_CHIP_HOLDOVER
#ifdef USE_RTC_CHIP_HOLDOVER
  if (!RtcChip.stopped &&
      RtcSetBoundary(time, b_ms, b_us, RTC_STRATUM_CHIP, RTC_PHASE_SYNC_SECS, RTC_PHASE_REGISTER, false)) {
    RTC_PAIR_LOCK();
    Rtc.held_us = RtcChipHoldover(time, elapsed);       // Error = read gap
    RTC_PAIR_UNLOCK();
    RtcChip.phase_coarse = true;
  }
#else
  RtcSetBoundary(time, b_ms, b_us, RTC_STRATUM_CHIP, RTC_PHASE_SYNC_SECS, RTC_PHASE_REGISTER, false);
#endif  // USE_RTC_CHIP_HOLDOVER
  RtcChip.phase_tries = 0;
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s phase synced, %u us in"), RtcChip.name, elapsed);
}
#ifdef USE_RTC_CHIP_HOLDOVER
/*********************************************************************************************\
 * RTC chip rate correction. The chip rate and the time it was last set are kept in its unused
 * alarm registers (layout in RtcClock.h), not in flash. The time wraps after 6 days
\*********************************************************************************************/
#define RTC_AUX_RATE_SPAN_S  300            // Min span to measure the chip rate against PPS, s
#define RTC_AUX_RATE_NET_SPAN_S 3000        // Min span against upstream NTP, s
#define RTC_AUX_LAG_MAX_US   30000000       // Max correction, us

struct {
  uint32_t placed_utc;             // UTC when the chip was last set, multiple of 128
  int32_t slow_ppb;                // Positive = chip runs slow, 0 = unknown
  bool placed_known;               // Set with a good phase
  bool trusted;                    // placed_utc is valid (no wrap)
  bool far;                        // Set from an internet NTP server
} RtcChipCorr;

void RtcAuxStore(void) {
  if (!RtcChip.AuxWrite) { return; }
  int32_t rate;
  uint32_t value = RtcAuxPack(RtcChipCorr.placed_known, RtcChipCorr.placed_utc, RtcChipCorr.slow_ppb,
                              RtcChipCorr.far, &rate);
  if (!RtcChipCorr.placed_known && !rate) {
    RtcChip.AuxWrite(value);
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s spare bits voided, placement unknown"), RtcChip.name);
    return;
  }
  bool ok = RtcChip.AuxWrite(value);
  uint32_t back = 0;
  bool kept = ok && RtcChip.AuxRead && RtcChip.AuxRead(&back) && (back == value);
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s spare bits %s, placed %u, %d ppb slow (wrote %08X read %08X)"),
    RtcChip.name, kept ? PSTR("kept") : PSTR("NOT kept"), RtcChipCorr.placed_utc,
    rate * RTC_AUX_RATE_PPB, value, back);
}

void RtcAuxLoad(uint32_t now) {
  uint32_t v;
  if (!RtcChip.AuxRead || !RtcChip.AuxRead(&v)) { return; }
  uint8_t got = RtcAuxUnpack(v, now, &RtcChipCorr.placed_utc, &RtcChipCorr.slow_ppb, &RtcChipCorr.far);
  if (RTC_AUX_FOREIGN == got) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s spare bits not ours (%06X)"), RtcChip.name, v);
    return;
  }
  if (RTC_AUX_RATE_ONLY == got) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s runs %d ppb slow, placement unknown"), RtcChip.name,
      RtcChipCorr.slow_ppb);
    return;
  }
  RtcChipCorr.placed_known = true;
  // The time field wraps after 6 days: valid only if the settings were saved less than 4 days ago
  RtcChipCorr.trusted = (Settings->cfg_timestamp > START_VALID_TIME) &&
                        ((uint32_t)(now - Settings->cfg_timestamp) < RTC_HOLDOVER_TRUST_S);
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s placed %u s ago%s, runs %d ppb slow: %d ms behind by now"),
    RtcChip.name, now - RtcChipCorr.placed_utc, RtcChipCorr.far ? PSTR(" via internet") : PSTR(""), RtcChipCorr.slow_ppb,
    (int32_t)(((int64_t)RtcChipCorr.slow_ppb * (now - RtcChipCorr.placed_utc)) / 1000000));
}

// ESP drift register usable? The BM8563 timer register only while the timer is off (TE and TIE clear)
bool RtcChipDriftReg(void) {
  if (!RtcChip.drift_reg) { return false; }
  if (0x0F != RtcChip.drift_reg) { return true; }
  uint8_t tc, c2;
  return I2cValidRead8(&tc, RtcChip.address, 0x0E, RtcChip.bus) && !(tc & 0x80) &&
         I2cValidRead8(&c2, RtcChip.address, 0x01, RtcChip.bus) && !(c2 & 0x01);
}

void RtcChipDriftStore(int32_t ppb) {
  static uint8_t written = 0;
  uint8_t b = RtcDriftToByte(ppb);
  if ((b == written) || !RtcChipDriftReg()) { return; }
  if (I2cWrite8(RtcChip.address, RtcChip.drift_reg, b, RtcChip.bus)) { written = b; }
}

// After a power loss: the ESP drift from the chip, only together with our record
void RtcChipDriftLoad(void) {
  uint8_t b;
  if (Rtc.drift_ppb || !(RtcChipCorr.placed_known || RtcChipCorr.slow_ppb) || !RtcChipDriftReg() ||
      !I2cValidRead8(&b, RtcChip.address, RtcChip.drift_reg, RtcChip.bus)) { return; }
  Rtc.drift_ppb = RtcDriftFromByte(b);
  if (!Rtc.drift_ppb) { return; }
  Rtc.drift_source = RTC_DRIFT_SRC_NTP;     // Replaced by the next measured figure
  Rtc.drift_span = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: drift %d ppb from %s"), Rtc.drift_ppb, RtcChip.name);
}

int32_t RtcChipLagUs(uint32_t utc) {
  if (!RtcChipCorr.placed_known || !RtcChipCorr.slow_ppb) { return 0; }
  int32_t since = (int32_t)(utc - RtcChipCorr.placed_utc);
  if (since < 0) { return 0; }
  int64_t lag = ((int64_t)RtcChipCorr.slow_ppb * since) / 1000;
  if ((lag > RTC_AUX_LAG_MAX_US) || (lag < -RTC_AUX_LAG_MAX_US)) { return 0; }
  return (int32_t)lag;
}

// Correct a chip reading by the chip rate since it was set
void RtcChipCorrect(uint32_t* utc, uint32_t* b_us, uint32_t* b_ms) {
  int32_t lag = RtcChipLagUs(*utc);
  if (!lag) { return; }
  int32_t n = (lag >= 0) ? (lag / 1000000) : -((999999 - lag) / 1000000);
  int32_t f = lag - (n * 1000000);
  *utc += n;
  *b_us -= f;
  *b_ms -= f / 1000;
  if ((uint32_t)(millis() - *b_ms) >= 1000) {                     // Next second
    *utc += 1;
    *b_us += 1000000;
    *b_ms += 1000;
  }
}

// Chip error after a power loss, us, 0 = unknown (1 s)
uint32_t RtcChipHoldover(uint32_t utc, uint32_t catch_us) {
  if (!RtcChipCorr.placed_known || !RtcChipCorr.trusted || !RtcChipCorr.slow_ppb) { return 0; }
  int32_t age = (int32_t)(utc - RtcChipCorr.placed_utc);
  return (age >= 0) ? RtcChipHoldoverUs((uint32_t)age, catch_us, RtcChipCorr.far) : 0;
}

/*********************************************************************************************\
 * GPS without PPS: the message gives the second, the chip step set by PPS gives its start
\*********************************************************************************************/
#define RTC_CHIP_PLACE_MAX_US     250000    // Max error, else use the message timing
#define RTC_CHIP_PLACE_KEEP_S     900       // Max age of the chip step, s

struct {
  uint32_t flip_us;                // micros() of the chip step, rate corrected
  uint32_t flip_ms;                // millis() of the chip step
  uint32_t utc;                    // UTC of that second
  uint32_t taken;                  // Uptime of the catch
  bool valid;
  bool told;                       // Logged
} RtcChipFlip;

// Caught chip step young enough to place a second
bool RtcChipFlipFresh(void) {
  return RtcChipFlip.valid && ((TasmotaGlobal.uptime - RtcChipFlip.taken) <= RTC_CHIP_PLACE_KEEP_S);
}

// Second at micros() at_us from the caught chip step: its label and start. false = no fresh step
bool RtcChipFlipSecond(uint32_t at_us, uint32_t* utc, uint32_t* b_ms, uint32_t* b_us) {
  if (!RtcChipFlipFresh()) { return false; }
  int32_t off;
  *utc = RtcChipFlip.utc + RtcNearestSecond((int32_t)(at_us - RtcChipFlip.flip_us), Rtc.drift_ppb, &off);
  *b_us = at_us - (uint32_t)off;
  *b_ms = RtcChipFlip.flip_ms + (uint32_t)((int32_t)(*b_us - RtcChipFlip.flip_us) / 1000);
  return true;
}

bool RtcChipPlacedKnown(void) {
  return RtcChipCorr.placed_known;
}

bool RtcChipPlaceDue(void) {
  if (!RtcChip.seconds_reg || !RtcChipCorr.placed_known || !RtcChipCorr.slow_ppb) { return false; }
  if ((Rtc.stratum >= RTC_STRATUM_CHIP) || RtcPhaseGood()) { return false; }
  if (RtcPhaseNetwork()) { return false; }      // Not with upstream NTP
  uint32_t every = Rtc.drift_ppb ? 300 : 60;   // s
  return !RtcChipFlip.valid || ((TasmotaGlobal.uptime - RtcChipFlip.taken) >= every);
}

// Start of GPS second `named` from the chip step: the message names the chip second it arrived in.
// false = use the arrival
bool RtcChipPlaceSecond(uint32_t named, uint32_t arrival_us, uint32_t* b_ms, uint32_t* b_us, uint32_t* err_us) {
  bool ok = false;
  do {
    if (!RtcChipFlipFresh() || RtcChip.set_pending || RtcChip.release_pending) { break; }
    uint32_t since_catch = TasmotaGlobal.uptime - RtcChipFlip.taken;
    uint32_t chip_us = RtcChipHoldover(named, RTC_HOLDOVER_CATCH_US);   // Same bound as after a power loss
    if (!chip_us) { break; }
    int32_t ppb = Rtc.drift_ppb;
    uint64_t err = chip_us + ((uint64_t)since_catch * (ppb ? RTC_DRIFT_RESIDUAL_PPB : NTP_DRIFT_UNKNOWN_PPB)) / 1000;
    if (err > RTC_CHIP_PLACE_MAX_US) { break; }
    uint32_t bu;
    uint32_t label = RtcSecondOfArrival(arrival_us, RtcChipFlip.flip_us, RtcChipFlip.utc, ppb,
                                        (uint32_t)err + RTC_MSG_GUARD_US, &bu);
    if (label != named) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("RTC: %s second %u, message names %u: not used"), RtcChip.name, label, named);
      break;
    }
    *b_us = bu;
    *b_ms = RtcChipFlip.flip_ms + (uint32_t)((int32_t)(bu - RtcChipFlip.flip_us) / 1000);
    *err_us = (uint32_t)err;
    ok = true;
  } while (0);
  if (ok != RtcChipFlip.told) {
    if (ok) {
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s flip places the GPS-named second, bound %u us"),
        RtcChip.name, *err_us);
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s flip no longer places the GPS-named second"), RtcChip.name);
    }
    RtcChipFlip.told = ok;
  }
  return ok;
}

/*********************************************************************************************\
 * Fine catch: chip second step to about 50 us (sequential equivalent-time sampling). A scan finds
 * the step, then ten bursts of reads around the expected step, one per second, each shifted by a
 * tenth of a read. A line through them gives the step time and the chip rate
\*********************************************************************************************/
#define RTC_FINE_BURSTS        10
#define RTC_FINE_SCAN_MAX_MS   1500         // Max scan time, ms
#define RTC_FINE_SCAN_WIDE_US  25000        // Max scan read gap, us
#define RTC_FINE_HALF_US       1200         // Half burst window, us
#define RTC_FINE_ENTER_US      20000        // Max wait for a burst, us
#define RTC_FINE_TRIES         10           // Max retries

#ifndef RTC_RELEASE_LEAD_US
#define RTC_RELEASE_LEAD_US    1100
#endif
#define RTC_RELEASE_LEAD_MAX   5000         // Max lead correction, us
int32_t rtc_release_lead_us = RTC_RELEASE_LEAD_US;
int32_t rtc_release_fired_us = 0;           // us the last start was off the second

enum { RTC_FINE_IDLE = 0, RTC_FINE_SCAN, RTC_FINE_BURST };

struct {
  uint8_t state;
  uint8_t n;                       // Bursts done
  uint8_t tries;
  uint8_t last_sec;                // Seconds register after the last step
  bool have_last;
  bool placement;                  // Catch after setting the chip
  uint32_t last_at;                // micros() of the last read of last_sec
  uint32_t base_us;                // micros() of the scan step
  uint32_t half_us;
  uint32_t started_ms;
  uint32_t read_us;                // Slowest read, us
  int32_t mid[RTC_FINE_BURSTS];    // Step time - (base_us + k * 1e6), us
  uint8_t k[RTC_FINE_BURSTS];      // Seconds after base_us
  int32_t prev_skew_us;
  uint32_t prev_skew_uptime;
  bool prev_skew_valid;
  bool prev_skew_edge;             // Measured against PPS
} RtcFine;

void RtcFineStart(bool placement) {
  if (!RtcChip.seconds_reg || (RTC_FINE_IDLE != RtcFine.state)) { return; }
  RtcFine.state = RTC_FINE_SCAN;
  RtcFine.n = 0;
  RtcFine.tries = 0;
  RtcFine.have_last = false;
  RtcFine.placement = placement;
  RtcFine.read_us = 0;
  RtcFine.started_ms = millis();
}

static uint32_t RtcFineHalfFloor(void) {
  uint32_t two_reads = (2 * RtcFine.read_us) + 300;
  return (two_reads > RTC_FINE_HALF_US) ? two_reads : RTC_FINE_HALF_US;
}

static void RtcFineAbort(const char* why) {
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s fine catch abandoned, %s"), RtcChip.name, why);
  RtcFine.state = RTC_FINE_IDLE;
}

static bool RtcFineRead(uint8_t* sec, uint32_t* at) {
  uint32_t before = micros();
  uint8_t s = RtcChipReadSeconds();
  uint32_t took = micros() - before;
  if (0xFF == s) { return false; }
  if (took > RtcFine.read_us) { RtcFine.read_us = took; }
  *sec = s;
  *at = before + (took / 2);
  return true;
}

static int64_t RtcDivRound(int64_t a, int64_t b) { return ((a >= 0) ? (a + b / 2) : (a - b / 2)) / b; }

// Line fit through (k, mid): value at k in us, slope in ns/s
static int32_t RtcFineLine(uint32_t k, int32_t* slope_ns) {
  int64_t n = RtcFine.n, sk = 0, sm = 0, skk = 0, skm = 0;
  for (uint32_t i = 0; i < n; i++) {
    int64_t x = RtcFine.k[i], y = RtcFine.mid[i];
    sk += x; sm += y; skk += x * x; skm += x * y;
  }
  int64_t d = n * skk - sk * sk;
  int64_t q = (d > 0) ? (n * skm - sk * sm) : 0;
  if (d <= 0) { d = 1; }
  if (slope_ns) { *slope_ns = (int32_t)RtcDivRound(q * 1000, d); }
  return n ? (int32_t)RtcDivRound(sm * d + q * (n * (int64_t)k - sk), n * d) : 0;
}

static int32_t RtcFineSkew(uint32_t flip) { int32_t off; RtcSecondAt(flip, &off); return off; }

// Skew taken as the one nearest the rate prediction: a lag beyond 0.5 s wraps the raw skew
static int32_t RtcFineSkewNear(uint32_t flip, int32_t predicted) {
  int32_t d = RtcFineSkew(flip) - predicted;
  d -= 1000000 * ((d + ((d >= 0) ? 500000 : -500000)) / 1000000);
  return predicted + d;
}

static void RtcFineFinish(void) {
  RtcFine.state = RTC_FINE_IDLE;
  uint32_t k_last = RtcFine.k[RtcFine.n - 1];
  int32_t slope_ns;
  uint32_t flip = RtcFine.base_us + (k_last * 1000000) + RtcFineLine(k_last, &slope_ns);
  uint32_t chip_utc = RtcChip.ReadTime();
  if (HighestLogLevel() >= LOG_LEVEL_DEBUG) {                     // Scatter of the bursts around the line
    uint64_t ss = 0;
    for (uint32_t i = 0; i < RtcFine.n; i++) {
      int64_t r = RtcFine.mid[i] - RtcFineLine(RtcFine.k[i], nullptr);
      ss += r * r;
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s flip caught, sd %u us over %u s, read %u us, chip %d ppb%s"),
      RtcChip.name, (uint32_t)sqrtf((float)ss / (RtcFine.n - 2)), k_last, RtcFine.read_us, Rtc.drift_ppb - slope_ns,
      Rtc.drift_ppb ? PSTR("") : PSTR(" (ESP drift unmeasured)"));
  }

  {
    uint32_t c_utc = chip_utc, c_us = flip;
    uint32_t c_ms = millis() - ((micros() - flip) / 1000);
    RtcChipCorrect(&c_utc, &c_us, &c_ms);
    RtcChipFlip.flip_us = c_us;
    RtcChipFlip.flip_ms = c_ms;
    RtcChipFlip.utc = c_utc;
    RtcChipFlip.taken = TasmotaGlobal.uptime;
    RtcChipFlip.valid = true;
  }

  int32_t predicted = RtcChipLagUs(chip_utc);                      // The chip's lag by its rate
  int32_t off_us;
  uint32_t ours = RtcSecondAt(RtcChipFlip.flip_us, &off_us);       // Our second at its step
  if ((Rtc.stratum < RTC_STRATUM_CHIP) && !RtcPhasePlaceable()) {
    RtcFine.prev_skew_valid = false;                 // No time source
    RtcChip.phase_limit = 0;
    if (RtcPhaseNetwork()) {
      int32_t skew = RtcFineSkewNear(flip, predicted);
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s skew %d us against upstream (correction predicted %d us)"),
        RtcChip.name, skew, predicted);
      if ((ours != RtcChipFlip.utc) && (abs(off_us) < 350000)) {
        RtcChipArmSecondsFix();
      } else if (abs(skew - predicted) > RTC_HOLDOVER_PLACE_FAR_US) {
        RtcChipArmWrite();
        RtcChip.net_reseat = true;
        AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s re-seating, skew beyond %d us of upstream"), RtcChip.name,
          RTC_HOLDOVER_PLACE_FAR_US);
      }
    }
    return;
  }
  if (Rtc.stratum < RTC_STRATUM_CHIP) {
    bool edge = RtcPhaseGood();                      // PPS, else upstream NTP
    int8_t place = RtcPhasePlaceClass();
    int32_t skew = RtcFineSkewNear(flip, predicted);
    if ((ours != RtcChipFlip.utc) && (abs(off_us) < 350000) && !RtcFine.placement) {
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s %d s off our clock, re-seating it"), RtcChip.name,
        (int32_t)(RtcChipFlip.utc - ours));
      RtcChipArmSecondsFix();
    }
    if (RtcFine.placement && (abs(abs(skew) - 500000) < 100000)) {    // First step 0.5 s or 1 s after the start
      RtcChip.release_flip_us = 1500000 - RtcChip.release_flip_us;
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s steps %u ms after the start, setting it again"), RtcChip.name,
        RtcChip.release_flip_us / 1000);
      RtcChipArmWrite();
      return;
    }
    if (RtcFine.placement) {
      int32_t response = skew - rtc_release_fired_us;
      bool learn = edge && (response >= 0) && (response <= RTC_RELEASE_LEAD_MAX);
      int32_t lead = learn ? ((rtc_release_lead_us + response) / 2) : rtc_release_lead_us;
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s placed %d us off our boundary (fired %d us, response %d us), lead %d -> %d us"),
        RtcChip.name, skew, rtc_release_fired_us, response, rtc_release_lead_us, lead);
      rtc_release_lead_us = lead;
    } else if (RtcFine.prev_skew_valid && (RtcFine.prev_skew_edge == edge) &&
               (TasmotaGlobal.uptime > RtcFine.prev_skew_uptime)) {
      uint32_t span = TasmotaGlobal.uptime - RtcFine.prev_skew_uptime;
      int32_t rate = (int32_t)(((int64_t)(skew - RtcFine.prev_skew_us) * 1000) / span);
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s skew %d us (correction predicted %d us), chip runs %d ppb slow over %u s"),
        RtcChip.name, skew, predicted, rate, span);
      if (span >= (edge ? RTC_AUX_RATE_SPAN_S : RTC_AUX_RATE_NET_SPAN_S)) {
        RtcChipCorr.slow_ppb = rate;
        RtcAuxStore();
      }
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s skew %d us (correction predicted %d us)"),
        RtcChip.name, skew, predicted);
    }
    RtcFine.prev_skew_us = skew;
    RtcFine.prev_skew_uptime = TasmotaGlobal.uptime;
    RtcFine.prev_skew_valid = true;
    RtcFine.prev_skew_edge = edge;
    int32_t limit_us = RtcChip.phase_limit ? (RtcChip.phase_limit * 1000) :
                       edge ? RTC_EPOCH_EDGE_US : (place ? RTC_HOLDOVER_PLACE_FAR_US : RTC_HOLDOVER_PLACE_US);
    if (!RtcFine.placement) {
      RtcChip.phase_limit = 0;
      int32_t age = (int32_t)(chip_utc - RtcChipCorr.placed_utc);
      int32_t off = skew - RtcChipLagUs(chip_utc);                // Off its prediction
      if (abs(off) > limit_us) {
        RtcChipArmWrite();
        AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s re-seating, %d us off its prediction, beyond %d us"), RtcChip.name,
          off, limit_us);
      } else if (!RtcChipCorr.placed_known) {
        RtcChipArmWrite();
        AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s re-seating, placement unknown"), RtcChip.name);
      } else if (age >= RTC_HOLDOVER_REPLACE_S) {
        RtcChipArmWrite();
        AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s re-seating, placed %d s ago"), RtcChip.name, age);
      }
    }
    return;
  }
  if (RtcChipCrystalCarries() && !RtcChip.phase_coarse) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s %d us from our clock, which keeps the crystal"),
      RtcChip.name, RtcFineSkew(flip));
    return;
  }
  if (RtcChip.stopped) { return; }                                 // Time lost, never a source
  uint32_t now_us = micros();
  uint32_t now_ms = millis();
  uint32_t b_ms = now_ms - ((now_us - flip) / 1000);
  RtcChipCorrect(&chip_utc, &flip, &b_ms);                         // Correct by the chip rate
  bool taken = RtcSetBoundary(chip_utc, b_ms, flip, RTC_STRATUM_CHIP, RTC_PHASE_SYNC_SECS,
                              RTC_PHASE_REGISTER, false);
  if (taken) {
    RTC_PAIR_LOCK();
    Rtc.held_us = RtcChipHoldover(chip_utc, RTC_HOLDOVER_CATCH_US);
    RTC_PAIR_UNLOCK();
    RtcChip.phase_coarse = false;
  }
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s phase from fine catch %s"), RtcChip.name,
    taken ? PSTR("taken") : PSTR("refused"));
}

// Fine catch, every loop. Waits at most RTC_FINE_ENTER_US plus the window
void RtcFineLoop(void) {
  if (RTC_FINE_IDLE == RtcFine.state) { return; }
  if (RtcChip.set_pending || RtcChip.release_pending) { RtcFineAbort(PSTR("write pending")); return; }

  if (RTC_FINE_SCAN == RtcFine.state) {
    uint8_t s;
    uint32_t at;
    if (!RtcFineRead(&s, &at)) { RtcFineAbort(PSTR("read failed")); return; }
    if (!RtcFine.have_last || (s == RtcFine.last_sec)) {
      if (!RtcFine.have_last) { RtcFine.last_sec = s; RtcFine.have_last = true; }
      RtcFine.last_at = at;
      if ((millis() - RtcFine.started_ms) > RTC_FINE_SCAN_MAX_MS) { RtcFineAbort(PSTR("no step")); }
      return;
    }
    uint32_t width = at - RtcFine.last_at;
    RtcFine.last_sec = s;
    if (width > RTC_FINE_SCAN_WIDE_US) {                          // Loop too slow
      if (++RtcFine.tries > RTC_FINE_TRIES) { RtcFineAbort(PSTR("loop too slow")); return; }
      RtcFine.last_at = at;
      RtcFine.started_ms = millis();
      return;
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s scan bracket %u us, read %u us"), RtcChip.name, width, RtcFine.read_us);
    RtcFine.base_us = RtcFine.last_at + (width / 2);
    RtcFine.half_us = (width / 2) + RtcFineHalfFloor();
    RtcFine.n = 0;
    RtcFine.tries = 0;
    RtcFine.state = RTC_FINE_BURST;
    return;
  }

  uint32_t k = (RtcFine.n ? RtcFine.k[RtcFine.n - 1] : 0) + 1;
  uint32_t pred;
  int32_t to_go;
  do {                                                            // Next burst
    pred = RtcFine.base_us + (k * 1000000) + RtcFineLine(k, nullptr);
    to_go = (int32_t)(pred - micros());
    if (to_go >= (int32_t)RtcFine.half_us) { break; }
    k++;
  } while (k < 250);
  if (k >= 250) { RtcFineAbort(PSTR("lost the flip")); return; }
  if (to_go > (int32_t)(RtcFine.half_us + RTC_FINE_ENTER_US)) { return; }

  uint32_t start = pred - RtcFine.half_us + ((RtcFine.read_us * RtcFine.n) / RTC_FINE_BURSTS)
                 - (RtcFine.read_us / 2);
  while ((int32_t)(micros() - start) < 0) { }
  uint32_t window = 2 * RtcFine.half_us;
  uint8_t s0, s;
  uint32_t at0, at;
  if (!RtcFineRead(&s0, &at0)) { RtcFineAbort(PSTR("read failed")); return; }
  bool stepped = false;
  while ((int32_t)(micros() - start) < (int32_t)window) {
    if (!RtcFineRead(&s, &at)) { RtcFineAbort(PSTR("read failed")); return; }
    if (s != s0) { stepped = true; break; }
    at0 = at;
  }
  if (!stepped) {
    if (++RtcFine.tries > RTC_FINE_TRIES) { RtcFineAbort(PSTR("bursts keep missing")); return; }
    RtcFine.half_us = (RtcFine.half_us < RTC_FINE_SCAN_WIDE_US / 2) ? (2 * RtcFine.half_us) : RTC_FINE_SCAN_WIDE_US;
    return;
  }
  uint32_t mid = at0 + ((at - at0) / 2);
  RtcFine.mid[RtcFine.n] = (int32_t)(mid - (RtcFine.base_us + (k * 1000000)));
  RtcFine.k[RtcFine.n] = k;
  RtcFine.n++;
  RtcFine.last_sec = s;
  if (RtcFine.n >= 2) { RtcFine.half_us = RtcFineHalfFloor(); }
  if (RtcFine.n >= RTC_FINE_BURSTS) { RtcFineFinish(); }
}

#endif  // USE_RTC_CHIP_HOLDOVER

// While powered, the drift corrected ESP crystal is better than the chip
bool RtcChipCrystalCarries(void) {
  return (Rtc.drift_ppb != 0) && (Rtc.stratum < RTC_STRATUM_NONE) &&
         (RTC_PHASE_COLD != Rtc.phase_source);
}

#ifdef USE_RTC_CHIP_HOLDOVER

// Request setting the chip, keep the current stratum
void RtcChipArmWrite(void) {
  if (!RtcChip.set_pending || (Rtc.stratum < RtcChip.armed_stratum)) { RtcChip.armed_stratum = Rtc.stratum; }
  if (!RtcChip.set_pending) { RtcChip.fix_seconds = false; }
  RtcChip.set_pending = true;
}

void RtcChipArmSecondsFix(void) {
  RtcChipArmWrite();
  RtcChip.fix_seconds = true;
}

#define RTC_RELEASE_SPIN_US    30000        // Max wait for the release second, us
#define RTC_RELEASE_GIVEUP_MS  5000         // Release anyway after this, ms

// Every loop: start the stopped chip at the next second
void RtcChipReleasePoll(void) {
  if (!RtcChip.release_pending || !RtcChip.ReleaseTime) { return; }

  RTC_PAIR_LOCK();
  uint32_t b_us = Rtc.boundary_micros;
  uint32_t utc = Rtc.utc_time;
  int32_t ppb = Rtc.drift_ppb;
  RTC_PAIR_UNLOCK();
  uint32_t now = micros();
  uint32_t shift = 1000000 - RtcChip.release_flip_us;              // PCF85063 steps 0.5 s after the start
  int32_t off;
  int32_t secs = RtcNearestSecond((int32_t)(now - shift - b_us), ppb, &off);
  if (off >= 0) { secs++; }
  uint32_t next = b_us + (uint32_t)RtcSecondsUs(secs, ppb);
  uint32_t next_utc = utc + secs;                                  // Second starting at `next`
  bool overdue = ((int32_t)(millis() - RtcChip.release_armed_ms) > RTC_RELEASE_GIVEUP_MS);
  if (!overdue && ((int32_t)(next + shift - now) > RTC_RELEASE_SPIN_US)) { return; }

  if (!overdue && (next_utc != RtcChip.release_utc)) {
    if (RtcChip.release_reloads < RTC_RELEASE_MAX_RELOADS) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s release missed its second (%u, next %u), reloading"),
        RtcChip.name, RtcChip.release_utc, next_utc);
      RtcChip.release_reloads++;
      RtcChip.SetTimeHeld(next_utc);
      RtcChip.release_utc = next_utc;
      return;
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s released %d s off, out of reloads"),
      RtcChip.name, (int32_t)(RtcChip.release_utc - next_utc));
  }

  uint32_t aim = next + shift - rtc_release_lead_us;
  while (!overdue && ((int32_t)(micros() - aim) < 0)) { }
  uint32_t before = micros();
  RtcChip.ReleaseTime();
  uint32_t took = micros() - before;
  rtc_release_fired_us = (int32_t)(before - (next + shift));
  RtcChip.release_pending = false;
  RtcChipCorr.placed_known = (!RtcChip.AuxWrite || (0 == (RtcChip.release_utc % RTC_AUX_UNIT_S))) &&
                             !overdue && RtcPhasePlaceable();   // Record chips are set on the 128 s grid
  RtcChipCorr.trusted = RtcChipCorr.placed_known;
  RtcChipCorr.far = (1 == RtcPhasePlaceClass());
  RtcChipCorr.placed_utc = RtcChip.release_utc;
  RtcAuxStore();
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s released %d us from the boundary, write %u us%s"),
    RtcChip.name, rtc_release_fired_us, took, overdue ? PSTR(", overdue") : PSTR(""));
  RtcFineStart(true);                                              // Check the result
}

// Set the chip on a multiple of 128 s (for the record)
void RtcChipSetTimePoll(void) {
  if (!RtcChip.set_pending || !RtcChip.SetTime) { return; }

  uint32_t since = RtcMillis();
  if (since >= 100) { return; }

  uint32_t utc = Rtc.utc_time + RtcCarry(nullptr);
  // Second to write: the next one, or this one for the PCF85063
  uint32_t load = utc + ((RtcChip.release_flip_us < 1000000) ? 0 : 1);
  if (RtcChip.AuxWrite && (load % RTC_AUX_UNIT_S)) { return; }
  if (RtcChip.AuxWrite && !((load / RTC_AUX_UNIT_S) & RTC_AUX_TIME_MASK)) { return; }

  RtcChip.set_pending = false;
  if (!RtcChip.fix_seconds && !RtcPhasePlaceable() && !RtcChip.net_reseat) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: %s re-seat dropped, no edge to place it from"), RtcChip.name);
    return;
  }
  RtcChip.net_reseat = false;
  RtcChipFlip.valid = false;
  if (RtcChip.SetTimeHeld && RtcChip.ReleaseTime) {
    RtcChip.SetTimeHeld(load);
    RtcChip.release_utc = load;
    RtcChip.release_reloads = 0;
    RtcChip.release_pending = true;
    RtcChip.release_armed_ms = millis();
  } else {
    RtcChip.SetTime(utc);
  }
  RtcChip.stopped = false;
  RtcSettings.chip_writer_stratum = (RtcChip.armed_stratum < Rtc.stratum) ? RtcChip.armed_stratum : Rtc.stratum;
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s re-synced to %u (+%u carried), %u ms into the second, stratum %d"),
    RtcChip.name, utc, (uint32_t)(utc - Rtc.utc_time), since, Rtc.stratum);
}
#endif  // USE_RTC_CHIP_HOLDOVER
#endif  // USE_NTP_SERVER

void RtcChipTimeSynced(void) {
#ifdef USE_NTP_SERVER
  if ((Rtc.stratum >= RTC_STRATUM_CHIP) && (RTC_PHASE_MANUAL != Rtc.phase_source)) { return; }   // Time from the chip
#endif  // USE_NTP_SERVER
#ifdef USE_RTC_CHIP_HOLDOVER
  if (RtcChip.set_pending) { return; }
#endif  // USE_RTC_CHIP_HOLDOVER
  if ((Rtc.utc_time > START_VALID_TIME) &&                        // Valid UTC time
      (abs((int32_t)(Rtc.utc_time - RtcChip.ReadTime())) > 2)) {  // Time has drifted from RTC more than 2 seconds
#ifdef USE_RTC_CHIP_HOLDOVER
    RtcChipArmSecondsFix();                                       // Set at the next second, with the phase
#else
    RtcChip.SetTime(Rtc.utc_time);                                // Update time
#ifdef USE_NTP_SERVER
    RtcSettings.chip_writer_stratum = Rtc.stratum;
#endif  // USE_NTP_SERVER
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: %s re-synced (" D_UTC_TIME ") %s"), RtcChip.name, GetDateAndTime(DT_UTC).c_str());
#endif  // USE_RTC_CHIP_HOLDOVER
  }
}

int32_t RtcChipMemSize(void) {
  return RtcChip.mem_size;                                        // Not supported or max size
}

int32_t RtcChipMemRead(uint8_t *buffer, uint32_t size) {
  if (size <= RtcChip.mem_size) {
    return RtcChip.MemRead(buffer, size);
  }
  return -1;                                                      // Not supported or too large
}

int32_t RtcChipMemWrite(uint8_t *buffer, uint32_t size) {
  if (size <= RtcChip.mem_size) {
    return RtcChip.MemWrite(buffer, size);
  }
  return -1;                                                      // Not supported or too large
}

/*********************************************************************************************\
 * NTP server functions
\*********************************************************************************************/
#ifdef RTC_NTP_SERVER

#include "NTPServer.h"
#include "NTPPacket.h"

const char kRtcChipCommands[] PROGMEM = "Rtc|"  // Prefix
  D_CMND_NTPSERVER
#ifdef USE_RTC_CHIP_HOLDOVER
  "|" D_CMND_RTCPHASE "|Catch"
#endif  // USE_RTC_CHIP_HOLDOVER
  ;

void (* const RtcChipCommand[])(void) PROGMEM = {
  &CmndRtcNtpServer,
#ifdef USE_RTC_CHIP_HOLDOVER
  &CmndRtcPhase, &CmndRtcCatch
#endif  // USE_RTC_CHIP_HOLDOVER
  };

NtpServer RtcChipTimeServer(PortUdp);

bool RtcChipNtpSource(NtpTimeBase* base, uint32_t* utc, uint32_t* boundary_us) {
  if (TasmotaGlobal.global_state.network_down || !Settings->sbflag1.local_ntp_server) { return false; }
  return RtcNtpRequest(RTC_NTP_OWNER_CHIP, base, utc, boundary_us);
}

// Open NTP port 123 for the chip server, unless already used
void RtcChipNtpListen(void) {
  if (RtcNtpBoundBy(RTC_NTP_OWNER_CHIP) || !RtcNtpBind(RTC_NTP_OWNER_CHIP)) { return; }
  RtcChipTimeServer.setSource(RtcChipNtpSource, RtcNtpSent);
  if (RtcChipTimeServer.beginListening()) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: NTP server started"));
  } else {
    RtcNtpRelease(RTC_NTP_OWNER_CHIP);             // Failed
  }
}

void RtcChipNtpEverySecond(void) {
  if (TasmotaGlobal.global_state.network_down) { return; }  // Exception on ESP32 if network is down (#17338)
  if (!RtcChip.detected) { return; }
  if (!Settings->sbflag1.local_ntp_server || (Rtc.utc_time <= START_VALID_TIME)) { return; }
  RtcChipNtpListen();
}

#ifdef USE_RTC_CHIP_HOLDOVER
void CmndRtcPhase(void) {
  if (!RtcChip.detected || (Rtc.stratum >= RTC_STRATUM_CHIP)) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: no %s, audit skipped"),
      RtcChip.detected ? PSTR("clock better than the chip") : PSTR("chip"));
    ResponseCmndNumber(-1);
    return;
  }
  RtcChip.phase_limit = (XdrvMailbox.payload > 0) ? XdrvMailbox.payload : RTC_CHIP_PHASE_MAX_MS;
  RtcChip.phase_audit = 0;                     // Check now
  RtcChip.phase_time = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: phase audit armed, limit %d ms"), RtcChip.phase_limit);
  ResponseCmndDone();
}

void CmndRtcCatch(void) {
  if (!RtcChip.seconds_reg) { ResponseCmndChar_P(PSTR("Not supported")); return; }
  if ((RTC_FINE_IDLE != RtcFine.state) || RtcChip.set_pending || RtcChip.release_pending) {
    ResponseCmndChar_P(PSTR("Busy"));
    return;
  }
  if (1 == XdrvMailbox.payload) {
    if (Rtc.stratum >= RTC_STRATUM_CHIP) { ResponseCmndChar_P(PSTR("No better clock")); return; }
    RtcChipArmWrite();
    RtcChip.net_reseat = true;               // Set even without a good phase
  } else {
    RtcFineStart(false);
  }
  ResponseCmndDone();
}
#endif  // USE_RTC_CHIP_HOLDOVER

void CmndRtcNtpServer(void) {
  if (!RtcChip.detected) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: no chip, NTP server not available"));
    Settings->sbflag1.local_ntp_server = 0;
    ResponseCmndStateText(0);
    return;
  }
  if (((0 == XdrvMailbox.payload) || (1 == XdrvMailbox.payload)) && !TasmotaGlobal.global_state.network_down) {
    if (XdrvMailbox.payload) {
      Settings->sbflag1.local_ntp_server = 1;
      RtcChipNtpListen();
    } else {
      Settings->sbflag1.local_ntp_server = 0;
      if (RtcNtpBoundBy(RTC_NTP_OWNER_CHIP)) {       // Release for the GPS server
        RtcChipTimeServer.stopListening();
        RtcNtpRelease(RTC_NTP_OWNER_CHIP);
      }
    }
  }
  ResponseCmndStateText(Settings->sbflag1.local_ntp_server);
}
#endif  // RTC_NTP_SERVER

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv56(uint32_t function) {
  bool result = false;

#ifdef RTC_NTP_SERVER
  switch (function) {
    case FUNC_LOOP:
    case FUNC_SLEEP_LOOP:
      RtcChipTimeServer.poll();
      break;
    case FUNC_EVERY_SECOND:
      RtcChipNtpEverySecond();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(kRtcChipCommands, RtcChipCommand);
      break;
  }
#endif  // RTC_NTP_SERVER

  if (FUNC_SETUP_RING1 == function) {
    RtcChipDetect();
  }
  else if (RtcChip.detected) {
    switch (function) {
      case FUNC_TIME_SYNCED:
        RtcChipTimeSynced();
        break;
#ifdef USE_RTC_CHIP_HOLDOVER
      case FUNC_LOOP:
      case FUNC_SLEEP_LOOP:
        RtcChipReleasePoll();
        RtcFineLoop();
        break;
#endif  // USE_RTC_CHIP_HOLDOVER
#ifdef USE_NTP_SERVER
      case FUNC_EVERY_100_MSECOND:
#ifdef USE_RTC_CHIP_HOLDOVER
        RtcChipSetTimePoll();
#endif  // USE_RTC_CHIP_HOLDOVER
        RtcChipSyncPhasePoll();
        break;
#endif  // USE_NTP_SERVER
      case FUNC_WEB_SENSOR:
        if (RtcChip.ShowSensor) RtcChip.ShowSensor(0);
        break;
      case FUNC_JSON_APPEND:
        if (RtcChip.ShowSensor) RtcChip.ShowSensor(1);
        break;
      case FUNC_ACTIVE:
        result = true;
        break;
    }
  }

  return result;
}

#endif  // USE_RTC_CHIPS
#endif  // USE_I2C
