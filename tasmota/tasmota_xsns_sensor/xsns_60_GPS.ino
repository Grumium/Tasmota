/*
  xsns_60_GPS.ino - GPS UBLOX support for Tasmota

  Copyright (C) 2021  Theo Arends, Christian Baars and Adrian Scillato
  Copyright (C) 2026  Jan-David Förster and Claude

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

#ifdef USE_GPS
/*********************************************************************************************\
  --------------------------------------------------------------------------------------------
  Version Date      Action    Description
  --------------------------------------------------------------------------------------------

  0.9.3.0 20200214  integrate - fix set lat/lon via commandd 13, V-Port now works parallel
  ---
  0.9.2.0 20200110  integrate - Added UART-over-TCP/IP-bridge (virtual serial port). Minor tweaks.
  ---
  0.9.1.0 20191216  integrate - Added pin specifications from Tasmota WEB UI. Minor tweaks.
  ---
  0.9.0.0 20190817  started   - further development by Christian Baars  - https://github.com/Staars/Sonoff-Tasmota
                    forked    - from arendst/tasmota                    - https://github.com/arendst/Sonoff-Tasmota
                    base      - code base from arendst and              - https://www.youtube.com/watch?v=TwhCX0c8Xe0

## GPS-driver for the Ublox-series 6-8
Driver is tested on a NEO-6m and a Beitian-220. Series 7 should work too. This adds only about 6kb to the program size, because the efficient UBX-protocol is used. These modules are quite cheap, starting at about 3.50€ for the NEO-6m.

## Features:
- get position and time data
- sets system time automatically and Settings->latitude and Settings->longitude via command
- can log postion data with timestamp to flash with a small memory footprint of only 12 Bytes per record
- constructs a GPX-file for download of this data
- Web-UI
- simplified NTP-server and UART-over-TCP/IP-bridge (virtual serial port)
- command interface
- velocity and heading information with #define USE_GPS_VELOCITY

## Usage:
The serial pins are GPS_RX and GPS_TX, no further installation steps needed. To get more debug information compile it with option "DEBUG_TASMOTA_SENSOR".


## Commands:

+ sensor60 0
  write to all available sectors, then restart and overwrite the older ones

+ sensor60 1
  write to all available sectors, then restart and overwrite the older ones

+ sensor60 2
  filter out horizontal drift noise

+ sensor60 3
  turn off noise filter

+ sensor60 4
  start recording, new data will be appended

+ sensor60 5
  start new recording, old data will lost

+ sensor60 6
  stop recording, download link will be visible in Web-UI

+ sensor60 7
  send mqtt on new postion + TELE -> consider to set TELE to a very high value

+ sensor60 8
  only TELE message

+ sensor60 9
  start NTP-server

+ sensor60 10
  deactivate NTP-server

+ sensor60 11
  force update of Tasmota-system-UTC with every new GPS-time-message

+ sensor60 12
  do not update of Tasmota-system-UTC with every new GPS-time-message

+ sensor60 13
  set latitude and longitude in settings

+ sensor60 14
  open virtual serial port over TCP, usable for u-center

+ sensor60 15
  pause virtual serial port over TCP

+ sensor60 18
  keep sending position and velocity while the NTP server runs
  without PPS this adds jitter to the served time

+ sensor60 19
  pure NTP-mode, stop position and velocity while the NTP server runs

  Without 18/19 the default depends on PPS: with a PPS pin the position keeps
  streaming, without it the driver switches to pure NTP-mode.

## Commands

+ GpsNtpServer 0|1
  0 = off, 1 = on. Kept in the settings.

+ GpsPps 0|1
  0 = ignore the PPS edge, 1 = use it (default). Kept in the settings.

+ GpsReset 0|1|2
  Restarts the receiver's GNSS part (UBX-CFG-RST), its configuration stays:
  0 = hot start (keeps time, position, orbits: fix within seconds),
  1 = warm start (drops the orbit data: fix in about half a minute or more),
  2 = cold start (drops everything the backup battery keeps: fix in minutes,
  as after a long time without power). Until the receiver has a fix again its
  pulse is not used; upstream or the chip hold the time.

## PPS

With a pin assigned to GPIO_GPS_PPS the pulse, not the serial message, is the
second boundary - the message only names the second. The driver gives the
unlocked pulse the width the locked one has and asks for UBX-TIM-TP, which is
the only way to tell a steered pulse from a free running one: they are identical
at the pin. Without a fix the phase stays good and only the claim drops, so the
server keeps answering at a lower stratum rather than going quiet.

## Rules examples for SSD1306 32x128


rule1 on tele-GPS#lat do DisplayText [s1p21c1l01f1]LAT: %value% endon on tele-GPS#lon do DisplayText [s1p21c1l2]LON: %value% endon on switch1#state==3 do sensor60 4 endon on switch1#state==2 do sensor60 6 endon

rule2  on tele-GPS#int>9 do DisplayText [f0c9l4]I%value%  endon  on tele-GPS#int<10 do DisplayText [f0c9l4]I0%value%  endon on tele-GPS#fil==1 do DisplayText [f0c18l4]F endon on tele-GPS#fil==0 do DisplayText [f0c18l4]N endon

rule3 on tele-FLOG#sec do DisplayText  [f0c1l4]SAV:%value%  endon on tele-FLOG#rec==1 do DisplayText [f0c1l4]REC: endon on tele-FLOG#mode do DisplayText [f0c14l4]M%value% endon

\*********************************************************************************************/

#define XSNS_60        60

#include "NTPServer.h"
#include "NTPPacket.h"

/*********************************************************************************************\
 * constants
\*********************************************************************************************/

#define D_CMND_UBX "UBX"

const char S_JSON_UBX_COMMAND_NVALUE[] PROGMEM = "{\"" D_CMND_UBX "%s\":%d}";

const char kUBXTypes[] PROGMEM = "UBX";

#define UBX_LAT_LON_THRESHOLD 100 // filter out some noise of local drift

#define UBX_SERIAL_BUFFER_SIZE 256
#define UBX_TCP_PORT           1234

#define UBX_PPS_MAX_AGE        850             // Max ms from PPS edge to its message
#define UBX_PPS_MIN_AGE        5               // Min ms from PPS edge to its message
#define UBX_MSG_FRESH_MS       250             // Max ms difference of iTOW steps and arrival steps
#define UBX_MSG_STREAK         12              // Messages on time before message timing is used (UART buffer holds ~9)
#define UBX_NAMED_EDGE_QUIET_MS 5000           // ms without messages before PPS alone is used
#define UBX_PPS_SETTLE         3               // Good PPS edges in a row before use
#define UBX_PPS_SPACING_TOL    500             // Max us a PPS edge may deviate from whole seconds
#define UBX_PPS_MAX_SKIP       10              // Max missed PPS edges
#define UBX_DRIFT_SPAN_HALFLIFE 1200           // Drift parameters, s (see DriftEstParams)
#define UBX_DRIFT_SPAN_MIN        60
#define UBX_DRIFT_WINDOW         120
#define UBX_DRIFT_MOVED_PPB     1000
// Drift parameters for GPS PPS
const DriftEstParams UBXDriftParams = { 60, 50000, 0, 1, UBX_DRIFT_WINDOW, UBX_DRIFT_SPAN_MIN,
                                        UBX_DRIFT_SPAN_HALFLIFE, UBX_DRIFT_MOVED_PPB, RTC_DRIFT_RESIDUAL_PPB };

// Gen 9 receivers send no PPS without fix: set a pulse width and enable UBX-TIM-TP.
// Written at every init, a receiver reset clears them
#define UBX_KEY_TP_PERIOD_TP1  0x40050002      // CFG-TP-PERIOD_TP1, U4, microseconds
#define UBX_KEY_TP_LEN_TP1     0x40050004      // CFG-TP-LEN_TP1, U4, microseconds
#define UBX_KEY_MSGOUT_TIM_TP  0x2091017e      // CFG-MSGOUT-UBX_TIM_TP_UART1, U1, per epoch
#define UBX_TP_PERIOD_1HZ      1000000         // us
#define UBX_TP_LEN_UNLOCKED    100000          // us, as factory CFG-TP-LEN_LOCK_TP1
#define UBX_TIM_TP_MISSED      3               // Missed TIM-TP messages before its status is dropped
#define UBX_TACC_NO_FIX        4000            // ns, tAcc below this counts as fix when NAV-STATUS is off

/********************************************************************************************\
| *globals
\*********************************************************************************************/

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x31,0x92, //NAV-TIMEUTC off
#ifdef USE_GPS_VELOCITY
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x29, //NAV-VELNED off
#endif  // USE_GPS_VELOCITY

  // Enable UBX
  // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x01,0x00,0x00,0x00,0x00,0x32,0x97, //NAV-TIMEUTC on
#ifdef USE_GPS_VELOCITY
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //NAV-VELNED on
#endif  // USE_GPS_VELOCITY
  // Rate - we will not reset it for the moment after restart
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0xD0,0x07,0x01,0x00,0x01,0x00,0xED,0xBD //(0.5Hz)
};

char       UBX_name[4];

struct UBX_t {
  const char UBX_HEADER[2]        = { 0xB5, 0x62 }; // TODO: Check if we really save space here inside the struct
  const char NAV_POSLLH_HEADER[2] = { 0x01, 0x02 };
  const char NAV_STATUS_HEADER[2] = { 0x01, 0x03 };
  const char NAV_TIME_HEADER[2]   = { 0x01, 0x21 };
  const char TIM_TP_HEADER[2]     = { 0x0D, 0x01 };
#ifdef USE_GPS_VELOCITY
  const char NAV_VEL_HEADER[2]    = { 0x01, 0x12 };
#endif  // USE_GPS_VELOCITY

  int32_t lat;
  int32_t lon;

  struct entry_t {
    int32_t lat;               // raw sensor value
    int32_t lon;               // raw sensor value
    uint32_t time;             // local time from system (maybe provided by the sensor)
  };

  union {
    entry_t values;
    uint8_t bytes[sizeof(entry_t)];
  } rec_buffer;

  struct POLL_MSG {
    uint8_t cls;
    uint8_t id;
    uint16_t zero;
  };

  struct NAV_POSLLH {
    uint8_t cls;               // 0x01
    uint8_t id;                // 0x02
    uint16_t len;              // 28 bytes
    uint32_t iTOW;             // ms
    int32_t lon;               // 1e-7, degree
    int32_t lat;               // 1e-7, degree
    int32_t alt;               // mm
    int32_t hMSL;              // mm
    uint32_t hAcc;             // mm
    uint32_t vAcc;             // mm
  };

  struct NAV_STATUS {
    uint8_t cls;               // 0x01
    uint8_t id;                // 0x03
    uint16_t len;              // 16 bytes
    uint32_t iTOW;             // ms
    uint8_t gpsFix;            //
    uint8_t flags;             // bit 0 - gpsfix valid
    uint8_t fixStat;           //
    uint8_t flags2;            //
    uint32_t ttff;             // ms
    uint32_t msss;             // ms
  };

  struct NAV_TIME_UTC {
    uint8_t cls;               // 0x01
    uint8_t id;                // 0x21
    uint16_t len;              // 20 bytes
    uint32_t iTOW;             // ms
    uint32_t tAcc;             // ns
    int32_t nano;              // Nanoseconds of second, range -1e9 .. 1e9 (UTC)
    uint16_t year;             // y
    uint8_t month;             // month
    uint8_t day;               // d
    uint8_t hour;              // h
    uint8_t min;               // min
    uint8_t sec;               // s
    struct {
      uint8_t TOW:1;           // time of week
      uint8_t WKN:1;           // week number
      uint8_t UTC:1;           // leap seconds known
      uint8_t reserved:1;
      uint8_t utcStandard:4;
    } valid;
  };

  struct TIM_TP {
    uint8_t cls;               // 0x0D
    uint8_t id;                // 0x01
    uint16_t len;              // 16 bytes
    uint32_t towMS;            // ms, time pulse time of week
    uint32_t towSubMS;         // 2^-32 ms
    int32_t qErr;              // ps, quantization error of the pulse
    uint16_t week;             // time pulse week number
    struct {
      uint8_t timeBase:1;      //
      uint8_t utc:1;           //
      uint8_t raim:2;          //
      uint8_t qErrInvalid:1;   // qErr not valid
      uint8_t TpNotLocked:1;   // not on M9, it stops sending the message
      uint8_t padding:2;
    } flags;
    uint8_t refInfo;           //
  };

#ifdef USE_GPS_VELOCITY
  struct NAV_VEL {
    uint8_t cls;               // 0x01
    uint8_t id;                // 0x12
    uint16_t len;              // 36 bytes
    uint32_t iTOW;             // ms
    int32_t velN;              // cm/s
    int32_t velE;              // cm/s
    int32_t velD;              // cm/s
    uint32_t speed;            // cm/s
    uint32_t gSpeed;           // cm/s
    int32_t heading;           // 1e-5, degree
    uint32_t sAcc;             // cm/s
    uint32_t cAcc;             // 1e-5, degree
  };
#endif  // USE_GPS_VELOCITY

  struct CFG_RATE {
    uint8_t cls;               // 0x06
    uint8_t id;                // 0x08
    uint16_t len;              // 6 bytes
    uint16_t measRate;         // in every ms -> 1 Hz = 1000 ms; 10 Hz = 100 ms -> x = 1000 ms / Hz
    uint16_t navRate;          //  x measurements for 1 navigation event
    uint16_t timeRef;          //  align to time system: 0= UTC, 1 = GPS, 2 = GLONASS, ...
    char CK[2];                // checksum
  };

  struct {
    uint32_t last_iTOW;        // iTOW of the previous NAV-TIMEUTC
    uint32_t last_msg_millis;  // millis() of its arrival
    uint8_t  msgStreak;        // Messages on time, max UBX_MSG_STREAK
    int32_t last_alt;
    uint32_t last_hAcc;
    uint32_t last_vAcc;
    uint8_t gpsFix;
    uint8_t non_empty_loops;   // in case of an unintended reset of the GPS, the serial interface will get flooded with NMEA
    uint16_t log_interval;     // in tenth of seconds
    uint32_t ppsCount;         // Number of PPS edges
    uint32_t ppsPrevMicros;    // micros() of the previous PPS edge
    uint32_t ppsPrevCount;     // ppsCount at ppsPrevMicros
    uint32_t ppsEdgeMicros;    // micros() of the last good PPS edge
    uint32_t ppsEdgeMillis;    // millis() of the same edge
    uint32_t ppsEdgeCount;     // ppsCount of the same edge
    uint8_t  carryMismatch;    // PPS lost: messages in a row naming another second than the clock
    uint32_t timTpMillis;      // millis() of the last UBX-TIM-TP
    uint32_t measRate;         // ms between measurements
    DriftEst drift;            // Crystal drift against PPS
    uint32_t tAcc;             // ns, time accuracy from NAV-TIMEUTC
    uint8_t ppsGood;           // Good edges in a row, max UBX_PPS_SETTLE
    uint32_t boundSecond;      // UTC of the PPS edge at boundCount, 0 = none (PPS without messages)
    uint32_t boundCount;       // ppsCount of that edge
    uint32_t pairKey;          // NAV-TIMEUTC second minus ppsCount, constant while both match
  } state;

  struct {
    uint32_t init:1;
    uint32_t filter_noise:1;
    uint32_t send_when_new:1;  // no teleinterval
    uint32_t send_UI_only:1;
    uint32_t runningNTP:1;
    // uint32_t blockedNTP:1;
    uint32_t forceUTCupdate:1;
    uint32_t runningVPort:1;
    uint32_t hasPPS:1;         // PPS pin configured
    uint32_t ppsLocked:1;      // Last message was timed by a PPS edge
    uint32_t keepPosition:1;   // Keep position/velocity messages while NTP runs
    uint32_t forcePosition:1;  // keepPosition set by command
    uint32_t hasTimTp:1;       // Receiver sends UBX-TIM-TP
    uint32_t tpFree:1;         // Last TpNotLocked
    uint32_t tpAligned:1;      // PPS was on the GNSS second since the last reset
    // TODO: more to come
  } mode;

  union {
    NAV_POSLLH navPosllh;
    NAV_STATUS navStatus;
    NAV_TIME_UTC navTime;
    TIM_TP timTp;
#ifdef USE_GPS_VELOCITY
    NAV_VEL navVel;
#endif  // USE_GPS_VELOCITY
    POLL_MSG pollMsg;
    CFG_RATE cfgRate;
  } Message;

  uint32_t utc_time;

  uint8_t TCPbuf[UBX_SERIAL_BUFFER_SIZE];
  size_t TCPbufSize;
} UBX;

enum UBXMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_TIME,
  MT_TIM_TP,
#ifdef USE_GPS_VELOCITY
  MT_NAV_VEL,
#endif  // USE_GPS_VELOCITY
  MT_POLL
};

#ifdef USE_FLOG
FLOG *Flog = nullptr;
#endif  // USE_FLOG
TasmotaSerial *UBXSerial;

NtpServer timeServer(PortUdp);

WiFiServer vPortServer(UBX_TCP_PORT);
WiFiClient vPortClient;

/*********************************************************************************************\
 * helper function
\*********************************************************************************************/

void UBXcalcChecksum(char* CK, size_t msgSize)
{
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((char*)(&UBX.Message))[i];
    CK[1] += CK[0];
  }
}

bool UBXcompareMsgHeader(const char* msgHeader)
{
  char* ptr = (char*)(&UBX.Message);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

// Send a UBX frame msg[0..len-1] followed by its checksum
void UBXsendChecked(const uint8_t* msg, uint32_t len)
{
  uint8_t ck_a = 0, ck_b = 0;
  for (uint32_t i = 2; i < len; i++) {
    ck_a += msg[i];
    ck_b += ck_a;
  }
  UBXSerial->write(msg, len);
  UBXSerial->write(ck_a);
  UBXSerial->write(ck_b);
}

void UBXsetCfgVal(uint32_t key, uint32_t value)
{
  uint32_t bytes = (((key >> 28) & 0x07) == 4) ? 4 : 1;    // Value size from the key
  uint8_t msg[18] = { 0xB5, 0x62, 0x06, 0x8A, 0x00, 0x00,  // header, class/id, length below
                      0x00, 0x01, 0x00, 0x00 };            // Version 0, layer 1 = RAM
  msg[4] = 8 + bytes;                                      // Payload: 4 header + 4 key + value
  for (uint32_t i = 0; i < 4; i++) { msg[10 + i] = (key >> (8 * i)) & 0xFF; }      // Little endian
  for (uint32_t i = 0; i < bytes; i++) { msg[14 + i] = (value >> (8 * i)) & 0xFF; }
  UBXsendChecked(msg, 14 + bytes);
  DEBUG_SENSOR_LOG(PSTR("UBX: CFG-VALSET key 0x%08X = %u"), key, value);
}

void UBXinitCFG(void)
{
  for (uint32_t i = 0; i < sizeof(UBLOX_INIT); i++) {
    UBXSerial->write( pgm_read_byte(UBLOX_INIT+i) );
  }
  if (UBX.mode.hasPPS) {
    UBXsetCfgVal(UBX_KEY_TP_PERIOD_TP1, UBX_TP_PERIOD_1HZ);
    UBXsetCfgVal(UBX_KEY_TP_LEN_TP1, UBX_TP_LEN_UNLOCKED);
    UBXsetCfgVal(UBX_KEY_MSGOUT_TIM_TP, 1);
    UBX.mode.hasTimTp = 0;              // Wait for TIM-TP with the new config
  }
  DEBUG_SENSOR_LOG(PSTR("UBX: turn off NMEA"));
}

void UBXsendCFGLine(uint8_t _line)
{
  if (_line>sizeof(UBLOX_INIT)/16) return;
  for (uint32_t i = 0; i < 16; i++) {
    UBXSerial->write( pgm_read_byte(UBLOX_INIT+i+(_line*16)) );
  }
  DEBUG_SENSOR_LOG(PSTR("UBX: send line %u of UBLOX_INIT"), _line);
}

/********************************************************************************************/

volatile uint32_t UBXppsMicros = 0;
volatile uint32_t UBXppsCount = 0;

static void IRAM_ATTR UBXppsIsr(void) {
  UBXppsMicros = micros();
  UBXppsCount++;
}

bool UBXHasPositionFix(void)
{
  return (UBX.state.gpsFix > 1) && (UBX.state.gpsFix != 5);
}

uint8_t UBXTimeSourceFix(void)
{
  return UBX.mode.init ? (UBX.state.gpsFix & 0x07) : 7;
}

bool UBXPpsExpected(void)
{
  return UBX.mode.init && UBX.mode.hasPPS && !Settings->sbflag1.gps_pps_off;
}

uint8_t UBXTimeSourcePps(void)
{
  if (!UBX.mode.init) { return 0; }
  return (UBXPpsExpected() ? 1 : 0) | (UBX.mode.ppsLocked ? 2 : 0);
}

bool UBXTimTpFresh(void)
{
  if (!UBX.mode.hasTimTp) { return false; }
  uint32_t interval = (UBX.state.measRate > 1000) ? UBX.state.measRate : 1000;
  return (millis() - UBX.state.timTpMillis) < (interval * UBX_TIM_TP_MISSED);
}

bool UBXPulseSteered(void)
{
  return UBXTimTpFresh() && !UBX.mode.tpFree;
}

// New PPS edge: the PPS edge gives the start of the second, the message gives the second
void UBXHandlePPS(void)
{
  if (!UBXPpsExpected()) { return; }
  uint32_t edge = UBXppsMicros;
  uint32_t count = UBXppsCount;
  if (edge != UBXppsMicros) { return; }          // New edge during the read
  if (count == UBX.state.ppsCount) { return; }   // No new edge
  UBX.state.ppsCount = count;

  // Ignore an edge seen late (e.g. during a flash write)
  uint32_t spacing = edge - UBX.state.ppsPrevMicros;
  uint32_t pulses = count - UBX.state.ppsPrevCount;
  bool spaced_right = UBX.state.ppsPrevMicros &&
                      RtcEdgeSpacedRight(spacing, pulses, Rtc.drift_ppb, UBX_PPS_MAX_SKIP, UBX_PPS_SPACING_TOL);
  UBX.state.ppsPrevMicros = edge;
  UBX.state.ppsPrevCount = count;
  if (!spaced_right) {
    UBX.state.ppsGood = 0;
    return;
  }
  if (UBX.state.ppsGood < UBX_PPS_SETTLE) { UBX.state.ppsGood++; return; }

  uint32_t now_ms = millis();
  uint32_t age_us = micros() - edge;
  if (age_us > 1000000) { return; }              // Too old
  uint32_t prev_edge = UBX.state.ppsEdgeMicros;
  UBX.state.ppsEdgeMicros = edge;
  UBX.state.ppsEdgeCount = count;
  UBX.state.ppsEdgeMillis = now_ms - (age_us / 1000);

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("UBX: EDGE pc=%u eus=%u ems=%u age=%u fix=%d ppb=%d"),
    UBX.state.ppsCount, edge, UBX.state.ppsEdgeMillis, age_us, UBX.state.gpsFix, Rtc.drift_ppb);

  if (!UBX.mode.ppsLocked) {
    UBX.mode.ppsLocked = 1;
    AddLog(LOG_LEVEL_INFO, PSTR("UBX: PPS locked"));
  }
  UBXMeasureDrift(edge - prev_edge);
  UBXPublishNamedEdge();
}

// Set the clock from a PPS edge. named_elsewhere: the second came from the RTC chip or NTP
bool UBXPublishEdge(uint32_t second, uint8_t stratum, bool named_elsewhere)
{
  return RtcSetBoundary(second, UBX.state.ppsEdgeMillis, UBX.state.ppsEdgeMicros,
                        stratum, 0, RTC_PHASE_EDGE, true, named_elsewhere);
}

// PPS without NAV-TIMEUTC: take the second from the clock once, within 250 ms, then count PPS edges
void UBXPublishNamedEdge(void)
{
  if (UBX.state.last_msg_millis &&
      ((millis() - UBX.state.last_msg_millis) < UBX_NAMED_EDGE_QUIET_MS)) { return; }
  if (Rtc.stratum > RTC_STRATUM_CHIP) { return; }              // No time yet
  if (RTC_PHASE_COLD == Rtc.phase_source) { return; }          // Wait for the RTC chip
  if ((UBX.state.ppsGood < UBX_PPS_SETTLE) || !UBX.mode.tpAligned) { return; }   // PPS not aligned after a receiver reset

  uint32_t since = UBX.state.ppsEdgeMillis - Rtc.millis;
  if (since > (RTC_TICK_MAX_CATCHUP * 1000)) { return; }       // Too old
  uint32_t whole = (since + 500) / 1000;
  int32_t in_second = (int32_t)since - (int32_t)(whole * 1000);  // -500..+500 ms
  if (abs(in_second) > 250) { return; }                        // Not near a whole second

  uint32_t names = Rtc.utc_time + whole;
  uint32_t expected = UBX.state.boundSecond + (UBX.state.ppsCount - UBX.state.boundCount);

  if (!UBX.state.boundSecond || (expected != names)) {
    UBX.state.boundSecond = names;
    UBX.state.boundCount = UBX.state.ppsCount;
    AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: edge bound to %u (pc=%u in=%d ms)"),
      names, UBX.state.ppsCount, (int32_t)in_second);
    return;
  }

  UBXPublishEdge(expected, Rtc.stratum, true);
}

// ESP crystal drift against PPS from the step since the previous good edge. Positive = micros() runs fast
void UBXMeasureDrift(uint32_t step_us)
{
  if (!UBX.state.drift.anchored) {
    DriftAnchor(&UBX.state.drift);
    return;
  }
  uint32_t step_secs = (step_us + 500000) / 1000000;      // Missed edges
  if (DRIFT_RESTART == RtcDriftStep("UBX", RTC_DRIFT_SRC_PPS, &UBX.state.drift, &UBXDriftParams,
                                    step_us, step_secs, Rtc.drift_ppb, nullptr)) { return; }

  if (UBX.state.drift.secs < UBX_DRIFT_SPAN_MIN) { return; }
  uint32_t period = (UBX.state.drift.secs < 300) ? 10 : 60;
  if (0 != (UBX.state.drift.secs % period)) { return; }
  RtcDriftPublish("UBX", RTC_DRIFT_SRC_PPS, &UBX.state.drift, &UBXDriftParams, nullptr);
}

void UBXDetect(void) {
  UBX.mode.init = 0;
  if (!(PinUsed(GPIO_GPS_RX, GPIO_ANY) && PinUsed(GPIO_GPS_TX))) { return; }

  uint32_t option = GetPin(Pin(GPIO_GPS_RX, GPIO_ANY)) - AGPIO(GPIO_GPS_RX);  // 0 .. 2
  uint32_t baudrate = 9600 << option;  // Support 1 (9600), 2 (19200), 3 (38400)
  UBXSerial = new TasmotaSerial(Pin(GPIO_GPS_RX, GPIO_ANY), Pin(GPIO_GPS_TX), 1, 0, UBX_SERIAL_BUFFER_SIZE); // 64 byte buffer is NOT enough
  if (!UBXSerial->begin(baudrate)) { return; }

  if (UBXSerial->hardwareSerial()) {
    ClaimSerial();
  }
#ifdef ESP32
  AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: Serial UART%d"), UBXSerial->getUart());
#endif

  if (PinUsed(GPIO_GPS_PPS)) {
    pinMode(Pin(GPIO_GPS_PPS), INPUT);
    attachInterrupt(digitalPinToInterrupt(Pin(GPIO_GPS_PPS)), UBXppsIsr, RISING);
    UBX.mode.hasPPS = 1;
    AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: PPS on GPIO%d"), Pin(GPIO_GPS_PPS));
  }

  UBXinitCFG();                 // turn off NMEA, only use "our" UBX-messages
  timeServer.setSource(UBXNtpSource, RtcNtpSent);   // Before beginListening()
  UBX.mode.init = 1;

#ifdef USE_FLOG
  if (!Flog) {
    Flog = new FLOG;            // init Flash Log
    Flog->init();
  }
#endif  // USE_FLOG

  UBX.state.log_interval = 10;  // 1 second
  UBX.mode.send_UI_only = true; // send UI data ...
//  MqttPublishTeleperiodSensor();  // ... once at after start (No MQTT ready yet so do NOT try to send)
}

uint32_t UBXprocessGPS()
{
  static uint32_t fpos = 0;
  static char checksum[2];
  static uint8_t currentMsgType = MT_NONE;
  static size_t payloadSize = sizeof(UBX.Message);

  // DEBUG_SENSOR_LOG(PSTR("UBX: check for serial data"));
  uint32_t data_bytes = 0;
  while ( UBXSerial->available() ) {
    data_bytes++;
    byte c = UBXSerial->read();
    if (UBX.mode.runningVPort){
      UBX.TCPbuf[data_bytes-1] = c; // immediately copy byte to TCP-buf
      UBX.TCPbufSize = data_bytes;
    }
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX.UBX_HEADER[fpos] ) {
        fpos++;
      } else {
        fpos = 0; // Reset to beginning state.
      }
    } else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.

      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize ) {
        ((char*)(&UBX.Message))[fpos-2] = c;
      }
      fpos++;

      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header,
        // so now we can check to see what kind of message it is.
        if ( UBXcompareMsgHeader(UBX.NAV_POSLLH_HEADER) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(UBX_t::NAV_POSLLH);
          DEBUG_SENSOR_LOG(PSTR("UBX: got NAV_POSLLH"));
        }
        else if ( UBXcompareMsgHeader(UBX.NAV_STATUS_HEADER) ) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(UBX_t::NAV_STATUS);
          DEBUG_SENSOR_LOG(PSTR("UBX: got NAV_STATUS"));
        }
        else if ( UBXcompareMsgHeader(UBX.NAV_TIME_HEADER) ) {
          currentMsgType = MT_NAV_TIME;
          payloadSize = sizeof(UBX_t::NAV_TIME_UTC);
          DEBUG_SENSOR_LOG(PSTR("UBX: got NAV_TIME_UTC"));
        }
        else if ( UBXcompareMsgHeader(UBX.TIM_TP_HEADER) ) {
          currentMsgType = MT_TIM_TP;
          payloadSize = sizeof(UBX_t::TIM_TP);
          DEBUG_SENSOR_LOG(PSTR("UBX: got TIM_TP"));
        }
#ifdef USE_GPS_VELOCITY
        else if ( UBXcompareMsgHeader(UBX.NAV_VEL_HEADER) ) {
          currentMsgType = MT_NAV_VEL;
          payloadSize = sizeof(UBX_t::NAV_VEL);
          DEBUG_SENSOR_LOG(PSTR("UBX: got NAV_VEL"));
        }
#endif  // USE_GPS_VELOCITY
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the
        // expected checksum value to compare with the next two incoming bytes.
        UBXcalcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0;
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  // DEBUG_SENSOR_LOG(PSTR("UBX: got none or unknown Message"));
  if (data_bytes!=0) {
    UBX.state.non_empty_loops++;
    DEBUG_SENSOR_LOG(PSTR("UBX: got %u bytes, non-empty-loop: %u"), data_bytes, UBX.state.non_empty_loops);
  } else {
    UBX.state.non_empty_loops = 0; // now a hidden GPS-device reset is unlikely
  }
  return MT_NONE;
}

/********************************************************************************************\
| * callback functions for the download
\*********************************************************************************************/

#ifdef USE_FLOG
void UBXsendHeader(void)
{
  Webserver->setContentLength(CONTENT_LENGTH_UNKNOWN);
  Webserver->sendHeader(F("Content-Disposition"), F("attachment; filename=TASMOTA.gpx"));
  WSSend(200, CT_APP_STREAM, F(
    "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\r\n"
    "<GPX version=\"1.1\" creator=\"TASMOTA\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
    "<trk>\r\n<trkseg>\r\n"));
}

void UBXsendRecord(uint8_t *buf)
{
	char record[100];
	char stime[32];
	UBX_t::entry_t *entry = (UBX_t::entry_t*)buf;
	snprintf_P(stime, sizeof(stime), GetDT(entry->time).c_str());
	char lat[FLOATSZ];
	char lon[FLOATSZ];
	dtostrfd((double)entry->lat/10000000.0f,7,lat);
	dtostrfd((double)entry->lon/10000000.0f,7,lon);
	snprintf_P(record, sizeof(record),PSTR("<trkpt\n\t lat=\"%s\" lon=\"%s\">\n\t<time>%s</time>\n</trkpt>\n"),lat ,lon, stime);
	// DEBUG_SENSOR_LOG(PSTR("FLOG: DL %u %u"), Flog->sector.dword_buffer[k+j],Flog->sector.dword_buffer[k+j+1]);
	Webserver->sendContent_P(record);
}

void UBXsendFooter(void)
{
  Webserver->sendContent(F("</trkseg>\n</trk>\n</GPX>"));
  Webserver->sendContent("");
  UBX.utc_time = 0;            // we have blocked the main loop and want a new valid time
}

/********************************************************************************************/

void UBXsendFile(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }
  Flog->startDownload(sizeof(UBX.rec_buffer),UBXsendHeader,UBXsendRecord,UBXsendFooter);
}
#endif  // USE_FLOG

/********************************************************************************************/

void UBXSetRate(uint16_t interval)
{
  UBX.Message.cfgRate.cls = 0x06;
  UBX.Message.cfgRate.id = 0x08;
  UBX.Message.cfgRate.len = 6;
  uint32_t measRate = (1000*(uint32_t)interval); //seconds to milliseconds
  if (measRate > 0xffff) {
    measRate = 0xffff; // max. 65535 ms interval
  }
  UBX.Message.cfgRate.measRate = (uint16_t)measRate;
  UBX.state.measRate = measRate;             // Used by UBXTimTpFresh()
  UBX.Message.cfgRate.navRate = 1;
  UBX.Message.cfgRate.timeRef = 1;
  UBXcalcChecksum(UBX.Message.cfgRate.CK, sizeof(UBX.Message.cfgRate)-sizeof(UBX.Message.cfgRate.CK));
  DEBUG_SENSOR_LOG(PSTR("UBX: requested interval: %u seconds measRate: %u ms"), interval, UBX.Message.cfgRate.measRate);
  UBXSerial->write(UBX.UBX_HEADER[0]);
  UBXSerial->write(UBX.UBX_HEADER[1]);
  for (uint32_t i =0; i<sizeof(UBX.Message.cfgRate); i++) {
    UBXSerial->write(((uint8_t*)(&UBX.Message.cfgRate))[i]);
    DEBUG_SENSOR_LOG(PSTR("UBX: cfgRate byte %u: %x"), i, ((uint8_t*)(&UBX.Message.cfgRate))[i]);
  }
  UBX.state.log_interval = 10*interval;
}

bool UBXKeepPosition(void)
{
  return UBX.mode.forcePosition ? UBX.mode.keepPosition : (bool)UBX.mode.hasPPS;
}

void UBXPositionMessages(bool on)
{
  if (on) {
#ifdef USE_GPS_VELOCITY
    UBXsendCFGLine(11); //NAV-POSLLH on
    UBXsendCFGLine(12); //NAV-STATUS on
    UBXsendCFGLine(14); //NAV-VELNED on
#else
    UBXsendCFGLine(10); //NAV-POSLLH on
    UBXsendCFGLine(11); //NAV-STATUS on
#endif  // USE_GPS_VELOCITY
  } else {
    UBXsendCFGLine(7);  // NAV-POSLLH off
    UBXsendCFGLine(8);  // NAV-STATUS off
#ifdef USE_GPS_VELOCITY
    UBXsendCFGLine(10); // NAV-VELNED off
#endif  // USE_GPS_VELOCITY
  }
}

void UBXSelectMode(uint16_t mode)
{
  DEBUG_SENSOR_LOG(PSTR("UBX: set mode to %u"),mode);
  switch(mode){
#ifdef USE_FLOG
    case 0:
      Flog->mode = 0; // write once to all available sectors, then stop
      break;
    case 1:
      Flog->mode = 1; // write to all available sectors, then restart and overwrite the older ones
      break;
    case 2:
      UBX.mode.filter_noise = true;  // filter out horizontal drift noise, TODO: find useful values
      break;
    case 3:
      UBX.mode.filter_noise = false;
      break;
    case 4:
      Flog->startRecording(true);
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: start recording - appending"));
      break;
    case 5:
      Flog->startRecording(false);
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: start recording - new log"));
      break;
    case 6:
      if(Flog->recording == true){
        Flog->stopRecording();
      }
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: stop recording"));
      break;
#endif  // USE_FLOG
    case 7:
      UBX.mode.send_when_new = 1; // send mqtt on new postion + TELE -> consider to set TELE to a very high value
      break;
    case 8:
      UBX.mode.send_when_new = 0; // only TELE
      break;
    case 9:
      if (!TasmotaGlobal.global_state.network_down) {
        UBXListenNTP();
        UBX.mode.runningNTP = true;
        Settings->sbflag1.gps_ntp_server = 1;
        if (UBXKeepPosition()) { UBXPositionMessages(true); }
      }
      break;
    case 10:
      UBX.mode.runningNTP = false;
      Settings->sbflag1.gps_ntp_server = 0;
      RtcReleaseSource(RTC_STRATUM_GPS_FREE);
      if (RtcNtpBoundBy(RTC_NTP_OWNER_GPS)) {
        timeServer.stopListening();
        RtcNtpRelease(RTC_NTP_OWNER_GPS);
      }
      UBXPositionMessages(true);
      break;
    case 11:
      UBX.mode.forceUTCupdate = true;
      break;
    case 12:
      UBX.mode.forceUTCupdate = false;
      break;
    case 13:
      Settings->latitude = UBX.rec_buffer.values.lat/10;
      Settings->longitude = UBX.rec_buffer.values.lon/10;
      break;
    case 14:
      vPortServer.begin();
      UBX.mode.runningVPort = 1;
      break;
    case 15:
      // vPortServer.stop(); // seems not to work reliably
      UBX.mode.runningVPort = 0;
      break;
    case 18:
      UBX.mode.keepPosition = 1;  // Keep position/velocity while NTP runs
      UBX.mode.forcePosition = 1;
      UBXPositionMessages(true);
      break;
    case 19:
      UBX.mode.keepPosition = 0;  // Pure NTP mode, no position/velocity
      UBX.mode.forcePosition = 1;
      if (UBX.mode.runningNTP) { UBXPositionMessages(false); }
      break;
    default:
      if (mode>1000 && mode <1066) {
        UBXSetRate(mode-1000); // set interval between measurements in seconds from 1 to 65
      }
      break;
  }
  UBX.mode.send_UI_only = true;
  MqttPublishTeleperiodSensor();
}

/********************************************************************************************/

bool UBXHandlePOSLLH()
{
  DEBUG_SENSOR_LOG(PSTR("UBX: iTOW: %u"),UBX.Message.navPosllh.iTOW);
  if (UBX.state.gpsFix>1) {
    if (UBX.mode.filter_noise) {
      if ((abs(UBX.Message.navPosllh.lat-UBX.rec_buffer.values.lat)<UBX_LAT_LON_THRESHOLD)||(abs(UBX.Message.navPosllh.lon-UBX.rec_buffer.values.lon)<UBX_LAT_LON_THRESHOLD)) {
        DEBUG_SENSOR_LOG(PSTR("UBX: Diff lat: %u lon: %u "),UBX.Message.navPosllh.lat-UBX.rec_buffer.values.lat, UBX.Message.navPosllh.lon-UBX.rec_buffer.values.lon);
        return false; //no new position
      }
    }
    UBX.rec_buffer.values.lat = UBX.Message.navPosllh.lat;
    UBX.rec_buffer.values.lon = UBX.Message.navPosllh.lon;
    DEBUG_SENSOR_LOG(PSTR("UBX: lat/lon: %i / %i"), UBX.rec_buffer.values.lat, UBX.rec_buffer.values.lon);
    DEBUG_SENSOR_LOG(PSTR("UBX: hAcc: %d"), UBX.Message.navPosllh.hAcc);
    UBX.state.last_alt = UBX.Message.navPosllh.alt;
    UBX.state.last_vAcc = UBX.Message.navPosllh.vAcc;
    UBX.state.last_hAcc = UBX.Message.navPosllh.hAcc;
    if (UBX.mode.send_when_new) {
      MqttPublishTeleperiodSensor();
    }
    if (UBX.mode.runningNTP && !UBXKeepPosition()){ // -> go to pure NTP-mode
      UBXPositionMessages(false);
    }
    //UBX_LAT_LON_THRESHOLD = 20 * UBX.Message.navPosllh.hAcc;
    return true; // new position
  } else {
    DEBUG_SENSOR_LOG(PSTR("UBX: no valid position data"));
  }
  return false; // no GPS-fix
}

#ifdef USE_GPS_VELOCITY
void UBXHandleVEL()
  {
    DEBUG_SENSOR_LOG(PSTR("UBX: iTOWvel: %u"),UBX.Message.navVel.iTOW);
    if (UBX.state.gpsFix>1) {
      DEBUG_SENSOR_LOG(PSTR("UBX: speed: %d"), UBX.Message.navVel.gSpeed);
      DEBUG_SENSOR_LOG(PSTR("UBX: heading: %i"), UBX.Message.navVel.heading);
      DEBUG_SENSOR_LOG(PSTR("UBX: spd accuracy: %i"), UBX.Message.navVel.sAcc);
      DEBUG_SENSOR_LOG(PSTR("UBX: hdng accuracy: %i"), UBX.Message.navVel.cAcc);
    }
  
}
#endif  // USE_GPS_VELOCITY

void UBXHandleSTATUS()
{
  DEBUG_SENSOR_LOG(PSTR("UBX: gpsFix: %u, valid: %u"), UBX.Message.navStatus.gpsFix, (UBX.Message.navStatus.flags)&1);
  if ((UBX.Message.navStatus.flags)&1) {
    UBX.state.gpsFix = UBX.Message.navStatus.gpsFix; //only store fixed status if flag is valid
  } else {
    UBX.state.gpsFix = 0; // without valid flag, everything is "no fix"
  }
}

void UBXHandleTIME()
{
  DEBUG_SENSOR_LOG(PSTR("UBX: UTC-Time: %u-%u-%u %u:%u:%u"), UBX.Message.navTime.year, UBX.Message.navTime.month ,UBX.Message.navTime.day,UBX.Message.navTime.hour,UBX.Message.navTime.min,UBX.Message.navTime.sec);
  static int8_t utc_said = -1;
  int8_t utc_now = UBX.Message.navTime.valid.UTC;
  if (UBX.Message.navTime.valid.TOW && (utc_now != utc_said)) {
    AddLog(LOG_LEVEL_INFO, PSTR("UBX: leap seconds %s"), utc_now ? PSTR("known, UTC valid")
      : PSTR("not yet known, UTC second not used"));
    utc_said = utc_now;
  }
  if (!utc_now) { UBX.mode.tpAligned = 0; }     // Receiver reset, PPS not aligned
 // validUTC (bit 2): leap seconds known
 if ((UBX.Message.navTime.valid.UTC == 1) && (UBX.Message.navTime.year >= 2023)) {
    uint32_t arrival = micros();  // iTOW%1000 should be 0 here, when NTP-server is enabled and in "pure mode"
    UBX.state.tAcc = UBX.Message.navTime.tAcc;   // For the UI
    if (!UBXKeepPosition()) {                     // NAV-STATUS is off: use tAcc as fix
      bool solving = (UBX.Message.navTime.tAcc < UBX_TACC_NO_FIX);
      if (solving != (UBX.state.gpsFix > 1)) {
        UBX.state.gpsFix = solving ? 5 : 0;
        AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: tAcc %u ns, fix %s"),
          UBX.Message.navTime.tAcc, solving ? PSTR("acquired") : PSTR("lost"));
      }
    }
    if ((UBX.state.gpsFix > 1) && (UBX.Message.navTime.tAcc < UBX_TACC_NO_FIX)) { UBX.mode.tpAligned = 1; }
    DEBUG_SENSOR_LOG(PSTR("UBX: UTC-Time is valid"));
    bool resync = (Rtc.utc_time > UBX.utc_time);  // Sync local time every hour
    bool clock_unset = (Rtc.utc_time < START_VALID_TIME);
    // Without the NTP server the GPS sets the clock hourly, unless a fresh upstream sync holds it
    bool hourly = resync && !UBX.mode.runningNTP &&
                  !(Rtc.from_upstream && ((TasmotaGlobal.uptime - Rtc.last_sync) <= RTC_SYNC_AGE_UPSTREAM));
    if (UBX.mode.runningNTP || UBX.mode.forceUTCupdate || clock_unset || hourly) {
      TIME_T gpsTime;
      gpsTime.year = UBX.Message.navTime.year - 1970;
      gpsTime.month = UBX.Message.navTime.month;
      gpsTime.day_of_month = UBX.Message.navTime.day;
      gpsTime.hour = UBX.Message.navTime.hour;
      gpsTime.minute = UBX.Message.navTime.min;
      gpsTime.second = UBX.Message.navTime.sec;
      UBX.rec_buffer.values.time = MakeTime(gpsTime);
      // The PPS edge belongs to this message if it came UBX_PPS_MIN_AGE..UBX_PPS_MAX_AGE ms before
      bool edge_seen = (0 != UBX.state.ppsEdgeMicros);
      int32_t edge_age = (int32_t)(arrival - UBX.state.ppsEdgeMicros) / 1000;
      bool edge_matches = edge_seen &&
                          (edge_age >= UBX_PPS_MIN_AGE) && (edge_age <= UBX_PPS_MAX_AGE);

      // New message or an old one from the buffer? Compare iTOW steps with arrival steps
      uint32_t itow = UBX.Message.navTime.iTOW;
      uint32_t now_ms = millis();
      int32_t itow_step = (int32_t)(itow - UBX.state.last_iTOW);
      int32_t real_step = (int32_t)(now_ms - UBX.state.last_msg_millis);
      bool rate_ok = (0 != UBX.state.last_iTOW) &&
                     (abs(itow_step - real_step) < UBX_MSG_FRESH_MS);
      if (rate_ok) {
        if (UBX.state.msgStreak < UBX_MSG_STREAK) { UBX.state.msgStreak++; }
      } else {
        UBX.state.msgStreak = 0;
      }
      bool fresh_msg = rate_ok && (edge_seen || (UBX_MSG_STREAK == UBX.state.msgStreak));
      UBX.state.last_iTOW = itow;
      UBX.state.last_msg_millis = now_ms;
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("UBX: MEAS pc=%u eus=%u ems=%u utc=%u itow=%u aus=%u "
                                  "nano=%d tacc=%u lock=%d fix=%d ppb=%d"),
        UBX.state.ppsCount, UBX.state.ppsEdgeMicros, UBX.state.ppsEdgeMillis,
        UBX.rec_buffer.values.time, UBX.Message.navTime.iTOW, arrival,
        UBX.Message.navTime.nano, UBX.Message.navTime.tAcc,
        UBX.mode.ppsLocked, UBX.state.gpsFix, Rtc.drift_ppb);

      // PPS locked to GNSS: TIM-TP if available, else a position fix
      bool traceable = UBX.mode.hasTimTp ? UBXPulseSteered() : UBXHasPositionFix();

      // Message second minus PPS count is constant while both match
      uint32_t pair_key = UBX.rec_buffer.values.time - UBX.state.ppsEdgeCount;
      bool pair_ok = (pair_key == UBX.state.pairKey);
      if (edge_matches && !pair_ok) {
        if (rate_ok) {
          if (UBX.state.pairKey) {
            AddLog(LOG_LEVEL_INFO, PSTR("UBX: message and pulse re-paired, second-count %d s"),
              (int32_t)(pair_key - UBX.state.pairKey));
          }
          UBX.state.pairKey = pair_key;
          pair_ok = true;
        } else {
          AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: late message %u not paired with edge pc=%u, age %d ms"),
            UBX.rec_buffer.values.time, UBX.state.ppsCount, edge_age);
        }
      }

      bool published = false;
      if (UBX.mode.ppsLocked && UBX.mode.tpAligned && edge_matches && pair_ok) {
        published = UBXPublishEdge(UBX.rec_buffer.values.time,
                                   traceable ? RTC_STRATUM_GPS : RTC_STRATUM_GPS_PPS, false);
      }
      // No matching PPS edge: the clock carries the last edge, the message names the second; else its arrival
      if (!published && !edge_matches && fresh_msg) {
        uint8_t stratum = traceable ? RTC_STRATUM_GPS : RTC_STRATUM_GPS_FREE;
        if (edge_seen && RtcPhaseGood()) {
          int8_t named = RtcCarryNamed(UBX.rec_buffer.values.time, arrival, stratum,
                                       (millis() - UBX.state.ppsEdgeMillis) / 1000, UBX.state.carryMismatch >= 4);
          published = (named > 0);
          if (named < 0) {
            UBX.state.carryMismatch++;
            AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: message %u not on the carried second"), UBX.rec_buffer.values.time);
          } else if (published && UBX.state.carryMismatch >= 4) {
            AddLog(LOG_LEVEL_INFO, PSTR("UBX: carried second renamed to %u"), UBX.rec_buffer.values.time);
          }
          if (published) { UBX.state.carryMismatch = 0; }
        } else {
          uint32_t b_ms = millis(), b_us = arrival;
#ifdef USE_RTC_CHIP_HOLDOVER
          uint32_t chip_err = 0;
          bool chip_placed = !edge_seen && RtcChipPlaceSecond(UBX.rec_buffer.values.time, arrival, &b_ms, &b_us, &chip_err);
#endif  // USE_RTC_CHIP_HOLDOVER
          published = RtcSetBoundary(UBX.rec_buffer.values.time, b_ms, b_us, stratum, 2, RTC_PHASE_NONE, true);
#ifdef USE_RTC_CHIP_HOLDOVER
          if (published && chip_placed) {
            RTC_PAIR_LOCK();
            Rtc.chip_placed_us = chip_err;
            RTC_PAIR_UNLOCK();
          }
#endif  // USE_RTC_CHIP_HOLDOVER
        }
      } else if (!published) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("UBX: no boundary this second, age %d"), edge_age);
      }
      if (UBX.mode.forceUTCupdate || clock_unset || resync) {
//        AddLog(LOG_LEVEL_INFO, PSTR("UBX: UTC-Time is valid, set system time"));
        UBX.utc_time = UBX.rec_buffer.values.time + 3600;
        // Whole seconds only if the clock is not set, or hourly without the NTP server
        if (published) {
          RtcSynced("UBX");
        } else if (!UBX.mode.ppsLocked && (fresh_msg || (hourly && rate_ok)) && (clock_unset || UBX.mode.forceUTCupdate || hourly)) {
          RtcSync("UBX", UBX.rec_buffer.values.time,
                  traceable ? RTC_STRATUM_GPS : RTC_STRATUM_GPS_FREE,
                  RTC_PHASE_NONE, RTC_SYNC_AGE_GPS);
        }
      }
    }
  }
}

void UBXHandleTIMTP(void)
{
  UBX.mode.hasTimTp = 1;
  UBX.state.timTpMillis = millis();
  bool was_free = UBX.mode.tpFree;
  UBX.mode.tpFree = UBX.Message.timTp.flags.TpNotLocked;
  if (was_free != (bool)UBX.mode.tpFree) {
    AddLog(LOG_LEVEL_INFO, PSTR("UBX: time pulse %s"),
      UBX.mode.tpFree ? PSTR("free running on the receiver clock") : PSTR("locked to GNSS"));
  }
  if (!UBX.mode.tpFree) { UBX.mode.tpAligned = 1; }
}

void UBXHandleOther(void)
{
  if (UBX.state.non_empty_loops>6) {  // we expect only 4-5 non-empty loops in a row, could change with other sensor speed (Hz)
    if(UBX.mode.runningVPort) return;
    UBXinitCFG();                     // this should only happen with lots of NMEA-messages, but it is only a guess!!
    AddLog(LOG_LEVEL_ERROR, PSTR("UBX: possible device-reset, will re-init"));
    UBXSerial->flush();
    UBX.state.non_empty_loops = 0;
  }
}

/********************************************************************************************/

void UBXLoop50msec(void)
{
  // handle virtual serial port
  if (UBX.mode.runningVPort){
    if(!vPortClient.connected()) {
      vPortClient = vPortServer.available();
    }
    while(vPortClient.available()) {
      byte _newByte = vPortClient.read();
      UBXSerial->write(_newByte);
    }

    if (UBX.TCPbufSize!=0){
      vPortClient.write((char*)UBX.TCPbuf, UBX.TCPbufSize);
      UBX.TCPbufSize = 0;
    }
  }
}

// Open NTP port 123 for the GPS server, unless already used. false = failed
bool UBXListenNTP(void)
{
  if (RtcNtpBoundBy(RTC_NTP_OWNER_GPS) || !RtcNtpBind(RTC_NTP_OWNER_GPS)) { return true; }
  if (timeServer.beginListening()) { return true; }
  RtcNtpRelease(RTC_NTP_OWNER_GPS);
  return false;
}

// Every 100 ms while enabled: takes port 123 as soon as the chip server frees it
void UBXRestoreNTP(void)
{
  if (!Settings->sbflag1.gps_ntp_server || RtcNtpBoundBy(RTC_NTP_OWNER_GPS)) { return; }
  if (TasmotaGlobal.global_state.network_down || !UBXListenNTP()) { return; }
  if (UBX.mode.runningNTP) { return; }
  UBX.mode.runningNTP = true;
  if (UBXKeepPosition()) { UBXPositionMessages(true); }
  AddLog(LOG_LEVEL_INFO, PSTR("UBX: NTP server restored"));
}

void UBXDropPPS(const char* why)
{
  if (!UBX.mode.ppsLocked) { return; }
  UBX.mode.ppsLocked = 0;
  UBX.state.ppsGood = 0;
  UBX.state.drift.lostUptime = TasmotaGlobal.uptime;
  AddLog(LOG_LEVEL_INFO, PSTR("UBX: PPS %s, using serial message time"), why);
}

#define UBX_SILENT_MS          3000            // ms without messages before the fix is cleared
void UBXCheckSilent(void)
{
  if (!UBX.state.gpsFix || !UBX.state.last_msg_millis) { return; }
  if ((millis() - UBX.state.last_msg_millis) < UBX_SILENT_MS) { return; }
  UBX.state.gpsFix = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("UBX: receiver silent, fix cleared"));
}

void UBXCheckPPSLost(void)
{
  if (!UBX.mode.ppsLocked) { return; }
  if ((micros() - UBX.state.ppsPrevMicros) < (2 * 1000000U)) { return; }
  UBXDropPPS(PSTR("lost"));
}

bool UBXNtpSource(NtpTimeBase* base, uint32_t* utc, uint32_t* boundary_us)
{
  if (TasmotaGlobal.global_state.network_down || !UBX.mode.runningNTP) { return false; }
  return RtcNtpRequest(RTC_NTP_OWNER_GPS, base, utc, boundary_us);
}

void UBXLoop(void)
{
  static uint16_t counter; //count up every 100 msec
  static bool new_position;

  uint32_t msgType = UBXprocessGPS();

  switch(msgType){
    case MT_NAV_POSLLH:
      new_position = UBXHandlePOSLLH();
      break;
    case MT_NAV_STATUS:
      UBXHandleSTATUS();
      break;
    case MT_NAV_TIME:
      UBXHandleTIME();
      break;
    case MT_TIM_TP:
      UBXHandleTIMTP();
      break;
#ifdef USE_GPS_VELOCITY
    case MT_NAV_VEL:
      UBXHandleVEL();
      break;
#endif  // USE_GPS_VELOCITY
    default:
      UBXHandleOther();
      break;
  }

#ifdef USE_FLOG
  if (counter>UBX.state.log_interval) {
    if (Flog->recording && new_position) {
      UBX.rec_buffer.values.time = Rtc.local_time;
      Flog->addToBuffer(UBX.rec_buffer.bytes, sizeof(UBX.rec_buffer.bytes));
      counter = 0;
    }
  }
#endif  // USE_FLOG

  counter++;
}

/********************************************************************************************/
// normaly in i18n.h

#ifdef USE_WEBSERVER
// {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>

#ifdef USE_FLOG
#ifdef DEBUG_TASMOTA_SENSOR
const char HTTP_SNS_FLOGVER[] PROGMEM = "{s}FLOG with %u sectors:{m}%u bytes{e}"
                                        "{s}FLOG next sector for REC:{m} %u {e}"
                                        "{s}%u sector(s) with data at sector:{m}%u{e}";
const char HTTP_SNS_FLOGREC[] PROGMEM = "{s}RECORDING (bytes in buffer){m}%u{e}";
#endif  // DEBUG_TASMOTA_SENSOR

const char HTTP_SNS_FLOG[] PROGMEM = "{s}GPS Logging{m}%s{e}";
const char kFLOGstate[] PROGMEM = "Ready|Recording";
const char HTTP_BTN_FLOG_DL[] PROGMEM = "<button><a href='/UBX'>Download GPX-File</a></button>";
#endif  // USE_FLOG

const char HTTP_SNS_NTPSERVER[] PROGMEM = "{s}GPS NTP server{m}Active{e}";
const char HTTP_SNS_NTPPPS[] PROGMEM = "{s}GPS NTP server{m}Active (%s){e}";
const char HTTP_SNS_NTPSTATE[] PROGMEM = "{s}NTP stratum{m}%d, drift %d ppb{e}";

const char HTTP_SNS_GPS[] PROGMEM = "{s}GPS " D_SAT_FIX "{m}%s{e}"
                                    "{s}GPS " D_LATITUDE "{m}%s{e}"
                                    "{s}GPS " D_LONGITUDE "{m}%s{e}"
                                    "{s}GPS " D_HORIZONTAL_ACCURACY "{m}%3_f " D_UNIT_METER "{e}"
                                    "{s}GPS " D_ALTITUDE "{m}%3_f " D_UNIT_METER "{e}"
                                    "{s}GPS " D_VERTICAL_ACCURACY "{m}%3_f " D_UNIT_METER "{e}";
#ifdef USE_GPS_VELOCITY
const char HTTP_SNS_GPS2[] PROGMEM = "{s}GPS " D_SPEED "{m}%2_f " D_UNIT_KILOMETER_PER_HOUR "{e}"
                                     "{s}GPS " D_SPEED_ACCURACY "{m}%2_f " D_UNIT_KILOMETER_PER_HOUR "{e}"
                                     "{s}GPS " D_HEADING "{m}%1_f{e}"
                                     "{s}GPS " D_HEADING_ACCURACY "{m}%1_f{e}";
#endif  // USE_GPS_VELOCITY

#ifdef USE_GPS_MAPS
const char UBX_GOOGLE_MAPS[] ="<iframe width='100%%' src='https://maps.google.com/maps?width=&amp;height=&amp;hl=en&amp;q=%s %s+(Tasmota)&amp;ie=UTF8&amp;t=&amp;z=10&amp;iwloc=B&amp;output=embed' frameborder='0' scrolling='no' marginheight='0' marginwidth='0'></iframe>";
#endif  // USE_GPS_MAPS

#endif  // USE_WEBSERVER

const char kGPSFix[] PROGMEM = D_SAT_FIX_NO_FIX "|" D_SAT_FIX_DEAD_RECK "|" D_SAT_FIX_2D "|" D_SAT_FIX_3D "|" D_SAT_FIX_GPS_DEAD "|" D_SAT_FIX_TIME;

/********************************************************************************************/

void UBXShow(bool json) {
  char fix[32];
  GetTextIndexed(fix, sizeof(fix), UBX.state.gpsFix, kGPSFix);
  char lat[FLOATSZ];
  dtostrfd((double)UBX.rec_buffer.values.lat / 10000000.0f, 7, lat);  // degrees
  char lon[FLOATSZ];
  dtostrfd((double)UBX.rec_buffer.values.lon / 10000000.0f, 7, lon);  // degrees
  float hAcc = (float)UBX.state.last_vAcc / 1000.0f;                  // mm -> meters
  float alt = (float)UBX.state.last_alt / 1000.0f;                    // mm -> meters
  float vAcc = (float)UBX.state.last_hAcc / 1000.0f;                  // mm -> meters
#ifdef USE_GPS_VELOCITY
  float spd = (float)UBX.Message.navVel.gSpeed / 27.778f;             // cm/s -> km/h
  float sAcc = (float)UBX.Message.navVel.sAcc / 27.778f;              // cm/s -> km/h
  float hdng = (float)UBX.Message.navVel.heading / 100000.0f;         // degrees
  float cAcc = (float)UBX.Message.navVel.cAcc / 100000.0f;            // degrees
  if (cAcc > 360) { cAcc = 0; }
#endif  // USE_GPS_VELOCITY

  if (json) {
    ResponseAppend_P(PSTR(",\"GPS\":{"));
    if (UBX.mode.send_UI_only) {
      uint32_t i = UBX.state.log_interval / 10;
      ResponseAppend_P(PSTR("\"Fil\":%u,\"Int\":%u}"), UBX.mode.filter_noise, i);
    } else {
      ResponseAppend_P(PSTR("\"Lat\":%s,\"Lon\":%s,\"Alt\":%3_f,\"hAcc\":%3_f,\"vAcc\":%3_f,\"Fix\":\"%s\""),
        lat, lon, &alt, &hAcc, &vAcc, fix);
#ifdef USE_GPS_VELOCITY
      ResponseAppend_P(PSTR(",\"Spd\":%2_f,\"Hdng\":%1_f,\"sAcc\":%2_f,\"cAcc\":%1_f"), 
        &spd, &hdng, &sAcc, &cAcc);
#endif  // USE_GPS_VELOCITY
      ResponseAppend_P(PSTR("}"));
    }
    if (UBX.mode.runningNTP) {
      ResponseAppend_P(PSTR(",\"NTP\":{\"PPS\":%d,\"Steered\":%d,\"Stratum\":%d,\"DriftPpb\":%d}"),
        UBX.mode.ppsLocked, UBXPulseSteered(), Rtc.stratum, Rtc.drift_ppb);
    }
#ifdef USE_FLOG
    ResponseAppend_P(PSTR(",\"FLOG\":{\"Rec\":%u,\"Mode\":%u,\"Sec\":%u}"), Flog->recording, Flog->mode, Flog->sectors_left);
#endif  // USE_FLOG
    UBX.mode.send_UI_only = false;
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SNS_GPS, fix, lat, lon, &hAcc, &alt, &vAcc);
#ifdef USE_GPS_VELOCITY
    WSContentSend_PD(HTTP_SNS_GPS2, &spd, &sAcc, &hdng, &cAcc);
#endif  // USE_GPS_VELOCITY

#ifdef USE_GPS_MAPS
    int32_t lat_diff = UBX.rec_buffer.values.lat - UBX.lat;
    int32_t lon_diff = UBX.rec_buffer.values.lon - UBX.lon;
    if ((lat_diff > 1000) || (lon_diff > 1000)) {
      UBX.lat = UBX.rec_buffer.values.lat;
      UBX.lon = UBX.rec_buffer.values.lon;
      WSContentSend_P(UBX_GOOGLE_MAPS, lat, lon);
    }
#endif  // USE_GPS_MAPS

#ifdef USE_FLOG
    WSContentSeparator(0);
#ifdef DEBUG_TASMOTA_SENSOR
    WSContentSend_PD(HTTP_SNS_FLOGVER, Flog->num_sectors, Flog->size, Flog->current_sector, Flog->sectors_left, Flog->sector.header.physical_start_sector);
    if (Flog->recording) {
      WSContentSend_PD(HTTP_SNS_FLOGREC, Flog->sector.header.buf_pointer - 8);
    }
#endif  // DEBUG_TASMOTA_SENSOR
    if (Flog->ready) {
      char flog_state[32];
      WSContentSend_P(HTTP_SNS_FLOG, GetTextIndexed(flog_state, sizeof(flog_state), Flog->recording, kFLOGstate));
    }
    if (!Flog->recording && Flog->found_saved_data) {
      WSContentSend_P(HTTP_BTN_FLOG_DL);
    }
#endif  // USE_FLOG

    if (UBX.mode.runningNTP) {
      WSContentSeparator(0);
      if (UBX.mode.ppsLocked) {
        WSContentSend_P(HTTP_SNS_NTPPPS, UBXPulseSteered() ? PSTR("PPS, GNSS locked")
                                                           : PSTR("PPS, free running"));
      } else {
        WSContentSend_P(HTTP_SNS_NTPSERVER);
      }
      WSContentSend_P(HTTP_SNS_NTPSTATE, Rtc.stratum, Rtc.drift_ppb);
    }

#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * check the UBX commands
\*********************************************************************************************/

bool UBXCmd(void)
{
  bool serviced = true;
  if (XdrvMailbox.data_len > 0) {
    UBXSelectMode(XdrvMailbox.payload);
    Response_P(S_JSON_UBX_COMMAND_NVALUE, XdrvMailbox.command, XdrvMailbox.payload);
  }
  return serviced;
}

const char kUBXCommands[] PROGMEM = "Gps|"  // Prefix
  D_CMND_NTPSERVER "|Pps|Reset";

void (* const UBXCommand[])(void) PROGMEM = {
  &CmndGpsNtpServer, &CmndGpsPps, &CmndGpsReset };

// GpsReset 0|1|2 - Hot, warm or cold start of the receiver, configuration is kept
void CmndGpsReset(void) {
  static const uint16_t bbr[3] = { 0x0000, 0x0001, 0xFFFF };
  if ((XdrvMailbox.payload < 0) || (XdrvMailbox.payload > 2) || !UBXSerial) {
    ResponseCmndChar_P(PSTR("0 hot, 1 warm, 2 cold"));
    return;
  }
  uint16_t mask = bbr[XdrvMailbox.payload];
  uint8_t msg[10] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, (uint8_t)mask, (uint8_t)(mask >> 8), 0x02, 0x00 };
  UBXsendChecked(msg, sizeof(msg));
  UBX.mode.tpAligned = 0;                        // PPS not aligned until the next fix
  AddLog(LOG_LEVEL_INFO, PSTR("UBX: receiver %s start requested"),
    (2 == XdrvMailbox.payload) ? PSTR("cold") : (1 == XdrvMailbox.payload) ? PSTR("warm") : PSTR("hot"));
  ResponseCmndNumber(XdrvMailbox.payload);
}

void CmndGpsPps(void) {
  if ((XdrvMailbox.payload == 0) || (XdrvMailbox.payload == 1)) {
    Settings->sbflag1.gps_pps_off = !XdrvMailbox.payload;
    if (Settings->sbflag1.gps_pps_off) { UBXDropPPS(PSTR("disabled")); }
  }
  ResponseCmndNumber(!Settings->sbflag1.gps_pps_off);
}

void CmndGpsNtpServer(void) {
  if (((0 == XdrvMailbox.payload) || (1 == XdrvMailbox.payload)) && !TasmotaGlobal.global_state.network_down) {
    UBXSelectMode(XdrvMailbox.payload ? 9 : 10);
  }
  ResponseCmndStateText(Settings->sbflag1.gps_ntp_server);
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns60(uint32_t function)
{
  bool result = false;

  if (FUNC_INIT == function) {
    UBXDetect();
  }

  if (UBX.mode.init) {
    switch (function) {
      case FUNC_COMMAND_SENSOR:
        if (XSNS_60 == XdrvMailbox.index) {
          result = UBXCmd();
        }
        break;
      case FUNC_COMMAND:
        result = DecodeCommand(kUBXCommands, UBXCommand);
        break;
      case FUNC_LOOP:
      case FUNC_SLEEP_LOOP:
        UBXHandlePPS();          // First, PPS sets the clock
        if (UBX.mode.runningNTP) { timeServer.poll(); }
        break;
      case FUNC_EVERY_50_MSECOND:
        UBXLoop50msec();
        break;
      case FUNC_EVERY_100_MSECOND:
#ifdef USE_FLOG
        if (!Flog->running_download)
#endif  // USE_FLOG
        {
          UBXLoop();
        }
        UBXCheckPPSLost();
        UBXCheckSilent();
        UBXRestoreNTP();
        break;
#ifdef USE_FLOG
      case FUNC_WEB_ADD_HANDLER:
        WebServer_on(PSTR("/UBX"), UBXsendFile);
        break;
#endif  // USE_FLOG
      case FUNC_JSON_APPEND:
        UBXShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
#ifdef USE_FLOG
        if (!Flog->running_download)
#endif  // USE_FLOG
        {
          UBXShow(0);
        }
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_GPS
