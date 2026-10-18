/*
  support_rtc.ino - Real Time Clock support for Tasmota

  Copyright (C) 2021  Theo Arends

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

/*********************************************************************************************\
 * Sources: Time by Michael Margolis and Paul Stoffregen (https://github.com/PaulStoffregen/Time)
 *          Timezone by Jack Christensen (https://github.com/JChristensen/Timezone)
\*********************************************************************************************/

const uint32_t SECS_PER_MIN = 60UL;
const uint32_t SECS_PER_HOUR = 3600UL;
const uint32_t SECS_PER_DAY = SECS_PER_HOUR * 24UL;
const uint32_t MINS_PER_HOUR = 60UL;

#define LEAP_YEAR(Y)  (((1970+Y)>0) && !((1970+Y)%4) && (((1970+Y)%100) || !((1970+Y)%400)))

#include <Ticker.h>

Ticker TickerRtc;
#ifdef USE_NTP_SERVER
Ticker TickerRtcAlign;                        // Moves the 1 s tick to just after the second starts
#define RTC_TICK_ALIGN_MS       10            // Tick target, ms after the second starts
#define RTC_TICK_ALIGN_LOW_MS    5            // Earlier, jitter could skip a second
#define RTC_TICK_ALIGN_HIGH_MS  30            // Later, the display lags
volatile bool rtc_tick_align_pending = false;
#endif  // USE_NTP_SERVER

static const uint8_t kDaysInMonth[] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; // API starts months from 1, this array starts from 0
static const char kMonthNamesEnglish[] PROGMEM = "JanFebMarAprMayJunJulAugSepOctNovDec";

#ifdef USE_NTP_SERVER
#define RTC_DRIFT_SRC_NONE     0              // Source of Rtc.drift_ppb
#define RTC_DRIFT_SRC_PPS      1
#define RTC_DRIFT_SRC_NTP      2

#define RTC_STEP_ANNOUNCE_MS   50             // Larger steps are announced in the NTP dispersion

#define RTC_NTP_POLL_MIN        64            // First drift poll interval, s, doubles up to the hourly sync
#define RTC_NTP_HALF_STEP_US   2000           // and the offset this small, us
#define RTC_NTP_SAMPLE_ANCHOR    0
#define RTC_NTP_SAMPLE_OK        1
#define RTC_NTP_SAMPLE_SKIP      2
#define RTC_SYNC_AGE_GPS        60            // GPS source max age, s
#define RTC_SYNC_FORCE           0            // Time set by the user

// support_wifi.ino is concatenated after this file
extern uint32_t wifi_ntp_root_delay_us;
extern uint32_t wifi_ntp_root_disp_us;
extern char wifi_ntp_server[];


#endif  // USE_NTP_SERVER
struct RTC {
  uint32_t utc_time = 0;
  uint32_t local_time = 0;
  uint32_t daylight_saving_time = 0;
  uint32_t standard_time = 0;
  uint32_t midnight = 0;
  uint32_t restart_time = 0;
  uint32_t nanos = 0;
  uint32_t millis = 0;
#ifdef USE_NTP_SERVER
  // millis and boundary_micros: start of the current second in both counters
  uint32_t boundary_micros = 0;      // micros() at the start of the second
  uint32_t edge_millis = 0;          // millis() of the last source update
  uint32_t last_sync = 0;                     // Uptime of the last source update
  uint32_t last_traceable = 0;                // Uptime of the last stratum 1 update
  uint32_t step_ms = 0;                       // Clock step to announce, ms
  uint32_t step_uptime = 0;                   // Uptime of that step
  uint32_t ntp_refid = 0;                     // Upstream IPv4 address, network order
  uint8_t phase_source = RTC_PHASE_NONE;      // RTC_PHASE_*
  bool boundary_renewed = false;              // Source updates every second
  bool from_chip = false;                     // Time from the RTC chip
  bool carried_edge = false;                  // PPS lost, carried by GPS serial messages
  uint32_t carried_secs = 0;                  // Seconds since the last PPS edge
  uint32_t chip_placed_us = 0;                // Error of a GPS second placed by the RTC chip, us, 0 = not placed
  uint32_t held_us = 0;                       // Error of a chip phase or of a lost source's time, us, 0 = 1 s
  bool from_upstream = false;                 // Time from upstream NTP or mesh
  int32_t drift_ppb = 0;                      // Crystal drift, 0 = not measured
  int32_t drift_frac_ppb = 0;                 // Drift correction remainder below 1 us
  int32_t drift_lag_us = 0;                   // Drift correction not yet in millis, us
  uint8_t drift_source = RTC_DRIFT_SRC_NONE;  // RTC_DRIFT_SRC_*
  uint32_t drift_span = 0;                    // Seconds the drift was measured over
  uint8_t stratum = RTC_STRATUM_NONE;         // Stratum of the time source
  bool boundary_valid = false;                // Start of the second is known
#else
//  uint32_t last_sync = 0;
#endif  // USE_NTP_SERVER
  int32_t time_timezone = 0;
  bool time_synced = false;
  bool last_synced = false;
  bool midnight_now = false;
  bool user_time_entry = false;               // Override NTP by user setting
} Rtc;

#ifdef USE_NTP_SERVER
#ifdef ESP32
// Drift kept in RTC memory across a restart
typedef struct {
  uint32_t magic;
  int32_t drift_ppb;
  uint32_t check;
  uint32_t source;
  uint32_t span;
} TRtcDriftKeep;
static RTC_NOINIT_ATTR TRtcDriftKeep RtcDriftKeep;
#define RTC_DRIFT_KEEP_MAGIC  0x44524654                          // "DRFT"

void RtcDriftKeepSave(int32_t ppb, uint8_t source, uint32_t span) {
  RtcDriftKeep.magic = RTC_DRIFT_KEEP_MAGIC;
  RtcDriftKeep.drift_ppb = ppb;
  RtcDriftKeep.source = source;
  RtcDriftKeep.span = span;
  RtcDriftKeep.check = RTC_DRIFT_KEEP_MAGIC ^ (uint32_t)ppb ^ source ^ span;
}

int32_t RtcDriftKeepLoad(uint8_t* source, uint32_t* span) {
  if ((RTC_DRIFT_KEEP_MAGIC != RtcDriftKeep.magic) ||
      ((RTC_DRIFT_KEEP_MAGIC ^ (uint32_t)RtcDriftKeep.drift_ppb ^ RtcDriftKeep.source ^ RtcDriftKeep.span) != RtcDriftKeep.check) ||
      (RtcDriftKeep.source > RTC_DRIFT_SRC_NTP) ||
      (abs(RtcDriftKeep.drift_ppb) >= 200000)) { return 0; }
  *source = RtcDriftKeep.drift_ppb ? RtcDriftKeep.source : RTC_DRIFT_SRC_NONE;
  *span = RtcDriftKeep.span;
  return RtcDriftKeep.drift_ppb;
}

// Protects the clock values read by the NTP server in the lwIP thread
portMUX_TYPE rtc_pair_mux = portMUX_INITIALIZER_UNLOCKED;
#endif  // ESP32

// Empty on ESP8266
static inline __attribute__((always_inline)) void RtcPairLock(void) {
#ifdef ESP32
  portENTER_CRITICAL(&rtc_pair_mux);
#endif  // ESP32
}
static inline __attribute__((always_inline)) void RtcPairUnlock(void) {
#ifdef ESP32
  portEXIT_CRITICAL(&rtc_pair_mux);
#endif  // ESP32
}
#define RTC_PAIR_LOCK()    RtcPairLock()
#define RTC_PAIR_UNLOCK()  RtcPairUnlock()

#endif  // USE_NTP_SERVER
uint32_t UtcTime(void) {
  return Rtc.utc_time;
}

uint32_t LocalTime(void) {
  return Rtc.local_time;
}

uint32_t Midnight(void) {
  return Rtc.midnight;
}

bool MidnightNow(void) {
  if (Rtc.midnight_now) {
    Rtc.midnight_now = false;
    return true;
  }
  return false;
}

bool IsDst(void) {
  return (Rtc.time_timezone == Settings->toffset[1]);
}

String GetBuildDateAndTime(void) {
  // "2017-03-07T11:08:02" - ISO8601:2004
  char bdt[21];
  char *p;
  static const char mdate_P[] PROGMEM = __DATE__;  // "Mar  7 2017"
  static const char mtime_P[] PROGMEM = __TIME__;  // "11:08:02"
  char mdate[strlen_P(mdate_P)+1];      // copy on stack first
  strcpy_P(mdate, mdate_P);
  char *smonth = mdate;
  int day = 0;
  int year = 0;

  // sscanf(mdate, "%s %d %d", bdt, &day, &year);  // Not implemented in 2.3.0 and probably too much code
  uint8_t i = 0;
  for (char *str = strtok_r(mdate, " ", &p); str && i < 3; str = strtok_r(nullptr, " ", &p)) {
    switch (i++) {
    case 0:  // Month
      smonth = str;
      break;
    case 1:  // Day
      day = atoi(str);
      break;
    case 2:  // Year
      year = atoi(str);
    }
  }
  char MonthNamesEnglish[sizeof(kMonthNamesEnglish)];
  strcpy_P(MonthNamesEnglish, kMonthNamesEnglish);
  int month = (strstr(MonthNamesEnglish, smonth) -MonthNamesEnglish) /3 +1;
  snprintf_P(bdt, sizeof(bdt), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%s"), year, month, day, mtime_P);
  return String(bdt);  // 2017-03-07T11:08:02
}

String GetSyslogDate(char* mxtime) {
  // Mmm dd hh:mm:ss
  // Jan  3 09:23:45
  // Assuming the day hasn't changed yet ;-)
  uint32_t month_idx = (RtcTime.month -1) * 3;
  char month[4] = { 0 };
  strncpy_P(month, kMonthNamesEnglish + month_idx, 3);
  char dt[16];
  snprintf_P(dt, sizeof(dt), PSTR("%s %2d %s"), month, RtcTime.day_of_month, mxtime);
  return String(dt);
}

String GetDate(void) {
  // yyyy-mm-ddT
  char dt[12];
  snprintf_P(dt, sizeof(dt), PSTR("%04d-%02d-%02dT"), RtcTime.year, RtcTime.month, RtcTime.day_of_month);
  return String(dt);
}

String GetMinuteTime(uint32_t minutes) {
  char tm[6];
  snprintf_P(tm, sizeof(tm), PSTR("%02d:%02d"), minutes / 60, minutes % 60);

  return String(tm);  // 03:45
}

String GetTimeZone(void) {
  char tz[7];
  snprintf_P(tz, sizeof(tz), PSTR("%+03d:%02d"), Rtc.time_timezone / 60, abs(Rtc.time_timezone % 60));

  return String(tz);  // -03:45
}

String GetDuration(uint32_t time) {
  char dt[16];

  TIME_T ut;
  BreakTime(time, ut);

  // "P128DT14H35M44S" - ISO8601:2004 - https://en.wikipedia.org/wiki/ISO_8601 Durations
//  snprintf_P(dt, sizeof(dt), PSTR("P%dDT%02dH%02dM%02dS"), ut.days, ut.hour, ut.minute, ut.second);

  // "128 14:35:44" - OpenVMS
  // "128T14:35:44" - Tasmota
  snprintf_P(dt, sizeof(dt), PSTR("%dT%02d:%02d:%02d"), ut.days, ut.hour, ut.minute, ut.second);

  return String(dt);  // 128T14:35:44
}

String GetDT(uint32_t time) {
  // "2017-03-07T11:08:02" - ISO8601:2004

  char dt[20];
  TIME_T tmpTime;

  BreakTime(time, tmpTime);
  snprintf_P(dt, sizeof(dt), PSTR("%04d-%02d-%02dT%02d:%02d:%02d"),
    tmpTime.year +1970, tmpTime.month, tmpTime.day_of_month, tmpTime.hour, tmpTime.minute, tmpTime.second);

  return String(dt);  // 2017-03-07T11:08:02
}

/*
 * timestamps in https://en.wikipedia.org/wiki/ISO_8601 format
 *
 *  DT_UTC     - current data and time in Greenwich, England (aka GMT)
 *  DT_LOCAL   - current date and time taking timezone into account
 *  DT_RESTART - the date and time this device last started, in local timezone
 *
 * Format:
 *  "2017-03-07T11:08:02-07:00" - if DT_LOCAL and SetOption52 = 1
 *  "2017-03-07T11:08:02"       - otherwise
 */
String GetDateAndTime(uint8_t time_type) {
  // "2017-03-07T11:08:02-07:00" - ISO8601:2004
  uint32_t time = Rtc.local_time;

  switch (time_type) {
    case DT_UTC:
      time = Rtc.utc_time;
      break;
//    case DT_LOCALNOTZ:  // Is default anyway but allows for not appendig timezone
//      time = Rtc.local_time;
//      break;
    case DT_DST:
      time = Rtc.daylight_saving_time;
      break;
    case DT_STD:
      time = Rtc.standard_time;
      break;
    case DT_RESTART:
      if (Rtc.restart_time == 0) {
        return "";
      }
      time = Rtc.restart_time;
      break;
    case DT_BOOTCOUNT:
      time = Settings->bootcount_reset_time;
      break;
  }
  uint32_t rest_ms = RtcMillis();
#ifdef USE_NTP_SERVER
  if (DT_LOCAL_MILLIS == time_type) {
    time += RtcCarry(&rest_ms);
  }
#endif  // USE_NTP_SERVER
  String dt = GetDT(time);  // 2017-03-07T11:08:02

  if (DT_LOCAL_MILLIS == time_type) {
    char ms[10];
    snprintf_P(ms, sizeof(ms), PSTR(".%03d"), rest_ms);
    dt += ms;               // 2017-03-07T11:08:02.123
    time_type = DT_LOCAL;
  }

  if (DT_UTC == time_type) {
    dt += "Z";              // 2017-03-07T11:08:02.123Z
  }
  else if (Settings->flag3.time_append_timezone && (DT_LOCAL == time_type)) {  // SetOption52 - Append timezone to JSON time
    dt += GetTimeZone();    // 2017-03-07T11:08:02-07:00
  }
  return dt;                // 2017-03-07T11:08:02-07:00 or 2017-03-07T11:08:02.123-07:00
}

uint32_t UpTime(void) {
  if (Rtc.restart_time) {
    return Rtc.utc_time - Rtc.restart_time;
  } else {
    return TasmotaGlobal.uptime;
  }
}

uint32_t MinutesUptime(void) {
  return (UpTime() / 60);
}

String GetUptime(void) {
  return GetDuration(UpTime());
}

uint32_t MinutesPastMidnight(void) {
  uint32_t minutes = 0;

  if (RtcTime.valid) {
    minutes = (RtcTime.hour *60) + RtcTime.minute;
  }
  return minutes;
}

uint32_t RtcMillis(void) {
  return (millis() - Rtc.millis) % 1000;
}

void BreakNanoTime(uint32_t time_input, uint32_t time_nanos, TIME_T &tm) {
// break the given time_input into time components
// this is a more compact version of the C library localtime function
// note that year is offset from 1970 !!!

  time_input += time_nanos / 1000000000U;
  tm.nanos = time_nanos % 1000000000U;

  uint8_t year;
  uint8_t month;
  uint8_t month_length;
  uint32_t time;
  unsigned long days;

  time = time_input;
  tm.second = time % 60;
  time /= 60;                // now it is minutes
  tm.minute = time % 60;
  time /= 60;                // now it is hours
  tm.hour = time % 24;
  time /= 24;                // now it is days
  tm.days = time;
  tm.day_of_week = ((time + 4) % 7) + 1;  // Sunday is day 1

  year = 0;
  days = 0;
  while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  tm.year = year;            // year is offset from 1970

  days -= LEAP_YEAR(year) ? 366 : 365;
  time -= days;              // now it is days in this year, starting at 0
  tm.day_of_year = time;

  for (month = 0; month < 12; month++) {
    if (1 == month) { // february
      if (LEAP_YEAR(year)) {
        month_length = 29;
      } else {
        month_length = 28;
      }
    } else {
      month_length = pgm_read_byte(&kDaysInMonth[month]);
    }

    if (time >= month_length) {
      time -= month_length;
    } else {
      break;
    }
  }
  strlcpy(tm.name_of_month, kMonthNames + (month *3), 4);
  tm.month = month + 1;      // jan is month 1
  tm.day_of_month = time + 1;         // day of month
  tm.valid = (time_input > START_VALID_TIME);  // 2016-01-01
}

void BreakTime(uint32_t time_input, TIME_T &tm) {
  BreakNanoTime(time_input, 0, tm);
}

uint32_t MakeTime(TIME_T &tm) {
// assemble time elements into time_t
// note year argument is offset from 1970

  int i;
  uint32_t seconds;

  // seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds = tm.year * (SECS_PER_DAY * 365);
  for (i = 0; i < tm.year; i++) {
    if (LEAP_YEAR(i)) {
      seconds +=  SECS_PER_DAY;   // add extra days for leap years
    }
  }

  // add days for this year, months start from 1
  for (i = 1; i < tm.month; i++) {
    if ((2 == i) && LEAP_YEAR(tm.year)) {
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * pgm_read_byte(&kDaysInMonth[i-1]);  // monthDay array starts from 0
    }
  }
  seconds+= (tm.day_of_month - 1) * SECS_PER_DAY;
  seconds+= tm.hour * SECS_PER_HOUR;
  seconds+= tm.minute * SECS_PER_MIN;
  seconds+= tm.second;
  return seconds;
}

uint32_t RuleToTime(TimeRule r, int yr) {
  TIME_T tm;
  uint32_t t;
  uint8_t m;
  uint8_t w;                // temp copies of r.month and r.week

  m = r.month;
  w = r.week;
  if (0 == w) {             // Last week = 0
    if (++m > 12) {         // for "Last", go to the next month
      m = 1;
      yr++;
    }
    w = 1;                  // and treat as first week of next month, subtract 7 days later
  }

  tm.hour = r.hour;
  tm.minute = 0;
  tm.second = 0;
  tm.day_of_month = 1;
  tm.month = m;
  tm.year = yr - 1970;
  t = MakeTime(tm);         // First day of the month, or first day of next month for "Last" rules
  BreakTime(t, tm);
  t += (7 * (w - 1) + (r.dow - tm.day_of_week + 7) % 7) * SECS_PER_DAY;
  if (0 == r.week) {
    t -= 7 * SECS_PER_DAY;  // back up a week if this is a "Last" rule
  }
  return t;
}

void RtcGetDaylightSavingTimes(uint32_t local_time) {
  TIME_T tmpTime;
  BreakTime(local_time, tmpTime);
  tmpTime.year += 1970;
  Rtc.daylight_saving_time = RuleToTime(Settings->tflag[1], tmpTime.year);
  Rtc.standard_time = RuleToTime(Settings->tflag[0], tmpTime.year);
}

uint32_t RtcTimeZoneOffset(uint32_t local_time) {
  int16_t timezone_minutes = Settings->timezone_minutes;
  if (Settings->timezone < 0) { timezone_minutes *= -1; }
  uint32_t timezone = (Settings->timezone * SECS_PER_HOUR) + (timezone_minutes * SECS_PER_MIN);
  if (99 == Settings->timezone) {
    int32_t dstoffset = Settings->toffset[1] * SECS_PER_MIN;
    int32_t stdoffset = Settings->toffset[0] * SECS_PER_MIN;
    if (Settings->tflag[1].hemis) {
      // Southern hemisphere
      if ((local_time >= (Rtc.standard_time - dstoffset)) && (local_time < (Rtc.daylight_saving_time - stdoffset))) {
        timezone = stdoffset;  // Standard Time
      } else {
        timezone = dstoffset;  // Daylight Saving Time
      }
    } else {
      // Northern hemisphere
      if ((local_time >= (Rtc.daylight_saving_time - stdoffset)) && (local_time < (Rtc.standard_time - dstoffset))) {
        timezone = dstoffset;  // Daylight Saving Time
      } else {
        timezone = stdoffset;  // Standard Time
      }
    }
  }
  return timezone;
}

void RtcSetTimeOfDay(uint32_t local_time) {
  // Sync Core/RTOS time to be used by file system time stamps
  struct timeval tv;
  tv.tv_sec = local_time;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

void RtcDeriveLocal(void) {
  if (Rtc.utc_time > START_VALID_TIME) {  // 2016-01-01
    Rtc.time_timezone = RtcTimeZoneOffset(Rtc.utc_time);
    Rtc.local_time = Rtc.utc_time + Rtc.time_timezone;
    Rtc.time_timezone /= 60;
    if (Settings->bootcount_reset_time < START_VALID_TIME) {
      Settings->bootcount_reset_time = Rtc.local_time;
    }
  } else {
    Rtc.local_time = Rtc.utc_time;
  }

  BreakNanoTime(Rtc.local_time, Rtc.nanos, RtcTime);
  if (RtcTime.valid) {
    if (!Rtc.midnight) {
      Rtc.midnight = Rtc.local_time - (RtcTime.hour * 3600) - (RtcTime.minute * 60) - RtcTime.second;
    }
    if (!RtcTime.hour && !RtcTime.minute && !RtcTime.second && (Rtc.midnight != Rtc.local_time)) {
      Rtc.midnight = Rtc.local_time;
      Rtc.midnight_now = true;
    }
  }

  RtcTime.year += 1970;
}

#ifdef USE_NTP_SERVER

// Current time for log timestamps, also when the tick is late
uint32_t RtcStampNow(TIME_T &stamp) {
  uint32_t rest_ms;
  uint32_t carry = RtcCarry(&rest_ms);
  stamp = RtcTime;
  if (carry && RtcTime.valid) { BreakTime(Rtc.local_time + carry, stamp); }
  return rest_ms;
}

/*********************************************************************************************\
 * Time sources set the clock: "UTC second N started at this time, stratum S"
\*********************************************************************************************/

bool RtcPpsExpected(void) {
#ifdef USE_GPS
  return UBXPpsExpected();
#else
  return false;
#endif  // USE_GPS
}

bool RtcPpsLive(void) {                          // PPS edges arriving
#ifdef USE_GPS
  return UBXTimeSourcePps() & 2;
#else
  return false;
#endif  // USE_GPS
}

/*********************************************************************************************\
 * Crystal drift from GPS PPS or upstream NTP. PPS is preferred, NTP replaces a PPS figure
 * only when it clearly differs
\*********************************************************************************************/

void RtcDriftSet(uint8_t source, int32_t ppb, bool moved, uint32_t span) {
  if (RTC_DRIFT_SRC_NTP == source) {
    if (RtcPpsLive()) { return; }
    if ((RTC_DRIFT_SRC_PPS == Rtc.drift_source) && !moved) { return; }
  }
  if (source != Rtc.drift_source) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: drift %d ppb, now measured by %s"), ppb,
      (RTC_DRIFT_SRC_PPS == source) ? "PPS" : "NTP");
  }
  Rtc.drift_ppb = ppb;
  Rtc.drift_source = source;
  Rtc.drift_span = span;
#ifdef ESP32
  RtcDriftKeepSave(ppb, source, span);                 // Keep across a restart
#endif  // ESP32
#ifdef USE_RTC_CHIP_HOLDOVER
  RtcChipDriftStore(ppb);                              // Keep across a power loss
#endif  // USE_RTC_CHIP_HOLDOVER
}

DriftEvent RtcDriftStep(const char* tag, uint8_t source, DriftEst* e, const DriftEstParams* p,
                        uint64_t step_us, uint32_t step_secs, int32_t standing, int32_t* figure) {
  DriftResult r;
  DriftEvent ev = DriftStep(e, p, step_us, step_secs, standing, &r);
  if (DRIFT_RESTART == ev) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s: drift restart, step %u s %d us off, lost %u s"),
      tag, step_secs, r.fracUs, r.secs);
  } else if (DRIFT_MOVED == ev) {
    AddLog(LOG_LEVEL_INFO, PSTR("%s: drift moved %d -> %d ppb over the last %u s, re-anchored"),
      tag, standing, r.ppb, r.secs);
    if (figure) { *figure = r.ppb; }
    RtcDriftSet(source, r.ppb, true, r.secs);
  }
  return ev;
}

void RtcDriftPublish(const char* tag, uint8_t source, DriftEst* e, const DriftEstParams* p, int32_t* figure) {
  DriftResult r;
  DriftEvent ev = DriftFigure(e, p, TasmotaGlobal.uptime, &r);
  if (DRIFT_HELD == ev) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s: drift %d ppb over %u s held back (bar %u s, best %u s)"),
      tag, r.ppb, r.secs, r.bar, e->spanBest);
  } else if (DRIFT_PUBLISH == ev) {
    if (figure) { *figure = r.ppb; }
    RtcDriftSet(source, r.ppb, false, r.secs);
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s: drift %d ppb over %u s (err %d us, res ~%u ppb)"),
      tag, r.ppb, r.secs, r.errUs, r.resPpb);
  }
}

// Drift parameters for upstream NTP (see DriftEstParams)
const DriftEstParams RtcNtpDriftParams = { 7200, 20000, 100, 880, 3600, 64, 1200, 1000, RTC_DRIFT_RESIDUAL_PPB };
struct {
  DriftEst est;
  uint64_t prev_us;                              // micros64 of the previous sample
  uint32_t prev_utc;
  uint32_t prev_refid;                           // A new server restarts the measurement
  uint32_t best_rtt_us;
  int32_t ppb;                                   // NTP drift, used or not
  uint8_t rejects;
} RtcNtpDrift;

uint64_t RtcMicros64(void) {
#ifdef ESP32
  return esp_timer_get_time();
#else
  return micros64();
#endif  // ESP32
}

// Drift sample: second utc started at micros() boundary_us. Returns RTC_NTP_SAMPLE_*
uint8_t RtcUpstreamSample(uint32_t utc, uint32_t boundary_us, uint32_t refid, uint32_t rtt_us) {
  uint64_t at_us = RtcMicros64() - (uint32_t)(micros() - boundary_us);
  if (!RtcNtpDrift.est.anchored || (refid != RtcNtpDrift.prev_refid) || (RtcNtpDrift.rejects >= 4)) {
    if (!RtcNtpDrift.est.anchored && (RTC_DRIFT_SRC_NTP == Rtc.drift_source)) {
      RtcNtpDrift.est.spanBest = Rtc.drift_span;   // Drift kept across a restart
      RtcNtpDrift.est.lostUptime = TasmotaGlobal.uptime;
      RtcNtpDrift.ppb = Rtc.drift_ppb;
    }
    DriftAnchor(&RtcNtpDrift.est);
    RtcNtpDrift.prev_us = at_us;
    RtcNtpDrift.prev_utc = utc;
    RtcNtpDrift.prev_refid = refid;
    RtcNtpDrift.best_rtt_us = rtt_us;
    RtcNtpDrift.rejects = 0;
    AddLog(LOG_LEVEL_DEBUG, PSTR("NTP: drift anchored at %_I, rtt %u us"), refid, rtt_us);
    return RTC_NTP_SAMPLE_ANCHOR;
  }
  if (rtt_us > (RtcNtpDrift.best_rtt_us * 3 / 2) + 1000) {   // Skip slow replies
    RtcNtpDrift.rejects++;
    AddLog(LOG_LEVEL_DEBUG, PSTR("NTP: drift sample skipped, rtt %u us (best %u us)"),
      rtt_us, RtcNtpDrift.best_rtt_us);
    return RTC_NTP_SAMPLE_SKIP;
  }
  RtcNtpDrift.rejects = 0;
  if (rtt_us < RtcNtpDrift.best_rtt_us) { RtcNtpDrift.best_rtt_us = rtt_us; }
  uint64_t step_us = at_us - RtcNtpDrift.prev_us;
  uint32_t step_secs = utc - RtcNtpDrift.prev_utc;
  RtcNtpDrift.prev_us = at_us;
  RtcNtpDrift.prev_utc = utc;
  int32_t standing = RtcPpsLive() ? RtcNtpDrift.ppb : Rtc.drift_ppb;
  DriftEvent ev = RtcDriftStep("NTP", RTC_DRIFT_SRC_NTP, &RtcNtpDrift.est, &RtcNtpDriftParams,
                               step_us, step_secs, standing, &RtcNtpDrift.ppb);
  if (DRIFT_RESTART == ev) { return RTC_NTP_SAMPLE_ANCHOR; }
  RtcDriftPublish("NTP", RTC_DRIFT_SRC_NTP, &RtcNtpDrift.est, &RtcNtpDriftParams, &RtcNtpDrift.ppb);
  return (DRIFT_MOVED == ev) ? RTC_NTP_SAMPLE_ANCHOR : RTC_NTP_SAMPLE_OK;
}

// Time from a PPS edge
bool RtcPhaseGood(void) {
  if (Rtc.stratum >= RTC_STRATUM_NONE) { return false; }
  return (RTC_PHASE_EDGE == Rtc.phase_source) || Rtc.carried_edge;
}

bool RtcPhaseNetwork(void) {
  return (Rtc.stratum < RTC_STRATUM_NONE) && (RTC_PHASE_NETWORK == Rtc.phase_source);
}

// Is the time good enough to set the RTC chip? 0 = within RTC_HOLDOVER_PLACE_US (PPS or LAN),
// 1 = within RTC_HOLDOVER_PLACE_FAR_US (internet), -1 = no
uint8_t RtcNtpDescribe(NtpTimeBase* base);
int8_t RtcPhasePlaceClass(void) {
  if (!RtcPhaseGood() && !RtcPhaseNetwork()) { return -1; }
  NtpTimeBase base;
  RtcNtpDescribe(&base);
  uint32_t distance_us = (base.rootDelayUs / 2) + NtpDispersionUs(base);   // As the reply claims it now
  if (distance_us < RTC_HOLDOVER_PLACE_US) { return 0; }
  return (distance_us < RTC_HOLDOVER_PLACE_FAR_US) ? 1 : -1;
}

bool RtcPhasePlaceable(void) {
  return RtcPhasePlaceClass() >= 0;
}

// Second nearest to micros() at_us, off_us = at_us minus its start
uint32_t RtcSecondAt(uint32_t at_us, int32_t* off_us) {
  RTC_PAIR_LOCK();
  uint32_t b_us = Rtc.boundary_micros;
  uint32_t utc = Rtc.utc_time;
  int32_t ppb = Rtc.drift_ppb;
  RTC_PAIR_UNLOCK();
  return utc + RtcNearestSecond((int32_t)(at_us - b_us), ppb, off_us);
}

RtcHolder RtcHolderNow(void) {
  RtcHolder h;
  h.stratum = Rtc.stratum;
  h.from_upstream = Rtc.from_upstream;
  h.drift_known = (0 != Rtc.drift_ppb);
  h.since_sync = TasmotaGlobal.uptime - Rtc.last_sync;
  h.since_traceable = Rtc.last_traceable ? (TasmotaGlobal.uptime - Rtc.last_traceable) : UINT32_MAX;
  return h;
}

RtcArbVerdict RtcArbitrateNow(uint8_t stratum, uint32_t max_age, uint8_t phase_source, bool pps_expected) {
  return RtcArbitrate(RtcHolderNow(), stratum, max_age, phase_source, pps_expected);
}

bool RtcBoundaryAllowed(uint8_t stratum, uint32_t max_age) {
  return RTC_ARB_WORSE != RtcArbitrateNow(stratum, max_age, RTC_PHASE_NETWORK, false);
}

bool RtcSetBoundary(uint32_t utc, uint32_t boundary_ms, uint32_t boundary_us, uint8_t stratum, uint32_t max_age, uint8_t phase_source, bool renewed, bool named = false) {
  if (utc < START_VALID_TIME) { return false; }
  if (RTC_ARB_TAKE != RtcArbitrateNow(stratum, max_age, phase_source, RtcPpsExpected())) { return false; }

  int32_t age = (int32_t)(millis() - boundary_ms);
  if ((age < 0) || (age >= 1000)) { return false; }          // Not in this second

  // Announce a step within the second
  if (Rtc.utc_time > START_VALID_TIME) {
    int32_t step_ms = age - (int32_t)((millis() - Rtc.millis) % 1000);
    if (step_ms > 500)  { step_ms -= 1000; }
    if (step_ms < -500) { step_ms += 1000; }
    if (abs(step_ms) > RTC_STEP_ANNOUNCE_MS) {
      RTC_PAIR_LOCK();
      Rtc.step_ms = (uint32_t)abs(step_ms);
      Rtc.step_uptime = TasmotaGlobal.uptime;
      RTC_PAIR_UNLOCK();
    }
  }

  if ((Rtc.utc_time > START_VALID_TIME) && (RTC_PHASE_EDGE == phase_source) &&
      (RTC_PHASE_EDGE != Rtc.phase_source) && (stratum < RTC_STRATUM_CHIP)) {
    int32_t step_us = (int32_t)(boundary_us - Rtc.boundary_micros) % 1000000;
    if (step_us > 500000)  { step_us -= 1000000; }
    if (step_us < -500000) { step_us += 1000000; }
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: reference back after %u s, phase step %d us (+ = we ran fast)"),
      Rtc.last_traceable ? (TasmotaGlobal.uptime - Rtc.last_traceable) : 0, step_us);
  }

  // Set all together for the NTP server
  RTC_PAIR_LOCK();
  Rtc.utc_time = utc;
  Rtc.millis = boundary_ms;
  Rtc.nanos = 0;
  Rtc.boundary_micros = boundary_us;
  Rtc.edge_millis = boundary_ms;
  Rtc.boundary_valid = true;
  Rtc.drift_lag_us = 0;
  Rtc.stratum = stratum;
  Rtc.phase_source = phase_source;
  Rtc.boundary_renewed = renewed;
  Rtc.last_sync = TasmotaGlobal.uptime;
  if (RTC_STRATUM_GPS == stratum) { Rtc.last_traceable = TasmotaGlobal.uptime; }
  Rtc.carried_edge = false;
  Rtc.carried_secs = 0;
  Rtc.chip_placed_us = 0;
  Rtc.held_us = 0;
  if (!named) {                                  // A second named by the chip or NTP keeps its origin
    Rtc.from_chip = RtcPhaseFromChip(phase_source);
    Rtc.from_upstream = (RTC_PHASE_NETWORK == phase_source);
    if (!Rtc.from_upstream) { Rtc.ntp_refid = 0; }
  }
  RTC_PAIR_UNLOCK();
  RtcDeriveLocal();
  return true;
}

// GPS message while the clock carries the last PPS edge: the message names the second it arrives in, the
// tick keeps the phase. rename takes the message's second. 1 = taken, 0 = refused, -1 = another second
int8_t RtcCarryNamed(uint32_t named, uint32_t arrival_us, uint8_t stratum, uint32_t carried_secs, bool rename) {
  if (RTC_ARB_TAKE != RtcArbitrateNow(stratum, 2, RTC_PHASE_NONE, RtcPpsExpected())) { return 0; }
  uint32_t start_us;
  RTC_PAIR_LOCK();
  uint32_t label = RtcSecondOfArrival(arrival_us, Rtc.boundary_micros, Rtc.utc_time, Rtc.drift_ppb, RTC_MSG_GUARD_US, &start_us);
  bool ok = label && ((label == named) || rename);
  if (ok) {
    Rtc.utc_time += named - label;
    Rtc.stratum = stratum;
    Rtc.phase_source = RTC_PHASE_NONE;
    Rtc.carried_edge = true;
    Rtc.carried_secs = carried_secs;
    Rtc.last_sync = TasmotaGlobal.uptime;
    if (RTC_STRATUM_GPS == stratum) { Rtc.last_traceable = TasmotaGlobal.uptime; }
  }
  RTC_PAIR_UNLOCK();
  return ok ? 1 : -1;
}

// Source lost. Call under RTC_PAIR_LOCK
void RtcDropSource(uint8_t stratum) {
  Rtc.stratum = stratum;
  Rtc.boundary_valid = false;
  Rtc.phase_source = RTC_PHASE_NONE;
  Rtc.boundary_renewed = false;
  Rtc.carried_edge = false;
  Rtc.carried_secs = 0;
  Rtc.chip_placed_us = 0;
  Rtc.held_us = 0;
  Rtc.from_upstream = false;
  Rtc.ntp_refid = 0;
}

// NTP reply values. Call under RTC_PAIR_LOCK
void RtcNtpBase(NtpTimeBase* base) {
  uint8_t chip_stratum = 0;
#ifdef USE_RTC_CHIPS
  chip_stratum = RtcChipStratum();
#endif  // USE_RTC_CHIPS
  RtcDescribe(Rtc, TasmotaGlobal.uptime, chip_stratum, wifi_ntp_root_delay_us, wifi_ntp_root_disp_us, base);
}

/*********************************************************************************************\
 * NTP port 123 is used by the GPS or the RTC chip server, whichever starts first. It serves the
 * best time source
\*********************************************************************************************/

#define RTC_NTP_OWNER_NONE  0
#define RTC_NTP_OWNER_GPS   1
#define RTC_NTP_OWNER_CHIP  2

struct {
  uint8_t bound = RTC_NTP_OWNER_NONE;    // Server using port 123
  uint8_t best = RTC_NTP_OWNER_NONE;     // Server with the best time
} RtcNtp;

bool RtcNtpBind(uint8_t owner) {
  if (RTC_NTP_OWNER_NONE == RtcNtp.bound) { RtcNtp.bound = owner; }
  return (owner == RtcNtp.bound);
}

bool RtcNtpBoundBy(uint8_t owner) {
  return (owner == RtcNtp.bound);
}

void RtcNtpRelease(uint8_t owner) {
  if (owner == RtcNtp.bound) {
    RtcNtp.bound = RTC_NTP_OWNER_NONE;
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: NTP port released by %s"),
      (RTC_NTP_OWNER_GPS == owner) ? PSTR("GPS") : PSTR("chip"));
  }
}

// Every second: drop a GPS source without update to the RTC chip stratum
void RtcNtpTrackSource(void) {
  if ((Rtc.last_sync > 0) && RtcSourceStale(RtcHolderNow())) {
    AddLog(LOG_LEVEL_INFO, PSTR("RTC: source silent %u s, stratum %d -> %d"),
      TasmotaGlobal.uptime - Rtc.last_sync, Rtc.stratum, RTC_STRATUM_CHIP);
    NtpTimeBase lost;
    RtcNtpDescribe(&lost);
    RTC_PAIR_LOCK();
    RtcDropSource(RTC_STRATUM_CHIP);
    Rtc.held_us = NtpDispersionUs(lost);           // The time keeps the error it had
#ifdef USE_RTC_CHIPS
    Rtc.from_chip = RtcChipPresent();
#endif  // USE_RTC_CHIPS
    RTC_PAIR_UNLOCK();
  }

  uint8_t now = (Rtc.stratum <= RTC_STRATUM_GPS_FREE) ? RTC_NTP_OWNER_GPS
              : (Rtc.stratum < RTC_STRATUM_NONE)      ? RTC_NTP_OWNER_CHIP
                                                      : RTC_NTP_OWNER_NONE;
  if (now == RtcNtp.best) { return; }
  RtcNtp.best = now;
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: NTP time from %s, stratum %d"),
    Rtc.from_upstream ? PSTR("upstream") :
    (RTC_NTP_OWNER_GPS == now) ? PSTR("GPS") : (RTC_NTP_OWNER_CHIP == now) ? PSTR("chip") : PSTR("nothing"),
    Rtc.stratum);
}

uint8_t RtcNtpDescribe(NtpTimeBase* base) {
  RTC_PAIR_LOCK();
  RtcNtpBase(base);
  uint8_t phase = Rtc.phase_source;
  RTC_PAIR_UNLOCK();
  return phase;
}

void RtcResponseAppendTimeSrc(void) {
  NtpTimeBase base;
  uint8_t phase = RtcNtpDescribe(&base);
  char source[10];
  GetTextIndexed(source, sizeof(source), base.source, PSTR("NONE|PPS|GPS|PPS_FREE|GPS_FREE|CHIP|UPSTREAM"));
  char refid[16] = "";
  const char *name = NtpRefName(base);
  if (name) { strlcpy(refid, name, sizeof(refid)); }
  else if (base.refId) { ext_snprintf_P(refid, sizeof(refid), PSTR("%_I"), base.refId); }
  ResponseAppend_P(PSTR(",\"TimeSrc\":{\"Src\":%d,\"Source\":\"%s\",\"Refid\":\"%s\",\"Stratum\":%d,\"Phase\":%d,\"Server\":\"%s\"}"),
    base.source, source, refid, base.stratum, phase, wifi_ntp_server);
}

uint32_t RtcTimeSourcePacked(void) {
  NtpTimeBase base;
  uint8_t phase = RtcNtpDescribe(&base);
  uint32_t fix = 7;
  uint32_t pps = 0;
#ifdef USE_GPS
  fix = UBXTimeSourceFix();
  pps = UBXTimeSourcePps();
#endif  // USE_GPS
  return (base.source & 0x0F) | ((phase & 0x0F) << 4) | ((base.stratum & 0x1F) << 8) | ((fix & 0x07) << 13) |
         ((pps & 0x03) << 16);
}


// Drop the GPS source
void RtcReleaseSource(uint8_t stratum) {
  if ((Rtc.stratum >= stratum) || Rtc.from_upstream) { return; }   // Only a GPS source
  AddLog(LOG_LEVEL_INFO, PSTR("RTC: release utc=%u phase=%u ms str=%d->16"),
    Rtc.utc_time, RtcMillis(), Rtc.stratum);
  RTC_PAIR_LOCK();
  RtcDropSource(RTC_STRATUM_NONE);
  Rtc.last_sync = 0;
  RTC_PAIR_UNLOCK();
}

void RtcNtpSent(const NtpTimeBase &base) {
  RTC_PAIR_LOCK();
  if (base.stepMs == Rtc.step_ms) { Rtc.step_ms = 0; }   // Step announced
  RTC_PAIR_UNLOCK();
}

bool RtcNtpRequest(uint8_t owner, NtpTimeBase* base, uint32_t* utc, uint32_t* boundary_us) {
  if (!RtcNtpBoundBy(owner)) { return false; }
  RTC_PAIR_LOCK();
  RtcNtpBase(base);
  *utc = Rtc.utc_time;
  *boundary_us = Rtc.boundary_micros;
  base->tickPpb = Rtc.drift_ppb;                     // Drift
  RTC_PAIR_UNLOCK();
  // No reply without a time source
  return (NTP_SRC_NONE != base->source);
}

// Whole seconds since the tick: it may be late, or a 1 Hz source has not set the second yet
uint32_t RtcCarry(uint32_t* rest_ms) {
  return RtcCarrySeconds(millis() - Rtc.millis, rest_ms);
}

void RtcSecond(void);

void RtcTickAligned(void) {
  TickerRtc.attach(1, RtcSecond);
  rtc_tick_align_pending = false;
  if ((uint32_t)(millis() - Rtc.millis) >= 1000) {
    RtcSecond();
  }
}

void RtcTickAlign(uint32_t rest_ms) {
  if (rtc_tick_align_pending) { return; }
  if (rest_ms >= 1000) { return; }                                  // More next tick
  if ((rest_ms >= RTC_TICK_ALIGN_LOW_MS) && (rest_ms <= RTC_TICK_ALIGN_HIGH_MS)) { return; }
  rtc_tick_align_pending = true;
  TickerRtcAlign.once_ms(1000 - rest_ms + RTC_TICK_ALIGN_MS, RtcTickAligned);
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: tick at %u ms into the second, moving it to %u ms"),
    rest_ms, RTC_TICK_ALIGN_MS);
}

#endif  // USE_NTP_SERVER
void RtcSecond(void) {
  static uint32_t last_sync = 0;
  static bool mutex = false;

  if (mutex) { return; }

  if (Rtc.time_synced) {
    mutex = true;

    Rtc.time_synced = false;
    Rtc.last_synced = true;
    last_sync = Rtc.utc_time;
#ifdef USE_NTP_SERVER
    if (!Rtc.boundary_valid) {
      RTC_PAIR_LOCK();
      Rtc.millis = millis();
      Rtc.boundary_micros = micros();
      Rtc.drift_lag_us = 0;
      Rtc.nanos = 0;
      RTC_PAIR_UNLOCK();
    }
#endif  // USE_NTP_SERVER

    if (Rtc.restart_time == 0) {
      Rtc.restart_time = Rtc.utc_time - TasmotaGlobal.uptime;  // save first synced time as restart time
    }

    RtcGetDaylightSavingTimes(Rtc.utc_time);

    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: " D_UTC_TIME " %s, " D_DST_TIME " %s, " D_STD_TIME " %s"),
      GetDateAndTime(DT_UTC).c_str(), GetDateAndTime(DT_DST).c_str(), GetDateAndTime(DT_STD).c_str());

    if (Rtc.local_time < START_VALID_TIME) {  // 2016-01-01
      TasmotaGlobal.rules_flag.time_init = 1;
    } else {
      TasmotaGlobal.rules_flag.time_set = 1;
    }
  } else {
#ifdef USE_NTP_SERVER
    Rtc.last_synced = false;
    uint32_t now = millis();
    // Skip while a 1 Hz source updates the clock
    int32_t since_edge = (int32_t)(now - Rtc.edge_millis);
    if (!Rtc.boundary_renewed || (since_edge < 0) || (since_edge >= 1000)) {
      uint32_t elapsed, b_ms = Rtc.millis, b_us = Rtc.boundary_micros;
      uint32_t seconds = RtcTickAdvance(now, Rtc.drift_ppb, &b_ms, &b_us, &Rtc.drift_frac_ppb, &Rtc.drift_lag_us, &elapsed);
      if (1 != seconds) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: tick +%u s utc=%u->%u (elapsed %u ms, rest %u, str %d)"),
          seconds, Rtc.utc_time, Rtc.utc_time + seconds, (now - Rtc.millis), elapsed, Rtc.stratum);
      }
      RTC_PAIR_LOCK();
      Rtc.utc_time += seconds;
      Rtc.nanos = 0;
      Rtc.millis = b_ms;
      Rtc.boundary_micros = b_us;
      Rtc.boundary_renewed = false;
      RTC_PAIR_UNLOCK();
      RtcTickAlign(elapsed);
    }
    else {
      RtcTickAlign((uint32_t)since_edge);
    }
  }
#else
    if (Rtc.last_synced) {
      Rtc.last_synced = false;
      uint32_t nanos = Rtc.nanos + (millis() - Rtc.millis) * 1000000U;
      Rtc.utc_time += nanos / 1000000000U;
      Rtc.nanos = nanos % 1000000000U;
    } else
      Rtc.utc_time++;  // Increment every second
  }
  Rtc.millis = millis();
#endif  // USE_NTP_SERVER

  if ((Rtc.utc_time > (2 * 60 * 60)) && (last_sync < Rtc.utc_time - (2 * 60 * 60))) {  // Every two hours a warning
    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: Not synced"));
    last_sync = Rtc.utc_time;
  }

  RtcDeriveLocal();
  if (mutex && RtcTime.valid) {  // Time is just synced and is valid
    // Sync Core/RTOS time to be used by file system time stamps
    RtcSetTimeOfDay(Rtc.local_time);
  }

#ifdef USE_NTP_SERVER
  RtcNtpTrackSource();
#endif  // USE_NTP_SERVER

  mutex = false;
}

// Time set: restart time, rules, file time, FUNC_TIME_SYNCED
void RtcSynced(const char* source) {
  Rtc.time_synced = true;
  RtcSecond();
#ifdef USE_NTP_SERVER
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: Synced by %s, stratum %d"), source, Rtc.stratum);
#else
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: Synced by %s"), source);
#endif  // USE_NTP_SERVER
  XdrvCall(FUNC_TIME_SYNCED);
}

#ifdef USE_NTP_SERVER
// Set whole seconds. RTC_SYNC_FORCE (command Time) always sets. RTC_PHASE_NONE keeps the phase,
// except for upstream whole seconds, which have none
bool RtcSync(const char* source, uint32_t utc, uint8_t stratum, uint8_t phase_source, uint32_t max_age, bool upstream = false) {
  bool forced = (RTC_SYNC_FORCE == max_age);
  bool allowed = forced || RtcBoundaryAllowed(stratum, max_age);
  int32_t step = (Rtc.utc_time > START_VALID_TIME) ? (int32_t)(utc - Rtc.utc_time) : 0;
  AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: sync %s %s str=%d->%d d=%d age=%d max=%d"),
    source, forced ? PSTR("forced") : (allowed ? PSTR("allow") : PSTR("REJECT")),
    Rtc.stratum, stratum, step,
    (int32_t)(TasmotaGlobal.uptime - Rtc.last_sync), forced ? -1 : (int32_t)max_age);
  if (!allowed) { return false; }

  RTC_PAIR_LOCK();
  Rtc.utc_time = utc;
  Rtc.stratum = stratum;
  Rtc.boundary_renewed = false;
  Rtc.last_sync = TasmotaGlobal.uptime;
  if (upstream) {
    Rtc.phase_source = RTC_PHASE_NONE;
  } else if (RTC_PHASE_NONE != phase_source) {
    Rtc.phase_source = phase_source;
    Rtc.from_chip = RtcPhaseFromChip(phase_source);
  }
  Rtc.from_upstream = upstream;
  Rtc.ntp_refid = 0;                             // The NTP client sets its server's values after this
  wifi_ntp_root_delay_us = 0;
  wifi_ntp_root_disp_us = 0;
  RTC_PAIR_UNLOCK();
  RtcSynced(source);
  return true;
}
#else
void RtcSync(const char* source) {
  RtcSynced(source);
}
#endif  // USE_NTP_SERVER

void RtcSetTime(uint32_t epoch) {
  if (epoch < START_VALID_TIME) {  // 2016-01-01
    Rtc.user_time_entry = false;
    TasmotaGlobal.ntp_force_sync = true;
  } else {
    Rtc.user_time_entry = true;
#ifdef USE_NTP_SERVER
    RtcSync("Time", epoch, RTC_STRATUM_CHIP, RTC_PHASE_MANUAL, RTC_SYNC_FORCE);
#else
//    Rtc.utc_time = epoch -1;    // Will be corrected by RtcSecond
    Rtc.utc_time = epoch;
    RtcSync("Time");
#endif  // USE_NTP_SERVER
  }
}

void RtcInit(void) {
  Rtc.utc_time = 0;
  BreakTime(Rtc.utc_time, RtcTime);
  TickerRtc.attach(1, RtcSecond);

  if (Settings->cfg_timestamp > START_VALID_TIME) {
    // Fix file timestamp while utctime is not synced
    uint32_t utc_time = Settings->cfg_timestamp;
    if (RtcSettings.utc_time > utc_time) {
      utc_time = RtcSettings.utc_time;
    }
    utc_time++;
    RtcGetDaylightSavingTimes(utc_time);
    uint32_t local_time = utc_time + RtcTimeZoneOffset(utc_time);
    RtcSetTimeOfDay(local_time);
//    AddLog(LOG_LEVEL_DEBUG, PSTR("RTC: Timestamp %s"), GetDT(local_time).c_str());
  }
}

void RtcPreInit(void) {
  Rtc.millis = millis();
#ifdef USE_NTP_SERVER
#ifdef ESP32
  Rtc.drift_ppb = RtcDriftKeepLoad(&Rtc.drift_source, &Rtc.drift_span);   // Kept across a restart
#endif  // ESP32
#endif  // USE_NTP_SERVER
}
