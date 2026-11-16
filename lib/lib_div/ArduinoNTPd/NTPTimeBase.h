/*
  NTPTimeBase.h - quality of the served time: source, stratum, dispersion

  SPDX-FileCopyrightText: 2026 Jan-David Förster

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef NTP_TIME_BASE_H
#define NTP_TIME_BASE_H

#include <stdint.h>

enum NtpSource : uint8_t
{
    NTP_SRC_NONE = 0,   // No time source, server does not reply
    NTP_SRC_PPS,        // GPS PPS edge, receiver has a fix
    NTP_SRC_GPS,        // GPS serial message, receiver has a fix, no PPS
    NTP_SRC_PPS_FREE,   // GPS PPS edge, receiver has no fix
    NTP_SRC_GPS_FREE,   // GPS serial message, no fix and no PPS
    NTP_SRC_CHIP,       // RTC chip
    NTP_SRC_UPSTREAM,   // Upstream NTP server
};

#define NTP_DRIFT_UNKNOWN_PPB 25000   // Crystal drift used until measured (25 ppm)

#define RTC_STRATUM_GPS       1               // GPS with fix
#define RTC_STRATUM_GPS_PPS   2               // GPS PPS without fix
#define RTC_STRATUM_GPS_FREE  3               // GPS serial message only, no fix and no PPS
#define RTC_STRATUM_CHIP      4               // RTC chip
#define RTC_STRATUM_NONE     16               // Unsynchronized
#define RTC_STRATUM_UPSTREAM(s)  (((s) > 0 && (s) < 15) ? (s) + 1 : 3)   // Upstream stratum + 1

// Error of the start of the second without a phase source
#define RTC_EPOCH_MESSAGE_US  450000          // GPS serial message timing
#define RTC_EPOCH_CHIP_US    1000000          // RTC chip, whole seconds

// Filled by the server's source function. No default values: an ESP8266 would keep them in RAM
struct NtpTimeBase
{
    NtpSource source;
    uint8_t stratum;               // Stratum to send, 16 = unsynchronized
    int8_t precision;              // Clock resolution, log2 seconds: -10 = 1 ms
    uint32_t baseDispersionUs;     // Error of the source itself, before holdover adds to it, us
};

// Reply values of a simple server: whole seconds with the error given
inline void NtpBaseFixed(NtpTimeBase *base, NtpSource source, uint8_t stratum, uint32_t dispersionUs)
{
    base->source = source;
    base->stratum = stratum;
    base->baseDispersionUs = dispersionUs;
    base->precision = -10;
}

// Reference id text
inline const char *NtpRefName(const NtpTimeBase &base)
{
    switch (base.source) {
    case NTP_SRC_PPS:
    case NTP_SRC_GPS:      return "GPS";
    case NTP_SRC_PPS_FREE: return "PPS";
    case NTP_SRC_CHIP:
    case NTP_SRC_GPS_FREE: return "LOCL";
    default:               return nullptr;
    }
}

#define NTP_TIMESTAMP_DIFF  (2208988800UL)      // 1900 to 1970 in seconds

#endif // NTP_TIME_BASE_H
