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
    NTP_SRC_GPS_FREE,   // GPS serial message timing, no PPS
    NTP_SRC_CHIP,       // RTC chip
    NTP_SRC_UPSTREAM,   // Upstream NTP server
};

#define NTP_DRIFT_UNKNOWN_PPB 25000   // Crystal drift used until measured (25 ppm)

#define RTC_STRATUM_GPS       1               // GPS with fix
#define RTC_STRATUM_GPS_PPS   2               // GPS PPS without fix
#define RTC_STRATUM_GPS_FREE  3               // GPS serial message timing only (no PPS), with or without fix
#define RTC_STRATUM_CHIP      4               // RTC chip
#define RTC_STRATUM_NONE     16               // Unsynchronized
#define RTC_STRATUM_UPSTREAM(s)  (((s) > 0 && (s) < 15) ? (s) + 1 : 3)   // Upstream stratum + 1

// Error of the start of the second without a phase source
#define RTC_EPOCH_MESSAGE_US  450000          // GPS serial message timing
#define RTC_EPOCH_CHIP_US    1000000          // RTC chip, whole seconds

// Filled by RtcDescribe(), tickPpb by RtcNtpRequest(). No default values: an ESP8266 would keep them in RAM
struct NtpTimeBase
{
    NtpSource source;
    uint8_t stratum;               // Stratum to send, 16 = unsynchronized
    int8_t precision;              // Clock resolution, log2 seconds: -10 = 1 ms
    uint32_t baseDispersionUs;     // Error of the source itself, before holdover adds to it, us
#ifdef USE_NTP_SERVER
    uint32_t refId;                // Upstream IPv4 address in network order, 0 = none
    uint32_t holdoverSecs;         // Seconds since the last update from the source
    int32_t driftPpb;              // Measured drift, 0 = unknown
    uint32_t rootDelayUs;          // Delay to the reference clock, 0 for a local one, us
    uint32_t stepMs;               // Recent clock step, added to the dispersion for a while, ms
    int32_t tickPpb;               // micros() drift, positive = fast
#endif  // USE_NTP_SERVER
};

// Reply values without the NTP server clock: whole seconds with the error given
inline void NtpBaseFixed(NtpTimeBase *base, NtpSource source, uint8_t stratum, uint32_t dispersionUs)
{
    base->source = source;
    base->stratum = stratum;
    base->baseDispersionUs = dispersionUs;
    base->precision = -10;
}

// Reference id text for local sources, nullptr to send refId
inline const char *NtpRefName(const NtpTimeBase &base)
{
    switch (base.source) {
    case NTP_SRC_PPS:
    case NTP_SRC_GPS:      return "GPS";
    case NTP_SRC_PPS_FREE: return "PPS";
    case NTP_SRC_CHIP:
#ifdef USE_NTP_SERVER
    case NTP_SRC_GPS_FREE: return base.refId ? nullptr : "LOCL";
#else
    case NTP_SRC_GPS_FREE: return "LOCL";
#endif  // USE_NTP_SERVER
    default:               return nullptr;
    }
}

#ifdef USE_NTP_SERVER
// Root dispersion, us: base + drift x holdover + a pending step
inline uint32_t NtpDispersionUs(const NtpTimeBase &base)
{
    int32_t drift = base.driftPpb ? base.driftPpb : NTP_DRIFT_UNKNOWN_PPB;
    if (drift < 0) { drift = -drift; }
    // Drift x holdover in 10 ppb steps, holdover capped at 400000 s: 32 bits up to 107 ppm
    uint32_t held = base.holdoverSecs;
    if (held > 400000UL) { held = 400000UL; }
    return base.baseDispersionUs + ((uint32_t)drift / 10u) * held / 100u + base.stepMs * 1000u;
}
#endif  // USE_NTP_SERVER

inline uint32_t NtpBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// NTP short format (16.16 s, big endian) to us, below 16 s; exact as 10^6 / 2^16 = 15625 / 2^10
inline uint32_t NtpShortToUs(const uint8_t *p)
{
    uint32_t v = NtpBe32(p);
    return ((v >> 16) * 1000000u) + (((v & 0xFFFFu) * 15625u) >> 10);
}

// One reply from an upstream NTP server, before the clock takes it
#define NTP_SERVER_NAME_LEN 64
struct NtpReplyInfo {
    uint32_t delayUs;                  // Server root delay + our round trip
    uint32_t dispUs;                   // Server root dispersion
    uint32_t rttUs;                    // Round trip of the time used
    uint32_t refId;                    // Server IPv4 address, network order
    uint8_t stratum;
    char server[NTP_SERVER_NAME_LEN];
};

// NTP seconds since 1900 to Unix seconds, also correct after the 2036 rollover
#define NTP_TIMESTAMP_DIFF  (2208988800UL)      // 1900 to 1970 in seconds
inline uint32_t NtpSecsToUnix(uint32_t ntpSecs) { return ntpSecs - NTP_TIMESTAMP_DIFF; }

#endif // NTP_TIME_BASE_H
