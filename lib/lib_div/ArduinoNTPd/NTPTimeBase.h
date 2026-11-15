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

// Filled by RtcDescribe(), tickPpb by RtcNtpRequest(). No default values: an ESP8266 would keep them in RAM
struct NtpTimeBase
{
    NtpSource source;
    uint8_t stratum;               // Stratum to send, 16 = unsynchronized
    uint32_t refId;                // Upstream IPv4 address in network order, 0 = none
    uint32_t holdoverSecs;         // Seconds since the last update from the source
    int32_t driftPpb;              // Measured drift, 0 = unknown
    uint32_t baseDispersionUs;     // Error of the source itself, before holdover adds to it, us
    uint32_t rootDelayUs;          // Delay to the reference clock, 0 for a local one, us
    int8_t precision;              // Clock resolution, log2 seconds: -10 = 1 ms
    uint32_t stepMs;               // Recent clock step, added to the dispersion for a while, ms
    int32_t tickPpb;               // micros() drift, positive = fast
};

// Reference id text for local sources, nullptr to send refId
inline const char *NtpRefName(const NtpTimeBase &base)
{
    switch (base.source) {
    case NTP_SRC_PPS:
    case NTP_SRC_GPS:      return "GPS";
    case NTP_SRC_PPS_FREE: return "PPS";
    case NTP_SRC_CHIP:
    case NTP_SRC_GPS_FREE: return base.refId ? nullptr : "LOCL";
    default:               return nullptr;
    }
}

// Root dispersion, us: base + drift x holdover + a pending step
inline uint32_t NtpDispersionUs(const NtpTimeBase &base)
{
    int32_t drift = base.driftPpb ? base.driftPpb : NTP_DRIFT_UNKNOWN_PPB;
    if (drift < 0) { drift = -drift; }
    // Holdover adds drift x seconds since the source; drift in 10 ppb steps and at most
    // 400000 s (4.6 days) keep it in 32 bits
    uint32_t held = base.holdoverSecs;
    if (held > 400000UL) { held = 400000UL; }
    return base.baseDispersionUs + ((uint32_t)drift / 10u) * held / 100u + base.stepMs * 1000u;
}

inline uint32_t NtpBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// NTP short format (16.16 seconds, big endian) to us, for values below 16 s. Exact in 32 bits:
// 10^6 / 2^16 = 15625 / 2^10
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
