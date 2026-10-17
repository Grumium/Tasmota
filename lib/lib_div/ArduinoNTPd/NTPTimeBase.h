/*
  NTPTimeBase.h - quality of the served time: source, stratum, dispersion

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

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

struct NtpTimeBase
{
    NtpSource source = NTP_SRC_NONE;
    uint8_t stratum = 16;          // Stratum to send, 16 = unsynchronized
    uint32_t refId = 0;            // Upstream IPv4 address in network order, 0 = none
    uint32_t holdoverSecs = 0;     // Seconds since the last update from the source
    int32_t driftPpb = 0;          // Measured drift, 0 = unknown
    uint32_t baseDispersionUs = 0; // Dispersion without holdover, us
    uint32_t rootDelayUs = 0;      // Delay to the reference clock, 0 for a local one, us
    int8_t precision = -10;        // log2 seconds
    uint32_t stepMs = 0;           // Clock step to announce once, ms
    int32_t tickPpb = 0;           // micros() drift, positive = fast
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
    // Holdover adds drift x seconds; 10 ppb steps and 400000 s max keep it in 32 bits
    uint32_t held = base.holdoverSecs;
    if (held > 400000UL) { held = 400000UL; }
    return base.baseDispersionUs + ((uint32_t)drift / 10u) * held / 100u + base.stepMs * 1000u;
}

// NTP short format (16.16 seconds, big endian) to us, for values below 16 s
inline uint32_t NtpShortToUs(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    return (uint32_t)(((uint64_t)v * 1000000ULL) >> 16);
}

// NTP seconds since 1900 to Unix seconds, also correct after the 2036 rollover
inline uint32_t NtpSecsToUnix(uint32_t ntpSecs) { return ntpSecs - 2208988800UL; }

#endif // NTP_TIME_BASE_H
