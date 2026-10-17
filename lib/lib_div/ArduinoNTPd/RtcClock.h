/*
  RtcClock.h - clock source selection, tick, drift arithmetic and RTC chip record

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RTC_CLOCK_H
#define RTC_CLOCK_H

#include <stdint.h>
#include <stdlib.h>
#include "NTPTimeBase.h"

#define RTC_STRATUM_GPS       1               // GPS with fix
#define RTC_STRATUM_GPS_PPS   2               // GPS PPS without fix
#define RTC_STRATUM_GPS_FREE  3               // GPS serial message only, no fix and no PPS
#define RTC_STRATUM_CHIP      4               // RTC chip
#define RTC_STRATUM_NONE     16               // Unsynchronized
#define RTC_STRATUM_UPSTREAM(s)  (((s) > 0 && (s) < 15) ? (s) + 1 : 3)   // Upstream stratum + 1

#define RTC_PHASE_NONE         0              // Whole seconds only
#define RTC_PHASE_EDGE         1              // GPS PPS edge
#define RTC_PHASE_REGISTER     2              // RTC chip seconds register step
#define RTC_PHASE_COLD         3              // RTC chip read at boot, step not seen yet
#define RTC_PHASE_NETWORK      4              // Upstream NTP
#define RTC_PHASE_MANUAL       5              // Set by command Time

#define RTC_TICK_MAX_CATCHUP   30             // Max seconds a delayed tick catches up
#define RTC_SOURCE_STALE_SECS   3             // Seconds without update before a GPS source is dropped
#define RTC_SYNC_AGE_UPSTREAM 3600            // Upstream NTP poll interval, s
#define RTC_HOLDOVER_KEEP_SECS 7200           // Holdover with measured drift beats GPS serial message timing for this long, s

// Current clock source
struct RtcHolder {
  uint8_t stratum;
  bool from_upstream;
  bool drift_known;
  uint32_t since_sync;                        // Seconds since its last update
  uint32_t since_traceable;                   // Seconds since the last stratum 1 update, UINT32_MAX = never
};

enum RtcArbVerdict : uint8_t { RTC_ARB_TAKE, RTC_ARB_WORSE, RTC_ARB_UPSTREAM_HOLDS, RTC_ARB_WAIT_FOR_EDGE };

// May a source set the clock? At equal or better stratum, or once the holder is older than max_age.
// Upstream at equal stratum yields only to an edge or NTP; with a PPS pin messages wait for the edge
inline RtcArbVerdict RtcArbitrate(const RtcHolder &h, uint8_t stratum, uint32_t max_age,
                                  uint8_t phase_source, bool pps_expected) {
  if ((stratum > h.stratum) && (h.since_sync <= max_age)) { return RTC_ARB_WORSE; }
  bool upstream_holds = h.from_upstream && (h.since_sync <= RTC_SYNC_AGE_UPSTREAM);
  if ((stratum == h.stratum) && upstream_holds &&
      (RTC_PHASE_NETWORK != phase_source) && (RTC_PHASE_EDGE != phase_source)) { return RTC_ARB_UPSTREAM_HOLDS; }
  bool holdover_holds = (h.stratum >= RTC_STRATUM_CHIP) && (h.stratum < RTC_STRATUM_NONE) &&
                        !h.from_upstream && h.drift_known && (h.since_traceable <= RTC_HOLDOVER_KEEP_SECS);
  if ((stratum <= RTC_STRATUM_GPS_FREE) && (RTC_PHASE_NONE == phase_source) &&
      pps_expected && (upstream_holds || holdover_holds)) { return RTC_ARB_WAIT_FOR_EDGE; }
  return RTC_ARB_TAKE;
}

// GPS source without update for RTC_SOURCE_STALE_SECS
inline bool RtcSourceStale(const RtcHolder &h) {
  return (h.stratum < RTC_STRATUM_CHIP) && !h.from_upstream && (h.since_sync > RTC_SOURCE_STALE_SECS);
}

// Drift correction in us, the remainder below 1 us stays in frac_ppb. Positive = micros() runs fast
inline int32_t RtcDriftUs(int32_t* frac_ppb, int32_t add_ppb) {
  *frac_ppb += add_ppb;
  int32_t us = *frac_ppb / 1000;
  *frac_ppb -= us * 1000;
  return us;
}

// micros() count of secs seconds at drift ppb
inline int32_t RtcSecondsUs(int32_t secs, int32_t ppb) { return secs * 1000000 + (int32_t)(((int64_t)secs * ppb) / 1000); }

// Whole seconds nearest to delta_us (< 35 min) at drift ppb, off_us = the remainder
inline int32_t RtcNearestSecond(int32_t delta_us, int32_t ppb, int32_t* off_us) {
  int32_t d = delta_us - (int32_t)(((int64_t)delta_us * ppb) / 1000000000);
  int32_t secs = (d >= 0) ? ((d + 500000) / 1000000) : ((d - 500000) / 1000000);
  if (off_us) { *off_us = delta_us - RtcSecondsUs(secs, ppb); }
  return secs;
}

// Second a message arrived in, from a chip step at flip_us (second flip_utc): the one that started at or
// before arrival_us, whatever the message's delay. 0 = within guard_us of a step, start_us = its start
inline uint32_t RtcSecondOfArrival(uint32_t arrival_us, uint32_t flip_us, uint32_t flip_utc, int32_t ppb,
                                   uint32_t guard_us, uint32_t* start_us) {
  int32_t r;
  int32_t secs = RtcNearestSecond((int32_t)(arrival_us - 500000 - flip_us), ppb, &r);
  if ((uint32_t)abs(r) + guard_us > 500000) { return 0; }
  *start_us = arrival_us - 500000 - (uint32_t)r;
  return flip_utc + secs;
}

// Whole seconds in elapsed_ms, rest_ms = the remainder. NtpStamp() does the same in us
inline uint32_t RtcCarrySeconds(uint32_t elapsed_ms, uint32_t* rest_ms) {
  uint32_t seconds = 0;
  while ((elapsed_ms >= 1000) && (seconds < RTC_TICK_MAX_CATCHUP)) {
    elapsed_ms -= 1000;
    seconds++;
  }
  if (rest_ms) { *rest_ms = elapsed_ms; }
  return seconds;
}

// One tick: whole seconds since millis_b; micros_b moves on with the drift, millis_b follows it to the ms
inline uint32_t RtcTickAdvance(uint32_t now_ms, int32_t ppb, uint32_t* millis_b, uint32_t* micros_b,
                               int32_t* frac_ppb, int32_t* lag_us, uint32_t* elapsed_ms) {
  uint32_t secs = RtcCarrySeconds(now_ms - *millis_b, elapsed_ms);
  int32_t drift_us = RtcDriftUs(frac_ppb, (int32_t)secs * ppb);
  *lag_us += drift_us;
  int32_t lag_ms = *lag_us / 1000;
  if (lag_ms > (int32_t)*elapsed_ms) { lag_ms = (int32_t)*elapsed_ms; }
  *lag_us -= lag_ms * 1000;
  *elapsed_ms -= lag_ms;
  *millis_b = now_ms - *elapsed_ms;
  *micros_b += (secs * 1000000) + (uint32_t)drift_us;
  return secs;
}

// RTC chip record in 22 unused alarm bits, written by RtcAuxStore() and read by RtcAuxLoad()
//   bits  0-11  time the chip was last set, 128 s units, 0 = not set
//   bits 12-18  chip rate, ppm, signed; -64 = measured, below 0.5 ppm
//   bit  19     0 = set within RTC_HOLDOVER_PLACE_US, 1 = within RTC_HOLDOVER_PLACE_FAR_US
//   bits 20-21  check
#define RTC_AUX_UNIT_S       128
#define RTC_AUX_RATE_PPB     1000
#define RTC_AUX_TIME_BITS    12
#define RTC_AUX_RATE_BITS    7
#define RTC_AUX_TIME_MASK    ((1UL << RTC_AUX_TIME_BITS) - 1)
#define RTC_AUX_RATE_MASK    ((1UL << RTC_AUX_RATE_BITS) - 1)
#define RTC_AUX_RATE_MAX     ((1L << (RTC_AUX_RATE_BITS - 1)) - 1)
#define RTC_AUX_RATE_SMALL   (-RTC_AUX_RATE_MAX - 1)    // Rate measured, rounds to 0
#define RTC_AUX_CLASS_BIT    (RTC_AUX_TIME_BITS + RTC_AUX_RATE_BITS)
#define RTC_AUX_PAYLOAD_BITS (RTC_AUX_CLASS_BIT + 1)
#define RTC_AUX_CHECK_MASK   0x03
#define RTC_AUX_PAYLOAD_MASK ((1UL << RTC_AUX_PAYLOAD_BITS) - 1)

// Check bits; the XOR makes all-zero registers invalid
inline uint32_t RtcAuxCheck(uint32_t payload) {
  return (((payload * 0x9E3779B1UL) >> 30) ^ 0x01) & RTC_AUX_CHECK_MASK;
}

// Record to write, rate = stored ppm; slow_ppb 0 = not measured. Without time and rate it is invalid
inline uint32_t RtcAuxPack(bool placed_known, uint32_t placed_utc, int32_t slow_ppb, bool far, int32_t *rate) {
  int32_t half = RTC_AUX_RATE_PPB / 2;
  int32_t r = (slow_ppb + ((slow_ppb >= 0) ? half : -half)) / RTC_AUX_RATE_PPB;
  if (r > RTC_AUX_RATE_MAX) { r = RTC_AUX_RATE_MAX; }
  if (r < -RTC_AUX_RATE_MAX) { r = -RTC_AUX_RATE_MAX; }
  *rate = r;
  if (!r && slow_ppb) { r = RTC_AUX_RATE_SMALL; }
  if (!placed_known && !r) { return ((RtcAuxCheck(0) + 1) & RTC_AUX_CHECK_MASK) << RTC_AUX_PAYLOAD_BITS; }
  uint32_t when = placed_known ? ((placed_utc / RTC_AUX_UNIT_S) & RTC_AUX_TIME_MASK) : 0;
  uint32_t payload = when | (((uint32_t)r & RTC_AUX_RATE_MASK) << RTC_AUX_TIME_BITS) |
                     ((uint32_t)(placed_known && far) << RTC_AUX_CLASS_BIT);
  return payload | (RtcAuxCheck(payload) << RTC_AUX_PAYLOAD_BITS);
}

enum RtcAuxRead : uint8_t { RTC_AUX_FOREIGN, RTC_AUX_RATE_ONLY, RTC_AUX_PLACED };

// Decode a record. placed_utc and far are set only for RTC_AUX_PLACED
inline uint8_t RtcAuxUnpack(uint32_t v, uint32_t now, uint32_t *placed_utc, int32_t *slow_ppb, bool *far = nullptr) {
  uint32_t payload = v & RTC_AUX_PAYLOAD_MASK;
  int32_t rate = (int32_t)((payload >> RTC_AUX_TIME_BITS) & RTC_AUX_RATE_MASK);
  if (rate > RTC_AUX_RATE_MAX) { rate -= (1L << RTC_AUX_RATE_BITS); }   // Sign extend
  if (!payload || !rate || (((v >> RTC_AUX_PAYLOAD_BITS) & RTC_AUX_CHECK_MASK) != RtcAuxCheck(payload))) { return RTC_AUX_FOREIGN; }
  *slow_ppb = (RTC_AUX_RATE_SMALL == rate) ? 1 : rate * RTC_AUX_RATE_PPB;   // 1 ppb: measured, near 0
  if (!(payload & RTC_AUX_TIME_MASK)) { return RTC_AUX_RATE_ONLY; }
  uint32_t now_units = now / RTC_AUX_UNIT_S;
  uint32_t ago = (now_units - payload) & RTC_AUX_TIME_MASK;
  *placed_utc = (now_units - ago) * RTC_AUX_UNIT_S;
  if (far) { *far = (payload >> RTC_AUX_CLASS_BIT) & 1; }
  return RTC_AUX_PLACED;
}

// Edge spaced a whole number of pulses (1 to max_skip) within tol_us; a late seen edge is refused
inline bool RtcEdgeSpacedRight(uint32_t spacing, uint32_t pulses, int32_t drift_ppb, uint32_t max_skip, uint32_t tol_us) {
  int32_t off = (int32_t)(spacing - pulses * (1000000 + drift_ppb / 1000));
  return (pulses >= 1) && (pulses <= max_skip) && ((uint32_t)abs(off) < tol_us);
}

// NTP reply values
#define RTC_PRECISION_NETWORK  -19            // Network delay is in the root delay
#define RTC_DRIFT_RESIDUAL_PPB 3000           // Crystal drift left after correction
#define RTC_DRIFT_RECEIVER_PPB 500            // GPS receiver PPS without fix
#define RTC_MAX_POLL_SECS     1024            // Longest NTP poll: a step is announced this long, a sync this recent is stepped by half, s
#define RTC_EPOCH_MESSAGE_US  450000          // GPS serial message timing
#define RTC_EPOCH_CHIP_US    1000000          // RTC chip, whole seconds

// RTC chip error after a power loss: set + read error + 15.5 ppm (temperature, rounding) x age
#define RTC_HOLDOVER_PLACE_US   5000          // Set from GPS PPS or a LAN NTP server
#define RTC_HOLDOVER_PLACE_FAR_US 20000       // Set from an internet NTP server
#define RTC_HOLDOVER_CATCH_US    200          // Reading error of the fine catch
#define RTC_HOLDOVER_TRUST_S   345600         // Record valid if settings were saved within 4 days
#define RTC_HOLDOVER_REPLACE_S 3000           // Set the chip again at the hourly check: keeps the claim small

// ESP drift in one chip byte, 0.5 ppm steps to +-31.5 ppm, 0 = none; odd only (BM8563 stores one less)
inline uint8_t RtcDriftToByte(int32_t ppb) {
  int32_t q = (ppb + ((ppb >= 0) ? 250 : -250)) / 500;
  q = (q > 63) ? 63 : ((q < -63) ? -63 : q);
  return (uint8_t)(((q + 64) << 1) | 1);
}

inline int32_t RtcDriftFromByte(uint8_t b) { return (b >> 1) ? ((int32_t)(b >> 1) - 64) * 500 : 0; }

inline uint32_t RtcChipHoldoverUs(uint32_t age_s, uint32_t catch_us, bool far = false) {
  if (age_s > 64000) { return RTC_EPOCH_CHIP_US; }                  // 1 s reached
  uint32_t us = (far ? RTC_HOLDOVER_PLACE_FAR_US : RTC_HOLDOVER_PLACE_US) + catch_us + ((age_s * 31) / 2);
  return (us < RTC_EPOCH_CHIP_US) ? us : RTC_EPOCH_CHIP_US;
}
#define RTC_EPOCH_NAMED_US    500000          // PPS edge on a second from upstream NTP
// Time came from the RTC chip (stratum alone cannot tell: upstream stratum 3 is also 4)
#define RtcPhaseFromChip(p)  ((RTC_PHASE_REGISTER == (p)) || (RTC_PHASE_COLD == (p)))

// Fill the NTP reply values from Rtc. chip_stratum 0 = no chip. Call under RTC_PAIR_LOCK
template <class R>
inline void RtcDescribe(const R &rtc, uint32_t uptime, uint8_t chip_stratum, uint32_t up_delay_us,
                        uint32_t up_disp_us, NtpTimeBase *base) {
  base->stratum = rtc.stratum;
  base->refId = rtc.ntp_refid;
  base->driftPpb = rtc.drift_ppb ? RTC_DRIFT_RESIDUAL_PPB : NTP_DRIFT_UNKNOWN_PPB;
  uint32_t ref = rtc.from_upstream ? rtc.last_sync : rtc.last_traceable;
  uint32_t since = ref ? (uptime - ref) : uptime;
  base->holdoverSecs = (rtc.stratum < RTC_STRATUM_NONE) ? since : 0;
  base->stepMs = ((uptime - rtc.step_uptime) <= RTC_MAX_POLL_SECS) ? rtc.step_ms : 0;
  base->rootDelayUs = 0;

  base->precision = -10;                             // 1 ms
  switch (rtc.from_upstream ? 0 : rtc.stratum) {
    case RTC_STRATUM_GPS: {
      bool edge = (RTC_PHASE_EDGE == rtc.phase_source);
      base->source = edge ? NTP_SRC_PPS : NTP_SRC_GPS;
      if (edge) {
        base->baseDispersionUs = 1000;
        base->precision = -19;                          // 2 us
      } else if (rtc.carried_edge) {                   // PPS lost, carried by serial messages
        base->baseDispersionUs = 2000;
        base->holdoverSecs = rtc.carried_secs;          // Seconds since the last PPS edge
      } else if (rtc.chip_placed_us) {
        base->baseDispersionUs = rtc.chip_placed_us;
        base->precision = -13;                          // Second placed by the RTC chip, 120 us
      } else {
        base->baseDispersionUs = RTC_EPOCH_MESSAGE_US;
      }
      break;
    }
    case RTC_STRATUM_GPS_PPS:
      base->source = NTP_SRC_PPS_FREE;
      base->baseDispersionUs = 1000;
      base->precision = -19;
      base->driftPpb = RTC_DRIFT_RECEIVER_PPB;
      break;
    case RTC_STRATUM_GPS_FREE:
      base->source = NTP_SRC_GPS_FREE;
      base->baseDispersionUs = rtc.chip_placed_us ? rtc.chip_placed_us : RTC_EPOCH_MESSAGE_US;
      if (rtc.chip_placed_us) { base->precision = -13; }  // Second placed by the RTC chip
      break;
    case RTC_STRATUM_NONE:
      base->source = NTP_SRC_NONE;
      base->baseDispersionUs = 1000000;
      break;
    default:
      base->source = rtc.from_upstream ? NTP_SRC_UPSTREAM : NTP_SRC_CHIP;
      if ((NTP_SRC_CHIP == base->source) && rtc.from_chip && chip_stratum) { base->stratum = chip_stratum; }
      if (RTC_PHASE_COLD == rtc.phase_source) {        // RTC chip step not seen yet: no reply
        base->source = NTP_SRC_NONE;
        base->stratum = RTC_STRATUM_NONE;
      }
      switch (rtc.phase_source) {
        case RTC_PHASE_REGISTER: base->precision = -13; break;   // 120 us
        case RTC_PHASE_COLD:     base->precision = -7;  break;   // 8 ms
        case RTC_PHASE_EDGE:     base->precision = -19; break;   // 2 us
        case RTC_PHASE_NETWORK:  base->precision = RTC_PRECISION_NETWORK; break;
      }
      if (RtcPhaseFromChip(rtc.phase_source)) {
        base->baseDispersionUs = rtc.chip_holdover_us ? rtc.chip_holdover_us : RTC_EPOCH_CHIP_US;
      } else if (RTC_PHASE_EDGE == rtc.phase_source) {
        base->baseDispersionUs = rtc.from_chip ? RTC_EPOCH_CHIP_US : RTC_EPOCH_NAMED_US;
      } else {
        base->baseDispersionUs = (RTC_PHASE_MANUAL == rtc.phase_source) ? RTC_EPOCH_CHIP_US : 0;
      }
      // Upstream NTP: root delay includes our round trip. Without a phase the second can be 1 s off
      if (rtc.from_upstream) {
        base->rootDelayUs = up_delay_us;
        base->baseDispersionUs += up_disp_us;
        if (RTC_PHASE_NETWORK != rtc.phase_source) { base->baseDispersionUs += RTC_EPOCH_CHIP_US; }
      }
      break;
  }
}

#endif // RTC_CLOCK_H
