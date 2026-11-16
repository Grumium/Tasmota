/*
  NtpWire.h - NTP reply content: timestamps, short format, request check

  SPDX-FileCopyrightText: 2026 Jan-David Förster

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef NTP_WIRE_H
#define NTP_WIRE_H

#include <string.h>
#include "NTPPacket.h"
#include "NTPTimeBase.h"


// Included by NTPServer.h into the firmware: USE_NTP_SERVER (the clock) is known here. Without it the
// reply carries the source's fixed values, no drift, holdover or crystal correction

// NTP timestamp (seconds since 1900, then the fraction in 1/2^32 s) from the UTC second and
// the crystal us since it began, minus the crystal error; us past 1 s roll over (late tick).
// us * 2^32 / 10^6 = us * 4294.967296, done as us * 4294 + us * 967 / 1000 in 32 bits
#ifdef USE_NTP_SERVER
#define NTP_TICK_PPB(base)  ((base).tickPpb)
#else
#define NTP_TICK_PPB(base)  0                     // Crystal error not known
#endif  // USE_NTP_SERVER

static inline uint64_t NtpStamp(uint32_t utc, uint32_t sinceBoundaryUs, int32_t tickPpb)
{
#ifdef USE_NTP_SERVER
  if (tickPpb && (sinceBoundaryUs < 2000000) && (tickPpb > -1000000) && (tickPpb < 1000000)) {
    int32_t corrUs = ((int32_t)(sinceBoundaryUs / 1000) * tickPpb) / 1000000;
    sinceBoundaryUs -= corrUs;                    // Positive = crystal runs fast
  }
#else
  (void)tickPpb;                                  // Without the clock the crystal error is not known
#endif  // USE_NTP_SERVER
  while (sinceBoundaryUs >= 1000000) {
    sinceBoundaryUs -= 1000000;
    utc++;
  }
  uint32_t fraction = (sinceBoundaryUs * 4294u) + ((sinceBoundaryUs * 967u) / 1000u);
  return ((uint64_t)(utc + NTP_TIMESTAMP_DIFF) << 32) | fraction;
}

// us to NTP short format (16.16 s) = us * 4295 >> 16. That overflows 32 bits near 1 s,
// so from 900 ms on divide by 64 first; capped at 60 s
#define NTP_SHORT_60S  ((60000UL * 8389u) >> 7)

static inline uint32_t NtpShortFromUs(uint32_t microsecs)
{
  if (microsecs > 60000000UL) { return NTP_SHORT_60S; }
  if (microsecs < 900000UL)   { return (microsecs * 4295u) >> 16; }
  return ((microsecs >> 6) * 4295u) >> 10;
}

// Answer requests only: mode 3 = client, 1 = peer. Drop replies (4 server, 5 broadcast)
// and control queries (6, 7)
static inline bool NtpRequestOk(const NtpPacket &request, uint32_t len)
{
  if (len < NtpPacket::PACKET_SIZE) { return false; }   // Too short
  unsigned int mode = request.mode();
  unsigned int version = request.versionNumber();
  return ((3 == mode) || (1 == mode)) && (version >= 1) && (version <= 4);
}

static inline void NtpFillReply(NtpPacket &packet, uint32_t utc, uint32_t sinceBoundaryUs, const NtpTimeBase &base)
{
  uint64_t recvStamp = NtpStamp(utc, sinceBoundaryUs, NTP_TICK_PPB(base));

  packet.swapEndian();
  packet.leapIndicator(0);            // No reply is sent without a time source
  packet.mode((1 == packet.mode()) ? 2 : 4);   // Server, or passive to a peer; version as in the request
  packet.stratum = base.stratum;
  packet.poll = 10; // 6-10 per RFC 5905.

  const char *id = NtpRefName(base);
#ifdef USE_NTP_SERVER
  if (!id) {
    memcpy(packet.referenceId, &base.refId, 4);   // Network order
  } else
#endif  // USE_NTP_SERVER
  memcpy(packet.referenceId, id, 4);              // Names are 4 bytes incl. the terminating zero

  packet.precision = base.precision;

#ifdef USE_NTP_SERVER
  packet.rootDelay = NtpShortFromUs(base.rootDelayUs);
  packet.rootDispersion = NtpShortFromUs(NtpDispersionUs(base));
  packet.referenceTimestampSeconds = (uint32_t)(recvStamp >> 32) - base.holdoverSecs;   // Last update of the source
#else
  packet.rootDelay = 0;                           // Local source
  packet.rootDispersion = NtpShortFromUs(base.baseDispersionUs);   // Fixed values without the clock
  packet.referenceTimestampSeconds = (uint32_t)(recvStamp >> 32);
#endif  // USE_NTP_SERVER
  packet.referenceTimestampFraction = 0;
  packet.originTimestampSeconds = packet.transmitTimestampSeconds;
  packet.originTimestampFraction = packet.transmitTimestampFraction;
  packet.receiveTimestampSeconds = (uint32_t)(recvStamp >> 32);
  packet.receiveTimestampFraction = (uint32_t)recvStamp;
}

static inline void NtpSetTransmit(NtpPacket &packet, uint32_t utc, uint32_t sinceBoundaryUs, int32_t tickPpb)
{
  uint64_t transStamp = NtpStamp(utc, sinceBoundaryUs, tickPpb);
  packet.transmitTimestampSeconds = (uint32_t)(transStamp >> 32);
  packet.transmitTimestampFraction = (uint32_t)transStamp;
}

#endif // NTP_WIRE_H
