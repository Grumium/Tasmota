/*
  NtpWire.h - NTP reply content: timestamps, short format, request check

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef NTP_WIRE_H
#define NTP_WIRE_H

#include <string.h>
#include "NTPPacket.h"
#include "NTPTimeBase.h"

#define NTP_TIMESTAMP_DIFF     (2208988800)    // 1900 to 1970 in seconds

// NTP timestamp from a second and the us since its start, corrected for the crystal drift.
// us * 4294 + us * 967 / 1000 avoids a 64-bit divide. Same carry as RtcCarrySeconds().
static inline uint64_t NtpStamp(uint32_t utc, uint32_t sinceBoundaryUs, int32_t tickPpb)
{
  if (tickPpb && (sinceBoundaryUs < 2000000) && (tickPpb > -1000000) && (tickPpb < 1000000)) {
    int32_t corrUs = ((int32_t)(sinceBoundaryUs / 1000) * tickPpb) / 1000000;
    sinceBoundaryUs -= corrUs;                    // Positive = crystal runs fast
  }
  while (sinceBoundaryUs >= 1000000) {
    sinceBoundaryUs -= 1000000;
    utc++;
  }
  uint32_t fraction = (sinceBoundaryUs * 4294u) + ((sinceBoundaryUs * 967u) / 1000u);
  return ((uint64_t)(utc + NTP_TIMESTAMP_DIFF) << 32) | fraction;
}

// us to NTP short format, limited to 60 s. Split at 900 ms to stay in 32 bits
#define NTP_SHORT_60S  ((60000UL * 8389u) >> 7)

static inline uint32_t NtpShortFromUs(uint32_t microsecs)
{
  if (microsecs > 60000000UL) { return NTP_SHORT_60S; }
  if (microsecs < 900000UL)   { return (microsecs * 4295u) >> 16; }
  return ((microsecs >> 6) * 4295u) >> 10;
}

// Accept client (mode 3) and symmetric active (mode 1) requests only
static inline bool NtpRequestOk(const NtpPacket &request, uint32_t len)
{
  if (len < NtpPacket::PACKET_SIZE) { return false; }   // Too short
  unsigned int mode = request.mode();
  unsigned int version = request.versionNumber();
  return ((3 == mode) || (1 == mode)) && (version >= 1) && (version <= 4);
}

static inline void NtpFillReply(NtpPacket &packet, uint32_t utc, uint32_t sinceBoundaryUs, const NtpTimeBase &base)
{
  uint64_t recvStamp = NtpStamp(utc, sinceBoundaryUs, base.tickPpb);

  packet.swapEndian();
  packet.leapIndicator(0);            // No reply is sent without a time source
  packet.mode(4);                     // Server, version as in the request
  packet.stratum = base.stratum;
  packet.poll = 10; // 6-10 per RFC 5905.

  const char *id = NtpRefName(base);
  if (id) {
    memcpy(packet.referenceId, id, 4);            // Names are 4 bytes incl. the terminating zero
  } else {
    memcpy(packet.referenceId, &base.refId, 4);   // Network order
  }

  packet.precision = base.precision;
  packet.rootDelay = NtpShortFromUs(base.rootDelayUs);

  packet.rootDispersion = NtpShortFromUs(NtpDispersionUs(base));

  packet.referenceTimestampSeconds = (uint32_t)(recvStamp >> 32) - base.holdoverSecs;   // Last update of the source
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
