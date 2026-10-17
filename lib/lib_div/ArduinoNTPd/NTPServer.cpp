/*
 * File: NTPServer.cpp
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */


#include <WiFiUdp.h>

#include "NTPPacket.h"
#include "NTPServer.h"
#include "NtpWire.h"

#ifdef ESP32
#include "NtpUdp.h"
#endif  // ESP32

#define NTP_PORT 123
#ifdef ESP32
// Answer a request in the tcpip thread, rxUs is its receive time. No logging
static void NtpRecv(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port,
                    uint32_t rxUs)
{
  NtpServer *self = (NtpServer *)u->arg;
  NtpPacket packet;
  if (p && self && self->source_ &&
      NtpRequestOk(packet, pbuf_copy_partial(p, &packet, NtpPacket::PACKET_SIZE, 0))) {
    NtpTimeBase base;
    uint32_t utc, boundaryUs;
    if (self->source_(&base, &utc, &boundaryUs)) {
      // A new second may have started after the packet arrived: use the previous one
      int32_t since = (int32_t)(rxUs - boundaryUs);
      if (since < 0) { utc--; since += 1000000; }
      NtpFillReply(packet, utc, (uint32_t)since, base);
      struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, NtpPacket::PACKET_SIZE, PBUF_RAM);
      if (reply) {
        NtpSetTransmit(packet, utc, (uint32_t)since + (micros() - rxUs), base.tickPpb);
        packet.swapEndian();
        memcpy(reply->payload, packet.packet(), NtpPacket::PACKET_SIZE);
        udp_sendto(pcb, reply, addr, port);
        pbuf_free(reply);
        if (self->sent_) { self->sent_(base); }
      }
    }
  }
  if (p) { pbuf_free(p); }
}
#endif  // ESP32

bool NtpServer::beginListening()
{
#ifdef ESP32
  if (source_) {
    udp_.fn = NtpRecv;
    udp_.arg = this;
    return NtpUdpOpen(&udp_, NTP_PORT);
  }
#endif  // ESP32
  if (timeServerPort_.begin(NTP_PORT)){
    return true;
  }
  return false;
}

void NtpServer::stopListening()
{
#ifdef ESP32
  if (udp_.pcb) {
    NtpUdpClose(&udp_);
    return;
  }
#endif  // ESP32
  timeServerPort_.stop();
}

bool NtpServer::processOneRequest(uint32_t utc, uint32_t microsecs, const NtpTimeBase &base)
{
  uint32_t elapsed = micros() - microsecs;

  bool processed = false;

  int packetDataSize = timeServerPort_.parsePacket();
  if (packetDataSize > 0)
  {
      NtpPacket packet;
      timeServerPort_.read((char*)&packet, NtpPacket::PACKET_SIZE);
      if (!NtpRequestOk(packet, packetDataSize)) { return false; }

      NtpFillReply(packet, utc, elapsed, base);

      NtpSetTransmit(packet, utc, micros() - microsecs, base.tickPpb);

      // Now transmit the response to the client.
      packet.swapEndian();

      timeServerPort_.beginPacket(timeServerPort_.remoteIP(), timeServerPort_.remotePort());
      timeServerPort_.write((const uint8_t *)packet.packet(), NtpPacket::PACKET_SIZE);
      timeServerPort_.endPacket();

      processed = true;
  }

  return processed;
}
