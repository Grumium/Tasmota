/*
 * File: NTPServer.h
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 *         Jan-David Förster
 * License: New BSD License
 */

#ifndef NTP_SERVER_H
#define NTP_SERVER_H

#include "NTPTimeBase.h"
#ifdef ESP32
#include "NtpUdp.h"
#endif

// Supplies the time quality, the UTC second and micros() at its start. false = do not answer
typedef bool (*NtpSourceFn)(NtpTimeBase* base, uint32_t* utc, uint32_t* boundaryUs);
typedef void (*NtpSentFn)(const NtpTimeBase &base);

class NtpServer
{
public:
#ifdef ESP32
    NtpServer(WiFiUDP Port) { (void)Port; }   // ESP32 answers from the lwIP callback, no WiFiUDP
#else
    NtpServer(WiFiUDP Port) { timeServerPort_ = Port; }
#endif

    void setSource(NtpSourceFn source, NtpSentFn sent)
    {
        source_ = source;
        sent_ = sent;
    }

    bool beginListening(void);
    void stopListening(void);        // Release port 123

#ifdef ESP32
    void poll(void) {}               // Replies are sent in NtpRecv()
#else
    void poll(void);                 // Answer or drop one request with the time from source_
#endif

    NtpSourceFn source_ = nullptr;   // Public for NtpRecv()
    NtpSentFn sent_ = nullptr;
#ifdef ESP32
    NtpUdp udp_;
#else
private:
    WiFiUDP timeServerPort_;
#endif
};


// Implementation in this header, compiled with the firmware
#include <WiFiUdp.h>
#include "NTPPacket.h"
#include "NtpWire.h"

#define NTP_PORT 123
#ifdef ESP32
// Answer a request in the tcpip thread, rxUs is its receive time. No logging
static inline void NtpRecv(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port,
                    uint32_t rxUs)
{
  NtpServer *self = (NtpServer *)u->arg;
  NtpPacket packet;
  if (NtpRequestOk(packet, pbuf_copy_partial(p, &packet, NtpPacket::PACKET_SIZE, 0))) {
    NtpTimeBase base;
    uint32_t utc, boundaryUs;
    if (self->source_(&base, &utc, &boundaryUs)) {
      // A new second may have started after the packet arrived: use the previous one
      int32_t since = (int32_t)(rxUs - boundaryUs);
      if (since < 0) { utc--; since += 1000000; }
      NtpFillReply(packet, utc, (uint32_t)since, base);
      struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, NtpPacket::PACKET_SIZE, PBUF_RAM);
      if (reply) {
        NtpSetTransmit(packet, utc, (uint32_t)since + (micros() - rxUs));
        packet.swapEndian();
        memcpy(reply->payload, packet.packet(), NtpPacket::PACKET_SIZE);
        err_t err = udp_sendto(pcb, reply, addr, port);
        pbuf_free(reply);
        if ((ERR_OK == err) && self->sent_) { self->sent_(base); }   // A step announcement went out
      }
    }
  }
  pbuf_free(p);
}
#endif  // ESP32

inline bool NtpServer::beginListening()
{
#ifdef ESP32
  if (!source_) { return false; }
  udp_.fn = NtpRecv;
  udp_.arg = this;
  return NtpUdpOpen(&udp_, NTP_PORT);
#else
  return timeServerPort_.begin(NTP_PORT);
#endif  // ESP32
}

inline void NtpServer::stopListening()
{
#ifdef ESP32
  NtpUdpClose(&udp_);
#else
  timeServerPort_.stop();
#endif  // ESP32
}

#ifndef ESP32
// Called on every loop pass: asks source_ for the time only when a request is waiting
inline void NtpServer::poll()
{
  if (!source_) { return; }
  int packetDataSize = timeServerPort_.parsePacket();
  if (packetDataSize <= 0) { return; }
  NtpTimeBase base;
  uint32_t utc, boundaryUs;
  // No time: the next parsePacket() drops the request, a late reply is worse
  if (!source_(&base, &utc, &boundaryUs)) { return; }
  uint32_t elapsed = micros() - boundaryUs;
  NtpPacket packet;
  timeServerPort_.read((char*)&packet, NtpPacket::PACKET_SIZE);
  if (!NtpRequestOk(packet, packetDataSize)) { return; }
  NtpFillReply(packet, utc, elapsed, base);
  NtpSetTransmit(packet, utc, micros() - boundaryUs);
  packet.swapEndian();
  timeServerPort_.beginPacket(timeServerPort_.remoteIP(), timeServerPort_.remotePort());
  timeServerPort_.write((const uint8_t *)packet.packet(), NtpPacket::PACKET_SIZE);
  timeServerPort_.endPacket();
  if (sent_) { sent_(base); }
}
#endif  // ESP32

#endif // NTP_SERVER_H
