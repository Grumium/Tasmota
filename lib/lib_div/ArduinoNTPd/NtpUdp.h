/*
  NtpUdp.h - raw lwIP UDP for the NTP server and client on ESP32

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

// Receive and send are stamped with micros() in the tcpip thread, without the main loop latency
#ifndef NTP_UDP_H
#define NTP_UDP_H

#ifdef ESP32
#include <stdint.h>
extern "C" {
#include "lwip/udp.h"
#include "lwip/pbuf.h"
}

struct NtpUdp;
// Called in the tcpip thread, must free p. No logging
typedef void (*NtpUdpRecvFn)(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                             uint16_t port, uint32_t rx_us);

struct NtpUdp {
  struct udp_pcb *pcb = nullptr;
  NtpUdpRecvFn fn = nullptr;
  void *arg = nullptr;               // Owner data for fn
};

// Client receive buffer: last 48-byte packet from port `from` with its receive time.
// Set u->fn = NtpUdpKeepLast and u->arg to this buffer before NtpUdpOpen()
struct NtpUdpLast {
  uint8_t data[48];
  uint32_t rx_us;
  uint32_t rx_ms;
  uint16_t from = 123;
  bool ready = false;
};
void NtpUdpKeepLast(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port,
                    uint32_t rx_us);
bool NtpUdpTakeLast(NtpUdpLast *l, uint8_t *data, uint32_t *rx_us, uint32_t *rx_ms);   // false = nothing received
void NtpUdpDropLast(NtpUdpLast *l);                  // Clear before sending a request

bool NtpUdpOpen(NtpUdp *u, uint16_t port);           // port 0 = any free port
void NtpUdpClose(NtpUdp *u);
// Send from outside the tcpip thread. tx_us = micros() just before udp_sendto()
bool NtpUdpSend(NtpUdp *u, const ip_addr_t *addr, uint16_t port, const void *data, uint16_t len,
                uint32_t *tx_us);
#endif  // ESP32

#endif  // NTP_UDP_H
