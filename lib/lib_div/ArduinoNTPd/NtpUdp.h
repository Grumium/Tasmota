/*
  NtpUdp.h - raw lwIP UDP for the NTP server and client on ESP32, stamped with micros() in the tcpip
  thread, without the main loop latency

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef NTP_UDP_H
#define NTP_UDP_H

#ifdef ESP32
#include <stdint.h>
extern "C" {
#include "lwip/udp.h"
#include "lwip/pbuf.h"
}

struct NtpUdp;
typedef void (*NtpUdpRecvFn)(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                             uint16_t port, uint32_t rx_us);   // In the tcpip thread: free p, no logging

struct NtpUdp {
  struct udp_pcb *pcb = nullptr;
  NtpUdpRecvFn fn = nullptr;
  void *arg = nullptr;               // Owner data for fn
};

// Client receive buffer, last packet from port 123: u->fn = NtpUdpKeepLast, u->arg = buffer
struct NtpUdpLast {
  uint8_t data[48];
  uint32_t rx_us;
  bool ready = false;
};
void NtpUdpKeepLast(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port,
                    uint32_t rx_us);
bool NtpUdpTakeLast(NtpUdpLast *l, uint8_t *data, uint32_t *rx_us);   // false = nothing received

bool NtpUdpOpen(NtpUdp *u, uint16_t port);           // port 0 = any free port
void NtpUdpClose(NtpUdp *u);
// Send from outside the tcpip thread. tx_us = micros() just before udp_sendto()
bool NtpUdpSend(NtpUdp *u, const ip_addr_t *addr, uint16_t port, const void *data, uint16_t len,
                uint32_t *tx_us);
#endif  // ESP32

#endif  // NTP_UDP_H
