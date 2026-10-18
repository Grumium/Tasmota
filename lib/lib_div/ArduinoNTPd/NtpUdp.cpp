/*
  NtpUdp.cpp - raw lwIP UDP for the NTP server and client on ESP32

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifdef ESP32
#include <Arduino.h>
#include <string.h>
#include "NtpUdp.h"
#include "lwip/priv/tcpip_priv.h"

// Timestamp first, then call the owner
static void NtpUdpRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  uint32_t rx_us = micros();
  NtpUdp *u = (NtpUdp *)arg;
  if (u && u->fn) {
    u->fn(u, pcb, p, addr, port, rx_us);
  } else if (p) {
    pbuf_free(p);
  }
}

// Calls from outside the tcpip thread go through tcpip_api_call (no lwIP core locking)
struct NtpUdpMsg {
  struct tcpip_api_call_data call;
  NtpUdp *u;
  uint16_t port;
  const ip_addr_t *addr;
  const void *data;
  uint16_t len;
  uint32_t tx_us;
  err_t err;
};

static err_t NtpUdpOpenApi(struct tcpip_api_call_data *api)
{
  NtpUdpMsg *msg = (NtpUdpMsg *)api;
#if LWIP_IPV6
  struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
#else
  struct udp_pcb *pcb = udp_new();
#endif
  if (!pcb) { msg->err = ERR_MEM; return msg->err; }
  msg->err = udp_bind(pcb, IP_ANY_TYPE, msg->port);
  if (ERR_OK != msg->err) {
    udp_remove(pcb);
    return msg->err;
  }
  udp_recv(pcb, NtpUdpRecv, msg->u);
  msg->u->pcb = pcb;
  return ERR_OK;
}

static err_t NtpUdpCloseApi(struct tcpip_api_call_data *api)
{
  NtpUdpMsg *msg = (NtpUdpMsg *)api;
  if (msg->u->pcb) {
    udp_recv(msg->u->pcb, NULL, NULL);
    udp_remove(msg->u->pcb);
    msg->u->pcb = nullptr;
  }
  return ERR_OK;
}

static err_t NtpUdpSendApi(struct tcpip_api_call_data *api)
{
  NtpUdpMsg *msg = (NtpUdpMsg *)api;
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, msg->len, PBUF_RAM);
  if (!p) { msg->err = ERR_MEM; return msg->err; }
  memcpy(p->payload, msg->data, msg->len);
  msg->tx_us = micros();                          // Send time
  msg->err = udp_sendto(msg->u->pcb, p, msg->addr, msg->port);
  pbuf_free(p);
  return msg->err;
}

// Receive buffer is shared with the main loop on the other core
static portMUX_TYPE ntp_udp_last_mux = portMUX_INITIALIZER_UNLOCKED;

void NtpUdpKeepLast(NtpUdp *u, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port,
                    uint32_t rx_us)
{
  (void)pcb; (void)addr;
  NtpUdpLast *l = (NtpUdpLast *)u->arg;
  if (p && l && (123 == port) && (p->tot_len >= sizeof(l->data))) {
    portENTER_CRITICAL(&ntp_udp_last_mux);
    pbuf_copy_partial(p, l->data, sizeof(l->data), 0);
    l->rx_us = rx_us;
    l->ready = true;                              // Overwrites an older packet
    portEXIT_CRITICAL(&ntp_udp_last_mux);
  }
  if (p) { pbuf_free(p); }
}

bool NtpUdpTakeLast(NtpUdpLast *l, uint8_t *data, uint32_t *rx_us)
{
  bool got = false;
  portENTER_CRITICAL(&ntp_udp_last_mux);
  if (l->ready) {
    memcpy(data, l->data, sizeof(l->data));
    *rx_us = l->rx_us;
    l->ready = false;
    got = true;
  }
  portEXIT_CRITICAL(&ntp_udp_last_mux);
  return got;
}

bool NtpUdpOpen(NtpUdp *u, uint16_t port)
{
  if (u->pcb) { return true; }
  NtpUdpMsg msg;
  msg.u = u;
  msg.port = port;
  tcpip_api_call(NtpUdpOpenApi, (struct tcpip_api_call_data *)&msg);
  return (ERR_OK == msg.err) && u->pcb;
}

void NtpUdpClose(NtpUdp *u)
{
  if (!u->pcb) { return; }
  NtpUdpMsg msg;
  msg.u = u;
  tcpip_api_call(NtpUdpCloseApi, (struct tcpip_api_call_data *)&msg);
}

bool NtpUdpSend(NtpUdp *u, const ip_addr_t *addr, uint16_t port, const void *data, uint16_t len,
                uint32_t *tx_us)
{
  if (!u->pcb) { return false; }
  NtpUdpMsg msg;
  msg.u = u;
  msg.addr = addr;
  msg.port = port;
  msg.data = data;
  msg.len = len;
  msg.tx_us = 0;
  tcpip_api_call(NtpUdpSendApi, (struct tcpip_api_call_data *)&msg);
  if (tx_us) { *tx_us = msg.tx_us; }
  return ERR_OK == msg.err;
}
#endif  // ESP32
