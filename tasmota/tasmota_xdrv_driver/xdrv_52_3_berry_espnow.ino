/*
  xdrv_52_3_berry_espnow.ino - Berry scripting language, ESP-NOW with a RAM-only WiFi hold

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_BERRY
#include <berry.h>

#ifdef USE_BERRY_ESPNOW

#include <esp_now.h>
#include <esp_wifi.h>
#include <deque>
#include <string>

/*********************************************************************************************\
 * ESP-NOW for Berry. Callbacks run in the WiFi task and only fill queues; Berry drains them.
 *
 * The WiFi hold lives in RAM only (BerryEspnowWifiHold): while set, support_tasmota.ino skips
 * WifiCheck(), so there is no WifiBegin, no scan and no WifiCheckIp restart. Settings are never
 * touched, unlike `Wifi 0/1` which writes flag4.network_wifi to flash.
\*********************************************************************************************/

bool BerryEspnowWifiHold = false;

static const size_t ESPNOW_RX_MAX = 24;
static const size_t ESPNOW_TX_MAX = 32;

struct EspnowRx { uint8_t src[6]; int8_t rssi; uint32_t ms; std::string data; };
struct EspnowTx { uint8_t dst[6]; bool ok; };

static struct {
  bool up = false;
  SemaphoreHandle_t lock = nullptr;
  std::deque<EspnowRx> rx;
  std::deque<EspnowTx> tx;
  uint32_t tx_ok = 0, tx_fail = 0, rx_count = 0, rx_dropped = 0;
} be_en;

static const uint8_t ESPNOW_BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Helpers as static members: the .ino prototype generator would hoist free functions
// above the includes, where the ESP-NOW types are not yet known.
struct BeEn {
  static void be_en_lock(void)   { xSemaphoreTake(be_en.lock, portMAX_DELAY); }
  static void be_en_unlock(void) { xSemaphoreGive(be_en.lock); }

  static void be_en_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len <= 0) { return; }
    BeEn::be_en_lock();
    if (be_en.rx.size() >= ESPNOW_RX_MAX) {
      be_en.rx_dropped++;
    } else {
      EspnowRx r;
      memcpy(r.src, info->src_addr, 6);
      r.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
      r.ms = millis();
      r.data.assign((const char *)data, len);
      be_en.rx.push_back(std::move(r));
      be_en.rx_count++;
    }
    BeEn::be_en_unlock();
  }

  static void be_en_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    bool ok = (status == ESP_NOW_SEND_SUCCESS);
    BeEn::be_en_lock();
    if (ok) { be_en.tx_ok++; } else { be_en.tx_fail++; }
    if (be_en.tx.size() >= ESPNOW_TX_MAX) { be_en.tx.pop_front(); }
    EspnowTx t;
    memcpy(t.dst, (info && info->des_addr) ? info->des_addr : ESPNOW_BCAST, 6);
    t.ok = ok;
    be_en.tx.push_back(t);
    BeEn::be_en_unlock();
  }

  static bool be_en_get_mac(bvm *vm, int idx, uint8_t *mac) {
    if (be_top(vm) < idx || !be_isbytes(vm, idx)) { return false; }
    size_t len = 0;
    const uint8_t *p = (const uint8_t *)be_tobytes(vm, idx, &len);
    if (!p || len != 6) { return false; }
    memcpy(mac, p, 6);
    return true;
  }

  static bool be_en_ensure_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) { return true; }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;                     // current channel
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
  }

  static void be_en_hold(bool hold) {
    if (hold == BerryEspnowWifiHold) { return; }
    BerryEspnowWifiHold = hold;
    WiFi.setAutoReconnect(!hold);
    if (hold) {
      if (WiFi.status() == WL_CONNECTED || WiFi.isConnected()) { WiFi.disconnect(false); }
      AddLog(LOG_LEVEL_INFO, PSTR("ESN: WiFi held (RAM only)"));
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("ESN: WiFi released"));
    }
  }

};

extern "C" {

  // espnow.begin(channel, hold_wifi) -> bool. channel 0 = keep the current one.
  bbool be_espnow_begin(struct bvm *vm, int32_t channel, bbool hold) {
    if (!be_en.lock) { be_en.lock = xSemaphoreCreateMutex(); }
    if (hold) { BeEn::be_en_hold(true); }
    if (WiFi.getMode() == WIFI_OFF) { WiFi.mode(WIFI_STA); }
    if (channel >= 1 && channel <= 14) {
      if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("ESN: channel %d refused (connected to an AP?)"), channel);
      }
    }
    if (!be_en.up) {
      if (esp_now_init() != ESP_OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("ESN: esp_now_init failed"));
        return bfalse;
      }
      esp_now_register_recv_cb(BeEn::be_en_recv_cb);
      esp_now_register_send_cb(BeEn::be_en_send_cb);
      be_en.up = true;
    }
    BeEn::be_en_ensure_peer(ESPNOW_BCAST);
    return btrue;
  }

  void be_espnow_end(struct bvm *vm) {
    if (be_en.up) {
      esp_now_unregister_recv_cb();
      esp_now_unregister_send_cb();
      esp_now_deinit();
      be_en.up = false;
    }
    BeEn::be_en_hold(false);
  }

  void be_espnow_wifi_hold(struct bvm *vm, bbool hold) {
    BeEn::be_en_hold(hold);
  }

  int32_t be_espnow_channel(struct bvm *vm) {
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    if (esp_wifi_get_channel(&ch, &sec) != ESP_OK) { return -1; }
    return ch;
  }

  // Fails while associated to an AP: the station follows the AP's channel.
  bbool be_espnow_set_channel(struct bvm *vm, int32_t channel) {
    if (channel < 1 || channel > 14) { return bfalse; }
    return esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) == ESP_OK ? btrue : bfalse;
  }

  // espnow.lr(true) admits Espressif Long Range in the station protocol bitmap
  bbool be_espnow_lr(struct bvm *vm, bbool on) {
    uint8_t proto = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    if (on) { proto |= WIFI_PROTOCOL_LR; }
    return esp_wifi_set_protocol(WIFI_IF_STA, proto) == ESP_OK ? btrue : bfalse;
  }

  int32_t be_espnow_version(struct bvm *vm) {
    uint32_t v = 0;
    if (esp_now_get_version(&v) != ESP_OK) { return -1; }
    return v;
  }

  int be_espnow_add_peer(bvm *vm) {
    uint8_t mac[6];
    bool ok = be_en.up && BeEn::be_en_get_mac(vm, 1, mac) && BeEn::be_en_ensure_peer(mac);
    be_pushbool(vm, ok);
    be_return(vm);
  }

  int be_espnow_del_peer(bvm *vm) {
    uint8_t mac[6];
    bool ok = be_en.up && BeEn::be_en_get_mac(vm, 1, mac) && esp_now_del_peer(mac) == ESP_OK;
    be_pushbool(vm, ok);
    be_return(vm);
  }

  // espnow.send(mac_or_nil, bytes) -> bool; nil = broadcast, at most 1470 bytes
  int be_espnow_send(bvm *vm) {
    bool ok = false;
    uint8_t mac[6];
    if (be_en.up && be_top(vm) >= 2 && be_isbytes(vm, 2)) {
      if (!BeEn::be_en_get_mac(vm, 1, mac)) { memcpy(mac, ESPNOW_BCAST, 6); }
      size_t len = 0;
      const uint8_t *p = (const uint8_t *)be_tobytes(vm, 2, &len);
      if (p && len > 0 && len <= ESP_NOW_MAX_DATA_LEN_V2 && BeEn::be_en_ensure_peer(mac)) {
        ok = esp_now_send(mac, p, len) == ESP_OK;
      }
    }
    be_pushbool(vm, ok);
    be_return(vm);
  }

  // espnow.rate(mac_or_nil, rate) -> bool; rate 1 or 2 (Mbit/s, 11b), 250 or 500 (LR kbit/s)
  int be_espnow_rate(bvm *vm) {
    bool ok = false;
    uint8_t mac[6];
    if (be_en.up && be_top(vm) >= 2 && be_isint(vm, 2)) {
      if (!BeEn::be_en_get_mac(vm, 1, mac)) { memcpy(mac, ESPNOW_BCAST, 6); }
      esp_now_rate_config_t cfg = {};
      int r = be_toint(vm, 2);
      cfg.phymode = WIFI_PHY_MODE_11B;
      if (r == 1)        { cfg.rate = WIFI_PHY_RATE_1M_L; }
      else if (r == 2)   { cfg.rate = WIFI_PHY_RATE_2M_L; }
      else if (r == 250) { cfg.phymode = WIFI_PHY_MODE_LR; cfg.rate = WIFI_PHY_RATE_LORA_250K; }
      else if (r == 500) { cfg.phymode = WIFI_PHY_MODE_LR; cfg.rate = WIFI_PHY_RATE_LORA_500K; }
      else { be_pushbool(vm, false); be_return(vm); }
      ok = BeEn::be_en_ensure_peer(mac) && esp_now_set_peer_rate_config(mac, &cfg) == ESP_OK;
    }
    be_pushbool(vm, ok);
    be_return(vm);
  }

  // espnow.recv() -> list of [src_mac, rssi, bytes, millis]
  int be_espnow_recv(bvm *vm) {
    std::deque<EspnowRx> q;
    if (be_en.lock) { BeEn::be_en_lock(); q.swap(be_en.rx); BeEn::be_en_unlock(); }
    be_newobject(vm, "list");
    for (auto &r : q) {
      be_newobject(vm, "list");
      be_pushbytes(vm, r.src, 6);              be_data_push(vm, -2); be_pop(vm, 1);
      be_pushint(vm, r.rssi);                  be_data_push(vm, -2); be_pop(vm, 1);
      be_pushbytes(vm, r.data.data(), r.data.size()); be_data_push(vm, -2); be_pop(vm, 1);
      be_pushint(vm, r.ms);                    be_data_push(vm, -2); be_pop(vm, 1);
      be_pop(vm, 1);
      be_data_push(vm, -2);
      be_pop(vm, 1);
    }
    be_pop(vm, 1);
    be_return(vm);
  }

  // espnow.sent() -> list of [dst_mac, ok]
  int be_espnow_sent(bvm *vm) {
    std::deque<EspnowTx> q;
    if (be_en.lock) { BeEn::be_en_lock(); q.swap(be_en.tx); BeEn::be_en_unlock(); }
    be_newobject(vm, "list");
    for (auto &t : q) {
      be_newobject(vm, "list");
      be_pushbytes(vm, t.dst, 6); be_data_push(vm, -2); be_pop(vm, 1);
      be_pushbool(vm, t.ok);      be_data_push(vm, -2); be_pop(vm, 1);
      be_pop(vm, 1);
      be_data_push(vm, -2);
      be_pop(vm, 1);
    }
    be_pop(vm, 1);
    be_return(vm);
  }

  // espnow.stats() -> [tx_ok, tx_fail, rx, rx_dropped]
  int be_espnow_stats(bvm *vm) {
    int32_t v[4] = {(int32_t)be_en.tx_ok, (int32_t)be_en.tx_fail, (int32_t)be_en.rx_count, (int32_t)be_en.rx_dropped};
    be_newobject(vm, "list");
    for (int i = 0; i < 4; i++) { be_pushint(vm, v[i]); be_data_push(vm, -2); be_pop(vm, 1); }
    be_pop(vm, 1);
    be_return(vm);
  }

  // espnow.mac() -> bytes(6), own station MAC
  int be_espnow_mac(bvm *vm) {
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    be_pushbytes(vm, mac, 6);
    be_return(vm);
  }

} // extern "C"

#endif  // USE_BERRY_ESPNOW
#endif  // USE_BERRY
