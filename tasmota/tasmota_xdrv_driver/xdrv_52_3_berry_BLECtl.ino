/*
  xdrv_52_3_berry_BLECtl.ino - Berry scripting language, GATT server for a control channel

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

#ifdef USE_BLE_CTL

#include <NimBLEDevice.h>
#include <deque>
#include <string>

/*********************************************************************************************\
 * One GATT service with two characteristics: RX (write / write-no-response, central to
 * device) and TX (notify, device to central). Only mechanics live here; framing, login and
 * policy are Berry.
 *
 * NimBLE callbacks run in the NimBLE host task and must not touch Berry. They only push into
 * the queues below; Berry drains them from its own loop. BLEAdv and BLECtl share the stack and
 * the single advertising set, so they exclude each other: Berry stops one before starting the
 * other. The stack is never deinitialised (a scanner or BLEAdv may be using it).
\*********************************************************************************************/

static const size_t BLE_CTL_QUEUE_MAX = 32;          // RX writes held for Berry
static const size_t BLE_CTL_EVENTS_MAX = 16;

struct BleCtlEvent { uint8_t type; int32_t value; };   // 1 connect, 2 disconnect, 3 mtu

static struct {
  NimBLEServer *server = nullptr;
  NimBLECharacteristic *rx = nullptr;
  NimBLECharacteristic *tx = nullptr;
  SemaphoreHandle_t lock = nullptr;
  std::deque<std::string> rx_q;
  std::deque<BleCtlEvent> ev_q;
  uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
  uint16_t mtu = 23;
  uint32_t rx_dropped = 0;
  uint32_t rx_count = 0;
  uint32_t tx_count = 0;
  uint32_t tx_fail = 0;
  bool adv_on = false;
} be_ctl;

// The negotiated ATT MTU, asked from the stack: the onMTUChange event can arrive after the
// first writes, and a stale 23 would make the handshake reply fail.
static uint16_t be_ctl_mtu_now(void) {
  if (!be_ctl.server || be_ctl.conn == BLE_HS_CONN_HANDLE_NONE) { return 0; }
  uint16_t m = be_ctl.server->getPeerMTU(be_ctl.conn);
  return m ? m : 23;
}

static void be_ctl_lock(void)   { xSemaphoreTake(be_ctl.lock, portMAX_DELAY); }
static void be_ctl_unlock(void) { xSemaphoreGive(be_ctl.lock); }

static void be_ctl_event(uint8_t type, int32_t value) {
  be_ctl_lock();
  if (be_ctl.ev_q.size() >= BLE_CTL_EVENTS_MAX) { be_ctl.ev_q.pop_front(); }
  be_ctl.ev_q.push_back({type, value});
  be_ctl_unlock();
}

class BleCtlServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *srv, NimBLEConnInfo &info) override {
    // One central at a time: a stale earlier link (BlueZ retry) gives way to the newest.
    if (be_ctl.conn != BLE_HS_CONN_HANDLE_NONE && be_ctl.conn != info.getConnHandle()) {
      srv->disconnect(be_ctl.conn);
    }
    be_ctl.conn = info.getConnHandle();
    be_ctl.mtu = 23;
    be_ctl.adv_on = false;      // the stack stops advertising on connect
    be_ctl_event(1, be_ctl.conn);
  }
  void onDisconnect(NimBLEServer *srv, NimBLEConnInfo &info, int reason) override {
    if (info.getConnHandle() != be_ctl.conn) { return; }   // the replaced stale link
    be_ctl.conn = BLE_HS_CONN_HANDLE_NONE;
    be_ctl.mtu = 23;
    be_ctl_event(2, reason);
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &info) override {
    be_ctl.mtu = mtu;
    be_ctl_event(3, mtu);
  }
};

class BleCtlRxCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    NimBLEAttValue v = chr->getValue();
    be_ctl_lock();
    if (be_ctl.rx_q.size() >= BLE_CTL_QUEUE_MAX) {
      be_ctl.rx_dropped++;
    } else {
      be_ctl.rx_q.emplace_back((const char *)v.data(), v.size());
      be_ctl.rx_count++;
    }
    be_ctl_unlock();
  }
};

extern "C" {

  // BLECtl.begin(service_uuid, rx_uuid, tx_uuid) -> bool. Once per boot; repeat calls succeed.
  bbool be_BLECtl_begin(struct bvm *vm, const char *svc, const char *rx, const char *tx) {
    if (be_ctl.server) { return btrue; }
    if (!svc || !rx || !tx) { return bfalse; }
    if (!NimBLEDevice::isInitialized()) {
      if (!NimBLEDevice::init("Tasmota")) {
        AddLog(LOG_LEVEL_INFO, PSTR("BLECtl: NimBLE init failed"));
        return bfalse;
      }
      AddLog(LOG_LEVEL_INFO, PSTR("BLECtl: NimBLE initialized"));
    }
    if (!be_ctl.lock) { be_ctl.lock = xSemaphoreCreateMutex(); }
    NimBLEDevice::setMTU(247);
    NimBLEServer *srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BleCtlServerCb(), true);
    srv->advertiseOnDisconnect(false);
    NimBLEService *s = srv->createService(svc);
    if (!s) { return bfalse; }
    be_ctl.rx = s->createCharacteristic(rx, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    be_ctl.tx = s->createCharacteristic(tx, NIMBLE_PROPERTY::NOTIFY);
    if (!be_ctl.rx || !be_ctl.tx) { return bfalse; }
    be_ctl.rx->setCallbacks(new BleCtlRxCb());
    s->start();
    srv->start();
    be_ctl.server = srv;
    AddLog(LOG_LEVEL_INFO, PSTR("BLECtl: GATT service up"));
    return btrue;
  }

  // BLECtl.adv(payload, scan_rsp, mac) -> bool. Connectable advertising with raw AD bytes
  // (1..31, flags included by the caller). scan_rsp and mac may be nil. mac in display order.
  int be_BLECtl_adv(bvm *vm) {
    if (!be_ctl.server) { be_pushbool(vm, bfalse); be_return(vm); }
    int argc = be_top(vm);
    size_t plen = 0, slen = 0, mlen = 0;
    const void *payload = (argc >= 1 && be_isbytes(vm, 1)) ? be_tobytes(vm, 1, &plen) : nullptr;
    const void *scan = (argc >= 2 && be_isbytes(vm, 2)) ? be_tobytes(vm, 2, &slen) : nullptr;
    const void *mac = (argc >= 3 && be_isbytes(vm, 3)) ? be_tobytes(vm, 3, &mlen) : nullptr;
    if (!payload || plen == 0 || plen > 31 || slen > 31 || (mac && mlen != 6)) {
      be_pushbool(vm, bfalse); be_return(vm);
    }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (!adv) { be_pushbool(vm, bfalse); be_return(vm); }
    if (adv->isAdvertising()) { adv->stop(); }
    if (mac) {
      uint8_t le[6];                                   // NimBLE wants LSB first, see BLEAdv.set_mac
      for (int i = 0; i < 6; i++) { le[i] = ((const uint8_t *)mac)[5 - i]; }
      if (!NimBLEDevice::setOwnAddr(le)) { be_pushbool(vm, bfalse); be_return(vm); }
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    }
    adv->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    NimBLEAdvertisementData data;
    data.addData((const uint8_t *)payload, plen);
    bool ok = adv->setAdvertisementData(data);
    if (ok) {
      adv->enableScanResponse(slen > 0);
      if (slen > 0) {
        NimBLEAdvertisementData sr;
        sr.addData((const uint8_t *)scan, slen);
        ok = adv->setScanResponseData(sr);
      }
    }
    if (ok) { ok = adv->start(); }
    be_ctl.adv_on = ok;
    be_pushbool(vm, ok ? btrue : bfalse);
    be_return(vm);
  }

  // BLECtl.adv_stop() -> bool; a connection stays up
  bbool be_BLECtl_adv_stop(struct bvm *vm) {
    be_ctl.adv_on = false;
    if (!NimBLEDevice::isInitialized()) { return btrue; }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (!adv || !adv->isAdvertising()) { return btrue; }
    return adv->stop() ? btrue : bfalse;
  }

  // BLECtl.recv() -> list of bytes, the RX writes since the last call, in order
  int be_BLECtl_recv(bvm *vm) {
    std::deque<std::string> q;
    if (be_ctl.lock) {
      be_ctl_lock();
      q.swap(be_ctl.rx_q);
      be_ctl_unlock();
    }
    be_newobject(vm, "list");
    for (auto &s : q) {
      be_pushbytes(vm, s.data(), s.size());
      be_data_push(vm, -2);
      be_pop(vm, 1);
    }
    be_pop(vm, 1);
    be_return(vm);
  }

  // BLECtl.notify(bytes) -> bool, at most mtu-3 bytes
  int be_BLECtl_notify(bvm *vm) {
    bool ok = false;
    if (be_top(vm) >= 1 && be_isbytes(vm, 1) && be_ctl.tx && be_ctl.conn != BLE_HS_CONN_HANDLE_NONE) {
      size_t len = 0;
      const uint8_t *p = (const uint8_t *)be_tobytes(vm, 1, &len);
      if (p && len > 0 && len + 3 <= be_ctl_mtu_now()) {
        ok = be_ctl.tx->notify(p, len, be_ctl.conn);
      }
    }
    if (ok) { be_ctl.tx_count++; } else { be_ctl.tx_fail++; }
    be_pushbool(vm, ok ? btrue : bfalse);
    be_return(vm);
  }

  // BLECtl.events() -> list of [name, value]: connect(handle), disconnect(reason), mtu(value)
  int be_BLECtl_events(bvm *vm) {
    std::deque<BleCtlEvent> q;
    if (be_ctl.lock) {
      be_ctl_lock();
      q.swap(be_ctl.ev_q);
      be_ctl_unlock();
    }
    static const char *names[] = {"", "connect", "disconnect", "mtu"};
    be_newobject(vm, "list");
    for (auto &e : q) {
      be_newobject(vm, "list");
      be_pushstring(vm, names[e.type <= 3 ? e.type : 0]);
      be_data_push(vm, -2);
      be_pop(vm, 1);
      be_pushint(vm, e.value);
      be_data_push(vm, -2);
      be_pop(vm, 1);
      be_pop(vm, 1);
      be_data_push(vm, -2);
      be_pop(vm, 1);
    }
    be_pop(vm, 1);
    be_return(vm);
  }

  // BLECtl.stats() -> [rx, rx_dropped, tx, tx_fail]
  int be_BLECtl_stats(bvm *vm) {
    be_newobject(vm, "list");
    int32_t vals[4] = {(int32_t)be_ctl.rx_count, (int32_t)be_ctl.rx_dropped,
                       (int32_t)be_ctl.tx_count, (int32_t)be_ctl.tx_fail};
    for (int i = 0; i < 4; i++) {
      be_pushint(vm, vals[i]);
      be_data_push(vm, -2);
      be_pop(vm, 1);
    }
    be_pop(vm, 1);
    be_return(vm);
  }

  bbool be_BLECtl_connected(struct bvm *vm) {
    return be_ctl.conn != BLE_HS_CONN_HANDLE_NONE ? btrue : bfalse;
  }

  // BLECtl.advertising() -> bool, the controller's view
  bbool be_BLECtl_advertising(struct bvm *vm) {
    if (!NimBLEDevice::isInitialized()) { return bfalse; }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    return (adv && adv->isAdvertising()) ? btrue : bfalse;
  }

  int32_t be_BLECtl_mtu(struct bvm *vm) {
    return be_ctl_mtu_now();
  }

  bbool be_BLECtl_disconnect(struct bvm *vm) {
    if (!be_ctl.server || be_ctl.conn == BLE_HS_CONN_HANDLE_NONE) { return bfalse; }
    return be_ctl.server->disconnect(be_ctl.conn) ? btrue : bfalse;
  }

} // extern "C"

#endif  // USE_BLE_CTL
#endif  // USE_BERRY
