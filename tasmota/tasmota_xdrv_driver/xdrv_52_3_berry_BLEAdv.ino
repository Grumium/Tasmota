/*
  xdrv_52_3_berry_BLEAdv.ino - Berry scripting language, native functions for BLE advertising

  Copyright (C) 2021 Stephan Hadinger, Berry language by Guan Wenliang https://github.com/Skiars/berry

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

#ifdef USE_BLE_ADV

#include <NimBLEDevice.h>

/*********************************************************************************************\
 * Thin binding to send raw BLE advertising payloads from Berry.
 *
 * The payload is handed to the controller byte for byte, so Berry keeps full
 * control over the AD structures. Nothing is appended - no flags, no name, no
 * TX power - because a beacon format such as Find My fills all 31 bytes itself.
 *
 * The module works standalone: if no BLE driver has brought NimBLE up, it is
 * initialized here on first use. Where the BLE_ESP32 scanner (xdrv_79) is also
 * compiled in, NimBLE is a shared resource - that driver inits and, on client
 * errors, deinits the very same stack. The payload and MAC are therefore
 * mirrored here and re-armed once the host has synced again, otherwise the
 * beacon would go silent on the first scanner restart without anyone noticing.
\*********************************************************************************************/

const size_t BLE_ADV_PAYLOAD_MAX = 31;    // BLE_HS_ADV_MAX_SZ, the whole AD stream including length bytes
const size_t BLE_ADV_MAC_LEN = 6;

struct {
  uint8_t payload[BLE_ADV_PAYLOAD_MAX];
  size_t payload_len = 0;
  uint8_t mac[BLE_ADV_MAC_LEN];
  bool mac_set = false;
  uint16_t itvl_min = 0;                  // 0 = leave NimBLE default
  uint16_t itvl_max = 0;
  bool want_active = false;               // what Berry asked for, used to re-arm
  bool armed = false;                     // what the stack currently has
  bool rearm_failed = false;              // re-arm already complained, stay quiet
} be_ble_adv;

extern "C" {

  // Bring NimBLE up if no other driver owns it yet. init() blocks until the
  // host and controller have synced, so advertising may start right after.
  // Never deinits: a scanner sharing the stack would lose it.
  static bool be_BLEAdv_stack_up(bool verbose) {
    if (NimBLEDevice::isInitialized()) { return true; }
    if (!NimBLEDevice::init("Tasmota")) {
      if (verbose) { AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: NimBLE init failed")); }
      return false;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: NimBLE initialized"));
    return true;
  }

  // Push MAC, payload and intervals into the stack and start advertising.
  // Split out so the re-arm path after a stack restart runs the identical code.
  // verbose=false on the once-a-second re-arm path, so a persistent failure
  // does not bury the console; the caller still sees the return value.
  static bool be_BLEAdv_arm(bool verbose) {
    if (be_ble_adv.payload_len == 0) { return false; }

    if (!be_BLEAdv_stack_up(verbose)) { return false; }

    if (be_ble_adv.mac_set) {
      // ble_hs_id_set_rnd() refuses while advertising is active, so this only
      // ever runs from the stopped state - see BLEAdv.set_mac().
      if (!NimBLEDevice::setOwnAddr(be_ble_adv.mac)) {
        if (verbose) { AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: could not set random address")); }
        return false;
      }
      // The address type is global to NimBLE: the xdrv_79 scanner and any
      // client use this address too from here on.
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    }

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (!adv) { return false; }

    // Must come before the payload: with ROLE_PERIPHERAL the constructor puts a
    // flags field into the advertisement, and only NON clears it again.
    adv->setConnectableMode(BLE_GAP_CONN_MODE_NON);
    adv->enableScanResponse(false);

    if (be_ble_adv.itvl_min) { adv->setMinInterval(be_ble_adv.itvl_min); }
    if (be_ble_adv.itvl_max) { adv->setMaxInterval(be_ble_adv.itvl_max); }

    NimBLEAdvertisementData data;
    data.addData(be_ble_adv.payload, be_ble_adv.payload_len);
    if (!adv->setAdvertisementData(data)) {
      if (verbose) { AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: advertisement data rejected")); }
      return false;
    }

    if (!adv->start()) { return false; }

    be_ble_adv.armed = true;
    return true;
  }

  // BLEAdv.set_mac(bytes) -> bool
  // 6 bytes in display order (C1:4A:.. is passed as C1 4A ..), byte-reversed
  // below for NimBLE. The caller owns the key derivation, including
  // addr[0] |= 0b11000000.
  bbool be_BLEAdv_set_mac(struct bvm *vm, const uint8_t *mac, size_t len) {
    if (len != BLE_ADV_MAC_LEN) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: MAC must be 6 bytes, got %u"), (unsigned)len);
      return bfalse;
    }
    if (be_ble_adv.armed) {
      // The controller rejects an address change while advertising; say so
      // instead of failing quietly. Rotate with stop() / set_mac() / start().
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: stop advertising before changing the MAC"));
      return bfalse;
    }
    // NimBLE takes addresses little-endian (LSB first), the way they travel over
    // HCI, while Berry passes them in display order the way a scanner shows them.
    // NimBLEAddress reverses for the same reason; ble_hs_id_set_rnd() gets raw
    // bytes and would otherwise find the two mandatory top bits of a random
    // static address in the wrong byte and reject it. Do not "simplify" to memcpy.
    for (size_t i = 0; i < BLE_ADV_MAC_LEN; i++) {
      be_ble_adv.mac[i] = mac[BLE_ADV_MAC_LEN - 1 - i];
    }
    be_ble_adv.mac_set = true;
    return btrue;
  }

  // BLEAdv.set_payload(bytes) -> bool
  // 1..31 raw advertising bytes, sent exactly as given.
  bbool be_BLEAdv_set_payload(struct bvm *vm, const uint8_t *payload, size_t len) {
    if (len == 0 || len > BLE_ADV_PAYLOAD_MAX) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: payload must be 1..31 bytes, got %u"), (unsigned)len);
      return bfalse;
    }
    if (be_ble_adv.armed) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: stop advertising before changing the payload"));
      return bfalse;
    }
    // An AD structure is [len][type][data...] and len does not count itself.
    // A single structure filling the packet therefore has payload[0] == len-1.
    // Two concatenated structures legitimately do not, so this only warns.
    if (payload[0] != len - 1) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: payload[0]=%u looks like a wrong AD length for %u bytes (expected %u) - sending unchanged"),
             payload[0], (unsigned)len, (unsigned)(len - 1));
    }
    memcpy(be_ble_adv.payload, payload, len);
    be_ble_adv.payload_len = len;
    return btrue;
  }

  // BLEAdv.set_interval(min, max) -> bool, in 0.625 ms units
  bbool be_BLEAdv_set_interval(struct bvm *vm, int32_t itvl_min, int32_t itvl_max) {
    if (itvl_min <= 0 || itvl_max <= 0 || itvl_min > 0xFFFF || itvl_max > 0xFFFF || itvl_min > itvl_max) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: invalid interval %i..%i"), (int)itvl_min, (int)itvl_max);
      return bfalse;
    }
    be_ble_adv.itvl_min = (uint16_t)itvl_min;
    be_ble_adv.itvl_max = (uint16_t)itvl_max;
    if (be_ble_adv.armed) {
      NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
      if (adv) {
        adv->setMinInterval(be_ble_adv.itvl_min);
        adv->setMaxInterval(be_ble_adv.itvl_max);
      }
    }
    return btrue;
  }

  // BLEAdv.set_power(dbm) -> bool
  bbool be_BLEAdv_set_power(struct bvm *vm, int32_t dbm) {
    if (!be_BLEAdv_stack_up(true)) { return bfalse; }
    // Power is global to the radio, so the scanner is affected as well.
    NimBLEDevice::setPower((int8_t)dbm);
    return btrue;
  }

  // BLEAdv.start() -> bool
  bbool be_BLEAdv_start(struct bvm *vm) {
    if (be_ble_adv.payload_len == 0) {
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: set a payload before starting"));
      return bfalse;
    }
    be_ble_adv.want_active = true;
    be_ble_adv.rearm_failed = false;    // a fresh start may complain again
    if (be_ble_adv.armed) { return btrue; }
    return be_BLEAdv_arm(true) ? btrue : bfalse;
  }

  // BLEAdv.stop() -> bool
  bbool be_BLEAdv_stop(struct bvm *vm) {
    be_ble_adv.want_active = false;
    if (!be_ble_adv.armed) { return btrue; }
    be_ble_adv.armed = false;
    if (!NimBLEDevice::isInitialized()) { return btrue; }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (!adv) { return btrue; }
    return adv->stop() ? btrue : bfalse;
  }

  // BLEAdv.is_active() -> bool, the controller's view rather than ours
  bbool be_BLEAdv_is_active(struct bvm *vm) {
    if (!be_ble_adv.armed || !NimBLEDevice::isInitialized()) { return bfalse; }
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    return (adv && adv->isAdvertising()) ? btrue : bfalse;
  }

} // extern "C"

/*********************************************************************************************\
 * Re-arm after the stack went away.
 *
 * xdrv_79 calls NimBLEDevice::deinit()/init() when a client misbehaves, which
 * drops advertising and resets the address. Without this the beacon would stay
 * silent until the next reboot.
\*********************************************************************************************/
void BLEAdvEverySecond(void) {
  if (!be_ble_adv.want_active) { return; }

  // Still advertising: nothing to do. Ask the stack rather than trusting our
  // own flag, since the scanner can tear it down behind our back.
  if (NimBLEDevice::isInitialized()) {
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (adv && adv->isAdvertising()) { return; }
  }

  // Complain once per outage, not once per second: this runs every second and
  // a lasting failure would otherwise drown out everything else on the console.
  if (be_BLEAdv_arm(!be_ble_adv.rearm_failed)) {
    AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: re-armed advertising after stack restart"));
    be_ble_adv.rearm_failed = false;
  } else {
    be_ble_adv.armed = false;
    if (!be_ble_adv.rearm_failed) {
      be_ble_adv.rearm_failed = true;
      AddLog(LOG_LEVEL_INFO, PSTR("BLEAdv: re-arm failed, retrying silently every second"));
    }
  }
}

#endif  // USE_BLE_ADV
#endif  // USE_BERRY
