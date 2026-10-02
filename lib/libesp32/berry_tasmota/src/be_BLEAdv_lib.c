/********************************************************************
 * Tasmota LVGL ctypes mapping
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_BLE_ADV

extern bbool be_BLEAdv_set_mac(struct bvm *vm, const uint8_t *mac, size_t len);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_set_mac, "b", "@(bytes)~");

extern bbool be_BLEAdv_set_payload(struct bvm *vm, const uint8_t *payload, size_t len);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_set_payload, "b", "@(bytes)~");

extern bbool be_BLEAdv_set_interval(struct bvm *vm, int32_t itvl_min, int32_t itvl_max);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_set_interval, "b", "@ii");

extern bbool be_BLEAdv_set_power(struct bvm *vm, int32_t dbm);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_set_power, "b", "@i");

extern bbool be_BLEAdv_start(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_start, "b", "@");

extern bbool be_BLEAdv_stop(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_stop, "b", "@");

extern bbool be_BLEAdv_is_active(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLEAdv_is_active, "b", "@");

/* @const_object_info_begin
module BLEAdv (scope: global) {
  set_mac,      ctype_func(be_BLEAdv_set_mac)
  set_payload,  ctype_func(be_BLEAdv_set_payload)
  set_interval, ctype_func(be_BLEAdv_set_interval)
  set_power,    ctype_func(be_BLEAdv_set_power)
  start,        ctype_func(be_BLEAdv_start)
  stop,         ctype_func(be_BLEAdv_stop)
  is_active,    ctype_func(be_BLEAdv_is_active)
}
@const_object_info_end */
#include "be_fixed_BLEAdv.h"

#endif // USE_BLE_ADV
