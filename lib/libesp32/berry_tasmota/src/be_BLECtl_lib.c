/********************************************************************
 * Berry module BLECtl: GATT server (one service, RX write / TX notify)
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_BLE_CTL

extern bbool be_BLECtl_begin(struct bvm *vm, const char *svc, const char *rx, const char *tx);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_begin, "b", "@sss");

extern bbool be_BLECtl_adv_stop(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_adv_stop, "b", "@");

extern bbool be_BLECtl_connected(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_connected, "b", "@");

extern bbool be_BLECtl_advertising(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_advertising, "b", "@");

extern int32_t be_BLECtl_mtu(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_mtu, "i", "@");

extern bbool be_BLECtl_disconnect(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_BLECtl_disconnect, "b", "@");

extern int be_BLECtl_adv(bvm *vm);
extern int be_BLECtl_recv(bvm *vm);
extern int be_BLECtl_notify(bvm *vm);
extern int be_BLECtl_events(bvm *vm);
extern int be_BLECtl_stats(bvm *vm);

/* @const_object_info_begin
module BLECtl (scope: global) {
  begin,      ctype_func(be_BLECtl_begin)
  adv,        func(be_BLECtl_adv)
  adv_stop,   ctype_func(be_BLECtl_adv_stop)
  recv,       func(be_BLECtl_recv)
  notify,     func(be_BLECtl_notify)
  events,     func(be_BLECtl_events)
  stats,      func(be_BLECtl_stats)
  connected,  ctype_func(be_BLECtl_connected)
  advertising, ctype_func(be_BLECtl_advertising)
  mtu,        ctype_func(be_BLECtl_mtu)
  disconnect, ctype_func(be_BLECtl_disconnect)
}
@const_object_info_end */
#include "be_fixed_BLECtl.h"

#endif // USE_BLE_CTL
