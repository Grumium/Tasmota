/********************************************************************
 * Berry module espnow: ESP-NOW with an optional RAM-only WiFi hold
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_BERRY_ESPNOW

extern bbool be_espnow_begin(struct bvm *vm, int32_t channel, bbool hold);
BE_FUNC_CTYPE_DECLARE(be_espnow_begin, "b", "@i[b]");

extern void be_espnow_end(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_espnow_end, "", "@");

extern void be_espnow_wifi_hold(struct bvm *vm, bbool hold);
BE_FUNC_CTYPE_DECLARE(be_espnow_wifi_hold, "", "@b");

extern int32_t be_espnow_channel(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_espnow_channel, "i", "@");

extern bbool be_espnow_set_channel(struct bvm *vm, int32_t channel);
BE_FUNC_CTYPE_DECLARE(be_espnow_set_channel, "b", "@i");

extern bbool be_espnow_lr(struct bvm *vm, bbool on);
BE_FUNC_CTYPE_DECLARE(be_espnow_lr, "b", "@b");

extern int32_t be_espnow_version(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_espnow_version, "i", "@");

extern int be_espnow_add_peer(bvm *vm);
extern int be_espnow_del_peer(bvm *vm);
extern int be_espnow_send(bvm *vm);
extern int be_espnow_rate(bvm *vm);
extern int be_espnow_recv(bvm *vm);
extern int be_espnow_sent(bvm *vm);
extern int be_espnow_stats(bvm *vm);
extern int be_espnow_mac(bvm *vm);

/* @const_object_info_begin
module espnow (scope: global) {
  begin,       ctype_func(be_espnow_begin)
  stop,        ctype_func(be_espnow_end)
  wifi_hold,   ctype_func(be_espnow_wifi_hold)
  channel,     ctype_func(be_espnow_channel)
  set_channel, ctype_func(be_espnow_set_channel)
  lr,          ctype_func(be_espnow_lr)
  version,     ctype_func(be_espnow_version)
  add_peer,    func(be_espnow_add_peer)
  del_peer,    func(be_espnow_del_peer)
  send,        func(be_espnow_send)
  rate,        func(be_espnow_rate)
  recv,        func(be_espnow_recv)
  sent,        func(be_espnow_sent)
  stats,       func(be_espnow_stats)
  mac,         func(be_espnow_mac)
}
@const_object_info_end */
#include "be_fixed_espnow.h"

#endif // USE_BERRY_ESPNOW
