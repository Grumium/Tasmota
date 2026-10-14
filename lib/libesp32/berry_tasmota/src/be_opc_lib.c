/********************************************************************
 * Berry module opc: Alphasense OPC state without rules
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_OPC

extern int32_t be_opc_count(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_opc_count, "i", "@");

extern int32_t be_opc_mode(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_mode, "i", "@i");

extern bbool be_opc_ready(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_ready, "b", "@i");

extern int32_t be_opc_gain(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_gain, "i", "@i");

extern int32_t be_opc_seq(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_seq, "i", "@i");

extern float be_opc_pm(struct bvm *vm, int32_t idx, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_opc_pm, "f", "@ii");

extern float be_opc_pm_dia(struct bvm *vm, int32_t idx, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_opc_pm_dia, "f", "@ii");

extern int32_t be_opc_hist_n(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_hist_n, "i", "@i");

extern float be_opc_hist(struct bvm *vm, int32_t idx, int32_t bin);
BE_FUNC_CTYPE_DECLARE(be_opc_hist, "f", "@ii");

extern float be_opc_flow(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_flow, "f", "@i");

extern float be_opc_period(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_period, "f", "@i");

extern float be_opc_temp(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_temp, "f", "@i");

extern float be_opc_humi(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_humi, "f", "@i");

extern float be_opc_abs_humi(struct bvm *vm, int32_t idx);
BE_FUNC_CTYPE_DECLARE(be_opc_abs_humi, "f", "@i");

/* @const_object_info_begin
module opc (scope: global) {
  count, ctype_func(be_opc_count)
  mode, ctype_func(be_opc_mode)
  ready, ctype_func(be_opc_ready)
  gain, ctype_func(be_opc_gain)
  seq, ctype_func(be_opc_seq)
  pm, ctype_func(be_opc_pm)
  pm_dia, ctype_func(be_opc_pm_dia)
  hist_n, ctype_func(be_opc_hist_n)
  hist, ctype_func(be_opc_hist)
  flow, ctype_func(be_opc_flow)
  period, ctype_func(be_opc_period)
  temp, ctype_func(be_opc_temp)
  humi, ctype_func(be_opc_humi)
  abs_humi, ctype_func(be_opc_abs_humi)
}
@const_object_info_end */
#include "be_fixed_opc.h"

#endif // USE_OPC
