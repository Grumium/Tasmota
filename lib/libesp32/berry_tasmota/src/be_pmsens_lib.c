/********************************************************************
 * Berry modules sen6x and bmv080: PM and number concentrations of the
 * Sensirion SEN6x and Bosch BMV080 without the sensor JSON
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_SEN6X

extern bbool be_sen6x_ready(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_sen6x_ready, "b", "@");

extern int32_t be_sen6x_seq(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_sen6x_seq, "i", "@");

extern float be_sen6x_pm(struct bvm *vm, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_sen6x_pm, "f", "@i");

extern float be_sen6x_nc(struct bvm *vm, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_sen6x_nc, "f", "@i");

/* @const_object_info_begin
module sen6x (scope: global) {
  ready, ctype_func(be_sen6x_ready)
  seq, ctype_func(be_sen6x_seq)
  pm, ctype_func(be_sen6x_pm)
  nc, ctype_func(be_sen6x_nc)
}
@const_object_info_end */
#include "be_fixed_sen6x.h"

#endif // USE_SEN6X

#ifdef USE_BMV080

extern bbool be_bmv080_ready(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_bmv080_ready, "b", "@");

extern int32_t be_bmv080_seq(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_bmv080_seq, "i", "@");

extern float be_bmv080_pm(struct bvm *vm, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_bmv080_pm, "f", "@i");

extern float be_bmv080_nc(struct bvm *vm, int32_t k);
BE_FUNC_CTYPE_DECLARE(be_bmv080_nc, "f", "@i");

extern bbool be_bmv080_obstructed(struct bvm *vm);
BE_FUNC_CTYPE_DECLARE(be_bmv080_obstructed, "b", "@");

/* @const_object_info_begin
module bmv080 (scope: global) {
  ready, ctype_func(be_bmv080_ready)
  seq, ctype_func(be_bmv080_seq)
  pm, ctype_func(be_bmv080_pm)
  nc, ctype_func(be_bmv080_nc)
  obstructed, ctype_func(be_bmv080_obstructed)
}
@const_object_info_end */
#include "be_fixed_bmv080.h"

#endif // USE_BMV080
