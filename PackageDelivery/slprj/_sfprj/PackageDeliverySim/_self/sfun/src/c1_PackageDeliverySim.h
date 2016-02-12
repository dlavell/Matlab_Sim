#ifndef __c1_PackageDeliverySim_h__
#define __c1_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_PackageDeliverySimInstanceStruct
#define typedef_SFc1_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_PackageDeliverySim;
  real_T (*c1_Rd_address)[9];
  int32_T c1_Rd_index;
  real_T *c1_alpha_address;
  int32_T c1_alpha_index;
  real_T *c1_delta_address;
  int32_T c1_delta_index;
  real_T *c1_g_address;
  int32_T c1_g_index;
  real_T *c1_k_address;
  int32_T c1_k_index;
  real_T *c1_kp_address;
  int32_T c1_kp_index;
  real_T *c1_kv_address;
  int32_T c1_kv_index;
  real_T *c1_satMax_address;
  int32_T c1_satMax_index;
  real_T (*c1_state)[18];
  real_T (*c1_ref)[15];
  real_T (*c1_x)[17];
  real_T *c1_C;
  real_T (*c1_G)[17];
  real_T *c1_D;
} SFc1_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc1_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c1_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
