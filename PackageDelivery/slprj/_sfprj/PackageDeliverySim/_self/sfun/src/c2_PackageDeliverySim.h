#ifndef __c2_PackageDeliverySim_h__
#define __c2_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_PackageDeliverySimInstanceStruct
#define typedef_SFc2_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_PackageDeliverySim;
  real_T (*c2_P_address)[4];
  int32_T c2_P_index;
  real_T (*c2_Rd_address)[9];
  int32_T c2_Rd_index;
  real_T *c2_g_address;
  int32_T c2_g_index;
  real_T *c2_k_address;
  int32_T c2_k_index;
  real_T *c2_kb_address;
  int32_T c2_kb_index;
  real_T *c2_m_address;
  int32_T c2_m_index;
  real_T *c2_satMax_address;
  int32_T c2_satMax_index;
  real_T (*c2_state)[18];
  real_T (*c2_ref)[15];
  real_T (*c2_z)[3];
  real_T (*c2_hat_b)[3];
  real_T (*c2_hat_b2)[3];
  real_T (*c2_w)[3];
  real_T *c2_h;
  real_T (*c2_qh)[4];
  real_T (*c2_cmd)[4];
  real_T *c2_kp;
  real_T *c2_kv;
  real_T (*c2_dot_z)[3];
  real_T (*c2_dot_hat_b)[3];
  real_T (*c2_dot_hat_b2)[3];
  real_T (*c2_dot_w)[3];
  real_T *c2_dot_h;
  real_T (*c2_dot_qh)[4];
  real_T *c2_kz;
  real_T *c2_kV0;
  real_T *c2_V2;
  real_T *c2_dot_V2;
  real_T *c2_kq;
  real_T *c2_kw;
  real_T (*c2_q0)[4];
  real_T (*c2_q)[4];
} SFc2_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc2_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c2_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c2_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
