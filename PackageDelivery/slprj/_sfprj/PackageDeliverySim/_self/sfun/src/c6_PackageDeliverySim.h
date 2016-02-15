#ifndef __c6_PackageDeliverySim_h__
#define __c6_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc6_PackageDeliverySimInstanceStruct
#define typedef_SFc6_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c6_sfEvent;
  boolean_T c6_isStable;
  boolean_T c6_doneDoubleBufferReInit;
  uint8_T c6_is_active_c6_PackageDeliverySim;
  real_T *c6_curr_tp_address;
  int32_T c6_curr_tp_index;
  real_T *c6_curr_wp_address;
  int32_T c6_curr_wp_index;
  real_T *c6_num_delivered_address;
  int32_T c6_num_delivered_index;
  real_T (*c6_ref)[3];
  real_T (*c6_cmd)[3];
  real_T (*c6_state)[3];
} SFc6_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc6_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c6_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c6_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c6_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
