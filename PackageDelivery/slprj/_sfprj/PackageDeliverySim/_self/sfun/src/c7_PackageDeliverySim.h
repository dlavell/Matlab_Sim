#ifndef __c7_PackageDeliverySim_h__
#define __c7_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc7_PackageDeliverySimInstanceStruct
#define typedef_SFc7_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c7_sfEvent;
  boolean_T c7_isStable;
  boolean_T c7_doneDoubleBufferReInit;
  uint8_T c7_is_active_c7_PackageDeliverySim;
  real_T *c7_curr_tp_address;
  int32_T c7_curr_tp_index;
  real_T *c7_curr_wp_address;
  int32_T c7_curr_wp_index;
  real_T *c7_num_delivered_address;
  int32_T c7_num_delivered_index;
  real_T (*c7_pos_address)[3];
  int32_T c7_pos_index;
  real_T (*c7_cmd)[3];
  real_T (*c7_state)[3];
} SFc7_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc7_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c7_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c7_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c7_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
