#ifndef __c5_PackageDeliverySim_h__
#define __c5_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc5_PackageDeliverySimInstanceStruct
#define typedef_SFc5_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c5_sfEvent;
  boolean_T c5_isStable;
  boolean_T c5_doneDoubleBufferReInit;
  uint8_T c5_is_active_c5_PackageDeliverySim;
  uint32_T c5_method;
  boolean_T c5_method_not_empty;
  uint32_T c5_state;
  boolean_T c5_state_not_empty;
  uint32_T c5_b_state[2];
  boolean_T c5_b_state_not_empty;
  uint32_T c5_c_state[625];
  boolean_T c5_c_state_not_empty;
  real_T *c5_lat_s;
  real_T *c5_dest_lat_hist;
  real_T *c5_lng_s;
  real_T *c5_radius_min;
  real_T *c5_radius_max;
  real_T *c5_n_dest;
  real_T *c5_dest_lng_hist;
} SFc5_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc5_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c5_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c5_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c5_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
