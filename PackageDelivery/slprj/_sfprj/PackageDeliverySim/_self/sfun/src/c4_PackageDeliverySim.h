#ifndef __c4_PackageDeliverySim_h__
#define __c4_PackageDeliverySim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_PackageDeliverySimInstanceStruct
#define typedef_SFc4_PackageDeliverySimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_PackageDeliverySim;
  real_T *c4_num_delivered_address;
  int32_T c4_num_delivered_index;
  real_T *c4_clock;
  real_T (*c4_trajectory)[21];
  real_T (*c4_origin)[3];
  real_T *c4_destinations;
  real_T *c4_altitude;
} SFc4_PackageDeliverySimInstanceStruct;

#endif                                 /*typedef_SFc4_PackageDeliverySimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c4_PackageDeliverySim_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c4_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
extern void c4_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
