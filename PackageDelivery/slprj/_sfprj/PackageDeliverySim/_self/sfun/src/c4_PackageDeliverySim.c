/* Include files */

#include <stddef.h>
#include "blas.h"
#include "PackageDeliverySim_sfun.h"
#include "c4_PackageDeliverySim.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "PackageDeliverySim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c4_debug_family_names[10] = { "dest", "num_dest_points",
  "waypoint1", "waypoint2", "origin", "destinations", "nargin", "nargout",
  "altitude", "trajectory" };

/* Function Declarations */
static void initialize_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void initialize_params_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void enable_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance);
static void disable_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct *
  chartInstance);
static void c4_update_debugger_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void set_sim_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance, const mxArray *c4_st);
static void finalize_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance);
static void sf_gateway_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void mdl_start_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void initSimStructsc4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_trajectory, const char_T *c4_identifier,
  real_T c4_y[21]);
static void c4_b_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[21]);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_c_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_d_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[9]);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_e_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_info_helper(const mxArray **c4_info);
static const mxArray *c4_emlrt_marshallOut(const char * c4_u);
static const mxArray *c4_b_emlrt_marshallOut(const uint32_T c4_u);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_f_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_g_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_PackageDeliverySim, const
  char_T *c4_identifier);
static uint8_T c4_h_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static real_T c4_get_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex);
static void c4_set_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex, real_T c4_elementValue);
static real_T *c4_access_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_rdOnly);
static void init_dsm_address_info(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c4_is_active_c4_PackageDeliverySim = 0U;
}

static void initialize_params_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct *
  chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c4_update_debugger_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  int32_T c4_i0;
  real_T c4_u[21];
  const mxArray *c4_b_y = NULL;
  uint8_T c4_hoistedGlobal;
  uint8_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellmatrix(2, 1), false);
  for (c4_i0 = 0; c4_i0 < 21; c4_i0++) {
    c4_u[c4_i0] = (*chartInstance->c4_trajectory)[c4_i0];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 7, 3), false);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_hoistedGlobal = chartInstance->c4_is_active_c4_PackageDeliverySim;
  c4_b_u = c4_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, false);
  return c4_st;
}

static void set_sim_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[21];
  int32_T c4_i1;
  chartInstance->c4_doneDoubleBufferReInit = true;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                      "trajectory", c4_dv0);
  for (c4_i1 = 0; c4_i1 < 21; c4_i1++) {
    (*chartInstance->c4_trajectory)[c4_i1] = c4_dv0[c4_i1];
  }

  chartInstance->c4_is_active_c4_PackageDeliverySim = c4_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_PackageDeliverySim");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_PackageDeliverySim(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_PackageDeliverySim(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  real_T c4_b_hoistedGlobal;
  int32_T c4_i2;
  real_T c4_b_origin[3];
  real_T c4_b_destinations;
  real_T c4_b_altitude;
  uint32_T c4_debug_family_var_map[10];
  real_T c4_dest[3];
  real_T c4_num_dest_points;
  real_T c4_waypoint1[3];
  real_T c4_waypoint2[3];
  real_T c4_c_origin[3];
  real_T c4_c_destinations[9];
  real_T c4_nargin = 3.0;
  real_T c4_nargout = 1.0;
  real_T c4_b_trajectory[21];
  int32_T c4_i3;
  int32_T c4_i4;
  int32_T c4_i5;
  int32_T c4_i6;
  static real_T c4_dv1[9] = { 10.0, 0.0, 10.0, 10.0, 15.0, 15.0, 0.0, 0.0, 0.0 };

  int32_T c4_i7;
  int32_T c4_i8;
  int32_T c4_i9;
  int32_T c4_i10;
  int32_T c4_i11;
  int32_T c4_i12;
  int32_T c4_i13;
  int32_T c4_i14;
  int32_T c4_i15;
  int32_T c4_i16;
  int32_T c4_i17;
  int32_T c4_i18;
  int32_T c4_i19;
  int32_T c4_i20;
  int32_T c4_i21;
  int32_T c4_i22;
  int32_T c4_i23;
  int32_T c4_i24;
  int32_T c4_i25;
  int32_T c4_i26;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  chartInstance->c4_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *chartInstance->c4_destinations;
  c4_b_hoistedGlobal = *chartInstance->c4_altitude;
  for (c4_i2 = 0; c4_i2 < 3; c4_i2++) {
    c4_b_origin[c4_i2] = (*chartInstance->c4_origin)[c4_i2];
  }

  c4_b_destinations = c4_hoistedGlobal;
  c4_b_altitude = c4_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 12U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_dest, 0U, c4_e_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_num_dest_points, 1U,
    c4_b_sf_marshallOut, c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_waypoint1, 2U, c4_e_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_waypoint2, 3U, c4_e_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_c_origin, MAX_uint32_T,
    c4_e_sf_marshallOut, c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_c_destinations, MAX_uint32_T,
    c4_d_sf_marshallOut, c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 6U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 7U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_origin, 4U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_destinations, 5U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_altitude, 8U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_trajectory, 9U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  for (c4_i3 = 0; c4_i3 < 3; c4_i3++) {
    c4_dest[c4_i3] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  for (c4_i4 = 0; c4_i4 < 21; c4_i4++) {
    c4_b_trajectory[c4_i4] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 7);
  for (c4_i5 = 0; c4_i5 < 3; c4_i5++) {
    c4_c_origin[c4_i5] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 8);
  for (c4_i6 = 0; c4_i6 < 9; c4_i6++) {
    c4_c_destinations[c4_i6] = c4_dv1[c4_i6];
  }

  _SFD_SYMBOL_SWITCH(5U, 5U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 11);
  for (c4_i7 = 0; c4_i7 < 3; c4_i7++) {
    c4_c_destinations[c4_i7] = -c4_c_destinations[c4_i7];
  }

  _SFD_SYMBOL_SWITCH(5U, 5U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 12);
  c4_num_dest_points = 3.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 13);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c4_num_dest_points,
        c4_get_num_delivered(chartInstance, 0), -1, 4U, c4_num_dest_points >
        c4_get_num_delivered(chartInstance, 0)))) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 14);
    c4_i8 = _SFD_EML_ARRAY_BOUNDS_CHECK("destinations", (int32_T)
      _SFD_INTEGER_CHECK("num_delivered+1", c4_get_num_delivered(chartInstance,
      0) + 1.0), 1, 3, 1, 0) - 1;
    for (c4_i9 = 0; c4_i9 < 3; c4_i9++) {
      c4_dest[c4_i9] = c4_c_destinations[c4_i8 + 3 * c4_i9];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 17);
  c4_waypoint1[0] = c4_c_origin[0];
  c4_waypoint1[1] = c4_c_origin[1];
  c4_waypoint1[2] = -c4_b_altitude;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 18);
  c4_waypoint2[0] = c4_dest[0];
  c4_waypoint2[1] = c4_dest[1];
  c4_waypoint2[2] = -c4_b_altitude;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 20);
  c4_i10 = 0;
  for (c4_i11 = 0; c4_i11 < 3; c4_i11++) {
    c4_b_trajectory[c4_i10] = c4_c_origin[c4_i11];
    c4_i10 += 7;
  }

  c4_i12 = 0;
  for (c4_i13 = 0; c4_i13 < 3; c4_i13++) {
    c4_b_trajectory[c4_i12 + 1] = c4_waypoint1[c4_i13];
    c4_i12 += 7;
  }

  c4_i14 = 0;
  for (c4_i15 = 0; c4_i15 < 3; c4_i15++) {
    c4_b_trajectory[c4_i14 + 2] = c4_waypoint2[c4_i15];
    c4_i14 += 7;
  }

  c4_i16 = 0;
  for (c4_i17 = 0; c4_i17 < 3; c4_i17++) {
    c4_b_trajectory[c4_i16 + 3] = c4_dest[c4_i17];
    c4_i16 += 7;
  }

  c4_i18 = 0;
  for (c4_i19 = 0; c4_i19 < 3; c4_i19++) {
    c4_b_trajectory[c4_i18 + 4] = c4_waypoint2[c4_i19];
    c4_i18 += 7;
  }

  c4_i20 = 0;
  for (c4_i21 = 0; c4_i21 < 3; c4_i21++) {
    c4_b_trajectory[c4_i20 + 5] = c4_waypoint1[c4_i21];
    c4_i20 += 7;
  }

  c4_i22 = 0;
  for (c4_i23 = 0; c4_i23 < 3; c4_i23++) {
    c4_b_trajectory[c4_i22 + 6] = c4_c_origin[c4_i23];
    c4_i22 += 7;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -20);
  _SFD_SYMBOL_SCOPE_POP();
  for (c4_i24 = 0; c4_i24 < 21; c4_i24++) {
    (*chartInstance->c4_trajectory)[c4_i24] = c4_b_trajectory[c4_i24];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_PackageDeliverySimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c4_i25 = 0; c4_i25 < 21; c4_i25++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_trajectory)[c4_i25], 0U);
  }

  for (c4_i26 = 0; c4_i26 < 3; c4_i26++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_origin)[c4_i26], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_destinations, 2U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_altitude, 3U);
}

static void mdl_start_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber)
{
  (void)c4_machineNumber;
  (void)c4_chartNumber;
  (void)c4_instanceNumber;
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i27;
  int32_T c4_i28;
  int32_T c4_i29;
  real_T c4_b_inData[21];
  int32_T c4_i30;
  int32_T c4_i31;
  int32_T c4_i32;
  real_T c4_u[21];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i27 = 0;
  for (c4_i28 = 0; c4_i28 < 3; c4_i28++) {
    for (c4_i29 = 0; c4_i29 < 7; c4_i29++) {
      c4_b_inData[c4_i29 + c4_i27] = (*(real_T (*)[21])c4_inData)[c4_i29 +
        c4_i27];
    }

    c4_i27 += 7;
  }

  c4_i30 = 0;
  for (c4_i31 = 0; c4_i31 < 3; c4_i31++) {
    for (c4_i32 = 0; c4_i32 < 7; c4_i32++) {
      c4_u[c4_i32 + c4_i30] = c4_b_inData[c4_i32 + c4_i30];
    }

    c4_i30 += 7;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 7, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_trajectory, const char_T *c4_identifier,
  real_T c4_y[21])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_trajectory), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_b_trajectory);
}

static void c4_b_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[21])
{
  real_T c4_dv2[21];
  int32_T c4_i33;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv2, 1, 0, 0U, 1, 0U, 2, 7, 3);
  for (c4_i33 = 0; c4_i33 < 21; c4_i33++) {
    c4_y[c4_i33] = c4_dv2[c4_i33];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_trajectory;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[21];
  int32_T c4_i34;
  int32_T c4_i35;
  int32_T c4_i36;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_trajectory = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_trajectory), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_b_trajectory);
  c4_i34 = 0;
  for (c4_i35 = 0; c4_i35 < 3; c4_i35++) {
    for (c4_i36 = 0; c4_i36 < 7; c4_i36++) {
      (*(real_T (*)[21])c4_outData)[c4_i36 + c4_i34] = c4_y[c4_i36 + c4_i34];
    }

    c4_i34 += 7;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i37;
  real_T c4_b_inData[3];
  int32_T c4_i38;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i37 = 0; c4_i37 < 3; c4_i37++) {
    c4_b_inData[c4_i37] = (*(real_T (*)[3])c4_inData)[c4_i37];
  }

  for (c4_i38 = 0; c4_i38 < 3; c4_i38++) {
    c4_u[c4_i38] = c4_b_inData[c4_i38];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static real_T c4_c_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_nargout;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_nargout = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i39;
  int32_T c4_i40;
  int32_T c4_i41;
  real_T c4_b_inData[9];
  int32_T c4_i42;
  int32_T c4_i43;
  int32_T c4_i44;
  real_T c4_u[9];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i39 = 0;
  for (c4_i40 = 0; c4_i40 < 3; c4_i40++) {
    for (c4_i41 = 0; c4_i41 < 3; c4_i41++) {
      c4_b_inData[c4_i41 + c4_i39] = (*(real_T (*)[9])c4_inData)[c4_i41 + c4_i39];
    }

    c4_i39 += 3;
  }

  c4_i42 = 0;
  for (c4_i43 = 0; c4_i43 < 3; c4_i43++) {
    for (c4_i44 = 0; c4_i44 < 3; c4_i44++) {
      c4_u[c4_i44 + c4_i42] = c4_b_inData[c4_i44 + c4_i42];
    }

    c4_i42 += 3;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_d_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[9])
{
  real_T c4_dv3[9];
  int32_T c4_i45;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv3, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c4_i45 = 0; c4_i45 < 9; c4_i45++) {
    c4_y[c4_i45] = c4_dv3[c4_i45];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_destinations;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[9];
  int32_T c4_i46;
  int32_T c4_i47;
  int32_T c4_i48;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_destinations = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_destinations), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_b_destinations);
  c4_i46 = 0;
  for (c4_i47 = 0; c4_i47 < 3; c4_i47++) {
    for (c4_i48 = 0; c4_i48 < 3; c4_i48++) {
      (*(real_T (*)[9])c4_outData)[c4_i48 + c4_i46] = c4_y[c4_i48 + c4_i46];
    }

    c4_i46 += 3;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i49;
  real_T c4_b_inData[3];
  int32_T c4_i50;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i49 = 0; c4_i49 < 3; c4_i49++) {
    c4_b_inData[c4_i49] = (*(real_T (*)[3])c4_inData)[c4_i49];
  }

  for (c4_i50 = 0; c4_i50 < 3; c4_i50++) {
    c4_u[c4_i50] = c4_b_inData[c4_i50];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_e_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3])
{
  real_T c4_dv4[3];
  int32_T c4_i51;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv4, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c4_i51 = 0; c4_i51 < 3; c4_i51++) {
    c4_y[c4_i51] = c4_dv4[c4_i51];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_origin;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[3];
  int32_T c4_i52;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_origin = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_origin), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_origin);
  for (c4_i52 = 0; c4_i52 < 3; c4_i52++) {
    (*(real_T (*)[3])c4_outData)[c4_i52] = c4_y[c4_i52];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_PackageDeliverySim_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  sf_mex_assign(&c4_nameCaptureInfo, sf_mex_createstruct("structure", 2, 8, 1),
                false);
  c4_info_helper(&c4_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(const mxArray **c4_info)
{
  const mxArray *c4_rhs0 = NULL;
  const mxArray *c4_lhs0 = NULL;
  const mxArray *c4_rhs1 = NULL;
  const mxArray *c4_lhs1 = NULL;
  const mxArray *c4_rhs2 = NULL;
  const mxArray *c4_lhs2 = NULL;
  const mxArray *c4_rhs3 = NULL;
  const mxArray *c4_lhs3 = NULL;
  const mxArray *c4_rhs4 = NULL;
  const mxArray *c4_lhs4 = NULL;
  const mxArray *c4_rhs5 = NULL;
  const mxArray *c4_lhs5 = NULL;
  const mxArray *c4_rhs6 = NULL;
  const mxArray *c4_lhs6 = NULL;
  const mxArray *c4_rhs7 = NULL;
  const mxArray *c4_lhs7 = NULL;
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("length"), "name", "name", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1303178606U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c4_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("mrdivide"), "name", "name", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1410840048U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1370042286U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c4_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389750174U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c4_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("rdivide"), "name", "name", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742680U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c4_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c4_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851196U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c4_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_div"), "name", "name", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1386456352U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c4_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c4_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs7), "lhs", "lhs", 7);
  sf_mex_destroy(&c4_rhs0);
  sf_mex_destroy(&c4_lhs0);
  sf_mex_destroy(&c4_rhs1);
  sf_mex_destroy(&c4_lhs1);
  sf_mex_destroy(&c4_rhs2);
  sf_mex_destroy(&c4_lhs2);
  sf_mex_destroy(&c4_rhs3);
  sf_mex_destroy(&c4_lhs3);
  sf_mex_destroy(&c4_rhs4);
  sf_mex_destroy(&c4_lhs4);
  sf_mex_destroy(&c4_rhs5);
  sf_mex_destroy(&c4_lhs5);
  sf_mex_destroy(&c4_rhs6);
  sf_mex_destroy(&c4_lhs6);
  sf_mex_destroy(&c4_rhs7);
  sf_mex_destroy(&c4_lhs7);
}

static const mxArray *c4_emlrt_marshallOut(const char * c4_u)
{
  const mxArray *c4_y = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c4_u)), false);
  return c4_y;
}

static const mxArray *c4_b_emlrt_marshallOut(const uint32_T c4_u)
{
  const mxArray *c4_y = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 7, 0U, 0U, 0U, 0), false);
  return c4_y;
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static int32_T c4_f_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i53;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i53, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i53;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_g_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_PackageDeliverySim, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_PackageDeliverySim), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_PackageDeliverySim);
  return c4_y;
}

static uint8_T c4_h_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static real_T c4_get_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 0, NULL, c4_elementIndex);
  return *chartInstance->c4_num_delivered_address;
}

static void c4_set_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex, real_T c4_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 0, NULL, c4_elementIndex);
  *chartInstance->c4_num_delivered_address = c4_elementValue;
}

static real_T *c4_access_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_rdOnly)
{
  real_T *c4_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 0, NULL);
  c4_dsmAddr = chartInstance->c4_num_delivered_address;
  if (c4_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 0, NULL);
  }

  return c4_dsmAddr;
}

static void init_dsm_address_info(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "num_delivered", (void
    **)&chartInstance->c4_num_delivered_address,
    &chartInstance->c4_num_delivered_index);
}

static void init_simulink_io_address(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  chartInstance->c4_trajectory = (real_T (*)[21])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_origin = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c4_destinations = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_altitude = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c4_PackageDeliverySim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1817827172U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1970282714U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1799832751U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(352235992U);
}

mxArray* sf_c4_PackageDeliverySim_get_post_codegen_info(void);
mxArray *sf_c4_PackageDeliverySim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("mbytYrrX0lBSQbhynMQHiG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(7);
      pr[1] = (double)(3);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c4_PackageDeliverySim_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c4_PackageDeliverySim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c4_PackageDeliverySim_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c4_PackageDeliverySim_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c4_PackageDeliverySim_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c4_PackageDeliverySim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"trajectory\",},{M[8],M[0],T\"is_active_c4_PackageDeliverySim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_PackageDeliverySim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_PackageDeliverySimInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _PackageDeliverySimMachineNumber_,
           4,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_PackageDeliverySimMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_PackageDeliverySimMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _PackageDeliverySimMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"trajectory");
          _SFD_SET_DATA_PROPS(1,1,1,0,"origin");
          _SFD_SET_DATA_PROPS(2,1,1,0,"destinations");
          _SFD_SET_DATA_PROPS(3,1,1,0,"altitude");
          _SFD_SET_DATA_PROPS(4,11,0,0,"num_delivered");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,578);
        _SFD_CV_INIT_EML_IF(0,1,0,311,346,-1,399);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,314,345,-1,4);

        {
          unsigned int dimVector[2];
          dimVector[0]= 7;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)
            c4_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c4_trajectory);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c4_origin);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c4_destinations);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c4_altitude);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c4_num_delivered_address);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _PackageDeliverySimMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "GEMjcO4joZO5zJDrbntEMH";
}

static void sf_opaque_initialize_c4_PackageDeliverySim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
  initialize_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c4_PackageDeliverySim(void *chartInstanceVar)
{
  enable_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c4_PackageDeliverySim(void *chartInstanceVar)
{
  disable_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c4_PackageDeliverySim(void *chartInstanceVar)
{
  sf_gateway_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c4_PackageDeliverySim(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c4_PackageDeliverySim
    ((SFc4_PackageDeliverySimInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c4_PackageDeliverySim(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c4_PackageDeliverySim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_PackageDeliverySimInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_PackageDeliverySim_optimization_info();
    }

    finalize_c4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_PackageDeliverySim((SFc4_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_PackageDeliverySim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c4_PackageDeliverySim
      ((SFc4_PackageDeliverySimInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_PackageDeliverySim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_PackageDeliverySim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,4,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,4);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3907416089U));
  ssSetChecksum1(S,(1028086625U));
  ssSetChecksum2(S,(1299081445U));
  ssSetChecksum3(S,(1437304420U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c4_PackageDeliverySim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_PackageDeliverySim(SimStruct *S)
{
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)utMalloc(sizeof
    (SFc4_PackageDeliverySimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_PackageDeliverySimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_PackageDeliverySim;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_PackageDeliverySim;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_PackageDeliverySim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_PackageDeliverySim;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_PackageDeliverySim;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_PackageDeliverySim;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_PackageDeliverySim;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_PackageDeliverySim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_PackageDeliverySim;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_PackageDeliverySim;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_PackageDeliverySim;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c4_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_PackageDeliverySim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_PackageDeliverySim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_PackageDeliverySim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_PackageDeliverySim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
