/* Include files */

#include <stddef.h>
#include "blas.h"
#include "PackageDeliverySim_sfun.h"
#include "c4_PackageDeliverySim.h"
#include "mwmathutil.h"
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
static const char * c4_debug_family_names[24] = { "u", "state", "speed", "dest",
  "p_", "Dp", "D2p", "D3p", "D4p", "num_dest_points", "temp", "waypoint1",
  "waypoint2", "rise_time", "transit_time", "total_time", "origin", "nargin",
  "nargout", "clock", "destinations", "altitude", "trajectory", "p" };

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
static void c4_chartstep_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void initSimStructsc4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_p, const char_T *c4_identifier, real_T
  c4_y[15]);
static void c4_b_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[15]);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_c_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_trajectory, const char_T *c4_identifier,
  real_T c4_y[12]);
static void c4_d_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[12]);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_e_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_f_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_g_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3]);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, char_T
  c4_inData_data[], int32_T c4_inData_sizes[2]);
static void c4_h_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  char_T c4_y_data[], int32_T c4_y_sizes[2]);
static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, char_T c4_outData_data[], int32_T
  c4_outData_sizes[2]);
static void c4_info_helper(const mxArray **c4_info);
static const mxArray *c4_emlrt_marshallOut(const char * c4_u);
static const mxArray *c4_b_emlrt_marshallOut(const uint32_T c4_u);
static real_T c4_mpower(SFc4_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c4_a);
static void c4_eml_scalar_eg(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c4_eml_error(SFc4_PackageDeliverySimInstanceStruct *chartInstance);
static real_T c4_eml_switch_helper(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, char_T c4_expr_data[], int32_T c4_expr_sizes[2]);
static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_i_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_j_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_PackageDeliverySim, const
  char_T *c4_identifier);
static uint8_T c4_k_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static real_T c4_get_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex);
static void c4_set_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex, real_T c4_elementValue);
static real_T *c4_access_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_rdOnly);
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
  real_T c4_u[15];
  const mxArray *c4_b_y = NULL;
  int32_T c4_i1;
  real_T c4_b_u[12];
  const mxArray *c4_c_y = NULL;
  uint8_T c4_hoistedGlobal;
  uint8_T c4_c_u;
  const mxArray *c4_d_y = NULL;
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellmatrix(3, 1), false);
  for (c4_i0 = 0; c4_i0 < 15; c4_i0++) {
    c4_u[c4_i0] = (*chartInstance->c4_p)[c4_i0];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 15), false);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  for (c4_i1 = 0; c4_i1 < 12; c4_i1++) {
    c4_b_u[c4_i1] = (*chartInstance->c4_trajectory)[c4_i1];
  }

  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", c4_b_u, 0, 0U, 1U, 0U, 2, 4, 3),
                false);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  c4_hoistedGlobal = chartInstance->c4_is_active_c4_PackageDeliverySim;
  c4_c_u = c4_hoistedGlobal;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_y, 2, c4_d_y);
  sf_mex_assign(&c4_st, c4_y, false);
  return c4_st;
}

static void set_sim_state_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[15];
  int32_T c4_i2;
  real_T c4_dv1[12];
  int32_T c4_i3;
  chartInstance->c4_doneDoubleBufferReInit = true;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)), "p",
                      c4_dv0);
  for (c4_i2 = 0; c4_i2 < 15; c4_i2++) {
    (*chartInstance->c4_p)[c4_i2] = c4_dv0[c4_i2];
  }

  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
                        "trajectory", c4_dv1);
  for (c4_i3 = 0; c4_i3 < 12; c4_i3++) {
    (*chartInstance->c4_trajectory)[c4_i3] = c4_dv1[c4_i3];
  }

  chartInstance->c4_is_active_c4_PackageDeliverySim = c4_j_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 2)),
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
  int32_T c4_i4;
  int32_T c4_i5;
  int32_T c4_i6;
  int32_T c4_i7;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_clock, 0U);
  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_PackageDeliverySim(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_PackageDeliverySimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c4_i4 = 0; c4_i4 < 12; c4_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_trajectory)[c4_i4], 1U);
  }

  for (c4_i5 = 0; c4_i5 < 3; c4_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_origin)[c4_i5], 2U);
  }

  for (c4_i6 = 0; c4_i6 < 9; c4_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_destinations)[c4_i6], 3U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_altitude, 4U);
  for (c4_i7 = 0; c4_i7 < 15; c4_i7++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_p)[c4_i7], 7U);
  }
}

static void mdl_start_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c4_chartstep_c4_PackageDeliverySim
  (SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  real_T c4_b_hoistedGlobal;
  real_T c4_b_clock;
  int32_T c4_i8;
  real_T c4_b_origin[3];
  int32_T c4_i9;
  real_T c4_b_destinations[9];
  real_T c4_b_altitude;
  uint32_T c4_debug_family_var_map[24];
  real_T c4_u;
  int32_T c4_state_sizes[2];
  char_T c4_state_data[13];
  real_T c4_speed;
  real_T c4_dest[3];
  real_T c4_p_[3];
  real_T c4_Dp[3];
  real_T c4_D2p[3];
  real_T c4_D3p[3];
  real_T c4_D4p[3];
  real_T c4_num_dest_points;
  real_T c4_temp[3];
  real_T c4_waypoint1[3];
  real_T c4_waypoint2[3];
  real_T c4_rise_time;
  real_T c4_transit_time;
  real_T c4_total_time;
  real_T c4_c_origin[3];
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 2.0;
  real_T c4_b_trajectory[12];
  real_T c4_b_p[15];
  int32_T c4_state;
  int32_T c4_b_state;
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
  int32_T c4_i27;
  int32_T c4_i28;
  int32_T c4_i29;
  int32_T c4_i30;
  real_T c4_A;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_d_x;
  real_T c4_e_x;
  real_T c4_b_A;
  real_T c4_f_x;
  real_T c4_g_x;
  real_T c4_h_x;
  int32_T c4_c_state;
  int32_T c4_d_state;
  int32_T c4_i31;
  static char_T c4_cv0[9] = { 'a', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g' };

  int32_T c4_e_state;
  int32_T c4_f_state;
  int32_T c4_i32;
  static char_T c4_cv1[13] = { 't', 'r', 'a', 'n', 's', 'i', 't', '_', 't', 'h',
    'e', 'r', 'e' };

  int32_T c4_g_state;
  int32_T c4_h_state;
  int32_T c4_i33;
  static char_T c4_cv2[10] = { 'd', 'e', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g'
  };

  int32_T c4_i_state;
  int32_T c4_j_state;
  int32_T c4_i34;
  static char_T c4_cv3[10] = { 'a', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g', '2'
  };

  int32_T c4_k_state;
  int32_T c4_l_state;
  int32_T c4_i35;
  static char_T c4_cv4[12] = { 't', 'r', 'a', 'n', 's', 'i', 't', '_', 'b', 'a',
    'c', 'k' };

  int32_T c4_m_state;
  int32_T c4_n_state;
  int32_T c4_i36;
  static char_T c4_cv5[11] = { 'd', 'e', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g',
    '2' };

  int32_T c4_b_state_sizes[2];
  int32_T c4_o_state;
  int32_T c4_p_state;
  int32_T c4_loop_ub;
  int32_T c4_i37;
  char_T c4_b_state_data[13];
  real_T c4_dv2[3];
  int32_T c4_i38;
  real_T c4_b_dest[3];
  int32_T c4_i39;
  real_T c4_c_dest[3];
  int32_T c4_i40;
  real_T c4_d_dest[3];
  int32_T c4_i41;
  real_T c4_dv3[3];
  int32_T c4_i42;
  int32_T c4_i43;
  int32_T c4_i44;
  int32_T c4_i45;
  int32_T c4_i46;
  int32_T c4_i47;
  int32_T c4_i48;
  int32_T c4_i49;
  int32_T c4_i50;
  int32_T c4_i51;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  boolean_T guard7 = false;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *chartInstance->c4_clock;
  c4_b_hoistedGlobal = *chartInstance->c4_altitude;
  c4_b_clock = c4_hoistedGlobal;
  for (c4_i8 = 0; c4_i8 < 3; c4_i8++) {
    c4_b_origin[c4_i8] = (*chartInstance->c4_origin)[c4_i8];
  }

  for (c4_i9 = 0; c4_i9 < 9; c4_i9++) {
    c4_b_destinations[c4_i9] = (*chartInstance->c4_destinations)[c4_i9];
  }

  c4_b_altitude = c4_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 24U, 25U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_u, 0U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c4_state_data, (const int32_T *)
    &c4_state_sizes, NULL, 0, 1, (void *)c4_g_sf_marshallOut, (void *)
    c4_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_speed, 2U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_dest, 3U, c4_f_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_p_, 4U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_Dp, 5U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_D2p, 6U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_D3p, 7U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_D4p, 8U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_num_dest_points, 9U,
    c4_c_sf_marshallOut, c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_temp, 10U, c4_f_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_waypoint1, 11U, c4_f_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_waypoint2, 12U, c4_f_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_rise_time, 13U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_transit_time, 14U,
    c4_c_sf_marshallOut, c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_total_time, 15U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_c_origin, MAX_uint32_T,
    c4_f_sf_marshallOut, c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 17U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 18U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_clock, 19U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_origin, 16U, c4_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_destinations, 20U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_altitude, 21U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_trajectory, 22U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_p, 23U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  c4_u = c4_b_clock - c4_get_clock_offset(chartInstance, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  c4_state_sizes[0] = 1;
  c4_state_sizes[1] = 0;
  c4_state = c4_state_sizes[0];
  c4_b_state = c4_state_sizes[1];
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 7);
  c4_speed = 1.75;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 8);
  for (c4_i10 = 0; c4_i10 < 3; c4_i10++) {
    c4_dest[c4_i10] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 9);
  for (c4_i11 = 0; c4_i11 < 3; c4_i11++) {
    c4_p_[c4_i11] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 10);
  for (c4_i12 = 0; c4_i12 < 3; c4_i12++) {
    c4_Dp[c4_i12] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 11);
  for (c4_i13 = 0; c4_i13 < 3; c4_i13++) {
    c4_D2p[c4_i13] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 12);
  for (c4_i14 = 0; c4_i14 < 3; c4_i14++) {
    c4_D3p[c4_i14] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 13);
  for (c4_i15 = 0; c4_i15 < 3; c4_i15++) {
    c4_D4p[c4_i15] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 16);
  for (c4_i16 = 0; c4_i16 < 3; c4_i16++) {
    c4_b_destinations[c4_i16] = -c4_b_destinations[c4_i16];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 17);
  c4_num_dest_points = 3.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 18);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, 3.0, c4_get_num_delivered
        (chartInstance, 0), -1, 4U, 3.0 > c4_get_num_delivered(chartInstance, 0))))
  {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 19);
    c4_i17 = _SFD_EML_ARRAY_BOUNDS_CHECK("destinations", (int32_T)
      _SFD_INTEGER_CHECK("num_delivered+1", c4_get_num_delivered(chartInstance,
      0) + 1.0), 1, 3, 1, 0) - 1;
    for (c4_i18 = 0; c4_i18 < 3; c4_i18++) {
      c4_dest[c4_i18] = c4_b_destinations[c4_i17 + 3 * c4_i18];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 21);
  for (c4_i19 = 0; c4_i19 < 3; c4_i19++) {
    c4_c_origin[c4_i19] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(16U, 16U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 22);
  c4_i20 = 0;
  for (c4_i21 = 0; c4_i21 < 3; c4_i21++) {
    c4_temp[c4_i21] = c4_b_destinations[c4_i20];
    c4_i20 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 23);
  for (c4_i22 = 0; c4_i22 < 12; c4_i22++) {
    c4_b_trajectory[c4_i22] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 24);
  c4_waypoint1[0] = 0.0;
  c4_waypoint1[1] = 0.0;
  c4_waypoint1[2] = -c4_b_altitude;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 25);
  c4_waypoint2[0] = c4_b_destinations[0];
  c4_waypoint2[1] = c4_b_destinations[3];
  c4_waypoint2[2] = -c4_b_altitude;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 26);
  c4_i23 = 0;
  for (c4_i24 = 0; c4_i24 < 3; c4_i24++) {
    c4_b_trajectory[c4_i23] = c4_c_origin[c4_i24];
    c4_i23 += 4;
  }

  c4_i25 = 0;
  for (c4_i26 = 0; c4_i26 < 3; c4_i26++) {
    c4_b_trajectory[c4_i25 + 1] = c4_waypoint1[c4_i26];
    c4_i25 += 4;
  }

  c4_i27 = 0;
  for (c4_i28 = 0; c4_i28 < 3; c4_i28++) {
    c4_b_trajectory[c4_i27 + 2] = c4_waypoint2[c4_i28];
    c4_i27 += 4;
  }

  c4_i29 = 0;
  for (c4_i30 = 0; c4_i30 < 3; c4_i30++) {
    c4_b_trajectory[c4_i29 + 3] = c4_temp[c4_i30];
    c4_i29 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 29);
  c4_A = c4_b_altitude;
  c4_x = c4_A;
  c4_b_x = c4_x;
  c4_c_x = c4_b_x;
  c4_rise_time = c4_c_x / 1.75;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 30);
  c4_d_x = c4_mpower(chartInstance, 0.0 - c4_dest[0]) + c4_mpower(chartInstance,
    0.0 - c4_dest[1]);
  c4_e_x = c4_d_x;
  if (c4_e_x < 0.0) {
    c4_eml_error(chartInstance);
  }

  c4_e_x = muDoubleScalarSqrt(c4_e_x);
  c4_b_A = c4_e_x;
  c4_f_x = c4_b_A;
  c4_g_x = c4_f_x;
  c4_h_x = c4_g_x;
  c4_transit_time = c4_h_x / 1.75;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 31);
  c4_total_time = 4.0 * c4_rise_time + 2.0 * c4_transit_time;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 34);
  guard7 = false;
  if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 1, c4_u, c4_total_time, -1,
        4U, c4_u > c4_total_time))) {
    if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 2, c4_u, c4_total_time +
          2.0, -1, 2U, c4_u < c4_total_time + 2.0))) {
      CV_EML_MCDC(0, 1, 0, true);
      CV_EML_IF(0, 1, 1, true);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 35);
      c4_set_num_delivered(chartInstance, 0, c4_get_num_delivered(chartInstance,
        0) + 1.0);
      ssUpdateDataStoreLog(chartInstance->S, 1);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 36);
      c4_set_clock_offset(chartInstance, 0, c4_get_clock_offset(chartInstance, 0)
                          + c4_u);
      ssUpdateDataStoreLog(chartInstance->S, 0);
    } else {
      guard7 = true;
    }
  } else {
    guard7 = true;
  }

  if (guard7 == true) {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 40);
  guard1 = false;
  if (CV_EML_COND(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 3, c4_u, 0.0, -1, 5U, c4_u
        >= 0.0))) {
    if (CV_EML_COND(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 4, c4_u, c4_rise_time,
          -1, 2U, c4_u < c4_rise_time))) {
      CV_EML_MCDC(0, 1, 1, true);
      CV_EML_IF(0, 1, 2, true);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 41);
      c4_state_sizes[0] = 1;
      c4_state_sizes[1] = 9;
      c4_c_state = c4_state_sizes[0];
      c4_d_state = c4_state_sizes[1];
      for (c4_i31 = 0; c4_i31 < 9; c4_i31++) {
        c4_state_data[c4_i31] = c4_cv0[c4_i31];
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 1, false);
    CV_EML_IF(0, 1, 2, false);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 42);
    guard2 = false;
    if (CV_EML_COND(0, 1, 4, CV_RELATIONAL_EVAL(4U, 0U, 5, c4_u, c4_rise_time,
          -1, 5U, c4_u >= c4_rise_time))) {
      if (CV_EML_COND(0, 1, 5, CV_RELATIONAL_EVAL(4U, 0U, 6, c4_u, c4_rise_time
            + c4_transit_time, -1, 2U, c4_u < c4_rise_time + c4_transit_time)))
      {
        CV_EML_MCDC(0, 1, 2, true);
        CV_EML_IF(0, 1, 3, true);
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 43);
        c4_state_sizes[0] = 1;
        c4_state_sizes[1] = 13;
        c4_e_state = c4_state_sizes[0];
        c4_f_state = c4_state_sizes[1];
        for (c4_i32 = 0; c4_i32 < 13; c4_i32++) {
          c4_state_data[c4_i32] = c4_cv1[c4_i32];
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2 == true) {
      CV_EML_MCDC(0, 1, 2, false);
      CV_EML_IF(0, 1, 3, false);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 44);
      guard3 = false;
      if (CV_EML_COND(0, 1, 6, CV_RELATIONAL_EVAL(4U, 0U, 7, c4_u, c4_rise_time
            + c4_transit_time, -1, 5U, c4_u >= c4_rise_time + c4_transit_time)))
      {
        if (CV_EML_COND(0, 1, 7, CV_RELATIONAL_EVAL(4U, 0U, 8, c4_u, 2.0 *
              c4_rise_time + c4_transit_time, -1, 2U, c4_u < 2.0 * c4_rise_time
              + c4_transit_time))) {
          CV_EML_MCDC(0, 1, 3, true);
          CV_EML_IF(0, 1, 4, true);
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 45);
          c4_state_sizes[0] = 1;
          c4_state_sizes[1] = 10;
          c4_g_state = c4_state_sizes[0];
          c4_h_state = c4_state_sizes[1];
          for (c4_i33 = 0; c4_i33 < 10; c4_i33++) {
            c4_state_data[c4_i33] = c4_cv2[c4_i33];
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3 == true) {
        CV_EML_MCDC(0, 1, 3, false);
        CV_EML_IF(0, 1, 4, false);
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 46);
        guard4 = false;
        if (CV_EML_COND(0, 1, 8, CV_RELATIONAL_EVAL(4U, 0U, 9, c4_u, 2.0 *
              c4_rise_time + c4_transit_time, -1, 5U, c4_u >= 2.0 * c4_rise_time
              + c4_transit_time))) {
          if (CV_EML_COND(0, 1, 9, CV_RELATIONAL_EVAL(4U, 0U, 10, c4_u, 3.0 *
                c4_rise_time + c4_transit_time, -1, 2U, c4_u < 3.0 *
                c4_rise_time + c4_transit_time))) {
            CV_EML_MCDC(0, 1, 4, true);
            CV_EML_IF(0, 1, 5, true);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 47);
            c4_state_sizes[0] = 1;
            c4_state_sizes[1] = 10;
            c4_i_state = c4_state_sizes[0];
            c4_j_state = c4_state_sizes[1];
            for (c4_i34 = 0; c4_i34 < 10; c4_i34++) {
              c4_state_data[c4_i34] = c4_cv3[c4_i34];
            }
          } else {
            guard4 = true;
          }
        } else {
          guard4 = true;
        }

        if (guard4 == true) {
          CV_EML_MCDC(0, 1, 4, false);
          CV_EML_IF(0, 1, 5, false);
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 48);
          guard5 = false;
          if (CV_EML_COND(0, 1, 10, CV_RELATIONAL_EVAL(4U, 0U, 11, c4_u, 3.0 *
                c4_rise_time + c4_transit_time, -1, 5U, c4_u >= 3.0 *
                c4_rise_time + c4_transit_time))) {
            if (CV_EML_COND(0, 1, 11, CV_RELATIONAL_EVAL(4U, 0U, 12, c4_u, 3.0 *
                  c4_rise_time + 2.0 * c4_transit_time, -1, 2U, c4_u < 3.0 *
                  c4_rise_time + 2.0 * c4_transit_time))) {
              CV_EML_MCDC(0, 1, 5, true);
              CV_EML_IF(0, 1, 6, true);
              _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 49);
              c4_state_sizes[0] = 1;
              c4_state_sizes[1] = 12;
              c4_k_state = c4_state_sizes[0];
              c4_l_state = c4_state_sizes[1];
              for (c4_i35 = 0; c4_i35 < 12; c4_i35++) {
                c4_state_data[c4_i35] = c4_cv4[c4_i35];
              }
            } else {
              guard5 = true;
            }
          } else {
            guard5 = true;
          }

          if (guard5 == true) {
            CV_EML_MCDC(0, 1, 5, false);
            CV_EML_IF(0, 1, 6, false);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 50);
            guard6 = false;
            if (CV_EML_COND(0, 1, 12, CV_RELATIONAL_EVAL(4U, 0U, 13, c4_u, 3.0 *
                  c4_rise_time + 2.0 * c4_transit_time, -1, 5U, c4_u >= 3.0 *
                  c4_rise_time + 2.0 * c4_transit_time))) {
              if (CV_EML_COND(0, 1, 13, CV_RELATIONAL_EVAL(4U, 0U, 14, c4_u, 4.0
                    * c4_rise_time + 2.0 * c4_transit_time, -1, 2U, c4_u < 4.0 *
                    c4_rise_time + 2.0 * c4_transit_time))) {
                CV_EML_MCDC(0, 1, 6, true);
                CV_EML_IF(0, 1, 7, true);
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 51);
                c4_state_sizes[0] = 1;
                c4_state_sizes[1] = 11;
                c4_m_state = c4_state_sizes[0];
                c4_n_state = c4_state_sizes[1];
                for (c4_i36 = 0; c4_i36 < 11; c4_i36++) {
                  c4_state_data[c4_i36] = c4_cv5[c4_i36];
                }
              } else {
                guard6 = true;
              }
            } else {
              guard6 = true;
            }

            if (guard6 == true) {
              CV_EML_MCDC(0, 1, 6, false);
              CV_EML_IF(0, 1, 7, false);
            }
          }
        }
      }
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 56);
  c4_b_state_sizes[0] = 1;
  c4_b_state_sizes[1] = c4_state_sizes[1];
  c4_o_state = c4_b_state_sizes[0];
  c4_p_state = c4_b_state_sizes[1];
  c4_loop_ub = c4_state_sizes[0] * c4_state_sizes[1] - 1;
  for (c4_i37 = 0; c4_i37 <= c4_loop_ub; c4_i37++) {
    c4_b_state_data[c4_i37] = c4_state_data[c4_i37];
  }

  switch ((int32_T)c4_eml_switch_helper(chartInstance, c4_b_state_data,
           c4_b_state_sizes)) {
   case 0:
    CV_EML_SWITCH(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 58);
    c4_dv2[0] = 0.0;
    c4_dv2[1] = 0.0;
    c4_dv2[2] = -c4_b_altitude;
    for (c4_i38 = 0; c4_i38 < 3; c4_i38++) {
      c4_p_[c4_i38] = c4_dv2[c4_i38];
    }
    break;

   case 1:
    CV_EML_SWITCH(0, 1, 0, 2);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 60);
    c4_b_dest[0] = c4_dest[0];
    c4_b_dest[1] = c4_dest[1];
    c4_b_dest[2] = -c4_b_altitude;
    for (c4_i39 = 0; c4_i39 < 3; c4_i39++) {
      c4_p_[c4_i39] = c4_b_dest[c4_i39];
    }
    break;

   case 2:
    CV_EML_SWITCH(0, 1, 0, 3);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 62);
    c4_c_dest[0] = c4_dest[0];
    c4_c_dest[1] = c4_dest[1];
    c4_c_dest[2] = 0.0;
    for (c4_i40 = 0; c4_i40 < 3; c4_i40++) {
      c4_p_[c4_i40] = c4_c_dest[c4_i40];
    }
    break;

   case 3:
    CV_EML_SWITCH(0, 1, 0, 4);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 64);
    c4_d_dest[0] = c4_dest[0];
    c4_d_dest[1] = c4_dest[1];
    c4_d_dest[2] = -c4_b_altitude;
    for (c4_i41 = 0; c4_i41 < 3; c4_i41++) {
      c4_p_[c4_i41] = c4_d_dest[c4_i41];
    }
    break;

   case 4:
    CV_EML_SWITCH(0, 1, 0, 5);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 66);
    c4_dv3[0] = 0.0;
    c4_dv3[1] = 0.0;
    c4_dv3[2] = -c4_b_altitude;
    for (c4_i42 = 0; c4_i42 < 3; c4_i42++) {
      c4_p_[c4_i42] = c4_dv3[c4_i42];
    }
    break;

   case 5:
    CV_EML_SWITCH(0, 1, 0, 6);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 68);
    for (c4_i43 = 0; c4_i43 < 3; c4_i43++) {
      c4_p_[c4_i43] = 0.0;
    }
    break;

   default:
    CV_EML_SWITCH(0, 1, 0, 0);
    break;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 71);
  if (CV_EML_IF(0, 1, 8, CV_RELATIONAL_EVAL(4U, 0U, 15, c4_get_num_delivered
        (chartInstance, 0), 3.0, -1, 0U, c4_get_num_delivered(chartInstance, 0) ==
        3.0))) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 72);
    for (c4_i44 = 0; c4_i44 < 3; c4_i44++) {
      c4_p_[c4_i44] = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 76);
  for (c4_i45 = 0; c4_i45 < 3; c4_i45++) {
    c4_b_p[c4_i45] = c4_p_[c4_i45];
  }

  for (c4_i46 = 0; c4_i46 < 3; c4_i46++) {
    c4_b_p[c4_i46 + 3] = c4_Dp[c4_i46];
  }

  for (c4_i47 = 0; c4_i47 < 3; c4_i47++) {
    c4_b_p[c4_i47 + 6] = c4_D2p[c4_i47];
  }

  for (c4_i48 = 0; c4_i48 < 3; c4_i48++) {
    c4_b_p[c4_i48 + 9] = c4_D3p[c4_i48];
  }

  for (c4_i49 = 0; c4_i49 < 3; c4_i49++) {
    c4_b_p[c4_i49 + 12] = c4_D4p[c4_i49];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -76);
  _SFD_SYMBOL_SCOPE_POP();
  for (c4_i50 = 0; c4_i50 < 12; c4_i50++) {
    (*chartInstance->c4_trajectory)[c4_i50] = c4_b_trajectory[c4_i50];
  }

  for (c4_i51 = 0; c4_i51 < 15; c4_i51++) {
    (*chartInstance->c4_p)[c4_i51] = c4_b_p[c4_i51];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
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
  int32_T c4_i52;
  real_T c4_b_inData[15];
  int32_T c4_i53;
  real_T c4_u[15];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i52 = 0; c4_i52 < 15; c4_i52++) {
    c4_b_inData[c4_i52] = (*(real_T (*)[15])c4_inData)[c4_i52];
  }

  for (c4_i53 = 0; c4_i53 < 15; c4_i53++) {
    c4_u[c4_i53] = c4_b_inData[c4_i53];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 15), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_p, const char_T *c4_identifier, real_T
  c4_y[15])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_p), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_p);
}

static void c4_b_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[15])
{
  real_T c4_dv4[15];
  int32_T c4_i54;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv4, 1, 0, 0U, 1, 0U, 1, 15);
  for (c4_i54 = 0; c4_i54 < 15; c4_i54++) {
    c4_y[c4_i54] = c4_dv4[c4_i54];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_p;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[15];
  int32_T c4_i55;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_p = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_p), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_p);
  for (c4_i55 = 0; c4_i55 < 15; c4_i55++) {
    (*(real_T (*)[15])c4_outData)[c4_i55] = c4_y[c4_i55];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i56;
  int32_T c4_i57;
  int32_T c4_i58;
  real_T c4_b_inData[12];
  int32_T c4_i59;
  int32_T c4_i60;
  int32_T c4_i61;
  real_T c4_u[12];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i56 = 0;
  for (c4_i57 = 0; c4_i57 < 3; c4_i57++) {
    for (c4_i58 = 0; c4_i58 < 4; c4_i58++) {
      c4_b_inData[c4_i58 + c4_i56] = (*(real_T (*)[12])c4_inData)[c4_i58 +
        c4_i56];
    }

    c4_i56 += 4;
  }

  c4_i59 = 0;
  for (c4_i60 = 0; c4_i60 < 3; c4_i60++) {
    for (c4_i61 = 0; c4_i61 < 4; c4_i61++) {
      c4_u[c4_i61 + c4_i59] = c4_b_inData[c4_i61 + c4_i59];
    }

    c4_i59 += 4;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 4, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_c_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_trajectory, const char_T *c4_identifier,
  real_T c4_y[12])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_trajectory), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_b_trajectory);
}

static void c4_d_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[12])
{
  real_T c4_dv5[12];
  int32_T c4_i62;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv5, 1, 0, 0U, 1, 0U, 2, 4, 3);
  for (c4_i62 = 0; c4_i62 < 12; c4_i62++) {
    c4_y[c4_i62] = c4_dv5[c4_i62];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_trajectory;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[12];
  int32_T c4_i63;
  int32_T c4_i64;
  int32_T c4_i65;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_trajectory = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_trajectory), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_b_trajectory);
  c4_i63 = 0;
  for (c4_i64 = 0; c4_i64 < 3; c4_i64++) {
    for (c4_i65 = 0; c4_i65 < 4; c4_i65++) {
      (*(real_T (*)[12])c4_outData)[c4_i65 + c4_i63] = c4_y[c4_i65 + c4_i63];
    }

    c4_i63 += 4;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
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

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i66;
  int32_T c4_i67;
  int32_T c4_i68;
  real_T c4_b_inData[9];
  int32_T c4_i69;
  int32_T c4_i70;
  int32_T c4_i71;
  real_T c4_u[9];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i66 = 0;
  for (c4_i67 = 0; c4_i67 < 3; c4_i67++) {
    for (c4_i68 = 0; c4_i68 < 3; c4_i68++) {
      c4_b_inData[c4_i68 + c4_i66] = (*(real_T (*)[9])c4_inData)[c4_i68 + c4_i66];
    }

    c4_i66 += 3;
  }

  c4_i69 = 0;
  for (c4_i70 = 0; c4_i70 < 3; c4_i70++) {
    for (c4_i71 = 0; c4_i71 < 3; c4_i71++) {
      c4_u[c4_i71 + c4_i69] = c4_b_inData[c4_i71 + c4_i69];
    }

    c4_i69 += 3;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i72;
  real_T c4_b_inData[3];
  int32_T c4_i73;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i72 = 0; c4_i72 < 3; c4_i72++) {
    c4_b_inData[c4_i72] = (*(real_T (*)[3])c4_inData)[c4_i72];
  }

  for (c4_i73 = 0; c4_i73 < 3; c4_i73++) {
    c4_u[c4_i73] = c4_b_inData[c4_i73];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static real_T c4_e_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
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

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c4_y = c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i74;
  real_T c4_b_inData[3];
  int32_T c4_i75;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i74 = 0; c4_i74 < 3; c4_i74++) {
    c4_b_inData[c4_i74] = (*(real_T (*)[3])c4_inData)[c4_i74];
  }

  for (c4_i75 = 0; c4_i75 < 3; c4_i75++) {
    c4_u[c4_i75] = c4_b_inData[c4_i75];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_f_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3])
{
  real_T c4_dv6[3];
  int32_T c4_i76;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv6, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c4_i76 = 0; c4_i76 < 3; c4_i76++) {
    c4_y[c4_i76] = c4_dv6[c4_i76];
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
  int32_T c4_i77;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_b_origin = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_origin), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_origin);
  for (c4_i77 = 0; c4_i77 < 3; c4_i77++) {
    (*(real_T (*)[3])c4_outData)[c4_i77] = c4_y[c4_i77];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static void c4_g_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3])
{
  real_T c4_dv7[3];
  int32_T c4_i78;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv7, 1, 0, 0U, 1, 0U, 1, 3);
  for (c4_i78 = 0; c4_i78 < 3; c4_i78++) {
    c4_y[c4_i78] = c4_dv7[c4_i78];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_D4p;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[3];
  int32_T c4_i79;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_D4p = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_D4p), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_D4p);
  for (c4_i79 = 0; c4_i79 < 3; c4_i79++) {
    (*(real_T (*)[3])c4_outData)[c4_i79] = c4_y[c4_i79];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, char_T
  c4_inData_data[], int32_T c4_inData_sizes[2])
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u_sizes[2];
  int32_T c4_u;
  int32_T c4_b_u;
  int32_T c4_inData;
  int32_T c4_b_inData;
  int32_T c4_b_inData_sizes;
  int32_T c4_loop_ub;
  int32_T c4_i80;
  char_T c4_b_inData_data[13];
  int32_T c4_b_loop_ub;
  int32_T c4_i81;
  char_T c4_u_data[13];
  const mxArray *c4_y = NULL;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u_sizes[0] = 1;
  c4_u_sizes[1] = c4_inData_sizes[1];
  c4_u = c4_u_sizes[0];
  c4_b_u = c4_u_sizes[1];
  c4_inData = c4_inData_sizes[0];
  c4_b_inData = c4_inData_sizes[1];
  c4_b_inData_sizes = c4_inData * c4_b_inData;
  c4_loop_ub = c4_inData * c4_b_inData - 1;
  for (c4_i80 = 0; c4_i80 <= c4_loop_ub; c4_i80++) {
    c4_b_inData_data[c4_i80] = c4_inData_data[c4_i80];
  }

  c4_b_loop_ub = c4_b_inData_sizes - 1;
  for (c4_i81 = 0; c4_i81 <= c4_b_loop_ub; c4_i81++) {
    c4_u_data[c4_i81] = c4_b_inData_data[c4_i81];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u_data, 10, 0U, 1U, 0U, 2,
    c4_u_sizes[0], c4_u_sizes[1]), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_h_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  char_T c4_y_data[], int32_T c4_y_sizes[2])
{
  int32_T c4_i82;
  uint32_T c4_uv0[2];
  int32_T c4_i83;
  static boolean_T c4_bv0[2] = { false, true };

  boolean_T c4_bv1[2];
  int32_T c4_tmp_sizes[2];
  char_T c4_tmp_data[13];
  int32_T c4_y;
  int32_T c4_b_y;
  int32_T c4_loop_ub;
  int32_T c4_i84;
  (void)chartInstance;
  for (c4_i82 = 0; c4_i82 < 2; c4_i82++) {
    c4_uv0[c4_i82] = 1U + 12U * (uint32_T)c4_i82;
  }

  for (c4_i83 = 0; c4_i83 < 2; c4_i83++) {
    c4_bv1[c4_i83] = c4_bv0[c4_i83];
  }

  sf_mex_import_vs(c4_parentId, sf_mex_dup(c4_u), c4_tmp_data, 1, 10, 0U, 1, 0U,
                   2, c4_bv1, c4_uv0, c4_tmp_sizes);
  c4_y_sizes[0] = 1;
  c4_y_sizes[1] = c4_tmp_sizes[1];
  c4_y = c4_y_sizes[0];
  c4_b_y = c4_y_sizes[1];
  c4_loop_ub = c4_tmp_sizes[0] * c4_tmp_sizes[1] - 1;
  for (c4_i84 = 0; c4_i84 <= c4_loop_ub; c4_i84++) {
    c4_y_data[c4_i84] = c4_tmp_data[c4_i84];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, char_T c4_outData_data[], int32_T
  c4_outData_sizes[2])
{
  const mxArray *c4_state;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y_sizes[2];
  char_T c4_y_data[13];
  int32_T c4_loop_ub;
  int32_T c4_i85;
  SFc4_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc4_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c4_state = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_state), &c4_thisId,
                        c4_y_data, c4_y_sizes);
  sf_mex_destroy(&c4_state);
  c4_outData_sizes[0] = 1;
  c4_outData_sizes[1] = c4_y_sizes[1];
  c4_loop_ub = c4_y_sizes[1] - 1;
  for (c4_i85 = 0; c4_i85 <= c4_loop_ub; c4_i85++) {
    c4_outData_data[c4_outData_sizes[0] * c4_i85] = c4_y_data[c4_y_sizes[0] *
      c4_i85];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_PackageDeliverySim_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  sf_mex_assign(&c4_nameCaptureInfo, sf_mex_createstruct("structure", 2, 26, 1),
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
  const mxArray *c4_rhs8 = NULL;
  const mxArray *c4_lhs8 = NULL;
  const mxArray *c4_rhs9 = NULL;
  const mxArray *c4_lhs9 = NULL;
  const mxArray *c4_rhs10 = NULL;
  const mxArray *c4_lhs10 = NULL;
  const mxArray *c4_rhs11 = NULL;
  const mxArray *c4_lhs11 = NULL;
  const mxArray *c4_rhs12 = NULL;
  const mxArray *c4_lhs12 = NULL;
  const mxArray *c4_rhs13 = NULL;
  const mxArray *c4_lhs13 = NULL;
  const mxArray *c4_rhs14 = NULL;
  const mxArray *c4_lhs14 = NULL;
  const mxArray *c4_rhs15 = NULL;
  const mxArray *c4_lhs15 = NULL;
  const mxArray *c4_rhs16 = NULL;
  const mxArray *c4_lhs16 = NULL;
  const mxArray *c4_rhs17 = NULL;
  const mxArray *c4_lhs17 = NULL;
  const mxArray *c4_rhs18 = NULL;
  const mxArray *c4_lhs18 = NULL;
  const mxArray *c4_rhs19 = NULL;
  const mxArray *c4_lhs19 = NULL;
  const mxArray *c4_rhs20 = NULL;
  const mxArray *c4_lhs20 = NULL;
  const mxArray *c4_rhs21 = NULL;
  const mxArray *c4_lhs21 = NULL;
  const mxArray *c4_rhs22 = NULL;
  const mxArray *c4_lhs22 = NULL;
  const mxArray *c4_rhs23 = NULL;
  const mxArray *c4_lhs23 = NULL;
  const mxArray *c4_rhs24 = NULL;
  const mxArray *c4_lhs24 = NULL;
  const mxArray *c4_rhs25 = NULL;
  const mxArray *c4_lhs25 = NULL;
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
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("mpower"), "name", "name", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742678U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c4_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c4_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("ismatrix"), "name", "name", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1331337258U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c4_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("power"), "name", "name", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1395357306U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c4_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c4_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c4_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c4_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c4_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c4_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("floor"), "name", "name", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c4_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c4_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c4_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c4_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("sqrt"), "name", "name", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1343862786U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c4_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_error"), "name", "name",
                  22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1343862758U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c4_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851138U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c4_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1393363258U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c4_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_strcmp"), "name", "name",
                  25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_strcmp.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1386456354U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c4_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs25), "lhs", "lhs",
                  25);
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
  sf_mex_destroy(&c4_rhs8);
  sf_mex_destroy(&c4_lhs8);
  sf_mex_destroy(&c4_rhs9);
  sf_mex_destroy(&c4_lhs9);
  sf_mex_destroy(&c4_rhs10);
  sf_mex_destroy(&c4_lhs10);
  sf_mex_destroy(&c4_rhs11);
  sf_mex_destroy(&c4_lhs11);
  sf_mex_destroy(&c4_rhs12);
  sf_mex_destroy(&c4_lhs12);
  sf_mex_destroy(&c4_rhs13);
  sf_mex_destroy(&c4_lhs13);
  sf_mex_destroy(&c4_rhs14);
  sf_mex_destroy(&c4_lhs14);
  sf_mex_destroy(&c4_rhs15);
  sf_mex_destroy(&c4_lhs15);
  sf_mex_destroy(&c4_rhs16);
  sf_mex_destroy(&c4_lhs16);
  sf_mex_destroy(&c4_rhs17);
  sf_mex_destroy(&c4_lhs17);
  sf_mex_destroy(&c4_rhs18);
  sf_mex_destroy(&c4_lhs18);
  sf_mex_destroy(&c4_rhs19);
  sf_mex_destroy(&c4_lhs19);
  sf_mex_destroy(&c4_rhs20);
  sf_mex_destroy(&c4_lhs20);
  sf_mex_destroy(&c4_rhs21);
  sf_mex_destroy(&c4_lhs21);
  sf_mex_destroy(&c4_rhs22);
  sf_mex_destroy(&c4_lhs22);
  sf_mex_destroy(&c4_rhs23);
  sf_mex_destroy(&c4_lhs23);
  sf_mex_destroy(&c4_rhs24);
  sf_mex_destroy(&c4_lhs24);
  sf_mex_destroy(&c4_rhs25);
  sf_mex_destroy(&c4_lhs25);
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

static real_T c4_mpower(SFc4_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c4_a)
{
  real_T c4_b_a;
  real_T c4_c_a;
  real_T c4_ak;
  real_T c4_d_a;
  c4_b_a = c4_a;
  c4_c_a = c4_b_a;
  c4_eml_scalar_eg(chartInstance);
  c4_ak = c4_c_a;
  c4_d_a = c4_ak;
  c4_eml_scalar_eg(chartInstance);
  return c4_d_a * c4_d_a;
}

static void c4_eml_scalar_eg(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_eml_error(SFc4_PackageDeliverySimInstanceStruct *chartInstance)
{
  int32_T c4_i86;
  static char_T c4_cv6[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c4_u[30];
  const mxArray *c4_y = NULL;
  int32_T c4_i87;
  static char_T c4_cv7[4] = { 's', 'q', 'r', 't' };

  char_T c4_b_u[4];
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  for (c4_i86 = 0; c4_i86 < 30; c4_i86++) {
    c4_u[c4_i86] = c4_cv6[c4_i86];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c4_i87 = 0; c4_i87 < 4; c4_i87++) {
    c4_b_u[c4_i87] = c4_cv7[c4_i87];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c4_y, 14, c4_b_y));
}

static real_T c4_eml_switch_helper(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, char_T c4_expr_data[], int32_T c4_expr_sizes[2])
{
  real_T c4_index;
  boolean_T c4_bool;
  int32_T c4_k;
  real_T c4_b_k;
  static char_T c4_cv8[9] = { 'a', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g' };

  boolean_T c4_b_bool;
  int32_T c4_c_k;
  real_T c4_d_k;
  static char_T c4_cv9[13] = { 't', 'r', 'a', 'n', 's', 'i', 't', '_', 't', 'h',
    'e', 'r', 'e' };

  boolean_T c4_c_bool;
  int32_T c4_e_k;
  real_T c4_f_k;
  static char_T c4_cv10[10] = { 'd', 'e', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g'
  };

  boolean_T c4_d_bool;
  int32_T c4_g_k;
  real_T c4_h_k;
  static char_T c4_cv11[10] = { 'a', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g', '2'
  };

  boolean_T c4_e_bool;
  int32_T c4_i_k;
  real_T c4_j_k;
  static char_T c4_cv12[12] = { 't', 'r', 'a', 'n', 's', 'i', 't', '_', 'b', 'a',
    'c', 'k' };

  boolean_T c4_f_bool;
  int32_T c4_k_k;
  real_T c4_l_k;
  static char_T c4_cv13[11] = { 'd', 'e', 's', 'c', 'e', 'n', 'd', 'i', 'n', 'g',
    '2' };

  int32_T exitg1;
  int32_T exitg2;
  int32_T exitg3;
  int32_T exitg4;
  int32_T exitg5;
  int32_T exitg6;
  (void)chartInstance;
  c4_bool = false;
  if ((real_T)c4_expr_sizes[1] != 9.0) {
  } else {
    c4_k = 0;
    do {
      exitg6 = 0;
      if (c4_k <= 8) {
        c4_b_k = 1.0 + (real_T)c4_k;
        if (c4_expr_data[(int32_T)c4_b_k - 1] != c4_cv8[(int32_T)c4_b_k - 1]) {
          exitg6 = 1;
        } else {
          c4_k++;
        }
      } else {
        c4_bool = true;
        exitg6 = 1;
      }
    } while (exitg6 == 0);
  }

  if (c4_bool) {
    c4_index = 0.0;
  } else {
    c4_b_bool = false;
    if ((real_T)c4_expr_sizes[1] != 13.0) {
    } else {
      c4_c_k = 0;
      do {
        exitg5 = 0;
        if (c4_c_k <= 12) {
          c4_d_k = 1.0 + (real_T)c4_c_k;
          if (c4_expr_data[(int32_T)c4_d_k - 1] != c4_cv9[(int32_T)c4_d_k - 1])
          {
            exitg5 = 1;
          } else {
            c4_c_k++;
          }
        } else {
          c4_b_bool = true;
          exitg5 = 1;
        }
      } while (exitg5 == 0);
    }

    if (c4_b_bool) {
      c4_index = 1.0;
    } else {
      c4_c_bool = false;
      if ((real_T)c4_expr_sizes[1] != 10.0) {
      } else {
        c4_e_k = 0;
        do {
          exitg4 = 0;
          if (c4_e_k <= 9) {
            c4_f_k = 1.0 + (real_T)c4_e_k;
            if (c4_expr_data[(int32_T)c4_f_k - 1] != c4_cv10[(int32_T)c4_f_k - 1])
            {
              exitg4 = 1;
            } else {
              c4_e_k++;
            }
          } else {
            c4_c_bool = true;
            exitg4 = 1;
          }
        } while (exitg4 == 0);
      }

      if (c4_c_bool) {
        c4_index = 2.0;
      } else {
        c4_d_bool = false;
        if ((real_T)c4_expr_sizes[1] != 10.0) {
        } else {
          c4_g_k = 0;
          do {
            exitg3 = 0;
            if (c4_g_k <= 9) {
              c4_h_k = 1.0 + (real_T)c4_g_k;
              if (c4_expr_data[(int32_T)c4_h_k - 1] != c4_cv11[(int32_T)c4_h_k -
                  1]) {
                exitg3 = 1;
              } else {
                c4_g_k++;
              }
            } else {
              c4_d_bool = true;
              exitg3 = 1;
            }
          } while (exitg3 == 0);
        }

        if (c4_d_bool) {
          c4_index = 3.0;
        } else {
          c4_e_bool = false;
          if ((real_T)c4_expr_sizes[1] != 12.0) {
          } else {
            c4_i_k = 0;
            do {
              exitg2 = 0;
              if (c4_i_k <= 11) {
                c4_j_k = 1.0 + (real_T)c4_i_k;
                if (c4_expr_data[(int32_T)c4_j_k - 1] != c4_cv12[(int32_T)c4_j_k
                    - 1]) {
                  exitg2 = 1;
                } else {
                  c4_i_k++;
                }
              } else {
                c4_e_bool = true;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }

          if (c4_e_bool) {
            c4_index = 4.0;
          } else {
            c4_f_bool = false;
            if ((real_T)c4_expr_sizes[1] != 11.0) {
            } else {
              c4_k_k = 0;
              do {
                exitg1 = 0;
                if (c4_k_k <= 10) {
                  c4_l_k = 1.0 + (real_T)c4_k_k;
                  if (c4_expr_data[(int32_T)c4_l_k - 1] != c4_cv13[(int32_T)
                      c4_l_k - 1]) {
                    exitg1 = 1;
                  } else {
                    c4_k_k++;
                  }
                } else {
                  c4_f_bool = true;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }

            if (c4_f_bool) {
              c4_index = 5.0;
            } else {
              c4_index = -1.0;
            }
          }
        }
      }
    }
  }

  return c4_index;
}

static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
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

static int32_T c4_i_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i88;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i88, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i88;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c4_y = c4_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_j_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_PackageDeliverySim, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_PackageDeliverySim), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_PackageDeliverySim);
  return c4_y;
}

static uint8_T c4_k_emlrt_marshallIn(SFc4_PackageDeliverySimInstanceStruct
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

static real_T c4_get_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 0, NULL, c4_elementIndex);
  return *chartInstance->c4_clock_offset_address;
}

static void c4_set_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex, real_T c4_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 0, NULL, c4_elementIndex);
  *chartInstance->c4_clock_offset_address = c4_elementValue;
}

static real_T *c4_access_clock_offset(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_rdOnly)
{
  real_T *c4_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 0, NULL);
  c4_dsmAddr = chartInstance->c4_clock_offset_address;
  if (c4_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 0, NULL);
  }

  return c4_dsmAddr;
}

static real_T c4_get_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 1, NULL, c4_elementIndex);
  return *chartInstance->c4_num_delivered_address;
}

static void c4_set_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_elementIndex, real_T c4_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 1, NULL, c4_elementIndex);
  *chartInstance->c4_num_delivered_address = c4_elementValue;
}

static real_T *c4_access_num_delivered(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c4_rdOnly)
{
  real_T *c4_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 1, NULL);
  c4_dsmAddr = chartInstance->c4_num_delivered_address;
  if (c4_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 1, NULL);
  }

  return c4_dsmAddr;
}

static void init_dsm_address_info(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "clock_offset", (void **)
    &chartInstance->c4_clock_offset_address,
    &chartInstance->c4_clock_offset_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "num_delivered", (void
    **)&chartInstance->c4_num_delivered_address,
    &chartInstance->c4_num_delivered_index);
}

static void init_simulink_io_address(SFc4_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  chartInstance->c4_clock = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c4_trajectory = (real_T (*)[12])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_origin = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_destinations = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c4_altitude = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c4_p = (real_T (*)[15])ssGetOutputPortSignal_wrapper
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
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1691117833U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1917631670U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2612981362U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2316016877U);
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
    mxArray *mxChecksum = mxCreateString("3gpruUj3IoKjVcC4hkEjqG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
      pr[0] = (double)(3);
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
      pr[0] = (double)(3);
      pr[1] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(15);
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
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[13],T\"p\",},{M[1],M[5],T\"trajectory\",},{M[8],M[0],T\"is_active_c4_PackageDeliverySim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
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
           8,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"clock");
          _SFD_SET_DATA_PROPS(1,2,0,1,"trajectory");
          _SFD_SET_DATA_PROPS(2,1,1,0,"origin");
          _SFD_SET_DATA_PROPS(3,1,1,0,"destinations");
          _SFD_SET_DATA_PROPS(4,1,1,0,"altitude");
          _SFD_SET_DATA_PROPS(5,11,0,0,"num_delivered");
          _SFD_SET_DATA_PROPS(6,11,0,0,"clock_offset");
          _SFD_SET_DATA_PROPS(7,2,0,1,"p");
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
        _SFD_CV_INIT_EML(0,1,1,9,0,0,1,0,0,14,7);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2221);
        _SFD_CV_INIT_EML_IF(0,1,0,385,420,-1,473);
        _SFD_CV_INIT_EML_IF(0,1,1,902,942,-1,1023);
        _SFD_CV_INIT_EML_IF(0,1,2,1043,1101,1127,1200);
        _SFD_CV_INIT_EML_IF(0,1,3,1127,1200,1230,1648);
        _SFD_CV_INIT_EML_IF(0,1,4,1230,1305,1332,1648);
        _SFD_CV_INIT_EML_IF(0,1,5,1332,1407,1434,1648);
        _SFD_CV_INIT_EML_IF(0,1,6,1434,1511,1540,1648);
        _SFD_CV_INIT_EML_IF(0,1,7,1540,1617,-1,1617);
        _SFD_CV_INIT_EML_IF(0,1,8,2102,2138,-1,2178);

        {
          static int caseStart[] = { -1, 1710, 1778, 1846, 1903, 1968, 2039 };

          static int caseExprEnd[] = { 8, 1726, 1798, 1863, 1920, 1987, 2057 };

          _SFD_CV_INIT_EML_SWITCH(0,1,0,1693,1706,2100,7,&(caseStart[0]),
            &(caseExprEnd[0]));
        }

        {
          static int condStart[] = { 905, 923 };

          static int condEnd[] = { 919, 941 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,905,941,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1050, 1087 };

          static int condEnd[] = { 1056, 1100 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,1050,1100,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1134, 1171 };

          static int condEnd[] = { 1148, 1199 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,1134,1199,2,4,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1237, 1274 };

          static int condEnd[] = { 1266, 1304 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,3,1237,1304,2,6,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1339, 1376 };

          static int condEnd[] = { 1370, 1406 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,4,1339,1406,2,8,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1441, 1478 };

          static int condEnd[] = { 1472, 1510 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,5,1441,1510,2,10,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1547, 1584 };

          static int condEnd[] = { 1580, 1616 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,6,1547,1616,2,12,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,388,419,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,905,919,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,923,941,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,1050,1056,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,4,1087,1100,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,5,1134,1148,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,6,1171,1199,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,7,1237,1266,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,8,1274,1304,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,9,1339,1370,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,10,1376,1406,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,11,1441,1472,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,12,1478,1510,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,13,1547,1580,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,14,1584,1616,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,15,2105,2137,-1,0);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)
            c4_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)c4_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)c4_c_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)
            c4_sf_marshallIn);
        }

        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c4_clock);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c4_trajectory);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c4_origin);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c4_destinations);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c4_altitude);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c4_num_delivered_address);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c4_clock_offset_address);
        _SFD_SET_DATA_VALUE_PTR(7U, *chartInstance->c4_p);
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
  return "d9QE5OuX4YINh34qm6EJgE";
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
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4280391310U));
  ssSetChecksum1(S,(1571055890U));
  ssSetChecksum2(S,(1547872844U));
  ssSetChecksum3(S,(167370184U));
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
