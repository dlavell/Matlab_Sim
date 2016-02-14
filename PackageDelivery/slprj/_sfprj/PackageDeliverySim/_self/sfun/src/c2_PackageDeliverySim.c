/* Include files */

#include <stddef.h>
#include "blas.h"
#include "PackageDeliverySim_sfun.h"
#include "c2_PackageDeliverySim.h"
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
static const char * c2_debug_family_names[94] = { "P", "p", "R", "v", "r", "I",
  "pd_", "Dpd_", "D2pd_", "D3pd_", "D4pd_", "pd", "vd", "dot_vd", "ddot_vd",
  "d3dot_vd", "p0", "v0", "u", "u0", "e3", "mu", "r3", "T", "gm", "R0", "bar_V0",
  "grad_bar_V0", "ei", "s", "H1", "H2", "Dr3_R0", "q1", "w1", "hat_dot_v",
  "hat_dot_mu", "hat_dot_r3", "hat_w0", "w_star", "hat_dot_v2", "hat_dot_mu2",
  "hat_dot_r32", "hat_w02", "hat_dot_hat_dot_v2", "hat_dot_u", "hat_dot_u2",
  "hat_dot_hat_dot_u2", "hess_bar_V0", "hat_ddot_z2", "hat_dot_hat_dot_mu2",
  "hat_dot_hat_dot_r32", "Dr3_H1", "Dr3_Y", "Dr3_H2", "H", "Dr3_H", "Dr3_X",
  "D2r3_R0", "hat_dot_hat_w0", "hat_dot_w1", "hat_dot_w_star", "u2", "thrust",
  "aileron", "elevator", "yaw", "nargin", "nargout", "state", "ref", "z",
  "hat_b", "hat_b2", "w", "h", "qh", "kp", "kv", "kz", "kV0", "kq", "kw", "cmd",
  "dot_z", "dot_hat_b", "dot_hat_b2", "dot_w", "dot_h", "dot_qh", "V2", "dot_V2",
  "q0", "q" };

static const char * c2_b_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_c_debug_family_names[4] = { "nargin", "nargout", "v",
  "out" };

static const char * c2_d_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_e_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_f_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_g_debug_family_names[4] = { "nargin", "nargout", "A",
  "out" };

static const char * c2_h_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_i_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

static const char * c2_j_debug_family_names[5] = { "I", "nargin", "nargout",
  "var", "out" };

/* Function Declarations */
static void initialize_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void initialize_params_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void enable_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void disable_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct *
  chartInstance);
static void c2_update_debugger_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void set_sim_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void sf_gateway_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void mdl_start_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void c2_chartstep_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void initSimStructsc2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_dot_V2, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_dot_w, const char_T *c2_identifier, real_T
  c2_y[3]);
static void c2_d_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[8]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_j_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[1000]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_k_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_l_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u);
static void c2_b_info_helper(const mxArray **c2_info);
static void c2_c_info_helper(const mxArray **c2_info);
static void c2_eml_switch_helper(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_threshold(SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void c2_eml_extremum(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_x[2], real_T *c2_extremum, int32_T *c2_indx);
static void c2_check_forloop_overflow_error
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance, boolean_T c2_overflow);
static void c2_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var[3], real_T c2_out[3]);
static real_T c2_norm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_x[3]);
static void c2_below_threshold(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_S(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                 c2_v[3], real_T c2_out[9]);
static void c2_eye(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                   c2_I[9]);
static void c2_c_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_b_threshold(SFc2_PackageDeliverySimInstanceStruct *chartInstance);
static void c2_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_a[9], real_T c2_c[9]);
static void c2_matrix_to_integer_power(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_a[9], real_T c2_c[9]);
static void c2_d_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_linspace(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_d1, real_T c2_d2, real_T c2_y[1000]);
static void c2_e_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static real_T c2_b_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var);
static void c2_f_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_c_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var[1000], real_T c2_out[1000]);
static real_T c2_trapz(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_x[1000], real_T c2_y[1000]);
static real_T c2_sat_atan_dot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var);
static real_T c2_b_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a);
static real_T c2_c_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_g_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_h_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_i_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                    c2_A[3], real_T c2_B[9], real_T c2_K[27]);
static void c2_b_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[3], real_T c2_K[27]);
static void c2_c_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[9], real_T c2_K[81]);
static void c2_Gamma(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     real_T c2_A[9], real_T c2_out[27]);
static void c2_j_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_k_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_l_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_m_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_b_sat_atan_dot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var[3], real_T c2_out[3]);
static void c2_n_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_o_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_d_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[2], real_T c2_B[36], real_T c2_K[72]);
static real_T c2_sat_atan_ddot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var);
static real_T c2_d_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_p_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_q_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_r_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_s_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_t_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_u_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_b_sat_atan_ddot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var[3], real_T c2_out[3]);
static void c2_power(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_y[3]);
static void c2_e_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[9], real_T c2_K[27]);
static void c2_v_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static real_T c2_e_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_w_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[729], real_T c2_B[81], real_T c2_C[81], real_T c2_b_C[81]);
static void c2_f_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[81], real_T c2_K[729]);
static void c2_x_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_g_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[81], real_T c2_K[243]);
static void c2_y_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_b_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[243], real_T c2_b_C[243]);
static void c2_ab_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_c_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[27], real_T c2_C[81], real_T c2_b_C[81]);
static void c2_bb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_d_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_B[81], real_T c2_C[81], real_T c2_b_C[81]);
static void c2_cb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_h_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[27], real_T c2_K[243]);
static void c2_i_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[27], real_T c2_B[81], real_T c2_K[2187]);
static void c2_db_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_e_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[2187], real_T c2_B[81], real_T c2_C[27], real_T c2_b_C[27]);
static void c2_j_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[81], real_T c2_K[243]);
static void c2_eb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_f_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[27], real_T c2_b_C[27]);
static void c2_fb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void c2_m_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_transpose, const char_T *c2_identifier,
  real_T c2_y[4]);
static void c2_n_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_o_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_p_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_PackageDeliverySim, const
  char_T *c2_identifier);
static uint8_T c2_q_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[729], real_T c2_B[81], real_T c2_C[81]);
static void c2_h_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[243]);
static void c2_i_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[27], real_T c2_C[81]);
static void c2_j_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_B[81], real_T c2_C[81]);
static void c2_k_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[2187], real_T c2_B[81], real_T c2_C[27]);
static void c2_l_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[27]);
static real_T c2_get_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly);
static real_T c2_get_satMax(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex);
static void c2_set_satMax(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex, real_T c2_elementValue);
static real_T *c2_access_satMax(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c2_rdOnly);
static void init_dsm_address_info(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_PackageDeliverySim = 0U;
}

static void initialize_params_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct *
  chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i0;
  real_T c2_b_u[4];
  const mxArray *c2_c_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_i1;
  real_T c2_e_u[3];
  const mxArray *c2_f_y = NULL;
  int32_T c2_i2;
  real_T c2_f_u[3];
  const mxArray *c2_g_y = NULL;
  int32_T c2_i3;
  real_T c2_g_u[4];
  const mxArray *c2_h_y = NULL;
  int32_T c2_i4;
  real_T c2_h_u[3];
  const mxArray *c2_i_y = NULL;
  int32_T c2_i5;
  real_T c2_i_u[3];
  const mxArray *c2_j_y = NULL;
  int32_T c2_i6;
  real_T c2_j_u[4];
  const mxArray *c2_k_y = NULL;
  int32_T c2_i7;
  real_T c2_k_u[4];
  const mxArray *c2_l_y = NULL;
  uint8_T c2_d_hoistedGlobal;
  uint8_T c2_l_u;
  const mxArray *c2_m_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(12, 1), false);
  c2_hoistedGlobal = *chartInstance->c2_V2;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    c2_b_u[c2_i0] = (*chartInstance->c2_cmd)[c2_i0];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_b_hoistedGlobal = *chartInstance->c2_dot_V2;
  c2_c_u = c2_b_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_c_hoistedGlobal = *chartInstance->c2_dot_h;
  c2_d_u = c2_c_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_e_u[c2_i1] = (*chartInstance->c2_dot_hat_b)[c2_i1];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_e_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    c2_f_u[c2_i2] = (*chartInstance->c2_dot_hat_b2)[c2_i2];
  }

  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_f_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 5, c2_g_y);
  for (c2_i3 = 0; c2_i3 < 4; c2_i3++) {
    c2_g_u[c2_i3] = (*chartInstance->c2_dot_qh)[c2_i3];
  }

  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_g_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_h_u[c2_i4] = (*chartInstance->c2_dot_w)[c2_i4];
  }

  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_h_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 7, c2_i_y);
  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    c2_i_u[c2_i5] = (*chartInstance->c2_dot_z)[c2_i5];
  }

  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", c2_i_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 8, c2_j_y);
  for (c2_i6 = 0; c2_i6 < 4; c2_i6++) {
    c2_j_u[c2_i6] = (*chartInstance->c2_q)[c2_i6];
  }

  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", c2_j_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 9, c2_k_y);
  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    c2_k_u[c2_i7] = (*chartInstance->c2_q0)[c2_i7];
  }

  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", c2_k_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 10, c2_l_y);
  c2_d_hoistedGlobal = chartInstance->c2_is_active_c2_PackageDeliverySim;
  c2_l_u = c2_d_hoistedGlobal;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_l_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 11, c2_m_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i8;
  real_T c2_dv1[3];
  int32_T c2_i9;
  real_T c2_dv2[3];
  int32_T c2_i10;
  real_T c2_dv3[4];
  int32_T c2_i11;
  real_T c2_dv4[3];
  int32_T c2_i12;
  real_T c2_dv5[3];
  int32_T c2_i13;
  real_T c2_dv6[4];
  int32_T c2_i14;
  real_T c2_dv7[4];
  int32_T c2_i15;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_V2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 0)), "V2");
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "cmd", c2_dv0);
  for (c2_i8 = 0; c2_i8 < 4; c2_i8++) {
    (*chartInstance->c2_cmd)[c2_i8] = c2_dv0[c2_i8];
  }

  *chartInstance->c2_dot_V2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 2)), "dot_V2");
  *chartInstance->c2_dot_h = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 3)), "dot_h");
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
                        "dot_hat_b", c2_dv1);
  for (c2_i9 = 0; c2_i9 < 3; c2_i9++) {
    (*chartInstance->c2_dot_hat_b)[c2_i9] = c2_dv1[c2_i9];
  }

  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 5)),
                        "dot_hat_b2", c2_dv2);
  for (c2_i10 = 0; c2_i10 < 3; c2_i10++) {
    (*chartInstance->c2_dot_hat_b2)[c2_i10] = c2_dv2[c2_i10];
  }

  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 6)),
                        "dot_qh", c2_dv3);
  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    (*chartInstance->c2_dot_qh)[c2_i11] = c2_dv3[c2_i11];
  }

  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 7)),
                        "dot_w", c2_dv4);
  for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
    (*chartInstance->c2_dot_w)[c2_i12] = c2_dv4[c2_i12];
  }

  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 8)),
                        "dot_z", c2_dv5);
  for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
    (*chartInstance->c2_dot_z)[c2_i13] = c2_dv5[c2_i13];
  }

  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)), "q",
                        c2_dv6);
  for (c2_i14 = 0; c2_i14 < 4; c2_i14++) {
    (*chartInstance->c2_q)[c2_i14] = c2_dv6[c2_i14];
  }

  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 10)),
                        "q0", c2_dv7);
  for (c2_i15 = 0; c2_i15 < 4; c2_i15++) {
    (*chartInstance->c2_q0)[c2_i15] = c2_dv7[c2_i15];
  }

  chartInstance->c2_is_active_c2_PackageDeliverySim = c2_p_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 11)),
     "is_active_c2_PackageDeliverySim");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_PackageDeliverySim(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_PackageDeliverySim(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  int32_T c2_i16;
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i16 = 0; c2_i16 < 18; c2_i16++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_state)[c2_i16], 0U);
  }

  for (c2_i17 = 0; c2_i17 < 15; c2_i17++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_ref)[c2_i17], 1U);
  }

  for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_z)[c2_i18], 2U);
  }

  for (c2_i19 = 0; c2_i19 < 3; c2_i19++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_hat_b)[c2_i19], 3U);
  }

  for (c2_i20 = 0; c2_i20 < 3; c2_i20++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_hat_b2)[c2_i20], 4U);
  }

  for (c2_i21 = 0; c2_i21 < 3; c2_i21++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_w)[c2_i21], 5U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_h, 6U);
  for (c2_i22 = 0; c2_i22 < 4; c2_i22++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_qh)[c2_i22], 7U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_PackageDeliverySim(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_PackageDeliverySimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i23 = 0; c2_i23 < 4; c2_i23++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_cmd)[c2_i23], 8U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kp, 11U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kv, 12U);
  for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_dot_z)[c2_i24], 14U);
  }

  for (c2_i25 = 0; c2_i25 < 3; c2_i25++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_dot_hat_b)[c2_i25], 15U);
  }

  for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_dot_hat_b2)[c2_i26], 16U);
  }

  for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_dot_w)[c2_i27], 17U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_dot_h, 18U);
  for (c2_i28 = 0; c2_i28 < 4; c2_i28++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_dot_qh)[c2_i28], 19U);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kz, 22U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kV0, 25U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_V2, 26U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_dot_V2, 27U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kq, 28U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_kw, 29U);
  for (c2_i29 = 0; c2_i29 < 4; c2_i29++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_q0)[c2_i29], 30U);
  }

  for (c2_i30 = 0; c2_i30 < 4; c2_i30++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_q)[c2_i30], 31U);
  }
}

static void mdl_start_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_chartstep_c2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_f_hoistedGlobal;
  real_T c2_g_hoistedGlobal;
  int32_T c2_i31;
  real_T c2_b_state[18];
  int32_T c2_i32;
  real_T c2_b_ref[15];
  int32_T c2_i33;
  real_T c2_b_z[3];
  int32_T c2_i34;
  real_T c2_b_hat_b[3];
  int32_T c2_i35;
  real_T c2_b_hat_b2[3];
  int32_T c2_i36;
  real_T c2_b_w[3];
  real_T c2_b_h;
  int32_T c2_i37;
  real_T c2_b_qh[4];
  real_T c2_b_kp;
  real_T c2_b_kv;
  real_T c2_b_kz;
  real_T c2_b_kV0;
  real_T c2_b_kq;
  real_T c2_b_kw;
  uint32_T c2_debug_family_var_map[94];
  real_T c2_P[4];
  real_T c2_p[3];
  real_T c2_R[9];
  real_T c2_v[3];
  real_T c2_r[4];
  real_T c2_I;
  real_T c2_pd_[3];
  real_T c2_Dpd_[3];
  real_T c2_D2pd_[3];
  real_T c2_D3pd_[3];
  real_T c2_D4pd_[3];
  real_T c2_pd[3];
  real_T c2_vd[3];
  real_T c2_dot_vd[3];
  real_T c2_ddot_vd[3];
  real_T c2_d3dot_vd[3];
  real_T c2_p0[3];
  real_T c2_v0[3];
  real_T c2_u[3];
  real_T c2_u0[3];
  real_T c2_e3[3];
  real_T c2_mu[3];
  real_T c2_r3[3];
  real_T c2_T;
  real_T c2_gm[3];
  real_T c2_R0[9];
  real_T c2_bar_V0;
  real_T c2_grad_bar_V0[6];
  real_T c2_ei[3];
  real_T c2_s[1000];
  real_T c2_H1[27];
  real_T c2_H2[9];
  real_T c2_Dr3_R0[27];
  real_T c2_q1[4];
  real_T c2_w1[3];
  real_T c2_hat_dot_v[3];
  real_T c2_hat_dot_mu[3];
  real_T c2_hat_dot_r3[3];
  real_T c2_hat_w0[3];
  real_T c2_w_star[3];
  real_T c2_hat_dot_v2[3];
  real_T c2_hat_dot_mu2[3];
  real_T c2_hat_dot_r32[3];
  real_T c2_hat_w02[3];
  real_T c2_hat_dot_hat_dot_v2[3];
  real_T c2_hat_dot_u[3];
  real_T c2_hat_dot_u2[3];
  real_T c2_hat_dot_hat_dot_u2[3];
  real_T c2_hess_bar_V0[36];
  real_T c2_hat_ddot_z2[3];
  real_T c2_hat_dot_hat_dot_mu2[3];
  real_T c2_hat_dot_hat_dot_r32[3];
  real_T c2_Dr3_H1[81];
  real_T c2_Dr3_Y[81];
  real_T c2_Dr3_H2[27];
  real_T c2_H[27];
  real_T c2_Dr3_H[81];
  real_T c2_Dr3_X[81];
  real_T c2_D2r3_R0[81];
  real_T c2_hat_dot_hat_w0[3];
  real_T c2_hat_dot_w1[3];
  real_T c2_hat_dot_w_star[3];
  real_T c2_u2[3];
  real_T c2_thrust;
  real_T c2_aileron;
  real_T c2_elevator;
  real_T c2_yaw;
  real_T c2_b_r[8];
  real_T c2_nargin = 14.0;
  real_T c2_nargout = 11.0;
  real_T c2_b_cmd[4];
  real_T c2_b_dot_z[3];
  real_T c2_b_dot_hat_b[3];
  real_T c2_b_dot_hat_b2[3];
  real_T c2_b_dot_w[3];
  real_T c2_b_dot_h;
  real_T c2_b_dot_qh[4];
  real_T c2_b_V2;
  real_T c2_b_dot_V2;
  real_T c2_b_q0[4];
  real_T c2_b_q[4];
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_e_y;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_h_y;
  int32_T c2_i38;
  int32_T c2_i39;
  real_T c2_g_x[9];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_i40;
  int32_T c2_i41;
  int32_T c2_i42;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  real_T c2_b_u[9];
  const mxArray *c2_i_y = NULL;
  real_T c2_dv8[4];
  int32_T c2_i46;
  int32_T c2_i47;
  real_T c2_c_r[4];
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  real_T c2_a[8];
  int32_T c2_i54;
  int32_T c2_i55;
  real_T c2_j_y[2];
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  real_T c2_k_y[2];
  int32_T c2_iindx;
  real_T c2_extremum;
  int32_T c2_b_iindx;
  real_T c2_indx;
  real_T c2_b_I;
  int32_T c2_c_I;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  real_T c2_b_a;
  int32_T c2_i73;
  real_T c2_b[3];
  int32_T c2_i74;
  real_T c2_c_a;
  int32_T c2_i75;
  real_T c2_b_b[3];
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  real_T c2_c_u[3];
  real_T c2_dv9[3];
  int32_T c2_i79;
  int32_T c2_i80;
  static real_T c2_c_b[3] = { 0.0, 0.0, 1.0 };

  real_T c2_h_hoistedGlobal;
  real_T c2_d_a;
  int32_T c2_i81;
  real_T c2_l_y[3];
  int32_T c2_i82;
  real_T c2_c_z[3];
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  real_T c2_b_mu[3];
  real_T c2_c_B;
  real_T c2_m_y;
  real_T c2_n_y;
  real_T c2_o_y;
  int32_T c2_i86;
  int32_T c2_i87;
  real_T c2_c_mu[3];
  int32_T c2_i88;
  int32_T c2_i89;
  int32_T c2_i90;
  int32_T c2_i91;
  real_T c2_p_y[3];
  real_T c2_e_a[9];
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  real_T c2_C[3];
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  real_T c2_d_b[9];
  int32_T c2_i106;
  int32_T c2_i107;
  real_T c2_q_y[3];
  int32_T c2_i108;
  static real_T c2_f_a[3] = { 0.0, 0.0, 1.0 };

  int32_T c2_i109;
  real_T c2_r_y;
  int32_T c2_c_k;
  int32_T c2_d_k;
  int32_T c2_i110;
  int32_T c2_i111;
  real_T c2_b_gm[3];
  real_T c2_d_B;
  real_T c2_s_y;
  real_T c2_t_y;
  real_T c2_u_y;
  int32_T c2_i112;
  real_T c2_g_a;
  int32_T c2_i113;
  real_T c2_e_b[3];
  real_T c2_dv10[9];
  int32_T c2_i114;
  real_T c2_dv11[9];
  int32_T c2_i115;
  int32_T c2_i116;
  real_T c2_i_hoistedGlobal[9];
  int32_T c2_i117;
  real_T c2_c_gm[3];
  int32_T c2_i118;
  int32_T c2_i119;
  int32_T c2_i120;
  int32_T c2_i121;
  real_T c2_b_C[9];
  int32_T c2_i122;
  int32_T c2_i123;
  int32_T c2_i124;
  int32_T c2_i125;
  int32_T c2_i126;
  int32_T c2_i127;
  int32_T c2_i128;
  int32_T c2_i129;
  int32_T c2_i130;
  int32_T c2_i131;
  int32_T c2_i132;
  int32_T c2_i133;
  int32_T c2_i134;
  real_T c2_d_u[9];
  const mxArray *c2_v_y = NULL;
  real_T c2_dv12[4];
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_d_I;
  int32_T c2_i137;
  int32_T c2_i138;
  int32_T c2_i139;
  real_T c2_w_y;
  int32_T c2_e_k;
  int32_T c2_f_k;
  real_T c2_dv13[1000];
  int32_T c2_i140;
  int32_T c2_i141;
  int32_T c2_i142;
  real_T c2_x_y;
  int32_T c2_g_k;
  int32_T c2_h_k;
  int32_T c2_i143;
  int32_T c2_i144;
  real_T c2_y_y;
  int32_T c2_i_k;
  int32_T c2_j_k;
  real_T c2_d0;
  real_T c2_h_a[2];
  int32_T c2_i145;
  int32_T c2_i146;
  real_T c2_f_b[4];
  int32_T c2_i147;
  int32_T c2_i148;
  real_T c2_ab_y[2];
  int32_T c2_i149;
  int32_T c2_i150;
  int32_T c2_i151;
  real_T c2_bb_y;
  int32_T c2_k_k;
  int32_T c2_l_k;
  int32_T c2_i152;
  int32_T c2_i153;
  real_T c2_cb_y;
  int32_T c2_m_k;
  int32_T c2_n_k;
  real_T c2_d1;
  real_T c2_db_y;
  int32_T c2_o_k;
  int32_T c2_p_k;
  int32_T c2_i154;
  real_T c2_b_s[1000];
  real_T c2_dv14[1000];
  int32_T c2_i155;
  real_T c2_c_s[1000];
  int32_T c2_i156;
  real_T c2_dv15[1000];
  int32_T c2_i157;
  int32_T c2_i158;
  real_T c2_eb_y;
  int32_T c2_q_k;
  int32_T c2_r_k;
  real_T c2_i_a;
  int32_T c2_i159;
  int32_T c2_i160;
  int32_T c2_i161;
  int32_T c2_i162;
  int32_T c2_i163;
  real_T c2_fb_y;
  int32_T c2_s_k;
  int32_T c2_t_k;
  real_T c2_j_a;
  int32_T c2_i164;
  int32_T c2_i165;
  int32_T c2_i166;
  real_T c2_k_a[12];
  int32_T c2_i167;
  int32_T c2_i168;
  int32_T c2_i169;
  int32_T c2_i170;
  int32_T c2_i171;
  int32_T c2_i172;
  int32_T c2_i173;
  int32_T c2_i174;
  real_T c2_gb_y[12];
  int32_T c2_i175;
  int32_T c2_i176;
  int32_T c2_i177;
  int32_T c2_i178;
  real_T c2_hb_y;
  int32_T c2_u_k;
  int32_T c2_v_k;
  int32_T c2_i179;
  int32_T c2_i180;
  real_T c2_ib_y;
  int32_T c2_w_k;
  int32_T c2_x_k;
  real_T c2_d2;
  int32_T c2_i181;
  real_T c2_g_b[6];
  int32_T c2_i182;
  int32_T c2_i183;
  int32_T c2_i184;
  int32_T c2_i185;
  real_T c2_jb_y;
  int32_T c2_y_k;
  int32_T c2_ab_k;
  real_T c2_l_a;
  int32_T c2_i186;
  int32_T c2_i187;
  int32_T c2_i188;
  int32_T c2_i189;
  real_T c2_kb_y;
  int32_T c2_bb_k;
  int32_T c2_cb_k;
  real_T c2_m_a;
  int32_T c2_i190;
  int32_T c2_i191;
  int32_T c2_i192;
  real_T c2_h_b[6];
  int32_T c2_i193;
  int32_T c2_i194;
  real_T c2_n_a;
  int32_T c2_i195;
  static real_T c2_i_b[18] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_lb_y[18];
  int32_T c2_i196;
  int32_T c2_i197;
  int32_T c2_i198;
  int32_T c2_i199;
  int32_T c2_i200;
  int32_T c2_i201;
  int32_T c2_i202;
  int32_T c2_i203;
  int32_T c2_i204;
  int32_T c2_i205;
  int32_T c2_i206;
  int32_T c2_i207;
  real_T c2_d_gm[3];
  real_T c2_e_B;
  real_T c2_mb_y;
  real_T c2_nb_y;
  real_T c2_ob_y;
  int32_T c2_i208;
  int32_T c2_i209;
  int32_T c2_i210;
  real_T c2_e_gm[3];
  real_T c2_f_B;
  real_T c2_pb_y;
  real_T c2_qb_y;
  real_T c2_rb_y;
  int32_T c2_i211;
  int32_T c2_i212;
  real_T c2_j_b[3];
  int32_T c2_i213;
  static real_T c2_k_b[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_l_b[9];
  real_T c2_sb_y[27];
  int32_T c2_i214;
  real_T c2_m_b[9];
  int32_T c2_i215;
  real_T c2_c_C[3];
  real_T c2_tb_y[27];
  int32_T c2_i216;
  int32_T c2_i217;
  int32_T c2_i218;
  real_T c2_f_gm[3];
  real_T c2_g_B;
  real_T c2_ub_y;
  real_T c2_vb_y;
  real_T c2_wb_y;
  int32_T c2_i219;
  int32_T c2_i220;
  int32_T c2_i221;
  int32_T c2_i222;
  int32_T c2_i223;
  real_T c2_n_b[3];
  int32_T c2_i224;
  real_T c2_dv16[9];
  int32_T c2_i225;
  real_T c2_xb_y[3];
  int32_T c2_i226;
  int32_T c2_i227;
  int32_T c2_i228;
  real_T c2_yb_y[9];
  int32_T c2_i229;
  int32_T c2_i230;
  int32_T c2_i231;
  real_T c2_g_gm[3];
  real_T c2_h_B;
  real_T c2_ac_y;
  real_T c2_bc_y;
  real_T c2_cc_y;
  int32_T c2_i232;
  int32_T c2_i233;
  int32_T c2_i234;
  int32_T c2_i235;
  int32_T c2_i236;
  int32_T c2_i237;
  real_T c2_dc_y[3];
  int32_T c2_i238;
  int32_T c2_i239;
  int32_T c2_i240;
  int32_T c2_i241;
  real_T c2_ec_y[27];
  int32_T c2_i242;
  int32_T c2_i243;
  static real_T c2_o_a[27] = { 0.0, -0.0, 0.0, 0.0, -0.0, 1.0, 0.0, -1.0, 0.0,
    0.0, 0.0, -1.0, 0.0, 0.0, -0.0, 1.0, 0.0, -0.0, -0.0, 1.0, 0.0, -1.0, 0.0,
    0.0, -0.0, 0.0, 0.0 };

  int32_T c2_i244;
  int32_T c2_i245;
  real_T c2_h_gm[3];
  real_T c2_i_B;
  real_T c2_fc_y;
  real_T c2_gc_y;
  real_T c2_hc_y;
  int32_T c2_i246;
  int32_T c2_i247;
  real_T c2_o_b[3];
  int32_T c2_i248;
  real_T c2_dv17[9];
  int32_T c2_i249;
  real_T c2_d_C[9];
  int32_T c2_i250;
  real_T c2_p_b[9];
  real_T c2_p_a[81];
  int32_T c2_i251;
  int32_T c2_i252;
  int32_T c2_i253;
  static real_T c2_q_b[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c2_i254;
  int32_T c2_i255;
  int32_T c2_i256;
  int32_T c2_i257;
  int32_T c2_i258;
  int32_T c2_i259;
  int32_T c2_i260;
  int32_T c2_i261;
  int32_T c2_i262;
  int32_T c2_i263;
  int32_T c2_i264;
  int32_T c2_i265;
  int32_T c2_i266;
  int32_T c2_i267;
  int32_T c2_i268;
  int32_T c2_i269;
  int32_T c2_i270;
  int32_T c2_i271;
  int32_T c2_i272;
  int32_T c2_i273;
  int32_T c2_i274;
  real_T c2_ic_y;
  int32_T c2_db_k;
  int32_T c2_eb_k;
  real_T c2_q_a;
  int32_T c2_i275;
  real_T c2_r_b[27];
  int32_T c2_i276;
  int32_T c2_i277;
  int32_T c2_i278;
  int32_T c2_i279;
  int32_T c2_i280;
  int32_T c2_i281;
  int32_T c2_i282;
  int32_T c2_i283;
  int32_T c2_i284;
  int32_T c2_i285;
  int32_T c2_i286;
  int32_T c2_i287;
  real_T c2_j_hoistedGlobal[9];
  int32_T c2_i288;
  real_T c2_s_b[9];
  int32_T c2_i289;
  int32_T c2_i290;
  int32_T c2_i291;
  int32_T c2_i292;
  int32_T c2_i293;
  int32_T c2_i294;
  int32_T c2_i295;
  int32_T c2_i296;
  int32_T c2_i297;
  int32_T c2_i298;
  int32_T c2_i299;
  int32_T c2_i300;
  int32_T c2_i301;
  int32_T c2_i302;
  real_T c2_e_u[4];
  const mxArray *c2_jc_y = NULL;
  int32_T c2_i303;
  real_T c2_f_u[4];
  const mxArray *c2_kc_y = NULL;
  real_T c2_dv18[4];
  int32_T c2_i304;
  real_T c2_k_hoistedGlobal;
  real_T c2_c_A;
  real_T c2_j_B;
  real_T c2_h_x;
  real_T c2_lc_y;
  real_T c2_i_x;
  real_T c2_mc_y;
  real_T c2_j_x;
  real_T c2_nc_y;
  real_T c2_oc_y;
  real_T c2_r_a;
  int32_T c2_i305;
  real_T c2_d_mu[3];
  int32_T c2_i306;
  int32_T c2_i307;
  real_T c2_e_mu[3];
  int32_T c2_i308;
  int32_T c2_i309;
  real_T c2_b_q1[3];
  int32_T c2_i310;
  int32_T c2_i311;
  int32_T c2_i312;
  int32_T c2_i313;
  int32_T c2_i314;
  real_T c2_s_a;
  int32_T c2_i315;
  int32_T c2_i316;
  int32_T c2_i317;
  int32_T c2_i318;
  int32_T c2_i319;
  int32_T c2_i320;
  int32_T c2_i321;
  int32_T c2_i322;
  int32_T c2_i323;
  int32_T c2_i324;
  int32_T c2_i325;
  real_T c2_t_a;
  int32_T c2_i326;
  int32_T c2_i327;
  int32_T c2_i328;
  int32_T c2_i329;
  int32_T c2_i330;
  int32_T c2_i331;
  int32_T c2_i332;
  real_T c2_t_b;
  int32_T c2_i333;
  real_T c2_l_hoistedGlobal;
  real_T c2_k_B;
  real_T c2_pc_y;
  real_T c2_qc_y;
  real_T c2_rc_y;
  int32_T c2_i334;
  real_T c2_m_hoistedGlobal;
  real_T c2_u_a;
  int32_T c2_i335;
  real_T c2_sc_y[3];
  int32_T c2_i336;
  real_T c2_v_a;
  int32_T c2_i337;
  int32_T c2_i338;
  real_T c2_w_a;
  int32_T c2_i339;
  int32_T c2_i340;
  real_T c2_x_a;
  int32_T c2_i341;
  int32_T c2_i342;
  real_T c2_y_a;
  int32_T c2_i343;
  int32_T c2_i344;
  int32_T c2_i345;
  real_T c2_u_b[3];
  int32_T c2_i346;
  real_T c2_d_z[3];
  int32_T c2_i347;
  int32_T c2_i348;
  real_T c2_b_r3[3];
  int32_T c2_i349;
  real_T c2_dv19[9];
  int32_T c2_i350;
  int32_T c2_i351;
  int32_T c2_i352;
  int32_T c2_i353;
  int32_T c2_i354;
  real_T c2_f_mu[3];
  real_T c2_l_B;
  real_T c2_tc_y;
  real_T c2_uc_y;
  real_T c2_vc_y;
  int32_T c2_i355;
  int32_T c2_i356;
  int32_T c2_i357;
  int32_T c2_i358;
  int32_T c2_i359;
  int32_T c2_i360;
  int32_T c2_i361;
  real_T c2_b_R0[9];
  int32_T c2_i362;
  int32_T c2_i363;
  int32_T c2_i364;
  int32_T c2_i365;
  real_T c2_v_b[27];
  int32_T c2_i366;
  int32_T c2_i367;
  int32_T c2_i368;
  real_T c2_wc_y[27];
  int32_T c2_i369;
  int32_T c2_i370;
  int32_T c2_i371;
  int32_T c2_i372;
  int32_T c2_i373;
  int32_T c2_i374;
  int32_T c2_i375;
  int32_T c2_i376;
  int32_T c2_i377;
  int32_T c2_i378;
  int32_T c2_i379;
  int32_T c2_i380;
  int32_T c2_i381;
  int32_T c2_i382;
  int32_T c2_i383;
  int32_T c2_i384;
  int32_T c2_i385;
  int32_T c2_i386;
  int32_T c2_i387;
  int32_T c2_i388;
  int32_T c2_i389;
  int32_T c2_i390;
  int32_T c2_i391;
  int32_T c2_i392;
  int32_T c2_i393;
  int32_T c2_i394;
  int32_T c2_i395;
  int32_T c2_i396;
  int32_T c2_i397;
  int32_T c2_i398;
  int32_T c2_i399;
  int32_T c2_i400;
  int32_T c2_i401;
  real_T c2_w_b;
  int32_T c2_i402;
  real_T c2_n_hoistedGlobal;
  real_T c2_m_B;
  real_T c2_xc_y;
  real_T c2_yc_y;
  real_T c2_ad_y;
  int32_T c2_i403;
  real_T c2_o_hoistedGlobal;
  real_T c2_ab_a;
  int32_T c2_i404;
  int32_T c2_i405;
  real_T c2_bb_a;
  int32_T c2_i406;
  int32_T c2_i407;
  real_T c2_cb_a;
  int32_T c2_i408;
  int32_T c2_i409;
  real_T c2_db_a;
  int32_T c2_i410;
  int32_T c2_i411;
  real_T c2_eb_a;
  int32_T c2_i412;
  int32_T c2_i413;
  int32_T c2_i414;
  real_T c2_x_b[3];
  int32_T c2_i415;
  real_T c2_e_z[3];
  int32_T c2_i416;
  int32_T c2_i417;
  real_T c2_c_r3[3];
  int32_T c2_i418;
  real_T c2_dv20[9];
  int32_T c2_i419;
  int32_T c2_i420;
  int32_T c2_i421;
  int32_T c2_i422;
  int32_T c2_i423;
  real_T c2_g_mu[3];
  real_T c2_n_B;
  real_T c2_bd_y;
  real_T c2_cd_y;
  real_T c2_dd_y;
  int32_T c2_i424;
  int32_T c2_i425;
  int32_T c2_i426;
  int32_T c2_i427;
  int32_T c2_i428;
  int32_T c2_i429;
  int32_T c2_i430;
  real_T c2_c_R0[9];
  int32_T c2_i431;
  int32_T c2_i432;
  int32_T c2_i433;
  int32_T c2_i434;
  int32_T c2_i435;
  int32_T c2_i436;
  int32_T c2_i437;
  int32_T c2_i438;
  int32_T c2_i439;
  int32_T c2_i440;
  int32_T c2_i441;
  int32_T c2_i442;
  int32_T c2_i443;
  int32_T c2_i444;
  int32_T c2_i445;
  int32_T c2_i446;
  int32_T c2_i447;
  int32_T c2_i448;
  int32_T c2_i449;
  int32_T c2_i450;
  int32_T c2_i451;
  int32_T c2_i452;
  int32_T c2_i453;
  int32_T c2_i454;
  int32_T c2_i455;
  int32_T c2_i456;
  int32_T c2_i457;
  int32_T c2_i458;
  real_T c2_c_w[3];
  int32_T c2_i459;
  int32_T c2_i460;
  int32_T c2_i461;
  int32_T c2_i462;
  int32_T c2_i463;
  int32_T c2_i464;
  int32_T c2_i465;
  int32_T c2_i466;
  real_T c2_y_b;
  int32_T c2_i467;
  real_T c2_p_hoistedGlobal;
  real_T c2_o_B;
  real_T c2_ed_y;
  real_T c2_fd_y;
  real_T c2_gd_y;
  int32_T c2_i468;
  int32_T c2_i469;
  int32_T c2_i470;
  int32_T c2_i471;
  int32_T c2_i472;
  int32_T c2_i473;
  int32_T c2_i474;
  int32_T c2_i475;
  int32_T c2_i476;
  int32_T c2_i477;
  int32_T c2_i478;
  int32_T c2_i479;
  int32_T c2_i480;
  int32_T c2_i481;
  real_T c2_h_mu[3];
  real_T c2_p_B;
  real_T c2_hd_y;
  real_T c2_id_y;
  real_T c2_jd_y;
  int32_T c2_i482;
  int32_T c2_i483;
  real_T c2_fb_a;
  int32_T c2_i484;
  int32_T c2_i485;
  real_T c2_gb_a;
  int32_T c2_i486;
  int32_T c2_i487;
  int32_T c2_i488;
  real_T c2_hb_a;
  int32_T c2_i489;
  int32_T c2_i490;
  real_T c2_ib_a;
  int32_T c2_i491;
  int32_T c2_i492;
  int32_T c2_i493;
  real_T c2_jb_a;
  int32_T c2_i494;
  int32_T c2_i495;
  real_T c2_kb_a;
  int32_T c2_i496;
  int32_T c2_i497;
  int32_T c2_i498;
  int32_T c2_i499;
  int32_T c2_e_I;
  int32_T c2_i500;
  int32_T c2_i501;
  int32_T c2_i502;
  real_T c2_kd_y;
  int32_T c2_fb_k;
  int32_T c2_gb_k;
  real_T c2_lb_a;
  int32_T c2_i503;
  int32_T c2_i504;
  int32_T c2_i505;
  int32_T c2_i506;
  int32_T c2_i507;
  real_T c2_ld_y;
  int32_T c2_hb_k;
  int32_T c2_ib_k;
  real_T c2_mb_a;
  int32_T c2_i508;
  int32_T c2_i509;
  int32_T c2_i510;
  int32_T c2_i511;
  real_T c2_md_y;
  int32_T c2_jb_k;
  int32_T c2_kb_k;
  int32_T c2_i512;
  int32_T c2_i513;
  real_T c2_nd_y;
  int32_T c2_lb_k;
  int32_T c2_mb_k;
  real_T c2_d3;
  int32_T c2_i514;
  int32_T c2_i515;
  int32_T c2_i516;
  int32_T c2_i517;
  int32_T c2_i518;
  int32_T c2_i519;
  real_T c2_od_y;
  int32_T c2_nb_k;
  int32_T c2_ob_k;
  int32_T c2_i520;
  real_T c2_pd_y[2];
  int32_T c2_i521;
  static real_T c2_dv21[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_dv22[36];
  real_T c2_nb_a[72];
  real_T c2_ab_b;
  int32_T c2_i522;
  int32_T c2_i523;
  int32_T c2_i524;
  int32_T c2_i525;
  int32_T c2_i526;
  int32_T c2_i527;
  real_T c2_ob_a;
  int32_T c2_i528;
  real_T c2_pb_a;
  int32_T c2_i529;
  real_T c2_qb_a;
  int32_T c2_i530;
  int32_T c2_i531;
  int32_T c2_i532;
  int32_T c2_i533;
  real_T c2_bb_b[18];
  int32_T c2_i534;
  int32_T c2_i535;
  int32_T c2_i536;
  int32_T c2_i537;
  int32_T c2_i538;
  int32_T c2_i539;
  int32_T c2_i540;
  int32_T c2_i541;
  int32_T c2_i542;
  int32_T c2_i543;
  int32_T c2_i544;
  int32_T c2_i545;
  int32_T c2_i546;
  real_T c2_rb_a;
  int32_T c2_i547;
  real_T c2_sb_a;
  int32_T c2_i548;
  real_T c2_tb_a;
  int32_T c2_i549;
  int32_T c2_i550;
  int32_T c2_i551;
  int32_T c2_i552;
  int32_T c2_i553;
  int32_T c2_i554;
  int32_T c2_i555;
  int32_T c2_i556;
  int32_T c2_i557;
  int32_T c2_i558;
  real_T c2_qd_y[18];
  int32_T c2_i559;
  int32_T c2_i560;
  int32_T c2_i561;
  int32_T c2_i562;
  int32_T c2_i563;
  int32_T c2_i564;
  real_T c2_cb_b[72];
  int32_T c2_i565;
  int32_T c2_i566;
  int32_T c2_i567;
  int32_T c2_i568;
  int32_T c2_i569;
  int32_T c2_i570;
  int32_T c2_i571;
  int32_T c2_i572;
  int32_T c2_i573;
  int32_T c2_i574;
  int32_T c2_i575;
  real_T c2_db_b[36];
  int32_T c2_i576;
  int32_T c2_i577;
  int32_T c2_i578;
  int32_T c2_i579;
  real_T c2_rd_y;
  int32_T c2_pb_k;
  int32_T c2_qb_k;
  real_T c2_ub_a;
  int32_T c2_i580;
  real_T c2_vb_a;
  int32_T c2_i581;
  real_T c2_wb_a;
  int32_T c2_i582;
  int32_T c2_i583;
  int32_T c2_i584;
  int32_T c2_i585;
  real_T c2_eb_b[18];
  int32_T c2_i586;
  int32_T c2_i587;
  int32_T c2_i588;
  int32_T c2_i589;
  int32_T c2_i590;
  int32_T c2_i591;
  int32_T c2_i592;
  int32_T c2_i593;
  int32_T c2_i594;
  int32_T c2_i595;
  int32_T c2_i596;
  int32_T c2_i597;
  int32_T c2_i598;
  int32_T c2_i599;
  real_T c2_sd_y[18];
  int32_T c2_i600;
  int32_T c2_i601;
  real_T c2_xb_a;
  int32_T c2_i602;
  real_T c2_yb_a;
  int32_T c2_i603;
  int32_T c2_i604;
  int32_T c2_i605;
  int32_T c2_i606;
  int32_T c2_i607;
  int32_T c2_i608;
  int32_T c2_i609;
  int32_T c2_i610;
  int32_T c2_i611;
  int32_T c2_i612;
  int32_T c2_i613;
  real_T c2_td_y[36];
  int32_T c2_i614;
  int32_T c2_i615;
  int32_T c2_i616;
  int32_T c2_i617;
  int32_T c2_i618;
  int32_T c2_i619;
  int32_T c2_i620;
  int32_T c2_i621;
  int32_T c2_i622;
  int32_T c2_i623;
  int32_T c2_i624;
  int32_T c2_i625;
  int32_T c2_i626;
  int32_T c2_i627;
  int32_T c2_i628;
  real_T c2_ud_y;
  int32_T c2_rb_k;
  int32_T c2_sb_k;
  real_T c2_ac_a;
  int32_T c2_i629;
  int32_T c2_i630;
  real_T c2_bc_a;
  int32_T c2_i631;
  real_T c2_cc_a;
  int32_T c2_i632;
  int32_T c2_i633;
  int32_T c2_i634;
  int32_T c2_i635;
  int32_T c2_i636;
  int32_T c2_i637;
  int32_T c2_i638;
  int32_T c2_i639;
  int32_T c2_i640;
  real_T c2_vd_y[6];
  int32_T c2_i641;
  int32_T c2_i642;
  int32_T c2_i643;
  real_T c2_fb_b[12];
  int32_T c2_i644;
  int32_T c2_i645;
  int32_T c2_i646;
  int32_T c2_i647;
  int32_T c2_i648;
  int32_T c2_i649;
  int32_T c2_i650;
  int32_T c2_i651;
  real_T c2_wd_y[36];
  int32_T c2_i652;
  int32_T c2_i653;
  int32_T c2_i654;
  real_T c2_dc_a;
  int32_T c2_i655;
  int32_T c2_i656;
  int32_T c2_i657;
  int32_T c2_i658;
  int32_T c2_i659;
  int32_T c2_i660;
  int32_T c2_i661;
  int32_T c2_i662;
  int32_T c2_i663;
  int32_T c2_i664;
  int32_T c2_i665;
  int32_T c2_i666;
  int32_T c2_i667;
  int32_T c2_i668;
  int32_T c2_i669;
  int32_T c2_i670;
  int32_T c2_i671;
  int32_T c2_i672;
  int32_T c2_i673;
  int32_T c2_i674;
  real_T c2_g_u[3];
  int32_T c2_i675;
  real_T c2_f_z[3];
  int32_T c2_i676;
  int32_T c2_i677;
  real_T c2_h_u[3];
  int32_T c2_i678;
  real_T c2_i_u[3];
  int32_T c2_i679;
  real_T c2_g_z[3];
  int32_T c2_i680;
  real_T c2_c_dot_z[3];
  int32_T c2_i681;
  real_T c2_h_z[3];
  int32_T c2_i682;
  int32_T c2_i683;
  real_T c2_d_r3[3];
  int32_T c2_i684;
  real_T c2_dv23[9];
  int32_T c2_i685;
  int32_T c2_i686;
  int32_T c2_i687;
  int32_T c2_i688;
  int32_T c2_i689;
  real_T c2_i_mu[3];
  real_T c2_q_B;
  real_T c2_xd_y;
  real_T c2_yd_y;
  real_T c2_ae_y;
  int32_T c2_i690;
  int32_T c2_i691;
  int32_T c2_i692;
  real_T c2_j_mu[3];
  real_T c2_r_B;
  real_T c2_be_y;
  real_T c2_ce_y;
  real_T c2_de_y;
  int32_T c2_i693;
  int32_T c2_i694;
  real_T c2_ee_y[3];
  int32_T c2_i695;
  real_T c2_gb_b[9];
  int32_T c2_i696;
  int32_T c2_i697;
  real_T c2_e_r3[3];
  int32_T c2_i698;
  int32_T c2_i699;
  int32_T c2_i700;
  int32_T c2_i701;
  real_T c2_dv24[9];
  int32_T c2_i702;
  real_T c2_hb_b[9];
  real_T c2_ib_b[81];
  int32_T c2_i703;
  real_T c2_f_r3[3];
  int32_T c2_i704;
  real_T c2_jb_b[9];
  int32_T c2_i705;
  real_T c2_dv25[9];
  real_T c2_fe_y[81];
  int32_T c2_i706;
  int32_T c2_i707;
  int32_T c2_i708;
  int32_T c2_i709;
  int32_T c2_i710;
  int32_T c2_i711;
  int32_T c2_i712;
  int32_T c2_i713;
  int32_T c2_i714;
  int32_T c2_i715;
  int32_T c2_i716;
  int32_T c2_i717;
  int32_T c2_i718;
  static real_T c2_kb_b[27] = { -0.0, 0.0, -0.0, -0.0, 0.0, -1.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, 0.0, -1.0, -0.0, 0.0, 0.0, -1.0, -0.0,
    1.0, -0.0, -0.0, 0.0, -0.0, -0.0 };

  int32_T c2_i719;
  int32_T c2_i720;
  int32_T c2_i721;
  int32_T c2_i722;
  int32_T c2_i723;
  int32_T c2_i724;
  real_T c2_k_mu[3];
  real_T c2_s_B;
  real_T c2_ge_y;
  real_T c2_he_y;
  real_T c2_ie_y;
  int32_T c2_i725;
  int32_T c2_i726;
  int32_T c2_i727;
  int32_T c2_i728;
  int32_T c2_i729;
  int32_T c2_i730;
  int32_T c2_i731;
  int32_T c2_i732;
  int32_T c2_i733;
  int32_T c2_i734;
  int32_T c2_i735;
  real_T c2_l_mu[3];
  real_T c2_t_B;
  real_T c2_je_y;
  real_T c2_ke_y;
  real_T c2_le_y;
  int32_T c2_i736;
  int32_T c2_i737;
  real_T c2_g_r3[3];
  int32_T c2_i738;
  real_T c2_dv26[9];
  int32_T c2_i739;
  int32_T c2_i740;
  int32_T c2_i741;
  int32_T c2_i742;
  int32_T c2_i743;
  int32_T c2_i744;
  real_T c2_me_y[81];
  int32_T c2_i745;
  static real_T c2_ec_a[729] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_fc_a[729];
  int32_T c2_i746;
  static real_T c2_lb_b[81] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_mb_b[81];
  int32_T c2_i747;
  real_T c2_ne_y[81];
  int32_T c2_i748;
  static real_T c2_gc_a[729] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_hc_a[729];
  int32_T c2_i749;
  static real_T c2_nb_b[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_ob_b[81];
  int32_T c2_i750;
  int32_T c2_i751;
  int32_T c2_i752;
  int32_T c2_i753;
  int32_T c2_i754;
  int32_T c2_i755;
  int32_T c2_i756;
  int32_T c2_i757;
  int32_T c2_i758;
  int32_T c2_i759;
  int32_T c2_i760;
  int32_T c2_i761;
  int32_T c2_i762;
  int32_T c2_i763;
  int32_T c2_i764;
  int32_T c2_i765;
  int32_T c2_i766;
  int32_T c2_i767;
  int32_T c2_i768;
  real_T c2_i_gm[3];
  real_T c2_u_B;
  real_T c2_oe_y;
  real_T c2_pe_y;
  real_T c2_qe_y;
  int32_T c2_i769;
  int32_T c2_i770;
  int32_T c2_i771;
  real_T c2_j_gm[3];
  real_T c2_v_B;
  real_T c2_re_y;
  real_T c2_se_y;
  real_T c2_te_y;
  int32_T c2_i772;
  int32_T c2_i773;
  real_T c2_ue_y[3];
  int32_T c2_i774;
  static real_T c2_dv27[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_dv28[81];
  real_T c2_ic_a[243];
  int32_T c2_i775;
  real_T c2_pb_b[3];
  int32_T c2_i776;
  real_T c2_qb_b[9];
  int32_T c2_i777;
  real_T c2_dv29[9];
  int32_T c2_i778;
  real_T c2_e_C[3];
  int32_T c2_i779;
  real_T c2_dv30[9];
  int32_T c2_i780;
  real_T c2_rb_b[9];
  int32_T c2_i781;
  int32_T c2_i782;
  real_T c2_ve_y[243];
  int32_T c2_i783;
  real_T c2_jc_a[243];
  int32_T c2_i784;
  real_T c2_sb_b[81];
  int32_T c2_i785;
  int32_T c2_i786;
  real_T c2_we_y[243];
  int32_T c2_i787;
  real_T c2_tb_b[27];
  int32_T c2_i788;
  int32_T c2_i789;
  real_T c2_k_gm[3];
  real_T c2_w_B;
  real_T c2_xe_y;
  real_T c2_ye_y;
  real_T c2_af_y;
  int32_T c2_i790;
  int32_T c2_i791;
  real_T c2_ub_b[3];
  int32_T c2_i792;
  real_T c2_dv31[9];
  int32_T c2_i793;
  int32_T c2_i794;
  int32_T c2_i795;
  int32_T c2_i796;
  int32_T c2_i797;
  int32_T c2_i798;
  int32_T c2_i799;
  int32_T c2_i800;
  int32_T c2_i801;
  int32_T c2_i802;
  real_T c2_bf_y[3];
  int32_T c2_i803;
  int32_T c2_i804;
  int32_T c2_i805;
  int32_T c2_i806;
  int32_T c2_i807;
  int32_T c2_i808;
  int32_T c2_i809;
  real_T c2_l_gm[3];
  real_T c2_x_B;
  real_T c2_cf_y;
  real_T c2_df_y;
  real_T c2_ef_y;
  int32_T c2_i810;
  int32_T c2_i811;
  int32_T c2_i812;
  real_T c2_m_gm[3];
  real_T c2_y_B;
  real_T c2_ff_y;
  real_T c2_gf_y;
  real_T c2_hf_y;
  int32_T c2_i813;
  int32_T c2_i814;
  int32_T c2_i815;
  int32_T c2_i816;
  int32_T c2_i817;
  real_T c2_vb_b[3];
  int32_T c2_i818;
  real_T c2_dv32[9];
  int32_T c2_i819;
  real_T c2_if_y[3];
  int32_T c2_i820;
  int32_T c2_i821;
  int32_T c2_i822;
  int32_T c2_i823;
  int32_T c2_i824;
  int32_T c2_i825;
  real_T c2_n_gm[3];
  real_T c2_ab_B;
  real_T c2_jf_y;
  real_T c2_kf_y;
  real_T c2_lf_y;
  int32_T c2_i826;
  int32_T c2_i827;
  int32_T c2_i828;
  int32_T c2_i829;
  int32_T c2_i830;
  real_T c2_mf_y[3];
  int32_T c2_i831;
  int32_T c2_i832;
  int32_T c2_i833;
  int32_T c2_i834;
  int32_T c2_i835;
  real_T c2_o_gm[3];
  real_T c2_bb_B;
  real_T c2_nf_y;
  real_T c2_of_y;
  real_T c2_pf_y;
  int32_T c2_i836;
  int32_T c2_i837;
  int32_T c2_i838;
  real_T c2_p_gm[3];
  real_T c2_cb_B;
  real_T c2_qf_y;
  real_T c2_rf_y;
  real_T c2_sf_y;
  int32_T c2_i839;
  int32_T c2_i840;
  int32_T c2_i841;
  real_T c2_q_gm[3];
  real_T c2_db_B;
  real_T c2_tf_y;
  real_T c2_uf_y;
  real_T c2_vf_y;
  int32_T c2_i842;
  int32_T c2_i843;
  real_T c2_f_C[9];
  int32_T c2_i844;
  real_T c2_wb_b[9];
  int32_T c2_i845;
  real_T c2_xb_b[3];
  int32_T c2_i846;
  real_T c2_yb_b[9];
  int32_T c2_i847;
  real_T c2_dv33[9];
  int32_T c2_i848;
  real_T c2_g_C[3];
  int32_T c2_i849;
  real_T c2_dv34[9];
  int32_T c2_i850;
  real_T c2_ac_b[9];
  int32_T c2_i851;
  int32_T c2_i852;
  int32_T c2_i853;
  real_T c2_kc_a[81];
  int32_T c2_i854;
  real_T c2_bc_b[81];
  int32_T c2_i855;
  int32_T c2_i856;
  int32_T c2_i857;
  int32_T c2_i858;
  int32_T c2_i859;
  int32_T c2_i860;
  int32_T c2_i861;
  real_T c2_r_gm[3];
  real_T c2_eb_B;
  real_T c2_wf_y;
  real_T c2_xf_y;
  real_T c2_yf_y;
  int32_T c2_i862;
  int32_T c2_i863;
  real_T c2_cc_b[3];
  int32_T c2_i864;
  real_T c2_dv35[9];
  int32_T c2_i865;
  int32_T c2_i866;
  int32_T c2_i867;
  int32_T c2_i868;
  int32_T c2_i869;
  int32_T c2_i870;
  int32_T c2_i871;
  int32_T c2_i872;
  int32_T c2_i873;
  int32_T c2_i874;
  real_T c2_ag_y[3];
  int32_T c2_i875;
  int32_T c2_i876;
  int32_T c2_i877;
  int32_T c2_i878;
  int32_T c2_i879;
  int32_T c2_i880;
  int32_T c2_i881;
  real_T c2_s_gm[3];
  real_T c2_fb_B;
  real_T c2_bg_y;
  real_T c2_cg_y;
  real_T c2_dg_y;
  int32_T c2_i882;
  int32_T c2_i883;
  int32_T c2_i884;
  real_T c2_t_gm[3];
  real_T c2_gb_B;
  real_T c2_eg_y;
  real_T c2_fg_y;
  real_T c2_gg_y;
  int32_T c2_i885;
  int32_T c2_i886;
  real_T c2_dc_b[3];
  int32_T c2_i887;
  real_T c2_dv36[9];
  int32_T c2_i888;
  real_T c2_ec_b[9];
  int32_T c2_i889;
  real_T c2_h_C[9];
  int32_T c2_i890;
  int32_T c2_i891;
  int32_T c2_i892;
  int32_T c2_i893;
  int32_T c2_i894;
  int32_T c2_i895;
  int32_T c2_i896;
  int32_T c2_i897;
  int32_T c2_i898;
  int32_T c2_i899;
  int32_T c2_i900;
  int32_T c2_i901;
  int32_T c2_i902;
  int32_T c2_i903;
  int32_T c2_i904;
  int32_T c2_i905;
  int32_T c2_i906;
  int32_T c2_i907;
  int32_T c2_i908;
  int32_T c2_i909;
  int32_T c2_i910;
  int32_T c2_i911;
  real_T c2_hg_y[3];
  int32_T c2_i912;
  int32_T c2_i913;
  int32_T c2_i914;
  int32_T c2_i915;
  int32_T c2_i916;
  int32_T c2_i917;
  int32_T c2_i918;
  real_T c2_u_gm[3];
  real_T c2_hb_B;
  real_T c2_ig_y;
  real_T c2_jg_y;
  real_T c2_kg_y;
  int32_T c2_i919;
  int32_T c2_i920;
  int32_T c2_i921;
  int32_T c2_i922;
  int32_T c2_i923;
  int32_T c2_i924;
  int32_T c2_i925;
  int32_T c2_i926;
  int32_T c2_i927;
  int32_T c2_i928;
  int32_T c2_i929;
  int32_T c2_i930;
  int32_T c2_i931;
  int32_T c2_i932;
  int32_T c2_i933;
  int32_T c2_i934;
  int32_T c2_i935;
  int32_T c2_i936;
  int32_T c2_i937;
  int32_T c2_i938;
  real_T c2_b_H2[9];
  int32_T c2_i939;
  real_T c2_dv37[81];
  real_T c2_lc_a[729];
  int32_T c2_i940;
  real_T c2_fc_b[81];
  int32_T c2_i941;
  int32_T c2_i942;
  real_T c2_mc_a[729];
  int32_T c2_i943;
  real_T c2_gc_b[81];
  int32_T c2_i944;
  real_T c2_hc_b[9];
  int32_T c2_i945;
  real_T c2_b_H1[27];
  int32_T c2_i946;
  int32_T c2_i947;
  int32_T c2_i948;
  real_T c2_nc_a[243];
  int32_T c2_i949;
  real_T c2_ic_b[27];
  int32_T c2_i950;
  int32_T c2_i951;
  int32_T c2_i952;
  int32_T c2_i953;
  int32_T c2_i954;
  real_T c2_b_H[27];
  int32_T c2_i955;
  real_T c2_dv38[81];
  real_T c2_oc_a[2187];
  int32_T c2_i956;
  int32_T c2_i957;
  real_T c2_lg_y[27];
  int32_T c2_i958;
  real_T c2_pc_a[2187];
  int32_T c2_i959;
  static real_T c2_jc_b[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_kc_b[81];
  int32_T c2_i960;
  int32_T c2_i961;
  int32_T c2_i962;
  int32_T c2_i963;
  int32_T c2_i964;
  int32_T c2_i965;
  int32_T c2_i966;
  int32_T c2_i967;
  int32_T c2_i968;
  int32_T c2_i969;
  int32_T c2_i970;
  int32_T c2_i971;
  int32_T c2_i972;
  int32_T c2_i973;
  int32_T c2_i974;
  int32_T c2_i975;
  int32_T c2_i976;
  int32_T c2_i977;
  int32_T c2_i978;
  int32_T c2_i979;
  int32_T c2_i980;
  real_T c2_mg_y;
  int32_T c2_tb_k;
  int32_T c2_ub_k;
  real_T c2_qc_a;
  int32_T c2_i981;
  int32_T c2_i982;
  int32_T c2_i983;
  int32_T c2_i984;
  int32_T c2_i985;
  int32_T c2_i986;
  int32_T c2_i987;
  real_T c2_dv39[9];
  int32_T c2_i988;
  real_T c2_lc_b[9];
  int32_T c2_i989;
  real_T c2_mc_b[9];
  int32_T c2_i990;
  real_T c2_ng_y[81];
  int32_T c2_i991;
  int32_T c2_i992;
  int32_T c2_i993;
  int32_T c2_i994;
  real_T c2_rc_a[729];
  int32_T c2_i995;
  real_T c2_nc_b[81];
  int32_T c2_i996;
  int32_T c2_i997;
  int32_T c2_i998;
  int32_T c2_i999;
  int32_T c2_i1000;
  int32_T c2_i1001;
  int32_T c2_i1002;
  real_T c2_d_R0[9];
  int32_T c2_i1003;
  int32_T c2_i1004;
  int32_T c2_i1005;
  int32_T c2_i1006;
  int32_T c2_i1007;
  int32_T c2_i1008;
  int32_T c2_i1009;
  int32_T c2_i1010;
  int32_T c2_i1011;
  int32_T c2_i1012;
  int32_T c2_i1013;
  int32_T c2_i1014;
  int32_T c2_i1015;
  int32_T c2_i1016;
  int32_T c2_i1017;
  int32_T c2_i1018;
  int32_T c2_i1019;
  int32_T c2_i1020;
  int32_T c2_i1021;
  int32_T c2_i1022;
  int32_T c2_i1023;
  int32_T c2_i1024;
  int32_T c2_i1025;
  int32_T c2_i1026;
  int32_T c2_i1027;
  int32_T c2_i1028;
  int32_T c2_i1029;
  real_T c2_b_hat_w0[3];
  int32_T c2_i1030;
  int32_T c2_i1031;
  int32_T c2_i1032;
  int32_T c2_i1033;
  int32_T c2_i1034;
  int32_T c2_i1035;
  int32_T c2_i1036;
  int32_T c2_i1037;
  int32_T c2_i1038;
  int32_T c2_i1039;
  int32_T c2_i1040;
  int32_T c2_i1041;
  int32_T c2_i1042;
  int32_T c2_i1043;
  int32_T c2_vb_k;
  int32_T c2_wb_k;
  int32_T c2_i1044;
  real_T c2_og_y[9];
  int32_T c2_i1045;
  int32_T c2_i1046;
  int32_T c2_i1047;
  int32_T c2_i1048;
  int32_T c2_i1049;
  int32_T c2_i1050;
  int32_T c2_i1051;
  int32_T c2_i1052;
  int32_T c2_i1053;
  int32_T c2_i1054;
  real_T c2_e_R0[9];
  int32_T c2_i1055;
  int32_T c2_i1056;
  int32_T c2_i1057;
  int32_T c2_i1058;
  int32_T c2_i1059;
  int32_T c2_i1060;
  int32_T c2_i1061;
  int32_T c2_i1062;
  int32_T c2_i1063;
  int32_T c2_i1064;
  int32_T c2_i1065;
  int32_T c2_i1066;
  int32_T c2_i1067;
  int32_T c2_i1068;
  int32_T c2_i1069;
  int32_T c2_i1070;
  int32_T c2_i1071;
  int32_T c2_i1072;
  int32_T c2_i1073;
  int32_T c2_i1074;
  int32_T c2_i1075;
  int32_T c2_i1076;
  real_T c2_f_R0[9];
  int32_T c2_i1077;
  int32_T c2_i1078;
  int32_T c2_i1079;
  int32_T c2_i1080;
  int32_T c2_i1081;
  int32_T c2_i1082;
  int32_T c2_i1083;
  int32_T c2_i1084;
  int32_T c2_i1085;
  int32_T c2_i1086;
  real_T c2_b_hat_dot_r3[3];
  int32_T c2_i1087;
  real_T c2_dv40[81];
  real_T c2_sc_a[243];
  int32_T c2_i1088;
  int32_T c2_i1089;
  int32_T c2_i1090;
  real_T c2_tc_a[243];
  int32_T c2_i1091;
  real_T c2_oc_b[81];
  int32_T c2_i1092;
  int32_T c2_i1093;
  int32_T c2_i1094;
  int32_T c2_i1095;
  int32_T c2_i1096;
  int32_T c2_i1097;
  int32_T c2_i1098;
  real_T c2_pg_y[9];
  int32_T c2_i1099;
  int32_T c2_i1100;
  int32_T c2_i1101;
  int32_T c2_i1102;
  int32_T c2_i1103;
  int32_T c2_i1104;
  int32_T c2_i1105;
  real_T c2_q_hoistedGlobal;
  real_T c2_d_A;
  real_T c2_ib_B;
  real_T c2_k_x;
  real_T c2_qg_y;
  real_T c2_l_x;
  real_T c2_rg_y;
  real_T c2_m_x;
  real_T c2_sg_y;
  real_T c2_tg_y;
  int32_T c2_i1106;
  int32_T c2_i1107;
  int32_T c2_i1108;
  int32_T c2_i1109;
  int32_T c2_i1110;
  real_T c2_m_mu[3];
  int32_T c2_i1111;
  real_T c2_dv41[9];
  int32_T c2_i1112;
  real_T c2_pc_b[9];
  int32_T c2_i1113;
  int32_T c2_i1114;
  int32_T c2_i1115;
  int32_T c2_i1116;
  int32_T c2_i1117;
  int32_T c2_i1118;
  int32_T c2_i1119;
  int32_T c2_i1120;
  int32_T c2_i1121;
  int32_T c2_i1122;
  int32_T c2_i1123;
  int32_T c2_i1124;
  int32_T c2_i1125;
  int32_T c2_i1126;
  int32_T c2_i1127;
  int32_T c2_i1128;
  int32_T c2_i1129;
  int32_T c2_i1130;
  int32_T c2_i1131;
  real_T c2_uc_a;
  int32_T c2_i1132;
  int32_T c2_i1133;
  int32_T c2_i1134;
  int32_T c2_i1135;
  int32_T c2_i1136;
  int32_T c2_i1137;
  real_T c2_c_q1[3];
  int32_T c2_i1138;
  real_T c2_dv42[9];
  int32_T c2_i1139;
  real_T c2_qc_b[9];
  int32_T c2_i1140;
  int32_T c2_i1141;
  int32_T c2_i1142;
  int32_T c2_i1143;
  int32_T c2_i1144;
  int32_T c2_i1145;
  int32_T c2_i1146;
  real_T c2_ug_y[9];
  int32_T c2_i1147;
  int32_T c2_i1148;
  int32_T c2_i1149;
  real_T c2_n_mu[3];
  int32_T c2_i1150;
  real_T c2_rc_b[9];
  int32_T c2_i1151;
  real_T c2_dv43[9];
  int32_T c2_i1152;
  int32_T c2_i1153;
  int32_T c2_i1154;
  int32_T c2_i1155;
  int32_T c2_i1156;
  int32_T c2_i1157;
  int32_T c2_i1158;
  real_T c2_vc_a;
  int32_T c2_i1159;
  int32_T c2_i1160;
  real_T c2_d_q1[3];
  int32_T c2_i1161;
  int32_T c2_i1162;
  int32_T c2_i1163;
  int32_T c2_i1164;
  int32_T c2_i1165;
  int32_T c2_i1166;
  int32_T c2_i1167;
  int32_T c2_i1168;
  int32_T c2_i1169;
  int32_T c2_i1170;
  int32_T c2_i1171;
  int32_T c2_i1172;
  int32_T c2_i1173;
  int32_T c2_i1174;
  int32_T c2_i1175;
  int32_T c2_i1176;
  real_T c2_vg_y[9];
  int32_T c2_i1177;
  int32_T c2_i1178;
  int32_T c2_i1179;
  real_T c2_wg_y[3];
  int32_T c2_i1180;
  real_T c2_sc_b[9];
  int32_T c2_i1181;
  int32_T c2_i1182;
  int32_T c2_i1183;
  int32_T c2_i1184;
  real_T c2_wc_a;
  int32_T c2_i1185;
  real_T c2_o_mu[3];
  int32_T c2_i1186;
  int32_T c2_i1187;
  real_T c2_p_mu[3];
  int32_T c2_i1188;
  real_T c2_e_q1[3];
  int32_T c2_i1189;
  int32_T c2_i1190;
  int32_T c2_i1191;
  int32_T c2_i1192;
  int32_T c2_i1193;
  int32_T c2_i1194;
  int32_T c2_i1195;
  int32_T c2_i1196;
  int32_T c2_i1197;
  int32_T c2_i1198;
  int32_T c2_i1199;
  int32_T c2_i1200;
  int32_T c2_i1201;
  int32_T c2_i1202;
  int32_T c2_i1203;
  int32_T c2_i1204;
  int32_T c2_i1205;
  int32_T c2_i1206;
  int32_T c2_i1207;
  int32_T c2_i1208;
  int32_T c2_i1209;
  int32_T c2_i1210;
  int32_T c2_i1211;
  real_T c2_xc_a;
  int32_T c2_i1212;
  int32_T c2_i1213;
  real_T c2_yc_a;
  int32_T c2_i1214;
  real_T c2_ad_a;
  int32_T c2_i1215;
  real_T c2_f_q1[3];
  int32_T c2_i1216;
  int32_T c2_i1217;
  int32_T c2_i1218;
  int32_T c2_i1219;
  int32_T c2_i1220;
  int32_T c2_i1221;
  int32_T c2_i1222;
  int32_T c2_i1223;
  int32_T c2_i1224;
  int32_T c2_i1225;
  int32_T c2_i1226;
  int32_T c2_i1227;
  int32_T c2_i1228;
  int32_T c2_i1229;
  int32_T c2_i1230;
  int32_T c2_i1231;
  int32_T c2_i1232;
  int32_T c2_i1233;
  int32_T c2_i1234;
  int32_T c2_i1235;
  int32_T c2_i1236;
  int32_T c2_i1237;
  real_T c2_b_w1[3];
  int32_T c2_i1238;
  real_T c2_tc_b[9];
  int32_T c2_i1239;
  int32_T c2_i1240;
  int32_T c2_i1241;
  int32_T c2_i1242;
  real_T c2_g_R0[9];
  int32_T c2_i1243;
  int32_T c2_i1244;
  int32_T c2_i1245;
  int32_T c2_i1246;
  int32_T c2_i1247;
  int32_T c2_i1248;
  int32_T c2_i1249;
  int32_T c2_i1250;
  int32_T c2_i1251;
  int32_T c2_i1252;
  int32_T c2_i1253;
  real_T c2_bd_a;
  int32_T c2_i1254;
  int32_T c2_i1255;
  real_T c2_r_hoistedGlobal;
  real_T c2_cd_a;
  int32_T c2_i1256;
  int32_T c2_i1257;
  int32_T c2_i1258;
  int32_T c2_i1259;
  int32_T c2_i1260;
  int32_T c2_i1261;
  int32_T c2_i1262;
  int32_T c2_i1263;
  int32_T c2_i1264;
  int32_T c2_i1265;
  int32_T c2_i1266;
  int32_T c2_i1267;
  real_T c2_e_A;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_xg_y;
  real_T c2_f_A;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_g_A;
  real_T c2_t_x;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_h_A;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_y_x;
  real_T c2_i_A;
  real_T c2_ab_x;
  real_T c2_bb_x;
  real_T c2_cb_x;
  int32_T c2_i1268;
  int32_T c2_i1269;
  int32_T c2_i1270;
  int32_T c2_i1271;
  int32_T c2_i1272;
  int32_T c2_i1273;
  int32_T c2_i1274;
  int32_T c2_i1275;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *chartInstance->c2_h;
  c2_b_hoistedGlobal = *chartInstance->c2_kp;
  c2_c_hoistedGlobal = *chartInstance->c2_kv;
  c2_d_hoistedGlobal = *chartInstance->c2_kz;
  c2_e_hoistedGlobal = *chartInstance->c2_kV0;
  c2_f_hoistedGlobal = *chartInstance->c2_kq;
  c2_g_hoistedGlobal = *chartInstance->c2_kw;
  for (c2_i31 = 0; c2_i31 < 18; c2_i31++) {
    c2_b_state[c2_i31] = (*chartInstance->c2_state)[c2_i31];
  }

  for (c2_i32 = 0; c2_i32 < 15; c2_i32++) {
    c2_b_ref[c2_i32] = (*chartInstance->c2_ref)[c2_i32];
  }

  for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
    c2_b_z[c2_i33] = (*chartInstance->c2_z)[c2_i33];
  }

  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_b_hat_b[c2_i34] = (*chartInstance->c2_hat_b)[c2_i34];
  }

  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    c2_b_hat_b2[c2_i35] = (*chartInstance->c2_hat_b2)[c2_i35];
  }

  for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
    c2_b_w[c2_i36] = (*chartInstance->c2_w)[c2_i36];
  }

  c2_b_h = c2_hoistedGlobal;
  for (c2_i37 = 0; c2_i37 < 4; c2_i37++) {
    c2_b_qh[c2_i37] = (*chartInstance->c2_qh)[c2_i37];
  }

  c2_b_kp = c2_b_hoistedGlobal;
  c2_b_kv = c2_c_hoistedGlobal;
  c2_b_kz = c2_d_hoistedGlobal;
  c2_b_kV0 = c2_e_hoistedGlobal;
  c2_b_kq = c2_f_hoistedGlobal;
  c2_b_kw = c2_g_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 94U, 95U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P, 0U, c2_m_sf_marshallOut,
    c2_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_p, 1U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 2U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_v, 3U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r, MAX_uint32_T, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_I, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pd_, 6U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dpd_, 7U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_D2pd_, 8U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_D3pd_, 9U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_D4pd_, 10U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pd, 11U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_vd, 12U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dot_vd, 13U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ddot_vd, 14U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_d3dot_vd, 15U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_p0, 16U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_v0, 17U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u, 18U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u0, 19U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_e3, 20U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_mu, 21U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r3, 22U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_T, 23U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_gm, 24U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R0, 25U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_bar_V0, 26U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_grad_bar_V0, 27U, c2_l_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ei, 28U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_s, 29U, c2_k_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_H1, 30U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_H2, 31U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_R0, 32U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q1, 33U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w1, 34U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_v, 35U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_mu, 36U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_r3, 37U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_w0, 38U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_star, 39U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_v2, 40U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_mu2, 41U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_r32, 42U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_w02, 43U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_hat_dot_v2, 44U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_u, 45U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_u2, 46U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_hat_dot_u2, 47U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hess_bar_V0, 48U, c2_i_sf_marshallOut,
    c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_ddot_z2, 49U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_hat_dot_mu2, 50U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_hat_dot_r32, 51U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_H1, 52U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_Y, 53U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_H2, 54U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_H, 55U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_H, 56U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Dr3_X, 57U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_D2r3_R0, 58U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_hat_w0, 59U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_w1, 60U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_hat_dot_w_star, 61U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u2, 62U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_thrust, 63U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_aileron, 64U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_elevator, 65U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yaw, 66U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_r, MAX_uint32_T, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 67U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 68U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_state, 69U, c2_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_ref, 70U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_z, 71U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_hat_b, 72U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_hat_b2, 73U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_w, 74U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_h, 75U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_qh, 76U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kp, 77U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kv, 78U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kz, 79U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kV0, 80U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kq, 81U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_kw, 82U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_cmd, 83U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_dot_z, 84U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_dot_hat_b, 85U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_dot_hat_b2, 86U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_dot_w, 87U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_dot_h, 88U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_dot_qh, 89U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_V2, 90U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_dot_V2, 91U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q0, 92U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q, 93U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_A = -c2_b_kp;
  c2_B = c2_b_kv;
  c2_x = c2_A;
  c2_y = c2_B;
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_c_x = c2_b_x;
  c2_c_y = c2_b_y;
  c2_d_y = c2_c_x / c2_c_y;
  c2_b_A = -c2_b_kp;
  c2_b_B = c2_b_kv;
  c2_d_x = c2_b_A;
  c2_e_y = c2_b_B;
  c2_e_x = c2_d_x;
  c2_f_y = c2_e_y;
  c2_f_x = c2_e_x;
  c2_g_y = c2_f_y;
  c2_h_y = c2_f_x / c2_g_y;
  c2_P[0] = 1.0;
  c2_P[2] = c2_d_y;
  c2_P[1] = c2_h_y;
  c2_P[3] = c2_b_kp;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  for (c2_i38 = 0; c2_i38 < 3; c2_i38++) {
    c2_p[c2_i38] = c2_b_state[c2_i38];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  for (c2_i39 = 0; c2_i39 < 9; c2_i39++) {
    c2_g_x[c2_i39] = c2_b_state[c2_i39 + 6];
  }

  c2_eml_switch_helper(chartInstance);
  for (c2_k = 1; c2_k < 10; c2_k++) {
    c2_b_k = c2_k;
    c2_R[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 9, 1, 0) - 1] = c2_g_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 9, 1, 0) - 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i40 = 0; c2_i40 < 3; c2_i40++) {
    c2_v[c2_i40] = c2_b_state[c2_i40 + 3];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
    c2_r[c2_i41] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_i42 = 0;
  for (c2_i43 = 0; c2_i43 < 3; c2_i43++) {
    c2_i44 = 0;
    for (c2_i45 = 0; c2_i45 < 3; c2_i45++) {
      c2_b_u[c2_i45 + c2_i42] = c2_R[c2_i44 + c2_i43];
      c2_i44 += 3;
    }

    c2_i42 += 3;
  }

  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "transpose", 1U, 1U, 14,
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "dcm2quat", 1U, 1U, 14,
                      c2_i_y)), "transpose", c2_dv8);
  for (c2_i46 = 0; c2_i46 < 4; c2_i46++) {
    c2_r[c2_i46] = c2_dv8[c2_i46];
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  for (c2_i47 = 0; c2_i47 < 4; c2_i47++) {
    c2_c_r[c2_i47] = -c2_r[c2_i47];
  }

  for (c2_i48 = 0; c2_i48 < 4; c2_i48++) {
    c2_b_r[c2_i48] = c2_r[c2_i48];
  }

  for (c2_i49 = 0; c2_i49 < 4; c2_i49++) {
    c2_b_r[c2_i49 + 4] = c2_c_r[c2_i49];
  }

  _SFD_SYMBOL_SWITCH(4U, 67U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_i50 = 0;
  for (c2_i51 = 0; c2_i51 < 4; c2_i51++) {
    c2_i52 = 0;
    for (c2_i53 = 0; c2_i53 < 2; c2_i53++) {
      c2_a[c2_i53 + c2_i50] = c2_b_r[c2_i52 + c2_i51];
      c2_i52 += 4;
    }

    c2_i50 += 2;
  }

  for (c2_i54 = 0; c2_i54 < 4; c2_i54++) {
    c2_c_r[c2_i54] = c2_b_qh[c2_i54];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i55 = 0; c2_i55 < 2; c2_i55++) {
    c2_j_y[c2_i55] = 0.0;
    c2_i56 = 0;
    for (c2_i57 = 0; c2_i57 < 4; c2_i57++) {
      c2_j_y[c2_i55] += c2_a[c2_i56 + c2_i55] * c2_c_r[c2_i57];
      c2_i56 += 2;
    }
  }

  for (c2_i58 = 0; c2_i58 < 2; c2_i58++) {
    c2_k_y[c2_i58] = c2_j_y[c2_i58];
  }

  c2_eml_extremum(chartInstance, c2_k_y, &c2_extremum, &c2_iindx);
  c2_b_iindx = c2_iindx;
  c2_indx = (real_T)c2_b_iindx;
  c2_b_I = c2_indx;
  c2_I = c2_b_I;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  c2_c_I = _SFD_EML_ARRAY_BOUNDS_CHECK("r", (int32_T)_SFD_INTEGER_CHECK("I",
    c2_I), 1, 2, 2, 0) - 1;
  for (c2_i59 = 0; c2_i59 < 4; c2_i59++) {
    c2_b_q[c2_i59] = c2_b_r[c2_i59 + (c2_c_I << 2)];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  for (c2_i60 = 0; c2_i60 < 4; c2_i60++) {
    c2_b_dot_qh[c2_i60] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  for (c2_i61 = 0; c2_i61 < 3; c2_i61++) {
    c2_pd_[c2_i61] = c2_b_ref[c2_i61];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  for (c2_i62 = 0; c2_i62 < 3; c2_i62++) {
    c2_Dpd_[c2_i62] = c2_b_ref[c2_i62 + 3];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  for (c2_i63 = 0; c2_i63 < 3; c2_i63++) {
    c2_D2pd_[c2_i63] = c2_b_ref[c2_i63 + 6];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  for (c2_i64 = 0; c2_i64 < 3; c2_i64++) {
    c2_D3pd_[c2_i64] = c2_b_ref[c2_i64 + 9];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  for (c2_i65 = 0; c2_i65 < 3; c2_i65++) {
    c2_D4pd_[c2_i65] = c2_b_ref[c2_i65 + 12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  for (c2_i66 = 0; c2_i66 < 3; c2_i66++) {
    c2_pd[c2_i66] = c2_pd_[c2_i66];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  for (c2_i67 = 0; c2_i67 < 3; c2_i67++) {
    c2_vd[c2_i67] = c2_Dpd_[c2_i67];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  for (c2_i68 = 0; c2_i68 < 3; c2_i68++) {
    c2_dot_vd[c2_i68] = c2_D2pd_[c2_i68];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  for (c2_i69 = 0; c2_i69 < 3; c2_i69++) {
    c2_ddot_vd[c2_i69] = c2_D3pd_[c2_i69];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  for (c2_i70 = 0; c2_i70 < 3; c2_i70++) {
    c2_d3dot_vd[c2_i70] = c2_D4pd_[c2_i70];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  for (c2_i71 = 0; c2_i71 < 3; c2_i71++) {
    c2_p0[c2_i71] = c2_p[c2_i71] - c2_pd[c2_i71];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  for (c2_i72 = 0; c2_i72 < 3; c2_i72++) {
    c2_v0[c2_i72] = c2_v[c2_i72] - c2_vd[c2_i72];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 41);
  c2_b_a = c2_b_kp;
  for (c2_i73 = 0; c2_i73 < 3; c2_i73++) {
    c2_b[c2_i73] = c2_p0[c2_i73];
  }

  for (c2_i74 = 0; c2_i74 < 3; c2_i74++) {
    c2_b[c2_i74] *= c2_b_a;
  }

  c2_c_a = c2_b_kv;
  for (c2_i75 = 0; c2_i75 < 3; c2_i75++) {
    c2_b_b[c2_i75] = c2_v0[c2_i75];
  }

  for (c2_i76 = 0; c2_i76 < 3; c2_i76++) {
    c2_b_b[c2_i76] *= c2_c_a;
  }

  for (c2_i77 = 0; c2_i77 < 3; c2_i77++) {
    c2_u[c2_i77] = c2_b[c2_i77] + c2_b_b[c2_i77];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  for (c2_i78 = 0; c2_i78 < 3; c2_i78++) {
    c2_c_u[c2_i78] = c2_u[c2_i78];
  }

  c2_sat_atan(chartInstance, c2_c_u, c2_dv9);
  for (c2_i79 = 0; c2_i79 < 3; c2_i79++) {
    c2_u0[c2_i79] = -c2_dv9[c2_i79];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
  for (c2_i80 = 0; c2_i80 < 3; c2_i80++) {
    c2_e3[c2_i80] = c2_c_b[c2_i80];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 45);
  c2_h_hoistedGlobal = c2_get_g(chartInstance, 0);
  c2_d_a = c2_h_hoistedGlobal;
  for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
    c2_l_y[c2_i81] = c2_d_a * c2_c_b[c2_i81];
  }

  for (c2_i82 = 0; c2_i82 < 3; c2_i82++) {
    c2_c_z[c2_i82] = c2_b_z[c2_i82];
  }

  c2_sat_atan(chartInstance, c2_c_z, c2_dv9);
  for (c2_i83 = 0; c2_i83 < 3; c2_i83++) {
    c2_mu[c2_i83] = ((c2_u0[c2_i83] - c2_l_y[c2_i83]) + c2_dot_vd[c2_i83]) -
      c2_dv9[c2_i83];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
  for (c2_i84 = 0; c2_i84 < 3; c2_i84++) {
    c2_b_b[c2_i84] = -c2_mu[c2_i84];
  }

  for (c2_i85 = 0; c2_i85 < 3; c2_i85++) {
    c2_b_mu[c2_i85] = c2_mu[c2_i85];
  }

  c2_c_B = c2_norm(chartInstance, c2_b_mu);
  c2_m_y = c2_c_B;
  c2_n_y = c2_m_y;
  c2_o_y = c2_n_y;
  for (c2_i86 = 0; c2_i86 < 3; c2_i86++) {
    c2_r3[c2_i86] = c2_b_b[c2_i86] / c2_o_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 47);
  for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
    c2_c_mu[c2_i87] = c2_mu[c2_i87];
  }

  c2_T = c2_get_m(chartInstance, 0) * c2_norm(chartInstance, c2_c_mu);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i88 = 0; c2_i88 < 3; c2_i88++) {
    c2_l_y[c2_i88] = 0.0;
    c2_i89 = 0;
    for (c2_i90 = 0; c2_i90 < 3; c2_i90++) {
      c2_l_y[c2_i88] += c2_get_Rd(chartInstance, c2_i89 + c2_i88) *
        c2_c_b[c2_i90];
      c2_i89 += 3;
    }
  }

  for (c2_i91 = 0; c2_i91 < 3; c2_i91++) {
    c2_p_y[c2_i91] = c2_l_y[c2_i91];
  }

  c2_S(chartInstance, c2_p_y, c2_e_a);
  for (c2_i92 = 0; c2_i92 < 3; c2_i92++) {
    c2_b[c2_i92] = c2_r3[c2_i92];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i93 = 0; c2_i93 < 3; c2_i93++) {
    c2_gm[c2_i93] = 0.0;
  }

  for (c2_i94 = 0; c2_i94 < 3; c2_i94++) {
    c2_gm[c2_i94] = 0.0;
  }

  for (c2_i95 = 0; c2_i95 < 3; c2_i95++) {
    c2_C[c2_i95] = c2_gm[c2_i95];
  }

  for (c2_i96 = 0; c2_i96 < 3; c2_i96++) {
    c2_gm[c2_i96] = c2_C[c2_i96];
  }

  c2_threshold(chartInstance);
  for (c2_i97 = 0; c2_i97 < 3; c2_i97++) {
    c2_C[c2_i97] = c2_gm[c2_i97];
  }

  for (c2_i98 = 0; c2_i98 < 3; c2_i98++) {
    c2_gm[c2_i98] = c2_C[c2_i98];
  }

  for (c2_i99 = 0; c2_i99 < 3; c2_i99++) {
    c2_gm[c2_i99] = 0.0;
    c2_i100 = 0;
    for (c2_i101 = 0; c2_i101 < 3; c2_i101++) {
      c2_gm[c2_i99] += c2_e_a[c2_i100 + c2_i99] * c2_b[c2_i101];
      c2_i100 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 49);
  c2_i102 = 0;
  for (c2_i103 = 0; c2_i103 < 3; c2_i103++) {
    c2_i104 = 0;
    for (c2_i105 = 0; c2_i105 < 3; c2_i105++) {
      c2_d_b[c2_i105 + c2_i102] = c2_get_Rd(chartInstance, c2_i104 + c2_i103);
      c2_i104 += 3;
    }

    c2_i102 += 3;
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  c2_i106 = 0;
  for (c2_i107 = 0; c2_i107 < 3; c2_i107++) {
    c2_q_y[c2_i107] = 0.0;
    for (c2_i108 = 0; c2_i108 < 3; c2_i108++) {
      c2_q_y[c2_i107] += c2_f_a[c2_i108] * c2_d_b[c2_i108 + c2_i106];
    }

    c2_i106 += 3;
  }

  for (c2_i109 = 0; c2_i109 < 3; c2_i109++) {
    c2_b[c2_i109] = c2_r3[c2_i109];
  }

  c2_b_threshold(chartInstance);
  c2_r_y = 0.0;
  c2_eml_switch_helper(chartInstance);
  for (c2_c_k = 1; c2_c_k < 4; c2_c_k++) {
    c2_d_k = c2_c_k;
    c2_r_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c2_d_k), 1, 3, 1, 0) - 1] * c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_d_k), 1, 3, 1, 0) - 1];
  }

  for (c2_i110 = 0; c2_i110 < 3; c2_i110++) {
    c2_b_b[c2_i110] = c2_gm[c2_i110];
  }

  for (c2_i111 = 0; c2_i111 < 3; c2_i111++) {
    c2_b_gm[c2_i111] = c2_gm[c2_i111];
  }

  c2_d_B = c2_norm(chartInstance, c2_b_gm);
  c2_s_y = c2_d_B;
  c2_t_y = c2_s_y;
  c2_u_y = c2_t_y;
  for (c2_i112 = 0; c2_i112 < 3; c2_i112++) {
    c2_b_b[c2_i112] /= c2_u_y;
  }

  c2_g_a = 1.0 - c2_r_y;
  for (c2_i113 = 0; c2_i113 < 3; c2_i113++) {
    c2_e_b[c2_i113] = c2_b_b[c2_i113];
  }

  c2_S(chartInstance, c2_e_b, c2_dv10);
  for (c2_i114 = 0; c2_i114 < 9; c2_i114++) {
    c2_dv11[c2_i114] = c2_dv10[c2_i114];
  }

  c2_mpower(chartInstance, c2_dv11, c2_d_b);
  for (c2_i115 = 0; c2_i115 < 9; c2_i115++) {
    c2_d_b[c2_i115] *= c2_g_a;
  }

  for (c2_i116 = 0; c2_i116 < 9; c2_i116++) {
    c2_i_hoistedGlobal[c2_i116] = c2_get_Rd(chartInstance, c2_i116);
  }

  c2_eye(chartInstance, c2_e_a);
  for (c2_i117 = 0; c2_i117 < 3; c2_i117++) {
    c2_c_gm[c2_i117] = c2_gm[c2_i117];
  }

  c2_S(chartInstance, c2_c_gm, c2_dv10);
  for (c2_i118 = 0; c2_i118 < 9; c2_i118++) {
    c2_e_a[c2_i118] = (c2_e_a[c2_i118] + c2_dv10[c2_i118]) + c2_d_b[c2_i118];
  }

  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  for (c2_i119 = 0; c2_i119 < 9; c2_i119++) {
    c2_R0[c2_i119] = 0.0;
  }

  for (c2_i120 = 0; c2_i120 < 9; c2_i120++) {
    c2_R0[c2_i120] = 0.0;
  }

  for (c2_i121 = 0; c2_i121 < 9; c2_i121++) {
    c2_b_C[c2_i121] = c2_R0[c2_i121];
  }

  for (c2_i122 = 0; c2_i122 < 9; c2_i122++) {
    c2_R0[c2_i122] = c2_b_C[c2_i122];
  }

  c2_threshold(chartInstance);
  for (c2_i123 = 0; c2_i123 < 9; c2_i123++) {
    c2_b_C[c2_i123] = c2_R0[c2_i123];
  }

  for (c2_i124 = 0; c2_i124 < 9; c2_i124++) {
    c2_R0[c2_i124] = c2_b_C[c2_i124];
  }

  for (c2_i125 = 0; c2_i125 < 3; c2_i125++) {
    c2_i126 = 0;
    for (c2_i127 = 0; c2_i127 < 3; c2_i127++) {
      c2_R0[c2_i126 + c2_i125] = 0.0;
      c2_i128 = 0;
      for (c2_i129 = 0; c2_i129 < 3; c2_i129++) {
        c2_R0[c2_i126 + c2_i125] += c2_e_a[c2_i128 + c2_i125] *
          c2_i_hoistedGlobal[c2_i129 + c2_i126];
        c2_i128 += 3;
      }

      c2_i126 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
  for (c2_i130 = 0; c2_i130 < 4; c2_i130++) {
    c2_b_q0[c2_i130] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
  c2_i131 = 0;
  for (c2_i132 = 0; c2_i132 < 3; c2_i132++) {
    c2_i133 = 0;
    for (c2_i134 = 0; c2_i134 < 3; c2_i134++) {
      c2_d_u[c2_i134 + c2_i131] = c2_R0[c2_i133 + c2_i132];
      c2_i133 += 3;
    }

    c2_i131 += 3;
  }

  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_create("y", c2_d_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "transpose", 1U, 1U, 14,
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "dcm2quat", 1U, 1U, 14,
                      c2_v_y)), "transpose", c2_dv12);
  for (c2_i135 = 0; c2_i135 < 4; c2_i135++) {
    c2_b_q0[c2_i135] = c2_dv12[c2_i135];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 57);
  c2_bar_V0 = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
  for (c2_i136 = 0; c2_i136 < 6; c2_i136++) {
    c2_grad_bar_V0[c2_i136] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
  c2_I = 1.0;
  c2_d_I = 0;
  while (c2_d_I < 3) {
    c2_I = 1.0 + (real_T)c2_d_I;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 60);
    for (c2_i137 = 0; c2_i137 < 3; c2_i137++) {
      c2_ei[c2_i137] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 61);
    c2_ei[_SFD_EML_ARRAY_BOUNDS_CHECK("ei", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 3, 1, 0) - 1] = 1.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 62);
    for (c2_i138 = 0; c2_i138 < 3; c2_i138++) {
      c2_q_y[c2_i138] = c2_ei[c2_i138];
    }

    for (c2_i139 = 0; c2_i139 < 3; c2_i139++) {
      c2_b[c2_i139] = c2_u[c2_i139];
    }

    c2_b_threshold(chartInstance);
    c2_w_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_e_k = 1; c2_e_k < 4; c2_e_k++) {
      c2_f_k = c2_e_k;
      c2_w_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_f_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_f_k), 1, 3, 1, 0) - 1];
    }

    c2_linspace(chartInstance, 0.0, c2_w_y, c2_dv13);
    for (c2_i140 = 0; c2_i140 < 1000; c2_i140++) {
      c2_s[c2_i140] = c2_dv13[c2_i140];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
    for (c2_i141 = 0; c2_i141 < 3; c2_i141++) {
      c2_q_y[c2_i141] = c2_ei[c2_i141];
    }

    for (c2_i142 = 0; c2_i142 < 3; c2_i142++) {
      c2_b[c2_i142] = c2_u[c2_i142];
    }

    c2_b_threshold(chartInstance);
    c2_x_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_g_k = 1; c2_g_k < 4; c2_g_k++) {
      c2_h_k = c2_g_k;
      c2_x_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_h_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_h_k), 1, 3, 1, 0) - 1];
    }

    for (c2_i143 = 0; c2_i143 < 3; c2_i143++) {
      c2_q_y[c2_i143] = c2_ei[c2_i143];
    }

    for (c2_i144 = 0; c2_i144 < 3; c2_i144++) {
      c2_b[c2_i144] = c2_v0[c2_i144];
    }

    c2_b_threshold(chartInstance);
    c2_y_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_i_k = 1; c2_i_k < 4; c2_i_k++) {
      c2_j_k = c2_i_k;
      c2_y_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_j_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_j_k), 1, 3, 1, 0) - 1];
    }

    c2_d0 = c2_b_sat_atan(chartInstance, c2_x_y);
    c2_h_a[0] = c2_d0;
    c2_h_a[1] = c2_y_y;
    for (c2_i145 = 0; c2_i145 < 2; c2_i145++) {
      c2_h_a[c2_i145] *= 0.5;
    }

    for (c2_i146 = 0; c2_i146 < 4; c2_i146++) {
      c2_f_b[c2_i146] = c2_P[c2_i146];
    }

    c2_f_eml_scalar_eg(chartInstance);
    c2_f_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    c2_i147 = 0;
    for (c2_i148 = 0; c2_i148 < 2; c2_i148++) {
      c2_ab_y[c2_i148] = 0.0;
      for (c2_i149 = 0; c2_i149 < 2; c2_i149++) {
        c2_ab_y[c2_i148] += c2_h_a[c2_i149] * c2_f_b[c2_i149 + c2_i147];
      }

      c2_i147 += 2;
    }

    for (c2_i150 = 0; c2_i150 < 3; c2_i150++) {
      c2_q_y[c2_i150] = c2_ei[c2_i150];
    }

    for (c2_i151 = 0; c2_i151 < 3; c2_i151++) {
      c2_b[c2_i151] = c2_u[c2_i151];
    }

    c2_b_threshold(chartInstance);
    c2_bb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_k_k = 1; c2_k_k < 4; c2_k_k++) {
      c2_l_k = c2_k_k;
      c2_bb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_l_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_l_k), 1, 3, 1, 0) - 1];
    }

    for (c2_i152 = 0; c2_i152 < 3; c2_i152++) {
      c2_q_y[c2_i152] = c2_ei[c2_i152];
    }

    for (c2_i153 = 0; c2_i153 < 3; c2_i153++) {
      c2_b[c2_i153] = c2_v0[c2_i153];
    }

    c2_b_threshold(chartInstance);
    c2_cb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_m_k = 1; c2_m_k < 4; c2_m_k++) {
      c2_n_k = c2_m_k;
      c2_cb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_n_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_n_k), 1, 3, 1, 0) - 1];
    }

    c2_d1 = c2_b_sat_atan(chartInstance, c2_bb_y);
    c2_j_y[0] = c2_d1;
    c2_j_y[1] = c2_cb_y;
    c2_b_threshold(chartInstance);
    c2_db_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_o_k = 1; c2_o_k < 3; c2_o_k++) {
      c2_p_k = c2_o_k;
      c2_db_y += c2_ab_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_p_k), 1, 2, 1, 0) - 1] *
        c2_j_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_p_k), 1, 2, 1, 0) - 1];
    }

    for (c2_i154 = 0; c2_i154 < 1000; c2_i154++) {
      c2_b_s[c2_i154] = c2_s[c2_i154];
    }

    c2_c_sat_atan(chartInstance, c2_b_s, c2_dv14);
    for (c2_i155 = 0; c2_i155 < 1000; c2_i155++) {
      c2_c_s[c2_i155] = c2_s[c2_i155];
    }

    for (c2_i156 = 0; c2_i156 < 1000; c2_i156++) {
      c2_dv15[c2_i156] = c2_dv14[c2_i156];
    }

    c2_bar_V0 = (c2_bar_V0 + c2_db_y) + c2_trapz(chartInstance, c2_c_s, c2_dv15);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 65);
    for (c2_i157 = 0; c2_i157 < 3; c2_i157++) {
      c2_q_y[c2_i157] = c2_ei[c2_i157];
    }

    for (c2_i158 = 0; c2_i158 < 3; c2_i158++) {
      c2_b[c2_i158] = c2_u[c2_i158];
    }

    c2_b_threshold(chartInstance);
    c2_eb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_q_k = 1; c2_q_k < 4; c2_q_k++) {
      c2_r_k = c2_q_k;
      c2_eb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_r_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_r_k), 1, 3, 1, 0) - 1];
    }

    c2_i_a = c2_sat_atan_dot(chartInstance, c2_eb_y) * c2_b_kp;
    for (c2_i159 = 0; c2_i159 < 3; c2_i159++) {
      c2_b[c2_i159] = c2_ei[c2_i159];
    }

    for (c2_i160 = 0; c2_i160 < 3; c2_i160++) {
      c2_b[c2_i160] *= c2_i_a;
    }

    for (c2_i161 = 0; c2_i161 < 3; c2_i161++) {
      c2_dv9[c2_i161] = 0.0;
    }

    for (c2_i162 = 0; c2_i162 < 3; c2_i162++) {
      c2_q_y[c2_i162] = c2_ei[c2_i162];
    }

    for (c2_i163 = 0; c2_i163 < 3; c2_i163++) {
      c2_b_b[c2_i163] = c2_u[c2_i163];
    }

    c2_b_threshold(chartInstance);
    c2_fb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_s_k = 1; c2_s_k < 4; c2_s_k++) {
      c2_t_k = c2_s_k;
      c2_fb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_t_k), 1, 3, 1, 0) - 1] *
        c2_b_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_t_k), 1, 3, 1, 0) - 1];
    }

    c2_j_a = c2_sat_atan_dot(chartInstance, c2_fb_y) * c2_b_kv;
    for (c2_i164 = 0; c2_i164 < 3; c2_i164++) {
      c2_b_b[c2_i164] = c2_ei[c2_i164];
    }

    for (c2_i165 = 0; c2_i165 < 3; c2_i165++) {
      c2_b_b[c2_i165] *= c2_j_a;
    }

    for (c2_i166 = 0; c2_i166 < 3; c2_i166++) {
      c2_k_a[c2_i166] = c2_b[c2_i166];
    }

    for (c2_i167 = 0; c2_i167 < 3; c2_i167++) {
      c2_k_a[c2_i167 + 6] = c2_dv9[c2_i167];
    }

    for (c2_i168 = 0; c2_i168 < 3; c2_i168++) {
      c2_k_a[c2_i168 + 3] = c2_b_b[c2_i168];
    }

    for (c2_i169 = 0; c2_i169 < 3; c2_i169++) {
      c2_k_a[c2_i169 + 9] = c2_ei[c2_i169];
    }

    for (c2_i170 = 0; c2_i170 < 4; c2_i170++) {
      c2_f_b[c2_i170] = c2_P[c2_i170];
    }

    c2_g_eml_scalar_eg(chartInstance);
    c2_g_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i171 = 0; c2_i171 < 6; c2_i171++) {
      c2_i172 = 0;
      c2_i173 = 0;
      for (c2_i174 = 0; c2_i174 < 2; c2_i174++) {
        c2_gb_y[c2_i172 + c2_i171] = 0.0;
        c2_i175 = 0;
        for (c2_i176 = 0; c2_i176 < 2; c2_i176++) {
          c2_gb_y[c2_i172 + c2_i171] += c2_k_a[c2_i175 + c2_i171] *
            c2_f_b[c2_i176 + c2_i173];
          c2_i175 += 6;
        }

        c2_i172 += 6;
        c2_i173 += 2;
      }
    }

    for (c2_i177 = 0; c2_i177 < 3; c2_i177++) {
      c2_q_y[c2_i177] = c2_ei[c2_i177];
    }

    for (c2_i178 = 0; c2_i178 < 3; c2_i178++) {
      c2_b[c2_i178] = c2_u[c2_i178];
    }

    c2_b_threshold(chartInstance);
    c2_hb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_u_k = 1; c2_u_k < 4; c2_u_k++) {
      c2_v_k = c2_u_k;
      c2_hb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_v_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_v_k), 1, 3, 1, 0) - 1];
    }

    for (c2_i179 = 0; c2_i179 < 3; c2_i179++) {
      c2_q_y[c2_i179] = c2_ei[c2_i179];
    }

    for (c2_i180 = 0; c2_i180 < 3; c2_i180++) {
      c2_b[c2_i180] = c2_v0[c2_i180];
    }

    c2_b_threshold(chartInstance);
    c2_ib_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_w_k = 1; c2_w_k < 4; c2_w_k++) {
      c2_x_k = c2_w_k;
      c2_ib_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_x_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_x_k), 1, 3, 1, 0) - 1];
    }

    c2_d2 = c2_b_sat_atan(chartInstance, c2_hb_y);
    c2_j_y[0] = c2_d2;
    c2_j_y[1] = c2_ib_y;
    c2_h_eml_scalar_eg(chartInstance);
    c2_h_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i181 = 0; c2_i181 < 6; c2_i181++) {
      c2_g_b[c2_i181] = 0.0;
      c2_i182 = 0;
      for (c2_i183 = 0; c2_i183 < 2; c2_i183++) {
        c2_g_b[c2_i181] += c2_gb_y[c2_i182 + c2_i181] * c2_j_y[c2_i183];
        c2_i182 += 6;
      }
    }

    for (c2_i184 = 0; c2_i184 < 3; c2_i184++) {
      c2_q_y[c2_i184] = c2_ei[c2_i184];
    }

    for (c2_i185 = 0; c2_i185 < 3; c2_i185++) {
      c2_b[c2_i185] = c2_u[c2_i185];
    }

    c2_b_threshold(chartInstance);
    c2_jb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_y_k = 1; c2_y_k < 4; c2_y_k++) {
      c2_ab_k = c2_y_k;
      c2_jb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_ab_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ab_k), 1, 3, 1, 0) - 1];
    }

    c2_l_a = c2_b_kp * c2_b_sat_atan(chartInstance, c2_jb_y);
    for (c2_i186 = 0; c2_i186 < 3; c2_i186++) {
      c2_b[c2_i186] = c2_ei[c2_i186];
    }

    for (c2_i187 = 0; c2_i187 < 3; c2_i187++) {
      c2_b[c2_i187] *= c2_l_a;
    }

    for (c2_i188 = 0; c2_i188 < 3; c2_i188++) {
      c2_q_y[c2_i188] = c2_ei[c2_i188];
    }

    for (c2_i189 = 0; c2_i189 < 3; c2_i189++) {
      c2_b_b[c2_i189] = c2_u[c2_i189];
    }

    c2_b_threshold(chartInstance);
    c2_kb_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_bb_k = 1; c2_bb_k < 4; c2_bb_k++) {
      c2_cb_k = c2_bb_k;
      c2_kb_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_cb_k), 1, 3, 1, 0) - 1] *
        c2_b_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_cb_k), 1, 3, 1, 0) - 1];
    }

    c2_m_a = c2_b_kv * c2_b_sat_atan(chartInstance, c2_kb_y);
    for (c2_i190 = 0; c2_i190 < 3; c2_i190++) {
      c2_b_b[c2_i190] = c2_ei[c2_i190];
    }

    for (c2_i191 = 0; c2_i191 < 3; c2_i191++) {
      c2_b_b[c2_i191] *= c2_m_a;
    }

    for (c2_i192 = 0; c2_i192 < 3; c2_i192++) {
      c2_h_b[c2_i192] = c2_b[c2_i192];
    }

    for (c2_i193 = 0; c2_i193 < 3; c2_i193++) {
      c2_h_b[c2_i193 + 3] = c2_b_b[c2_i193];
    }

    for (c2_i194 = 0; c2_i194 < 6; c2_i194++) {
      c2_grad_bar_V0[c2_i194] = (c2_grad_bar_V0[c2_i194] + c2_g_b[c2_i194]) +
        c2_h_b[c2_i194];
    }

    c2_d_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 70);
  c2_n_a = c2_b_kz;
  for (c2_i195 = 0; c2_i195 < 18; c2_i195++) {
    c2_lb_y[c2_i195] = c2_n_a * c2_i_b[c2_i195];
  }

  for (c2_i196 = 0; c2_i196 < 6; c2_i196++) {
    c2_g_b[c2_i196] = c2_grad_bar_V0[c2_i196];
  }

  c2_i_eml_scalar_eg(chartInstance);
  c2_i_eml_scalar_eg(chartInstance);
  for (c2_i197 = 0; c2_i197 < 3; c2_i197++) {
    c2_b_dot_z[c2_i197] = 0.0;
  }

  for (c2_i198 = 0; c2_i198 < 3; c2_i198++) {
    c2_b_dot_z[c2_i198] = 0.0;
  }

  for (c2_i199 = 0; c2_i199 < 3; c2_i199++) {
    c2_C[c2_i199] = c2_b_dot_z[c2_i199];
  }

  for (c2_i200 = 0; c2_i200 < 3; c2_i200++) {
    c2_b_dot_z[c2_i200] = c2_C[c2_i200];
  }

  c2_threshold(chartInstance);
  for (c2_i201 = 0; c2_i201 < 3; c2_i201++) {
    c2_C[c2_i201] = c2_b_dot_z[c2_i201];
  }

  for (c2_i202 = 0; c2_i202 < 3; c2_i202++) {
    c2_b_dot_z[c2_i202] = c2_C[c2_i202];
  }

  for (c2_i203 = 0; c2_i203 < 3; c2_i203++) {
    c2_b_dot_z[c2_i203] = 0.0;
    c2_i204 = 0;
    for (c2_i205 = 0; c2_i205 < 6; c2_i205++) {
      c2_b_dot_z[c2_i203] += c2_lb_y[c2_i204 + c2_i203] * c2_g_b[c2_i205];
      c2_i204 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 75);
  for (c2_i206 = 0; c2_i206 < 3; c2_i206++) {
    c2_b_b[c2_i206] = c2_gm[c2_i206];
  }

  for (c2_i207 = 0; c2_i207 < 3; c2_i207++) {
    c2_d_gm[c2_i207] = c2_gm[c2_i207];
  }

  c2_e_B = c2_norm(chartInstance, c2_d_gm);
  c2_mb_y = c2_e_B;
  c2_nb_y = c2_mb_y;
  c2_ob_y = c2_nb_y;
  for (c2_i208 = 0; c2_i208 < 3; c2_i208++) {
    c2_b_b[c2_i208] /= c2_ob_y;
  }

  for (c2_i209 = 0; c2_i209 < 3; c2_i209++) {
    c2_C[c2_i209] = c2_gm[c2_i209];
  }

  for (c2_i210 = 0; c2_i210 < 3; c2_i210++) {
    c2_e_gm[c2_i210] = c2_gm[c2_i210];
  }

  c2_f_B = c2_norm(chartInstance, c2_e_gm);
  c2_pb_y = c2_f_B;
  c2_qb_y = c2_pb_y;
  c2_rb_y = c2_qb_y;
  for (c2_i211 = 0; c2_i211 < 3; c2_i211++) {
    c2_C[c2_i211] /= c2_rb_y;
  }

  for (c2_i212 = 0; c2_i212 < 3; c2_i212++) {
    c2_j_b[c2_i212] = c2_b_b[c2_i212];
  }

  for (c2_i213 = 0; c2_i213 < 9; c2_i213++) {
    c2_l_b[c2_i213] = c2_k_b[c2_i213];
  }

  c2_kron(chartInstance, c2_j_b, c2_l_b, c2_sb_y);
  for (c2_i214 = 0; c2_i214 < 9; c2_i214++) {
    c2_m_b[c2_i214] = c2_k_b[c2_i214];
  }

  for (c2_i215 = 0; c2_i215 < 3; c2_i215++) {
    c2_c_C[c2_i215] = c2_C[c2_i215];
  }

  c2_b_kron(chartInstance, c2_m_b, c2_c_C, c2_tb_y);
  for (c2_i216 = 0; c2_i216 < 27; c2_i216++) {
    c2_H1[c2_i216] = c2_sb_y[c2_i216] + c2_tb_y[c2_i216];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 76);
  for (c2_i217 = 0; c2_i217 < 3; c2_i217++) {
    c2_b_b[c2_i217] = c2_gm[c2_i217];
  }

  for (c2_i218 = 0; c2_i218 < 3; c2_i218++) {
    c2_f_gm[c2_i218] = c2_gm[c2_i218];
  }

  c2_g_B = c2_norm(chartInstance, c2_f_gm);
  c2_ub_y = c2_g_B;
  c2_vb_y = c2_ub_y;
  c2_wb_y = c2_vb_y;
  for (c2_i219 = 0; c2_i219 < 3; c2_i219++) {
    c2_b_b[c2_i219] /= c2_wb_y;
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i220 = 0; c2_i220 < 3; c2_i220++) {
    c2_l_y[c2_i220] = 0.0;
    c2_i221 = 0;
    for (c2_i222 = 0; c2_i222 < 3; c2_i222++) {
      c2_l_y[c2_i220] += c2_get_Rd(chartInstance, c2_i221 + c2_i220) *
        c2_c_b[c2_i222];
      c2_i221 += 3;
    }
  }

  for (c2_i223 = 0; c2_i223 < 3; c2_i223++) {
    c2_n_b[c2_i223] = c2_b_b[c2_i223];
  }

  c2_S(chartInstance, c2_n_b, c2_dv10);
  for (c2_i224 = 0; c2_i224 < 9; c2_i224++) {
    c2_dv16[c2_i224] = c2_dv10[c2_i224];
  }

  c2_mpower(chartInstance, c2_dv16, c2_e_a);
  for (c2_i225 = 0; c2_i225 < 3; c2_i225++) {
    c2_xb_y[c2_i225] = c2_l_y[c2_i225];
  }

  c2_S(chartInstance, c2_xb_y, c2_d_b);
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i226 = 0; c2_i226 < 3; c2_i226++) {
    c2_i227 = 0;
    for (c2_i228 = 0; c2_i228 < 3; c2_i228++) {
      c2_yb_y[c2_i227 + c2_i226] = 0.0;
      c2_i229 = 0;
      for (c2_i230 = 0; c2_i230 < 3; c2_i230++) {
        c2_yb_y[c2_i227 + c2_i226] += c2_e_a[c2_i229 + c2_i226] * c2_d_b[c2_i230
          + c2_i227];
        c2_i229 += 3;
      }

      c2_i227 += 3;
    }
  }

  for (c2_i231 = 0; c2_i231 < 3; c2_i231++) {
    c2_g_gm[c2_i231] = c2_gm[c2_i231];
  }

  c2_h_B = c2_norm(chartInstance, c2_g_gm);
  c2_ac_y = c2_h_B;
  c2_bc_y = c2_ac_y;
  c2_cc_y = c2_bc_y;
  for (c2_i232 = 0; c2_i232 < 9; c2_i232++) {
    c2_H2[c2_i232] = c2_yb_y[c2_i232] / c2_cc_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 77);
  for (c2_i233 = 0; c2_i233 < 9; c2_i233++) {
    c2_i_hoistedGlobal[c2_i233] = c2_get_Rd(chartInstance, c2_i233);
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i234 = 0; c2_i234 < 3; c2_i234++) {
    c2_l_y[c2_i234] = 0.0;
    c2_i235 = 0;
    for (c2_i236 = 0; c2_i236 < 3; c2_i236++) {
      c2_l_y[c2_i234] += c2_get_Rd(chartInstance, c2_i235 + c2_i234) *
        c2_c_b[c2_i236];
      c2_i235 += 3;
    }
  }

  for (c2_i237 = 0; c2_i237 < 3; c2_i237++) {
    c2_dc_y[c2_i237] = c2_l_y[c2_i237];
  }

  c2_S(chartInstance, c2_dc_y, c2_d_b);
  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i238 = 0; c2_i238 < 9; c2_i238++) {
    c2_i239 = 0;
    c2_i240 = 0;
    for (c2_i241 = 0; c2_i241 < 3; c2_i241++) {
      c2_ec_y[c2_i239 + c2_i238] = 0.0;
      c2_i242 = 0;
      for (c2_i243 = 0; c2_i243 < 3; c2_i243++) {
        c2_ec_y[c2_i239 + c2_i238] += c2_o_a[c2_i242 + c2_i238] * c2_d_b[c2_i243
          + c2_i240];
        c2_i242 += 9;
      }

      c2_i239 += 9;
      c2_i240 += 3;
    }
  }

  for (c2_i244 = 0; c2_i244 < 3; c2_i244++) {
    c2_b_b[c2_i244] = c2_gm[c2_i244];
  }

  for (c2_i245 = 0; c2_i245 < 3; c2_i245++) {
    c2_h_gm[c2_i245] = c2_gm[c2_i245];
  }

  c2_i_B = c2_norm(chartInstance, c2_h_gm);
  c2_fc_y = c2_i_B;
  c2_gc_y = c2_fc_y;
  c2_hc_y = c2_gc_y;
  for (c2_i246 = 0; c2_i246 < 3; c2_i246++) {
    c2_b_b[c2_i246] /= c2_hc_y;
  }

  for (c2_i247 = 0; c2_i247 < 3; c2_i247++) {
    c2_o_b[c2_i247] = c2_b_b[c2_i247];
  }

  c2_S(chartInstance, c2_o_b, c2_dv10);
  for (c2_i248 = 0; c2_i248 < 9; c2_i248++) {
    c2_dv17[c2_i248] = c2_dv10[c2_i248];
  }

  c2_mpower(chartInstance, c2_dv17, c2_b_C);
  for (c2_i249 = 0; c2_i249 < 9; c2_i249++) {
    c2_d_C[c2_i249] = c2_b_C[c2_i249];
  }

  for (c2_i250 = 0; c2_i250 < 9; c2_i250++) {
    c2_p_b[c2_i250] = c2_k_b[c2_i250];
  }

  c2_c_kron(chartInstance, c2_d_C, c2_p_b, c2_p_a);
  c2_k_eml_scalar_eg(chartInstance);
  c2_k_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i251 = 0; c2_i251 < 9; c2_i251++) {
    c2_g_x[c2_i251] = 0.0;
    c2_i252 = 0;
    for (c2_i253 = 0; c2_i253 < 9; c2_i253++) {
      c2_g_x[c2_i251] += c2_p_a[c2_i252 + c2_i251] * c2_q_b[c2_i253];
      c2_i252 += 9;
    }
  }

  for (c2_i254 = 0; c2_i254 < 9; c2_i254++) {
    c2_i255 = 0;
    for (c2_i256 = 0; c2_i256 < 3; c2_i256++) {
      c2_sb_y[c2_i255 + c2_i254] = c2_g_x[c2_i254] * c2_f_a[c2_i256];
      c2_i255 += 9;
    }
  }

  c2_i257 = 0;
  for (c2_i258 = 0; c2_i258 < 3; c2_i258++) {
    c2_i259 = 0;
    for (c2_i260 = 0; c2_i260 < 3; c2_i260++) {
      c2_d_b[c2_i260 + c2_i257] = c2_get_Rd(chartInstance, c2_i259 + c2_i258);
      c2_i259 += 3;
    }

    c2_i257 += 3;
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i261 = 0; c2_i261 < 9; c2_i261++) {
    c2_i262 = 0;
    c2_i263 = 0;
    for (c2_i264 = 0; c2_i264 < 3; c2_i264++) {
      c2_tb_y[c2_i262 + c2_i261] = 0.0;
      c2_i265 = 0;
      for (c2_i266 = 0; c2_i266 < 3; c2_i266++) {
        c2_tb_y[c2_i262 + c2_i261] += c2_sb_y[c2_i265 + c2_i261] *
          c2_d_b[c2_i266 + c2_i263];
        c2_i265 += 9;
      }

      c2_i262 += 9;
      c2_i263 += 3;
    }
  }

  c2_i267 = 0;
  for (c2_i268 = 0; c2_i268 < 3; c2_i268++) {
    c2_i269 = 0;
    for (c2_i270 = 0; c2_i270 < 3; c2_i270++) {
      c2_d_b[c2_i270 + c2_i267] = c2_get_Rd(chartInstance, c2_i269 + c2_i268);
      c2_i269 += 3;
    }

    c2_i267 += 3;
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  c2_i271 = 0;
  for (c2_i272 = 0; c2_i272 < 3; c2_i272++) {
    c2_q_y[c2_i272] = 0.0;
    for (c2_i273 = 0; c2_i273 < 3; c2_i273++) {
      c2_q_y[c2_i272] += c2_f_a[c2_i273] * c2_d_b[c2_i273 + c2_i271];
    }

    c2_i271 += 3;
  }

  for (c2_i274 = 0; c2_i274 < 3; c2_i274++) {
    c2_b[c2_i274] = c2_r3[c2_i274];
  }

  c2_b_threshold(chartInstance);
  c2_ic_y = 0.0;
  c2_eml_switch_helper(chartInstance);
  for (c2_db_k = 1; c2_db_k < 4; c2_db_k++) {
    c2_eb_k = c2_db_k;
    c2_ic_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_eb_k), 1, 3, 1, 0) - 1] *
      c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_eb_k), 1, 3, 1, 0) - 1];
  }

  c2_q_a = 1.0 - c2_ic_y;
  for (c2_i275 = 0; c2_i275 < 27; c2_i275++) {
    c2_r_b[c2_i275] = c2_H1[c2_i275];
  }

  for (c2_i276 = 0; c2_i276 < 27; c2_i276++) {
    c2_r_b[c2_i276] *= c2_q_a;
  }

  for (c2_i277 = 0; c2_i277 < 9; c2_i277++) {
    c2_d_b[c2_i277] = c2_H2[c2_i277];
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i278 = 0; c2_i278 < 9; c2_i278++) {
    c2_i279 = 0;
    c2_i280 = 0;
    for (c2_i281 = 0; c2_i281 < 3; c2_i281++) {
      c2_sb_y[c2_i279 + c2_i278] = 0.0;
      c2_i282 = 0;
      for (c2_i283 = 0; c2_i283 < 3; c2_i283++) {
        c2_sb_y[c2_i279 + c2_i278] += c2_r_b[c2_i282 + c2_i278] * c2_d_b[c2_i283
          + c2_i280];
        c2_i282 += 9;
      }

      c2_i279 += 9;
      c2_i280 += 3;
    }
  }

  c2_i284 = 0;
  for (c2_i285 = 0; c2_i285 < 3; c2_i285++) {
    c2_i286 = 0;
    for (c2_i287 = 0; c2_i287 < 3; c2_i287++) {
      c2_j_hoistedGlobal[c2_i287 + c2_i284] = c2_i_hoistedGlobal[c2_i286 +
        c2_i285];
      c2_i286 += 3;
    }

    c2_i284 += 3;
  }

  for (c2_i288 = 0; c2_i288 < 9; c2_i288++) {
    c2_s_b[c2_i288] = c2_k_b[c2_i288];
  }

  c2_c_kron(chartInstance, c2_j_hoistedGlobal, c2_s_b, c2_p_a);
  for (c2_i289 = 0; c2_i289 < 27; c2_i289++) {
    c2_ec_y[c2_i289] = (c2_ec_y[c2_i289] - c2_tb_y[c2_i289]) - c2_sb_y[c2_i289];
  }

  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  for (c2_i290 = 0; c2_i290 < 27; c2_i290++) {
    c2_Dr3_R0[c2_i290] = 0.0;
  }

  for (c2_i291 = 0; c2_i291 < 27; c2_i291++) {
    c2_Dr3_R0[c2_i291] = 0.0;
  }

  for (c2_i292 = 0; c2_i292 < 27; c2_i292++) {
    c2_tb_y[c2_i292] = c2_Dr3_R0[c2_i292];
  }

  for (c2_i293 = 0; c2_i293 < 27; c2_i293++) {
    c2_Dr3_R0[c2_i293] = c2_tb_y[c2_i293];
  }

  c2_threshold(chartInstance);
  for (c2_i294 = 0; c2_i294 < 27; c2_i294++) {
    c2_tb_y[c2_i294] = c2_Dr3_R0[c2_i294];
  }

  for (c2_i295 = 0; c2_i295 < 27; c2_i295++) {
    c2_Dr3_R0[c2_i295] = c2_tb_y[c2_i295];
  }

  for (c2_i296 = 0; c2_i296 < 9; c2_i296++) {
    c2_i297 = 0;
    for (c2_i298 = 0; c2_i298 < 3; c2_i298++) {
      c2_Dr3_R0[c2_i297 + c2_i296] = 0.0;
      c2_i299 = 0;
      for (c2_i300 = 0; c2_i300 < 9; c2_i300++) {
        c2_Dr3_R0[c2_i297 + c2_i296] += c2_p_a[c2_i299 + c2_i296] *
          c2_ec_y[c2_i300 + c2_i297];
        c2_i299 += 9;
      }

      c2_i297 += 9;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 98);
  for (c2_i301 = 0; c2_i301 < 4; c2_i301++) {
    c2_q1[c2_i301] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 99);
  for (c2_i302 = 0; c2_i302 < 4; c2_i302++) {
    c2_e_u[c2_i302] = c2_b_q[c2_i302];
  }

  c2_jc_y = NULL;
  sf_mex_assign(&c2_jc_y, sf_mex_create("y", c2_e_u, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  for (c2_i303 = 0; c2_i303 < 4; c2_i303++) {
    c2_f_u[c2_i303] = c2_b_q0[c2_i303];
  }

  c2_kc_y = NULL;
  sf_mex_assign(&c2_kc_y, sf_mex_create("y", c2_f_u, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "transpose", 1U, 1U, 14,
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "quatmultiply", 1U, 2U, 14,
                      c2_jc_y, 14, sf_mex_call_debug(sfGlobalDebugInstanceStruct,
    "quatinv", 1U, 1U, 14, c2_kc_y))), "transpose", c2_dv18);
  for (c2_i304 = 0; c2_i304 < 4; c2_i304++) {
    c2_q1[c2_i304] = c2_dv18[c2_i304];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 105);
  c2_k_hoistedGlobal = c2_get_k(chartInstance, 0);
  c2_c_A = c2_b_h * c2_b_kz * c2_b_kV0;
  c2_j_B = c2_k_hoistedGlobal;
  c2_h_x = c2_c_A;
  c2_lc_y = c2_j_B;
  c2_i_x = c2_h_x;
  c2_mc_y = c2_lc_y;
  c2_j_x = c2_i_x;
  c2_nc_y = c2_mc_y;
  c2_oc_y = c2_j_x / c2_nc_y;
  c2_r_a = 2.0 * c2_q1[0];
  for (c2_i305 = 0; c2_i305 < 3; c2_i305++) {
    c2_d_mu[c2_i305] = c2_mu[c2_i305];
  }

  c2_S(chartInstance, c2_d_mu, c2_d_b);
  for (c2_i306 = 0; c2_i306 < 9; c2_i306++) {
    c2_d_b[c2_i306] *= c2_r_a;
  }

  for (c2_i307 = 0; c2_i307 < 3; c2_i307++) {
    c2_e_mu[c2_i307] = c2_mu[c2_i307];
  }

  c2_S(chartInstance, c2_e_mu, c2_i_hoistedGlobal);
  for (c2_i308 = 0; c2_i308 < 9; c2_i308++) {
    c2_i_hoistedGlobal[c2_i308] *= 2.0;
  }

  for (c2_i309 = 0; c2_i309 < 3; c2_i309++) {
    c2_b_q1[c2_i309] = c2_q1[c2_i309 + 1];
  }

  c2_S(chartInstance, c2_b_q1, c2_b_C);
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i310 = 0; c2_i310 < 3; c2_i310++) {
    c2_i311 = 0;
    for (c2_i312 = 0; c2_i312 < 3; c2_i312++) {
      c2_yb_y[c2_i311 + c2_i310] = 0.0;
      c2_i313 = 0;
      for (c2_i314 = 0; c2_i314 < 3; c2_i314++) {
        c2_yb_y[c2_i311 + c2_i310] += c2_i_hoistedGlobal[c2_i313 + c2_i310] *
          c2_b_C[c2_i314 + c2_i311];
        c2_i313 += 3;
      }

      c2_i311 += 3;
    }
  }

  c2_s_a = -c2_oc_y;
  for (c2_i315 = 0; c2_i315 < 9; c2_i315++) {
    c2_d_b[c2_i315] -= c2_yb_y[c2_i315];
  }

  for (c2_i316 = 0; c2_i316 < 9; c2_i316++) {
    c2_d_b[c2_i316] *= c2_s_a;
  }

  c2_m_eml_scalar_eg(chartInstance);
  c2_m_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i317 = 0; c2_i317 < 3; c2_i317++) {
    c2_i318 = 0;
    for (c2_i319 = 0; c2_i319 < 6; c2_i319++) {
      c2_lb_y[c2_i318 + c2_i317] = 0.0;
      c2_i320 = 0;
      for (c2_i321 = 0; c2_i321 < 3; c2_i321++) {
        c2_lb_y[c2_i318 + c2_i317] += c2_d_b[c2_i320 + c2_i317] * c2_i_b[c2_i321
          + c2_i318];
        c2_i320 += 3;
      }

      c2_i318 += 3;
    }
  }

  for (c2_i322 = 0; c2_i322 < 6; c2_i322++) {
    c2_g_b[c2_i322] = c2_grad_bar_V0[c2_i322];
  }

  c2_i_eml_scalar_eg(chartInstance);
  c2_i_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i323 = 0; c2_i323 < 3; c2_i323++) {
    c2_l_y[c2_i323] = 0.0;
    c2_i324 = 0;
    for (c2_i325 = 0; c2_i325 < 6; c2_i325++) {
      c2_l_y[c2_i323] += c2_lb_y[c2_i324 + c2_i323] * c2_g_b[c2_i325];
      c2_i324 += 3;
    }
  }

  c2_t_a = c2_b_h * c2_b_kq;
  for (c2_i326 = 0; c2_i326 < 3; c2_i326++) {
    c2_b[c2_i326] = c2_q1[c2_i326 + 1];
  }

  for (c2_i327 = 0; c2_i327 < 3; c2_i327++) {
    c2_b[c2_i327] *= c2_t_a;
  }

  for (c2_i328 = 0; c2_i328 < 3; c2_i328++) {
    c2_w1[c2_i328] = c2_l_y[c2_i328] - c2_b[c2_i328];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 107);
  for (c2_i329 = 0; c2_i329 < 9; c2_i329++) {
    c2_e_a[c2_i329] = -c2_R[c2_i329];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i330 = 0; c2_i330 < 3; c2_i330++) {
    c2_l_y[c2_i330] = 0.0;
    c2_i331 = 0;
    for (c2_i332 = 0; c2_i332 < 3; c2_i332++) {
      c2_l_y[c2_i330] += c2_e_a[c2_i331 + c2_i330] * c2_c_b[c2_i332];
      c2_i331 += 3;
    }
  }

  c2_t_b = c2_T;
  for (c2_i333 = 0; c2_i333 < 3; c2_i333++) {
    c2_l_y[c2_i333] *= c2_t_b;
  }

  c2_l_hoistedGlobal = c2_get_m(chartInstance, 0);
  c2_k_B = c2_l_hoistedGlobal;
  c2_pc_y = c2_k_B;
  c2_qc_y = c2_pc_y;
  c2_rc_y = c2_qc_y;
  for (c2_i334 = 0; c2_i334 < 3; c2_i334++) {
    c2_l_y[c2_i334] /= c2_rc_y;
  }

  c2_m_hoistedGlobal = c2_get_g(chartInstance, 0);
  c2_u_a = c2_m_hoistedGlobal;
  for (c2_i335 = 0; c2_i335 < 3; c2_i335++) {
    c2_sc_y[c2_i335] = c2_u_a * c2_c_b[c2_i335];
  }

  for (c2_i336 = 0; c2_i336 < 3; c2_i336++) {
    c2_hat_dot_v[c2_i336] = (c2_l_y[c2_i336] + c2_sc_y[c2_i336]) +
      c2_b_hat_b[c2_i336];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 108);
  c2_v_a = c2_b_kp;
  for (c2_i337 = 0; c2_i337 < 3; c2_i337++) {
    c2_b[c2_i337] = c2_p0[c2_i337];
  }

  for (c2_i338 = 0; c2_i338 < 3; c2_i338++) {
    c2_b[c2_i338] *= c2_v_a;
  }

  c2_w_a = c2_b_kv;
  for (c2_i339 = 0; c2_i339 < 3; c2_i339++) {
    c2_b_b[c2_i339] = c2_v0[c2_i339];
  }

  for (c2_i340 = 0; c2_i340 < 3; c2_i340++) {
    c2_b_b[c2_i340] *= c2_w_a;
  }

  c2_x_a = c2_b_kp;
  for (c2_i341 = 0; c2_i341 < 3; c2_i341++) {
    c2_C[c2_i341] = c2_v0[c2_i341];
  }

  for (c2_i342 = 0; c2_i342 < 3; c2_i342++) {
    c2_C[c2_i342] *= c2_x_a;
  }

  c2_y_a = c2_b_kv;
  for (c2_i343 = 0; c2_i343 < 3; c2_i343++) {
    c2_sc_y[c2_i343] = c2_hat_dot_v[c2_i343] - c2_dot_vd[c2_i343];
  }

  for (c2_i344 = 0; c2_i344 < 3; c2_i344++) {
    c2_sc_y[c2_i344] *= c2_y_a;
  }

  for (c2_i345 = 0; c2_i345 < 3; c2_i345++) {
    c2_u_b[c2_i345] = c2_b[c2_i345] + c2_b_b[c2_i345];
  }

  c2_b_sat_atan_dot(chartInstance, c2_u_b, c2_dv9);
  for (c2_i346 = 0; c2_i346 < 3; c2_i346++) {
    c2_d_z[c2_i346] = c2_b_z[c2_i346];
  }

  c2_b_sat_atan_dot(chartInstance, c2_d_z, c2_l_y);
  for (c2_i347 = 0; c2_i347 < 3; c2_i347++) {
    c2_hat_dot_mu[c2_i347] = (-c2_dv9[c2_i347] * (c2_C[c2_i347] +
      c2_sc_y[c2_i347]) + c2_ddot_vd[c2_i347]) - c2_l_y[c2_i347] *
      c2_b_dot_z[c2_i347];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 109);
  for (c2_i348 = 0; c2_i348 < 3; c2_i348++) {
    c2_b_r3[c2_i348] = c2_r3[c2_i348];
  }

  c2_S(chartInstance, c2_b_r3, c2_dv10);
  for (c2_i349 = 0; c2_i349 < 9; c2_i349++) {
    c2_dv19[c2_i349] = c2_dv10[c2_i349];
  }

  c2_mpower(chartInstance, c2_dv19, c2_e_a);
  for (c2_i350 = 0; c2_i350 < 3; c2_i350++) {
    c2_b[c2_i350] = c2_hat_dot_mu[c2_i350];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i351 = 0; c2_i351 < 3; c2_i351++) {
    c2_l_y[c2_i351] = 0.0;
    c2_i352 = 0;
    for (c2_i353 = 0; c2_i353 < 3; c2_i353++) {
      c2_l_y[c2_i351] += c2_e_a[c2_i352 + c2_i351] * c2_b[c2_i353];
      c2_i352 += 3;
    }
  }

  for (c2_i354 = 0; c2_i354 < 3; c2_i354++) {
    c2_f_mu[c2_i354] = c2_mu[c2_i354];
  }

  c2_l_B = c2_norm(chartInstance, c2_f_mu);
  c2_tc_y = c2_l_B;
  c2_uc_y = c2_tc_y;
  c2_vc_y = c2_uc_y;
  for (c2_i355 = 0; c2_i355 < 3; c2_i355++) {
    c2_hat_dot_r3[c2_i355] = c2_l_y[c2_i355] / c2_vc_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 110);
  c2_i356 = 0;
  for (c2_i357 = 0; c2_i357 < 3; c2_i357++) {
    c2_i358 = 0;
    for (c2_i359 = 0; c2_i359 < 3; c2_i359++) {
      c2_d_b[c2_i359 + c2_i356] = c2_R0[c2_i358 + c2_i357];
      c2_i358 += 3;
    }

    c2_i356 += 3;
  }

  for (c2_i360 = 0; c2_i360 < 9; c2_i360++) {
    c2_d_b[c2_i360] *= -0.5;
  }

  for (c2_i361 = 0; c2_i361 < 9; c2_i361++) {
    c2_b_R0[c2_i361] = c2_R0[c2_i361];
  }

  c2_Gamma(chartInstance, c2_b_R0, c2_sb_y);
  c2_i362 = 0;
  for (c2_i363 = 0; c2_i363 < 9; c2_i363++) {
    c2_i364 = 0;
    for (c2_i365 = 0; c2_i365 < 3; c2_i365++) {
      c2_v_b[c2_i365 + c2_i362] = c2_sb_y[c2_i364 + c2_i363];
      c2_i364 += 9;
    }

    c2_i362 += 3;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i366 = 0; c2_i366 < 3; c2_i366++) {
    c2_i367 = 0;
    for (c2_i368 = 0; c2_i368 < 9; c2_i368++) {
      c2_wc_y[c2_i367 + c2_i366] = 0.0;
      c2_i369 = 0;
      for (c2_i370 = 0; c2_i370 < 3; c2_i370++) {
        c2_wc_y[c2_i367 + c2_i366] += c2_d_b[c2_i369 + c2_i366] * c2_v_b[c2_i370
          + c2_i367];
        c2_i369 += 3;
      }

      c2_i367 += 3;
    }
  }

  for (c2_i371 = 0; c2_i371 < 27; c2_i371++) {
    c2_r_b[c2_i371] = c2_Dr3_R0[c2_i371];
  }

  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i372 = 0; c2_i372 < 3; c2_i372++) {
    c2_i373 = 0;
    c2_i374 = 0;
    for (c2_i375 = 0; c2_i375 < 3; c2_i375++) {
      c2_yb_y[c2_i373 + c2_i372] = 0.0;
      c2_i376 = 0;
      for (c2_i377 = 0; c2_i377 < 9; c2_i377++) {
        c2_yb_y[c2_i373 + c2_i372] += c2_wc_y[c2_i376 + c2_i372] *
          c2_r_b[c2_i377 + c2_i374];
        c2_i376 += 3;
      }

      c2_i373 += 3;
      c2_i374 += 9;
    }
  }

  for (c2_i378 = 0; c2_i378 < 3; c2_i378++) {
    c2_b[c2_i378] = c2_hat_dot_r3[c2_i378];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i379 = 0; c2_i379 < 3; c2_i379++) {
    c2_hat_w0[c2_i379] = 0.0;
  }

  for (c2_i380 = 0; c2_i380 < 3; c2_i380++) {
    c2_hat_w0[c2_i380] = 0.0;
  }

  for (c2_i381 = 0; c2_i381 < 3; c2_i381++) {
    c2_C[c2_i381] = c2_hat_w0[c2_i381];
  }

  for (c2_i382 = 0; c2_i382 < 3; c2_i382++) {
    c2_hat_w0[c2_i382] = c2_C[c2_i382];
  }

  c2_threshold(chartInstance);
  for (c2_i383 = 0; c2_i383 < 3; c2_i383++) {
    c2_C[c2_i383] = c2_hat_w0[c2_i383];
  }

  for (c2_i384 = 0; c2_i384 < 3; c2_i384++) {
    c2_hat_w0[c2_i384] = c2_C[c2_i384];
  }

  for (c2_i385 = 0; c2_i385 < 3; c2_i385++) {
    c2_hat_w0[c2_i385] = 0.0;
    c2_i386 = 0;
    for (c2_i387 = 0; c2_i387 < 3; c2_i387++) {
      c2_hat_w0[c2_i385] += c2_yb_y[c2_i386 + c2_i385] * c2_b[c2_i387];
      c2_i386 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 112);
  c2_i388 = 0;
  for (c2_i389 = 0; c2_i389 < 3; c2_i389++) {
    c2_i390 = 0;
    for (c2_i391 = 0; c2_i391 < 3; c2_i391++) {
      c2_e_a[c2_i391 + c2_i388] = c2_R0[c2_i390 + c2_i389];
      c2_i390 += 3;
    }

    c2_i388 += 3;
  }

  for (c2_i392 = 0; c2_i392 < 3; c2_i392++) {
    c2_b[c2_i392] = c2_w1[c2_i392];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i393 = 0; c2_i393 < 3; c2_i393++) {
    c2_l_y[c2_i393] = 0.0;
    c2_i394 = 0;
    for (c2_i395 = 0; c2_i395 < 3; c2_i395++) {
      c2_l_y[c2_i393] += c2_e_a[c2_i394 + c2_i393] * c2_b[c2_i395];
      c2_i394 += 3;
    }
  }

  for (c2_i396 = 0; c2_i396 < 3; c2_i396++) {
    c2_w_star[c2_i396] = c2_hat_w0[c2_i396] + c2_l_y[c2_i396];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 122);
  for (c2_i397 = 0; c2_i397 < 3; c2_i397++) {
    c2_b_dot_hat_b[c2_i397] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 128U);
  for (c2_i398 = 0; c2_i398 < 9; c2_i398++) {
    c2_e_a[c2_i398] = -c2_R[c2_i398];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i399 = 0; c2_i399 < 3; c2_i399++) {
    c2_l_y[c2_i399] = 0.0;
    c2_i400 = 0;
    for (c2_i401 = 0; c2_i401 < 3; c2_i401++) {
      c2_l_y[c2_i399] += c2_e_a[c2_i400 + c2_i399] * c2_c_b[c2_i401];
      c2_i400 += 3;
    }
  }

  c2_w_b = c2_T;
  for (c2_i402 = 0; c2_i402 < 3; c2_i402++) {
    c2_l_y[c2_i402] *= c2_w_b;
  }

  c2_n_hoistedGlobal = c2_get_m(chartInstance, 0);
  c2_m_B = c2_n_hoistedGlobal;
  c2_xc_y = c2_m_B;
  c2_yc_y = c2_xc_y;
  c2_ad_y = c2_yc_y;
  for (c2_i403 = 0; c2_i403 < 3; c2_i403++) {
    c2_l_y[c2_i403] /= c2_ad_y;
  }

  c2_o_hoistedGlobal = c2_get_g(chartInstance, 0);
  c2_ab_a = c2_o_hoistedGlobal;
  for (c2_i404 = 0; c2_i404 < 3; c2_i404++) {
    c2_sc_y[c2_i404] = c2_ab_a * c2_c_b[c2_i404];
  }

  for (c2_i405 = 0; c2_i405 < 3; c2_i405++) {
    c2_hat_dot_v2[c2_i405] = (c2_l_y[c2_i405] + c2_sc_y[c2_i405]) +
      c2_b_hat_b2[c2_i405];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 129U);
  c2_bb_a = c2_b_kp;
  for (c2_i406 = 0; c2_i406 < 3; c2_i406++) {
    c2_b[c2_i406] = c2_p0[c2_i406];
  }

  for (c2_i407 = 0; c2_i407 < 3; c2_i407++) {
    c2_b[c2_i407] *= c2_bb_a;
  }

  c2_cb_a = c2_b_kv;
  for (c2_i408 = 0; c2_i408 < 3; c2_i408++) {
    c2_b_b[c2_i408] = c2_v0[c2_i408];
  }

  for (c2_i409 = 0; c2_i409 < 3; c2_i409++) {
    c2_b_b[c2_i409] *= c2_cb_a;
  }

  c2_db_a = c2_b_kp;
  for (c2_i410 = 0; c2_i410 < 3; c2_i410++) {
    c2_C[c2_i410] = c2_v0[c2_i410];
  }

  for (c2_i411 = 0; c2_i411 < 3; c2_i411++) {
    c2_C[c2_i411] *= c2_db_a;
  }

  c2_eb_a = c2_b_kv;
  for (c2_i412 = 0; c2_i412 < 3; c2_i412++) {
    c2_sc_y[c2_i412] = c2_hat_dot_v2[c2_i412] - c2_dot_vd[c2_i412];
  }

  for (c2_i413 = 0; c2_i413 < 3; c2_i413++) {
    c2_sc_y[c2_i413] *= c2_eb_a;
  }

  for (c2_i414 = 0; c2_i414 < 3; c2_i414++) {
    c2_x_b[c2_i414] = c2_b[c2_i414] + c2_b_b[c2_i414];
  }

  c2_b_sat_atan_dot(chartInstance, c2_x_b, c2_dv9);
  for (c2_i415 = 0; c2_i415 < 3; c2_i415++) {
    c2_e_z[c2_i415] = c2_b_z[c2_i415];
  }

  c2_b_sat_atan_dot(chartInstance, c2_e_z, c2_l_y);
  for (c2_i416 = 0; c2_i416 < 3; c2_i416++) {
    c2_hat_dot_mu2[c2_i416] = (-c2_dv9[c2_i416] * (c2_C[c2_i416] +
      c2_sc_y[c2_i416]) + c2_ddot_vd[c2_i416]) - c2_l_y[c2_i416] *
      c2_b_dot_z[c2_i416];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 130U);
  for (c2_i417 = 0; c2_i417 < 3; c2_i417++) {
    c2_c_r3[c2_i417] = c2_r3[c2_i417];
  }

  c2_S(chartInstance, c2_c_r3, c2_dv10);
  for (c2_i418 = 0; c2_i418 < 9; c2_i418++) {
    c2_dv20[c2_i418] = c2_dv10[c2_i418];
  }

  c2_mpower(chartInstance, c2_dv20, c2_e_a);
  for (c2_i419 = 0; c2_i419 < 3; c2_i419++) {
    c2_b[c2_i419] = c2_hat_dot_mu2[c2_i419];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i420 = 0; c2_i420 < 3; c2_i420++) {
    c2_l_y[c2_i420] = 0.0;
    c2_i421 = 0;
    for (c2_i422 = 0; c2_i422 < 3; c2_i422++) {
      c2_l_y[c2_i420] += c2_e_a[c2_i421 + c2_i420] * c2_b[c2_i422];
      c2_i421 += 3;
    }
  }

  for (c2_i423 = 0; c2_i423 < 3; c2_i423++) {
    c2_g_mu[c2_i423] = c2_mu[c2_i423];
  }

  c2_n_B = c2_norm(chartInstance, c2_g_mu);
  c2_bd_y = c2_n_B;
  c2_cd_y = c2_bd_y;
  c2_dd_y = c2_cd_y;
  for (c2_i424 = 0; c2_i424 < 3; c2_i424++) {
    c2_hat_dot_r32[c2_i424] = c2_l_y[c2_i424] / c2_dd_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 131U);
  c2_i425 = 0;
  for (c2_i426 = 0; c2_i426 < 3; c2_i426++) {
    c2_i427 = 0;
    for (c2_i428 = 0; c2_i428 < 3; c2_i428++) {
      c2_d_b[c2_i428 + c2_i425] = c2_R0[c2_i427 + c2_i426];
      c2_i427 += 3;
    }

    c2_i425 += 3;
  }

  for (c2_i429 = 0; c2_i429 < 9; c2_i429++) {
    c2_d_b[c2_i429] *= -0.5;
  }

  for (c2_i430 = 0; c2_i430 < 9; c2_i430++) {
    c2_c_R0[c2_i430] = c2_R0[c2_i430];
  }

  c2_Gamma(chartInstance, c2_c_R0, c2_sb_y);
  c2_i431 = 0;
  for (c2_i432 = 0; c2_i432 < 9; c2_i432++) {
    c2_i433 = 0;
    for (c2_i434 = 0; c2_i434 < 3; c2_i434++) {
      c2_v_b[c2_i434 + c2_i431] = c2_sb_y[c2_i433 + c2_i432];
      c2_i433 += 9;
    }

    c2_i431 += 3;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i435 = 0; c2_i435 < 3; c2_i435++) {
    c2_i436 = 0;
    for (c2_i437 = 0; c2_i437 < 9; c2_i437++) {
      c2_wc_y[c2_i436 + c2_i435] = 0.0;
      c2_i438 = 0;
      for (c2_i439 = 0; c2_i439 < 3; c2_i439++) {
        c2_wc_y[c2_i436 + c2_i435] += c2_d_b[c2_i438 + c2_i435] * c2_v_b[c2_i439
          + c2_i436];
        c2_i438 += 3;
      }

      c2_i436 += 3;
    }
  }

  for (c2_i440 = 0; c2_i440 < 27; c2_i440++) {
    c2_r_b[c2_i440] = c2_Dr3_R0[c2_i440];
  }

  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i441 = 0; c2_i441 < 3; c2_i441++) {
    c2_i442 = 0;
    c2_i443 = 0;
    for (c2_i444 = 0; c2_i444 < 3; c2_i444++) {
      c2_yb_y[c2_i442 + c2_i441] = 0.0;
      c2_i445 = 0;
      for (c2_i446 = 0; c2_i446 < 9; c2_i446++) {
        c2_yb_y[c2_i442 + c2_i441] += c2_wc_y[c2_i445 + c2_i441] *
          c2_r_b[c2_i446 + c2_i443];
        c2_i445 += 3;
      }

      c2_i442 += 3;
      c2_i443 += 9;
    }
  }

  for (c2_i447 = 0; c2_i447 < 3; c2_i447++) {
    c2_b[c2_i447] = c2_hat_dot_r32[c2_i447];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i448 = 0; c2_i448 < 3; c2_i448++) {
    c2_hat_w02[c2_i448] = 0.0;
  }

  for (c2_i449 = 0; c2_i449 < 3; c2_i449++) {
    c2_hat_w02[c2_i449] = 0.0;
  }

  for (c2_i450 = 0; c2_i450 < 3; c2_i450++) {
    c2_C[c2_i450] = c2_hat_w02[c2_i450];
  }

  for (c2_i451 = 0; c2_i451 < 3; c2_i451++) {
    c2_hat_w02[c2_i451] = c2_C[c2_i451];
  }

  c2_threshold(chartInstance);
  for (c2_i452 = 0; c2_i452 < 3; c2_i452++) {
    c2_C[c2_i452] = c2_hat_w02[c2_i452];
  }

  for (c2_i453 = 0; c2_i453 < 3; c2_i453++) {
    c2_hat_w02[c2_i453] = c2_C[c2_i453];
  }

  for (c2_i454 = 0; c2_i454 < 3; c2_i454++) {
    c2_hat_w02[c2_i454] = 0.0;
    c2_i455 = 0;
    for (c2_i456 = 0; c2_i456 < 3; c2_i456++) {
      c2_hat_w02[c2_i454] += c2_yb_y[c2_i455 + c2_i454] * c2_b[c2_i456];
      c2_i455 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 135U);
  for (c2_i457 = 0; c2_i457 < 9; c2_i457++) {
    c2_e_a[c2_i457] = -c2_R[c2_i457];
  }

  for (c2_i458 = 0; c2_i458 < 3; c2_i458++) {
    c2_c_w[c2_i458] = c2_b_w[c2_i458];
  }

  c2_S(chartInstance, c2_c_w, c2_d_b);
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i459 = 0; c2_i459 < 3; c2_i459++) {
    c2_i460 = 0;
    for (c2_i461 = 0; c2_i461 < 3; c2_i461++) {
      c2_yb_y[c2_i460 + c2_i459] = 0.0;
      c2_i462 = 0;
      for (c2_i463 = 0; c2_i463 < 3; c2_i463++) {
        c2_yb_y[c2_i460 + c2_i459] += c2_e_a[c2_i462 + c2_i459] * c2_d_b[c2_i463
          + c2_i460];
        c2_i462 += 3;
      }

      c2_i460 += 3;
    }
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i464 = 0; c2_i464 < 3; c2_i464++) {
    c2_l_y[c2_i464] = 0.0;
    c2_i465 = 0;
    for (c2_i466 = 0; c2_i466 < 3; c2_i466++) {
      c2_l_y[c2_i464] += c2_yb_y[c2_i465 + c2_i464] * c2_c_b[c2_i466];
      c2_i465 += 3;
    }
  }

  c2_y_b = c2_T;
  for (c2_i467 = 0; c2_i467 < 3; c2_i467++) {
    c2_l_y[c2_i467] *= c2_y_b;
  }

  c2_p_hoistedGlobal = c2_get_m(chartInstance, 0);
  c2_o_B = c2_p_hoistedGlobal;
  c2_ed_y = c2_o_B;
  c2_fd_y = c2_ed_y;
  c2_gd_y = c2_fd_y;
  for (c2_i468 = 0; c2_i468 < 3; c2_i468++) {
    c2_l_y[c2_i468] /= c2_gd_y;
  }

  for (c2_i469 = 0; c2_i469 < 9; c2_i469++) {
    c2_e_a[c2_i469] = c2_R[c2_i469];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i470 = 0; c2_i470 < 3; c2_i470++) {
    c2_sc_y[c2_i470] = 0.0;
    c2_i471 = 0;
    for (c2_i472 = 0; c2_i472 < 3; c2_i472++) {
      c2_sc_y[c2_i470] += c2_e_a[c2_i471 + c2_i470] * c2_c_b[c2_i472];
      c2_i471 += 3;
    }
  }

  for (c2_i473 = 0; c2_i473 < 3; c2_i473++) {
    c2_q_y[c2_i473] = c2_mu[c2_i473];
  }

  for (c2_i474 = 0; c2_i474 < 3; c2_i474++) {
    c2_i475 = 0;
    for (c2_i476 = 0; c2_i476 < 3; c2_i476++) {
      c2_yb_y[c2_i475 + c2_i474] = c2_sc_y[c2_i474] * c2_q_y[c2_i476];
      c2_i475 += 3;
    }
  }

  for (c2_i477 = 0; c2_i477 < 3; c2_i477++) {
    c2_b[c2_i477] = c2_hat_dot_mu2[c2_i477];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i478 = 0; c2_i478 < 3; c2_i478++) {
    c2_sc_y[c2_i478] = 0.0;
    c2_i479 = 0;
    for (c2_i480 = 0; c2_i480 < 3; c2_i480++) {
      c2_sc_y[c2_i478] += c2_yb_y[c2_i479 + c2_i478] * c2_b[c2_i480];
      c2_i479 += 3;
    }
  }

  for (c2_i481 = 0; c2_i481 < 3; c2_i481++) {
    c2_h_mu[c2_i481] = c2_mu[c2_i481];
  }

  c2_p_B = c2_norm(chartInstance, c2_h_mu);
  c2_hd_y = c2_p_B;
  c2_id_y = c2_hd_y;
  c2_jd_y = c2_id_y;
  for (c2_i482 = 0; c2_i482 < 3; c2_i482++) {
    c2_sc_y[c2_i482] /= c2_jd_y;
  }

  for (c2_i483 = 0; c2_i483 < 3; c2_i483++) {
    c2_hat_dot_hat_dot_v2[c2_i483] = (c2_l_y[c2_i483] - c2_sc_y[c2_i483]) +
      c2_b_dot_hat_b[c2_i483];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 139U);
  c2_fb_a = c2_b_kp;
  for (c2_i484 = 0; c2_i484 < 3; c2_i484++) {
    c2_b[c2_i484] = c2_v0[c2_i484];
  }

  for (c2_i485 = 0; c2_i485 < 3; c2_i485++) {
    c2_b[c2_i485] *= c2_fb_a;
  }

  c2_gb_a = c2_b_kv;
  for (c2_i486 = 0; c2_i486 < 3; c2_i486++) {
    c2_b_b[c2_i486] = c2_hat_dot_v[c2_i486] - c2_dot_vd[c2_i486];
  }

  for (c2_i487 = 0; c2_i487 < 3; c2_i487++) {
    c2_b_b[c2_i487] *= c2_gb_a;
  }

  for (c2_i488 = 0; c2_i488 < 3; c2_i488++) {
    c2_hat_dot_u[c2_i488] = c2_b[c2_i488] + c2_b_b[c2_i488];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 140U);
  c2_hb_a = c2_b_kp;
  for (c2_i489 = 0; c2_i489 < 3; c2_i489++) {
    c2_b[c2_i489] = c2_v0[c2_i489];
  }

  for (c2_i490 = 0; c2_i490 < 3; c2_i490++) {
    c2_b[c2_i490] *= c2_hb_a;
  }

  c2_ib_a = c2_b_kv;
  for (c2_i491 = 0; c2_i491 < 3; c2_i491++) {
    c2_b_b[c2_i491] = c2_hat_dot_v2[c2_i491] - c2_dot_vd[c2_i491];
  }

  for (c2_i492 = 0; c2_i492 < 3; c2_i492++) {
    c2_b_b[c2_i492] *= c2_ib_a;
  }

  for (c2_i493 = 0; c2_i493 < 3; c2_i493++) {
    c2_hat_dot_u2[c2_i493] = c2_b[c2_i493] + c2_b_b[c2_i493];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 143U);
  c2_jb_a = c2_b_kp;
  for (c2_i494 = 0; c2_i494 < 3; c2_i494++) {
    c2_b[c2_i494] = c2_hat_dot_v2[c2_i494] - c2_dot_vd[c2_i494];
  }

  for (c2_i495 = 0; c2_i495 < 3; c2_i495++) {
    c2_b[c2_i495] *= c2_jb_a;
  }

  c2_kb_a = c2_b_kv;
  for (c2_i496 = 0; c2_i496 < 3; c2_i496++) {
    c2_b_b[c2_i496] = c2_hat_dot_hat_dot_v2[c2_i496] - c2_ddot_vd[c2_i496];
  }

  for (c2_i497 = 0; c2_i497 < 3; c2_i497++) {
    c2_b_b[c2_i497] *= c2_kb_a;
  }

  for (c2_i498 = 0; c2_i498 < 3; c2_i498++) {
    c2_hat_dot_hat_dot_u2[c2_i498] = c2_b[c2_i498] + c2_b_b[c2_i498];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 145U);
  for (c2_i499 = 0; c2_i499 < 36; c2_i499++) {
    c2_hess_bar_V0[c2_i499] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 146U);
  c2_I = 1.0;
  c2_e_I = 0;
  while (c2_e_I < 3) {
    c2_I = 1.0 + (real_T)c2_e_I;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 147U);
    for (c2_i500 = 0; c2_i500 < 3; c2_i500++) {
      c2_ei[c2_i500] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 148U);
    c2_ei[_SFD_EML_ARRAY_BOUNDS_CHECK("ei", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 3, 1, 0) - 1] = 1.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 149U);
    for (c2_i501 = 0; c2_i501 < 3; c2_i501++) {
      c2_q_y[c2_i501] = c2_ei[c2_i501];
    }

    for (c2_i502 = 0; c2_i502 < 3; c2_i502++) {
      c2_b[c2_i502] = c2_u[c2_i502];
    }

    c2_b_threshold(chartInstance);
    c2_kd_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_fb_k = 1; c2_fb_k < 4; c2_fb_k++) {
      c2_gb_k = c2_fb_k;
      c2_kd_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_gb_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_gb_k), 1, 3, 1, 0) - 1];
    }

    c2_lb_a = c2_sat_atan_dot(chartInstance, c2_kd_y) * c2_b_kp;
    for (c2_i503 = 0; c2_i503 < 3; c2_i503++) {
      c2_b[c2_i503] = c2_ei[c2_i503];
    }

    for (c2_i504 = 0; c2_i504 < 3; c2_i504++) {
      c2_b[c2_i504] *= c2_lb_a;
    }

    for (c2_i505 = 0; c2_i505 < 3; c2_i505++) {
      c2_dv9[c2_i505] = 0.0;
    }

    for (c2_i506 = 0; c2_i506 < 3; c2_i506++) {
      c2_q_y[c2_i506] = c2_ei[c2_i506];
    }

    for (c2_i507 = 0; c2_i507 < 3; c2_i507++) {
      c2_b_b[c2_i507] = c2_u[c2_i507];
    }

    c2_b_threshold(chartInstance);
    c2_ld_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_hb_k = 1; c2_hb_k < 4; c2_hb_k++) {
      c2_ib_k = c2_hb_k;
      c2_ld_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_ib_k), 1, 3, 1, 0) - 1] *
        c2_b_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ib_k), 1, 3, 1, 0) - 1];
    }

    c2_mb_a = c2_sat_atan_dot(chartInstance, c2_ld_y) * c2_b_kv;
    for (c2_i508 = 0; c2_i508 < 3; c2_i508++) {
      c2_b_b[c2_i508] = c2_ei[c2_i508];
    }

    for (c2_i509 = 0; c2_i509 < 3; c2_i509++) {
      c2_b_b[c2_i509] *= c2_mb_a;
    }

    for (c2_i510 = 0; c2_i510 < 3; c2_i510++) {
      c2_q_y[c2_i510] = c2_ei[c2_i510];
    }

    for (c2_i511 = 0; c2_i511 < 3; c2_i511++) {
      c2_C[c2_i511] = c2_u[c2_i511];
    }

    c2_b_threshold(chartInstance);
    c2_md_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_jb_k = 1; c2_jb_k < 4; c2_jb_k++) {
      c2_kb_k = c2_jb_k;
      c2_md_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_kb_k), 1, 3, 1, 0) - 1] *
        c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_kb_k), 1, 3, 1, 0) - 1];
    }

    for (c2_i512 = 0; c2_i512 < 3; c2_i512++) {
      c2_q_y[c2_i512] = c2_ei[c2_i512];
    }

    for (c2_i513 = 0; c2_i513 < 3; c2_i513++) {
      c2_C[c2_i513] = c2_v0[c2_i513];
    }

    c2_b_threshold(chartInstance);
    c2_nd_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_lb_k = 1; c2_lb_k < 4; c2_lb_k++) {
      c2_mb_k = c2_lb_k;
      c2_nd_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_mb_k), 1, 3, 1, 0) - 1] *
        c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_mb_k), 1, 3, 1, 0) - 1];
    }

    c2_d3 = c2_b_sat_atan(chartInstance, c2_md_y);
    c2_h_a[0] = c2_d3;
    c2_h_a[1] = c2_nd_y;
    for (c2_i514 = 0; c2_i514 < 4; c2_i514++) {
      c2_f_b[c2_i514] = c2_P[c2_i514];
    }

    c2_f_eml_scalar_eg(chartInstance);
    c2_f_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    c2_i515 = 0;
    for (c2_i516 = 0; c2_i516 < 2; c2_i516++) {
      c2_ab_y[c2_i516] = 0.0;
      for (c2_i517 = 0; c2_i517 < 2; c2_i517++) {
        c2_ab_y[c2_i516] += c2_h_a[c2_i517] * c2_f_b[c2_i517 + c2_i515];
      }

      c2_i515 += 2;
    }

    for (c2_i518 = 0; c2_i518 < 3; c2_i518++) {
      c2_q_y[c2_i518] = c2_ei[c2_i518];
    }

    for (c2_i519 = 0; c2_i519 < 3; c2_i519++) {
      c2_C[c2_i519] = c2_u[c2_i519];
    }

    c2_b_threshold(chartInstance);
    c2_od_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_nb_k = 1; c2_nb_k < 4; c2_nb_k++) {
      c2_ob_k = c2_nb_k;
      c2_od_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_ob_k), 1, 3, 1, 0) - 1] *
        c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ob_k), 1, 3, 1, 0) - 1];
    }

    for (c2_i520 = 0; c2_i520 < 2; c2_i520++) {
      c2_pd_y[c2_i520] = c2_ab_y[c2_i520];
    }

    for (c2_i521 = 0; c2_i521 < 36; c2_i521++) {
      c2_dv22[c2_i521] = c2_dv21[c2_i521];
    }

    c2_d_kron(chartInstance, c2_pd_y, c2_dv22, c2_nb_a);
    c2_ab_b = c2_sat_atan_ddot(chartInstance, c2_od_y);
    for (c2_i522 = 0; c2_i522 < 72; c2_i522++) {
      c2_nb_a[c2_i522] *= c2_ab_b;
    }

    for (c2_i523 = 0; c2_i523 < 3; c2_i523++) {
      c2_C[c2_i523] = c2_ei[c2_i523];
    }

    for (c2_i524 = 0; c2_i524 < 3; c2_i524++) {
      c2_q_y[c2_i524] = c2_ei[c2_i524];
    }

    for (c2_i525 = 0; c2_i525 < 3; c2_i525++) {
      c2_i526 = 0;
      for (c2_i527 = 0; c2_i527 < 3; c2_i527++) {
        c2_yb_y[c2_i526 + c2_i525] = c2_C[c2_i525] * c2_q_y[c2_i527];
        c2_i526 += 3;
      }
    }

    c2_ob_a = c2_b_kp;
    for (c2_i528 = 0; c2_i528 < 9; c2_i528++) {
      c2_yb_y[c2_i528] *= c2_ob_a;
    }

    c2_pb_a = c2_b_kp;
    for (c2_i529 = 0; c2_i529 < 9; c2_i529++) {
      c2_i_hoistedGlobal[c2_i529] = c2_pb_a * c2_k_b[c2_i529];
    }

    c2_qb_a = c2_b_kv;
    for (c2_i530 = 0; c2_i530 < 9; c2_i530++) {
      c2_b_C[c2_i530] = c2_qb_a * c2_k_b[c2_i530];
    }

    c2_i531 = 0;
    for (c2_i532 = 0; c2_i532 < 3; c2_i532++) {
      for (c2_i533 = 0; c2_i533 < 3; c2_i533++) {
        c2_bb_b[c2_i533 + c2_i531] = c2_i_hoistedGlobal[c2_i533 + c2_i531];
      }

      c2_i531 += 3;
    }

    c2_i534 = 0;
    for (c2_i535 = 0; c2_i535 < 3; c2_i535++) {
      for (c2_i536 = 0; c2_i536 < 3; c2_i536++) {
        c2_bb_b[(c2_i536 + c2_i534) + 9] = c2_b_C[c2_i536 + c2_i534];
      }

      c2_i534 += 3;
    }

    c2_m_eml_scalar_eg(chartInstance);
    c2_m_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i537 = 0; c2_i537 < 3; c2_i537++) {
      c2_i538 = 0;
      for (c2_i539 = 0; c2_i539 < 6; c2_i539++) {
        c2_lb_y[c2_i538 + c2_i537] = 0.0;
        c2_i540 = 0;
        for (c2_i541 = 0; c2_i541 < 3; c2_i541++) {
          c2_lb_y[c2_i538 + c2_i537] += c2_yb_y[c2_i540 + c2_i537] *
            c2_bb_b[c2_i541 + c2_i538];
          c2_i540 += 3;
        }

        c2_i538 += 3;
      }
    }

    for (c2_i542 = 0; c2_i542 < 3; c2_i542++) {
      c2_C[c2_i542] = c2_ei[c2_i542];
    }

    for (c2_i543 = 0; c2_i543 < 3; c2_i543++) {
      c2_q_y[c2_i543] = c2_ei[c2_i543];
    }

    for (c2_i544 = 0; c2_i544 < 3; c2_i544++) {
      c2_i545 = 0;
      for (c2_i546 = 0; c2_i546 < 3; c2_i546++) {
        c2_yb_y[c2_i545 + c2_i544] = c2_C[c2_i544] * c2_q_y[c2_i546];
        c2_i545 += 3;
      }
    }

    c2_rb_a = c2_b_kv;
    for (c2_i547 = 0; c2_i547 < 9; c2_i547++) {
      c2_yb_y[c2_i547] *= c2_rb_a;
    }

    c2_sb_a = c2_b_kp;
    for (c2_i548 = 0; c2_i548 < 9; c2_i548++) {
      c2_i_hoistedGlobal[c2_i548] = c2_sb_a * c2_k_b[c2_i548];
    }

    c2_tb_a = c2_b_kv;
    for (c2_i549 = 0; c2_i549 < 9; c2_i549++) {
      c2_b_C[c2_i549] = c2_tb_a * c2_k_b[c2_i549];
    }

    c2_i550 = 0;
    for (c2_i551 = 0; c2_i551 < 3; c2_i551++) {
      for (c2_i552 = 0; c2_i552 < 3; c2_i552++) {
        c2_bb_b[c2_i552 + c2_i550] = c2_i_hoistedGlobal[c2_i552 + c2_i550];
      }

      c2_i550 += 3;
    }

    c2_i553 = 0;
    for (c2_i554 = 0; c2_i554 < 3; c2_i554++) {
      for (c2_i555 = 0; c2_i555 < 3; c2_i555++) {
        c2_bb_b[(c2_i555 + c2_i553) + 9] = c2_b_C[c2_i555 + c2_i553];
      }

      c2_i553 += 3;
    }

    c2_m_eml_scalar_eg(chartInstance);
    c2_m_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i556 = 0; c2_i556 < 3; c2_i556++) {
      c2_i557 = 0;
      for (c2_i558 = 0; c2_i558 < 6; c2_i558++) {
        c2_qd_y[c2_i557 + c2_i556] = 0.0;
        c2_i559 = 0;
        for (c2_i560 = 0; c2_i560 < 3; c2_i560++) {
          c2_qd_y[c2_i557 + c2_i556] += c2_yb_y[c2_i559 + c2_i556] *
            c2_bb_b[c2_i560 + c2_i557];
          c2_i559 += 3;
        }

        c2_i557 += 3;
      }
    }

    c2_i561 = 0;
    c2_i562 = 0;
    for (c2_i563 = 0; c2_i563 < 6; c2_i563++) {
      for (c2_i564 = 0; c2_i564 < 3; c2_i564++) {
        c2_cb_b[c2_i564 + c2_i561] = c2_lb_y[c2_i564 + c2_i562];
      }

      c2_i561 += 12;
      c2_i562 += 3;
    }

    c2_i565 = 0;
    c2_i566 = 0;
    for (c2_i567 = 0; c2_i567 < 6; c2_i567++) {
      for (c2_i568 = 0; c2_i568 < 3; c2_i568++) {
        c2_cb_b[(c2_i568 + c2_i565) + 3] = c2_qd_y[c2_i568 + c2_i566];
      }

      c2_i565 += 12;
      c2_i566 += 3;
    }

    c2_i569 = 0;
    for (c2_i570 = 0; c2_i570 < 6; c2_i570++) {
      for (c2_i571 = 0; c2_i571 < 6; c2_i571++) {
        c2_cb_b[(c2_i571 + c2_i569) + 6] = 0.0;
      }

      c2_i569 += 12;
    }

    c2_p_eml_scalar_eg(chartInstance);
    c2_p_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i572 = 0; c2_i572 < 6; c2_i572++) {
      c2_i573 = 0;
      c2_i574 = 0;
      for (c2_i575 = 0; c2_i575 < 6; c2_i575++) {
        c2_db_b[c2_i573 + c2_i572] = 0.0;
        c2_i576 = 0;
        for (c2_i577 = 0; c2_i577 < 12; c2_i577++) {
          c2_db_b[c2_i573 + c2_i572] += c2_nb_a[c2_i576 + c2_i572] *
            c2_cb_b[c2_i577 + c2_i574];
          c2_i576 += 6;
        }

        c2_i573 += 6;
        c2_i574 += 12;
      }
    }

    for (c2_i578 = 0; c2_i578 < 3; c2_i578++) {
      c2_q_y[c2_i578] = c2_ei[c2_i578];
    }

    for (c2_i579 = 0; c2_i579 < 3; c2_i579++) {
      c2_C[c2_i579] = c2_u[c2_i579];
    }

    c2_b_threshold(chartInstance);
    c2_rd_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_pb_k = 1; c2_pb_k < 4; c2_pb_k++) {
      c2_qb_k = c2_pb_k;
      c2_rd_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_qb_k), 1, 3, 1, 0) - 1] *
        c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_qb_k), 1, 3, 1, 0) - 1];
    }

    c2_ub_a = c2_b_kp;
    for (c2_i580 = 0; c2_i580 < 9; c2_i580++) {
      c2_yb_y[c2_i580] = c2_ub_a * c2_k_b[c2_i580];
    }

    c2_vb_a = c2_b_kv;
    for (c2_i581 = 0; c2_i581 < 9; c2_i581++) {
      c2_i_hoistedGlobal[c2_i581] = c2_vb_a * c2_k_b[c2_i581];
    }

    c2_wb_a = c2_sat_atan_dot(chartInstance, c2_rd_y);
    c2_i582 = 0;
    c2_i583 = 0;
    for (c2_i584 = 0; c2_i584 < 3; c2_i584++) {
      for (c2_i585 = 0; c2_i585 < 3; c2_i585++) {
        c2_eb_b[c2_i585 + c2_i582] = c2_yb_y[c2_i585 + c2_i583];
      }

      c2_i582 += 6;
      c2_i583 += 3;
    }

    c2_i586 = 0;
    c2_i587 = 0;
    for (c2_i588 = 0; c2_i588 < 3; c2_i588++) {
      for (c2_i589 = 0; c2_i589 < 3; c2_i589++) {
        c2_eb_b[(c2_i589 + c2_i586) + 3] = c2_i_hoistedGlobal[c2_i589 + c2_i587];
      }

      c2_i586 += 6;
      c2_i587 += 3;
    }

    for (c2_i590 = 0; c2_i590 < 18; c2_i590++) {
      c2_eb_b[c2_i590] *= c2_wb_a;
    }

    for (c2_i591 = 0; c2_i591 < 3; c2_i591++) {
      c2_C[c2_i591] = c2_ei[c2_i591];
    }

    for (c2_i592 = 0; c2_i592 < 3; c2_i592++) {
      c2_q_y[c2_i592] = c2_ei[c2_i592];
    }

    for (c2_i593 = 0; c2_i593 < 3; c2_i593++) {
      c2_i594 = 0;
      for (c2_i595 = 0; c2_i595 < 3; c2_i595++) {
        c2_yb_y[c2_i594 + c2_i593] = c2_C[c2_i593] * c2_q_y[c2_i595];
        c2_i594 += 3;
      }
    }

    c2_q_eml_scalar_eg(chartInstance);
    c2_q_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i596 = 0; c2_i596 < 6; c2_i596++) {
      c2_i597 = 0;
      c2_i598 = 0;
      for (c2_i599 = 0; c2_i599 < 3; c2_i599++) {
        c2_sd_y[c2_i597 + c2_i596] = 0.0;
        c2_i600 = 0;
        for (c2_i601 = 0; c2_i601 < 3; c2_i601++) {
          c2_sd_y[c2_i597 + c2_i596] += c2_eb_b[c2_i600 + c2_i596] *
            c2_yb_y[c2_i601 + c2_i598];
          c2_i600 += 6;
        }

        c2_i597 += 6;
        c2_i598 += 3;
      }
    }

    c2_xb_a = c2_b_kp;
    for (c2_i602 = 0; c2_i602 < 9; c2_i602++) {
      c2_yb_y[c2_i602] = c2_xb_a * c2_k_b[c2_i602];
    }

    c2_yb_a = c2_b_kv;
    for (c2_i603 = 0; c2_i603 < 9; c2_i603++) {
      c2_i_hoistedGlobal[c2_i603] = c2_yb_a * c2_k_b[c2_i603];
    }

    c2_i604 = 0;
    for (c2_i605 = 0; c2_i605 < 3; c2_i605++) {
      for (c2_i606 = 0; c2_i606 < 3; c2_i606++) {
        c2_bb_b[c2_i606 + c2_i604] = c2_yb_y[c2_i606 + c2_i604];
      }

      c2_i604 += 3;
    }

    c2_i607 = 0;
    for (c2_i608 = 0; c2_i608 < 3; c2_i608++) {
      for (c2_i609 = 0; c2_i609 < 3; c2_i609++) {
        c2_bb_b[(c2_i609 + c2_i607) + 9] = c2_i_hoistedGlobal[c2_i609 + c2_i607];
      }

      c2_i607 += 3;
    }

    c2_r_eml_scalar_eg(chartInstance);
    c2_r_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i610 = 0; c2_i610 < 6; c2_i610++) {
      c2_i611 = 0;
      c2_i612 = 0;
      for (c2_i613 = 0; c2_i613 < 6; c2_i613++) {
        c2_td_y[c2_i611 + c2_i610] = 0.0;
        c2_i614 = 0;
        for (c2_i615 = 0; c2_i615 < 3; c2_i615++) {
          c2_td_y[c2_i611 + c2_i610] += c2_sd_y[c2_i614 + c2_i610] *
            c2_bb_b[c2_i615 + c2_i612];
          c2_i614 += 6;
        }

        c2_i611 += 6;
        c2_i612 += 3;
      }
    }

    for (c2_i616 = 0; c2_i616 < 3; c2_i616++) {
      c2_k_a[c2_i616] = c2_b[c2_i616];
    }

    for (c2_i617 = 0; c2_i617 < 3; c2_i617++) {
      c2_k_a[c2_i617 + 6] = c2_dv9[c2_i617];
    }

    for (c2_i618 = 0; c2_i618 < 3; c2_i618++) {
      c2_k_a[c2_i618 + 3] = c2_b_b[c2_i618];
    }

    for (c2_i619 = 0; c2_i619 < 3; c2_i619++) {
      c2_k_a[c2_i619 + 9] = c2_ei[c2_i619];
    }

    for (c2_i620 = 0; c2_i620 < 4; c2_i620++) {
      c2_f_b[c2_i620] = c2_P[c2_i620];
    }

    c2_g_eml_scalar_eg(chartInstance);
    c2_g_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i621 = 0; c2_i621 < 6; c2_i621++) {
      c2_i622 = 0;
      c2_i623 = 0;
      for (c2_i624 = 0; c2_i624 < 2; c2_i624++) {
        c2_gb_y[c2_i622 + c2_i621] = 0.0;
        c2_i625 = 0;
        for (c2_i626 = 0; c2_i626 < 2; c2_i626++) {
          c2_gb_y[c2_i622 + c2_i621] += c2_k_a[c2_i625 + c2_i621] *
            c2_f_b[c2_i626 + c2_i623];
          c2_i625 += 6;
        }

        c2_i622 += 6;
        c2_i623 += 2;
      }
    }

    for (c2_i627 = 0; c2_i627 < 3; c2_i627++) {
      c2_q_y[c2_i627] = c2_ei[c2_i627];
    }

    for (c2_i628 = 0; c2_i628 < 3; c2_i628++) {
      c2_b[c2_i628] = c2_u[c2_i628];
    }

    c2_b_threshold(chartInstance);
    c2_ud_y = 0.0;
    c2_eml_switch_helper(chartInstance);
    for (c2_rb_k = 1; c2_rb_k < 4; c2_rb_k++) {
      c2_sb_k = c2_rb_k;
      c2_ud_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_sb_k), 1, 3, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_sb_k), 1, 3, 1, 0) - 1];
    }

    c2_ac_a = c2_sat_atan_dot(chartInstance, c2_ud_y);
    for (c2_i629 = 0; c2_i629 < 3; c2_i629++) {
      c2_q_y[c2_i629] = c2_ei[c2_i629];
    }

    for (c2_i630 = 0; c2_i630 < 3; c2_i630++) {
      c2_q_y[c2_i630] *= c2_ac_a;
    }

    c2_bc_a = c2_b_kp;
    for (c2_i631 = 0; c2_i631 < 9; c2_i631++) {
      c2_yb_y[c2_i631] = c2_bc_a * c2_k_b[c2_i631];
    }

    c2_cc_a = c2_b_kv;
    for (c2_i632 = 0; c2_i632 < 9; c2_i632++) {
      c2_i_hoistedGlobal[c2_i632] = c2_cc_a * c2_k_b[c2_i632];
    }

    c2_i633 = 0;
    for (c2_i634 = 0; c2_i634 < 3; c2_i634++) {
      for (c2_i635 = 0; c2_i635 < 3; c2_i635++) {
        c2_bb_b[c2_i635 + c2_i633] = c2_yb_y[c2_i635 + c2_i633];
      }

      c2_i633 += 3;
    }

    c2_i636 = 0;
    for (c2_i637 = 0; c2_i637 < 3; c2_i637++) {
      for (c2_i638 = 0; c2_i638 < 3; c2_i638++) {
        c2_bb_b[(c2_i638 + c2_i636) + 9] = c2_i_hoistedGlobal[c2_i638 + c2_i636];
      }

      c2_i636 += 3;
    }

    c2_s_eml_scalar_eg(chartInstance);
    c2_s_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    c2_i639 = 0;
    for (c2_i640 = 0; c2_i640 < 6; c2_i640++) {
      c2_vd_y[c2_i640] = 0.0;
      for (c2_i641 = 0; c2_i641 < 3; c2_i641++) {
        c2_vd_y[c2_i640] += c2_q_y[c2_i641] * c2_bb_b[c2_i641 + c2_i639];
      }

      c2_i639 += 3;
    }

    c2_i642 = 0;
    for (c2_i643 = 0; c2_i643 < 6; c2_i643++) {
      c2_fb_b[c2_i642] = c2_vd_y[c2_i643];
      c2_i642 += 2;
    }

    c2_i644 = 0;
    for (c2_i645 = 0; c2_i645 < 3; c2_i645++) {
      c2_fb_b[c2_i644 + 1] = 0.0;
      c2_i644 += 2;
    }

    c2_i646 = 0;
    for (c2_i647 = 0; c2_i647 < 3; c2_i647++) {
      c2_fb_b[c2_i646 + 7] = c2_ei[c2_i647];
      c2_i646 += 2;
    }

    c2_t_eml_scalar_eg(chartInstance);
    c2_t_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i648 = 0; c2_i648 < 6; c2_i648++) {
      c2_i649 = 0;
      c2_i650 = 0;
      for (c2_i651 = 0; c2_i651 < 6; c2_i651++) {
        c2_wd_y[c2_i649 + c2_i648] = 0.0;
        c2_i652 = 0;
        for (c2_i653 = 0; c2_i653 < 2; c2_i653++) {
          c2_wd_y[c2_i649 + c2_i648] += c2_gb_y[c2_i652 + c2_i648] *
            c2_fb_b[c2_i653 + c2_i650];
          c2_i652 += 6;
        }

        c2_i649 += 6;
        c2_i650 += 2;
      }
    }

    for (c2_i654 = 0; c2_i654 < 36; c2_i654++) {
      c2_hess_bar_V0[c2_i654] = ((c2_hess_bar_V0[c2_i654] + c2_db_b[c2_i654]) +
        c2_td_y[c2_i654]) + c2_wd_y[c2_i654];
    }

    c2_e_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 163U);
  c2_dc_a = c2_b_kz;
  for (c2_i655 = 0; c2_i655 < 18; c2_i655++) {
    c2_lb_y[c2_i655] = c2_dc_a * c2_i_b[c2_i655];
  }

  for (c2_i656 = 0; c2_i656 < 36; c2_i656++) {
    c2_db_b[c2_i656] = c2_hess_bar_V0[c2_i656];
  }

  c2_u_eml_scalar_eg(chartInstance);
  c2_u_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i657 = 0; c2_i657 < 3; c2_i657++) {
    c2_i658 = 0;
    c2_i659 = 0;
    for (c2_i660 = 0; c2_i660 < 6; c2_i660++) {
      c2_qd_y[c2_i658 + c2_i657] = 0.0;
      c2_i661 = 0;
      for (c2_i662 = 0; c2_i662 < 6; c2_i662++) {
        c2_qd_y[c2_i658 + c2_i657] += c2_lb_y[c2_i661 + c2_i657] *
          c2_db_b[c2_i662 + c2_i659];
        c2_i661 += 3;
      }

      c2_i658 += 3;
      c2_i659 += 6;
    }
  }

  for (c2_i663 = 0; c2_i663 < 3; c2_i663++) {
    c2_g_b[c2_i663] = c2_v0[c2_i663];
  }

  for (c2_i664 = 0; c2_i664 < 3; c2_i664++) {
    c2_g_b[c2_i664 + 3] = c2_hat_dot_v2[c2_i664] - c2_dot_vd[c2_i664];
  }

  c2_i_eml_scalar_eg(chartInstance);
  c2_i_eml_scalar_eg(chartInstance);
  for (c2_i665 = 0; c2_i665 < 3; c2_i665++) {
    c2_hat_ddot_z2[c2_i665] = 0.0;
  }

  for (c2_i666 = 0; c2_i666 < 3; c2_i666++) {
    c2_hat_ddot_z2[c2_i666] = 0.0;
  }

  for (c2_i667 = 0; c2_i667 < 3; c2_i667++) {
    c2_C[c2_i667] = c2_hat_ddot_z2[c2_i667];
  }

  for (c2_i668 = 0; c2_i668 < 3; c2_i668++) {
    c2_hat_ddot_z2[c2_i668] = c2_C[c2_i668];
  }

  c2_threshold(chartInstance);
  for (c2_i669 = 0; c2_i669 < 3; c2_i669++) {
    c2_C[c2_i669] = c2_hat_ddot_z2[c2_i669];
  }

  for (c2_i670 = 0; c2_i670 < 3; c2_i670++) {
    c2_hat_ddot_z2[c2_i670] = c2_C[c2_i670];
  }

  for (c2_i671 = 0; c2_i671 < 3; c2_i671++) {
    c2_hat_ddot_z2[c2_i671] = 0.0;
    c2_i672 = 0;
    for (c2_i673 = 0; c2_i673 < 6; c2_i673++) {
      c2_hat_ddot_z2[c2_i671] += c2_qd_y[c2_i672 + c2_i671] * c2_g_b[c2_i673];
      c2_i672 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 165U);
  for (c2_i674 = 0; c2_i674 < 3; c2_i674++) {
    c2_g_u[c2_i674] = c2_u[c2_i674];
  }

  c2_b_sat_atan_dot(chartInstance, c2_g_u, c2_dv9);
  for (c2_i675 = 0; c2_i675 < 3; c2_i675++) {
    c2_f_z[c2_i675] = c2_b_z[c2_i675];
  }

  c2_b_sat_atan_dot(chartInstance, c2_f_z, c2_l_y);
  for (c2_i676 = 0; c2_i676 < 3; c2_i676++) {
    c2_hat_dot_mu[c2_i676] = (-c2_dv9[c2_i676] * c2_hat_dot_u[c2_i676] +
      c2_ddot_vd[c2_i676]) - c2_l_y[c2_i676] * c2_b_dot_z[c2_i676];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 166U);
  for (c2_i677 = 0; c2_i677 < 3; c2_i677++) {
    c2_h_u[c2_i677] = c2_u[c2_i677];
  }

  c2_b_sat_atan_ddot(chartInstance, c2_h_u, c2_dv9);
  for (c2_i678 = 0; c2_i678 < 3; c2_i678++) {
    c2_i_u[c2_i678] = c2_u[c2_i678];
  }

  c2_b_sat_atan_dot(chartInstance, c2_i_u, c2_l_y);
  for (c2_i679 = 0; c2_i679 < 3; c2_i679++) {
    c2_g_z[c2_i679] = c2_b_z[c2_i679];
  }

  c2_b_sat_atan_ddot(chartInstance, c2_g_z, c2_C);
  for (c2_i680 = 0; c2_i680 < 3; c2_i680++) {
    c2_c_dot_z[c2_i680] = c2_b_dot_z[c2_i680];
  }

  c2_power(chartInstance, c2_c_dot_z, c2_sc_y);
  for (c2_i681 = 0; c2_i681 < 3; c2_i681++) {
    c2_h_z[c2_i681] = c2_b_z[c2_i681];
  }

  c2_b_sat_atan_dot(chartInstance, c2_h_z, c2_b_b);
  for (c2_i682 = 0; c2_i682 < 3; c2_i682++) {
    c2_hat_dot_hat_dot_mu2[c2_i682] = (((-c2_dv9[c2_i682] * c2_hat_dot_u[c2_i682]
      * c2_hat_dot_u2[c2_i682] + c2_d3dot_vd[c2_i682]) - c2_l_y[c2_i682] *
      c2_hat_dot_hat_dot_u2[c2_i682]) - c2_C[c2_i682] * c2_sc_y[c2_i682]) -
      c2_b_b[c2_i682] * c2_hat_ddot_z2[c2_i682];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 172U);
  for (c2_i683 = 0; c2_i683 < 3; c2_i683++) {
    c2_d_r3[c2_i683] = c2_r3[c2_i683];
  }

  c2_S(chartInstance, c2_d_r3, c2_dv10);
  for (c2_i684 = 0; c2_i684 < 9; c2_i684++) {
    c2_dv23[c2_i684] = c2_dv10[c2_i684];
  }

  c2_mpower(chartInstance, c2_dv23, c2_e_a);
  for (c2_i685 = 0; c2_i685 < 3; c2_i685++) {
    c2_b[c2_i685] = c2_hat_dot_mu[c2_i685];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i686 = 0; c2_i686 < 3; c2_i686++) {
    c2_l_y[c2_i686] = 0.0;
    c2_i687 = 0;
    for (c2_i688 = 0; c2_i688 < 3; c2_i688++) {
      c2_l_y[c2_i686] += c2_e_a[c2_i687 + c2_i686] * c2_b[c2_i688];
      c2_i687 += 3;
    }
  }

  for (c2_i689 = 0; c2_i689 < 3; c2_i689++) {
    c2_i_mu[c2_i689] = c2_mu[c2_i689];
  }

  c2_q_B = c2_norm(chartInstance, c2_i_mu);
  c2_xd_y = c2_q_B;
  c2_yd_y = c2_xd_y;
  c2_ae_y = c2_yd_y;
  for (c2_i690 = 0; c2_i690 < 3; c2_i690++) {
    c2_hat_dot_r3[c2_i690] = c2_l_y[c2_i690] / c2_ae_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 173U);
  for (c2_i691 = 0; c2_i691 < 3; c2_i691++) {
    c2_q_y[c2_i691] = c2_hat_dot_mu[c2_i691];
  }

  for (c2_i692 = 0; c2_i692 < 3; c2_i692++) {
    c2_j_mu[c2_i692] = c2_mu[c2_i692];
  }

  c2_r_B = c2_norm(chartInstance, c2_j_mu);
  c2_be_y = c2_r_B;
  c2_ce_y = c2_be_y;
  c2_de_y = c2_ce_y;
  for (c2_i693 = 0; c2_i693 < 3; c2_i693++) {
    c2_q_y[c2_i693] /= c2_de_y;
  }

  for (c2_i694 = 0; c2_i694 < 3; c2_i694++) {
    c2_ee_y[c2_i694] = c2_q_y[c2_i694];
  }

  for (c2_i695 = 0; c2_i695 < 9; c2_i695++) {
    c2_gb_b[c2_i695] = c2_k_b[c2_i695];
  }

  c2_e_kron(chartInstance, c2_ee_y, c2_gb_b, c2_v_b);
  for (c2_i696 = 0; c2_i696 < 27; c2_i696++) {
    c2_v_b[c2_i696] = -c2_v_b[c2_i696];
  }

  for (c2_i697 = 0; c2_i697 < 3; c2_i697++) {
    c2_e_r3[c2_i697] = c2_r3[c2_i697];
  }

  c2_S(chartInstance, c2_e_r3, c2_dv10);
  c2_i698 = 0;
  for (c2_i699 = 0; c2_i699 < 3; c2_i699++) {
    c2_i700 = 0;
    for (c2_i701 = 0; c2_i701 < 3; c2_i701++) {
      c2_dv24[c2_i701 + c2_i698] = c2_dv10[c2_i700 + c2_i699];
      c2_i700 += 3;
    }

    c2_i698 += 3;
  }

  for (c2_i702 = 0; c2_i702 < 9; c2_i702++) {
    c2_hb_b[c2_i702] = c2_k_b[c2_i702];
  }

  c2_c_kron(chartInstance, c2_dv24, c2_hb_b, c2_ib_b);
  for (c2_i703 = 0; c2_i703 < 3; c2_i703++) {
    c2_f_r3[c2_i703] = c2_r3[c2_i703];
  }

  c2_S(chartInstance, c2_f_r3, c2_dv10);
  for (c2_i704 = 0; c2_i704 < 9; c2_i704++) {
    c2_jb_b[c2_i704] = c2_k_b[c2_i704];
  }

  for (c2_i705 = 0; c2_i705 < 9; c2_i705++) {
    c2_dv25[c2_i705] = c2_dv10[c2_i705];
  }

  c2_c_kron(chartInstance, c2_jb_b, c2_dv25, c2_fe_y);
  for (c2_i706 = 0; c2_i706 < 81; c2_i706++) {
    c2_ib_b[c2_i706] += c2_fe_y[c2_i706];
  }

  c2_v_eml_scalar_eg(chartInstance);
  c2_v_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i707 = 0; c2_i707 < 3; c2_i707++) {
    c2_i708 = 0;
    c2_i709 = 0;
    for (c2_i710 = 0; c2_i710 < 9; c2_i710++) {
      c2_wc_y[c2_i708 + c2_i707] = 0.0;
      c2_i711 = 0;
      for (c2_i712 = 0; c2_i712 < 9; c2_i712++) {
        c2_wc_y[c2_i708 + c2_i707] += c2_v_b[c2_i711 + c2_i707] *
          c2_ib_b[c2_i712 + c2_i709];
        c2_i711 += 3;
      }

      c2_i708 += 3;
      c2_i709 += 9;
    }
  }

  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i713 = 0; c2_i713 < 3; c2_i713++) {
    c2_i714 = 0;
    c2_i715 = 0;
    for (c2_i716 = 0; c2_i716 < 3; c2_i716++) {
      c2_yb_y[c2_i714 + c2_i713] = 0.0;
      c2_i717 = 0;
      for (c2_i718 = 0; c2_i718 < 9; c2_i718++) {
        c2_yb_y[c2_i714 + c2_i713] += c2_wc_y[c2_i717 + c2_i713] *
          c2_kb_b[c2_i718 + c2_i715];
        c2_i717 += 3;
      }

      c2_i714 += 3;
      c2_i715 += 9;
    }
  }

  for (c2_i719 = 0; c2_i719 < 3; c2_i719++) {
    c2_b[c2_i719] = c2_hat_dot_r32[c2_i719];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i720 = 0; c2_i720 < 3; c2_i720++) {
    c2_l_y[c2_i720] = 0.0;
    c2_i721 = 0;
    for (c2_i722 = 0; c2_i722 < 3; c2_i722++) {
      c2_l_y[c2_i720] += c2_yb_y[c2_i721 + c2_i720] * c2_b[c2_i722];
      c2_i721 += 3;
    }
  }

  for (c2_i723 = 0; c2_i723 < 3; c2_i723++) {
    c2_b_b[c2_i723] = c2_hat_dot_hat_dot_mu2[c2_i723];
  }

  for (c2_i724 = 0; c2_i724 < 3; c2_i724++) {
    c2_k_mu[c2_i724] = c2_mu[c2_i724];
  }

  c2_s_B = c2_norm(chartInstance, c2_k_mu);
  c2_ge_y = c2_s_B;
  c2_he_y = c2_ge_y;
  c2_ie_y = c2_he_y;
  for (c2_i725 = 0; c2_i725 < 3; c2_i725++) {
    c2_b_b[c2_i725] /= c2_ie_y;
  }

  for (c2_i726 = 0; c2_i726 < 3; c2_i726++) {
    c2_C[c2_i726] = c2_hat_dot_mu[c2_i726];
  }

  for (c2_i727 = 0; c2_i727 < 3; c2_i727++) {
    c2_q_y[c2_i727] = c2_mu[c2_i727];
  }

  for (c2_i728 = 0; c2_i728 < 3; c2_i728++) {
    c2_i729 = 0;
    for (c2_i730 = 0; c2_i730 < 3; c2_i730++) {
      c2_yb_y[c2_i729 + c2_i728] = c2_C[c2_i728] * c2_q_y[c2_i730];
      c2_i729 += 3;
    }
  }

  for (c2_i731 = 0; c2_i731 < 3; c2_i731++) {
    c2_b[c2_i731] = c2_hat_dot_mu2[c2_i731];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i732 = 0; c2_i732 < 3; c2_i732++) {
    c2_sc_y[c2_i732] = 0.0;
    c2_i733 = 0;
    for (c2_i734 = 0; c2_i734 < 3; c2_i734++) {
      c2_sc_y[c2_i732] += c2_yb_y[c2_i733 + c2_i732] * c2_b[c2_i734];
      c2_i733 += 3;
    }
  }

  for (c2_i735 = 0; c2_i735 < 3; c2_i735++) {
    c2_l_mu[c2_i735] = c2_mu[c2_i735];
  }

  c2_t_B = c2_e_mpower(chartInstance, c2_norm(chartInstance, c2_l_mu));
  c2_je_y = c2_t_B;
  c2_ke_y = c2_je_y;
  c2_le_y = c2_ke_y;
  for (c2_i736 = 0; c2_i736 < 3; c2_i736++) {
    c2_sc_y[c2_i736] /= c2_le_y;
  }

  for (c2_i737 = 0; c2_i737 < 3; c2_i737++) {
    c2_g_r3[c2_i737] = c2_r3[c2_i737];
  }

  c2_S(chartInstance, c2_g_r3, c2_dv10);
  for (c2_i738 = 0; c2_i738 < 9; c2_i738++) {
    c2_dv26[c2_i738] = c2_dv10[c2_i738];
  }

  c2_mpower(chartInstance, c2_dv26, c2_e_a);
  for (c2_i739 = 0; c2_i739 < 3; c2_i739++) {
    c2_b_b[c2_i739] -= c2_sc_y[c2_i739];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i740 = 0; c2_i740 < 3; c2_i740++) {
    c2_sc_y[c2_i740] = 0.0;
    c2_i741 = 0;
    for (c2_i742 = 0; c2_i742 < 3; c2_i742++) {
      c2_sc_y[c2_i740] += c2_e_a[c2_i741 + c2_i740] * c2_b_b[c2_i742];
      c2_i741 += 3;
    }
  }

  for (c2_i743 = 0; c2_i743 < 3; c2_i743++) {
    c2_hat_dot_hat_dot_r32[c2_i743] = c2_l_y[c2_i743] + c2_sc_y[c2_i743];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 176U);
  c2_w_eml_scalar_eg(chartInstance);
  c2_w_eml_scalar_eg(chartInstance);
  for (c2_i744 = 0; c2_i744 < 81; c2_i744++) {
    c2_me_y[c2_i744] = 0.0;
  }

  for (c2_i745 = 0; c2_i745 < 729; c2_i745++) {
    c2_fc_a[c2_i745] = c2_ec_a[c2_i745];
  }

  for (c2_i746 = 0; c2_i746 < 81; c2_i746++) {
    c2_mb_b[c2_i746] = c2_lb_b[c2_i746];
  }

  c2_g_eml_xgemm(chartInstance, c2_fc_a, c2_mb_b, c2_me_y);
  c2_w_eml_scalar_eg(chartInstance);
  c2_w_eml_scalar_eg(chartInstance);
  for (c2_i747 = 0; c2_i747 < 81; c2_i747++) {
    c2_ne_y[c2_i747] = 0.0;
  }

  for (c2_i748 = 0; c2_i748 < 729; c2_i748++) {
    c2_hc_a[c2_i748] = c2_gc_a[c2_i748];
  }

  for (c2_i749 = 0; c2_i749 < 81; c2_i749++) {
    c2_ob_b[c2_i749] = c2_nb_b[c2_i749];
  }

  c2_g_eml_xgemm(chartInstance, c2_hc_a, c2_ob_b, c2_ne_y);
  for (c2_i750 = 0; c2_i750 < 81; c2_i750++) {
    c2_me_y[c2_i750] = -(c2_me_y[c2_i750] + c2_ne_y[c2_i750]);
  }

  for (c2_i751 = 0; c2_i751 < 9; c2_i751++) {
    c2_d_b[c2_i751] = c2_H2[c2_i751];
  }

  c2_x_eml_scalar_eg(chartInstance);
  c2_x_eml_scalar_eg(chartInstance);
  for (c2_i752 = 0; c2_i752 < 81; c2_i752++) {
    c2_Dr3_H1[c2_i752] = 0.0;
  }

  for (c2_i753 = 0; c2_i753 < 81; c2_i753++) {
    c2_Dr3_H1[c2_i753] = 0.0;
  }

  for (c2_i754 = 0; c2_i754 < 81; c2_i754++) {
    c2_ne_y[c2_i754] = c2_Dr3_H1[c2_i754];
  }

  for (c2_i755 = 0; c2_i755 < 81; c2_i755++) {
    c2_Dr3_H1[c2_i755] = c2_ne_y[c2_i755];
  }

  c2_threshold(chartInstance);
  for (c2_i756 = 0; c2_i756 < 81; c2_i756++) {
    c2_ne_y[c2_i756] = c2_Dr3_H1[c2_i756];
  }

  for (c2_i757 = 0; c2_i757 < 81; c2_i757++) {
    c2_Dr3_H1[c2_i757] = c2_ne_y[c2_i757];
  }

  for (c2_i758 = 0; c2_i758 < 27; c2_i758++) {
    c2_i759 = 0;
    c2_i760 = 0;
    for (c2_i761 = 0; c2_i761 < 3; c2_i761++) {
      c2_Dr3_H1[c2_i759 + c2_i758] = 0.0;
      c2_i762 = 0;
      for (c2_i763 = 0; c2_i763 < 3; c2_i763++) {
        c2_Dr3_H1[c2_i759 + c2_i758] += c2_me_y[c2_i762 + c2_i758] *
          c2_d_b[c2_i763 + c2_i760];
        c2_i762 += 27;
      }

      c2_i759 += 27;
      c2_i760 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 179U);
  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i764 = 0; c2_i764 < 3; c2_i764++) {
    c2_l_y[c2_i764] = 0.0;
    c2_i765 = 0;
    for (c2_i766 = 0; c2_i766 < 3; c2_i766++) {
      c2_l_y[c2_i764] += c2_get_Rd(chartInstance, c2_i765 + c2_i764) *
        c2_c_b[c2_i766];
      c2_i765 += 3;
    }
  }

  for (c2_i767 = 0; c2_i767 < 3; c2_i767++) {
    c2_b_b[c2_i767] = c2_gm[c2_i767];
  }

  for (c2_i768 = 0; c2_i768 < 3; c2_i768++) {
    c2_i_gm[c2_i768] = c2_gm[c2_i768];
  }

  c2_u_B = c2_norm(chartInstance, c2_i_gm);
  c2_oe_y = c2_u_B;
  c2_pe_y = c2_oe_y;
  c2_qe_y = c2_pe_y;
  for (c2_i769 = 0; c2_i769 < 3; c2_i769++) {
    c2_b_b[c2_i769] /= c2_qe_y;
  }

  for (c2_i770 = 0; c2_i770 < 3; c2_i770++) {
    c2_C[c2_i770] = c2_gm[c2_i770];
  }

  for (c2_i771 = 0; c2_i771 < 3; c2_i771++) {
    c2_j_gm[c2_i771] = c2_gm[c2_i771];
  }

  c2_v_B = c2_norm(chartInstance, c2_j_gm);
  c2_re_y = c2_v_B;
  c2_se_y = c2_re_y;
  c2_te_y = c2_se_y;
  for (c2_i772 = 0; c2_i772 < 3; c2_i772++) {
    c2_C[c2_i772] /= c2_te_y;
  }

  for (c2_i773 = 0; c2_i773 < 3; c2_i773++) {
    c2_ue_y[c2_i773] = c2_l_y[c2_i773];
  }

  for (c2_i774 = 0; c2_i774 < 81; c2_i774++) {
    c2_dv28[c2_i774] = c2_dv27[c2_i774];
  }

  c2_g_kron(chartInstance, c2_ue_y, c2_dv28, c2_ic_a);
  for (c2_i775 = 0; c2_i775 < 3; c2_i775++) {
    c2_pb_b[c2_i775] = c2_b_b[c2_i775];
  }

  c2_S(chartInstance, c2_pb_b, c2_dv10);
  for (c2_i776 = 0; c2_i776 < 9; c2_i776++) {
    c2_qb_b[c2_i776] = c2_k_b[c2_i776];
  }

  for (c2_i777 = 0; c2_i777 < 9; c2_i777++) {
    c2_dv29[c2_i777] = c2_dv10[c2_i777];
  }

  c2_c_kron(chartInstance, c2_qb_b, c2_dv29, c2_ib_b);
  for (c2_i778 = 0; c2_i778 < 3; c2_i778++) {
    c2_e_C[c2_i778] = c2_C[c2_i778];
  }

  c2_S(chartInstance, c2_e_C, c2_dv10);
  for (c2_i779 = 0; c2_i779 < 9; c2_i779++) {
    c2_dv30[c2_i779] = c2_dv10[c2_i779];
  }

  for (c2_i780 = 0; c2_i780 < 9; c2_i780++) {
    c2_rb_b[c2_i780] = c2_k_b[c2_i780];
  }

  c2_c_kron(chartInstance, c2_dv30, c2_rb_b, c2_fe_y);
  for (c2_i781 = 0; c2_i781 < 81; c2_i781++) {
    c2_ib_b[c2_i781] -= c2_fe_y[c2_i781];
  }

  c2_y_eml_scalar_eg(chartInstance);
  c2_y_eml_scalar_eg(chartInstance);
  for (c2_i782 = 0; c2_i782 < 243; c2_i782++) {
    c2_ve_y[c2_i782] = 0.0;
  }

  for (c2_i783 = 0; c2_i783 < 243; c2_i783++) {
    c2_jc_a[c2_i783] = c2_ic_a[c2_i783];
  }

  for (c2_i784 = 0; c2_i784 < 81; c2_i784++) {
    c2_sb_b[c2_i784] = c2_ib_b[c2_i784];
  }

  c2_h_eml_xgemm(chartInstance, c2_jc_a, c2_sb_b, c2_ve_y);
  c2_ab_eml_scalar_eg(chartInstance);
  c2_ab_eml_scalar_eg(chartInstance);
  for (c2_i785 = 0; c2_i785 < 81; c2_i785++) {
    c2_me_y[c2_i785] = 0.0;
  }

  for (c2_i786 = 0; c2_i786 < 243; c2_i786++) {
    c2_we_y[c2_i786] = c2_ve_y[c2_i786];
  }

  for (c2_i787 = 0; c2_i787 < 27; c2_i787++) {
    c2_tb_b[c2_i787] = c2_kb_b[c2_i787];
  }

  c2_i_eml_xgemm(chartInstance, c2_we_y, c2_tb_b, c2_me_y);
  for (c2_i788 = 0; c2_i788 < 3; c2_i788++) {
    c2_b_b[c2_i788] = c2_gm[c2_i788];
  }

  for (c2_i789 = 0; c2_i789 < 3; c2_i789++) {
    c2_k_gm[c2_i789] = c2_gm[c2_i789];
  }

  c2_w_B = c2_norm(chartInstance, c2_k_gm);
  c2_xe_y = c2_w_B;
  c2_ye_y = c2_xe_y;
  c2_af_y = c2_ye_y;
  for (c2_i790 = 0; c2_i790 < 3; c2_i790++) {
    c2_b_b[c2_i790] /= c2_af_y;
  }

  for (c2_i791 = 0; c2_i791 < 3; c2_i791++) {
    c2_ub_b[c2_i791] = c2_b_b[c2_i791];
  }

  c2_S(chartInstance, c2_ub_b, c2_dv10);
  for (c2_i792 = 0; c2_i792 < 9; c2_i792++) {
    c2_dv31[c2_i792] = c2_dv10[c2_i792];
  }

  c2_mpower(chartInstance, c2_dv31, c2_d_b);
  c2_x_eml_scalar_eg(chartInstance);
  c2_x_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i793 = 0; c2_i793 < 27; c2_i793++) {
    c2_i794 = 0;
    c2_i795 = 0;
    for (c2_i796 = 0; c2_i796 < 3; c2_i796++) {
      c2_ne_y[c2_i794 + c2_i793] = 0.0;
      c2_i797 = 0;
      for (c2_i798 = 0; c2_i798 < 3; c2_i798++) {
        c2_ne_y[c2_i794 + c2_i793] += c2_me_y[c2_i797 + c2_i793] *
          c2_d_b[c2_i798 + c2_i795];
        c2_i797 += 27;
      }

      c2_i794 += 27;
      c2_i795 += 3;
    }
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i799 = 0; c2_i799 < 3; c2_i799++) {
    c2_l_y[c2_i799] = 0.0;
    c2_i800 = 0;
    for (c2_i801 = 0; c2_i801 < 3; c2_i801++) {
      c2_l_y[c2_i799] += c2_get_Rd(chartInstance, c2_i800 + c2_i799) *
        c2_c_b[c2_i801];
      c2_i800 += 3;
    }
  }

  for (c2_i802 = 0; c2_i802 < 3; c2_i802++) {
    c2_bf_y[c2_i802] = c2_l_y[c2_i802];
  }

  c2_S(chartInstance, c2_bf_y, c2_d_b);
  c2_x_eml_scalar_eg(chartInstance);
  c2_x_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i803 = 0; c2_i803 < 27; c2_i803++) {
    c2_i804 = 0;
    c2_i805 = 0;
    for (c2_i806 = 0; c2_i806 < 3; c2_i806++) {
      c2_me_y[c2_i804 + c2_i803] = 0.0;
      c2_i807 = 0;
      for (c2_i808 = 0; c2_i808 < 3; c2_i808++) {
        c2_me_y[c2_i804 + c2_i803] += c2_ne_y[c2_i807 + c2_i803] *
          c2_d_b[c2_i808 + c2_i805];
        c2_i807 += 27;
      }

      c2_i804 += 27;
      c2_i805 += 3;
    }
  }

  for (c2_i809 = 0; c2_i809 < 3; c2_i809++) {
    c2_l_gm[c2_i809] = c2_gm[c2_i809];
  }

  c2_x_B = c2_norm(chartInstance, c2_l_gm);
  c2_cf_y = c2_x_B;
  c2_df_y = c2_cf_y;
  c2_ef_y = c2_df_y;
  for (c2_i810 = 0; c2_i810 < 81; c2_i810++) {
    c2_Dr3_Y[c2_i810] = c2_me_y[c2_i810] / c2_ef_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 180U);
  for (c2_i811 = 0; c2_i811 < 3; c2_i811++) {
    c2_b_b[c2_i811] = c2_gm[c2_i811];
  }

  for (c2_i812 = 0; c2_i812 < 3; c2_i812++) {
    c2_m_gm[c2_i812] = c2_gm[c2_i812];
  }

  c2_y_B = c2_norm(chartInstance, c2_m_gm);
  c2_ff_y = c2_y_B;
  c2_gf_y = c2_ff_y;
  c2_hf_y = c2_gf_y;
  for (c2_i813 = 0; c2_i813 < 3; c2_i813++) {
    c2_b_b[c2_i813] /= c2_hf_y;
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i814 = 0; c2_i814 < 3; c2_i814++) {
    c2_l_y[c2_i814] = 0.0;
    c2_i815 = 0;
    for (c2_i816 = 0; c2_i816 < 3; c2_i816++) {
      c2_l_y[c2_i814] += c2_get_Rd(chartInstance, c2_i815 + c2_i814) *
        c2_c_b[c2_i816];
      c2_i815 += 3;
    }
  }

  for (c2_i817 = 0; c2_i817 < 3; c2_i817++) {
    c2_vb_b[c2_i817] = c2_b_b[c2_i817];
  }

  c2_S(chartInstance, c2_vb_b, c2_dv10);
  for (c2_i818 = 0; c2_i818 < 9; c2_i818++) {
    c2_dv32[c2_i818] = c2_dv10[c2_i818];
  }

  c2_mpower(chartInstance, c2_dv32, c2_e_a);
  for (c2_i819 = 0; c2_i819 < 3; c2_i819++) {
    c2_if_y[c2_i819] = c2_l_y[c2_i819];
  }

  c2_S(chartInstance, c2_if_y, c2_d_b);
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i820 = 0; c2_i820 < 3; c2_i820++) {
    c2_i821 = 0;
    for (c2_i822 = 0; c2_i822 < 3; c2_i822++) {
      c2_yb_y[c2_i821 + c2_i820] = 0.0;
      c2_i823 = 0;
      for (c2_i824 = 0; c2_i824 < 3; c2_i824++) {
        c2_yb_y[c2_i821 + c2_i820] += c2_e_a[c2_i823 + c2_i820] * c2_d_b[c2_i824
          + c2_i821];
        c2_i823 += 3;
      }

      c2_i821 += 3;
    }
  }

  for (c2_i825 = 0; c2_i825 < 3; c2_i825++) {
    c2_n_gm[c2_i825] = c2_gm[c2_i825];
  }

  c2_ab_B = c2_norm(chartInstance, c2_n_gm);
  c2_jf_y = c2_ab_B;
  c2_kf_y = c2_jf_y;
  c2_lf_y = c2_kf_y;
  for (c2_i826 = 0; c2_i826 < 9; c2_i826++) {
    c2_H2[c2_i826] = c2_yb_y[c2_i826] / c2_lf_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 181U);
  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i827 = 0; c2_i827 < 3; c2_i827++) {
    c2_l_y[c2_i827] = 0.0;
    c2_i828 = 0;
    for (c2_i829 = 0; c2_i829 < 3; c2_i829++) {
      c2_l_y[c2_i827] += c2_get_Rd(chartInstance, c2_i828 + c2_i827) *
        c2_c_b[c2_i829];
      c2_i828 += 3;
    }
  }

  for (c2_i830 = 0; c2_i830 < 3; c2_i830++) {
    c2_mf_y[c2_i830] = c2_l_y[c2_i830];
  }

  c2_S(chartInstance, c2_mf_y, c2_dv10);
  c2_i831 = 0;
  for (c2_i832 = 0; c2_i832 < 3; c2_i832++) {
    c2_i833 = 0;
    for (c2_i834 = 0; c2_i834 < 3; c2_i834++) {
      c2_b_C[c2_i834 + c2_i831] = c2_dv10[c2_i833 + c2_i832];
      c2_i833 += 3;
    }

    c2_i831 += 3;
  }

  for (c2_i835 = 0; c2_i835 < 3; c2_i835++) {
    c2_o_gm[c2_i835] = c2_gm[c2_i835];
  }

  c2_bb_B = c2_norm(chartInstance, c2_o_gm);
  c2_nf_y = c2_bb_B;
  c2_of_y = c2_nf_y;
  c2_pf_y = c2_of_y;
  for (c2_i836 = 0; c2_i836 < 9; c2_i836++) {
    c2_b_C[c2_i836] /= c2_pf_y;
  }

  for (c2_i837 = 0; c2_i837 < 3; c2_i837++) {
    c2_b_b[c2_i837] = c2_gm[c2_i837];
  }

  for (c2_i838 = 0; c2_i838 < 3; c2_i838++) {
    c2_p_gm[c2_i838] = c2_gm[c2_i838];
  }

  c2_cb_B = c2_norm(chartInstance, c2_p_gm);
  c2_qf_y = c2_cb_B;
  c2_rf_y = c2_qf_y;
  c2_sf_y = c2_rf_y;
  for (c2_i839 = 0; c2_i839 < 3; c2_i839++) {
    c2_b_b[c2_i839] /= c2_sf_y;
  }

  for (c2_i840 = 0; c2_i840 < 3; c2_i840++) {
    c2_C[c2_i840] = c2_gm[c2_i840];
  }

  for (c2_i841 = 0; c2_i841 < 3; c2_i841++) {
    c2_q_gm[c2_i841] = c2_gm[c2_i841];
  }

  c2_db_B = c2_norm(chartInstance, c2_q_gm);
  c2_tf_y = c2_db_B;
  c2_uf_y = c2_tf_y;
  c2_vf_y = c2_uf_y;
  for (c2_i842 = 0; c2_i842 < 3; c2_i842++) {
    c2_C[c2_i842] /= c2_vf_y;
  }

  for (c2_i843 = 0; c2_i843 < 9; c2_i843++) {
    c2_f_C[c2_i843] = c2_b_C[c2_i843];
  }

  for (c2_i844 = 0; c2_i844 < 9; c2_i844++) {
    c2_wb_b[c2_i844] = c2_k_b[c2_i844];
  }

  c2_c_kron(chartInstance, c2_f_C, c2_wb_b, c2_p_a);
  for (c2_i845 = 0; c2_i845 < 3; c2_i845++) {
    c2_xb_b[c2_i845] = c2_b_b[c2_i845];
  }

  c2_S(chartInstance, c2_xb_b, c2_dv10);
  for (c2_i846 = 0; c2_i846 < 9; c2_i846++) {
    c2_yb_b[c2_i846] = c2_k_b[c2_i846];
  }

  for (c2_i847 = 0; c2_i847 < 9; c2_i847++) {
    c2_dv33[c2_i847] = c2_dv10[c2_i847];
  }

  c2_c_kron(chartInstance, c2_yb_b, c2_dv33, c2_ib_b);
  for (c2_i848 = 0; c2_i848 < 3; c2_i848++) {
    c2_g_C[c2_i848] = c2_C[c2_i848];
  }

  c2_S(chartInstance, c2_g_C, c2_dv10);
  for (c2_i849 = 0; c2_i849 < 9; c2_i849++) {
    c2_dv34[c2_i849] = c2_dv10[c2_i849];
  }

  for (c2_i850 = 0; c2_i850 < 9; c2_i850++) {
    c2_ac_b[c2_i850] = c2_k_b[c2_i850];
  }

  c2_c_kron(chartInstance, c2_dv34, c2_ac_b, c2_fe_y);
  for (c2_i851 = 0; c2_i851 < 81; c2_i851++) {
    c2_ib_b[c2_i851] -= c2_fe_y[c2_i851];
  }

  c2_bb_eml_scalar_eg(chartInstance);
  c2_bb_eml_scalar_eg(chartInstance);
  for (c2_i852 = 0; c2_i852 < 81; c2_i852++) {
    c2_fe_y[c2_i852] = 0.0;
  }

  for (c2_i853 = 0; c2_i853 < 81; c2_i853++) {
    c2_kc_a[c2_i853] = c2_p_a[c2_i853];
  }

  for (c2_i854 = 0; c2_i854 < 81; c2_i854++) {
    c2_bc_b[c2_i854] = c2_ib_b[c2_i854];
  }

  c2_j_eml_xgemm(chartInstance, c2_kc_a, c2_bc_b, c2_fe_y);
  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i855 = 0; c2_i855 < 9; c2_i855++) {
    c2_i856 = 0;
    for (c2_i857 = 0; c2_i857 < 3; c2_i857++) {
      c2_ec_y[c2_i856 + c2_i855] = 0.0;
      c2_i858 = 0;
      for (c2_i859 = 0; c2_i859 < 9; c2_i859++) {
        c2_ec_y[c2_i856 + c2_i855] += c2_fe_y[c2_i858 + c2_i855] *
          c2_kb_b[c2_i859 + c2_i856];
        c2_i858 += 9;
      }

      c2_i856 += 9;
    }
  }

  for (c2_i860 = 0; c2_i860 < 3; c2_i860++) {
    c2_b_b[c2_i860] = c2_gm[c2_i860];
  }

  for (c2_i861 = 0; c2_i861 < 3; c2_i861++) {
    c2_r_gm[c2_i861] = c2_gm[c2_i861];
  }

  c2_eb_B = c2_norm(chartInstance, c2_r_gm);
  c2_wf_y = c2_eb_B;
  c2_xf_y = c2_wf_y;
  c2_yf_y = c2_xf_y;
  for (c2_i862 = 0; c2_i862 < 3; c2_i862++) {
    c2_b_b[c2_i862] /= c2_yf_y;
  }

  for (c2_i863 = 0; c2_i863 < 3; c2_i863++) {
    c2_cc_b[c2_i863] = c2_b_b[c2_i863];
  }

  c2_S(chartInstance, c2_cc_b, c2_dv10);
  for (c2_i864 = 0; c2_i864 < 9; c2_i864++) {
    c2_dv35[c2_i864] = c2_dv10[c2_i864];
  }

  c2_mpower(chartInstance, c2_dv35, c2_d_b);
  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i865 = 0; c2_i865 < 9; c2_i865++) {
    c2_i866 = 0;
    c2_i867 = 0;
    for (c2_i868 = 0; c2_i868 < 3; c2_i868++) {
      c2_sb_y[c2_i866 + c2_i865] = 0.0;
      c2_i869 = 0;
      for (c2_i870 = 0; c2_i870 < 3; c2_i870++) {
        c2_sb_y[c2_i866 + c2_i865] += c2_ec_y[c2_i869 + c2_i865] *
          c2_d_b[c2_i870 + c2_i867];
        c2_i869 += 9;
      }

      c2_i866 += 9;
      c2_i867 += 3;
    }
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i871 = 0; c2_i871 < 3; c2_i871++) {
    c2_l_y[c2_i871] = 0.0;
    c2_i872 = 0;
    for (c2_i873 = 0; c2_i873 < 3; c2_i873++) {
      c2_l_y[c2_i871] += c2_get_Rd(chartInstance, c2_i872 + c2_i871) *
        c2_c_b[c2_i873];
      c2_i872 += 3;
    }
  }

  for (c2_i874 = 0; c2_i874 < 3; c2_i874++) {
    c2_ag_y[c2_i874] = c2_l_y[c2_i874];
  }

  c2_S(chartInstance, c2_ag_y, c2_d_b);
  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i875 = 0; c2_i875 < 9; c2_i875++) {
    c2_i876 = 0;
    c2_i877 = 0;
    for (c2_i878 = 0; c2_i878 < 3; c2_i878++) {
      c2_ec_y[c2_i876 + c2_i875] = 0.0;
      c2_i879 = 0;
      for (c2_i880 = 0; c2_i880 < 3; c2_i880++) {
        c2_ec_y[c2_i876 + c2_i875] += c2_sb_y[c2_i879 + c2_i875] *
          c2_d_b[c2_i880 + c2_i877];
        c2_i879 += 9;
      }

      c2_i876 += 9;
      c2_i877 += 3;
    }
  }

  for (c2_i881 = 0; c2_i881 < 3; c2_i881++) {
    c2_s_gm[c2_i881] = c2_gm[c2_i881];
  }

  c2_fb_B = c2_norm(chartInstance, c2_s_gm);
  c2_bg_y = c2_fb_B;
  c2_cg_y = c2_bg_y;
  c2_dg_y = c2_cg_y;
  for (c2_i882 = 0; c2_i882 < 27; c2_i882++) {
    c2_ec_y[c2_i882] /= c2_dg_y;
  }

  for (c2_i883 = 0; c2_i883 < 3; c2_i883++) {
    c2_b_b[c2_i883] = c2_gm[c2_i883];
  }

  for (c2_i884 = 0; c2_i884 < 3; c2_i884++) {
    c2_t_gm[c2_i884] = c2_gm[c2_i884];
  }

  c2_gb_B = c2_norm(chartInstance, c2_t_gm);
  c2_eg_y = c2_gb_B;
  c2_fg_y = c2_eg_y;
  c2_gg_y = c2_fg_y;
  for (c2_i885 = 0; c2_i885 < 3; c2_i885++) {
    c2_b_b[c2_i885] /= c2_gg_y;
  }

  for (c2_i886 = 0; c2_i886 < 3; c2_i886++) {
    c2_dc_b[c2_i886] = c2_b_b[c2_i886];
  }

  c2_S(chartInstance, c2_dc_b, c2_dv10);
  for (c2_i887 = 0; c2_i887 < 9; c2_i887++) {
    c2_dv36[c2_i887] = c2_dv10[c2_i887];
  }

  c2_mpower(chartInstance, c2_dv36, c2_b_C);
  for (c2_i888 = 0; c2_i888 < 9; c2_i888++) {
    c2_ec_b[c2_i888] = c2_k_b[c2_i888];
  }

  for (c2_i889 = 0; c2_i889 < 9; c2_i889++) {
    c2_h_C[c2_i889] = c2_b_C[c2_i889];
  }

  c2_c_kron(chartInstance, c2_ec_b, c2_h_C, c2_p_a);
  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i890 = 0; c2_i890 < 9; c2_i890++) {
    c2_i891 = 0;
    for (c2_i892 = 0; c2_i892 < 3; c2_i892++) {
      c2_sb_y[c2_i891 + c2_i890] = 0.0;
      c2_i893 = 0;
      for (c2_i894 = 0; c2_i894 < 9; c2_i894++) {
        c2_sb_y[c2_i891 + c2_i890] += c2_p_a[c2_i893 + c2_i890] *
          c2_kb_b[c2_i894 + c2_i891];
        c2_i893 += 9;
      }

      c2_i891 += 9;
    }
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i895 = 0; c2_i895 < 9; c2_i895++) {
    c2_i896 = 0;
    c2_i897 = 0;
    for (c2_i898 = 0; c2_i898 < 3; c2_i898++) {
      c2_tb_y[c2_i896 + c2_i895] = 0.0;
      c2_i899 = 0;
      for (c2_i900 = 0; c2_i900 < 3; c2_i900++) {
        c2_tb_y[c2_i896 + c2_i895] += c2_sb_y[c2_i899 + c2_i895] * c2_get_Rd
          (chartInstance, c2_i900 + c2_i897);
        c2_i899 += 9;
      }

      c2_i896 += 9;
      c2_i897 += 3;
    }
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i901 = 0; c2_i901 < 9; c2_i901++) {
    c2_g_x[c2_i901] = 0.0;
    c2_i902 = 0;
    for (c2_i903 = 0; c2_i903 < 3; c2_i903++) {
      c2_g_x[c2_i901] += c2_tb_y[c2_i902 + c2_i901] * c2_c_b[c2_i903];
      c2_i902 += 9;
    }
  }

  for (c2_i904 = 0; c2_i904 < 3; c2_i904++) {
    c2_q_y[c2_i904] = c2_gm[c2_i904];
  }

  for (c2_i905 = 0; c2_i905 < 9; c2_i905++) {
    c2_i906 = 0;
    for (c2_i907 = 0; c2_i907 < 3; c2_i907++) {
      c2_sb_y[c2_i906 + c2_i905] = c2_g_x[c2_i905] * c2_q_y[c2_i907];
      c2_i906 += 9;
    }
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i908 = 0; c2_i908 < 3; c2_i908++) {
    c2_l_y[c2_i908] = 0.0;
    c2_i909 = 0;
    for (c2_i910 = 0; c2_i910 < 3; c2_i910++) {
      c2_l_y[c2_i908] += c2_get_Rd(chartInstance, c2_i909 + c2_i908) *
        c2_c_b[c2_i910];
      c2_i909 += 3;
    }
  }

  for (c2_i911 = 0; c2_i911 < 3; c2_i911++) {
    c2_hg_y[c2_i911] = c2_l_y[c2_i911];
  }

  c2_S(chartInstance, c2_hg_y, c2_d_b);
  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i912 = 0; c2_i912 < 9; c2_i912++) {
    c2_i913 = 0;
    c2_i914 = 0;
    for (c2_i915 = 0; c2_i915 < 3; c2_i915++) {
      c2_tb_y[c2_i913 + c2_i912] = 0.0;
      c2_i916 = 0;
      for (c2_i917 = 0; c2_i917 < 3; c2_i917++) {
        c2_tb_y[c2_i913 + c2_i912] += c2_sb_y[c2_i916 + c2_i912] *
          c2_d_b[c2_i917 + c2_i914];
        c2_i916 += 9;
      }

      c2_i913 += 9;
      c2_i914 += 3;
    }
  }

  for (c2_i918 = 0; c2_i918 < 3; c2_i918++) {
    c2_u_gm[c2_i918] = c2_gm[c2_i918];
  }

  c2_hb_B = c2_e_mpower(chartInstance, c2_norm(chartInstance, c2_u_gm));
  c2_ig_y = c2_hb_B;
  c2_jg_y = c2_ig_y;
  c2_kg_y = c2_jg_y;
  for (c2_i919 = 0; c2_i919 < 27; c2_i919++) {
    c2_tb_y[c2_i919] /= c2_kg_y;
  }

  for (c2_i920 = 0; c2_i920 < 27; c2_i920++) {
    c2_Dr3_H2[c2_i920] = c2_ec_y[c2_i920] + c2_tb_y[c2_i920];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 184U);
  for (c2_i921 = 0; c2_i921 < 27; c2_i921++) {
    c2_sb_y[c2_i921] = c2_H1[c2_i921];
  }

  for (c2_i922 = 0; c2_i922 < 9; c2_i922++) {
    c2_d_b[c2_i922] = c2_H2[c2_i922];
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  for (c2_i923 = 0; c2_i923 < 27; c2_i923++) {
    c2_H[c2_i923] = 0.0;
  }

  for (c2_i924 = 0; c2_i924 < 27; c2_i924++) {
    c2_H[c2_i924] = 0.0;
  }

  for (c2_i925 = 0; c2_i925 < 27; c2_i925++) {
    c2_tb_y[c2_i925] = c2_H[c2_i925];
  }

  for (c2_i926 = 0; c2_i926 < 27; c2_i926++) {
    c2_H[c2_i926] = c2_tb_y[c2_i926];
  }

  c2_threshold(chartInstance);
  for (c2_i927 = 0; c2_i927 < 27; c2_i927++) {
    c2_tb_y[c2_i927] = c2_H[c2_i927];
  }

  for (c2_i928 = 0; c2_i928 < 27; c2_i928++) {
    c2_H[c2_i928] = c2_tb_y[c2_i928];
  }

  for (c2_i929 = 0; c2_i929 < 9; c2_i929++) {
    c2_i930 = 0;
    c2_i931 = 0;
    for (c2_i932 = 0; c2_i932 < 3; c2_i932++) {
      c2_H[c2_i930 + c2_i929] = 0.0;
      c2_i933 = 0;
      for (c2_i934 = 0; c2_i934 < 3; c2_i934++) {
        c2_H[c2_i930 + c2_i929] += c2_sb_y[c2_i933 + c2_i929] * c2_d_b[c2_i934 +
          c2_i931];
        c2_i933 += 9;
      }

      c2_i930 += 9;
      c2_i931 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 185U);
  c2_i935 = 0;
  for (c2_i936 = 0; c2_i936 < 3; c2_i936++) {
    c2_i937 = 0;
    for (c2_i938 = 0; c2_i938 < 3; c2_i938++) {
      c2_b_H2[c2_i938 + c2_i935] = c2_H2[c2_i937 + c2_i936];
      c2_i937 += 3;
    }

    c2_i935 += 3;
  }

  for (c2_i939 = 0; c2_i939 < 81; c2_i939++) {
    c2_dv37[c2_i939] = c2_dv27[c2_i939];
  }

  c2_f_kron(chartInstance, c2_b_H2, c2_dv37, c2_lc_a);
  for (c2_i940 = 0; c2_i940 < 81; c2_i940++) {
    c2_fc_b[c2_i940] = c2_Dr3_H1[c2_i940];
  }

  c2_w_eml_scalar_eg(chartInstance);
  c2_w_eml_scalar_eg(chartInstance);
  for (c2_i941 = 0; c2_i941 < 81; c2_i941++) {
    c2_me_y[c2_i941] = 0.0;
  }

  for (c2_i942 = 0; c2_i942 < 729; c2_i942++) {
    c2_mc_a[c2_i942] = c2_lc_a[c2_i942];
  }

  for (c2_i943 = 0; c2_i943 < 81; c2_i943++) {
    c2_gc_b[c2_i943] = c2_fc_b[c2_i943];
  }

  c2_g_eml_xgemm(chartInstance, c2_mc_a, c2_gc_b, c2_me_y);
  for (c2_i944 = 0; c2_i944 < 9; c2_i944++) {
    c2_hc_b[c2_i944] = c2_k_b[c2_i944];
  }

  for (c2_i945 = 0; c2_i945 < 27; c2_i945++) {
    c2_b_H1[c2_i945] = c2_H1[c2_i945];
  }

  c2_h_kron(chartInstance, c2_hc_b, c2_b_H1, c2_ic_a);
  for (c2_i946 = 0; c2_i946 < 27; c2_i946++) {
    c2_r_b[c2_i946] = c2_Dr3_H2[c2_i946];
  }

  c2_ab_eml_scalar_eg(chartInstance);
  c2_ab_eml_scalar_eg(chartInstance);
  for (c2_i947 = 0; c2_i947 < 81; c2_i947++) {
    c2_ne_y[c2_i947] = 0.0;
  }

  for (c2_i948 = 0; c2_i948 < 243; c2_i948++) {
    c2_nc_a[c2_i948] = c2_ic_a[c2_i948];
  }

  for (c2_i949 = 0; c2_i949 < 27; c2_i949++) {
    c2_ic_b[c2_i949] = c2_r_b[c2_i949];
  }

  c2_i_eml_xgemm(chartInstance, c2_nc_a, c2_ic_b, c2_ne_y);
  for (c2_i950 = 0; c2_i950 < 81; c2_i950++) {
    c2_Dr3_H[c2_i950] = c2_me_y[c2_i950] + c2_ne_y[c2_i950];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 188U);
  c2_i951 = 0;
  for (c2_i952 = 0; c2_i952 < 9; c2_i952++) {
    c2_i953 = 0;
    for (c2_i954 = 0; c2_i954 < 3; c2_i954++) {
      c2_b_H[c2_i954 + c2_i951] = c2_H[c2_i953 + c2_i952];
      c2_i953 += 9;
    }

    c2_i951 += 3;
  }

  for (c2_i955 = 0; c2_i955 < 81; c2_i955++) {
    c2_dv38[c2_i955] = c2_dv27[c2_i955];
  }

  c2_i_kron(chartInstance, c2_b_H, c2_dv38, c2_oc_a);
  for (c2_i956 = 0; c2_i956 < 2187; c2_i956++) {
    c2_oc_a[c2_i956] = -c2_oc_a[c2_i956];
  }

  c2_db_eml_scalar_eg(chartInstance);
  c2_db_eml_scalar_eg(chartInstance);
  for (c2_i957 = 0; c2_i957 < 27; c2_i957++) {
    c2_lg_y[c2_i957] = 0.0;
  }

  for (c2_i958 = 0; c2_i958 < 2187; c2_i958++) {
    c2_pc_a[c2_i958] = c2_oc_a[c2_i958];
  }

  for (c2_i959 = 0; c2_i959 < 81; c2_i959++) {
    c2_kc_b[c2_i959] = c2_jc_b[c2_i959];
  }

  c2_k_eml_xgemm(chartInstance, c2_pc_a, c2_kc_b, c2_lg_y);
  for (c2_i960 = 0; c2_i960 < 27; c2_i960++) {
    c2_i961 = 0;
    for (c2_i962 = 0; c2_i962 < 3; c2_i962++) {
      c2_me_y[c2_i961 + c2_i960] = c2_lg_y[c2_i960] * c2_f_a[c2_i962];
      c2_i961 += 27;
    }
  }

  c2_i963 = 0;
  for (c2_i964 = 0; c2_i964 < 3; c2_i964++) {
    c2_i965 = 0;
    for (c2_i966 = 0; c2_i966 < 3; c2_i966++) {
      c2_d_b[c2_i966 + c2_i963] = c2_get_Rd(chartInstance, c2_i965 + c2_i964);
      c2_i965 += 3;
    }

    c2_i963 += 3;
  }

  c2_x_eml_scalar_eg(chartInstance);
  c2_x_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i967 = 0; c2_i967 < 27; c2_i967++) {
    c2_i968 = 0;
    c2_i969 = 0;
    for (c2_i970 = 0; c2_i970 < 3; c2_i970++) {
      c2_ne_y[c2_i968 + c2_i967] = 0.0;
      c2_i971 = 0;
      for (c2_i972 = 0; c2_i972 < 3; c2_i972++) {
        c2_ne_y[c2_i968 + c2_i967] += c2_me_y[c2_i971 + c2_i967] *
          c2_d_b[c2_i972 + c2_i969];
        c2_i971 += 27;
      }

      c2_i968 += 27;
      c2_i969 += 3;
    }
  }

  c2_i973 = 0;
  for (c2_i974 = 0; c2_i974 < 3; c2_i974++) {
    c2_i975 = 0;
    for (c2_i976 = 0; c2_i976 < 3; c2_i976++) {
      c2_d_b[c2_i976 + c2_i973] = c2_get_Rd(chartInstance, c2_i975 + c2_i974);
      c2_i975 += 3;
    }

    c2_i973 += 3;
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  c2_i977 = 0;
  for (c2_i978 = 0; c2_i978 < 3; c2_i978++) {
    c2_q_y[c2_i978] = 0.0;
    for (c2_i979 = 0; c2_i979 < 3; c2_i979++) {
      c2_q_y[c2_i978] += c2_f_a[c2_i979] * c2_d_b[c2_i979 + c2_i977];
    }

    c2_i977 += 3;
  }

  for (c2_i980 = 0; c2_i980 < 3; c2_i980++) {
    c2_b[c2_i980] = c2_r3[c2_i980];
  }

  c2_b_threshold(chartInstance);
  c2_mg_y = 0.0;
  c2_eml_switch_helper(chartInstance);
  for (c2_tb_k = 1; c2_tb_k < 4; c2_tb_k++) {
    c2_ub_k = c2_tb_k;
    c2_mg_y += c2_q_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_ub_k), 1, 3, 1, 0) - 1] *
      c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ub_k), 1, 3, 1, 0) - 1];
  }

  c2_qc_a = 1.0 - c2_mg_y;
  for (c2_i981 = 0; c2_i981 < 81; c2_i981++) {
    c2_fc_b[c2_i981] = c2_Dr3_H[c2_i981];
  }

  for (c2_i982 = 0; c2_i982 < 81; c2_i982++) {
    c2_fc_b[c2_i982] *= c2_qc_a;
  }

  for (c2_i983 = 0; c2_i983 < 81; c2_i983++) {
    c2_Dr3_X[c2_i983] = c2_ne_y[c2_i983] + c2_fc_b[c2_i983];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 190U);
  c2_i984 = 0;
  for (c2_i985 = 0; c2_i985 < 3; c2_i985++) {
    c2_i986 = 0;
    for (c2_i987 = 0; c2_i987 < 3; c2_i987++) {
      c2_dv39[c2_i987 + c2_i984] = c2_get_Rd(chartInstance, c2_i986 + c2_i985);
      c2_i986 += 3;
    }

    c2_i984 += 3;
  }

  for (c2_i988 = 0; c2_i988 < 9; c2_i988++) {
    c2_lc_b[c2_i988] = c2_k_b[c2_i988];
  }

  c2_c_kron(chartInstance, c2_dv39, c2_lc_b, c2_fe_y);
  for (c2_i989 = 0; c2_i989 < 9; c2_i989++) {
    c2_mc_b[c2_i989] = c2_k_b[c2_i989];
  }

  for (c2_i990 = 0; c2_i990 < 81; c2_i990++) {
    c2_ng_y[c2_i990] = c2_fe_y[c2_i990];
  }

  c2_f_kron(chartInstance, c2_mc_b, c2_ng_y, c2_lc_a);
  for (c2_i991 = 0; c2_i991 < 81; c2_i991++) {
    c2_fc_b[c2_i991] = -c2_Dr3_Y[c2_i991] - c2_Dr3_X[c2_i991];
  }

  c2_w_eml_scalar_eg(chartInstance);
  c2_w_eml_scalar_eg(chartInstance);
  for (c2_i992 = 0; c2_i992 < 81; c2_i992++) {
    c2_D2r3_R0[c2_i992] = 0.0;
  }

  for (c2_i993 = 0; c2_i993 < 81; c2_i993++) {
    c2_ne_y[c2_i993] = 0.0;
  }

  for (c2_i994 = 0; c2_i994 < 729; c2_i994++) {
    c2_rc_a[c2_i994] = c2_lc_a[c2_i994];
  }

  for (c2_i995 = 0; c2_i995 < 81; c2_i995++) {
    c2_nc_b[c2_i995] = c2_fc_b[c2_i995];
  }

  c2_g_eml_xgemm(chartInstance, c2_rc_a, c2_nc_b, c2_ne_y);
  for (c2_i996 = 0; c2_i996 < 81; c2_i996++) {
    c2_D2r3_R0[c2_i996] = c2_ne_y[c2_i996];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 193U);
  c2_i997 = 0;
  for (c2_i998 = 0; c2_i998 < 3; c2_i998++) {
    c2_i999 = 0;
    for (c2_i1000 = 0; c2_i1000 < 3; c2_i1000++) {
      c2_d_b[c2_i1000 + c2_i997] = c2_R0[c2_i999 + c2_i998];
      c2_i999 += 3;
    }

    c2_i997 += 3;
  }

  for (c2_i1001 = 0; c2_i1001 < 9; c2_i1001++) {
    c2_d_b[c2_i1001] *= -0.5;
  }

  for (c2_i1002 = 0; c2_i1002 < 9; c2_i1002++) {
    c2_d_R0[c2_i1002] = c2_R0[c2_i1002];
  }

  c2_Gamma(chartInstance, c2_d_R0, c2_sb_y);
  c2_i1003 = 0;
  for (c2_i1004 = 0; c2_i1004 < 9; c2_i1004++) {
    c2_i1005 = 0;
    for (c2_i1006 = 0; c2_i1006 < 3; c2_i1006++) {
      c2_v_b[c2_i1006 + c2_i1003] = c2_sb_y[c2_i1005 + c2_i1004];
      c2_i1005 += 9;
    }

    c2_i1003 += 3;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1007 = 0; c2_i1007 < 3; c2_i1007++) {
    c2_i1008 = 0;
    for (c2_i1009 = 0; c2_i1009 < 9; c2_i1009++) {
      c2_wc_y[c2_i1008 + c2_i1007] = 0.0;
      c2_i1010 = 0;
      for (c2_i1011 = 0; c2_i1011 < 3; c2_i1011++) {
        c2_wc_y[c2_i1008 + c2_i1007] += c2_d_b[c2_i1010 + c2_i1007] *
          c2_v_b[c2_i1011 + c2_i1008];
        c2_i1010 += 3;
      }

      c2_i1008 += 3;
    }
  }

  for (c2_i1012 = 0; c2_i1012 < 27; c2_i1012++) {
    c2_r_b[c2_i1012] = c2_Dr3_R0[c2_i1012];
  }

  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1013 = 0; c2_i1013 < 3; c2_i1013++) {
    c2_i1014 = 0;
    c2_i1015 = 0;
    for (c2_i1016 = 0; c2_i1016 < 3; c2_i1016++) {
      c2_yb_y[c2_i1014 + c2_i1013] = 0.0;
      c2_i1017 = 0;
      for (c2_i1018 = 0; c2_i1018 < 9; c2_i1018++) {
        c2_yb_y[c2_i1014 + c2_i1013] += c2_wc_y[c2_i1017 + c2_i1013] *
          c2_r_b[c2_i1018 + c2_i1015];
        c2_i1017 += 3;
      }

      c2_i1014 += 3;
      c2_i1015 += 9;
    }
  }

  for (c2_i1019 = 0; c2_i1019 < 3; c2_i1019++) {
    c2_b[c2_i1019] = c2_hat_dot_r3[c2_i1019];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i1020 = 0; c2_i1020 < 3; c2_i1020++) {
    c2_hat_w0[c2_i1020] = 0.0;
  }

  for (c2_i1021 = 0; c2_i1021 < 3; c2_i1021++) {
    c2_hat_w0[c2_i1021] = 0.0;
  }

  for (c2_i1022 = 0; c2_i1022 < 3; c2_i1022++) {
    c2_C[c2_i1022] = c2_hat_w0[c2_i1022];
  }

  for (c2_i1023 = 0; c2_i1023 < 3; c2_i1023++) {
    c2_hat_w0[c2_i1023] = c2_C[c2_i1023];
  }

  c2_threshold(chartInstance);
  for (c2_i1024 = 0; c2_i1024 < 3; c2_i1024++) {
    c2_C[c2_i1024] = c2_hat_w0[c2_i1024];
  }

  for (c2_i1025 = 0; c2_i1025 < 3; c2_i1025++) {
    c2_hat_w0[c2_i1025] = c2_C[c2_i1025];
  }

  for (c2_i1026 = 0; c2_i1026 < 3; c2_i1026++) {
    c2_hat_w0[c2_i1026] = 0.0;
    c2_i1027 = 0;
    for (c2_i1028 = 0; c2_i1028 < 3; c2_i1028++) {
      c2_hat_w0[c2_i1026] += c2_yb_y[c2_i1027 + c2_i1026] * c2_b[c2_i1028];
      c2_i1027 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 194U);
  for (c2_i1029 = 0; c2_i1029 < 3; c2_i1029++) {
    c2_b_hat_w0[c2_i1029] = c2_hat_w0[c2_i1029];
  }

  c2_S(chartInstance, c2_b_hat_w0, c2_e_a);
  for (c2_i1030 = 0; c2_i1030 < 3; c2_i1030++) {
    c2_b[c2_i1030] = c2_hat_w02[c2_i1030];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1031 = 0; c2_i1031 < 3; c2_i1031++) {
    c2_l_y[c2_i1031] = 0.0;
    c2_i1032 = 0;
    for (c2_i1033 = 0; c2_i1033 < 3; c2_i1033++) {
      c2_l_y[c2_i1031] += c2_e_a[c2_i1032 + c2_i1031] * c2_b[c2_i1033];
      c2_i1032 += 3;
    }
  }

  c2_i1034 = 0;
  for (c2_i1035 = 0; c2_i1035 < 3; c2_i1035++) {
    c2_i1036 = 0;
    for (c2_i1037 = 0; c2_i1037 < 3; c2_i1037++) {
      c2_d_b[c2_i1037 + c2_i1034] = c2_R0[c2_i1036 + c2_i1035];
      c2_i1036 += 3;
    }

    c2_i1034 += 3;
  }

  for (c2_i1038 = 0; c2_i1038 < 9; c2_i1038++) {
    c2_d_b[c2_i1038] *= 0.5;
  }

  for (c2_i1039 = 0; c2_i1039 < 27; c2_i1039++) {
    c2_sb_y[c2_i1039] = c2_Dr3_R0[c2_i1039];
  }

  for (c2_i1040 = 0; c2_i1040 < 3; c2_i1040++) {
    c2_b[c2_i1040] = c2_hat_dot_r3[c2_i1040];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1041 = 0; c2_i1041 < 9; c2_i1041++) {
    c2_g_x[c2_i1041] = 0.0;
    c2_i1042 = 0;
    for (c2_i1043 = 0; c2_i1043 < 3; c2_i1043++) {
      c2_g_x[c2_i1041] += c2_sb_y[c2_i1042 + c2_i1041] * c2_b[c2_i1043];
      c2_i1042 += 9;
    }
  }

  c2_eml_switch_helper(chartInstance);
  for (c2_vb_k = 1; c2_vb_k < 10; c2_vb_k++) {
    c2_wb_k = c2_vb_k;
    c2_yb_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_wb_k), 1, 9, 1, 0) - 1] = c2_g_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_wb_k), 1, 9, 1, 0) - 1];
  }

  for (c2_i1044 = 0; c2_i1044 < 9; c2_i1044++) {
    c2_og_y[c2_i1044] = c2_yb_y[c2_i1044];
  }

  c2_Gamma(chartInstance, c2_og_y, c2_sb_y);
  c2_i1045 = 0;
  for (c2_i1046 = 0; c2_i1046 < 9; c2_i1046++) {
    c2_i1047 = 0;
    for (c2_i1048 = 0; c2_i1048 < 3; c2_i1048++) {
      c2_v_b[c2_i1048 + c2_i1045] = c2_sb_y[c2_i1047 + c2_i1046];
      c2_i1047 += 9;
    }

    c2_i1045 += 3;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1049 = 0; c2_i1049 < 3; c2_i1049++) {
    c2_i1050 = 0;
    for (c2_i1051 = 0; c2_i1051 < 9; c2_i1051++) {
      c2_wc_y[c2_i1050 + c2_i1049] = 0.0;
      c2_i1052 = 0;
      for (c2_i1053 = 0; c2_i1053 < 3; c2_i1053++) {
        c2_wc_y[c2_i1050 + c2_i1049] += c2_d_b[c2_i1052 + c2_i1049] *
          c2_v_b[c2_i1053 + c2_i1050];
        c2_i1052 += 3;
      }

      c2_i1050 += 3;
    }
  }

  for (c2_i1054 = 0; c2_i1054 < 9; c2_i1054++) {
    c2_e_R0[c2_i1054] = c2_R0[c2_i1054];
  }

  c2_Gamma(chartInstance, c2_e_R0, c2_r_b);
  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1055 = 0; c2_i1055 < 3; c2_i1055++) {
    c2_i1056 = 0;
    c2_i1057 = 0;
    for (c2_i1058 = 0; c2_i1058 < 3; c2_i1058++) {
      c2_yb_y[c2_i1056 + c2_i1055] = 0.0;
      c2_i1059 = 0;
      for (c2_i1060 = 0; c2_i1060 < 9; c2_i1060++) {
        c2_yb_y[c2_i1056 + c2_i1055] += c2_wc_y[c2_i1059 + c2_i1055] *
          c2_r_b[c2_i1060 + c2_i1057];
        c2_i1059 += 3;
      }

      c2_i1056 += 3;
      c2_i1057 += 9;
    }
  }

  for (c2_i1061 = 0; c2_i1061 < 9; c2_i1061++) {
    c2_d_b[c2_i1061] = c2_R0[c2_i1061];
  }

  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1062 = 0; c2_i1062 < 3; c2_i1062++) {
    c2_i1063 = 0;
    for (c2_i1064 = 0; c2_i1064 < 3; c2_i1064++) {
      c2_i_hoistedGlobal[c2_i1063 + c2_i1062] = 0.0;
      c2_i1065 = 0;
      for (c2_i1066 = 0; c2_i1066 < 3; c2_i1066++) {
        c2_i_hoistedGlobal[c2_i1063 + c2_i1062] += c2_yb_y[c2_i1065 + c2_i1062] *
          c2_d_b[c2_i1066 + c2_i1063];
        c2_i1065 += 3;
      }

      c2_i1063 += 3;
    }
  }

  for (c2_i1067 = 0; c2_i1067 < 3; c2_i1067++) {
    c2_b[c2_i1067] = c2_hat_w02[c2_i1067];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1068 = 0; c2_i1068 < 3; c2_i1068++) {
    c2_sc_y[c2_i1068] = 0.0;
    c2_i1069 = 0;
    for (c2_i1070 = 0; c2_i1070 < 3; c2_i1070++) {
      c2_sc_y[c2_i1068] += c2_i_hoistedGlobal[c2_i1069 + c2_i1068] *
        c2_b[c2_i1070];
      c2_i1069 += 3;
    }
  }

  c2_i1071 = 0;
  for (c2_i1072 = 0; c2_i1072 < 3; c2_i1072++) {
    c2_i1073 = 0;
    for (c2_i1074 = 0; c2_i1074 < 3; c2_i1074++) {
      c2_d_b[c2_i1074 + c2_i1071] = c2_R0[c2_i1073 + c2_i1072];
      c2_i1073 += 3;
    }

    c2_i1071 += 3;
  }

  for (c2_i1075 = 0; c2_i1075 < 9; c2_i1075++) {
    c2_d_b[c2_i1075] *= 0.5;
  }

  for (c2_i1076 = 0; c2_i1076 < 9; c2_i1076++) {
    c2_f_R0[c2_i1076] = c2_R0[c2_i1076];
  }

  c2_Gamma(chartInstance, c2_f_R0, c2_sb_y);
  c2_i1077 = 0;
  for (c2_i1078 = 0; c2_i1078 < 9; c2_i1078++) {
    c2_i1079 = 0;
    for (c2_i1080 = 0; c2_i1080 < 3; c2_i1080++) {
      c2_v_b[c2_i1080 + c2_i1077] = c2_sb_y[c2_i1079 + c2_i1078];
      c2_i1079 += 9;
    }

    c2_i1077 += 3;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1081 = 0; c2_i1081 < 3; c2_i1081++) {
    c2_i1082 = 0;
    for (c2_i1083 = 0; c2_i1083 < 9; c2_i1083++) {
      c2_wc_y[c2_i1082 + c2_i1081] = 0.0;
      c2_i1084 = 0;
      for (c2_i1085 = 0; c2_i1085 < 3; c2_i1085++) {
        c2_wc_y[c2_i1082 + c2_i1081] += c2_d_b[c2_i1084 + c2_i1081] *
          c2_v_b[c2_i1085 + c2_i1082];
        c2_i1084 += 3;
      }

      c2_i1082 += 3;
    }
  }

  for (c2_i1086 = 0; c2_i1086 < 3; c2_i1086++) {
    c2_b_hat_dot_r3[c2_i1086] = c2_hat_dot_r3[c2_i1086];
  }

  for (c2_i1087 = 0; c2_i1087 < 81; c2_i1087++) {
    c2_dv40[c2_i1087] = c2_dv27[c2_i1087];
  }

  c2_j_kron(chartInstance, c2_b_hat_dot_r3, c2_dv40, c2_sc_a);
  for (c2_i1088 = 0; c2_i1088 < 81; c2_i1088++) {
    c2_fc_b[c2_i1088] = c2_D2r3_R0[c2_i1088];
  }

  c2_eb_eml_scalar_eg(chartInstance);
  c2_eb_eml_scalar_eg(chartInstance);
  for (c2_i1089 = 0; c2_i1089 < 27; c2_i1089++) {
    c2_ec_y[c2_i1089] = 0.0;
  }

  for (c2_i1090 = 0; c2_i1090 < 243; c2_i1090++) {
    c2_tc_a[c2_i1090] = c2_sc_a[c2_i1090];
  }

  for (c2_i1091 = 0; c2_i1091 < 81; c2_i1091++) {
    c2_oc_b[c2_i1091] = c2_fc_b[c2_i1091];
  }

  c2_l_eml_xgemm(chartInstance, c2_tc_a, c2_oc_b, c2_ec_y);
  for (c2_i1092 = 0; c2_i1092 < 3; c2_i1092++) {
    c2_b[c2_i1092] = c2_hat_dot_r32[c2_i1092];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1093 = 0; c2_i1093 < 9; c2_i1093++) {
    c2_g_x[c2_i1093] = 0.0;
    c2_i1094 = 0;
    for (c2_i1095 = 0; c2_i1095 < 3; c2_i1095++) {
      c2_g_x[c2_i1093] += c2_ec_y[c2_i1094 + c2_i1093] * c2_b[c2_i1095];
      c2_i1094 += 9;
    }
  }

  for (c2_i1096 = 0; c2_i1096 < 27; c2_i1096++) {
    c2_sb_y[c2_i1096] = c2_Dr3_R0[c2_i1096];
  }

  for (c2_i1097 = 0; c2_i1097 < 3; c2_i1097++) {
    c2_b[c2_i1097] = c2_hat_dot_hat_dot_r32[c2_i1097];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1098 = 0; c2_i1098 < 9; c2_i1098++) {
    c2_pg_y[c2_i1098] = 0.0;
    c2_i1099 = 0;
    for (c2_i1100 = 0; c2_i1100 < 3; c2_i1100++) {
      c2_pg_y[c2_i1098] += c2_sb_y[c2_i1099 + c2_i1098] * c2_b[c2_i1100];
      c2_i1099 += 9;
    }
  }

  for (c2_i1101 = 0; c2_i1101 < 9; c2_i1101++) {
    c2_g_x[c2_i1101] += c2_pg_y[c2_i1101];
  }

  c2_fb_eml_scalar_eg(chartInstance);
  c2_fb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1102 = 0; c2_i1102 < 3; c2_i1102++) {
    c2_C[c2_i1102] = 0.0;
    c2_i1103 = 0;
    for (c2_i1104 = 0; c2_i1104 < 9; c2_i1104++) {
      c2_C[c2_i1102] += c2_wc_y[c2_i1103 + c2_i1102] * c2_g_x[c2_i1104];
      c2_i1103 += 3;
    }
  }

  for (c2_i1105 = 0; c2_i1105 < 3; c2_i1105++) {
    c2_hat_dot_hat_w0[c2_i1105] = (c2_l_y[c2_i1105] - c2_sc_y[c2_i1105]) -
      c2_C[c2_i1105];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 205U);
  c2_q_hoistedGlobal = c2_get_k(chartInstance, 0);
  c2_d_A = c2_b_h * c2_b_kz * c2_b_kV0 * 2.0;
  c2_ib_B = c2_q_hoistedGlobal;
  c2_k_x = c2_d_A;
  c2_qg_y = c2_ib_B;
  c2_l_x = c2_k_x;
  c2_rg_y = c2_qg_y;
  c2_m_x = c2_l_x;
  c2_sg_y = c2_rg_y;
  c2_tg_y = c2_m_x / c2_sg_y;
  for (c2_i1106 = 0; c2_i1106 < 6; c2_i1106++) {
    c2_g_b[c2_i1106] = c2_grad_bar_V0[c2_i1106];
  }

  c2_i_eml_scalar_eg(chartInstance);
  c2_i_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1107 = 0; c2_i1107 < 3; c2_i1107++) {
    c2_l_y[c2_i1107] = 0.0;
    c2_i1108 = 0;
    for (c2_i1109 = 0; c2_i1109 < 6; c2_i1109++) {
      c2_l_y[c2_i1107] += c2_i_b[c2_i1108 + c2_i1107] * c2_g_b[c2_i1109];
      c2_i1108 += 3;
    }
  }

  for (c2_i1110 = 0; c2_i1110 < 3; c2_i1110++) {
    c2_m_mu[c2_i1110] = c2_mu[c2_i1110];
  }

  c2_S(chartInstance, c2_m_mu, c2_dv10);
  for (c2_i1111 = 0; c2_i1111 < 9; c2_i1111++) {
    c2_dv41[c2_i1111] = c2_dv10[c2_i1111];
  }

  for (c2_i1112 = 0; c2_i1112 < 9; c2_i1112++) {
    c2_pc_b[c2_i1112] = c2_k_b[c2_i1112];
  }

  c2_c_kron(chartInstance, c2_dv41, c2_pc_b, c2_p_a);
  c2_k_eml_scalar_eg(chartInstance);
  c2_k_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1113 = 0; c2_i1113 < 9; c2_i1113++) {
    c2_g_x[c2_i1113] = 0.0;
    c2_i1114 = 0;
    for (c2_i1115 = 0; c2_i1115 < 9; c2_i1115++) {
      c2_g_x[c2_i1113] += c2_p_a[c2_i1114 + c2_i1113] * c2_q_b[c2_i1115];
      c2_i1114 += 9;
    }
  }

  for (c2_i1116 = 0; c2_i1116 < 9; c2_i1116++) {
    c2_g_x[c2_i1116] *= 0.5;
  }

  for (c2_i1117 = 0; c2_i1117 < 3; c2_i1117++) {
    c2_q_y[c2_i1117] = c2_q1[c2_i1117 + 1];
  }

  for (c2_i1118 = 0; c2_i1118 < 9; c2_i1118++) {
    c2_i1119 = 0;
    for (c2_i1120 = 0; c2_i1120 < 3; c2_i1120++) {
      c2_ec_y[c2_i1119 + c2_i1118] = c2_g_x[c2_i1118] * c2_q_y[c2_i1120];
      c2_i1119 += 9;
    }
  }

  for (c2_i1121 = 0; c2_i1121 < 9; c2_i1121++) {
    c2_d_b[c2_i1121] = c2_R0[c2_i1121];
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1122 = 0; c2_i1122 < 9; c2_i1122++) {
    c2_i1123 = 0;
    c2_i1124 = 0;
    for (c2_i1125 = 0; c2_i1125 < 3; c2_i1125++) {
      c2_sb_y[c2_i1123 + c2_i1122] = 0.0;
      c2_i1126 = 0;
      for (c2_i1127 = 0; c2_i1127 < 3; c2_i1127++) {
        c2_sb_y[c2_i1123 + c2_i1122] += c2_ec_y[c2_i1126 + c2_i1122] *
          c2_d_b[c2_i1127 + c2_i1124];
        c2_i1126 += 9;
      }

      c2_i1123 += 9;
      c2_i1124 += 3;
    }
  }

  for (c2_i1128 = 0; c2_i1128 < 3; c2_i1128++) {
    c2_b[c2_i1128] = c2_b_w[c2_i1128] - c2_hat_w02[c2_i1128];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1129 = 0; c2_i1129 < 9; c2_i1129++) {
    c2_g_x[c2_i1129] = 0.0;
    c2_i1130 = 0;
    for (c2_i1131 = 0; c2_i1131 < 3; c2_i1131++) {
      c2_g_x[c2_i1129] += c2_sb_y[c2_i1130 + c2_i1129] * c2_b[c2_i1131];
      c2_i1130 += 9;
    }
  }

  c2_uc_a = c2_q1[0];
  for (c2_i1132 = 0; c2_i1132 < 27; c2_i1132++) {
    c2_ec_y[c2_i1132] = c2_uc_a * c2_kb_b[c2_i1132];
  }

  for (c2_i1133 = 0; c2_i1133 < 3; c2_i1133++) {
    c2_b[c2_i1133] = c2_hat_dot_mu2[c2_i1133];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1134 = 0; c2_i1134 < 9; c2_i1134++) {
    c2_pg_y[c2_i1134] = 0.0;
    c2_i1135 = 0;
    for (c2_i1136 = 0; c2_i1136 < 3; c2_i1136++) {
      c2_pg_y[c2_i1134] += c2_ec_y[c2_i1135 + c2_i1134] * c2_b[c2_i1136];
      c2_i1135 += 9;
    }
  }

  for (c2_i1137 = 0; c2_i1137 < 3; c2_i1137++) {
    c2_c_q1[c2_i1137] = c2_q1[c2_i1137 + 1];
  }

  c2_S(chartInstance, c2_c_q1, c2_dv10);
  for (c2_i1138 = 0; c2_i1138 < 9; c2_i1138++) {
    c2_dv42[c2_i1138] = c2_dv10[c2_i1138];
  }

  for (c2_i1139 = 0; c2_i1139 < 9; c2_i1139++) {
    c2_qc_b[c2_i1139] = c2_k_b[c2_i1139];
  }

  c2_c_kron(chartInstance, c2_dv42, c2_qc_b, c2_p_a);
  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1140 = 0; c2_i1140 < 9; c2_i1140++) {
    c2_i1141 = 0;
    for (c2_i1142 = 0; c2_i1142 < 3; c2_i1142++) {
      c2_ec_y[c2_i1141 + c2_i1140] = 0.0;
      c2_i1143 = 0;
      for (c2_i1144 = 0; c2_i1144 < 9; c2_i1144++) {
        c2_ec_y[c2_i1141 + c2_i1140] += c2_p_a[c2_i1143 + c2_i1140] *
          c2_kb_b[c2_i1144 + c2_i1141];
        c2_i1143 += 9;
      }

      c2_i1141 += 9;
    }
  }

  for (c2_i1145 = 0; c2_i1145 < 3; c2_i1145++) {
    c2_b[c2_i1145] = c2_hat_dot_mu2[c2_i1145];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1146 = 0; c2_i1146 < 9; c2_i1146++) {
    c2_ug_y[c2_i1146] = 0.0;
    c2_i1147 = 0;
    for (c2_i1148 = 0; c2_i1148 < 3; c2_i1148++) {
      c2_ug_y[c2_i1146] += c2_ec_y[c2_i1147 + c2_i1146] * c2_b[c2_i1148];
      c2_i1147 += 9;
    }
  }

  for (c2_i1149 = 0; c2_i1149 < 3; c2_i1149++) {
    c2_n_mu[c2_i1149] = c2_mu[c2_i1149];
  }

  c2_S(chartInstance, c2_n_mu, c2_dv10);
  for (c2_i1150 = 0; c2_i1150 < 9; c2_i1150++) {
    c2_rc_b[c2_i1150] = c2_k_b[c2_i1150];
  }

  for (c2_i1151 = 0; c2_i1151 < 9; c2_i1151++) {
    c2_dv43[c2_i1151] = c2_dv10[c2_i1151];
  }

  c2_c_kron(chartInstance, c2_rc_b, c2_dv43, c2_p_a);
  for (c2_i1152 = 0; c2_i1152 < 81; c2_i1152++) {
    c2_p_a[c2_i1152] = -c2_p_a[c2_i1152];
  }

  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1153 = 0; c2_i1153 < 9; c2_i1153++) {
    c2_i1154 = 0;
    for (c2_i1155 = 0; c2_i1155 < 3; c2_i1155++) {
      c2_ec_y[c2_i1154 + c2_i1153] = 0.0;
      c2_i1156 = 0;
      for (c2_i1157 = 0; c2_i1157 < 9; c2_i1157++) {
        c2_ec_y[c2_i1154 + c2_i1153] += c2_p_a[c2_i1156 + c2_i1153] *
          c2_kb_b[c2_i1157 + c2_i1154];
        c2_i1156 += 9;
      }

      c2_i1154 += 9;
    }
  }

  for (c2_i1158 = 0; c2_i1158 < 27; c2_i1158++) {
    c2_ec_y[c2_i1158] *= 0.5;
  }

  c2_vc_a = c2_q1[0];
  for (c2_i1159 = 0; c2_i1159 < 9; c2_i1159++) {
    c2_yb_y[c2_i1159] = c2_vc_a * c2_k_b[c2_i1159];
  }

  for (c2_i1160 = 0; c2_i1160 < 3; c2_i1160++) {
    c2_d_q1[c2_i1160] = c2_q1[c2_i1160 + 1];
  }

  c2_S(chartInstance, c2_d_q1, c2_dv10);
  for (c2_i1161 = 0; c2_i1161 < 9; c2_i1161++) {
    c2_yb_y[c2_i1161] += c2_dv10[c2_i1161];
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1162 = 0; c2_i1162 < 9; c2_i1162++) {
    c2_i1163 = 0;
    c2_i1164 = 0;
    for (c2_i1165 = 0; c2_i1165 < 3; c2_i1165++) {
      c2_sb_y[c2_i1163 + c2_i1162] = 0.0;
      c2_i1166 = 0;
      for (c2_i1167 = 0; c2_i1167 < 3; c2_i1167++) {
        c2_sb_y[c2_i1163 + c2_i1162] += c2_ec_y[c2_i1166 + c2_i1162] *
          c2_yb_y[c2_i1167 + c2_i1164];
        c2_i1166 += 9;
      }

      c2_i1163 += 9;
      c2_i1164 += 3;
    }
  }

  for (c2_i1168 = 0; c2_i1168 < 9; c2_i1168++) {
    c2_d_b[c2_i1168] = c2_R0[c2_i1168];
  }

  c2_j_eml_scalar_eg(chartInstance);
  c2_j_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1169 = 0; c2_i1169 < 9; c2_i1169++) {
    c2_i1170 = 0;
    c2_i1171 = 0;
    for (c2_i1172 = 0; c2_i1172 < 3; c2_i1172++) {
      c2_ec_y[c2_i1170 + c2_i1169] = 0.0;
      c2_i1173 = 0;
      for (c2_i1174 = 0; c2_i1174 < 3; c2_i1174++) {
        c2_ec_y[c2_i1170 + c2_i1169] += c2_sb_y[c2_i1173 + c2_i1169] *
          c2_d_b[c2_i1174 + c2_i1171];
        c2_i1173 += 9;
      }

      c2_i1170 += 9;
      c2_i1171 += 3;
    }
  }

  for (c2_i1175 = 0; c2_i1175 < 3; c2_i1175++) {
    c2_b[c2_i1175] = c2_b_w[c2_i1175] - c2_hat_w02[c2_i1175];
  }

  c2_cb_eml_scalar_eg(chartInstance);
  c2_cb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1176 = 0; c2_i1176 < 9; c2_i1176++) {
    c2_vg_y[c2_i1176] = 0.0;
    c2_i1177 = 0;
    for (c2_i1178 = 0; c2_i1178 < 3; c2_i1178++) {
      c2_vg_y[c2_i1176] += c2_ec_y[c2_i1177 + c2_i1176] * c2_b[c2_i1178];
      c2_i1177 += 9;
    }
  }

  for (c2_i1179 = 0; c2_i1179 < 3; c2_i1179++) {
    c2_wg_y[c2_i1179] = c2_l_y[c2_i1179];
  }

  for (c2_i1180 = 0; c2_i1180 < 9; c2_i1180++) {
    c2_sc_b[c2_i1180] = c2_k_b[c2_i1180];
  }

  c2_e_kron(chartInstance, c2_wg_y, c2_sc_b, c2_v_b);
  for (c2_i1181 = 0; c2_i1181 < 9; c2_i1181++) {
    c2_g_x[c2_i1181] = ((c2_g_x[c2_i1181] - c2_pg_y[c2_i1181]) -
                        c2_ug_y[c2_i1181]) - c2_vg_y[c2_i1181];
  }

  c2_fb_eml_scalar_eg(chartInstance);
  c2_fb_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1182 = 0; c2_i1182 < 3; c2_i1182++) {
    c2_l_y[c2_i1182] = 0.0;
    c2_i1183 = 0;
    for (c2_i1184 = 0; c2_i1184 < 9; c2_i1184++) {
      c2_l_y[c2_i1182] += c2_v_b[c2_i1183 + c2_i1182] * c2_g_x[c2_i1184];
      c2_i1183 += 3;
    }
  }

  c2_wc_a = c2_q1[0];
  for (c2_i1185 = 0; c2_i1185 < 3; c2_i1185++) {
    c2_o_mu[c2_i1185] = c2_mu[c2_i1185];
  }

  c2_S(chartInstance, c2_o_mu, c2_d_b);
  for (c2_i1186 = 0; c2_i1186 < 9; c2_i1186++) {
    c2_d_b[c2_i1186] *= c2_wc_a;
  }

  for (c2_i1187 = 0; c2_i1187 < 3; c2_i1187++) {
    c2_p_mu[c2_i1187] = c2_mu[c2_i1187];
  }

  c2_S(chartInstance, c2_p_mu, c2_e_a);
  for (c2_i1188 = 0; c2_i1188 < 3; c2_i1188++) {
    c2_e_q1[c2_i1188] = c2_q1[c2_i1188 + 1];
  }

  c2_S(chartInstance, c2_e_q1, c2_i_hoistedGlobal);
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1189 = 0; c2_i1189 < 3; c2_i1189++) {
    c2_i1190 = 0;
    for (c2_i1191 = 0; c2_i1191 < 3; c2_i1191++) {
      c2_yb_y[c2_i1190 + c2_i1189] = 0.0;
      c2_i1192 = 0;
      for (c2_i1193 = 0; c2_i1193 < 3; c2_i1193++) {
        c2_yb_y[c2_i1190 + c2_i1189] += c2_e_a[c2_i1192 + c2_i1189] *
          c2_i_hoistedGlobal[c2_i1193 + c2_i1190];
        c2_i1192 += 3;
      }

      c2_i1190 += 3;
    }
  }

  for (c2_i1194 = 0; c2_i1194 < 9; c2_i1194++) {
    c2_d_b[c2_i1194] -= c2_yb_y[c2_i1194];
  }

  c2_m_eml_scalar_eg(chartInstance);
  c2_m_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1195 = 0; c2_i1195 < 3; c2_i1195++) {
    c2_i1196 = 0;
    for (c2_i1197 = 0; c2_i1197 < 6; c2_i1197++) {
      c2_lb_y[c2_i1196 + c2_i1195] = 0.0;
      c2_i1198 = 0;
      for (c2_i1199 = 0; c2_i1199 < 3; c2_i1199++) {
        c2_lb_y[c2_i1196 + c2_i1195] += c2_d_b[c2_i1198 + c2_i1195] *
          c2_i_b[c2_i1199 + c2_i1196];
        c2_i1198 += 3;
      }

      c2_i1196 += 3;
    }
  }

  for (c2_i1200 = 0; c2_i1200 < 36; c2_i1200++) {
    c2_db_b[c2_i1200] = c2_hess_bar_V0[c2_i1200];
  }

  c2_u_eml_scalar_eg(chartInstance);
  c2_u_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1201 = 0; c2_i1201 < 3; c2_i1201++) {
    c2_i1202 = 0;
    c2_i1203 = 0;
    for (c2_i1204 = 0; c2_i1204 < 6; c2_i1204++) {
      c2_qd_y[c2_i1202 + c2_i1201] = 0.0;
      c2_i1205 = 0;
      for (c2_i1206 = 0; c2_i1206 < 6; c2_i1206++) {
        c2_qd_y[c2_i1202 + c2_i1201] += c2_lb_y[c2_i1205 + c2_i1201] *
          c2_db_b[c2_i1206 + c2_i1203];
        c2_i1205 += 3;
      }

      c2_i1202 += 3;
      c2_i1203 += 6;
    }
  }

  for (c2_i1207 = 0; c2_i1207 < 3; c2_i1207++) {
    c2_g_b[c2_i1207] = c2_v0[c2_i1207];
  }

  for (c2_i1208 = 0; c2_i1208 < 3; c2_i1208++) {
    c2_g_b[c2_i1208 + 3] = c2_hat_dot_v2[c2_i1208] - c2_dot_vd[c2_i1208];
  }

  c2_i_eml_scalar_eg(chartInstance);
  c2_i_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1209 = 0; c2_i1209 < 3; c2_i1209++) {
    c2_sc_y[c2_i1209] = 0.0;
    c2_i1210 = 0;
    for (c2_i1211 = 0; c2_i1211 < 6; c2_i1211++) {
      c2_sc_y[c2_i1209] += c2_qd_y[c2_i1210 + c2_i1209] * c2_g_b[c2_i1211];
      c2_i1210 += 3;
    }
  }

  c2_xc_a = -c2_tg_y;
  for (c2_i1212 = 0; c2_i1212 < 3; c2_i1212++) {
    c2_l_y[c2_i1212] += c2_sc_y[c2_i1212];
  }

  for (c2_i1213 = 0; c2_i1213 < 3; c2_i1213++) {
    c2_l_y[c2_i1213] *= c2_xc_a;
  }

  c2_yc_a = c2_q1[0];
  for (c2_i1214 = 0; c2_i1214 < 9; c2_i1214++) {
    c2_yb_y[c2_i1214] = c2_yc_a * c2_k_b[c2_i1214];
  }

  c2_ad_a = 0.5 * c2_b_h * c2_b_kq;
  for (c2_i1215 = 0; c2_i1215 < 3; c2_i1215++) {
    c2_f_q1[c2_i1215] = c2_q1[c2_i1215 + 1];
  }

  c2_S(chartInstance, c2_f_q1, c2_dv10);
  for (c2_i1216 = 0; c2_i1216 < 9; c2_i1216++) {
    c2_yb_y[c2_i1216] += c2_dv10[c2_i1216];
  }

  for (c2_i1217 = 0; c2_i1217 < 9; c2_i1217++) {
    c2_yb_y[c2_i1217] *= c2_ad_a;
  }

  for (c2_i1218 = 0; c2_i1218 < 9; c2_i1218++) {
    c2_d_b[c2_i1218] = c2_R0[c2_i1218];
  }

  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1219 = 0; c2_i1219 < 3; c2_i1219++) {
    c2_i1220 = 0;
    for (c2_i1221 = 0; c2_i1221 < 3; c2_i1221++) {
      c2_i_hoistedGlobal[c2_i1220 + c2_i1219] = 0.0;
      c2_i1222 = 0;
      for (c2_i1223 = 0; c2_i1223 < 3; c2_i1223++) {
        c2_i_hoistedGlobal[c2_i1220 + c2_i1219] += c2_yb_y[c2_i1222 + c2_i1219] *
          c2_d_b[c2_i1223 + c2_i1220];
        c2_i1222 += 3;
      }

      c2_i1220 += 3;
    }
  }

  for (c2_i1224 = 0; c2_i1224 < 3; c2_i1224++) {
    c2_b[c2_i1224] = c2_b_w[c2_i1224] - c2_hat_w02[c2_i1224];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1225 = 0; c2_i1225 < 3; c2_i1225++) {
    c2_sc_y[c2_i1225] = 0.0;
    c2_i1226 = 0;
    for (c2_i1227 = 0; c2_i1227 < 3; c2_i1227++) {
      c2_sc_y[c2_i1225] += c2_i_hoistedGlobal[c2_i1226 + c2_i1225] *
        c2_b[c2_i1227];
      c2_i1226 += 3;
    }
  }

  for (c2_i1228 = 0; c2_i1228 < 3; c2_i1228++) {
    c2_hat_dot_w1[c2_i1228] = c2_l_y[c2_i1228] - c2_sc_y[c2_i1228];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 221U);
  c2_i1229 = 0;
  for (c2_i1230 = 0; c2_i1230 < 3; c2_i1230++) {
    c2_i1231 = 0;
    for (c2_i1232 = 0; c2_i1232 < 3; c2_i1232++) {
      c2_e_a[c2_i1232 + c2_i1229] = c2_R0[c2_i1231 + c2_i1230];
      c2_i1231 += 3;
    }

    c2_i1229 += 3;
  }

  for (c2_i1233 = 0; c2_i1233 < 3; c2_i1233++) {
    c2_b[c2_i1233] = c2_hat_dot_w1[c2_i1233];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1234 = 0; c2_i1234 < 3; c2_i1234++) {
    c2_l_y[c2_i1234] = 0.0;
    c2_i1235 = 0;
    for (c2_i1236 = 0; c2_i1236 < 3; c2_i1236++) {
      c2_l_y[c2_i1234] += c2_e_a[c2_i1235 + c2_i1234] * c2_b[c2_i1236];
      c2_i1235 += 3;
    }
  }

  for (c2_i1237 = 0; c2_i1237 < 3; c2_i1237++) {
    c2_b_w1[c2_i1237] = c2_w1[c2_i1237];
  }

  for (c2_i1238 = 0; c2_i1238 < 9; c2_i1238++) {
    c2_tc_b[c2_i1238] = c2_k_b[c2_i1238];
  }

  c2_e_kron(chartInstance, c2_b_w1, c2_tc_b, c2_v_b);
  c2_i1239 = 0;
  for (c2_i1240 = 0; c2_i1240 < 3; c2_i1240++) {
    c2_i1241 = 0;
    for (c2_i1242 = 0; c2_i1242 < 3; c2_i1242++) {
      c2_g_R0[c2_i1242 + c2_i1239] = c2_R0[c2_i1241 + c2_i1240];
      c2_i1241 += 3;
    }

    c2_i1239 += 3;
  }

  c2_Gamma(chartInstance, c2_g_R0, c2_r_b);
  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1243 = 0; c2_i1243 < 3; c2_i1243++) {
    c2_i1244 = 0;
    c2_i1245 = 0;
    for (c2_i1246 = 0; c2_i1246 < 3; c2_i1246++) {
      c2_yb_y[c2_i1244 + c2_i1243] = 0.0;
      c2_i1247 = 0;
      for (c2_i1248 = 0; c2_i1248 < 9; c2_i1248++) {
        c2_yb_y[c2_i1244 + c2_i1243] += c2_v_b[c2_i1247 + c2_i1243] *
          c2_r_b[c2_i1248 + c2_i1245];
        c2_i1247 += 3;
      }

      c2_i1244 += 3;
      c2_i1245 += 9;
    }
  }

  for (c2_i1249 = 0; c2_i1249 < 3; c2_i1249++) {
    c2_b[c2_i1249] = c2_hat_w02[c2_i1249];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1250 = 0; c2_i1250 < 3; c2_i1250++) {
    c2_sc_y[c2_i1250] = 0.0;
    c2_i1251 = 0;
    for (c2_i1252 = 0; c2_i1252 < 3; c2_i1252++) {
      c2_sc_y[c2_i1250] += c2_yb_y[c2_i1251 + c2_i1250] * c2_b[c2_i1252];
      c2_i1251 += 3;
    }
  }

  for (c2_i1253 = 0; c2_i1253 < 3; c2_i1253++) {
    c2_hat_dot_w_star[c2_i1253] = (c2_hat_dot_hat_w0[c2_i1253] + c2_l_y[c2_i1253])
      + c2_sc_y[c2_i1253];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 222U);
  c2_bd_a = c2_b_kw;
  for (c2_i1254 = 0; c2_i1254 < 3; c2_i1254++) {
    c2_b[c2_i1254] = c2_b_w[c2_i1254] - c2_w_star[c2_i1254];
  }

  for (c2_i1255 = 0; c2_i1255 < 3; c2_i1255++) {
    c2_b[c2_i1255] *= c2_bd_a;
  }

  c2_r_hoistedGlobal = c2_get_k(chartInstance, 0);
  c2_cd_a = c2_r_hoistedGlobal * c2_b_h;
  c2_i1256 = 0;
  for (c2_i1257 = 0; c2_i1257 < 3; c2_i1257++) {
    c2_i1258 = 0;
    for (c2_i1259 = 0; c2_i1259 < 3; c2_i1259++) {
      c2_d_b[c2_i1259 + c2_i1256] = c2_R0[c2_i1258 + c2_i1257];
      c2_i1258 += 3;
    }

    c2_i1256 += 3;
  }

  for (c2_i1260 = 0; c2_i1260 < 9; c2_i1260++) {
    c2_d_b[c2_i1260] *= c2_cd_a;
  }

  for (c2_i1261 = 0; c2_i1261 < 3; c2_i1261++) {
    c2_b_b[c2_i1261] = c2_q1[c2_i1261 + 1];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1262 = 0; c2_i1262 < 3; c2_i1262++) {
    c2_l_y[c2_i1262] = 0.0;
    c2_i1263 = 0;
    for (c2_i1264 = 0; c2_i1264 < 3; c2_i1264++) {
      c2_l_y[c2_i1262] += c2_d_b[c2_i1263 + c2_i1262] * c2_b_b[c2_i1264];
      c2_i1263 += 3;
    }
  }

  for (c2_i1265 = 0; c2_i1265 < 3; c2_i1265++) {
    c2_u2[c2_i1265] = (c2_hat_dot_w_star[c2_i1265] - c2_b[c2_i1265]) -
      c2_l_y[c2_i1265];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 223U);
  for (c2_i1266 = 0; c2_i1266 < 3; c2_i1266++) {
    c2_b_dot_w[c2_i1266] = c2_u2[c2_i1266];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 228U);
  for (c2_i1267 = 0; c2_i1267 < 3; c2_i1267++) {
    c2_b_dot_hat_b2[c2_i1267] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 231U);
  c2_b_V2 = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 232U);
  c2_b_dot_V2 = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 235U);
  c2_e_A = c2_T;
  c2_n_x = c2_e_A;
  c2_o_x = c2_n_x;
  c2_p_x = c2_o_x;
  c2_xg_y = c2_p_x / 0.00981;
  c2_f_A = c2_xg_y - 25.584;
  c2_q_x = c2_f_A;
  c2_r_x = c2_q_x;
  c2_s_x = c2_r_x;
  c2_thrust = c2_s_x / 113.3;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 236U);
  c2_g_A = c2_b_w[0];
  c2_t_x = c2_g_A;
  c2_u_x = c2_t_x;
  c2_v_x = c2_u_x;
  c2_aileron = c2_v_x / 3.9062;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 237U);
  c2_h_A = c2_b_w[1];
  c2_w_x = c2_h_A;
  c2_x_x = c2_w_x;
  c2_y_x = c2_x_x;
  c2_elevator = c2_y_x / 3.9062;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 238U);
  c2_i_A = c2_b_w[2];
  c2_ab_x = c2_i_A;
  c2_bb_x = c2_ab_x;
  c2_cb_x = c2_bb_x;
  c2_yaw = c2_cb_x / 3.7;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 240U);
  c2_b_cmd[0] = c2_thrust;
  c2_b_cmd[1] = c2_aileron;
  c2_b_cmd[2] = c2_elevator;
  c2_b_cmd[3] = c2_yaw;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 242U);
  c2_b_dot_h = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -242);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i1268 = 0; c2_i1268 < 4; c2_i1268++) {
    (*chartInstance->c2_cmd)[c2_i1268] = c2_b_cmd[c2_i1268];
  }

  for (c2_i1269 = 0; c2_i1269 < 3; c2_i1269++) {
    (*chartInstance->c2_dot_z)[c2_i1269] = c2_b_dot_z[c2_i1269];
  }

  for (c2_i1270 = 0; c2_i1270 < 3; c2_i1270++) {
    (*chartInstance->c2_dot_hat_b)[c2_i1270] = c2_b_dot_hat_b[c2_i1270];
  }

  for (c2_i1271 = 0; c2_i1271 < 3; c2_i1271++) {
    (*chartInstance->c2_dot_hat_b2)[c2_i1271] = c2_b_dot_hat_b2[c2_i1271];
  }

  for (c2_i1272 = 0; c2_i1272 < 3; c2_i1272++) {
    (*chartInstance->c2_dot_w)[c2_i1272] = c2_b_dot_w[c2_i1272];
  }

  *chartInstance->c2_dot_h = c2_b_dot_h;
  for (c2_i1273 = 0; c2_i1273 < 4; c2_i1273++) {
    (*chartInstance->c2_dot_qh)[c2_i1273] = c2_b_dot_qh[c2_i1273];
  }

  *chartInstance->c2_V2 = c2_b_V2;
  *chartInstance->c2_dot_V2 = c2_b_dot_V2;
  for (c2_i1274 = 0; c2_i1274 < 4; c2_i1274++) {
    (*chartInstance->c2_q0)[c2_i1274] = c2_b_q0[c2_i1274];
  }

  for (c2_i1275 = 0; c2_i1275 < 4; c2_i1275++) {
    (*chartInstance->c2_q)[c2_i1275] = c2_b_q[c2_i1275];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_PackageDeliverySim
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, c2_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\dlavell\\Documents\\MATLAB\\hybridQuadSim\\controllers\\models\\vec.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1276;
  real_T c2_b_inData[4];
  int32_T c2_i1277;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1276 = 0; c2_i1276 < 4; c2_i1276++) {
    c2_b_inData[c2_i1276] = (*(real_T (*)[4])c2_inData)[c2_i1276];
  }

  for (c2_i1277 = 0; c2_i1277 < 4; c2_i1277++) {
    c2_u[c2_i1277] = c2_b_inData[c2_i1277];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_transpose;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i1278;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_transpose = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_transpose), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_transpose);
  for (c2_i1278 = 0; c2_i1278 < 4; c2_i1278++) {
    (*(real_T (*)[4])c2_outData)[c2_i1278] = c2_y[c2_i1278];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_dot_V2, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dot_V2),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dot_V2);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d4;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d4, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d4;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_dot_V2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_b_dot_V2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dot_V2),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dot_V2);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1279;
  real_T c2_b_inData[3];
  int32_T c2_i1280;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1279 = 0; c2_i1279 < 3; c2_i1279++) {
    c2_b_inData[c2_i1279] = (*(real_T (*)[3])c2_inData)[c2_i1279];
  }

  for (c2_i1280 = 0; c2_i1280 < 3; c2_i1280++) {
    c2_u[c2_i1280] = c2_b_inData[c2_i1280];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_dot_w, const char_T *c2_identifier, real_T
  c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dot_w), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_dot_w);
}

static void c2_d_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv44[3];
  int32_T c2_i1281;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv44, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i1281 = 0; c2_i1281 < 3; c2_i1281++) {
    c2_y[c2_i1281] = c2_dv44[c2_i1281];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_dot_w;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i1282;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_b_dot_w = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dot_w), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_dot_w);
  for (c2_i1282 = 0; c2_i1282 < 3; c2_i1282++) {
    (*(real_T (*)[3])c2_outData)[c2_i1282] = c2_y[c2_i1282];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1283;
  real_T c2_b_inData[15];
  int32_T c2_i1284;
  real_T c2_u[15];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1283 = 0; c2_i1283 < 15; c2_i1283++) {
    c2_b_inData[c2_i1283] = (*(real_T (*)[15])c2_inData)[c2_i1283];
  }

  for (c2_i1284 = 0; c2_i1284 < 15; c2_i1284++) {
    c2_u[c2_i1284] = c2_b_inData[c2_i1284];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 15), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1285;
  real_T c2_b_inData[18];
  int32_T c2_i1286;
  real_T c2_u[18];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1285 = 0; c2_i1285 < 18; c2_i1285++) {
    c2_b_inData[c2_i1285] = (*(real_T (*)[18])c2_inData)[c2_i1285];
  }

  for (c2_i1286 = 0; c2_i1286 < 18; c2_i1286++) {
    c2_u[c2_i1286] = c2_b_inData[c2_i1286];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 18), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1287;
  int32_T c2_i1288;
  int32_T c2_i1289;
  real_T c2_b_inData[8];
  int32_T c2_i1290;
  int32_T c2_i1291;
  int32_T c2_i1292;
  real_T c2_u[8];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1287 = 0;
  for (c2_i1288 = 0; c2_i1288 < 2; c2_i1288++) {
    for (c2_i1289 = 0; c2_i1289 < 4; c2_i1289++) {
      c2_b_inData[c2_i1289 + c2_i1287] = (*(real_T (*)[8])c2_inData)[c2_i1289 +
        c2_i1287];
    }

    c2_i1287 += 4;
  }

  c2_i1290 = 0;
  for (c2_i1291 = 0; c2_i1291 < 2; c2_i1291++) {
    for (c2_i1292 = 0; c2_i1292 < 4; c2_i1292++) {
      c2_u[c2_i1292 + c2_i1290] = c2_b_inData[c2_i1292 + c2_i1290];
    }

    c2_i1290 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[8])
{
  real_T c2_dv45[8];
  int32_T c2_i1293;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv45, 1, 0, 0U, 1, 0U, 2, 4, 2);
  for (c2_i1293 = 0; c2_i1293 < 8; c2_i1293++) {
    c2_y[c2_i1293] = c2_dv45[c2_i1293];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_r;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[8];
  int32_T c2_i1294;
  int32_T c2_i1295;
  int32_T c2_i1296;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_r = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_r), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_r);
  c2_i1294 = 0;
  for (c2_i1295 = 0; c2_i1295 < 2; c2_i1295++) {
    for (c2_i1296 = 0; c2_i1296 < 4; c2_i1296++) {
      (*(real_T (*)[8])c2_outData)[c2_i1296 + c2_i1294] = c2_y[c2_i1296 +
        c2_i1294];
    }

    c2_i1294 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1297;
  int32_T c2_i1298;
  int32_T c2_i1299;
  real_T c2_b_inData[81];
  int32_T c2_i1300;
  int32_T c2_i1301;
  int32_T c2_i1302;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1297 = 0;
  for (c2_i1298 = 0; c2_i1298 < 3; c2_i1298++) {
    for (c2_i1299 = 0; c2_i1299 < 27; c2_i1299++) {
      c2_b_inData[c2_i1299 + c2_i1297] = (*(real_T (*)[81])c2_inData)[c2_i1299 +
        c2_i1297];
    }

    c2_i1297 += 27;
  }

  c2_i1300 = 0;
  for (c2_i1301 = 0; c2_i1301 < 3; c2_i1301++) {
    for (c2_i1302 = 0; c2_i1302 < 27; c2_i1302++) {
      c2_u[c2_i1302 + c2_i1300] = c2_b_inData[c2_i1302 + c2_i1300];
    }

    c2_i1300 += 27;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 27, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81])
{
  real_T c2_dv46[81];
  int32_T c2_i1303;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv46, 1, 0, 0U, 1, 0U, 2, 27,
                3);
  for (c2_i1303 = 0; c2_i1303 < 81; c2_i1303++) {
    c2_y[c2_i1303] = c2_dv46[c2_i1303];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_D2r3_R0;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i1304;
  int32_T c2_i1305;
  int32_T c2_i1306;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_D2r3_R0 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_D2r3_R0), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_D2r3_R0);
  c2_i1304 = 0;
  for (c2_i1305 = 0; c2_i1305 < 3; c2_i1305++) {
    for (c2_i1306 = 0; c2_i1306 < 27; c2_i1306++) {
      (*(real_T (*)[81])c2_outData)[c2_i1306 + c2_i1304] = c2_y[c2_i1306 +
        c2_i1304];
    }

    c2_i1304 += 27;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1307;
  int32_T c2_i1308;
  int32_T c2_i1309;
  real_T c2_b_inData[27];
  int32_T c2_i1310;
  int32_T c2_i1311;
  int32_T c2_i1312;
  real_T c2_u[27];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1307 = 0;
  for (c2_i1308 = 0; c2_i1308 < 3; c2_i1308++) {
    for (c2_i1309 = 0; c2_i1309 < 9; c2_i1309++) {
      c2_b_inData[c2_i1309 + c2_i1307] = (*(real_T (*)[27])c2_inData)[c2_i1309 +
        c2_i1307];
    }

    c2_i1307 += 9;
  }

  c2_i1310 = 0;
  for (c2_i1311 = 0; c2_i1311 < 3; c2_i1311++) {
    for (c2_i1312 = 0; c2_i1312 < 9; c2_i1312++) {
      c2_u[c2_i1312 + c2_i1310] = c2_b_inData[c2_i1312 + c2_i1310];
    }

    c2_i1310 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27])
{
  real_T c2_dv47[27];
  int32_T c2_i1313;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv47, 1, 0, 0U, 1, 0U, 2, 9, 3);
  for (c2_i1313 = 0; c2_i1313 < 27; c2_i1313++) {
    c2_y[c2_i1313] = c2_dv47[c2_i1313];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_H;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[27];
  int32_T c2_i1314;
  int32_T c2_i1315;
  int32_T c2_i1316;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_H = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_H), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_H);
  c2_i1314 = 0;
  for (c2_i1315 = 0; c2_i1315 < 3; c2_i1315++) {
    for (c2_i1316 = 0; c2_i1316 < 9; c2_i1316++) {
      (*(real_T (*)[27])c2_outData)[c2_i1316 + c2_i1314] = c2_y[c2_i1316 +
        c2_i1314];
    }

    c2_i1314 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1317;
  int32_T c2_i1318;
  int32_T c2_i1319;
  real_T c2_b_inData[36];
  int32_T c2_i1320;
  int32_T c2_i1321;
  int32_T c2_i1322;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1317 = 0;
  for (c2_i1318 = 0; c2_i1318 < 6; c2_i1318++) {
    for (c2_i1319 = 0; c2_i1319 < 6; c2_i1319++) {
      c2_b_inData[c2_i1319 + c2_i1317] = (*(real_T (*)[36])c2_inData)[c2_i1319 +
        c2_i1317];
    }

    c2_i1317 += 6;
  }

  c2_i1320 = 0;
  for (c2_i1321 = 0; c2_i1321 < 6; c2_i1321++) {
    for (c2_i1322 = 0; c2_i1322 < 6; c2_i1322++) {
      c2_u[c2_i1322 + c2_i1320] = c2_b_inData[c2_i1322 + c2_i1320];
    }

    c2_i1320 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36])
{
  real_T c2_dv48[36];
  int32_T c2_i1323;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv48, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c2_i1323 = 0; c2_i1323 < 36; c2_i1323++) {
    c2_y[c2_i1323] = c2_dv48[c2_i1323];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_hess_bar_V0;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i1324;
  int32_T c2_i1325;
  int32_T c2_i1326;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_hess_bar_V0 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_hess_bar_V0), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_hess_bar_V0);
  c2_i1324 = 0;
  for (c2_i1325 = 0; c2_i1325 < 6; c2_i1325++) {
    for (c2_i1326 = 0; c2_i1326 < 6; c2_i1326++) {
      (*(real_T (*)[36])c2_outData)[c2_i1326 + c2_i1324] = c2_y[c2_i1326 +
        c2_i1324];
    }

    c2_i1324 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1327;
  int32_T c2_i1328;
  int32_T c2_i1329;
  real_T c2_b_inData[9];
  int32_T c2_i1330;
  int32_T c2_i1331;
  int32_T c2_i1332;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1327 = 0;
  for (c2_i1328 = 0; c2_i1328 < 3; c2_i1328++) {
    for (c2_i1329 = 0; c2_i1329 < 3; c2_i1329++) {
      c2_b_inData[c2_i1329 + c2_i1327] = (*(real_T (*)[9])c2_inData)[c2_i1329 +
        c2_i1327];
    }

    c2_i1327 += 3;
  }

  c2_i1330 = 0;
  for (c2_i1331 = 0; c2_i1331 < 3; c2_i1331++) {
    for (c2_i1332 = 0; c2_i1332 < 3; c2_i1332++) {
      c2_u[c2_i1332 + c2_i1330] = c2_b_inData[c2_i1332 + c2_i1330];
    }

    c2_i1330 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv49[9];
  int32_T c2_i1333;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv49, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i1333 = 0; c2_i1333 < 9; c2_i1333++) {
    c2_y[c2_i1333] = c2_dv49[c2_i1333];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_H2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i1334;
  int32_T c2_i1335;
  int32_T c2_i1336;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_H2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_H2), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_H2);
  c2_i1334 = 0;
  for (c2_i1335 = 0; c2_i1335 < 3; c2_i1335++) {
    for (c2_i1336 = 0; c2_i1336 < 3; c2_i1336++) {
      (*(real_T (*)[9])c2_outData)[c2_i1336 + c2_i1334] = c2_y[c2_i1336 +
        c2_i1334];
    }

    c2_i1334 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1337;
  real_T c2_b_inData[1000];
  int32_T c2_i1338;
  real_T c2_u[1000];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1337 = 0; c2_i1337 < 1000; c2_i1337++) {
    c2_b_inData[c2_i1337] = (*(real_T (*)[1000])c2_inData)[c2_i1337];
  }

  for (c2_i1338 = 0; c2_i1338 < 1000; c2_i1338++) {
    c2_u[c2_i1338] = c2_b_inData[c2_i1338];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 1000), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_j_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[1000])
{
  real_T c2_dv50[1000];
  int32_T c2_i1339;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv50, 1, 0, 0U, 1, 0U, 1, 1000);
  for (c2_i1339 = 0; c2_i1339 < 1000; c2_i1339++) {
    c2_y[c2_i1339] = c2_dv50[c2_i1339];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_s;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[1000];
  int32_T c2_i1340;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_s = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_s), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_s);
  for (c2_i1340 = 0; c2_i1340 < 1000; c2_i1340++) {
    (*(real_T (*)[1000])c2_outData)[c2_i1340] = c2_y[c2_i1340];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1341;
  real_T c2_b_inData[6];
  int32_T c2_i1342;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i1341 = 0; c2_i1341 < 6; c2_i1341++) {
    c2_b_inData[c2_i1341] = (*(real_T (*)[6])c2_inData)[c2_i1341];
  }

  for (c2_i1342 = 0; c2_i1342 < 6; c2_i1342++) {
    c2_u[c2_i1342] = c2_b_inData[c2_i1342];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_k_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6])
{
  real_T c2_dv51[6];
  int32_T c2_i1343;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv51, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i1343 = 0; c2_i1343 < 6; c2_i1343++) {
    c2_y[c2_i1343] = c2_dv51[c2_i1343];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_grad_bar_V0;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i1344;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_grad_bar_V0 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_grad_bar_V0), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_grad_bar_V0);
  for (c2_i1344 = 0; c2_i1344 < 6; c2_i1344++) {
    (*(real_T (*)[6])c2_outData)[c2_i1344] = c2_y[c2_i1344];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i1345;
  int32_T c2_i1346;
  int32_T c2_i1347;
  real_T c2_b_inData[4];
  int32_T c2_i1348;
  int32_T c2_i1349;
  int32_T c2_i1350;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i1345 = 0;
  for (c2_i1346 = 0; c2_i1346 < 2; c2_i1346++) {
    for (c2_i1347 = 0; c2_i1347 < 2; c2_i1347++) {
      c2_b_inData[c2_i1347 + c2_i1345] = (*(real_T (*)[4])c2_inData)[c2_i1347 +
        c2_i1345];
    }

    c2_i1345 += 2;
  }

  c2_i1348 = 0;
  for (c2_i1349 = 0; c2_i1349 < 2; c2_i1349++) {
    for (c2_i1350 = 0; c2_i1350 < 2; c2_i1350++) {
      c2_u[c2_i1350 + c2_i1348] = c2_b_inData[c2_i1350 + c2_i1348];
    }

    c2_i1348 += 2;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_l_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv52[4];
  int32_T c2_i1351;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv52, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c2_i1351 = 0; c2_i1351 < 4; c2_i1351++) {
    c2_y[c2_i1351] = c2_dv52[c2_i1351];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_P;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i1352;
  int32_T c2_i1353;
  int32_T c2_i1354;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_P = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_P), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_P);
  c2_i1352 = 0;
  for (c2_i1353 = 0; c2_i1353 < 2; c2_i1353++) {
    for (c2_i1354 = 0; c2_i1354 < 2; c2_i1354++) {
      (*(real_T (*)[4])c2_outData)[c2_i1354 + c2_i1352] = c2_y[c2_i1354 +
        c2_i1352];
    }

    c2_i1352 += 2;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_PackageDeliverySim_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 167, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  c2_b_info_helper(&c2_nameCaptureInfo);
  c2_c_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  const mxArray *c2_rhs42 = NULL;
  const mxArray *c2_lhs42 = NULL;
  const mxArray *c2_rhs43 = NULL;
  const mxArray *c2_lhs43 = NULL;
  const mxArray *c2_rhs44 = NULL;
  const mxArray *c2_lhs44 = NULL;
  const mxArray *c2_rhs45 = NULL;
  const mxArray *c2_lhs45 = NULL;
  const mxArray *c2_rhs46 = NULL;
  const mxArray *c2_lhs46 = NULL;
  const mxArray *c2_rhs47 = NULL;
  const mxArray *c2_lhs47 = NULL;
  const mxArray *c2_rhs48 = NULL;
  const mxArray *c2_lhs48 = NULL;
  const mxArray *c2_rhs49 = NULL;
  const mxArray *c2_lhs49 = NULL;
  const mxArray *c2_rhs50 = NULL;
  const mxArray *c2_lhs50 = NULL;
  const mxArray *c2_rhs51 = NULL;
  const mxArray *c2_lhs51 = NULL;
  const mxArray *c2_rhs52 = NULL;
  const mxArray *c2_lhs52 = NULL;
  const mxArray *c2_rhs53 = NULL;
  const mxArray *c2_lhs53 = NULL;
  const mxArray *c2_rhs54 = NULL;
  const mxArray *c2_lhs54 = NULL;
  const mxArray *c2_rhs55 = NULL;
  const mxArray *c2_lhs55 = NULL;
  const mxArray *c2_rhs56 = NULL;
  const mxArray *c2_lhs56 = NULL;
  const mxArray *c2_rhs57 = NULL;
  const mxArray *c2_lhs57 = NULL;
  const mxArray *c2_rhs58 = NULL;
  const mxArray *c2_lhs58 = NULL;
  const mxArray *c2_rhs59 = NULL;
  const mxArray *c2_lhs59 = NULL;
  const mxArray *c2_rhs60 = NULL;
  const mxArray *c2_lhs60 = NULL;
  const mxArray *c2_rhs61 = NULL;
  const mxArray *c2_lhs61 = NULL;
  const mxArray *c2_rhs62 = NULL;
  const mxArray *c2_lhs62 = NULL;
  const mxArray *c2_rhs63 = NULL;
  const mxArray *c2_lhs63 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840048U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370042286U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389750174U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742680U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851196U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1386456352U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("reshape"), "name", "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378328382U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!reshape_varargin_to_size"),
                  "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!varargin_nempty"),
                  "context", "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!varargin_nempty"),
                  "context", "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isfi"), "name", "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346542758U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnumerictype"), "name",
                  "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isnumerictype.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1398907998U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1393363258U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1393363258U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368215430U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742656U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851182U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760722U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1393363258U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!reshape_varargin_to_size"),
                  "context", "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383909694U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013090U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c2_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c2_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c2_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c2_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c2_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1393363258U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c2_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c2_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c2_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("max"), "name", "name", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1311287716U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c2_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378328384U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c2_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c2_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c2_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c2_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c2_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c2_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742658U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c2_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c2_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c2_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c2_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c2_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_relop"), "name", "name",
                  62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1342483582U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c2_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("atan"), "name", "name", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "resolved",
                  "resolved", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395357296U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c2_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
  sf_mex_destroy(&c2_rhs42);
  sf_mex_destroy(&c2_lhs42);
  sf_mex_destroy(&c2_rhs43);
  sf_mex_destroy(&c2_lhs43);
  sf_mex_destroy(&c2_rhs44);
  sf_mex_destroy(&c2_lhs44);
  sf_mex_destroy(&c2_rhs45);
  sf_mex_destroy(&c2_lhs45);
  sf_mex_destroy(&c2_rhs46);
  sf_mex_destroy(&c2_lhs46);
  sf_mex_destroy(&c2_rhs47);
  sf_mex_destroy(&c2_lhs47);
  sf_mex_destroy(&c2_rhs48);
  sf_mex_destroy(&c2_lhs48);
  sf_mex_destroy(&c2_rhs49);
  sf_mex_destroy(&c2_lhs49);
  sf_mex_destroy(&c2_rhs50);
  sf_mex_destroy(&c2_lhs50);
  sf_mex_destroy(&c2_rhs51);
  sf_mex_destroy(&c2_lhs51);
  sf_mex_destroy(&c2_rhs52);
  sf_mex_destroy(&c2_lhs52);
  sf_mex_destroy(&c2_rhs53);
  sf_mex_destroy(&c2_lhs53);
  sf_mex_destroy(&c2_rhs54);
  sf_mex_destroy(&c2_lhs54);
  sf_mex_destroy(&c2_rhs55);
  sf_mex_destroy(&c2_lhs55);
  sf_mex_destroy(&c2_rhs56);
  sf_mex_destroy(&c2_lhs56);
  sf_mex_destroy(&c2_rhs57);
  sf_mex_destroy(&c2_lhs57);
  sf_mex_destroy(&c2_rhs58);
  sf_mex_destroy(&c2_lhs58);
  sf_mex_destroy(&c2_rhs59);
  sf_mex_destroy(&c2_lhs59);
  sf_mex_destroy(&c2_rhs60);
  sf_mex_destroy(&c2_lhs60);
  sf_mex_destroy(&c2_rhs61);
  sf_mex_destroy(&c2_lhs61);
  sf_mex_destroy(&c2_rhs62);
  sf_mex_destroy(&c2_lhs62);
  sf_mex_destroy(&c2_rhs63);
  sf_mex_destroy(&c2_lhs63);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), false);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  return c2_y;
}

static void c2_b_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs64 = NULL;
  const mxArray *c2_lhs64 = NULL;
  const mxArray *c2_rhs65 = NULL;
  const mxArray *c2_lhs65 = NULL;
  const mxArray *c2_rhs66 = NULL;
  const mxArray *c2_lhs66 = NULL;
  const mxArray *c2_rhs67 = NULL;
  const mxArray *c2_lhs67 = NULL;
  const mxArray *c2_rhs68 = NULL;
  const mxArray *c2_lhs68 = NULL;
  const mxArray *c2_rhs69 = NULL;
  const mxArray *c2_lhs69 = NULL;
  const mxArray *c2_rhs70 = NULL;
  const mxArray *c2_lhs70 = NULL;
  const mxArray *c2_rhs71 = NULL;
  const mxArray *c2_lhs71 = NULL;
  const mxArray *c2_rhs72 = NULL;
  const mxArray *c2_lhs72 = NULL;
  const mxArray *c2_rhs73 = NULL;
  const mxArray *c2_lhs73 = NULL;
  const mxArray *c2_rhs74 = NULL;
  const mxArray *c2_lhs74 = NULL;
  const mxArray *c2_rhs75 = NULL;
  const mxArray *c2_lhs75 = NULL;
  const mxArray *c2_rhs76 = NULL;
  const mxArray *c2_lhs76 = NULL;
  const mxArray *c2_rhs77 = NULL;
  const mxArray *c2_lhs77 = NULL;
  const mxArray *c2_rhs78 = NULL;
  const mxArray *c2_lhs78 = NULL;
  const mxArray *c2_rhs79 = NULL;
  const mxArray *c2_lhs79 = NULL;
  const mxArray *c2_rhs80 = NULL;
  const mxArray *c2_lhs80 = NULL;
  const mxArray *c2_rhs81 = NULL;
  const mxArray *c2_lhs81 = NULL;
  const mxArray *c2_rhs82 = NULL;
  const mxArray *c2_lhs82 = NULL;
  const mxArray *c2_rhs83 = NULL;
  const mxArray *c2_lhs83 = NULL;
  const mxArray *c2_rhs84 = NULL;
  const mxArray *c2_lhs84 = NULL;
  const mxArray *c2_rhs85 = NULL;
  const mxArray *c2_lhs85 = NULL;
  const mxArray *c2_rhs86 = NULL;
  const mxArray *c2_lhs86 = NULL;
  const mxArray *c2_rhs87 = NULL;
  const mxArray *c2_lhs87 = NULL;
  const mxArray *c2_rhs88 = NULL;
  const mxArray *c2_lhs88 = NULL;
  const mxArray *c2_rhs89 = NULL;
  const mxArray *c2_lhs89 = NULL;
  const mxArray *c2_rhs90 = NULL;
  const mxArray *c2_lhs90 = NULL;
  const mxArray *c2_rhs91 = NULL;
  const mxArray *c2_lhs91 = NULL;
  const mxArray *c2_rhs92 = NULL;
  const mxArray *c2_lhs92 = NULL;
  const mxArray *c2_rhs93 = NULL;
  const mxArray *c2_lhs93 = NULL;
  const mxArray *c2_rhs94 = NULL;
  const mxArray *c2_lhs94 = NULL;
  const mxArray *c2_rhs95 = NULL;
  const mxArray *c2_lhs95 = NULL;
  const mxArray *c2_rhs96 = NULL;
  const mxArray *c2_lhs96 = NULL;
  const mxArray *c2_rhs97 = NULL;
  const mxArray *c2_lhs97 = NULL;
  const mxArray *c2_rhs98 = NULL;
  const mxArray *c2_lhs98 = NULL;
  const mxArray *c2_rhs99 = NULL;
  const mxArray *c2_lhs99 = NULL;
  const mxArray *c2_rhs100 = NULL;
  const mxArray *c2_lhs100 = NULL;
  const mxArray *c2_rhs101 = NULL;
  const mxArray *c2_lhs101 = NULL;
  const mxArray *c2_rhs102 = NULL;
  const mxArray *c2_lhs102 = NULL;
  const mxArray *c2_rhs103 = NULL;
  const mxArray *c2_lhs103 = NULL;
  const mxArray *c2_rhs104 = NULL;
  const mxArray *c2_lhs104 = NULL;
  const mxArray *c2_rhs105 = NULL;
  const mxArray *c2_lhs105 = NULL;
  const mxArray *c2_rhs106 = NULL;
  const mxArray *c2_lhs106 = NULL;
  const mxArray *c2_rhs107 = NULL;
  const mxArray *c2_lhs107 = NULL;
  const mxArray *c2_rhs108 = NULL;
  const mxArray *c2_lhs108 = NULL;
  const mxArray *c2_rhs109 = NULL;
  const mxArray *c2_lhs109 = NULL;
  const mxArray *c2_rhs110 = NULL;
  const mxArray *c2_lhs110 = NULL;
  const mxArray *c2_rhs111 = NULL;
  const mxArray *c2_lhs111 = NULL;
  const mxArray *c2_rhs112 = NULL;
  const mxArray *c2_lhs112 = NULL;
  const mxArray *c2_rhs113 = NULL;
  const mxArray *c2_lhs113 = NULL;
  const mxArray *c2_rhs114 = NULL;
  const mxArray *c2_lhs114 = NULL;
  const mxArray *c2_rhs115 = NULL;
  const mxArray *c2_lhs115 = NULL;
  const mxArray *c2_rhs116 = NULL;
  const mxArray *c2_lhs116 = NULL;
  const mxArray *c2_rhs117 = NULL;
  const mxArray *c2_lhs117 = NULL;
  const mxArray *c2_rhs118 = NULL;
  const mxArray *c2_lhs118 = NULL;
  const mxArray *c2_rhs119 = NULL;
  const mxArray *c2_lhs119 = NULL;
  const mxArray *c2_rhs120 = NULL;
  const mxArray *c2_lhs120 = NULL;
  const mxArray *c2_rhs121 = NULL;
  const mxArray *c2_lhs121 = NULL;
  const mxArray *c2_rhs122 = NULL;
  const mxArray *c2_lhs122 = NULL;
  const mxArray *c2_rhs123 = NULL;
  const mxArray *c2_lhs123 = NULL;
  const mxArray *c2_rhs124 = NULL;
  const mxArray *c2_lhs124 = NULL;
  const mxArray *c2_rhs125 = NULL;
  const mxArray *c2_lhs125 = NULL;
  const mxArray *c2_rhs126 = NULL;
  const mxArray *c2_lhs126 = NULL;
  const mxArray *c2_rhs127 = NULL;
  const mxArray *c2_lhs127 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_atan"), "name",
                  "name", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851118U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c2_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("norm"), "name", "name", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742668U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c2_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c2_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c2_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013092U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c2_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c2_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c2_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c2_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c2_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c2_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("realmin"), "name", "name", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307683642U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c2_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmin"), "name", "name",
                  75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307683644U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c2_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c2_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c2_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c2_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c2_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c2_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742652U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c2_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c2_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851112U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c2_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eye"), "name", "name", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1406845548U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c2_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368215430U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c2_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c2_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c2_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdotu"), "name", "name",
                  88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013090U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c2_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c2_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xdotu"),
                  "name", "name", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotu.p"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c2_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotu.p"),
                  "context", "context", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c2_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c2_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c2_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c2_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xdotx"),
                  "name", "name", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c2_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c2_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c2_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c2_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c2_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c2_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mpower"), "name", "name", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742678U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c2_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c2_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name",
                  103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331337258U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c2_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c2_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c2_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383909694U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c2_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("linspace"), "name", "name",
                  107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "resolved",
                  "resolved", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381882700U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c2_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c2_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c2_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c2_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c2_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("realmax"), "name", "name", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmax.m"), "resolved",
                  "resolved", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307683642U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c2_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmax.m"), "context",
                  "context", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmax"), "name", "name",
                  113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmax.m"), "resolved",
                  "resolved", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c2_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmax.m"), "context",
                  "context", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c2_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/linspace.m"), "context",
                  "context", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name",
                  115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840048U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370042286U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c2_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("trapz"), "name", "name", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m"), "resolved",
                  "resolved", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851100U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c2_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c2_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c2_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c2_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c2_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360314750U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c2_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360314988U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c2_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360314750U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c2_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360314988U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c2_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c2_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c2_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c2_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c2_rhs64);
  sf_mex_destroy(&c2_lhs64);
  sf_mex_destroy(&c2_rhs65);
  sf_mex_destroy(&c2_lhs65);
  sf_mex_destroy(&c2_rhs66);
  sf_mex_destroy(&c2_lhs66);
  sf_mex_destroy(&c2_rhs67);
  sf_mex_destroy(&c2_lhs67);
  sf_mex_destroy(&c2_rhs68);
  sf_mex_destroy(&c2_lhs68);
  sf_mex_destroy(&c2_rhs69);
  sf_mex_destroy(&c2_lhs69);
  sf_mex_destroy(&c2_rhs70);
  sf_mex_destroy(&c2_lhs70);
  sf_mex_destroy(&c2_rhs71);
  sf_mex_destroy(&c2_lhs71);
  sf_mex_destroy(&c2_rhs72);
  sf_mex_destroy(&c2_lhs72);
  sf_mex_destroy(&c2_rhs73);
  sf_mex_destroy(&c2_lhs73);
  sf_mex_destroy(&c2_rhs74);
  sf_mex_destroy(&c2_lhs74);
  sf_mex_destroy(&c2_rhs75);
  sf_mex_destroy(&c2_lhs75);
  sf_mex_destroy(&c2_rhs76);
  sf_mex_destroy(&c2_lhs76);
  sf_mex_destroy(&c2_rhs77);
  sf_mex_destroy(&c2_lhs77);
  sf_mex_destroy(&c2_rhs78);
  sf_mex_destroy(&c2_lhs78);
  sf_mex_destroy(&c2_rhs79);
  sf_mex_destroy(&c2_lhs79);
  sf_mex_destroy(&c2_rhs80);
  sf_mex_destroy(&c2_lhs80);
  sf_mex_destroy(&c2_rhs81);
  sf_mex_destroy(&c2_lhs81);
  sf_mex_destroy(&c2_rhs82);
  sf_mex_destroy(&c2_lhs82);
  sf_mex_destroy(&c2_rhs83);
  sf_mex_destroy(&c2_lhs83);
  sf_mex_destroy(&c2_rhs84);
  sf_mex_destroy(&c2_lhs84);
  sf_mex_destroy(&c2_rhs85);
  sf_mex_destroy(&c2_lhs85);
  sf_mex_destroy(&c2_rhs86);
  sf_mex_destroy(&c2_lhs86);
  sf_mex_destroy(&c2_rhs87);
  sf_mex_destroy(&c2_lhs87);
  sf_mex_destroy(&c2_rhs88);
  sf_mex_destroy(&c2_lhs88);
  sf_mex_destroy(&c2_rhs89);
  sf_mex_destroy(&c2_lhs89);
  sf_mex_destroy(&c2_rhs90);
  sf_mex_destroy(&c2_lhs90);
  sf_mex_destroy(&c2_rhs91);
  sf_mex_destroy(&c2_lhs91);
  sf_mex_destroy(&c2_rhs92);
  sf_mex_destroy(&c2_lhs92);
  sf_mex_destroy(&c2_rhs93);
  sf_mex_destroy(&c2_lhs93);
  sf_mex_destroy(&c2_rhs94);
  sf_mex_destroy(&c2_lhs94);
  sf_mex_destroy(&c2_rhs95);
  sf_mex_destroy(&c2_lhs95);
  sf_mex_destroy(&c2_rhs96);
  sf_mex_destroy(&c2_lhs96);
  sf_mex_destroy(&c2_rhs97);
  sf_mex_destroy(&c2_lhs97);
  sf_mex_destroy(&c2_rhs98);
  sf_mex_destroy(&c2_lhs98);
  sf_mex_destroy(&c2_rhs99);
  sf_mex_destroy(&c2_lhs99);
  sf_mex_destroy(&c2_rhs100);
  sf_mex_destroy(&c2_lhs100);
  sf_mex_destroy(&c2_rhs101);
  sf_mex_destroy(&c2_lhs101);
  sf_mex_destroy(&c2_rhs102);
  sf_mex_destroy(&c2_lhs102);
  sf_mex_destroy(&c2_rhs103);
  sf_mex_destroy(&c2_lhs103);
  sf_mex_destroy(&c2_rhs104);
  sf_mex_destroy(&c2_lhs104);
  sf_mex_destroy(&c2_rhs105);
  sf_mex_destroy(&c2_lhs105);
  sf_mex_destroy(&c2_rhs106);
  sf_mex_destroy(&c2_lhs106);
  sf_mex_destroy(&c2_rhs107);
  sf_mex_destroy(&c2_lhs107);
  sf_mex_destroy(&c2_rhs108);
  sf_mex_destroy(&c2_lhs108);
  sf_mex_destroy(&c2_rhs109);
  sf_mex_destroy(&c2_lhs109);
  sf_mex_destroy(&c2_rhs110);
  sf_mex_destroy(&c2_lhs110);
  sf_mex_destroy(&c2_rhs111);
  sf_mex_destroy(&c2_lhs111);
  sf_mex_destroy(&c2_rhs112);
  sf_mex_destroy(&c2_lhs112);
  sf_mex_destroy(&c2_rhs113);
  sf_mex_destroy(&c2_lhs113);
  sf_mex_destroy(&c2_rhs114);
  sf_mex_destroy(&c2_lhs114);
  sf_mex_destroy(&c2_rhs115);
  sf_mex_destroy(&c2_lhs115);
  sf_mex_destroy(&c2_rhs116);
  sf_mex_destroy(&c2_lhs116);
  sf_mex_destroy(&c2_rhs117);
  sf_mex_destroy(&c2_lhs117);
  sf_mex_destroy(&c2_rhs118);
  sf_mex_destroy(&c2_lhs118);
  sf_mex_destroy(&c2_rhs119);
  sf_mex_destroy(&c2_lhs119);
  sf_mex_destroy(&c2_rhs120);
  sf_mex_destroy(&c2_lhs120);
  sf_mex_destroy(&c2_rhs121);
  sf_mex_destroy(&c2_lhs121);
  sf_mex_destroy(&c2_rhs122);
  sf_mex_destroy(&c2_lhs122);
  sf_mex_destroy(&c2_rhs123);
  sf_mex_destroy(&c2_lhs123);
  sf_mex_destroy(&c2_rhs124);
  sf_mex_destroy(&c2_lhs124);
  sf_mex_destroy(&c2_rhs125);
  sf_mex_destroy(&c2_lhs125);
  sf_mex_destroy(&c2_rhs126);
  sf_mex_destroy(&c2_lhs126);
  sf_mex_destroy(&c2_rhs127);
  sf_mex_destroy(&c2_lhs127);
}

static void c2_c_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs128 = NULL;
  const mxArray *c2_lhs128 = NULL;
  const mxArray *c2_rhs129 = NULL;
  const mxArray *c2_lhs129 = NULL;
  const mxArray *c2_rhs130 = NULL;
  const mxArray *c2_lhs130 = NULL;
  const mxArray *c2_rhs131 = NULL;
  const mxArray *c2_lhs131 = NULL;
  const mxArray *c2_rhs132 = NULL;
  const mxArray *c2_lhs132 = NULL;
  const mxArray *c2_rhs133 = NULL;
  const mxArray *c2_lhs133 = NULL;
  const mxArray *c2_rhs134 = NULL;
  const mxArray *c2_lhs134 = NULL;
  const mxArray *c2_rhs135 = NULL;
  const mxArray *c2_lhs135 = NULL;
  const mxArray *c2_rhs136 = NULL;
  const mxArray *c2_lhs136 = NULL;
  const mxArray *c2_rhs137 = NULL;
  const mxArray *c2_lhs137 = NULL;
  const mxArray *c2_rhs138 = NULL;
  const mxArray *c2_lhs138 = NULL;
  const mxArray *c2_rhs139 = NULL;
  const mxArray *c2_lhs139 = NULL;
  const mxArray *c2_rhs140 = NULL;
  const mxArray *c2_lhs140 = NULL;
  const mxArray *c2_rhs141 = NULL;
  const mxArray *c2_lhs141 = NULL;
  const mxArray *c2_rhs142 = NULL;
  const mxArray *c2_lhs142 = NULL;
  const mxArray *c2_rhs143 = NULL;
  const mxArray *c2_lhs143 = NULL;
  const mxArray *c2_rhs144 = NULL;
  const mxArray *c2_lhs144 = NULL;
  const mxArray *c2_rhs145 = NULL;
  const mxArray *c2_lhs145 = NULL;
  const mxArray *c2_rhs146 = NULL;
  const mxArray *c2_lhs146 = NULL;
  const mxArray *c2_rhs147 = NULL;
  const mxArray *c2_lhs147 = NULL;
  const mxArray *c2_rhs148 = NULL;
  const mxArray *c2_lhs148 = NULL;
  const mxArray *c2_rhs149 = NULL;
  const mxArray *c2_lhs149 = NULL;
  const mxArray *c2_rhs150 = NULL;
  const mxArray *c2_lhs150 = NULL;
  const mxArray *c2_rhs151 = NULL;
  const mxArray *c2_lhs151 = NULL;
  const mxArray *c2_rhs152 = NULL;
  const mxArray *c2_lhs152 = NULL;
  const mxArray *c2_rhs153 = NULL;
  const mxArray *c2_lhs153 = NULL;
  const mxArray *c2_rhs154 = NULL;
  const mxArray *c2_lhs154 = NULL;
  const mxArray *c2_rhs155 = NULL;
  const mxArray *c2_lhs155 = NULL;
  const mxArray *c2_rhs156 = NULL;
  const mxArray *c2_lhs156 = NULL;
  const mxArray *c2_rhs157 = NULL;
  const mxArray *c2_lhs157 = NULL;
  const mxArray *c2_rhs158 = NULL;
  const mxArray *c2_lhs158 = NULL;
  const mxArray *c2_rhs159 = NULL;
  const mxArray *c2_lhs159 = NULL;
  const mxArray *c2_rhs160 = NULL;
  const mxArray *c2_lhs160 = NULL;
  const mxArray *c2_rhs161 = NULL;
  const mxArray *c2_lhs161 = NULL;
  const mxArray *c2_rhs162 = NULL;
  const mxArray *c2_lhs162 = NULL;
  const mxArray *c2_rhs163 = NULL;
  const mxArray *c2_lhs163 = NULL;
  const mxArray *c2_rhs164 = NULL;
  const mxArray *c2_lhs164 = NULL;
  const mxArray *c2_rhs165 = NULL;
  const mxArray *c2_lhs165 = NULL;
  const mxArray *c2_rhs166 = NULL;
  const mxArray *c2_lhs166 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/trapz.m!trapwork"),
                  "context", "context", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1386456352U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c2_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395357306U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c2_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c2_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c2_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c2_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c2_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c2_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c2_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c2_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c2_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name",
                  138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840048U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370042286U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c2_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("kron"), "name", "name", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "resolved",
                  "resolved", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742678U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c2_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c2_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c2_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c2_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372615560U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c2_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c2_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1397289822U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c2_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/kron.m"), "context",
                  "context", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372614816U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c2_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("vec"), "name", "name", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[E]C:/Users/dlavell/Documents/MATLAB/hybridQuadSim/controllers/models/vec.m"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1454107529U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c2_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[E]C:/Users/dlavell/Documents/MATLAB/hybridQuadSim/controllers/models/vec.m"),
                  "context", "context", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("reshape"), "name", "name", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "resolved",
                  "resolved", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378328382U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c2_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395357306U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c2_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mod"), "name", "name", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m"), "resolved",
                  "resolved", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c2_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m"), "context",
                  "context", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1395960656U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c2_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m"), "context",
                  "context", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c2_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m"), "context",
                  "context", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c2_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod"), "context",
                  "context", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c2_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod"), "context",
                  "context", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c2_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod"), "context",
                  "context", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_round"), "name",
                  "name", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307683638U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c2_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod"), "context",
                  "context", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851112U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c2_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod"), "context",
                  "context", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eps"), "name", "name", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c2_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286851182U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c2_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_eps"), "name", "name", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c2_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c2_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c2_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840172U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c2_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c2_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1410840170U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c2_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!reshape_varargin_to_size"),
                  "context", "context", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368215430U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c2_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs166), "lhs", "lhs",
                  166);
  sf_mex_destroy(&c2_rhs128);
  sf_mex_destroy(&c2_lhs128);
  sf_mex_destroy(&c2_rhs129);
  sf_mex_destroy(&c2_lhs129);
  sf_mex_destroy(&c2_rhs130);
  sf_mex_destroy(&c2_lhs130);
  sf_mex_destroy(&c2_rhs131);
  sf_mex_destroy(&c2_lhs131);
  sf_mex_destroy(&c2_rhs132);
  sf_mex_destroy(&c2_lhs132);
  sf_mex_destroy(&c2_rhs133);
  sf_mex_destroy(&c2_lhs133);
  sf_mex_destroy(&c2_rhs134);
  sf_mex_destroy(&c2_lhs134);
  sf_mex_destroy(&c2_rhs135);
  sf_mex_destroy(&c2_lhs135);
  sf_mex_destroy(&c2_rhs136);
  sf_mex_destroy(&c2_lhs136);
  sf_mex_destroy(&c2_rhs137);
  sf_mex_destroy(&c2_lhs137);
  sf_mex_destroy(&c2_rhs138);
  sf_mex_destroy(&c2_lhs138);
  sf_mex_destroy(&c2_rhs139);
  sf_mex_destroy(&c2_lhs139);
  sf_mex_destroy(&c2_rhs140);
  sf_mex_destroy(&c2_lhs140);
  sf_mex_destroy(&c2_rhs141);
  sf_mex_destroy(&c2_lhs141);
  sf_mex_destroy(&c2_rhs142);
  sf_mex_destroy(&c2_lhs142);
  sf_mex_destroy(&c2_rhs143);
  sf_mex_destroy(&c2_lhs143);
  sf_mex_destroy(&c2_rhs144);
  sf_mex_destroy(&c2_lhs144);
  sf_mex_destroy(&c2_rhs145);
  sf_mex_destroy(&c2_lhs145);
  sf_mex_destroy(&c2_rhs146);
  sf_mex_destroy(&c2_lhs146);
  sf_mex_destroy(&c2_rhs147);
  sf_mex_destroy(&c2_lhs147);
  sf_mex_destroy(&c2_rhs148);
  sf_mex_destroy(&c2_lhs148);
  sf_mex_destroy(&c2_rhs149);
  sf_mex_destroy(&c2_lhs149);
  sf_mex_destroy(&c2_rhs150);
  sf_mex_destroy(&c2_lhs150);
  sf_mex_destroy(&c2_rhs151);
  sf_mex_destroy(&c2_lhs151);
  sf_mex_destroy(&c2_rhs152);
  sf_mex_destroy(&c2_lhs152);
  sf_mex_destroy(&c2_rhs153);
  sf_mex_destroy(&c2_lhs153);
  sf_mex_destroy(&c2_rhs154);
  sf_mex_destroy(&c2_lhs154);
  sf_mex_destroy(&c2_rhs155);
  sf_mex_destroy(&c2_lhs155);
  sf_mex_destroy(&c2_rhs156);
  sf_mex_destroy(&c2_lhs156);
  sf_mex_destroy(&c2_rhs157);
  sf_mex_destroy(&c2_lhs157);
  sf_mex_destroy(&c2_rhs158);
  sf_mex_destroy(&c2_lhs158);
  sf_mex_destroy(&c2_rhs159);
  sf_mex_destroy(&c2_lhs159);
  sf_mex_destroy(&c2_rhs160);
  sf_mex_destroy(&c2_lhs160);
  sf_mex_destroy(&c2_rhs161);
  sf_mex_destroy(&c2_lhs161);
  sf_mex_destroy(&c2_rhs162);
  sf_mex_destroy(&c2_lhs162);
  sf_mex_destroy(&c2_rhs163);
  sf_mex_destroy(&c2_lhs163);
  sf_mex_destroy(&c2_rhs164);
  sf_mex_destroy(&c2_lhs164);
  sf_mex_destroy(&c2_rhs165);
  sf_mex_destroy(&c2_lhs165);
  sf_mex_destroy(&c2_rhs166);
  sf_mex_destroy(&c2_lhs166);
}

static void c2_eml_switch_helper(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_threshold(SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_eml_extremum(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_x[2], real_T *c2_extremum, int32_T *c2_indx)
{
  int32_T c2_ixstart;
  real_T c2_mtmp;
  int32_T c2_itmp;
  real_T c2_b_x;
  boolean_T c2_b;
  int32_T c2_ix;
  int32_T c2_b_ix;
  real_T c2_c_x;
  boolean_T c2_b_b;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_i1355;
  int32_T c2_c_a;
  int32_T c2_d_a;
  boolean_T c2_overflow;
  int32_T c2_c_ix;
  real_T c2_e_a;
  real_T c2_c_b;
  boolean_T c2_p;
  real_T c2_b_mtmp;
  int32_T c2_b_itmp;
  boolean_T exitg1;
  c2_ixstart = 1;
  c2_mtmp = c2_x[0];
  c2_itmp = 1;
  c2_b_x = c2_mtmp;
  c2_b = muDoubleScalarIsNaN(c2_b_x);
  if (c2_b) {
    c2_eml_switch_helper(chartInstance);
    c2_eml_switch_helper(chartInstance);
    c2_ix = 2;
    exitg1 = false;
    while ((exitg1 == false) && (c2_ix < 3)) {
      c2_b_ix = c2_ix;
      c2_ixstart = c2_b_ix;
      c2_c_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_ix), 1, 2, 1, 0) - 1];
      c2_b_b = muDoubleScalarIsNaN(c2_c_x);
      if (!c2_b_b) {
        c2_mtmp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 2, 1, 0) - 1];
        c2_itmp = c2_b_ix;
        exitg1 = true;
      } else {
        c2_ix++;
      }
    }
  }

  if (c2_ixstart < 2) {
    c2_a = c2_ixstart;
    c2_b_a = c2_a + 1;
    c2_i1355 = c2_b_a;
    c2_c_a = c2_i1355;
    c2_d_a = c2_c_a;
    if (c2_d_a > 2) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_eml_switch_helper(chartInstance);
      c2_overflow = false;
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_c_ix = c2_i1355; c2_c_ix < 3; c2_c_ix++) {
      c2_b_ix = c2_c_ix;
      c2_e_a = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_ix), 1, 2, 1, 0) - 1];
      c2_c_b = c2_mtmp;
      c2_p = (c2_e_a > c2_c_b);
      if (c2_p) {
        c2_mtmp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 2, 1, 0) - 1];
        c2_itmp = c2_b_ix;
      }
    }
  }

  c2_b_mtmp = c2_mtmp;
  c2_b_itmp = c2_itmp;
  *c2_extremum = c2_b_mtmp;
  *c2_indx = c2_b_itmp;
}

static void c2_check_forloop_overflow_error
  (SFc2_PackageDeliverySimInstanceStruct *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i1356;
  static char_T c2_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i1357;
  static char_T c2_cv1[5] = { 'i', 'n', 't', '3', '2' };

  char_T c2_b_u[5];
  const mxArray *c2_b_y = NULL;
  (void)chartInstance;
  if (!c2_overflow) {
  } else {
    for (c2_i1356 = 0; c2_i1356 < 34; c2_i1356++) {
      c2_u[c2_i1356] = c2_cv0[c2_i1356];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c2_i1357 = 0; c2_i1357 < 5; c2_i1357++) {
      c2_b_u[c2_i1357] = c2_cv1[c2_i1357];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c2_y, 14, c2_b_y));
  }
}

static void c2_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var[3], real_T c2_out[3])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i1358;
  int32_T c2_b_I;
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_A;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_e_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_I, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_var, 3U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 4U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 246U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  for (c2_i1358 = 0; c2_i1358 < 3; c2_i1358++) {
    c2_out[c2_i1358] = c2_var[c2_i1358];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  CV_EML_IF(0, 1, 0, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 253U);
  c2_I = 1.0;
  c2_b_I = 0;
  while (c2_b_I < 3) {
    c2_I = 1.0 + (real_T)c2_b_I;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 254U);
    c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_b_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_A = c2_var[_SFD_EML_ARRAY_BOUNDS_CHECK("var", (int32_T)_SFD_INTEGER_CHECK
      ("I", c2_I), 1, 3, 1, 0) - 1];
    c2_B = c2_b_hoistedGlobal;
    c2_x = c2_A;
    c2_y = c2_B;
    c2_b_x = c2_x;
    c2_b_y = c2_y;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_d_y = c2_c_x / c2_c_y;
    c2_d_x = c2_d_y;
    c2_e_x = c2_d_x;
    c2_e_x = muDoubleScalarAtan(c2_e_x);
    c2_b_A = c2_hoistedGlobal * 2.0 * c2_e_x;
    c2_f_x = c2_b_A;
    c2_g_x = c2_f_x;
    c2_h_x = c2_g_x;
    c2_e_y = c2_h_x / 3.1415926535897931;
    c2_out[_SFD_EML_ARRAY_BOUNDS_CHECK("out", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 3, 1, 0) - 1] = c2_e_y;
    c2_b_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -254);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c2_norm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_x[3])
{
  real_T c2_y;
  real_T c2_scale;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_below_threshold(chartInstance);
  c2_y = 0.0;
  c2_scale = 2.2250738585072014E-308;
  c2_eml_switch_helper(chartInstance);
  for (c2_k = 1; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 3, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_absxk = muDoubleScalarAbs(c2_c_x);
    if (c2_absxk > c2_scale) {
      c2_t = c2_scale / c2_absxk;
      c2_y = 1.0 + c2_y * c2_t * c2_t;
      c2_scale = c2_absxk;
    } else {
      c2_t = c2_absxk / c2_scale;
      c2_y += c2_t * c2_t;
    }
  }

  return c2_scale * muDoubleScalarSqrt(c2_y);
}

static void c2_below_threshold(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_S(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                 c2_v[3], real_T c2_out[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_v, 2U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 3U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 292);
  c2_out[0] = 0.0;
  c2_out[3] = -c2_v[2];
  c2_out[6] = c2_v[1];
  c2_out[1] = c2_v[2];
  c2_out[4] = 0.0;
  c2_out[7] = -c2_v[0];
  c2_out[2] = -c2_v[1];
  c2_out[5] = c2_v[0];
  c2_out[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -292);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_eye(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                   c2_I[9])
{
  int32_T c2_i1359;
  int32_T c2_k;
  int32_T c2_b_k;
  for (c2_i1359 = 0; c2_i1359 < 9; c2_i1359++) {
    c2_I[c2_i1359] = 0.0;
  }

  c2_eml_switch_helper(chartInstance);
  for (c2_k = 1; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 3, 2, 0) - 1)) -
      1] = 1.0;
  }
}

static void c2_c_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_threshold(SFc2_PackageDeliverySimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_a[9], real_T c2_c[9])
{
  int32_T c2_i1360;
  real_T c2_b_a[9];
  for (c2_i1360 = 0; c2_i1360 < 9; c2_i1360++) {
    c2_b_a[c2_i1360] = c2_a[c2_i1360];
  }

  c2_matrix_to_integer_power(chartInstance, c2_b_a, c2_c);
}

static void c2_matrix_to_integer_power(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_a[9], real_T c2_c[9])
{
  int32_T c2_i1361;
  int32_T c2_i1362;
  int32_T c2_i1363;
  int32_T c2_i1364;
  int32_T c2_i1365;
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1361 = 0; c2_i1361 < 3; c2_i1361++) {
    c2_i1362 = 0;
    for (c2_i1363 = 0; c2_i1363 < 3; c2_i1363++) {
      c2_c[c2_i1362 + c2_i1361] = 0.0;
      c2_i1364 = 0;
      for (c2_i1365 = 0; c2_i1365 < 3; c2_i1365++) {
        c2_c[c2_i1362 + c2_i1361] += c2_a[c2_i1364 + c2_i1361] * c2_a[c2_i1365 +
          c2_i1362];
        c2_i1364 += 3;
      }

      c2_i1362 += 3;
    }
  }
}

static void c2_d_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_linspace(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_d1, real_T c2_d2, real_T c2_y[1000])
{
  real_T c2_A;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_delta1;
  real_T c2_b_A;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_delta2;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_c_A;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  int32_T c2_c_k;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  c2_e_eml_scalar_eg(chartInstance);
  c2_y[999] = c2_d2;
  c2_y[0] = c2_d1;
  guard1 = false;
  guard2 = false;
  if (c2_d1 < 0.0 != c2_d2 < 0.0) {
    if (muDoubleScalarAbs(c2_d1) > 8.9884656743115785E+307) {
      guard2 = true;
    } else if (muDoubleScalarAbs(c2_d2) > 8.9884656743115785E+307) {
      guard2 = true;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard2 == true) {
    c2_A = c2_d1;
    c2_x = c2_A;
    c2_b_x = c2_x;
    c2_c_x = c2_b_x;
    c2_delta1 = c2_c_x / 999.0;
    c2_b_A = c2_d2;
    c2_d_x = c2_b_A;
    c2_e_x = c2_d_x;
    c2_f_x = c2_e_x;
    c2_delta2 = c2_f_x / 999.0;
    for (c2_k = 0; c2_k < 998; c2_k++) {
      c2_b_k = 1.0 + (real_T)c2_k;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c2_b_k + 1.0), 1, 1000, 1, 0) - 1] = (c2_d1 + c2_delta2 * c2_b_k) -
        c2_delta1 * c2_b_k;
    }
  }

  if (guard1 == true) {
    c2_c_A = c2_d2 - c2_d1;
    c2_g_x = c2_c_A;
    c2_h_x = c2_g_x;
    c2_i_x = c2_h_x;
    c2_delta1 = c2_i_x / 999.0;
    for (c2_c_k = 0; c2_c_k < 998; c2_c_k++) {
      c2_b_k = 1.0 + (real_T)c2_c_k;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c2_b_k + 1.0), 1, 1000, 1, 0) - 1] = c2_d1 + c2_b_k * c2_delta1;
    }
  }
}

static void c2_e_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_b_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var)
{
  real_T c2_out;
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_A;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_d_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I, 0U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_var, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_out, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 246U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  c2_out = c2_var;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  CV_EML_IF(0, 1, 0, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 253U);
  c2_I = 1.0;
  CV_EML_FOR(0, 1, 3, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 254U);
  c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
  c2_b_hoistedGlobal = c2_get_satMax(chartInstance, 0);
  c2_A = c2_var;
  c2_B = c2_b_hoistedGlobal;
  c2_x = c2_A;
  c2_y = c2_B;
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_c_x = c2_b_x;
  c2_c_y = c2_b_y;
  c2_d_y = c2_c_x / c2_c_y;
  c2_d_x = c2_d_y;
  c2_e_x = c2_d_x;
  c2_e_x = muDoubleScalarAtan(c2_e_x);
  c2_b_A = c2_hoistedGlobal * 2.0 * c2_e_x;
  c2_f_x = c2_b_A;
  c2_g_x = c2_f_x;
  c2_h_x = c2_g_x;
  c2_out = c2_h_x / 3.1415926535897931;
  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -254);
  _SFD_SYMBOL_SCOPE_POP();
  return c2_out;
}

static void c2_f_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_c_sat_atan(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_var[1000], real_T c2_out[1000])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i1366;
  int32_T c2_b_I;
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_A;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_e_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_I, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_var, 3U, c2_k_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 4U, c2_k_sf_marshallOut,
    c2_i_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 246U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  for (c2_i1366 = 0; c2_i1366 < 1000; c2_i1366++) {
    c2_out[c2_i1366] = c2_var[c2_i1366];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  CV_EML_IF(0, 1, 0, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 253U);
  c2_I = 1.0;
  c2_b_I = 0;
  while (c2_b_I < 1000) {
    c2_I = 1.0 + (real_T)c2_b_I;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 254U);
    c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_b_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_A = c2_var[_SFD_EML_ARRAY_BOUNDS_CHECK("var", (int32_T)_SFD_INTEGER_CHECK
      ("I", c2_I), 1, 1000, 1, 0) - 1];
    c2_B = c2_b_hoistedGlobal;
    c2_x = c2_A;
    c2_y = c2_B;
    c2_b_x = c2_x;
    c2_b_y = c2_y;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_d_y = c2_c_x / c2_c_y;
    c2_d_x = c2_d_y;
    c2_e_x = c2_d_x;
    c2_e_x = muDoubleScalarAtan(c2_e_x);
    c2_b_A = c2_hoistedGlobal * 2.0 * c2_e_x;
    c2_f_x = c2_b_A;
    c2_g_x = c2_f_x;
    c2_h_x = c2_g_x;
    c2_e_y = c2_h_x / 3.1415926535897931;
    c2_out[_SFD_EML_ARRAY_BOUNDS_CHECK("out", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 1000, 1, 0) - 1] = c2_e_y;
    c2_b_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -254);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c2_trapz(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_x[1000], real_T c2_y[1000])
{
  int32_T c2_i1367;
  real_T c2_b_x[1000];
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_s;
  int32_T c2_ix;
  int32_T c2_iy;
  real_T c2_ylast;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_c_a;
  int32_T c2_d_a;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_c_z;
  for (c2_i1367 = 0; c2_i1367 < 1000; c2_i1367++) {
    c2_b_x[c2_i1367] = c2_x[c2_i1367];
  }

  c2_eml_switch_helper(chartInstance);
  for (c2_i = 1; c2_i < 1000; c2_i++) {
    c2_b_i = c2_i;
    c2_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_i), 1, 1000, 1, 0) - 1] = c2_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)(c2_b_i + 1)), 1, 1000, 1, 0)
      - 1] - c2_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_b_i), 1, 1000, 1, 0) - 1];
  }

  c2_s = 0.0;
  c2_ix = 0;
  c2_iy = 1;
  c2_ylast = c2_y[0];
  for (c2_k = 0; c2_k < 999; c2_k++) {
    c2_a = c2_iy;
    c2_b_a = c2_a + 1;
    c2_iy = c2_b_a;
    c2_c_a = c2_ix;
    c2_d_a = c2_c_a + 1;
    c2_ix = c2_d_a;
    c2_c_x = c2_ylast + c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 1000, 1, 0) - 1];
    c2_d_x = c2_c_x;
    c2_c_z = c2_d_x / 2.0;
    c2_s += c2_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 1000, 1, 0) - 1] * c2_c_z;
    c2_ylast = c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_iy), 1, 1000, 1, 0) - 1];
  }

  return c2_s;
}

static real_T c2_sat_atan_dot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var)
{
  real_T c2_out;
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_f_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I, 0U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_var, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_out, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 260);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 261);
  c2_out = c2_var;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 262);
  CV_EML_IF(0, 1, 1, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 267);
  c2_I = 1.0;
  CV_EML_FOR(0, 1, 5, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 268);
  c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
  c2_A = c2_var;
  c2_B = c2_hoistedGlobal;
  c2_x = c2_A;
  c2_y = c2_B;
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_c_x = c2_b_x;
  c2_c_y = c2_b_y;
  c2_d_y = c2_c_x / c2_c_y;
  c2_out = 0.63661977236758138 * c2_c_mpower(chartInstance, 1.0 + c2_b_mpower
    (chartInstance, c2_d_y));
  CV_EML_FOR(0, 1, 5, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -268);
  _SFD_SYMBOL_SCOPE_POP();
  return c2_out;
}

static real_T c2_b_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_e_eml_scalar_eg(chartInstance);
  return c2_d_a * c2_d_a;
}

static real_T c2_c_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_B;
  real_T c2_y;
  real_T c2_b_y;
  real_T c2_c_y;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_e_eml_scalar_eg(chartInstance);
  c2_B = c2_d_a;
  c2_y = c2_B;
  c2_b_y = c2_y;
  c2_c_y = c2_b_y;
  return 1.0 / c2_c_y;
}

static void c2_g_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_h_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_i_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance, real_T
                    c2_A[3], real_T c2_B[9], real_T c2_K[27])
{
  int32_T c2_kidx;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j2 = 1; c2_j2 < 4; c2_j2++) {
    c2_b_j2 = c2_j2;
    c2_eml_switch_helper(chartInstance);
    for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
      c2_b_i1 = c2_i1;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 4; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 27, 1, 0) - 1] = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i1), 1, 3, 1, 0) - 1]
          * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c2_b_i2), 1, 3, 1, 0) + 3 *
                  (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_b_j2), 1, 3, 2, 0) - 1)) - 1];
      }
    }
  }
}

static void c2_b_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[3], real_T c2_K[27])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_eml_scalar_eg(chartInstance);
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
      c2_b_i1 = c2_i1;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 4; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 27, 1, 0) - 1] = c2_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_i1), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 3, 2, 0)
             - 1)) - 1] * c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 3, 1, 0) - 1];
      }
    }
  }
}

static void c2_c_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[9], real_T c2_K[81])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_d_eml_scalar_eg(chartInstance);
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 4; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
        c2_b_i1 = c2_i1;
        c2_eml_switch_helper(chartInstance);
        for (c2_i2 = 1; c2_i2 < 4; c2_i2++) {
          c2_b_i2 = c2_i2;
          c2_a = c2_kidx;
          c2_b_a = c2_a + 1;
          c2_kidx = c2_b_a;
          c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kidx), 1, 81, 1, 0) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_i1), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 3, 2, 0)
               - 1)) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 3, 1, 0) + 3 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j2), 1, 3, 2, 0) - 1)) - 1];
        }
      }
    }
  }
}

static void c2_Gamma(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     real_T c2_A[9], real_T c2_out[27])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i1368;
  real_T c2_a[9];
  int32_T c2_i1369;
  real_T c2_y[3];
  int32_T c2_i1370;
  int32_T c2_i1371;
  static real_T c2_b[3] = { 1.0, 0.0, 0.0 };

  int32_T c2_i1372;
  int32_T c2_i1373;
  real_T c2_b_y[3];
  int32_T c2_i1374;
  int32_T c2_i1375;
  static real_T c2_b_b[3] = { 0.0, 1.0, 0.0 };

  int32_T c2_i1376;
  real_T c2_v[3];
  real_T c2_b_nargin = 1.0;
  real_T c2_b_nargout = 1.0;
  real_T c2_b_out[9];
  int32_T c2_i1377;
  int32_T c2_i1378;
  int32_T c2_i1379;
  int32_T c2_i1380;
  static real_T c2_c_b[3] = { 0.0, 0.0, 1.0 };

  int32_T c2_i1381;
  real_T c2_b_v[3];
  real_T c2_c_nargin = 1.0;
  real_T c2_c_nargout = 1.0;
  real_T c2_c_out[9];
  int32_T c2_i1382;
  real_T c2_c_y[3];
  int32_T c2_i1383;
  int32_T c2_i1384;
  int32_T c2_i1385;
  int32_T c2_i1386;
  real_T c2_b_a[27];
  int32_T c2_i1387;
  int32_T c2_i1388;
  int32_T c2_i1389;
  int32_T c2_i1390;
  int32_T c2_i1391;
  int32_T c2_i1392;
  int32_T c2_i1393;
  int32_T c2_i1394;
  int32_T c2_i1395;
  int32_T c2_i1396;
  int32_T c2_i1397;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_g_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_A, 2U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 3U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 288);
  for (c2_i1368 = 0; c2_i1368 < 9; c2_i1368++) {
    c2_a[c2_i1368] = c2_A[c2_i1368];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1369 = 0; c2_i1369 < 3; c2_i1369++) {
    c2_y[c2_i1369] = 0.0;
    c2_i1370 = 0;
    for (c2_i1371 = 0; c2_i1371 < 3; c2_i1371++) {
      c2_y[c2_i1369] += c2_a[c2_i1370 + c2_i1369] * c2_b[c2_i1371];
      c2_i1370 += 3;
    }
  }

  for (c2_i1372 = 0; c2_i1372 < 9; c2_i1372++) {
    c2_a[c2_i1372] = c2_A[c2_i1372];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1373 = 0; c2_i1373 < 3; c2_i1373++) {
    c2_b_y[c2_i1373] = 0.0;
    c2_i1374 = 0;
    for (c2_i1375 = 0; c2_i1375 < 3; c2_i1375++) {
      c2_b_y[c2_i1373] += c2_a[c2_i1374 + c2_i1373] * c2_b_b[c2_i1375];
      c2_i1374 += 3;
    }
  }

  for (c2_i1376 = 0; c2_i1376 < 3; c2_i1376++) {
    c2_v[c2_i1376] = c2_b_y[c2_i1376];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_v, 2U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_out, 3U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 292);
  c2_b_out[0] = 0.0;
  c2_b_out[3] = -c2_v[2];
  c2_b_out[6] = c2_v[1];
  c2_b_out[1] = c2_v[2];
  c2_b_out[4] = 0.0;
  c2_b_out[7] = -c2_v[0];
  c2_b_out[2] = -c2_v[1];
  c2_b_out[5] = c2_v[0];
  c2_b_out[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -292);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i1377 = 0; c2_i1377 < 9; c2_i1377++) {
    c2_a[c2_i1377] = c2_A[c2_i1377];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i1378 = 0; c2_i1378 < 3; c2_i1378++) {
    c2_b_y[c2_i1378] = 0.0;
    c2_i1379 = 0;
    for (c2_i1380 = 0; c2_i1380 < 3; c2_i1380++) {
      c2_b_y[c2_i1378] += c2_a[c2_i1379 + c2_i1378] * c2_c_b[c2_i1380];
      c2_i1379 += 3;
    }
  }

  for (c2_i1381 = 0; c2_i1381 < 3; c2_i1381++) {
    c2_b_v[c2_i1381] = c2_b_y[c2_i1381];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_v, 2U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_c_out, 3U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 292);
  c2_c_out[0] = 0.0;
  c2_c_out[3] = -c2_b_v[2];
  c2_c_out[6] = c2_b_v[1];
  c2_c_out[1] = c2_b_v[2];
  c2_c_out[4] = 0.0;
  c2_c_out[7] = -c2_b_v[0];
  c2_c_out[2] = -c2_b_v[1];
  c2_c_out[5] = c2_b_v[0];
  c2_c_out[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -292);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i1382 = 0; c2_i1382 < 3; c2_i1382++) {
    c2_c_y[c2_i1382] = c2_y[c2_i1382];
  }

  c2_S(chartInstance, c2_c_y, c2_a);
  c2_i1383 = 0;
  for (c2_i1384 = 0; c2_i1384 < 3; c2_i1384++) {
    c2_i1385 = 0;
    for (c2_i1386 = 0; c2_i1386 < 3; c2_i1386++) {
      c2_b_a[c2_i1386 + c2_i1383] = c2_a[c2_i1385 + c2_i1384];
      c2_i1385 += 3;
    }

    c2_i1383 += 9;
  }

  c2_i1387 = 0;
  for (c2_i1388 = 0; c2_i1388 < 3; c2_i1388++) {
    c2_i1389 = 0;
    for (c2_i1390 = 0; c2_i1390 < 3; c2_i1390++) {
      c2_b_a[(c2_i1390 + c2_i1387) + 3] = c2_b_out[c2_i1389 + c2_i1388];
      c2_i1389 += 3;
    }

    c2_i1387 += 9;
  }

  c2_i1391 = 0;
  for (c2_i1392 = 0; c2_i1392 < 3; c2_i1392++) {
    c2_i1393 = 0;
    for (c2_i1394 = 0; c2_i1394 < 3; c2_i1394++) {
      c2_b_a[(c2_i1394 + c2_i1391) + 6] = c2_c_out[c2_i1393 + c2_i1392];
      c2_i1393 += 3;
    }

    c2_i1391 += 9;
  }

  c2_i1395 = 0;
  for (c2_i1396 = 0; c2_i1396 < 3; c2_i1396++) {
    for (c2_i1397 = 0; c2_i1397 < 9; c2_i1397++) {
      c2_out[c2_i1397 + c2_i1395] = -c2_b_a[c2_i1397 + c2_i1395];
    }

    c2_i1395 += 9;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -288);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_j_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_k_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_l_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_m_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_sat_atan_dot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var[3], real_T c2_out[3])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i1398;
  int32_T c2_b_I;
  real_T c2_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_h_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_I, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_var, 3U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 4U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 260);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 261);
  for (c2_i1398 = 0; c2_i1398 < 3; c2_i1398++) {
    c2_out[c2_i1398] = c2_var[c2_i1398];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 262);
  CV_EML_IF(0, 1, 1, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 267);
  c2_I = 1.0;
  c2_b_I = 0;
  while (c2_b_I < 3) {
    c2_I = 1.0 + (real_T)c2_b_I;
    CV_EML_FOR(0, 1, 5, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 268);
    c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_A = c2_var[_SFD_EML_ARRAY_BOUNDS_CHECK("var", (int32_T)_SFD_INTEGER_CHECK
      ("I", c2_I), 1, 3, 1, 0) - 1];
    c2_B = c2_hoistedGlobal;
    c2_x = c2_A;
    c2_y = c2_B;
    c2_b_x = c2_x;
    c2_b_y = c2_y;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_d_y = c2_c_x / c2_c_y;
    c2_out[_SFD_EML_ARRAY_BOUNDS_CHECK("out", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 3, 1, 0) - 1] = 0.63661977236758138 * c2_c_mpower(chartInstance,
      1.0 + c2_b_mpower(chartInstance, c2_d_y));
    c2_b_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 5, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -268);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_n_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_o_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[2], real_T c2_B[36], real_T c2_K[72])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 3; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 7; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 7; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 72, 1, 0) - 1] = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 2, 2, 0) - 1]
          * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c2_b_i2), 1, 6, 1, 0) + 6 *
                  (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_b_j2), 1, 6, 2, 0) - 1)) - 1];
      }
    }
  }
}

static real_T c2_sat_atan_ddot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var)
{
  real_T c2_out;
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_e_y;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_h_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_i_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I, 0U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_var, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_out, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 274);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 275);
  c2_out = c2_var;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 276);
  CV_EML_IF(0, 1, 2, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 281);
  c2_I = 1.0;
  CV_EML_FOR(0, 1, 7, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 282);
  c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
  c2_A = c2_var;
  c2_B = c2_hoistedGlobal;
  c2_x = c2_A;
  c2_y = c2_B;
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_c_x = c2_b_x;
  c2_c_y = c2_b_y;
  c2_d_y = c2_c_x / c2_c_y;
  c2_b_hoistedGlobal = c2_get_satMax(chartInstance, 0);
  c2_b_A = c2_var;
  c2_b_B = c2_b_mpower(chartInstance, c2_b_hoistedGlobal);
  c2_d_x = c2_b_A;
  c2_e_y = c2_b_B;
  c2_e_x = c2_d_x;
  c2_f_y = c2_e_y;
  c2_f_x = c2_e_x;
  c2_g_y = c2_f_y;
  c2_h_y = c2_f_x / c2_g_y;
  c2_out = -0.63661977236758138 * c2_d_mpower(chartInstance, 1.0 + c2_b_mpower
    (chartInstance, c2_d_y)) * 2.0 * c2_h_y;
  CV_EML_FOR(0, 1, 7, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -282);
  _SFD_SYMBOL_SCOPE_POP();
  return c2_out;
}

static real_T c2_d_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_ar;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ar = c2_d_a;
  return muDoubleScalarPower(c2_ar, -2.0);
}

static void c2_p_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_q_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_r_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_s_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_t_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_u_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_sat_atan_ddot(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, real_T c2_var[3], real_T c2_out[3])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_I;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i1399;
  int32_T c2_b_I;
  real_T c2_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_e_y;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_h_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_j_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_I, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_var, 3U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_out, 4U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 274);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 275);
  for (c2_i1399 = 0; c2_i1399 < 3; c2_i1399++) {
    c2_out[c2_i1399] = c2_var[c2_i1399];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 276);
  CV_EML_IF(0, 1, 2, false);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 281);
  c2_I = 1.0;
  c2_b_I = 0;
  while (c2_b_I < 3) {
    c2_I = 1.0 + (real_T)c2_b_I;
    CV_EML_FOR(0, 1, 7, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 282);
    c2_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_A = c2_var[_SFD_EML_ARRAY_BOUNDS_CHECK("var", (int32_T)_SFD_INTEGER_CHECK
      ("I", c2_I), 1, 3, 1, 0) - 1];
    c2_B = c2_hoistedGlobal;
    c2_x = c2_A;
    c2_y = c2_B;
    c2_b_x = c2_x;
    c2_b_y = c2_y;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_d_y = c2_c_x / c2_c_y;
    c2_b_hoistedGlobal = c2_get_satMax(chartInstance, 0);
    c2_b_A = c2_var[_SFD_EML_ARRAY_BOUNDS_CHECK("var", (int32_T)
      _SFD_INTEGER_CHECK("I", c2_I), 1, 3, 1, 0) - 1];
    c2_b_B = c2_b_mpower(chartInstance, c2_b_hoistedGlobal);
    c2_d_x = c2_b_A;
    c2_e_y = c2_b_B;
    c2_e_x = c2_d_x;
    c2_f_y = c2_e_y;
    c2_f_x = c2_e_x;
    c2_g_y = c2_f_y;
    c2_h_y = c2_f_x / c2_g_y;
    c2_out[_SFD_EML_ARRAY_BOUNDS_CHECK("out", (int32_T)_SFD_INTEGER_CHECK("I",
      c2_I), 1, 3, 1, 0) - 1] = -0.63661977236758138 * c2_d_mpower(chartInstance,
      1.0 + c2_b_mpower(chartInstance, c2_d_y)) * 2.0 * c2_h_y;
    c2_b_I++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 7, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -282);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_power(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_y[3])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  real_T c2_b_a;
  real_T c2_b_y;
  for (c2_k = 0; c2_k < 3; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 3, 1, 0) - 1];
    c2_b_a = c2_ak;
    c2_e_eml_scalar_eg(chartInstance);
    c2_b_y = c2_b_a * c2_b_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 3, 1, 0) - 1] = c2_b_y;
  }
}

static void c2_e_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[9], real_T c2_K[27])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_c_eml_scalar_eg(chartInstance);
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 4; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 4; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 27, 1, 0) - 1] = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 3, 2, 0) - 1]
          * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c2_b_i2), 1, 3, 1, 0) + 3 *
                  (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_b_j2), 1, 3, 2, 0) - 1)) - 1];
      }
    }
  }
}

static void c2_v_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_e_mpower(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_ar;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_e_eml_scalar_eg(chartInstance);
  c2_ar = c2_d_a;
  return muDoubleScalarPower(c2_ar, 3.0);
}

static void c2_w_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[729], real_T c2_B[81], real_T c2_C[81], real_T c2_b_C[81])
{
  int32_T c2_i1400;
  int32_T c2_i1401;
  real_T c2_b_A[729];
  int32_T c2_i1402;
  real_T c2_b_B[81];
  for (c2_i1400 = 0; c2_i1400 < 81; c2_i1400++) {
    c2_b_C[c2_i1400] = c2_C[c2_i1400];
  }

  for (c2_i1401 = 0; c2_i1401 < 729; c2_i1401++) {
    c2_b_A[c2_i1401] = c2_A[c2_i1401];
  }

  for (c2_i1402 = 0; c2_i1402 < 81; c2_i1402++) {
    c2_b_B[c2_i1402] = c2_B[c2_i1402];
  }

  c2_g_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_f_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[81], real_T c2_K[729])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 10; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
        c2_b_i1 = c2_i1;
        c2_eml_switch_helper(chartInstance);
        for (c2_i2 = 1; c2_i2 < 10; c2_i2++) {
          c2_b_i2 = c2_i2;
          c2_a = c2_kidx;
          c2_b_a = c2_a + 1;
          c2_kidx = c2_b_a;
          c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kidx), 1, 729, 1, 0) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_i1), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 3, 2, 0)
               - 1)) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 9, 1, 0) + 9 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j2), 1, 9, 2, 0) - 1)) - 1];
        }
      }
    }
  }
}

static void c2_x_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_g_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[81], real_T c2_K[243])
{
  int32_T c2_kidx;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j2 = 1; c2_j2 < 10; c2_j2++) {
    c2_b_j2 = c2_j2;
    c2_eml_switch_helper(chartInstance);
    for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
      c2_b_i1 = c2_i1;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 10; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 243, 1, 0) - 1] =
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i1), 1, 3, 1, 0) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 9, 1, 0) + 9
          * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j2), 1, 9, 2, 0) - 1)) - 1];
      }
    }
  }
}

static void c2_y_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[243], real_T c2_b_C[243])
{
  int32_T c2_i1403;
  int32_T c2_i1404;
  real_T c2_b_A[243];
  int32_T c2_i1405;
  real_T c2_b_B[81];
  for (c2_i1403 = 0; c2_i1403 < 243; c2_i1403++) {
    c2_b_C[c2_i1403] = c2_C[c2_i1403];
  }

  for (c2_i1404 = 0; c2_i1404 < 243; c2_i1404++) {
    c2_b_A[c2_i1404] = c2_A[c2_i1404];
  }

  for (c2_i1405 = 0; c2_i1405 < 81; c2_i1405++) {
    c2_b_B[c2_i1405] = c2_B[c2_i1405];
  }

  c2_h_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_ab_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_c_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[27], real_T c2_C[81], real_T c2_b_C[81])
{
  int32_T c2_i1406;
  int32_T c2_i1407;
  real_T c2_b_A[243];
  int32_T c2_i1408;
  real_T c2_b_B[27];
  for (c2_i1406 = 0; c2_i1406 < 81; c2_i1406++) {
    c2_b_C[c2_i1406] = c2_C[c2_i1406];
  }

  for (c2_i1407 = 0; c2_i1407 < 243; c2_i1407++) {
    c2_b_A[c2_i1407] = c2_A[c2_i1407];
  }

  for (c2_i1408 = 0; c2_i1408 < 27; c2_i1408++) {
    c2_b_B[c2_i1408] = c2_B[c2_i1408];
  }

  c2_i_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_bb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_B[81], real_T c2_C[81], real_T c2_b_C[81])
{
  int32_T c2_i1409;
  int32_T c2_i1410;
  real_T c2_b_A[81];
  int32_T c2_i1411;
  real_T c2_b_B[81];
  for (c2_i1409 = 0; c2_i1409 < 81; c2_i1409++) {
    c2_b_C[c2_i1409] = c2_C[c2_i1409];
  }

  for (c2_i1410 = 0; c2_i1410 < 81; c2_i1410++) {
    c2_b_A[c2_i1410] = c2_A[c2_i1410];
  }

  for (c2_i1411 = 0; c2_i1411 < 81; c2_i1411++) {
    c2_b_B[c2_i1411] = c2_B[c2_i1411];
  }

  c2_j_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_cb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_h_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[9], real_T c2_B[27], real_T c2_K[243])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 4; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
        c2_b_i1 = c2_i1;
        c2_eml_switch_helper(chartInstance);
        for (c2_i2 = 1; c2_i2 < 10; c2_i2++) {
          c2_b_i2 = c2_i2;
          c2_a = c2_kidx;
          c2_b_a = c2_a + 1;
          c2_kidx = c2_b_a;
          c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kidx), 1, 243, 1, 0) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_i1), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 3, 2, 0)
               - 1)) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 9, 1, 0) + 9 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j2), 1, 3, 2, 0) - 1)) - 1];
        }
      }
    }
  }
}

static void c2_i_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[27], real_T c2_B[81], real_T c2_K[2187])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i1;
  int32_T c2_b_i1;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_v_eml_scalar_eg(chartInstance);
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 10; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 10; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i1 = 1; c2_i1 < 4; c2_i1++) {
        c2_b_i1 = c2_i1;
        c2_eml_switch_helper(chartInstance);
        for (c2_i2 = 1; c2_i2 < 10; c2_i2++) {
          c2_b_i2 = c2_i2;
          c2_a = c2_kidx;
          c2_b_a = c2_a + 1;
          c2_kidx = c2_b_a;
          c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kidx), 1, 2187, 1, 0) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_i1), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j1), 1, 9, 2, 0)
               - 1)) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 9, 1, 0) + 9 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j2), 1, 9, 2, 0) - 1)) - 1];
        }
      }
    }
  }
}

static void c2_db_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_e_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[2187], real_T c2_B[81], real_T c2_C[27], real_T c2_b_C[27])
{
  int32_T c2_i1412;
  int32_T c2_i1413;
  real_T c2_b_A[2187];
  int32_T c2_i1414;
  real_T c2_b_B[81];
  for (c2_i1412 = 0; c2_i1412 < 27; c2_i1412++) {
    c2_b_C[c2_i1412] = c2_C[c2_i1412];
  }

  for (c2_i1413 = 0; c2_i1413 < 2187; c2_i1413++) {
    c2_b_A[c2_i1413] = c2_A[c2_i1413];
  }

  for (c2_i1414 = 0; c2_i1414 < 81; c2_i1414++) {
    c2_b_B[c2_i1414] = c2_B[c2_i1414];
  }

  c2_k_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_j_kron(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      real_T c2_A[3], real_T c2_B[81], real_T c2_K[243])
{
  int32_T c2_kidx;
  int32_T c2_j1;
  int32_T c2_b_j1;
  int32_T c2_j2;
  int32_T c2_b_j2;
  int32_T c2_i2;
  int32_T c2_b_i2;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_kidx = 0;
  c2_eml_switch_helper(chartInstance);
  for (c2_j1 = 1; c2_j1 < 4; c2_j1++) {
    c2_b_j1 = c2_j1;
    c2_eml_switch_helper(chartInstance);
    for (c2_j2 = 1; c2_j2 < 10; c2_j2++) {
      c2_b_j2 = c2_j2;
      c2_eml_switch_helper(chartInstance);
      for (c2_i2 = 1; c2_i2 < 10; c2_i2++) {
        c2_b_i2 = c2_i2;
        c2_a = c2_kidx;
        c2_b_a = c2_a + 1;
        c2_kidx = c2_b_a;
        c2_K[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_kidx), 1, 243, 1, 0) - 1] =
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j1), 1, 3, 2, 0) - 1] * c2_B[(_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i2), 1, 9, 1, 0) + 9
          * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j2), 1, 9, 2, 0) - 1)) - 1];
      }
    }
  }
}

static void c2_eb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_f_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[27], real_T c2_b_C[27])
{
  int32_T c2_i1415;
  int32_T c2_i1416;
  real_T c2_b_A[243];
  int32_T c2_i1417;
  real_T c2_b_B[81];
  for (c2_i1415 = 0; c2_i1415 < 27; c2_i1415++) {
    c2_b_C[c2_i1415] = c2_C[c2_i1415];
  }

  for (c2_i1416 = 0; c2_i1416 < 243; c2_i1416++) {
    c2_b_A[c2_i1416] = c2_A[c2_i1416];
  }

  for (c2_i1417 = 0; c2_i1417 < 81; c2_i1417++) {
    c2_b_B[c2_i1417] = c2_B[c2_i1417];
  }

  c2_l_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_fb_eml_scalar_eg(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_m_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_transpose, const char_T *c2_identifier,
  real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_transpose), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_transpose);
}

static void c2_n_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv53[4];
  int32_T c2_i1418;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv53, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i1418 = 0; c2_i1418 < 4; c2_i1418++) {
    c2_y[c2_i1418] = c2_dv53[c2_i1418];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_o_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i1419;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i1419, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i1419;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_p_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_PackageDeliverySim, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_PackageDeliverySim), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_PackageDeliverySim);
  return c2_y;
}

static uint8_T c2_q_emlrt_marshallIn(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[729], real_T c2_B[81], real_T c2_C[81])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(27);
  c2_n_t = (ptrdiff_t)(3);
  c2_k_t = (ptrdiff_t)(27);
  c2_lda_t = (ptrdiff_t)(27);
  c2_ldb_t = (ptrdiff_t)(27);
  c2_ldc_t = (ptrdiff_t)(27);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_h_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[243])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(27);
  c2_n_t = (ptrdiff_t)(9);
  c2_k_t = (ptrdiff_t)(9);
  c2_lda_t = (ptrdiff_t)(27);
  c2_ldb_t = (ptrdiff_t)(9);
  c2_ldc_t = (ptrdiff_t)(27);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_i_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[27], real_T c2_C[81])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(27);
  c2_n_t = (ptrdiff_t)(3);
  c2_k_t = (ptrdiff_t)(9);
  c2_lda_t = (ptrdiff_t)(27);
  c2_ldb_t = (ptrdiff_t)(9);
  c2_ldc_t = (ptrdiff_t)(27);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_j_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_B[81], real_T c2_C[81])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(9);
  c2_n_t = (ptrdiff_t)(9);
  c2_k_t = (ptrdiff_t)(9);
  c2_lda_t = (ptrdiff_t)(9);
  c2_ldb_t = (ptrdiff_t)(9);
  c2_ldc_t = (ptrdiff_t)(9);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_k_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[2187], real_T c2_B[81], real_T c2_C[27])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(27);
  c2_n_t = (ptrdiff_t)(1);
  c2_k_t = (ptrdiff_t)(81);
  c2_lda_t = (ptrdiff_t)(27);
  c2_ldb_t = (ptrdiff_t)(81);
  c2_ldc_t = (ptrdiff_t)(27);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_l_eml_xgemm(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  real_T c2_A[243], real_T c2_B[81], real_T c2_C[27])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_threshold(chartInstance);
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(9);
  c2_n_t = (ptrdiff_t)(3);
  c2_k_t = (ptrdiff_t)(27);
  c2_lda_t = (ptrdiff_t)(9);
  c2_ldb_t = (ptrdiff_t)(27);
  c2_ldc_t = (ptrdiff_t)(9);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static real_T c2_get_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 0, NULL, c2_elementIndex);
  return (*chartInstance->c2_P_address)[c2_elementIndex];
}

static void c2_set_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 0, NULL, c2_elementIndex);
  (*chartInstance->c2_P_address)[c2_elementIndex] = c2_elementValue;
}

static real_T *c2_access_P(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 0, NULL);
  c2_dsmAddr = &(*chartInstance->c2_P_address)[0U];
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 0, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 1, NULL, c2_elementIndex);
  return (*chartInstance->c2_Rd_address)[c2_elementIndex];
}

static void c2_set_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 1, NULL, c2_elementIndex);
  (*chartInstance->c2_Rd_address)[c2_elementIndex] = c2_elementValue;
}

static real_T *c2_access_Rd(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 1, NULL);
  c2_dsmAddr = &(*chartInstance->c2_Rd_address)[0U];
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 1, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 2, NULL, c2_elementIndex);
  return *chartInstance->c2_g_address;
}

static void c2_set_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 2, NULL, c2_elementIndex);
  *chartInstance->c2_g_address = c2_elementValue;
}

static real_T *c2_access_g(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 2, NULL);
  c2_dsmAddr = chartInstance->c2_g_address;
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 2, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 3, NULL, c2_elementIndex);
  return *chartInstance->c2_k_address;
}

static void c2_set_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 3, NULL, c2_elementIndex);
  *chartInstance->c2_k_address = c2_elementValue;
}

static real_T *c2_access_k(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 3, NULL);
  c2_dsmAddr = chartInstance->c2_k_address;
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 3, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 4, NULL, c2_elementIndex);
  return *chartInstance->c2_kb_address;
}

static void c2_set_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                      uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 4, NULL, c2_elementIndex);
  *chartInstance->c2_kb_address = c2_elementValue;
}

static real_T *c2_access_kb(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 4, NULL);
  c2_dsmAddr = chartInstance->c2_kb_address;
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 4, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 5, NULL, c2_elementIndex);
  return *chartInstance->c2_m_address;
}

static void c2_set_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
                     uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 5, NULL, c2_elementIndex);
  *chartInstance->c2_m_address = c2_elementValue;
}

static real_T *c2_access_m(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 5, NULL);
  c2_dsmAddr = chartInstance->c2_m_address;
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 5, NULL);
  }

  return c2_dsmAddr;
}

static real_T c2_get_satMax(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex)
{
  ssReadFromDataStoreElement_wrapper(chartInstance->S, 6, NULL, c2_elementIndex);
  return *chartInstance->c2_satMax_address;
}

static void c2_set_satMax(SFc2_PackageDeliverySimInstanceStruct *chartInstance,
  uint32_T c2_elementIndex, real_T c2_elementValue)
{
  ssWriteToDataStoreElement_wrapper(chartInstance->S, 6, NULL, c2_elementIndex);
  *chartInstance->c2_satMax_address = c2_elementValue;
}

static real_T *c2_access_satMax(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance, uint32_T c2_rdOnly)
{
  real_T *c2_dsmAddr;
  ssReadFromDataStore_wrapper(chartInstance->S, 6, NULL);
  c2_dsmAddr = chartInstance->c2_satMax_address;
  if (c2_rdOnly == 0U) {
    ssWriteToDataStore_wrapper(chartInstance->S, 6, NULL);
  }

  return c2_dsmAddr;
}

static void init_dsm_address_info(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "P", (void **)
    &chartInstance->c2_P_address, &chartInstance->c2_P_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "Rd", (void **)
    &chartInstance->c2_Rd_address, &chartInstance->c2_Rd_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "g", (void **)
    &chartInstance->c2_g_address, &chartInstance->c2_g_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "k", (void **)
    &chartInstance->c2_k_address, &chartInstance->c2_k_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "kb", (void **)
    &chartInstance->c2_kb_address, &chartInstance->c2_kb_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "m", (void **)
    &chartInstance->c2_m_address, &chartInstance->c2_m_index);
  ssGetSFcnDataStoreNameAddrIdx_wrapper(chartInstance->S, "satMax", (void **)
    &chartInstance->c2_satMax_address, &chartInstance->c2_satMax_index);
}

static void init_simulink_io_address(SFc2_PackageDeliverySimInstanceStruct
  *chartInstance)
{
  chartInstance->c2_state = (real_T (*)[18])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_ref = (real_T (*)[15])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_z = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_hat_b = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c2_hat_b2 = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c2_w = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c2_h = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    6);
  chartInstance->c2_qh = (real_T (*)[4])ssGetInputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c2_cmd = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_kp = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    8);
  chartInstance->c2_kv = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    9);
  chartInstance->c2_dot_z = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_dot_hat_b = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c2_dot_hat_b2 = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c2_dot_w = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c2_dot_h = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c2_dot_qh = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c2_kz = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    10);
  chartInstance->c2_kV0 = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 11);
  chartInstance->c2_V2 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 8);
  chartInstance->c2_dot_V2 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 9);
  chartInstance->c2_kq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    12);
  chartInstance->c2_kw = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    13);
  chartInstance->c2_q0 = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 10);
  chartInstance->c2_q = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 11);
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

void sf_c2_PackageDeliverySim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3345145092U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3784500373U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4047054635U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3308490370U);
}

mxArray* sf_c2_PackageDeliverySim_get_post_codegen_info(void);
mxArray *sf_c2_PackageDeliverySim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("23m95CXp0BTyxOM4OgsoGF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,14,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(18);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,11,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c2_PackageDeliverySim_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_PackageDeliverySim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_PackageDeliverySim_jit_fallback_info(void)
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

mxArray *sf_c2_PackageDeliverySim_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_PackageDeliverySim_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c2_PackageDeliverySim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[67],T\"V2\",},{M[1],M[7],T\"cmd\",},{M[1],M[68],T\"dot_V2\",},{M[1],M[76],T\"dot_h\",},{M[1],M[37],T\"dot_hat_b\",},{M[1],M[72],T\"dot_hat_b2\",},{M[1],M[78],T\"dot_qh\",},{M[1],M[73],T\"dot_w\",},{M[1],M[38],T\"dot_z\",},{M[1],M[80],T\"q\",}}",
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[79],T\"q0\",},{M[8],M[0],T\"is_active_c2_PackageDeliverySim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 12, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_PackageDeliverySim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_PackageDeliverySimInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _PackageDeliverySimMachineNumber_,
           2,
           1,
           1,
           0,
           32,
           0,
           0,
           0,
           0,
           1,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"state");
          _SFD_SET_DATA_PROPS(1,1,1,0,"ref");
          _SFD_SET_DATA_PROPS(2,1,1,0,"z");
          _SFD_SET_DATA_PROPS(3,1,1,0,"hat_b");
          _SFD_SET_DATA_PROPS(4,1,1,0,"hat_b2");
          _SFD_SET_DATA_PROPS(5,1,1,0,"w");
          _SFD_SET_DATA_PROPS(6,1,1,0,"h");
          _SFD_SET_DATA_PROPS(7,1,1,0,"qh");
          _SFD_SET_DATA_PROPS(8,2,0,1,"cmd");
          _SFD_SET_DATA_PROPS(9,11,0,0,"m");
          _SFD_SET_DATA_PROPS(10,11,0,0,"g");
          _SFD_SET_DATA_PROPS(11,1,1,0,"kp");
          _SFD_SET_DATA_PROPS(12,1,1,0,"kv");
          _SFD_SET_DATA_PROPS(13,11,0,0,"k");
          _SFD_SET_DATA_PROPS(14,2,0,1,"dot_z");
          _SFD_SET_DATA_PROPS(15,2,0,1,"dot_hat_b");
          _SFD_SET_DATA_PROPS(16,2,0,1,"dot_hat_b2");
          _SFD_SET_DATA_PROPS(17,2,0,1,"dot_w");
          _SFD_SET_DATA_PROPS(18,2,0,1,"dot_h");
          _SFD_SET_DATA_PROPS(19,2,0,1,"dot_qh");
          _SFD_SET_DATA_PROPS(20,11,0,0,"kb");
          _SFD_SET_DATA_PROPS(21,11,0,0,"P");
          _SFD_SET_DATA_PROPS(22,1,1,0,"kz");
          _SFD_SET_DATA_PROPS(23,11,0,0,"satMax");
          _SFD_SET_DATA_PROPS(24,11,0,0,"Rd");
          _SFD_SET_DATA_PROPS(25,1,1,0,"kV0");
          _SFD_SET_DATA_PROPS(26,2,0,1,"V2");
          _SFD_SET_DATA_PROPS(27,2,0,1,"dot_V2");
          _SFD_SET_DATA_PROPS(28,1,1,0,"kq");
          _SFD_SET_DATA_PROPS(29,1,1,0,"kw");
          _SFD_SET_DATA_PROPS(30,2,0,1,"q0");
          _SFD_SET_DATA_PROPS(31,2,0,1,"q");
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
        _SFD_CV_INIT_EML(0,1,7,3,0,0,0,9,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,9748);
        _SFD_CV_INIT_EML_FCN(0,1,"sat_atan",9750,-1,10023);
        _SFD_CV_INIT_EML_FCN(0,2,"sat_atan_dot",10025,-1,10311);
        _SFD_CV_INIT_EML_FCN(0,3,"sat_atan_ddot",10321,-1,10639);
        _SFD_CV_INIT_EML_FCN(0,4,"Gamma",10641,-1,10722);
        _SFD_CV_INIT_EML_FCN(0,5,"S",10724,-1,10823);
        _SFD_CV_INIT_EML_FCN(0,6,"K",10826,-1,10963);
        _SFD_CV_INIT_EML_IF(0,1,0,9816,9834,9916,10019);
        _SFD_CV_INIT_EML_IF(0,1,1,10095,10113,10202,10307);
        _SFD_CV_INIT_EML_IF(0,1,2,10392,10410,10509,10635);
        _SFD_CV_INIT_EML_FOR(0,1,0,1207,1219,1619);
        _SFD_CV_INIT_EML_FOR(0,1,1,4713,4723,5328);
        _SFD_CV_INIT_EML_FOR(0,1,2,9843,9862,9911);
        _SFD_CV_INIT_EML_FOR(0,1,3,9929,9948,10011);
        _SFD_CV_INIT_EML_FOR(0,1,4,10122,10141,10197);
        _SFD_CV_INIT_EML_FOR(0,1,5,10215,10234,10299);
        _SFD_CV_INIT_EML_FOR(0,1,6,10419,10438,10504);
        _SFD_CV_INIT_EML_FOR(0,1,7,10522,10541,10627);
        _SFD_CV_INIT_EML_FOR(0,1,8,10874,10888,10959);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"vec",172,-1,1567);

        {
          unsigned int dimVector[1];
          dimVector[0]= 18;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)
            c2_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)
            c2_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)
            c2_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)
            c2_c_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)
            c2_k_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(23,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(24,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_j_sf_marshallOut,(MexInFcnForType)
            c2_h_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(25,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(26,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(27,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(28,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(29,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(30,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(31,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c2_state);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c2_ref);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c2_z);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c2_hat_b);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c2_hat_b2);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c2_w);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c2_h);
        _SFD_SET_DATA_VALUE_PTR(7U, *chartInstance->c2_qh);
        _SFD_SET_DATA_VALUE_PTR(8U, *chartInstance->c2_cmd);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c2_m_address);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c2_g_address);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c2_kp);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c2_kv);
        _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c2_k_address);
        _SFD_SET_DATA_VALUE_PTR(14U, *chartInstance->c2_dot_z);
        _SFD_SET_DATA_VALUE_PTR(15U, *chartInstance->c2_dot_hat_b);
        _SFD_SET_DATA_VALUE_PTR(16U, *chartInstance->c2_dot_hat_b2);
        _SFD_SET_DATA_VALUE_PTR(17U, *chartInstance->c2_dot_w);
        _SFD_SET_DATA_VALUE_PTR(18U, chartInstance->c2_dot_h);
        _SFD_SET_DATA_VALUE_PTR(19U, *chartInstance->c2_dot_qh);
        _SFD_SET_DATA_VALUE_PTR(20U, chartInstance->c2_kb_address);
        _SFD_SET_DATA_VALUE_PTR(21U, *chartInstance->c2_P_address);
        _SFD_SET_DATA_VALUE_PTR(22U, chartInstance->c2_kz);
        _SFD_SET_DATA_VALUE_PTR(23U, chartInstance->c2_satMax_address);
        _SFD_SET_DATA_VALUE_PTR(24U, *chartInstance->c2_Rd_address);
        _SFD_SET_DATA_VALUE_PTR(25U, chartInstance->c2_kV0);
        _SFD_SET_DATA_VALUE_PTR(26U, chartInstance->c2_V2);
        _SFD_SET_DATA_VALUE_PTR(27U, chartInstance->c2_dot_V2);
        _SFD_SET_DATA_VALUE_PTR(28U, chartInstance->c2_kq);
        _SFD_SET_DATA_VALUE_PTR(29U, chartInstance->c2_kw);
        _SFD_SET_DATA_VALUE_PTR(30U, *chartInstance->c2_q0);
        _SFD_SET_DATA_VALUE_PTR(31U, *chartInstance->c2_q);
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
  return "DrKiqnpw2OVCQcgVYAvGiC";
}

static void sf_opaque_initialize_c2_PackageDeliverySim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
  initialize_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_PackageDeliverySim(void *chartInstanceVar)
{
  enable_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_PackageDeliverySim(void *chartInstanceVar)
{
  disable_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_PackageDeliverySim(void *chartInstanceVar)
{
  sf_gateway_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_PackageDeliverySim(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c2_PackageDeliverySim
    ((SFc2_PackageDeliverySimInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_PackageDeliverySim(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c2_PackageDeliverySim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_PackageDeliverySimInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_PackageDeliverySim_optimization_info();
    }

    finalize_c2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
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
  initSimStructsc2_PackageDeliverySim((SFc2_PackageDeliverySimInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_PackageDeliverySim(SimStruct *S)
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
    initialize_params_c2_PackageDeliverySim
      ((SFc2_PackageDeliverySimInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_PackageDeliverySim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_PackageDeliverySim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 13, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,14);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,11);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=11; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 14; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4175238065U));
  ssSetChecksum1(S,(2014731891U));
  ssSetChecksum2(S,(3047358152U));
  ssSetChecksum3(S,(953924519U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c2_PackageDeliverySim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_PackageDeliverySim(SimStruct *S)
{
  SFc2_PackageDeliverySimInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_PackageDeliverySimInstanceStruct *)utMalloc(sizeof
    (SFc2_PackageDeliverySimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_PackageDeliverySimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_PackageDeliverySim;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_PackageDeliverySim;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_PackageDeliverySim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_PackageDeliverySim;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_PackageDeliverySim;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_PackageDeliverySim;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_PackageDeliverySim;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_PackageDeliverySim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_PackageDeliverySim;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_PackageDeliverySim;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_PackageDeliverySim;
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

void c2_PackageDeliverySim_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_PackageDeliverySim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_PackageDeliverySim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_PackageDeliverySim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_PackageDeliverySim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
