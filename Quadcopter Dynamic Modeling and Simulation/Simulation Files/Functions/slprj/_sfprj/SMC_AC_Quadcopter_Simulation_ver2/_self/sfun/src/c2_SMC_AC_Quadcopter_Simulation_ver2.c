/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SMC_AC_Quadcopter_Simulation_ver2_sfun.h"
#include "c2_SMC_AC_Quadcopter_Simulation_ver2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SMC_AC_Quadcopter_Simulation_ver2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[30] = { "Jr", "Ix", "Iy", "Iz", "l",
  "a1", "a2", "a3", "a4", "a5", "b1", "b2", "b3", "alpha1", "k1", "k2", "S",
  "OMG_r", "nargin", "nargout", "cmd_dot", "z", "p", "q", "r", "w1", "w2", "w3",
  "w4", "U" };

/* Function Declarations */
static void initialize_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void initialize_params_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void enable_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void disable_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_st);
static void finalize_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void sf_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void c2_chartstep_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void initSimStructsc2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_U, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_c_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_d_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2, const char_T
   *c2_identifier);
static uint8_T c2_e_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2 = 0U;
}

static void initialize_params_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
}

static void enable_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T *c2_U;
  c2_U = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  c2_hoistedGlobal = *c2_U;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal =
    chartInstance->c2_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_U;
  c2_U = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_U = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
    "U");
  chartInstance->c2_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2 =
    c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
    "is_active_c2_SMC_AC_Quadcopter_Simulation_ver2");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_SMC_AC_Quadcopter_Simulation_ver2(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
}

static void sf_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  real_T *c2_cmd_dot;
  real_T *c2_U;
  real_T *c2_z;
  real_T *c2_p;
  real_T *c2_q;
  real_T *c2_r;
  real_T *c2_w1;
  real_T *c2_w2;
  real_T *c2_w3;
  real_T *c2_w4;
  c2_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_r = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_q = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_U = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_cmd_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_cmd_dot, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_U, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_z, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_p, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_q, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_r, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_w1, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_w2, 7U);
  _SFD_DATA_RANGE_CHECK(*c2_w3, 8U);
  _SFD_DATA_RANGE_CHECK(*c2_w4, 9U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_SMC_AC_Quadcopter_Simulation_ver2(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
     chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_f_hoistedGlobal;
  real_T c2_g_hoistedGlobal;
  real_T c2_h_hoistedGlobal;
  real_T c2_i_hoistedGlobal;
  real_T c2_cmd_dot;
  real_T c2_z;
  real_T c2_p;
  real_T c2_q;
  real_T c2_r;
  real_T c2_w1;
  real_T c2_w2;
  real_T c2_w3;
  real_T c2_w4;
  uint32_T c2_debug_family_var_map[30];
  real_T c2_Jr;
  real_T c2_Ix;
  real_T c2_Iy;
  real_T c2_Iz;
  real_T c2_l;
  real_T c2_a1;
  real_T c2_a2;
  real_T c2_a3;
  real_T c2_a4;
  real_T c2_a5;
  real_T c2_b1;
  real_T c2_b2;
  real_T c2_b3;
  real_T c2_alpha1;
  real_T c2_k1;
  real_T c2_k2;
  real_T c2_S;
  real_T c2_OMG_r;
  real_T c2_nargin = 9.0;
  real_T c2_nargout = 1.0;
  real_T c2_U;
  real_T c2_b;
  real_T c2_y;
  real_T c2_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_b_a;
  real_T c2_d_b;
  real_T c2_d_y;
  real_T c2_e_b;
  real_T c2_e_y;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_f_b;
  real_T c2_f_y;
  real_T c2_g_b;
  real_T c2_g_y;
  real_T c2_h_b;
  real_T *c2_b_w4;
  real_T *c2_b_w3;
  real_T *c2_b_w2;
  real_T *c2_b_w1;
  real_T *c2_b_r;
  real_T *c2_b_q;
  real_T *c2_b_p;
  real_T *c2_b_z;
  real_T *c2_b_cmd_dot;
  real_T *c2_b_U;
  c2_b_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_b_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_b_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_b_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_b_r = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_b_q = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_U = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_cmd_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_cmd_dot;
  c2_b_hoistedGlobal = *c2_b_z;
  c2_c_hoistedGlobal = *c2_b_p;
  c2_d_hoistedGlobal = *c2_b_q;
  c2_e_hoistedGlobal = *c2_b_r;
  c2_f_hoistedGlobal = *c2_b_w1;
  c2_g_hoistedGlobal = *c2_b_w2;
  c2_h_hoistedGlobal = *c2_b_w3;
  c2_i_hoistedGlobal = *c2_b_w4;
  c2_cmd_dot = c2_hoistedGlobal;
  c2_z = c2_b_hoistedGlobal;
  c2_p = c2_c_hoistedGlobal;
  c2_q = c2_d_hoistedGlobal;
  c2_r = c2_e_hoistedGlobal;
  c2_w1 = c2_f_hoistedGlobal;
  c2_w2 = c2_g_hoistedGlobal;
  c2_w3 = c2_h_hoistedGlobal;
  c2_w4 = c2_i_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 30U, 30U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Jr, 0U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Ix, 1U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Iy, 2U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Iz, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_l, 4U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_a1, 5U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_a2, 6U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a3, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a4, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a5, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b1, 10U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b2, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b3, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_alpha1, 13U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_k1, 14U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_k2, 15U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_S, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_OMG_r, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_cmd_dot, 20U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_z, 21U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_p, 22U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_q, 23U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_r, 24U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_w1, 25U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_w2, 26U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_w3, 27U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_w4, 28U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_U, 29U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_Jr = 0.0003441;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_Ix = 0.013261;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_Iy = 0.013261;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_Iz = 0.025362;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_l = 0.25;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_a1 = -0.91252545056933854;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  c2_a2 = 0.02594826936128497;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  c2_a3 = 0.91252545056933854;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  c2_a4 = 0.02594826936128497;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_a5 = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  c2_b1 = 18.852273584194254;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  c2_b2 = 18.852273584194254;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  c2_b3 = 39.429067108272221;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  c2_alpha1 = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_k1 = 3.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_k2 = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
  c2_S = (c2_p - c2_cmd_dot) - c2_z;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  c2_OMG_r = ((c2_w1 + c2_w3) - c2_w2) - c2_w4;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  c2_b = c2_q;
  c2_y = 0.91252545056933854 * c2_b;
  c2_a = c2_y;
  c2_b_b = c2_r;
  c2_b_y = c2_a * c2_b_b;
  c2_c_b = c2_q;
  c2_c_y = 0.02594826936128497 * c2_c_b;
  c2_b_a = c2_c_y;
  c2_d_b = c2_OMG_r;
  c2_d_y = c2_b_a * c2_d_b;
  c2_e_b = c2_z;
  c2_e_y = c2_e_b;
  c2_x = c2_S;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarSign(c2_b_x);
  c2_f_b = c2_b_x;
  c2_f_y = 3.0 * c2_f_b;
  c2_g_b = c2_S;
  c2_g_y = c2_g_b;
  c2_h_b = (((c2_b_y - c2_d_y) - c2_e_y) - c2_f_y) - c2_g_y;
  c2_U = 0.053044 * c2_h_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -38);
  _SFD_SYMBOL_SCOPE_POP();
  *c2_b_U = c2_U;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_SMC_AC_Quadcopter_Simulation_ver2
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_U, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_U), &c2_thisId);
  sf_mex_destroy(&c2_U);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_U;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
    chartInstanceVoid;
  c2_U = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_U), &c2_thisId);
  sf_mex_destroy(&c2_U);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_SMC_AC_Quadcopter_Simulation_ver2_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 10, 1),
                FALSE);
  c2_info_helper(&c2_nameCaptureInfo);
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
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1373281308U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319704766U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685080U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685756U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286793596U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685066U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685078U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685756U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sign"), "name", "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685056U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363685756U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_sign"), "name",
                  "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1356512694U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
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
}

static const mxArray *c2_emlrt_marshallOut(char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), FALSE);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c2_y;
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_c_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i0, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_d_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2, const char_T
   *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_SMC_AC_Quadcopter_Simulation_ver2);
  return c2_y;
}

static uint8_T c2_e_emlrt_marshallIn
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info
  (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance)
{
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

void sf_c2_SMC_AC_Quadcopter_Simulation_ver2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2244743088U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3634951521U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1002564095U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3690564751U);
}

mxArray *sf_c2_SMC_AC_Quadcopter_Simulation_ver2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("skEJzdr3HFmJVYjxXHtuxC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,9,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_SMC_AC_Quadcopter_Simulation_ver2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_SMC_AC_Quadcopter_Simulation_ver2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_SMC_AC_Quadcopter_Simulation_ver2
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[4],T\"U\",},{M[8],M[0],T\"is_active_c2_SMC_AC_Quadcopter_Simulation_ver2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_SMC_AC_Quadcopter_Simulation_ver2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
    chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
           2,
           1,
           1,
           10,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation
            (_SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
             chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"cmd_dot");
          _SFD_SET_DATA_PROPS(1,2,0,1,"U");
          _SFD_SET_DATA_PROPS(2,1,1,0,"z");
          _SFD_SET_DATA_PROPS(3,1,1,0,"p");
          _SFD_SET_DATA_PROPS(4,1,1,0,"q");
          _SFD_SET_DATA_PROPS(5,1,1,0,"r");
          _SFD_SET_DATA_PROPS(6,1,1,0,"w1");
          _SFD_SET_DATA_PROPS(7,1,1,0,"w2");
          _SFD_SET_DATA_PROPS(8,1,1,0,"w3");
          _SFD_SET_DATA_PROPS(9,1,1,0,"w4");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",108,-1,847);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c2_cmd_dot;
          real_T *c2_U;
          real_T *c2_z;
          real_T *c2_p;
          real_T *c2_q;
          real_T *c2_r;
          real_T *c2_w1;
          real_T *c2_w2;
          real_T *c2_w3;
          real_T *c2_w4;
          c2_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
          c2_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c2_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c2_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c2_r = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_q = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_U = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_cmd_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_cmd_dot);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_U);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_z);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_p);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_q);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_r);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_w1);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_w2);
          _SFD_SET_DATA_VALUE_PTR(8U, c2_w3);
          _SFD_SET_DATA_VALUE_PTR(9U, c2_w4);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SMC_AC_Quadcopter_Simulation_ver2MachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "VS8WXEYUoX4LEgKErkYbZF";
}

static void sf_opaque_initialize_c2_SMC_AC_Quadcopter_Simulation_ver2(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar
     )->S,0);
  initialize_params_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
  initialize_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_SMC_AC_Quadcopter_Simulation_ver2(void
  *chartInstanceVar)
{
  enable_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_SMC_AC_Quadcopter_Simulation_ver2(void
  *chartInstanceVar)
{
  disable_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_SMC_AC_Quadcopter_Simulation_ver2(void
  *chartInstanceVar)
{
  sf_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
}

extern const mxArray*
  sf_internal_get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*)
     chartInfo->chartInstance);        /* raw sim ctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c2_SMC_AC_Quadcopter_Simulation_ver2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SimStruct* S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c2_SMC_AC_Quadcopter_Simulation_ver2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*)
     chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray*
  sf_opaque_get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2(S);
}

static void sf_opaque_set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2
  (SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2(S, st);
}

static void sf_opaque_terminate_c2_SMC_AC_Quadcopter_Simulation_ver2(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SMC_AC_Quadcopter_Simulation_ver2_optimization_info();
    }

    finalize_c2_SMC_AC_Quadcopter_Simulation_ver2
      ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_SMC_AC_Quadcopter_Simulation_ver2
    ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct *
  S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_SMC_AC_Quadcopter_Simulation_ver2
      ((SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct*)(((ChartInfoStruct
          *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct =
      load_SMC_AC_Quadcopter_Simulation_ver2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,9);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 9; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(707300062U));
  ssSetChecksum1(S,(2936264228U));
  ssSetChecksum2(S,(3213787222U));
  ssSetChecksum3(S,(2666161941U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_SMC_AC_Quadcopter_Simulation_ver2(SimStruct *S)
{
  SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct *)
    utMalloc(sizeof(SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc2_SMC_AC_Quadcopter_Simulation_ver2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_SMC_AC_Quadcopter_Simulation_ver2;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_SMC_AC_Quadcopter_Simulation_ver2_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_SMC_AC_Quadcopter_Simulation_ver2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_SMC_AC_Quadcopter_Simulation_ver2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_SMC_AC_Quadcopter_Simulation_ver2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_SMC_AC_Quadcopter_Simulation_ver2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
