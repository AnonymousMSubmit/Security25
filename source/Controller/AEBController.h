#ifndef AEBController_h_
#define AEBController_h_
#include <stdio.h>
#include <string.h>
#include "rtwtypes.h"
#include "AEBController_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include "rtw_modelmap.h"
#include <cstring>
#include <stddef.h>

// Block signals and states (default storage) for model 'AEBController'
struct DW_AEBController_T {
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T memspace;
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T WorkingSet;
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T b_obj;
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T c_WorkingSet;
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T CholManager;
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T QPObjective;
  s_aN87P1cKlEBgtxywOa8jBC_AEBController_T TrialState;
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T QRManager;
  coder_internal_stickyStruct_20_AEBController_T FcnEvaluator;
  matlabshared_tracking_internal_blocks_ekfCorrectFcn_AEBContro_T obj;// '<S10>/MATLAB System' 
  matlabshared_tracking_internal_blocks_ekfPredictFcn_AEBContro_T obj_i;// '<S12>/MATLAB System' 
  real_T driver_set_velocity[3];       // '<S8>/NLMPC Reference Values'
  real_T Delay_DSTATE[2];              // '<S3>/Delay'
  real_T mv_Delay_DSTATE[22];          // '<S16>/mv_Delay'
  real_T x_Delay_DSTATE[77];           // '<S16>/x_Delay'
  real_T P[49];                        // '<S9>/DataStoreMemory - P'
  real_T x[7];                         // '<S9>/DataStoreMemory - x'
  real_T A_data[7280];
  real_T Au[1600];
  real_T Auf_data[1600];
  real_T JacCineqTrans_data[4500];
  real_T JacCeqTrans[5250];
  real_T y_data[641];
  real_T unusedExpr[5625];
  real_T b_varargin_1_data[4500];
  real_T Jx[4900];
  real_T b_Jx[4200];
  real_T Jmv[1400];
  real_T b_Jx_data[4200];
  real_T tmp_data[1600];
  real_T a__4_data[4500];
  real_T JacEqTrans_tmp[5250];
  real_T y_data_m[641];
  real_T work_data[641];
  real_T work_data_c[641];
  real_T vn1_data[641];
  real_T vn2_data[641];
  real_T work_data_k[641];
  real_T y_data_c[410881];
  real_T b_data[641];
  real_T B_data[228196];
  real_T y_data_b[641];
  real_T varargin_1_data[4500];
  real_T y_data_p[641];
  real_T headway;                      // '<S6>/Sum'
  real_T TTC;                          // '<S6>/Divide3'
  real_T FCWstoppingTime;              // '<S5>/Sum1'
  real_T PB1stoppingTime;              // '<S5>/Sum2'
  real_T PB2stoppingTime;              // '<S5>/Sum3'
  real_T FBstoppingTime;               // '<S5>/Sum4'
  real_T decel;                        // '<S1>/AEBLogic'
  real_T Delay3_DSTATE;                // '<S2>/Delay3'
  real_T slack_delay_DSTATE;           // '<S16>/slack_delay'
  uint8_T Delay_DSTATE_c;              // '<S1>/Delay'
  uint8_T is_active_c5_AEBController;  // '<S1>/AEBLogic'
  uint8_T is_c5_AEBController;         // '<S1>/AEBLogic'
  boolean_T Delay2_DSTATE;             // '<S2>/Delay2'
  boolean_T icLoad;                    // '<S16>/mv_Delay'
  boolean_T icLoad_f;                  // '<S16>/x_Delay'
  boolean_T icLoad_k;                  // '<S16>/slack_delay'
  boolean_T objisempty;                // '<S12>/MATLAB System'
  boolean_T objisempty_h;              // '<S10>/MATLAB System'
};

// Real-time Model Data Structure
struct tag_RTM_AEBController_T {
  //
  //  DataMapInfo:
  //  The following substructure contains information regarding
  //  structures generated in the model's C API.

  struct DataMapInfo_T {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[17];
    int32_T* vardimsAddress[17];
    RTWLoggingFcnPtr loggingPtrs[17];
  };

  DataMapInfo_T DataMapInfo;
  RT_MODEL_AEBController_T::DataMapInfo_T getDataMapInfo() const;
  void setDataMapInfo(RT_MODEL_AEBController_T::DataMapInfo_T aDataMapInfo);
};

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  AEBController_GetCAPIStaticMap(void);

// Class declaration for model AEBController
class ACCWithSensorFusionModelClass
{
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // Initial conditions function
  void init();

  // Real-Time Model get method
  RT_MODEL_AEBController_T * getRTM();

  // member function to set up the C-API information
  void setupCAPIInfo(rtwCAPI_ModelMappingInfo *rt_ParentMMI, const char_T
                     *rt_ChildPath, int_T rt_ChildMMIIdx, int_T rt_CSTATEIdx);

  // model step function
  void step(const real_T *rtu_LongitudinalVelocity, const real_T
            rtu_CurvatureSequence[10], const real_T *rtu_LateralDeviation, const
            real_T *rtu_RelativeYawAngle, const real_T *rtu_RelativeDistance,
            const real_T *rtu_RelativeVelocity, uint8_T *rty_AEBStatus, uint8_T *
            rty_FCWActivate, boolean_T *rty_EgoCarStop, real_T
            *rty_SteeringAngle, real_T *rty_Throttle);

  // Block states
  DW_AEBController_T AEBController_DW;

  // Constructor
  ACCWithSensorFusionModelClass();

  // Destructor
  ~ACCWithSensorFusionModelClass();

  // private data and function members
 private:
  // private member function(s) for subsystem '<Root>/TmpModelReferenceSubsystem'
  real_T AEBController_xnrm2(int32_T n, const real_T x[30], int32_T ix0);
  void AEBController_qr(const real_T A[30], real_T Q[30], real_T R[9]);
  void AEBController_trisolve(const real_T A[9], real_T B[21]);
  void AEBController_trisolve_l(const real_T A[9], real_T B[21]);
  real_T AEBController_xnrm2_p(int32_T n, const real_T x[70], int32_T ix0);
  void AEBController_qr_c(const real_T A[70], real_T Q[70], real_T R[49]);
  void AEBController_mtimes(const real_T A_data[], const int32_T A_size[2],
    real_T C_data[], int32_T C_size[2]);
  boolean_T AEBController_any(const boolean_T x[6]);
  void AEBController_c4_mpclib_anonFcn2(const real_T runtimedata_x[7], const
    real_T runtimedata_md[11], const real_T runtimedata_OutputMin[30], const
    real_T runtimedata_OutputMax[30], const real_T runtimedata_Parameters[7],
    const real_T z[75], real_T varargout_1_data[], int32_T varargout_1_size[2],
    real_T varargout_2[70], real_T varargout_3_data[], int32_T varargout_3_size
    [2], real_T varargout_4[5250]);
  void AEBController_factoryConstruct(int32_T nVarMax, int32_T mConstrMax,
    int32_T mIneq, int32_T mNonlinIneq, s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
    *obj);
  void AEBController_factoryConstruct_oogs(int32_T MaxVars, int32_T
    obj_grad_size[1], int32_T obj_Hx_size[1], boolean_T *obj_hasLinear, int32_T *
    obj_nvar, int32_T *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T
    *obj_objtype, int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
    *obj_prev_hasLinear, real_T *obj_gammaScalar);
  void AEBController_factoryConstruct_oogsz(int32_T mIneqMax, int32_T nVarMax,
    int32_T mConstrMax, s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj);
  int32_T AEBController_checkVectorNonFinite(int32_T N, const real_T vec_data[],
    int32_T iv0);
  int32_T AEBController_computeConstraintsAndUserJacobian_(int32_T
    obj_next_next_next_next_next_b_value, const
    s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const
    real_T x[75], real_T Cineq_workspace_data[], int32_T ineq0, real_T
    Ceq_workspace[70], real_T JacIneqTrans_workspace_data[], int32_T iJI_col,
    int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T ldJE);
  void AEBController_evalObjAndConstrAndDerivatives(int32_T
    obj_next_next_next_next_next_b_value, const
    s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const
    s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
    *obj_next_next_next_next_next_next_next_next_b_value_workspace, const real_T
    x[75], real_T grad_workspace_data[], real_T Cineq_workspace_data[], int32_T
    ineq0, real_T Ceq_workspace[70], real_T JacIneqTrans_workspace_data[],
    int32_T iJI_col, int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T
    ldJE, real_T *fval, int32_T *status);
  void AEBController_modifyOverheadPhaseOne_
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj);
  void AEBController_setProblemType(s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
    *obj, int32_T PROBLEM_TYPE);
  void AEBController_initActiveSet(s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj);
  void AEBController_factoryConstruct_oo(int32_T maxRows, int32_T maxCols,
    int32_T *obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T
    obj_Q_size[2], int32_T obj_jpvt_data[], int32_T obj_jpvt_size[1], int32_T
    *obj_mrows, int32_T *obj_ncols, int32_T obj_tau_size[1], int32_T
    *obj_minRowCol, boolean_T *obj_usedPivoting);
  void AEBController_factoryConstruct_oog(int32_T MaxDims, int32_T
    obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info,
    real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_,
    real_T *obj_workspace_, real_T *obj_workspace2_);
  void AEBController_computeGradLag(real_T workspace_data[], int32_T ldA,
    int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
    AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
    finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T
    mLB, const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
  real_T AEBController_computePrimalFeasError(const real_T x[75], int32_T
    mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq
    [70], const int32_T finiteLB_data[], int32_T mLB, const real_T lb[75], const
    int32_T finiteUB_data[], int32_T mUB);
  void AEBController_computeDualFeasError(int32_T nVar, const real_T
    gradLag_data[], boolean_T *gradOK, real_T *val);
  void AEBController_test_exit(sG8JZ69axY52WWR6RKyApQC_AEBController_T
    *MeritFunction, const s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState, const real_T lb[75],
    boolean_T *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done,
    boolean_T *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
    *Flags_stepType);
  void AEBController_saveJacobian(s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *obj,
    int32_T nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T
    ineqCol0, const real_T JacCeqTrans_data[], int32_T ldJ);
  void AEBController_updateWorkingSetForNewQP(const real_T xk[75],
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet, int32_T mIneq, int32_T
    mNonlinIneq, const real_T cIneq_data[], const real_T cEq[70], int32_T mLB,
    const real_T lb[75], int32_T mUB, int32_T mFixed);
  void AEBController_xgemv_jty(int32_T m, int32_T n, const real_T A_data[],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  real_T AEBController_maxConstraintViolation_AMats_nonregularized_
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[]);
  real_T AEBController_maxConstraintViolation_AMats_regularized_
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[]);
  real_T AEBController_maxConstraintViolation_dsy
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[]);
  void AEBController_xgemv_jtyg(int32_T m, int32_T n, const real_T A[5625],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  void AEBController_computeGrad_StoreHx
    (s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *obj, const real_T H[5625], const
     real_T f_data[], const real_T x_data[]);
  real_T AEBController_computeFval_ReuseHx(const
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *obj, real_T workspace_data[],
    const real_T f_data[], const real_T x_data[]);
  real_T AEBController_xnrm2_pf(int32_T n, const real_T x_data[], int32_T ix0);
  real_T AEBController_xzlarfg(int32_T n, real_T *alpha1, real_T x_data[],
    int32_T ix0);
  void AEBController_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T
    ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
  void AEBController_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
    const real_T y_data[], real_T A_data[], int32_T ia0, int32_T lda);
  void AEBController_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau,
    real_T C_data[], int32_T ic0, int32_T ldc, real_T work_data[]);
  void AEBController_qrf(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, int32_T nfxd, real_T tau_data[]);
  void AEBController_xgeqrf(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, real_T tau_data[], int32_T tau_size[1]);
  void AEBController_factorQR(s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj,
    const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
  void AEBController_squareQ_appendCol(s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T
    *obj, const real_T vec_data[], int32_T iv0);
  void AEBController_deleteColMoveEnd(s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T
    *obj, int32_T idx);
  boolean_T AEBController_strcmp(const char_T a[7]);
  void AEBController_fullColLDL2_(s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj,
    int32_T LD_offset, int32_T NColsRemain);
  void AEBController_partialColLDL3_(s_NZ75DufWqh8NxBuGZX5zO_AEBController_T
    *obj, int32_T LD_offset, int32_T NColsRemain);
  void AEBController_factor_i(s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj,
    const real_T A[5625], int32_T ndims, int32_T ldA);
  int32_T AEBController_xpotrf(int32_T n, real_T A_data[], int32_T lda);
  void AEBController_factor(s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, const
    real_T A[5625], int32_T ndims, int32_T ldA);
  void AEBController_xgemm(int32_T m, int32_T n, int32_T k, const real_T A[5625],
    int32_T lda, const real_T B_data[], int32_T ib0, int32_T ldb, real_T C_data[],
    int32_T ldc);
  void AEBController_xgemm_p(int32_T m, int32_T n, int32_T k, const real_T
    A_data[], int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb,
    real_T C_data[], int32_T ldc);
  void AEBController_xgemv_jtygq(int32_T m, int32_T n, const real_T A_data[],
    int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[]);
  void AEBController_solve(const s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj,
    real_T rhs_data[]);
  void AEBController_solve_j(const s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj,
    real_T rhs_data[]);
  void AEBController_compute_deltax(const real_T H[5625],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager, const
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, boolean_T
    alwaysPositiveDef);
  real_T AEBController_xnrm2_pfp(int32_T n, const real_T x_data[]);
  void AEBController_xgemv_jtygqg(int32_T m, int32_T n, const real_T A_data[],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  void AEBController_feasibleratiotest(const real_T solution_xstar_data[], const
    real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
    workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
    real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
    real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
    int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
    boolean_T workingset_isActiveConstr_data[], const int32_T
    workingset_nWConstr[5], boolean_T isPhaseOne, real_T *alpha, boolean_T
    *newBlocking, int32_T *constrType, int32_T *constrIdx);
  void AEBController_compute_lambda(real_T workspace_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution, const
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager);
  void AEBController_xorgqr(int32_T m, int32_T n, int32_T k, real_T A_data[],
    const int32_T A_size[2], int32_T lda, const real_T tau_data[]);
  void AEBController_checkUnboundedOrIllPosed
    (s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution, const
     s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective);
  void AEBController_addBoundToActiveSetMatrix_
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T TYPE, int32_T
     idx_local);
  void AEBController_addAineqConstr(s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
    *obj, int32_T idx_local);
  void AEBController_removeConstr(s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj,
    int32_T idx_global);
  void AEBController_xswap(int32_T n, real_T x_data[], int32_T ix0, int32_T iy0);
  void AEBController_qrpf(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, int32_T nfxd, real_T tau_data[], int32_T jpvt_data[]);
  void AEBController_xgeqp3(real_T A_data[], const int32_T A_size[2], int32_T m,
    int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T tau_size[1]);
  void AEBController_factorQRE(s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj,
    const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
  void AEBController_factorQRE_i(s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj,
    int32_T mrows, int32_T ncols);
  int32_T AEBController_rank(const real_T qrmanager_QR_data[], const int32_T
    qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols);
  void AEBController_xgemv_j(int32_T m, int32_T n, const real_T A_data[],
    int32_T lda, const real_T x_data[], real_T y_data[]);
  real_T AEBController_maxConstraintViolation_d
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[]);
  void AEBController_xgemv_jt(int32_T m, int32_T n, const real_T A_data[],
    int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
  real_T AEBController_maxConstraintViolation_ds
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[],
     int32_T ix0);
  boolean_T AEBController_feasibleX0ForWorkingSet(real_T workspace_data[], const
    int32_T workspace_size[2], real_T xCurrent_data[],
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager);
  void AEBController_checkStoppingAndUpdateFval_p(int32_T *activeSetChangeID,
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager, int32_T
    runTimeOptions_MaxIterations, boolean_T *updateFval);
  void AEBController_iterate_k(const real_T H[5625], const real_T f_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
    options_SolverName[7], int32_T runTimeOptions_MaxIterations);
  void AEBController_checkStoppingAndUpdateFval(int32_T *activeSetChangeID,
    const real_T f_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager, int32_T
    runTimeOptions_MaxIterations, const boolean_T *updateFval);
  void AEBController_iterate(const real_T H[5625], const real_T f_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
    options_SolverName[7], int32_T runTimeOptions_MaxIterations);
  void AEBController_phaseone(const real_T H[5625], const real_T f_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
    options_SolverName[7], const somzaGboVhDG7PNQS6E98jD_AEBController_T
    *runTimeOptions);
  void AEBController_countsort(int32_T x_data[], int32_T xLen, int32_T
    workspace_data[], int32_T xMin, int32_T xMax);
  int32_T AEBController_RemoveDependentEq_
    (s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
     s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
     s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager);
  void AEBController_RemoveDependentIneq_
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
     s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
     s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace);
  void AEBController_RemoveDependentIneq__j
    (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
     s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
     s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace);
  void AEBController_PresolveWorkingSet(s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
    *solution, s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager);
  void AEBController_linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
    real_T workspace_data[], const real_T H[5625], const real_T f_data[], const
    real_T x_data[]);
  void AEBController_driver_g(const real_T H[5625], const real_T f_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const
    somzaGboVhDG7PNQS6E98jD_AEBController_T *options, const
    somzaGboVhDG7PNQS6E98jD_AEBController_T *runTimeOptions);
  void AEBController_addAeqConstr(s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj,
    int32_T idx_local);
  void AEBController_sortLambdaQP(real_T lambda_data[], int32_T
    WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
    WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const
    int32_T WorkingSet_Wlocalidx_data[], real_T workspace_data[]);
  boolean_T AEBController_soc(const real_T Hessian[5625], const real_T
    grad_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const
    somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions);
  real_T AEBController_maxConstraintViolation(const
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[]);
  void AEBController_normal(const real_T Hessian[5625], const real_T grad_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const
    somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions,
    s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *stepFlags);
  void AEBController_relaxed(const real_T Hessian[5625], const real_T grad_data[],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective,
    somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions);
  void AEBController_step(s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *stepFlags,
    real_T Hessian[5625], const real_T lb[75],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective,
    somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions);
  void AEBController_evalObjAndConstr(int32_T
    obj_next_next_next_next_next_b_value, const
    s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const
    s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
    *obj_next_next_next_next_next_next_next_next_b_value_workspace, const real_T
    x[75], real_T Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[70],
    real_T *fval, int32_T *status);
  void AEBController_computeLinearResiduals(const real_T x[75], int32_T nVar,
    real_T workspaceIneq_data[], const int32_T workspaceIneq_size[1], int32_T
    mLinIneq, const real_T AineqT_data[], const real_T bineq_data[], int32_T
    ldAi);
  real_T AEBController_computeMeritFcn(real_T obj_penaltyParam, real_T fval,
    const real_T Cineq_workspace_data[], int32_T mIneq, const real_T
    Ceq_workspace[70], boolean_T evalWellDefined);
  void AEBController_linesearch(boolean_T *evalWellDefined, const real_T
    bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const real_T
    WorkingSet_Aineq_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
    *TrialState, real_T MeritFunction_penaltyParam, real_T MeritFunction_phi,
    real_T MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
    FcnEvaluator_next_next_next_next_next_b_value, const
    s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *FcnEvaluator_next_next_next_next_next_next_next_b_value_workspa, const
    s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
    *FcnEvaluator_next_next_next_next_next_next_next_next_b_value_wo, boolean_T
    socTaken, real_T *alpha, int32_T *exitflag);
  real_T AEBController_computeComplError(const int32_T
    fscales_lineq_constraint_size[1], const int32_T
    fscales_cineq_constraint_size[1], const real_T xCurrent[75], int32_T mIneq,
    const real_T cIneq_data[], const int32_T finiteLB_data[], int32_T mLB, const
    real_T lb[75], const int32_T finiteUB_data[], int32_T mUB, const real_T
    lambda_data[], int32_T iL0);
  void AEBController_computeGradLag_b(real_T workspace_data[], int32_T ldA,
    int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
    AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
    finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T
    mLB, const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
  void AEBController_computeDualFeasError_c(int32_T nVar, const real_T
    gradLag_data[], boolean_T *gradOK, real_T *val);
  void AEBController_test_exit_n(s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *Flags,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction, const int32_T
    fscales_lineq_constraint_size[1], const int32_T
    fscales_cineq_constraint_size[1], s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
    *WorkingSet, s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager, const real_T lb[75]);
  boolean_T AEBController_BFGSUpdate(int32_T nvar, real_T Bk[5625], const real_T
    sk_data[], real_T yk_data[], real_T workspace_data[]);
  void AEBController_driver(const real_T bineq_data[], const real_T lb[75],
    s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
    sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction, const
    coder_internal_stickyStruct_20_AEBController_T *FcnEvaluator,
    s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
    s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
    s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
    s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
    s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const int32_T
    fscales_lineq_constraint_size[1], const int32_T
    fscales_cineq_constraint_size[1], real_T Hessian[5625]);
  void AEBController_fmincon(const s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *fun_workspace_runtimedata, const sjqToqDl6Xs23mBXoA194vG_AEBController_T
    *fun_workspace_userdata, const real_T x0[75], const real_T Aineq_data[],
    const real_T bineq_data[], const int32_T bineq_size[1], const real_T lb[75],
    const s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
    *nonlcon_workspace_runtimedata, real_T x[75], real_T *fval, real_T *exitflag,
    real_T *output_iterations, real_T *output_funcCount, char_T
    output_algorithm[3], real_T *output_constrviolation, real_T *output_stepsize,
    real_T *output_lssteplength, real_T *output_firstorderopt);
  void AEBController_helperEKFStateFcn(real_T xk[7], const real_T u[12]);
  real_T AEBController_xnrm2_po(int32_T n, const real_T x[98], int32_T ix0);
  void AEBController_qr_c0(const real_T A[98], real_T Q[98], real_T R[49]);

  // Real-Time Model
  RT_MODEL_AEBController_T AEBController_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S9>/checkMeasurementFcn1Signals' : Unused code path elimination
//  Block '<S9>/checkStateTransitionFcnSignals' : Unused code path elimination
//  Block '<S17>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S18>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S19>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S20>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S21>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S22>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S23>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S24>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S25>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S26>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S27>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S28>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S29>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S30>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S31>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S32>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S33>/Vector Dimension Check' : Unused code path elimination
//  Block '<S34>/Vector Dimension Check' : Unused code path elimination
//  Block '<S35>/Vector Dimension Check' : Unused code path elimination
//  Block '<S36>/Vector Dimension Check' : Unused code path elimination
//  Block '<S14>/mv.init_zero' : Unused code path elimination
//  Block '<S14>/x.init_zero' : Unused code path elimination
//  Block '<S9>/DataTypeConversion_Enable1' : Eliminate redundant data type conversion
//  Block '<S9>/DataTypeConversion_Q' : Eliminate redundant data type conversion
//  Block '<S9>/DataTypeConversion_R1' : Eliminate redundant data type conversion
//  Block '<S9>/DataTypeConversion_uMeas1' : Eliminate redundant data type conversion
//  Block '<S9>/DataTypeConversion_uState' : Eliminate redundant data type conversion
//  Block '<S9>/DataTypeConversion_y1' : Eliminate redundant data type conversion
//  Block '<S7>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S15>/Reshape' : Reshape block reduction
//  Block '<S15>/Reshape1' : Reshape block reduction
//  Block '<S15>/mo or x Conversion' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion1' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion10' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion11' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion12' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion13' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion14' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion15' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion16' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion17' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion18' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion19' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion2' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion3' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion4' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion5' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion6' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion7' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion8' : Eliminate redundant data type conversion
//  Block '<S15>/mo or x Conversion9' : Eliminate redundant data type conversion
//  Block '<S16>/reshape_mv' : Reshape block reduction
//  Block '<S16>/reshape_x' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'AEBController'
//  '<S1>'   : 'AEBController/Braking Controller'
//  '<S2>'   : 'AEBController/Controller Mode Selector'
//  '<S3>'   : 'AEBController/NLMPC Controller'
//  '<S4>'   : 'AEBController/Braking Controller/AEBLogic'
//  '<S5>'   : 'AEBController/Braking Controller/StoppingTimeCalculation'
//  '<S6>'   : 'AEBController/Braking Controller/TTCCalculation'
//  '<S7>'   : 'AEBController/NLMPC Controller/Extended Kalman Filter'
//  '<S8>'   : 'AEBController/NLMPC Controller/Nonlinear MPC Controller'
//  '<S9>'   : 'AEBController/NLMPC Controller/Extended Kalman Filter/Nonlinear State Estimator  (Extended Kalman Filter)'
//  '<S10>'  : 'AEBController/NLMPC Controller/Extended Kalman Filter/Nonlinear State Estimator  (Extended Kalman Filter)/Correct1'
//  '<S11>'  : 'AEBController/NLMPC Controller/Extended Kalman Filter/Nonlinear State Estimator  (Extended Kalman Filter)/Output'
//  '<S12>'  : 'AEBController/NLMPC Controller/Extended Kalman Filter/Nonlinear State Estimator  (Extended Kalman Filter)/Predict'
//  '<S13>'  : 'AEBController/NLMPC Controller/Extended Kalman Filter/Nonlinear State Estimator  (Extended Kalman Filter)/Output/MATLAB Function'
//  '<S14>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller'
//  '<S15>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC'
//  '<S16>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/xmvs_router'
//  '<S17>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check'
//  '<S18>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check1'
//  '<S19>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check10'
//  '<S20>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check11'
//  '<S21>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check12'
//  '<S22>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check13'
//  '<S23>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check14'
//  '<S24>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check15'
//  '<S25>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check16'
//  '<S26>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check3'
//  '<S27>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check4'
//  '<S28>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check5'
//  '<S29>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check6'
//  '<S30>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check7'
//  '<S31>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check8'
//  '<S32>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Preview Signal Check9'
//  '<S33>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check1'
//  '<S34>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check2'
//  '<S35>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Vector Signal Check1'
//  '<S36>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/MPC Vector Signal Check11'
//  '<S37>'  : 'AEBController/NLMPC Controller/Nonlinear MPC Controller/Nonlinear MPC Controller/MPC/NLMPC'

#endif                                 // AEBController_h_

//
// File trailer for generated code.
//
// [EOF]
//
