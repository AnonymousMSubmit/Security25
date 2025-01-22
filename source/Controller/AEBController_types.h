#ifndef AEBController_types_h_
#define AEBController_types_h_
#include "rtwtypes.h"
#include "coder_bounded_array.h"
#ifndef DEFINED_TYPEDEF_FOR_paramsBusObject_
#define DEFINED_TYPEDEF_FOR_paramsBusObject_

struct paramsBusObject
{
  real_T Constant;
  real_T Constant3;
  real_T Constant1;
  real_T Constant2;
  real_T Constant4;
  real_T Constant5;
  real_T Constant6;
};

#endif

#ifndef struct_matlabshared_tracking_internal_blocks_ekfCorrectFcn_AEBContro_T
#define struct_matlabshared_tracking_internal_blocks_ekfCorrectFcn_AEBContro_T

struct matlabshared_tracking_internal_blocks_ekfCorrectFcn_AEBContro_T
{
  int32_T isInitialized;
};

#endif
      // struct_matlabshared_tracking_internal_blocks_ekfCorrectFcn_AEBContro_T

#ifndef struct_matlabshared_tracking_internal_blocks_ekfPredictFcn_AEBContro_T
#define struct_matlabshared_tracking_internal_blocks_ekfPredictFcn_AEBContro_T

struct matlabshared_tracking_internal_blocks_ekfPredictFcn_AEBContro_T
{
  int32_T isInitialized;
};

#endif
      // struct_matlabshared_tracking_internal_blocks_ekfPredictFcn_AEBContro_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_sG8JZ69axY52WWR6RKyApQC_AEBController_T
#define struct_sG8JZ69axY52WWR6RKyApQC_AEBController_T

struct sG8JZ69axY52WWR6RKyApQC_AEBController_T
{
  real_T penaltyParam;
  real_T threshold;
  int32_T nPenaltyDecreases;
  real_T linearizedConstrViol;
  real_T initFval;
  real_T initConstrViolationEq;
  real_T initConstrViolationIneq;
  real_T phi;
  real_T phiPrimePlus;
  real_T phiFullStep;
  real_T feasRelativeFactor;
  real_T nlpPrimalFeasError;
  real_T nlpDualFeasError;
  real_T nlpComplError;
  real_T firstOrderOpt;
  boolean_T hasObjective;
};

#endif                        // struct_sG8JZ69axY52WWR6RKyApQC_AEBController_T

#ifndef struct_s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T
#define struct_s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T

struct s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T
{
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int32_T stepType;
};

#endif                        // struct_s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T

#ifndef struct_sjqToqDl6Xs23mBXoA194vG_AEBController_T
#define struct_sjqToqDl6Xs23mBXoA194vG_AEBController_T

struct sjqToqDl6Xs23mBXoA194vG_AEBController_T
{
  real_T Ts;
  real_T CurrentStates[7];
  real_T LastMV[2];
  real_T References[30];
  real_T MVTarget[20];
  real_T PredictionHorizon;
  real_T NumOfStates;
  real_T NumOfOutputs;
  real_T NumOfInputs;
  real_T MVIndex[2];
  real_T MDIndex;
  real_T UDIndex;
  real_T InputPassivityIndex;
  real_T OutputPassivityIndex;
  boolean_T PassivityUsePredictedX;
};

#endif                        // struct_sjqToqDl6Xs23mBXoA194vG_AEBController_T

#ifndef struct_somzaGboVhDG7PNQS6E98jD_AEBController_T
#define struct_somzaGboVhDG7PNQS6E98jD_AEBController_T

struct somzaGboVhDG7PNQS6E98jD_AEBController_T
{
  char_T SolverName[7];
  int32_T MaxIterations;
  real_T StepTolerance;
  real_T OptimalityTolerance;
  real_T ConstraintTolerance;
  real_T ObjectiveLimit;
  real_T PricingTolerance;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
  boolean_T IterDisplayQP;
};

#endif                        // struct_somzaGboVhDG7PNQS6E98jD_AEBController_T

#ifndef struct_s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
#define struct_s_tD85q5PFvXiFImVLjXNPSB_AEBController_T

struct s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
{
  real_T x[7];
  real_T lastMV[2];
  real_T ref[30];
  real_T md[11];
  real_T OutputWeights[30];
  real_T MVWeights[20];
  real_T MVRateWeights[20];
  real_T ECRWeight;
  real_T OutputMin[30];
  real_T OutputMax[30];
  real_T StateMin[70];
  real_T StateMax[70];
  real_T MVMin[20];
  real_T MVMax[20];
  real_T MVRateMin[20];
  real_T MVRateMax[20];
  real_T MVScaledTarget[20];
  real_T Parameters[7];
};

#endif                       // struct_s_tD85q5PFvXiFImVLjXNPSB_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
#define struct_s_aN87P1cKlEBgtxywOa8jBC_AEBController_T

struct s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
{
  int32_T nVarMax;
  int32_T mNonlinIneq;
  int32_T mNonlinEq;
  int32_T mIneq;
  int32_T mEq;
  int32_T iNonIneq0;
  int32_T iNonEq0;
  real_T sqpFval;
  real_T sqpFval_old;
  real_T xstarsqp[75];
  real_T xstarsqp_old[75];
  coder::bounded_array<real_T, 140U, 1U> cIneq;
  coder::bounded_array<real_T, 140U, 1U> cIneq_old;
  real_T cEq[70];
  real_T cEq_old[70];
  coder::bounded_array<real_T, 356U, 1U> grad;
  coder::bounded_array<real_T, 356U, 1U> grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  coder::bounded_array<real_T, 641U, 1U> lambdasqp;
  coder::bounded_array<real_T, 641U, 1U> lambdaStopTest;
  coder::bounded_array<real_T, 641U, 1U> lambdaStopTestPrev;
  real_T steplength;
  coder::bounded_array<real_T, 641U, 1U> delta_x;
  coder::bounded_array<real_T, 356U, 1U> socDirection;
  coder::bounded_array<int32_T, 641U, 1U> workingset_old;
  coder::bounded_array<real_T, 21360U, 2U> JacCineqTrans_old;
  coder::bounded_array<real_T, 24920U, 2U> JacCeqTrans_old;
  coder::bounded_array<real_T, 356U, 1U> gradLag;
  coder::bounded_array<real_T, 356U, 1U> delta_gradLag;
  coder::bounded_array<real_T, 641U, 1U> xstar;
  real_T fstar;
  real_T firstorderopt;
  coder::bounded_array<real_T, 641U, 1U> lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  coder::bounded_array<real_T, 641U, 1U> searchDir;
};

#endif                       // struct_s_aN87P1cKlEBgtxywOa8jBC_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T
#define struct_s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T

struct s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T
{
  coder::bounded_array<real_T, 228196U, 2U> workspace_float;
  coder::bounded_array<int32_T, 641U, 1U> workspace_int;
  coder::bounded_array<int32_T, 641U, 1U> workspace_sort;
};

#endif                       // struct_s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
#define struct_s_H2r48KAwe9QhSMXfaacI5D_AEBController_T

struct s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  coder::bounded_array<real_T, 49840U, 1U> Aineq;
  coder::bounded_array<real_T, 140U, 1U> bineq;
  coder::bounded_array<real_T, 24920U, 1U> Aeq;
  real_T beq[70];
  coder::bounded_array<real_T, 356U, 1U> lb;
  coder::bounded_array<real_T, 356U, 1U> ub;
  coder::bounded_array<int32_T, 356U, 1U> indexLB;
  coder::bounded_array<int32_T, 356U, 1U> indexUB;
  coder::bounded_array<int32_T, 356U, 1U> indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[70];
  coder::bounded_array<real_T, 228196U, 1U> ATwset;
  coder::bounded_array<real_T, 641U, 1U> bwset;
  int32_T nActiveConstr;
  coder::bounded_array<real_T, 641U, 1U> maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  coder::bounded_array<boolean_T, 641U, 1U> isActiveConstr;
  coder::bounded_array<int32_T, 641U, 1U> Wid;
  coder::bounded_array<int32_T, 641U, 1U> Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

#endif                       // struct_s_H2r48KAwe9QhSMXfaacI5D_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T
#define struct_s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T

struct s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T
{
  coder::bounded_array<real_T, 356U, 1U> grad;
  coder::bounded_array<real_T, 355U, 1U> Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

#endif                       // struct_s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T
#define struct_s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T

struct s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T
{
  int32_T ldq;
  coder::bounded_array<real_T, 410881U, 2U> QR;
  coder::bounded_array<real_T, 410881U, 2U> Q;
  coder::bounded_array<int32_T, 641U, 1U> jpvt;
  int32_T mrows;
  int32_T ncols;
  coder::bounded_array<real_T, 641U, 1U> tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
};

#endif                        // struct_s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T

#ifndef struct_s_NZ75DufWqh8NxBuGZX5zO_AEBController_T
#define struct_s_NZ75DufWqh8NxBuGZX5zO_AEBController_T

struct s_NZ75DufWqh8NxBuGZX5zO_AEBController_T
{
  coder::bounded_array<real_T, 410881U, 2U> FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_;
  real_T workspace2_;
};

#endif                        // struct_s_NZ75DufWqh8NxBuGZX5zO_AEBController_T

// Custom Type definition for MATLAB Function: '<S15>/NLMPC'
#ifndef struct_s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
#define struct_s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T

struct s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
{
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T runtimedata;
  sjqToqDl6Xs23mBXoA194vG_AEBController_T userdata;
};

#endif                       // struct_s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T

#ifndef struct_anonymous_function_AEBController_T
#define struct_anonymous_function_AEBController_T

struct anonymous_function_AEBController_T
{
  s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T workspace;
};

#endif                             // struct_anonymous_function_AEBController_T

#ifndef struct_coder_internal_stickyStruct_12_AEBController_T
#define struct_coder_internal_stickyStruct_12_AEBController_T

struct coder_internal_stickyStruct_12_AEBController_T
{
  anonymous_function_AEBController_T b_value;
};

#endif                 // struct_coder_internal_stickyStruct_12_AEBController_T

#ifndef struct_coder_internal_stickyStruct_13_AEBController_T
#define struct_coder_internal_stickyStruct_13_AEBController_T

struct coder_internal_stickyStruct_13_AEBController_T
{
  anonymous_function_AEBController_T b_value;
  coder_internal_stickyStruct_12_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_13_AEBController_T

#ifndef struct_coder_internal_stickyStruct_14_AEBController_T
#define struct_coder_internal_stickyStruct_14_AEBController_T

struct coder_internal_stickyStruct_14_AEBController_T
{
  coder_internal_stickyStruct_13_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_14_AEBController_T

#ifndef struct_coder_internal_stickyStruct_15_AEBController_T
#define struct_coder_internal_stickyStruct_15_AEBController_T

struct coder_internal_stickyStruct_15_AEBController_T
{
  int32_T b_value;
  coder_internal_stickyStruct_14_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_15_AEBController_T

#ifndef struct_coder_internal_stickyStruct_16_AEBController_T
#define struct_coder_internal_stickyStruct_16_AEBController_T

struct coder_internal_stickyStruct_16_AEBController_T
{
  coder_internal_stickyStruct_15_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_16_AEBController_T

#ifndef struct_coder_internal_stickyStruct_17_AEBController_T
#define struct_coder_internal_stickyStruct_17_AEBController_T

struct coder_internal_stickyStruct_17_AEBController_T
{
  coder_internal_stickyStruct_16_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_17_AEBController_T

#ifndef struct_coder_internal_stickyStruct_18_AEBController_T
#define struct_coder_internal_stickyStruct_18_AEBController_T

struct coder_internal_stickyStruct_18_AEBController_T
{
  coder_internal_stickyStruct_17_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_18_AEBController_T

#ifndef struct_coder_internal_stickyStruct_19_AEBController_T
#define struct_coder_internal_stickyStruct_19_AEBController_T

struct coder_internal_stickyStruct_19_AEBController_T
{
  coder_internal_stickyStruct_18_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_19_AEBController_T

#ifndef struct_coder_internal_stickyStruct_20_AEBController_T
#define struct_coder_internal_stickyStruct_20_AEBController_T

struct coder_internal_stickyStruct_20_AEBController_T
{
  coder_internal_stickyStruct_19_AEBController_T next;
};

#endif                 // struct_coder_internal_stickyStruct_20_AEBController_T

#ifndef struct_s_UwEJl0aQlkhk8Ldhhy1HQB_AEBController_T
#define struct_s_UwEJl0aQlkhk8Ldhhy1HQB_AEBController_T

struct s_UwEJl0aQlkhk8Ldhhy1HQB_AEBController_T
{
  anonymous_function_AEBController_T objfun;
  anonymous_function_AEBController_T nonlin;
  real_T f_1;
  coder::bounded_array<real_T, 60U, 1U> cIneq_1;
  real_T cEq_1[70];
  real_T f_2;
  coder::bounded_array<real_T, 60U, 1U> cIneq_2;
  real_T cEq_2[70];
  int32_T nVar;
  int32_T mIneq;
  int32_T mEq;
  int32_T numEvals;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T isEmptyNonlcon;
  boolean_T hasLB[75];
  boolean_T hasUB[75];
  boolean_T hasBounds;
  int32_T FiniteDifferenceType;
};

#endif                       // struct_s_UwEJl0aQlkhk8Ldhhy1HQB_AEBController_T

#ifndef SS_UINT64
#define SS_UINT64                      20
#endif

#ifndef SS_INT64
#define SS_INT64                       21
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_AEBController_T RT_MODEL_AEBController_T;

#endif                                 // AEBController_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
