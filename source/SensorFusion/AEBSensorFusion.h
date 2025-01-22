#ifndef AEBSensorFusion_h_
#define AEBSensorFusion_h_
#include "rtwtypes.h"
#include "AEBSensorFusion_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rtw_modelmap.h"
#include <cstring>
#include <stddef.h>

// Block signals and states (default storage) for model 'AEBSensorFusion'
struct DW_AEBSensorFusion_T {
  multiObjectTracker_AEBSensorFusion_T obj;// '<Root>/Multi-Object Tracker'
  BusDetectionConcatenation1 DetectionConcatenation;// '<Root>/Detection Concatenation' 
  matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T obj_c;// '<Root>/Detection Concatenation' 
  BusDetectionConcatenation1 r;
  BusDetectionConcatenation1Detections b_dets_data[70];
  BusVisionDetections rv[20];
  BusMultiObjectTracker1 TmpMLSysMemLayoutBufferAtMultiObjectTrackerOutport1;// '<Root>/Multi-Object Tracker' 
  BusMultiObjectTracker1Tracks tracks_data[20];
  c_trackingEKF_AEBSensorFusion_T lobj_2[5];
  real_T costMatrix_data[1400];
  real_T costMatrix_data_m[1400];
  real_T costMatrix_data_c[1400];
  real_T paddedCost_data[8100];
  real_T paddedCost_data_k[8100];
  boolean_T objisempty;                // '<Root>/Multi-Object Tracker'
  boolean_T objisempty_m;              // '<Root>/Detection Concatenation'
};

// Real-time Model Data Structure
struct tag_RTM_AEBSensorFusion_T {
  //
  //  DataMapInfo:
  //  The following substructure contains information regarding
  //  structures generated in the model's C API.

  struct DataMapInfo_T {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[1];
    int32_T* vardimsAddress[1];
    RTWLoggingFcnPtr loggingPtrs[1];
  };

  DataMapInfo_T DataMapInfo;
  RT_MODEL_AEBSensorFusion_T::DataMapInfo_T getDataMapInfo() const;
  void setDataMapInfo(RT_MODEL_AEBSensorFusion_T::DataMapInfo_T aDataMapInfo);
};

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  AEBSensorFusion_GetCAPIStaticMap(void);

// Class declaration for model AEBSensorFusion
class ACCWithSensorFusionModelClass
{
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // Initial conditions function
  void init();

  // model terminate function
  void terminate();

  // Real-Time Model get method
  RT_MODEL_AEBSensorFusion_T * getRTM();

  // member function to set up the C-API information
  void setupCAPIInfo(rtwCAPI_ModelMappingInfo *rt_ParentMMI, const char_T
                     *rt_ChildPath, int_T rt_ChildMMIIdx, int_T rt_CSTATEIdx);

  // model step function
  void step(const BusVision *rtu_Vision, const BusRadar *rtu_Radar, const real_T
            *rtu_SystemTime, BusMultiObjectTracker1 *rty_Tracks);

  // Constructor
  ACCWithSensorFusionModelClass();

  // Destructor
  ~ACCWithSensorFusionModelClass();

  // private data and function members
 private:
  // Block states
  DW_AEBSensorFusion_T AEBSensorFusion_DW;

  // private member function(s) for subsystem '<Root>/TmpModelReferenceSubsystem'
  void AEBSensorFusion_DetectionConcatenation_setupImpl
    (matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T *obj, const
     BusVisionDetections varargin_1_Detections[20]);
  void AEBSensorFusion_char(drivingCoordinateFrameType varargin_1, char_T
    y_data[], int32_T y_size[2]);
  boolean_T AEBSensorFusion_strcmp(const char_T a_data[], const int32_T a_size[2]);
  real_T AEBSensorFusion_norm(const real_T x[3]);
  real_T AEBSensorFusion_cosd(real_T x);
  void AEBSensorFusion_parseDetectionForInitFcn(const real_T
    detection_Measurement[6], const real_T detection_MeasurementNoise[36],
    drivingCoordinateFrameType detection_MeasurementParameters_Frame, const
    real_T detection_MeasurementParameters_OriginPosition[3], const real_T
    detection_MeasurementParameters_Orientation[9], boolean_T
    detection_MeasurementParameters_HasVelocity, const real_T
    detection_MeasurementParameters_OriginVelocity[3], boolean_T
    detection_MeasurementParameters_IsParentToChild, boolean_T
    detection_MeasurementParameters_HasElevation, real_T posMeas[3], real_T
    velMeas[3], real_T posCov[9], real_T velCov[9], boolean_T *invalidDet);
  void AEBSensorFusion_xzlascl(real_T cfrom, real_T cto, real_T A[36]);
  real_T AEBSensorFusion_xnrm2(int32_T n, const real_T x[36], int32_T ix0);
  void AEBSensorFusion_xzlarfg(int32_T n, real_T alpha1, real_T x[36], int32_T
    ix0, real_T *b_alpha1, real_T *tau);
  void AEBSensorFusion_xzgehrd(real_T a[36], int32_T ilo, int32_T ihi);
  void AEBSensorFusion_xdlanv2(real_T a, real_T b, real_T c, real_T d, real_T
    *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *b_a, real_T *b_b,
    real_T *b_c, real_T *b_d, real_T *cs, real_T *sn);
  real_T AEBSensorFusion_xnrm2_p(int32_T n, const real_T x[3]);
  void AEBSensorFusion_xdlahqr(int32_T ilo, int32_T ihi, real_T h[36], int32_T
    *info, real_T *b_z, real_T wr[6], real_T wi[6]);
  void AEBSensorFusion_xzlascl_i(real_T cfrom, real_T cto, int32_T m, real_T A[6],
    int32_T iA0);
  void AEBSensorFusion_eigStandard(const real_T A[36], creal_T V[6]);
  void AEBSensorFusion_insertionsort(real_T x[6], int32_T xstart, int32_T xend);
  void AEBSensorFusion_xzlascl_iv(real_T cfrom, real_T cto, int32_T m, real_T A
    [5], int32_T iA0);
  void AEBSensorFusion_xdlaev2(real_T a, real_T b, real_T c, real_T *rt1, real_T
    *rt2);
  int32_T AEBSensorFusion_xdsterf(real_T d[6], real_T e[5]);
  void AEBSensorFusion_eig(const real_T A[36], creal_T V[6]);
  int32_T AEBSensorFusion_xpotrf_ba(real_T A[36]);
  real_T AEBSensorFusion_xzlangeM(const real_T x[36]);
  real_T AEBSensorFusion_xdotc(int32_T n, const real_T x[36], int32_T ix0, const
    real_T y[36], int32_T iy0);
  void AEBSensorFusion_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[36],
    int32_T iy0);
  real_T AEBSensorFusion_xnrm2_po1(int32_T n, const real_T x[6], int32_T ix0);
  void AEBSensorFusion_xaxpy_g(int32_T n, real_T a, const real_T x[36], int32_T
    ix0, real_T y[6], int32_T iy0);
  void AEBSensorFusion_xaxpy_go(int32_T n, real_T a, const real_T x[6], int32_T
    ix0, real_T y[36], int32_T iy0);
  void AEBSensorFusion_xrotg(real_T a, real_T b, real_T *b_a, real_T *b_b,
    real_T *c, real_T *s);
  void AEBSensorFusion_xrot_ps(real_T x[36], int32_T ix0, int32_T iy0, real_T c,
    real_T s);
  void AEBSensorFusion_xswap_dj(real_T x[36], int32_T ix0, int32_T iy0);
  void AEBSensorFusion_svd(const real_T A[36], real_T U[36], real_T s[6], real_T
    V[36]);
  void AEBSensorFusion_cholPSD(const real_T A[36], real_T b_value[36]);
  c_trackingEKF_AEBSensorFusion_T *AEBSensorFusion_initcvekf(const real_T
    Detection_Measurement[6], const real_T Detection_MeasurementNoise[36],
    drivingCoordinateFrameType Detection_MeasurementParameters_Frame, const
    real_T Detection_MeasurementParameters_OriginPosition[3], const real_T
    Detection_MeasurementParameters_Orientation[9], boolean_T
    Detection_MeasurementParameters_HasVelocity, const real_T
    Detection_MeasurementParameters_OriginVelocity[3], boolean_T
    Detection_MeasurementParameters_IsParentToChild, boolean_T
    Detection_MeasurementParameters_HasElevation,
    c_trackingEKF_AEBSensorFusion_T *iobj_0);
  c_trackingEKF_AEBSensorFusion_T *AEBSensorFusion_ExtendedKalmanFilter_clone(
    const c_trackingEKF_AEBSensorFusion_T *EKF, c_trackingEKF_AEBSensorFusion_T *
    iobj_0);
  c_trackingEKF_AEBSensorFusion_T *AEBSensorFusion_trackingEKF_clone
    (c_trackingEKF_AEBSensorFusion_T *obj, c_trackingEKF_AEBSensorFusion_T
     *iobj_0);
  void AEBSensorFusion_ObjectTrack_nullify
    (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track);
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
    *AEBSensorFusion_TrackManager_createSampleTrack(const
    multiObjectTracker_AEBSensorFusion_T *obj, c_trackingEKF_AEBSensorFusion_T
    *iobj_0, c_trackHistoryLogic_AEBSensorFusion_T *iobj_1,
    b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *iobj_2);
  int32_T AEBSensorFusion_xpotrf_ban(real_T A[9]);
  real_T AEBSensorFusion_xzlangeM_k(const real_T x[9]);
  void AEBSensorFusion_xzlascl_ivk(real_T cfrom, real_T cto, real_T A[9]);
  real_T AEBSensorFusion_xnrm2_po1g(int32_T n, const real_T x[9], int32_T ix0);
  real_T AEBSensorFusion_xdotc_o(int32_T n, const real_T x[9], int32_T ix0,
    const real_T y[9], int32_T iy0);
  void AEBSensorFusion_xaxpy_gop(int32_T n, real_T a, int32_T ix0, real_T y[9],
    int32_T iy0);
  real_T AEBSensorFusion_xnrm2_po1g3(const real_T x[3], int32_T ix0);
  void AEBSensorFusion_xaxpy_goph(int32_T n, real_T a, const real_T x[9],
    int32_T ix0, real_T y[3], int32_T iy0);
  void AEBSensorFusion_xaxpy_gopha(int32_T n, real_T a, const real_T x[3],
    int32_T ix0, real_T y[9], int32_T iy0);
  void AEBSensorFusion_xzlascl_ivky(real_T cfrom, real_T cto, real_T A[3]);
  void AEBSensorFusion_xrot_psj(real_T x[9], int32_T ix0, int32_T iy0, real_T c,
    real_T s);
  void AEBSensorFusion_xswap_dj5(real_T x[9], int32_T ix0, int32_T iy0);
  void AEBSensorFusion_svd_a(const real_T A[9], real_T U[9], real_T s[3], real_T
    V[9]);
  void AEBSensorFusion_ExtendedKalmanFilter_set_pProcessNoise
    (c_trackingEKF_AEBSensorFusion_T *obj, const real_T b_value[9]);
  void AEBSensorFusion_trackingEKF_sync(c_trackingEKF_AEBSensorFusion_T *EKF,
    c_trackingEKF_AEBSensorFusion_T *EKF2);
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
    *AEBSensorFusion_ObjectTrack_copy
    (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track,
     b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *iobj_0,
     c_trackHistoryLogic_AEBSensorFusion_T *iobj_1,
     c_trackingEKF_AEBSensorFusion_T *iobj_2);
  void AEBSensorFusion_ObjectTrack_trackToStruct
    (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track,
     sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T *trackStruct);
  void AEBSensorFusion_SystemCore_setup(multiObjectTracker_AEBSensorFusion_T
    *obj, real_T varargin_1_NumDetections, const
    BusDetectionConcatenation1Detections varargin_1_Detections[70]);
  void AEBSensorFusion_GNNTracker_resetImpl(multiObjectTracker_AEBSensorFusion_T
    *obj);
  void AEBSensorFusion_merge(int32_T idx_data[], uint32_T x_data[], int32_T
    offset, int32_T np, int32_T nq, int32_T iwork_data[], uint32_T xwork_data[]);
  void AEBSensorFusion_sort(const uint32_T x_data[], const int32_T x_size[2],
    uint32_T b_x_data[], int32_T b_x_size[2]);
  void AEBSensorFusion_sort_m(const uint32_T x[2], uint32_T b_x[2]);
  void AEBSensorFusion_do_vectors(const uint32_T a_data[], const int32_T a_size
    [2], const uint32_T b[2], uint32_T c_data[], int32_T c_size[2], int32_T
    ia_data[], int32_T ia_size[1], int32_T ib_size[1]);
  void AEBSensorFusion_TrackManager_getLiveTrackIndices(const
    multiObjectTracker_AEBSensorFusion_T *obj, int32_T indices_data[], int32_T
    indices_size[2]);
  void AEBSensorFusion_unique_vector(const real_T a_data[], const int32_T
    a_size[2], real_T b_data[], int32_T b_size[2]);
  void AEBSensorFusion_binary_expand_op_1(boolean_T in1_data[], int32_T
    in1_size[2], const real_T in2_data[], const int32_T in2_size[2], real_T in3,
    const boolean_T in4_data[], const int32_T in4_size[2]);
  void AEBSensorFusion_GNNTracker_selectDetections(const real_T origSen_data[],
    const int32_T origSen_size[2], real_T sensorID, const boolean_T
    insequence_data[], const int32_T insequence_size[2], int32_T idx_data[],
    int32_T idx_size[2]);
  void AEBSensorFusion_merge_e(int32_T idx_data[], real_T x_data[], int32_T
    offset, int32_T np, int32_T nq, int32_T iwork_data[], real_T xwork_data[]);
  void AEBSensorFusion_sort_mh(const real_T x_data[], const int32_T x_size[2],
    real_T b_x_data[], int32_T b_x_size[2], int32_T idx_data[], int32_T
    idx_size[2]);
  void AEBSensorFusion_merge_ef(int32_T idx_data[], real_T x_data[], int32_T
    offset, int32_T np, int32_T nq, int32_T iwork_data[], real_T xwork_data[]);
  void AEBSensorFusion_sort_mhq(real_T x_data[], const int32_T x_size[1],
    int32_T idx_data[], int32_T idx_size[1]);
  void AEBSensorFusion_measmodelsvalidateoptionalstruct
    (drivingCoordinateFrameType candidateStructs_Frame, const real_T
     candidateStructs_OriginPosition[3], const real_T
     candidateStructs_Orientation[9], boolean_T candidateStructs_HasVelocity,
     const real_T candidateStructs_OriginVelocity[3], boolean_T
     candidateStructs_IsParentToChild, boolean_T candidateStructs_HasAzimuth,
     boolean_T candidateStructs_HasElevation, boolean_T
     candidateStructs_HasRange, n_cell_AEBSensorFusion_T *argsinCell);
  void AEBSensorFusion_get_match(const char_T str_data[], const int32_T
    str_size[2], char_T match_data[], int32_T match_size[2], int32_T *nmatched);
  real_T AEBSensorFusion_radialspeed(const real_T tgtpos[3], const real_T
    tgtvel[3], const real_T refpos[3], const real_T refvel[3]);
  void AEBSensorFusion_cvmeas(const real_T state[6], drivingCoordinateFrameType
    varargin_1_Frame, const real_T varargin_1_OriginPosition[3], const real_T
    varargin_1_Orientation[9], boolean_T varargin_1_HasVelocity, const real_T
    varargin_1_OriginVelocity[3], boolean_T varargin_1_IsParentToChild,
    boolean_T varargin_1_HasAzimuth, boolean_T varargin_1_HasElevation,
    boolean_T varargin_1_HasRange, real_T measurement_data[], int32_T
    measurement_size[1]);
  void AEBSensorFusion_binary_expand_op(real_T in1[6], const
    BusDetectionConcatenation1Detections *in2);
  void AEBSensorFusion_xzgetrf(real_T A[36], int32_T ipiv[6], int32_T *info);
  void AEBSensorFusion_mrdiv_b(real_T A[6], const real_T B[36]);
  real_T AEBSensorFusion_xnrm2_po1g3p(int32_T n, const real_T x[54], int32_T ix0);
  void AEBSensorFusion_EKFPredictorNonAdditive_predict_k(const real_T Qs[9],
    real_T x[6], const real_T S[36], real_T varargin_1, real_T b_S[36], real_T
    dFdx[36]);
  void AEBSensorFusion_predictTrackFilter(c_trackingEKF_AEBSensorFusion_T
    *filter, real_T dt);
  void AEBSensorFusion_cvmeasjac(const real_T state[6],
    drivingCoordinateFrameType varargin_1_Frame, const real_T
    varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
    boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
    boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
    boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
    jacobian_data[], int32_T jacobian_size[2]);
  void AEBSensorFusion_cvmeas_h(const real_T state[6],
    drivingCoordinateFrameType varargin_1_Frame, const real_T
    varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
    boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
    boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
    boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
    measurement_data[], int32_T measurement_size[1], real_T bounds_data[],
    int32_T bounds_size[2]);
  void AEBSensorFusion_binary_expand_op_13(real_T in1[36], const real_T
    in2_data[], const int32_T in2_size[2], const real_T in3[36]);
  void AEBSensorFusion_binary_expand_op_12(real_T in1[6], const real_T in2[6],
    const real_T in3_data[], const int32_T in3_size[1]);
  void AEBSensorFusion_all(const boolean_T x_data[], const int32_T x_size[2],
    boolean_T y_data[], int32_T y_size[1]);
  boolean_T AEBSensorFusion_any(const boolean_T x_data[], const int32_T x_size[1]);
  void AEBSensorFusion_plus(real_T in1[6], const real_T in2_data[], const
    int32_T in2_size[1]);
  void AEBSensorFusion_binary_expand_op_2(real_T in1[6], const real_T in2_data[],
    const int32_T in2_size[2], const real_T in3_data[], const int32_T in3_size[1]);
  void AEBSensorFusion_minus(real_T in1_data[], int32_T in1_size[1], const
    real_T in2_data[], const int32_T in2_size[1], const real_T in3_data[], const
    int32_T in3_size[1]);
  real_T AEBSensorFusion_normalizedDistance(const real_T z_data[], const int32_T
    z_size[1], const real_T mu_data[], const int32_T mu_size[1], const real_T
    sigma[36]);
  void AEBSensorFusion_binary_expand_op_10(real_T in1[6], const real_T in2_data[],
    const int32_T in2_size[2]);
  void AEBSensorFusion_binary_expand_op_9(boolean_T in1[6], const boolean_T
    in2_data[], const int32_T in2_size[1], const real_T in3[6]);
  void AEBSensorFusion_binary_expand_op_6(real_T in1_data[], int32_T in1_size[1],
    const real_T in2_data[], const int32_T in2_size[2]);
  void AEBSensorFusion_binary_expand_op_5(boolean_T in1_data[], int32_T
    in1_size[1], const boolean_T in2_data[], const int32_T in2_size[1], const
    real_T in3_data[], const int32_T in3_size[1]);
  real_T AEBSensorFusion_mod(real_T x, real_T y);
  void AEBSensorFusion_binary_expand_op_8(real_T in1[6], const int32_T in2_data[],
    const int32_T in2_size[1], const real_T in3_data[], const int32_T in3_size[1],
    const real_T in4_data[]);
  void AEBSensorFusion_expand_mod(const real_T a_data[], const int32_T a_size[2],
    const real_T b_data[], const int32_T b_size[1], real_T c_data[], int32_T
    c_size[2]);
  void AEBSensorFusion_binary_expand_op_3(real_T in1_data[], int32_T in1_size[2],
    const real_T in2_data[], const int32_T in2_size[2], const int32_T in3_data[],
    const int32_T in3_size[1], const real_T in4_data[], const int32_T in4_size[2]);
  void AEBSensorFusion_binary_expand_op_4(real_T in1_data[], const int32_T
    in1_size[2], const int32_T in2_data[], const int32_T in2_size[1], const
    real_T in3_data[], const int32_T in3_size[2]);
  void AEBSensorFusion_ObjectTrack_calcCostOneDetection
    (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track,
     const real_T detection_Measurement[6], drivingCoordinateFrameType
     detection_MeasurementParameters_Frame, const real_T
     detection_MeasurementParameters_OriginPosition[3], const real_T
     detection_MeasurementParameters_Orientation[9], boolean_T
     detection_MeasurementParameters_HasVelocity, const real_T
     detection_MeasurementParameters_OriginVelocity[3], boolean_T
     detection_MeasurementParameters_IsParentToChild, boolean_T
     detection_MeasurementParameters_HasAzimuth, boolean_T
     detection_MeasurementParameters_HasElevation, boolean_T
     detection_MeasurementParameters_HasRange, real_T costValue_data[], int32_T
     costValue_size[2]);
  void AEBSensorFusion_GNNTracker_calcCostMatrixAllSensors
    (multiObjectTracker_AEBSensorFusion_T *obj, const
     BusDetectionConcatenation1Detections dets_data[], const int32_T dets_size[1],
     real_T overallCostMatrix_data[], int32_T overallCostMatrix_size[2]);
  void AEBSensorFusion_binary_expand_op_14(boolean_T in1_data[], int32_T
    in1_size[1], real_T in2, real_T in3);
  void AEBSensorFusion_minPriorityQueue_percUp
    (matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T *obj, int32_T i,
     const real_T dist_data[]);
  void AEBSensorFusion_perfectMatching(const real_T A_data[], const int32_T
    A_size[2], int32_T m1_data[], int32_T m1_size[1], int32_T m2_data[], int32_T
    m2_size[1]);
  void AEBSensorFusion_SystemCore_step
    (c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T *obj, const
     real_T varargin_1_data[], const int32_T varargin_1_size[2], uint32_T
     varargout_1_data[], int32_T varargout_1_size[2]);
  void AEBSensorFusion_binary_expand_op_15(boolean_T in1_data[], int32_T
    in1_size[1], const boolean_T in2[70], real_T in3, real_T in4, const
    boolean_T in5_data[], const int32_T in5_size[2]);
  void AEBSensorFusion_GNNTracker_associate(multiObjectTracker_AEBSensorFusion_T
    *obj, const real_T costMatrix_data[], const int32_T costMatrix_size[2],
    uint32_T assigned_data[], int32_T assigned_size[2], uint32_T
    unassignedTrs_data[], int32_T unassignedTrs_size[1], uint32_T
    unassignedDets_data[], int32_T unassignedDets_size[1]);
  void AEBSensorFusion_eml_find(const boolean_T x[2], int32_T i_data[], int32_T
    i_size[2]);
  boolean_T AEBSensorFusion_TrackManager_initiateTrack
    (multiObjectTracker_AEBSensorFusion_T *obj, uint32_T newTrackID, real_T
     Det_Time, const real_T Det_Measurement[6], const real_T
     Det_MeasurementNoise[36], real_T Det_SensorIndex, real_T Det_ObjectClassID,
     drivingCoordinateFrameType Det_MeasurementParameters_Frame, const real_T
     Det_MeasurementParameters_OriginPosition[3], const real_T
     Det_MeasurementParameters_Orientation[9], boolean_T
     Det_MeasurementParameters_HasVelocity, const real_T
     Det_MeasurementParameters_OriginVelocity[3], boolean_T
     Det_MeasurementParameters_IsParentToChild, boolean_T
     Det_MeasurementParameters_HasElevation, const
     BusRadarDetectionsObjectAttributes Det_ObjectAttributes);
  real_T AEBSensorFusion_xnrm2_po1g3pk(int32_T n, const real_T x_data[], int32_T
    ix0);
  void AEBSensorFusion_qrFactor(const real_T A_data[], const int32_T A_size[2],
    const real_T S[36], const real_T Ns[36], real_T b_S[36]);
  void AEBSen_EKFCorrectorAdditive_getMeasurementJacobianAndCovariance(const
    real_T Rs[36], const real_T x[6], const real_T S[36],
    drivingCoordinateFrameType varargin_1_Frame, const real_T
    varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
    boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
    boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
    boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
    zEstimated_data[], int32_T zEstimated_size[1], real_T Pxy_data[], int32_T
    Pxy_size[2], real_T Sy[36], real_T dHdx_data[], int32_T dHdx_size[2], real_T
    wrapping_data[], int32_T wrapping_size[2]);
  void AEBSensorFusion_wrapResidual(real_T residual[6], real_T bounds_data[],
    const int32_T bounds_size[2]);
  real_T AEBSensorFusion_trackingEKF_likelihood(c_trackingEKF_AEBSensorFusion_T *
    EKF, const real_T z[6], const
    BusDetectionConcatenation1DetectionsMeasurementParameters *varargin_1);
  void AEBSensorFusion_trisolve(const real_T A[36], real_T B[36]);
  void AEBSensorFusion_trisolve_l(const real_T A[36], real_T B[36]);
  real_T AEBSensorFusion_xnrm2_po1g3pki(int32_T n, const real_T x[72], int32_T
    ix0);
  void AEBSensorFusion_qrFactor_g(const real_T A[36], const real_T S[36], const
    real_T Ns[36], real_T b_S[36]);
  void AEBSensorFusion_trackingEKF_correct(c_trackingEKF_AEBSensorFusion_T
    *filter, const real_T varargin_1[6], drivingCoordinateFrameType
    varargin_2_Frame, const real_T varargin_2_OriginPosition[3], const real_T
    varargin_2_Orientation[9], boolean_T varargin_2_HasVelocity, const real_T
    varargin_2_OriginVelocity[3], boolean_T varargin_2_IsParentToChild,
    boolean_T varargin_2_HasAzimuth, boolean_T varargin_2_HasElevation,
    boolean_T varargin_2_HasRange);
  void AEBSensorFusion_GNNTracker_initiateTracks
    (multiObjectTracker_AEBSensorFusion_T *obj, uint32_T OverallUnassigned_data[],
     int32_T OverallUnassigned_size[1], const
     BusDetectionConcatenation1Detections dets_data[], const int32_T dets_size[1],
     uint32_T initTrIDs_data[], int32_T initTrIDs_size[2]);
  void AEBSensorFusion_GNNTracker_updateAssignedTracks
    (multiObjectTracker_AEBSensorFusion_T *obj, const uint32_T
     OverallAssignments_data[], const int32_T OverallAssignments_size[2], const
     BusDetectionConcatenation1Detections dets_data[]);
  boolean_T AEBSensorFusion_ObjectTrack_checkDeletion
    (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track);
  void AEBSensorFusion_GNNTracker_coreAlgorithm
    (multiObjectTracker_AEBSensorFusion_T *obj, const
     BusDetectionConcatenation1Detections dets_data[], const int32_T dets_size[1],
     real_T b_time);

  // Real-Time Model
  RT_MODEL_AEBSensorFusion_T AEBSensorFusion_M;
};

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
//  '<Root>' : 'AEBSensorFusion'

#endif                                 // AEBSensorFusion_h_

//
// File trailer for generated code.
//
// [EOF]
//
