#ifndef AEBSensorFusion_types_h_
#define AEBSensorFusion_types_h_
#include "rtwtypes.h"
#include "coder_bounded_array.h"
#ifndef DEFINED_TYPEDEF_FOR_drivingCoordinateFrameType_
#define DEFINED_TYPEDEF_FOR_drivingCoordinateFrameType_

typedef uint8_T drivingCoordinateFrameType;

// enum drivingCoordinateFrameType
const drivingCoordinateFrameType Invalid = 0U;// Default value
const drivingCoordinateFrameType Rectangular = 1U;
const drivingCoordinateFrameType Spherical = 2U;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVisionDetectionsMeasurementParameters_
#define DEFINED_TYPEDEF_FOR_BusVisionDetectionsMeasurementParameters_

struct BusVisionDetectionsMeasurementParameters
{
  drivingCoordinateFrameType Frame;
  real_T OriginPosition[3];
  real_T Orientation[9];
  boolean_T HasVelocity;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVisionDetectionsObjectAttributes_
#define DEFINED_TYPEDEF_FOR_BusVisionDetectionsObjectAttributes_

struct BusVisionDetectionsObjectAttributes
{
  real_T TargetIndex;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVisionDetections_
#define DEFINED_TYPEDEF_FOR_BusVisionDetections_

struct BusVisionDetections
{
  real_T Time;
  real_T Measurement[6];
  real_T MeasurementNoise[36];
  real_T SensorIndex;
  real_T ObjectClassID;
  BusVisionDetectionsMeasurementParameters MeasurementParameters;
  BusVisionDetectionsObjectAttributes ObjectAttributes;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVision_
#define DEFINED_TYPEDEF_FOR_BusVision_

struct BusVision
{
  real_T NumDetections;
  boolean_T IsValidTime;
  BusVisionDetections Detections[20];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusRadarDetectionsMeasurementParameters_
#define DEFINED_TYPEDEF_FOR_BusRadarDetectionsMeasurementParameters_

struct BusRadarDetectionsMeasurementParameters
{
  drivingCoordinateFrameType Frame;
  real_T OriginPosition[3];
  real_T OriginVelocity[3];
  real_T Orientation[9];
  boolean_T IsParentToChild;
  boolean_T HasAzimuth;
  boolean_T HasElevation;
  boolean_T HasRange;
  boolean_T HasVelocity;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusRadarDetectionsObjectAttributes_
#define DEFINED_TYPEDEF_FOR_BusRadarDetectionsObjectAttributes_

struct BusRadarDetectionsObjectAttributes
{
  real_T TargetIndex;
  real_T SNR;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusRadarDetections_
#define DEFINED_TYPEDEF_FOR_BusRadarDetections_

struct BusRadarDetections
{
  real_T Time;
  real_T Measurement[6];
  real_T MeasurementNoise[36];
  real_T SensorIndex;
  real_T ObjectClassID;
  BusRadarDetectionsMeasurementParameters MeasurementParameters;
  BusRadarDetectionsObjectAttributes ObjectAttributes;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusRadar_
#define DEFINED_TYPEDEF_FOR_BusRadar_

struct BusRadar
{
  real_T NumDetections;
  boolean_T IsValidTime;
  BusRadarDetections Detections[50];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_trackLogicType_
#define DEFINED_TYPEDEF_FOR_trackLogicType_

typedef int32_T trackLogicType;

// enum trackLogicType
const trackLogicType trackLogicType_Invalid = 0;// Default value
const trackLogicType trackLogicType_History = 1;
const trackLogicType trackLogicType_Score = 2;
const trackLogicType trackLogicType_Integrated = 3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusMultiObjectTracker1Tracks_
#define DEFINED_TYPEDEF_FOR_BusMultiObjectTracker1Tracks_

struct BusMultiObjectTracker1Tracks
{
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  trackLogicType TrackLogic;
  boolean_T TrackLogicState[3];
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  BusRadarDetectionsObjectAttributes ObjectAttributes[2];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusMultiObjectTracker1_
#define DEFINED_TYPEDEF_FOR_BusMultiObjectTracker1_

struct BusMultiObjectTracker1
{
  real_T NumTracks;
  BusMultiObjectTracker1Tracks Tracks[20];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1DetectionsMeasurementParameters_
#define DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1DetectionsMeasurementParameters_

struct BusDetectionConcatenation1DetectionsMeasurementParameters
{
  drivingCoordinateFrameType Frame;
  real_T OriginPosition[3];
  real_T Orientation[9];
  boolean_T HasVelocity;
  real_T OriginVelocity[3];
  boolean_T IsParentToChild;
  boolean_T HasAzimuth;
  boolean_T HasElevation;
  boolean_T HasRange;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1Detections_
#define DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1Detections_

struct BusDetectionConcatenation1Detections
{
  real_T Time;
  real_T Measurement[6];
  real_T MeasurementNoise[36];
  real_T SensorIndex;
  real_T ObjectClassID;
  BusDetectionConcatenation1DetectionsMeasurementParameters
    MeasurementParameters;
  BusRadarDetectionsObjectAttributes ObjectAttributes;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1_
#define DEFINED_TYPEDEF_FOR_BusDetectionConcatenation1_

struct BusDetectionConcatenation1
{
  real_T NumDetections;
  boolean_T IsValidTime;
  BusDetectionConcatenation1Detections Detections[70];
};

#endif

#ifndef struct_cell_wrap_AEBSensorFusion_T
#define struct_cell_wrap_AEBSensorFusion_T

struct cell_wrap_AEBSensorFusion_T
{
  uint32_T f1[8];
};

#endif                                 // struct_cell_wrap_AEBSensorFusion_T

#ifndef struct_c_trackHistoryLogic_AEBSensorFusion_T
#define struct_c_trackHistoryLogic_AEBSensorFusion_T

struct c_trackHistoryLogic_AEBSensorFusion_T
{
  boolean_T pRecentHistory[50];
  boolean_T pIsFirstUpdate;
};

#endif                          // struct_c_trackHistoryLogic_AEBSensorFusion_T

#ifndef struct_c_matlabshared_tracking_internal_fusion_AssignmentCostCalcula_T
#define struct_c_matlabshared_tracking_internal_fusion_AssignmentCostCalcula_T

struct c_matlabshared_tracking_internal_fusion_AssignmentCostCalcula_T
{
  int32_T isInitialized;
  cell_wrap_AEBSensorFusion_T inputVarSize[4];
  real_T MaxAssignmentCost;
  real_T pMaxAssignmentCost;
};

#endif
      // struct_c_matlabshared_tracking_internal_fusion_AssignmentCostCalcula_T

#ifndef struct_c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T
#define struct_c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T

struct c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T AssignmentThreshold[2];
  real_T pCostOfNonAssignment;
};

#endif
      // struct_c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T

#ifndef struct_h_cell_wrap_AEBSensorFusion_T
#define struct_h_cell_wrap_AEBSensorFusion_T

struct h_cell_wrap_AEBSensorFusion_T
{
  coder::bounded_array<char_T, 11U, 2U> f1;
};

#endif                                 // struct_h_cell_wrap_AEBSensorFusion_T

#ifndef struct_n_cell_AEBSensorFusion_T
#define struct_n_cell_AEBSensorFusion_T

struct n_cell_AEBSensorFusion_T
{
  coder::bounded_array<char_T, 11U, 2U> f1;
  real_T f2[3];
  real_T f3[3];
  real_T f4[9];
  boolean_T f5;
  boolean_T f6;
  boolean_T f7;
  boolean_T f8;
  boolean_T f9;
};

#endif                                 // struct_n_cell_AEBSensorFusion_T

#ifndef struct_matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T
#define struct_matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T

struct matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T
{
  coder::bounded_array<int32_T, 90U, 1U> heap;
  coder::bounded_array<int32_T, 90U, 1U> indexToHeap;
  int32_T len;
};

#endif       // struct_matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T

#ifndef struct_c_trackingEKF_AEBSensorFusion_T
#define struct_c_trackingEKF_AEBSensorFusion_T

struct c_trackingEKF_AEBSensorFusion_T
{
  real_T pState[6];
  real_T pSqrtStateCovariance[36];
  real_T pSqrtStateCovarianceScalar;
  boolean_T pIsSetStateCovariance;
  real_T pSqrtProcessNoise[9];
  real_T pSqrtProcessNoiseScalar;
  boolean_T pIsSetProcessNoise;
  real_T pSqrtMeasurementNoise[36];
  real_T pSqrtMeasurementNoiseScalar;
  boolean_T pHasPrediction;
  boolean_T pIsValidStateTransitionFcn;
  boolean_T pIsValidMeasurementFcn;
  boolean_T pIsFirstCallPredict;
  boolean_T pIsFirstCallCorrect;
  real_T pJacobian[36];
  boolean_T pIsDistributionsSetup;
  boolean_T pIsInitialized;
  boolean_T IsLastJacobianInitialized;
  boolean_T pShouldWarn;
  boolean_T pWasRetrodicted;
};

#endif                                // struct_c_trackingEKF_AEBSensorFusion_T

#ifndef struct_b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
#define struct_b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T

struct b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
{
  uint32_T BranchID;
  uint32_T TrackID;
  real_T Time;
  c_trackHistoryLogic_AEBSensorFusion_T *TrackLogic;
  boolean_T IsConfirmed;
  real_T ObjectClassID;
  real_T UpdateTime;
  c_trackingEKF_AEBSensorFusion_T *Filter;
  uint32_T pAge;
  boolean_T pIsCoasted;
  c_trackingEKF_AEBSensorFusion_T *pDistanceFilter;
  BusRadarDetectionsObjectAttributes pObjectAttributes[2];
  boolean_T pUsedObjectAttributes[2];
};

#endif
      // struct_b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T

#ifndef struct_matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T
#define struct_matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T

struct matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T
{
  int32_T isInitialized;
  BusDetectionConcatenation1 pOutTemp;
};

#endif
      // struct_matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T

#ifndef struct_c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T
#define struct_c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T

struct c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T
{
  int32_T isInitialized;
  real_T pNumDetections;
  BusDetectionConcatenation1Detections pDetections[70];
  real_T pOriginatingSensor[70];
  boolean_T pIsOOSM[70];
};

#endif
      // struct_c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T

#ifndef struct_multiObjectTracker_AEBSensorFusion_T
#define struct_multiObjectTracker_AEBSensorFusion_T

struct multiObjectTracker_AEBSensorFusion_T
{
  boolean_T matlabCodegenIsDeleted;
  boolean_T tunablePropertyChanged[2];
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *pTracksList
    [20];
  int32_T pNumLiveTracks;
  uint32_T pTrackIDs[20];
  boolean_T pConfirmedTracks[20];
  BusDetectionConcatenation1Detections pSampleDetection;
  c_trackingEKF_AEBSensorFusion_T *pDistFilter;
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T
    *cDetectionManager;
  uint32_T pUsedSensors[2];
  real_T pLastTimeStamp;
  real_T AssignmentThreshold[2];
  boolean_T pWasDetectable[20];
  real_T pNumDetections;
  uint32_T pLastTrackID;
  real_T pCostOfNonAssignment;
  boolean_T pConfThreshChanged;
  boolean_T pDelThreshChanged;
  c_matlabshared_tracking_internal_fusion_AssignmentCostCalcula_T
    cCostCalculator;
  c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T *cAssigner;
  c_trackingEKF_AEBSensorFusion_T _pobj0[41];
  c_trackHistoryLogic_AEBSensorFusion_T _pobj1[20];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T _pobj2[20];
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T _pobj3;
  c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T _pobj4;
};

#endif                           // struct_multiObjectTracker_AEBSensorFusion_T

#ifndef struct_sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T
#define struct_sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T

struct sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T
{
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  real_T ObjectClassProbabilities;
  trackLogicType TrackLogic;
  boolean_T TrackLogicState[3];
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  BusRadarDetectionsObjectAttributes ObjectAttributes[2];
};

#endif                      // struct_sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T

#ifndef SS_UINT64
#define SS_UINT64                      34
#endif

#ifndef SS_INT64
#define SS_INT64                       35
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_AEBSensorFusion_T RT_MODEL_AEBSensorFusion_T;

#endif                                 // AEBSensorFusion_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
