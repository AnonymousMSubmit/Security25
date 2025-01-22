#ifndef AEBDecisionLogic_types_h_
#define AEBDecisionLogic_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_trackLogicType_
#define DEFINED_TYPEDEF_FOR_trackLogicType_

typedef int32_T trackLogicType;

// enum trackLogicType
const trackLogicType trackLogicType_Invalid = 0;// Default value
const trackLogicType trackLogicType_History = 1;
const trackLogicType trackLogicType_Score = 2;
const trackLogicType trackLogicType_Integrated = 3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusRadarDetectionsObjectAttributes_
#define DEFINED_TYPEDEF_FOR_BusRadarDetectionsObjectAttributes_

struct BusRadarDetectionsObjectAttributes
{
  real_T TargetIndex;
  real_T SNR;
};

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

#ifndef DEFINED_TYPEDEF_FOR_BusEgoRefPath_
#define DEFINED_TYPEDEF_FOR_BusEgoRefPath_

struct BusEgoRefPath
{
  real_T time[450];
  real_T x[450];
  real_T y[450];
  real_T theta[450];
  real_T kappa[450];
  real_T speed[450];
  real_T arcLength[450];
  real_T numPoints;
};

#endif

#ifndef struct_HelperCalculateReferencePose_AEBDecisionLogic_T
#define struct_HelperCalculateReferencePose_AEBDecisionLogic_T

struct HelperCalculateReferencePose_AEBDecisionLogic_T
{
  int32_T isInitialized;
  real_T CurrentIndex;
  BusEgoRefPath ReferencePath;
  real_T NumWaypoints;
};

#endif                // struct_HelperCalculateReferencePose_AEBDecisionLogic_T

#ifndef SS_UINT64
#define SS_UINT64                      24
#endif

#ifndef SS_INT64
#define SS_INT64                       25
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_AEBDecisionLogic_T RT_MODEL_AEBDecisionLogic_T;

#endif                                 // AEBDecisionLogic_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
