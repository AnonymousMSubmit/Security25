#ifndef AEBDecisionLogic_h_
#define AEBDecisionLogic_h_
#include "rtwtypes.h"
#include "AEBDecisionLogic_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rtGetNaN.h"
#include "rtw_modelmap.h"
#include <cstring>
#include <stddef.h>

// Block signals and states (default storage) for model 'AEBDecisionLogic'
struct DW_AEBDecisionLogic_T {
  HelperCalculateReferencePose_AEBDecisionLogic_T obj;// '<Root>/Ego Reference Path Generator' 
  real_T lateral_deviation;            // '<Root>/Ego Reference Path Generator'
  real_T x;                            // '<Root>/Find Lead Car'
  boolean_T objisempty;                // '<Root>/Ego Reference Path Generator'
};

// Real-time Model Data Structure
struct tag_RTM_AEBDecisionLogic_T {
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
  RT_MODEL_AEBDecisionLogic_T::DataMapInfo_T getDataMapInfo() const;
  void setDataMapInfo(RT_MODEL_AEBDecisionLogic_T::DataMapInfo_T aDataMapInfo);
};

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  AEBDecisionLogic_GetCAPIStaticMap(void);

// Class declaration for model AEBDecisionLogic
class ACCWithSensorFusionModelClass
{
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // Initial conditions function
  void init();

  // Real-Time Model get method
  RT_MODEL_AEBDecisionLogic_T * getRTM();

  // member function to set up the C-API information
  void setupCAPIInfo(rtwCAPI_ModelMappingInfo *rt_ParentMMI, const char_T
                     *rt_ChildPath, int_T rt_ChildMMIIdx, int_T rt_CSTATEIdx);

  // model step function
  void step(const BusMultiObjectTracker1 *rtu_Tracks, const real_T
            rtu_Ego_Position[3], const real_T rtu_Ego_Velocity[3], const real_T *
            rtu_Ego_Yaw, const real_T rtu_ReferencePath_time[450], const real_T
            rtu_ReferencePath_x[450], const real_T rtu_ReferencePath_y[450],
            const real_T rtu_ReferencePath_theta[450], const real_T
            rtu_ReferencePath_kappa[450], const real_T rtu_ReferencePath_speed
            [450], const real_T rtu_ReferencePath_arcLength[450], const real_T
            *rtu_ReferencePath_numPoints, real_T *rty_RelativeDistance, real_T
            *rty_RelativeVelocity, real_T rty_CurvatureSequence[10], real_T
            *rty_LateralDeviation, real_T *rty_RelativeYawAngle, boolean_T
            *rty_GoalReached, real_T *rty_MIOTrack);

  // Block states
  DW_AEBDecisionLogic_T AEBDecisionLogic_DW;

  // Constructor
  ACCWithSensorFusionModelClass();

  // Destructor
  ~ACCWithSensorFusionModelClass();

  // private data and function members
 private:
  // private member function(s) for subsystem '<Root>/TmpModelReferenceSubsystem'
  real_T AEBDecisionLogic_norm(const real_T x[2]);
  real_T AEBDecisionLogic_xnrm2(const real_T x[2]);
  void AEBDecisionLogic_interp1(const real_T varargin_1_data[], const int32_T
    *varargin_1_size, const real_T varargin_2_data[], const int32_T
    *varargin_2_size, const real_T varargin_3[10], real_T Vq[10]);
  real_T AEBDecisionLogic_mod(real_T x);

  // Real-Time Model
  RT_MODEL_AEBDecisionLogic_T AEBDecisionLogic_M;
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
//  '<Root>' : 'AEBDecisionLogic'
//  '<S1>'   : 'AEBDecisionLogic/Find Lead Car'

#endif                                 // AEBDecisionLogic_h_

//
// File trailer for generated code.
//
// [EOF]
//
