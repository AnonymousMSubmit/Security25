#ifndef AEBSensorFusion_cap_host_h__
#define AEBSensorFusion_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

struct AEBSensorFusion_host_DataMapInfo_T {
  rtwCAPI_ModelMappingInfo mmi;
};

#ifdef __cplusplus

extern "C"
{

#endif

  void AEBSensorFusion_host_InitializeDataMapInfo
    (AEBSensorFusion_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}

#endif
#endif                                 // HOST_CAPI_BUILD
#endif                                 // AEBSensorFusion_cap_host_h__

// EOF: AEBSensorFusion_capi_host.h
