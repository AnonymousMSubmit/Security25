#ifndef AEBDecisionLogic_cap_host_h__
#define AEBDecisionLogic_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

struct AEBDecisionLogic_host_DataMapInfo_T {
  rtwCAPI_ModelMappingInfo mmi;
};

#ifdef __cplusplus

extern "C"
{

#endif

  void AEBDecisionLogic_host_InitializeDataMapInfo
    (AEBDecisionLogic_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}

#endif
#endif                                 // HOST_CAPI_BUILD
#endif                                 // AEBDecisionLogic_cap_host_h__

// EOF: AEBDecisionLogic_capi_host.h
