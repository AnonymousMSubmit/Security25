#ifndef AEBController_private_h_
#define AEBController_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "AEBController_types.h"

extern const real_T rtCP_pooled_HgVWx9pIEzZu[7];
extern const real_T rtCP_pooled_KX6WUg9M2rgy[9];
extern const real_T rtCP_pooled_5pbXUAWZBbBJ[9];
extern const real_T rtCP_pooled_Vn3xsxvhzS10[9];
extern const real_T rtCP_pooled_Unvkl7y7urQf[49];
extern const real_T rtCP_pooled_PIt3sJ96SapJ[7];

#define rtCP_Constant_Value            rtCP_pooled_HgVWx9pIEzZu  // Expression: [m, Iz, Cf, Cr, lf,lr, tau]
                                                                 //  Referenced by: '<S7>/Constant'

#define rtCP_Constant_Value_k          rtCP_pooled_KX6WUg9M2rgy  // Expression: min(3,PredictionHorizon+1):(PredictionHorizon+1)
                                                                 //  Referenced by: '<S16>/Constant'

#define rtCP_Constant1_Value_j         rtCP_pooled_5pbXUAWZBbBJ  // Expression: 2:max(2,PredictionHorizon)
                                                                 //  Referenced by: '<S16>/Constant1'

#define rtCP_R1_Value                  rtCP_pooled_Vn3xsxvhzS10  // Expression: p.R{1}
                                                                 //  Referenced by: '<S9>/R1'

#define rtCP_Q_Value                   rtCP_pooled_Unvkl7y7urQf  // Expression: p.Q
                                                                 //  Referenced by: '<S9>/Q'

#define rtCP_DataStoreMemoryP_InitialValue rtCP_pooled_Unvkl7y7urQf// Expression: p.InitialCovariance
                                                                   //  Referenced by: '<S9>/DataStoreMemory - P'

#define rtCP_DataStoreMemoryx_InitialValue rtCP_pooled_PIt3sJ96SapJ// Expression: p.InitialState
                                                                   //  Referenced by: '<S9>/DataStoreMemory - x'

#endif                                 // AEBController_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
