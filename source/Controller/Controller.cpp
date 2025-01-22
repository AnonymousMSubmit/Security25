#include "AEBController.h"
#include "rtwtypes.h"
#include "AEBController_types.h"
#include <cstring>
#include <cmath>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "getXUe_eNtkTHmx.h"
#include "rt_hypotd_snf.h"
#include "helperNLMPCStateJacFcn_3EmkNgDE.h"
#include "all_WqXfAvPF.h"
#include "helperNLMPCStateFcn_pC1vAxkb.h"
#include "checkVectorNonFinite_KkAetmxy.h"
#include "xrotg_fUOOztoj.h"
#include "AEBController_capi.h"
#include "AEBController_private.h"
#include "div_nde_s32_floor.h"

// Named constants for Chart: '<S1>/AEBLogic'
const uint8_T AEBController_IN_Default = 1U;
const uint8_T AEBController_IN_FCW = 2U;
const uint8_T AEBController_IN_Full_Braking = 3U;
const uint8_T AEBController_IN_Partial_Braking1 = 4U;
const uint8_T AEBController_IN_Partial_Braking2 = 5U;
real_T ACCWithSensorFusionModelClass::AEBController_xnrm2(int32_T n, const
  real_T x[30], int32_T ix0)
{
  real_T y;

  // Start for MATLABSystem: '<S10>/MATLAB System'
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<S10>/MATLAB System'
  return y;
}

void ACCWithSensorFusionModelClass::AEBController_qr(const real_T A[30], real_T
  Q[30], real_T R[9])
{
  real_T b_tau[3];
  real_T work[3];
  real_T b_atmp;
  real_T beta1;
  real_T c_A;
  int32_T exitg1;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T ii;
  int32_T itau;
  int32_T ix0;
  int32_T jA;
  int32_T knt;
  int32_T lastv;
  boolean_T exitg2;

  // Start for MATLABSystem: '<S10>/MATLAB System'
  b_tau[0] = 0.0;
  b_tau[1] = 0.0;
  b_tau[2] = 0.0;
  std::memcpy(&Q[0], &A[0], 30U * sizeof(real_T));
  work[0] = 0.0;
  work[1] = 0.0;
  work[2] = 0.0;
  for (itau = 0; itau < 3; itau++) {
    ii = itau * 10 + itau;
    ix0 = ii + 2;
    b_atmp = Q[ii];
    b_tau[itau] = 0.0;
    beta1 = AEBController_xnrm2(9 - itau, Q, ii + 2);
    if (beta1 != 0.0) {
      c_A = Q[ii];
      beta1 = rt_hypotd_snf(c_A, beta1);
      if (c_A >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - itau;
          for (lastv = ix0; lastv <= i + 10; lastv++) {
            Q[lastv - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

        beta1 = rt_hypotd_snf(b_atmp, AEBController_xnrm2(9 - itau, Q, ii + 2));
        if (b_atmp >= 0.0) {
          beta1 = -beta1;
        }

        b_tau[itau] = (beta1 - b_atmp) / beta1;
        b_atmp = 1.0 / (b_atmp - beta1);
        for (lastv = ix0; lastv <= i + 10; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        for (lastv = 0; lastv <= knt; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        b_atmp = beta1;
      } else {
        b_tau[itau] = (beta1 - c_A) / beta1;
        b_atmp = 1.0 / (c_A - beta1);
        i = ii - itau;
        for (lastv = ix0; lastv <= i + 10; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        b_atmp = beta1;
      }
    }

    Q[ii] = b_atmp;
    if (itau + 1 < 3) {
      Q[ii] = 1.0;
      ix0 = ii + 11;
      if (b_tau[itau] != 0.0) {
        lastv = 10 - itau;
        i = ii - itau;
        while ((lastv > 0) && (Q[i + 9] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 1 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 10 + ii;
          ia = i + 11;
          do {
            exitg1 = 0;
            if (ia <= (i + lastv) + 10) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (10 * knt + ii) + 11;
          for (iac = ix0; iac <= i; iac += 10) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[(ii + ia) - iac] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 11, 10);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 11;
              ix0 = (lastv + jA) + 10;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 11] * beta1;
              }
            }

            jA += 10;
          }
        }
      }

      Q[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 3; ii++) {
    for (itau = 0; itau <= ii; itau++) {
      // Start for MATLABSystem: '<S10>/MATLAB System'
      R[itau + 3 * ii] = Q[10 * ii + itau];
    }

    for (itau = ii + 2; itau < 4; itau++) {
      R[(itau + 3 * ii) - 1] = 0.0;
    }

    // Start for MATLABSystem: '<S10>/MATLAB System'
    work[ii] = 0.0;
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  for (itau = 2; itau >= 0; itau--) {
    ii = (itau * 10 + itau) + 10;
    if (itau + 1 < 3) {
      Q[ii - 10] = 1.0;
      ix0 = ii + 1;
      if (b_tau[itau] != 0.0) {
        lastv = 10 - itau;
        i = (ii - itau) - 1;
        while ((lastv > 0) && (Q[i] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 1 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 10 + ii;
          ia = i + 1;
          do {
            exitg1 = 0;
            if (ia <= i + lastv) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (10 * knt + ii) + 1;
          for (iac = ix0; iac <= i; iac += 10) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[((ii + ia) - iac) - 10] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 1, 10);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 1;
              ix0 = lastv + jA;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 11] * beta1;
              }
            }

            jA += 10;
          }
        }
      }
    }

    i = ii - itau;
    for (lastv = ii - 8; lastv <= i; lastv++) {
      Q[lastv - 1] *= -b_tau[itau];
    }

    Q[ii - 10] = 1.0 - b_tau[itau];
    for (i = 0; i < itau; i++) {
      Q[(ii - i) - 11] = 0.0;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBController_trisolve(const real_T A[9],
  real_T B[21])
{
  // Start for MATLABSystem: '<S10>/MATLAB System'
  for (int32_T b_j = 0; b_j < 7; b_j++) {
    int32_T jBcol;
    jBcol = 3 * b_j - 1;
    for (int32_T b_k = 0; b_k < 3; b_k++) {
      real_T B_0;
      int32_T B_tmp;
      int32_T k;
      int32_T kAcol;
      k = b_k + 1;
      kAcol = b_k * 3 - 1;
      B_tmp = (b_k + jBcol) + 1;
      B_0 = B[B_tmp];
      if (B_0 != 0.0) {
        B[B_tmp] = B_0 / A[(b_k + kAcol) + 1];
        for (int32_T i = k + 1; i < 4; i++) {
          int32_T tmp;
          tmp = i + jBcol;
          B[tmp] -= A[i + kAcol] * B[B_tmp];
        }
      }
    }
  }

  // End of Start for MATLABSystem: '<S10>/MATLAB System'
}

void ACCWithSensorFusionModelClass::AEBController_trisolve_l(const real_T A[9],
  real_T B[21])
{
  // Start for MATLABSystem: '<S10>/MATLAB System'
  for (int32_T b_j = 0; b_j < 7; b_j++) {
    int32_T jBcol;
    jBcol = 3 * b_j;
    for (int32_T k = 2; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = 3 * k;
      tmp_0 = k + jBcol;
      tmp = B[tmp_0];
      if (tmp != 0.0) {
        B[tmp_0] = tmp / A[k + kAcol];
        for (int32_T b_i = 0; b_i < k; b_i++) {
          int32_T tmp_1;
          tmp_1 = b_i + jBcol;
          B[tmp_1] -= A[b_i + kAcol] * B[tmp_0];
        }
      }
    }
  }

  // End of Start for MATLABSystem: '<S10>/MATLAB System'
}

real_T ACCWithSensorFusionModelClass::AEBController_xnrm2_p(int32_T n, const
  real_T x[70], int32_T ix0)
{
  real_T y;

  // Start for MATLABSystem: '<S10>/MATLAB System'
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<S10>/MATLAB System'
  return y;
}

void ACCWithSensorFusionModelClass::AEBController_qr_c(const real_T A[70],
  real_T Q[70], real_T R[49])
{
  real_T b_tau[7];
  real_T work[7];
  real_T b_atmp;
  real_T beta1;
  real_T c_A;
  int32_T exitg1;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T ii;
  int32_T itau;
  int32_T ix0;
  int32_T jA;
  int32_T knt;
  int32_T lastv;
  boolean_T exitg2;
  for (i = 0; i < 7; i++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    b_tau[i] = 0.0;
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  std::memcpy(&Q[0], &A[0], 70U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    work[i] = 0.0;
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  for (itau = 0; itau < 7; itau++) {
    ii = itau * 10 + itau;
    ix0 = ii + 2;
    b_atmp = Q[ii];
    b_tau[itau] = 0.0;
    beta1 = AEBController_xnrm2_p(9 - itau, Q, ii + 2);
    if (beta1 != 0.0) {
      c_A = Q[ii];
      beta1 = rt_hypotd_snf(c_A, beta1);
      if (c_A >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - itau;
          for (lastv = ix0; lastv <= i + 10; lastv++) {
            Q[lastv - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

        beta1 = rt_hypotd_snf(b_atmp, AEBController_xnrm2_p(9 - itau, Q, ii + 2));
        if (b_atmp >= 0.0) {
          beta1 = -beta1;
        }

        b_tau[itau] = (beta1 - b_atmp) / beta1;
        b_atmp = 1.0 / (b_atmp - beta1);
        for (lastv = ix0; lastv <= i + 10; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        for (lastv = 0; lastv <= knt; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        b_atmp = beta1;
      } else {
        b_tau[itau] = (beta1 - c_A) / beta1;
        b_atmp = 1.0 / (c_A - beta1);
        i = ii - itau;
        for (lastv = ix0; lastv <= i + 10; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        b_atmp = beta1;
      }
    }

    Q[ii] = b_atmp;
    if (itau + 1 < 7) {
      Q[ii] = 1.0;
      ix0 = ii + 11;
      if (b_tau[itau] != 0.0) {
        lastv = 10 - itau;
        i = ii - itau;
        while ((lastv > 0) && (Q[i + 9] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 5 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 10 + ii;
          ia = i + 11;
          do {
            exitg1 = 0;
            if (ia <= (i + lastv) + 10) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (10 * knt + ii) + 11;
          for (iac = ix0; iac <= i; iac += 10) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[(ii + ia) - iac] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 11, 10);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 11;
              ix0 = (lastv + jA) + 10;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 11] * beta1;
              }
            }

            jA += 10;
          }
        }
      }

      Q[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 7; ii++) {
    for (itau = 0; itau <= ii; itau++) {
      // Start for MATLABSystem: '<S10>/MATLAB System'
      R[itau + 7 * ii] = Q[10 * ii + itau];
    }

    for (itau = ii + 2; itau < 8; itau++) {
      R[(itau + 7 * ii) - 1] = 0.0;
    }

    // Start for MATLABSystem: '<S10>/MATLAB System'
    work[ii] = 0.0;
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  for (itau = 6; itau >= 0; itau--) {
    ii = (itau * 10 + itau) + 10;
    if (itau + 1 < 7) {
      Q[ii - 10] = 1.0;
      ix0 = ii + 1;
      if (b_tau[itau] != 0.0) {
        lastv = 10 - itau;
        i = (ii - itau) - 1;
        while ((lastv > 0) && (Q[i] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 5 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 10 + ii;
          ia = i + 1;
          do {
            exitg1 = 0;
            if (ia <= i + lastv) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (10 * knt + ii) + 1;
          for (iac = ix0; iac <= i; iac += 10) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[((ii + ia) - iac) - 10] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 1, 10);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 1;
              ix0 = lastv + jA;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 11] * beta1;
              }
            }

            jA += 10;
          }
        }
      }
    }

    i = ii - itau;
    for (lastv = ii - 8; lastv <= i; lastv++) {
      Q[lastv - 1] *= -b_tau[itau];
    }

    Q[ii - 10] = 1.0 - b_tau[itau];
    for (i = 0; i < itau; i++) {
      Q[(ii - i) - 11] = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_mtimes(const real_T A_data[],
  const int32_T A_size[2], real_T C_data[], int32_T C_size[2])
{
  int32_T mc;
  static const real_T b[80] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0,
    0.0, 6.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0,
    2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26 };

  mc = A_size[0] - 1;
  C_size[0] = A_size[0];
  C_size[1] = 4;
  for (int32_T j = 0; j < 4; j++) {
    int32_T boffset;
    int32_T coffset;
    coffset = (mc + 1) * j;
    boffset = j * 20;
    for (int32_T i = 0; i <= mc; i++) {
      C_data[coffset + i] = 0.0;
    }

    for (int32_T i = 0; i < 20; i++) {
      real_T bkj;
      int32_T aoffset;
      aoffset = i * A_size[0];
      bkj = b[boffset + i];
      for (int32_T b_i = 0; b_i <= mc; b_i++) {
        int32_T tmp;
        tmp = coffset + b_i;
        C_data[tmp] += A_data[aoffset + b_i] * bkj;
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
boolean_T ACCWithSensorFusionModelClass::AEBController_any(const boolean_T x[6])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 5)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_c4_mpclib_anonFcn2(const
  real_T runtimedata_x[7], const real_T runtimedata_md[11], const real_T
  runtimedata_OutputMin[30], const real_T runtimedata_OutputMax[30], const
  real_T runtimedata_Parameters[7], const real_T z[75], real_T varargout_1_data[],
  int32_T varargout_1_size[2], real_T varargout_2[70], real_T varargout_3_data[],
  int32_T varargout_3_size[2], real_T varargout_4[5250])
{
  real_T tmp_data_0[320];
  real_T Jmv[280];
  real_T varargin_2_data[240];
  real_T X[77];
  real_T b_X[77];
  real_T c[60];
  real_T varargin_1_data[60];
  real_T Ak[49];
  real_T Ak1[49];
  real_T U[44];
  real_T b_U[44];
  real_T Ck[21];
  real_T b_val[21];
  real_T Bk1[14];
  real_T val[14];
  real_T ic[7];
  real_T tmp_0[7];
  real_T tmp_1[7];
  real_T yk[3];
  real_T b_ic_idx_0;
  real_T b_ic_idx_1;
  real_T b_ic_idx_2;
  real_T e;
  real_T runtimedata_OutputMax_0;
  real_T runtimedata_OutputMax_1;
  real_T runtimedata_OutputMin_0;
  real_T s;
  int32_T icf_tmp_0[6];
  int32_T icf_tmp[3];
  int32_T Jx_tmp;
  int32_T i;
  int32_T k;
  int32_T trueCount;
  int8_T Je[60];
  int8_T Je_data[60];
  int8_T tmp_data[60];
  int8_T c_empty_non_axis_sizes_0[2];
  int8_T c_empty_non_axis_sizes_1[2];
  int8_T c_empty_non_axis_sizes_2[2];
  int8_T b_sizes_idx_0;
  int8_T tmp_2;
  boolean_T icf[60];
  boolean_T tmp[30];
  boolean_T icf_0[6];
  boolean_T x[3];
  boolean_T c_empty_non_axis_sizes;
  static const real_T b_b[80] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 6.0, 0.0,
    6.0, 0.0, 6.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0,
    2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26, 0.0, 2.26 };

  static const int8_T d[21] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1 };

  static const real_T s_0[3] = { 15.0, 0.5, 0.5 };

  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T b_varargin_1_size_idx_0;
  int32_T b_varargin_1_size_idx_1;
  int32_T varargin_1_size_idx_1;
  boolean_T exitg1;
  getXUe_eNtkTHmx(z, runtimedata_x, runtimedata_md, X, U, &e);
  for (trueCount = 0; trueCount < 4900; trueCount++) {
    AEBController_DW.Jx[trueCount] = 0.0;
  }

  for (trueCount = 0; trueCount < 1400; trueCount++) {
    AEBController_DW.Jmv[trueCount] = 0.0;
  }

  for (i = 0; i < 70; i++) {
    varargout_2[i] = 0.0;
  }

  for (trueCount = 0; trueCount < 7; trueCount++) {
    ic[trueCount] = static_cast<real_T>(trueCount) + 1.0;
  }

  for (trueCount = 0; trueCount < 11; trueCount++) {
    i = trueCount << 2;
    b_U[i] = U[trueCount];
    b_U[i + 1] = U[trueCount + 11];
    b_U[i + 2] = U[trueCount + 22];
    b_U[i + 3] = U[trueCount + 33];
  }

  for (Jx_tmp = 0; Jx_tmp < 7; Jx_tmp++) {
    for (trueCount = 0; trueCount < 11; trueCount++) {
      b_X[Jx_tmp + 7 * trueCount] = X[11 * Jx_tmp + trueCount];
    }
  }

  for (i = 0; i < 10; i++) {
    helperNLMPCStateJacFcn_3EmkNgDE(&b_X[7 * i], runtimedata_Parameters[0],
      runtimedata_Parameters[1], runtimedata_Parameters[2],
      runtimedata_Parameters[3], runtimedata_Parameters[4],
      runtimedata_Parameters[5], runtimedata_Parameters[6], Ak, val);
    trueCount = (i + 1) * 7;
    helperNLMPCStateJacFcn_3EmkNgDE(&b_X[trueCount], runtimedata_Parameters[0],
      runtimedata_Parameters[1], runtimedata_Parameters[2],
      runtimedata_Parameters[3], runtimedata_Parameters[4],
      runtimedata_Parameters[5], runtimedata_Parameters[6], Ak1, Bk1);
    Jx_tmp = i << 2;
    helperNLMPCStateFcn_pC1vAxkb(&b_X[7 * i], &b_U[Jx_tmp],
      runtimedata_Parameters[0], runtimedata_Parameters[1],
      runtimedata_Parameters[2], runtimedata_Parameters[3],
      runtimedata_Parameters[4], runtimedata_Parameters[5],
      runtimedata_Parameters[6], tmp_0);
    helperNLMPCStateFcn_pC1vAxkb(&b_X[trueCount], &b_U[Jx_tmp],
      runtimedata_Parameters[0], runtimedata_Parameters[1],
      runtimedata_Parameters[2], runtimedata_Parameters[3],
      runtimedata_Parameters[4], runtimedata_Parameters[5],
      runtimedata_Parameters[6], tmp_1);
    for (k = 0; k < 7; k++) {
      varargout_2[static_cast<int32_T>(ic[k]) - 1] = (b_X[7 * i + k] + (tmp_0[k]
        + tmp_1[k]) * 0.025) - b_X[trueCount + k];
      if (i + 1 > 1) {
        for (Jx_tmp = 0; Jx_tmp < 7; Jx_tmp++) {
          AEBController_DW.Jx[((static_cast<int32_T>(ic[Jx_tmp]) + 70 * k) + 490
                               * (i - 1)) - 1] = Ak[7 * k + Jx_tmp] * 0.025;
        }

        Jx_tmp = ((70 * k + static_cast<int32_T>(ic[k])) + (i - 1) * 490) - 1;
        AEBController_DW.Jx[Jx_tmp]++;
      }
    }

    for (k = 0; k < 7; k++) {
      for (trueCount = 0; trueCount < 7; trueCount++) {
        AEBController_DW.Jx[((static_cast<int32_T>(ic[trueCount]) + 70 * k) +
                             490 * i) - 1] = Ak1[7 * k + trueCount] * 0.025;
      }

      Jx_tmp = ((70 * k + static_cast<int32_T>(ic[k])) + 490 * i) - 1;
      AEBController_DW.Jx[Jx_tmp]--;
    }

    for (trueCount = 0; trueCount < 14; trueCount++) {
      val[trueCount] = (val[trueCount] + Bk1[trueCount]) * 0.025;
    }

    for (k = 0; k < 2; k++) {
      for (trueCount = 0; trueCount < 7; trueCount++) {
        AEBController_DW.Jmv[((static_cast<int32_T>(ic[trueCount]) + 70 * k) +
                              140 * i) - 1] = val[7 * k + trueCount];
      }
    }

    for (trueCount = 0; trueCount < 7; trueCount++) {
      ic[trueCount] += 7.0;
    }
  }

  for (trueCount = 0; trueCount < 30; trueCount++) {
    tmp[trueCount] = rtIsInf(runtimedata_OutputMin[trueCount]);
  }

  all_WqXfAvPF(tmp, x);
  c_empty_non_axis_sizes = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 3)) {
    if (!x[i]) {
      c_empty_non_axis_sizes = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (c_empty_non_axis_sizes) {
    for (trueCount = 0; trueCount < 30; trueCount++) {
      tmp[trueCount] = rtIsInf(runtimedata_OutputMax[trueCount]);
    }

    all_WqXfAvPF(tmp, x);
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 3)) {
      if (!x[i]) {
        c_empty_non_axis_sizes = false;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (c_empty_non_axis_sizes) {
    k = 0;
    varargin_1_size_idx_1 = 0;
    b_varargin_1_size_idx_0 = 0;
    b_varargin_1_size_idx_1 = 0;
  } else {
    for (i = 0; i < 60; i++) {
      c[i] = 0.0;
      icf[i] = true;
    }

    for (trueCount = 0; trueCount < 4200; trueCount++) {
      AEBController_DW.b_Jx[trueCount] = 0.0;
    }

    for (i = 0; i < 60; i++) {
      Je[i] = 0;
    }

    b_ic_idx_0 = 1.0;
    b_ic_idx_1 = 2.0;
    b_ic_idx_2 = 3.0;
    for (i = 0; i < 10; i++) {
      icf_tmp[0] = static_cast<int32_T>(b_ic_idx_0);
      runtimedata_OutputMin_0 = runtimedata_OutputMin[i];
      icf[static_cast<int32_T>(b_ic_idx_0) - 1] = ((!rtIsInf
        (runtimedata_OutputMin_0)) && (!rtIsNaN(runtimedata_OutputMin_0)));
      icf_tmp[1] = static_cast<int32_T>(b_ic_idx_1);
      runtimedata_OutputMin_0 = runtimedata_OutputMin[i + 10];
      icf[static_cast<int32_T>(b_ic_idx_1) - 1] = ((!rtIsInf
        (runtimedata_OutputMin_0)) && (!rtIsNaN(runtimedata_OutputMin_0)));
      icf_tmp[2] = static_cast<int32_T>(b_ic_idx_2);
      runtimedata_OutputMin_0 = runtimedata_OutputMin[i + 20];
      icf[static_cast<int32_T>(b_ic_idx_2) - 1] = ((!rtIsInf
        (runtimedata_OutputMin_0)) && (!rtIsNaN(runtimedata_OutputMin_0)));
      runtimedata_OutputMin_0 = runtimedata_OutputMax[i];
      icf[static_cast<int32_T>(b_ic_idx_0 + 3.0) - 1] = ((!rtIsInf
        (runtimedata_OutputMin_0)) && (!rtIsNaN(runtimedata_OutputMin_0)));
      icf_tmp_0[0] = static_cast<int32_T>(b_ic_idx_0) - 1;
      icf_tmp_0[3] = static_cast<int32_T>(b_ic_idx_0 + 3.0) - 1;
      runtimedata_OutputMax_0 = runtimedata_OutputMax[i + 10];
      icf[static_cast<int32_T>(b_ic_idx_1 + 3.0) - 1] = ((!rtIsInf
        (runtimedata_OutputMax_0)) && (!rtIsNaN(runtimedata_OutputMax_0)));
      icf_tmp_0[1] = static_cast<int32_T>(b_ic_idx_1) - 1;
      icf_tmp_0[4] = static_cast<int32_T>(b_ic_idx_1 + 3.0) - 1;
      runtimedata_OutputMax_1 = runtimedata_OutputMax[i + 20];
      icf[static_cast<int32_T>(b_ic_idx_2 + 3.0) - 1] = ((!rtIsInf
        (runtimedata_OutputMax_1)) && (!rtIsNaN(runtimedata_OutputMax_1)));
      icf_tmp_0[2] = static_cast<int32_T>(b_ic_idx_2) - 1;
      icf_tmp_0[5] = static_cast<int32_T>(b_ic_idx_2 + 3.0) - 1;
      for (trueCount = 0; trueCount < 6; trueCount++) {
        icf_0[trueCount] = icf[icf_tmp_0[trueCount]];
      }

      if (AEBController_any(icf_0)) {
        for (trueCount = 0; trueCount < 21; trueCount++) {
          Ck[trueCount] = d[trueCount];
        }

        yk[0] = X[i + 23] / 15.0;
        yk[1] = X[i + 45] / 0.5;
        yk[2] = (X[i + 56] + X[i + 67]) / 0.5;
        for (k = 0; k < 3; k++) {
          s = s_0[k];
          for (trueCount = 0; trueCount < 7; trueCount++) {
            Jx_tmp = 3 * trueCount + k;
            Ck[Jx_tmp] /= s;
          }

          c[icf_tmp[k] - 1] = (runtimedata_OutputMin[10 * k + i] - e) - yk[k];
        }

        c[static_cast<int32_T>(b_ic_idx_0 + 3.0) - 1] = (yk[0] -
          runtimedata_OutputMin_0) - e;
        c[static_cast<int32_T>(b_ic_idx_1 + 3.0) - 1] = (yk[1] -
          runtimedata_OutputMax_0) - e;
        c[static_cast<int32_T>(b_ic_idx_2 + 3.0) - 1] = (yk[2] -
          runtimedata_OutputMax_1) - e;
        for (trueCount = 0; trueCount < 21; trueCount++) {
          b_val[trueCount] = -Ck[trueCount];
        }

        for (k = 0; k < 7; k++) {
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_0) + 60 * k) +
            420 * i) - 1] = b_val[3 * k];
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_1) + 60 * k) +
            420 * i) - 1] = b_val[3 * k + 1];
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_2) + 60 * k) +
            420 * i) - 1] = b_val[3 * k + 2];
        }

        for (k = 0; k < 7; k++) {
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_0 + 3.0) + 60 *
            k) + 420 * i) - 1] = Ck[3 * k];
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_1 + 3.0) + 60 *
            k) + 420 * i) - 1] = Ck[3 * k + 1];
          AEBController_DW.b_Jx[((static_cast<int32_T>(b_ic_idx_2 + 3.0) + 60 *
            k) + 420 * i) - 1] = Ck[3 * k + 2];
        }

        Je[static_cast<int32_T>(b_ic_idx_0) - 1] = -1;
        Je[static_cast<int32_T>(b_ic_idx_1) - 1] = -1;
        Je[static_cast<int32_T>(b_ic_idx_2) - 1] = -1;
        Je[static_cast<int32_T>(b_ic_idx_0 + 3.0) - 1] = -1;
        Je[static_cast<int32_T>(b_ic_idx_1 + 3.0) - 1] = -1;
        Je[static_cast<int32_T>(b_ic_idx_2 + 3.0) - 1] = -1;
      }

      b_ic_idx_0 += 6.0;
      b_ic_idx_1 += 6.0;
      b_ic_idx_2 += 6.0;
    }

    trueCount = 0;
    for (i = 0; i < 60; i++) {
      if (icf[i]) {
        trueCount++;
      }
    }

    k = trueCount;
    trueCount = 0;
    for (i = 0; i < 60; i++) {
      if (icf[i]) {
        tmp_data[trueCount] = static_cast<int8_T>(i);
        trueCount++;
      }
    }

    varargin_1_size_idx_1 = 1;
    for (trueCount = 0; trueCount < k; trueCount++) {
      varargin_1_data[trueCount] = c[tmp_data[trueCount]];
    }

    if (k == 0) {
      b_varargin_1_size_idx_0 = 0;
      b_varargin_1_size_idx_1 = 0;
    } else {
      for (trueCount = 0; trueCount < 10; trueCount++) {
        for (Jx_tmp = 0; Jx_tmp < 7; Jx_tmp++) {
          for (i = 0; i < k; i++) {
            AEBController_DW.b_Jx_data[(i + k * Jx_tmp) + k * 7 * trueCount] =
              AEBController_DW.b_Jx[(60 * Jx_tmp + tmp_data[i]) + 420 *
              trueCount];
          }
        }
      }

      c_empty_non_axis_sizes_0[0] = static_cast<int8_T>(k);
      for (trueCount = 0; trueCount < k; trueCount++) {
        for (Jx_tmp = 0; Jx_tmp < 70; Jx_tmp++) {
          AEBController_DW.b_Jx[Jx_tmp + 70 * trueCount] =
            AEBController_DW.b_Jx_data[c_empty_non_axis_sizes_0[0] * Jx_tmp +
            trueCount];
        }
      }

      tmp_size[0] = k;
      tmp_size[1] = 20;
      i = k * 20;
      for (trueCount = 0; trueCount < i; trueCount++) {
        AEBController_DW.tmp_data[trueCount] = 0.0;
      }

      AEBController_mtimes(AEBController_DW.tmp_data, tmp_size, tmp_data_0,
                           tmp_size_0);
      i = tmp_size_0[0];
      for (trueCount = 0; trueCount < i; trueCount++) {
        varargin_2_data[trueCount << 2] = tmp_data_0[trueCount];
        varargin_2_data[1 + (trueCount << 2)] = tmp_data_0[trueCount +
          tmp_size_0[0]];
        varargin_2_data[2 + (trueCount << 2)] = tmp_data_0[(tmp_size_0[0] << 1)
          + trueCount];
        varargin_2_data[3 + (trueCount << 2)] = tmp_data_0[tmp_size_0[0] * 3 +
          trueCount];
      }

      c_empty_non_axis_sizes_0[0] = 70;
      if (tmp_size_0[0] != 0) {
        c_empty_non_axis_sizes_1[0] = 4;
      } else {
        c_empty_non_axis_sizes_1[0] = 0;
      }

      c_empty_non_axis_sizes_2[0] = 1;
      b_varargin_1_size_idx_0 = c_empty_non_axis_sizes_1[0] + 71;
      b_varargin_1_size_idx_1 = k;
      i = c_empty_non_axis_sizes_1[0];
      for (trueCount = 0; trueCount < k; trueCount++) {
        Je_data[trueCount] = Je[tmp_data[trueCount]];
        for (Jx_tmp = 0; Jx_tmp < 70; Jx_tmp++) {
          AEBController_DW.b_varargin_1_data[Jx_tmp + b_varargin_1_size_idx_0 *
            trueCount] = AEBController_DW.b_Jx[c_empty_non_axis_sizes_0[0] *
            trueCount + Jx_tmp];
        }

        for (Jx_tmp = 0; Jx_tmp < i; Jx_tmp++) {
          AEBController_DW.b_varargin_1_data[(Jx_tmp + b_varargin_1_size_idx_0 *
            trueCount) + 70] = varargin_2_data[c_empty_non_axis_sizes_1[0] *
            trueCount + Jx_tmp];
        }
      }

      for (trueCount = 0; trueCount < k; trueCount++) {
        AEBController_DW.b_varargin_1_data[(c_empty_non_axis_sizes_1[0] +
          b_varargin_1_size_idx_0 * trueCount) + 70] =
          Je_data[c_empty_non_axis_sizes_2[0] * trueCount];
      }
    }
  }

  c_empty_non_axis_sizes = ((k != 0) && (varargin_1_size_idx_1 != 0));
  if (!c_empty_non_axis_sizes) {
    c_empty_non_axis_sizes_0[0] = static_cast<int8_T>(k);
  } else if (c_empty_non_axis_sizes) {
    c_empty_non_axis_sizes_0[0] = static_cast<int8_T>(k);
  } else {
    c_empty_non_axis_sizes_0[0] = 0;
  }

  varargout_1_size[0] = c_empty_non_axis_sizes_0[0];
  varargout_1_size[1] = c_empty_non_axis_sizes;
  i = c_empty_non_axis_sizes;
  for (trueCount = 0; trueCount < i; trueCount++) {
    k = c_empty_non_axis_sizes_0[0];
    if (k - 1 >= 0) {
      std::memcpy(&varargout_1_data[0], &varargin_1_data[0],
                  static_cast<uint32_T>(k) * sizeof(real_T));
    }
  }

  c_empty_non_axis_sizes = ((b_varargin_1_size_idx_0 != 0) &&
    (b_varargin_1_size_idx_1 != 0));
  if (c_empty_non_axis_sizes) {
    b_sizes_idx_0 = static_cast<int8_T>(b_varargin_1_size_idx_0);
  } else {
    b_sizes_idx_0 = 0;
  }

  varargout_3_size[0] = b_sizes_idx_0;
  if (b_sizes_idx_0 == 0) {
    varargout_3_size[1] = b_varargin_1_size_idx_1;
    tmp_2 = static_cast<int8_T>(b_varargin_1_size_idx_1);
  } else if (c_empty_non_axis_sizes) {
    varargout_3_size[1] = b_varargin_1_size_idx_1;
    tmp_2 = static_cast<int8_T>(b_varargin_1_size_idx_1);
  } else {
    varargout_3_size[1] = 0;
    tmp_2 = 0;
  }

  i = b_sizes_idx_0 * tmp_2;
  if (i - 1 >= 0) {
    std::memcpy(&varargout_3_data[0], &AEBController_DW.b_varargin_1_data[0],
                static_cast<uint32_T>(i) * sizeof(real_T));
  }

  for (trueCount = 0; trueCount < 4; trueCount++) {
    for (Jx_tmp = 0; Jx_tmp < 70; Jx_tmp++) {
      e = 0.0;
      for (i = 0; i < 20; i++) {
        e += AEBController_DW.Jmv[70 * i + Jx_tmp] * b_b[20 * trueCount + i];
      }

      Jmv[trueCount + (Jx_tmp << 2)] = e;
    }
  }

  for (trueCount = 0; trueCount < 70; trueCount++) {
    for (Jx_tmp = 0; Jx_tmp < 70; Jx_tmp++) {
      varargout_4[Jx_tmp + 75 * trueCount] = AEBController_DW.Jx[70 * Jx_tmp +
        trueCount];
    }

    i = trueCount << 2;
    varargout_4[75 * trueCount + 70] = Jmv[i];
    varargout_4[75 * trueCount + 71] = Jmv[i + 1];
    varargout_4[75 * trueCount + 72] = Jmv[i + 2];
    varargout_4[75 * trueCount + 73] = Jmv[i + 3];
    varargout_4[75 * trueCount + 74] = 0.0;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factoryConstruct(int32_T
  nVarMax, int32_T mConstrMax, int32_T mIneq, int32_T mNonlinIneq,
  s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *obj)
{
  obj->nVarMax = nVarMax;
  obj->mNonlinIneq = mNonlinIneq;
  obj->mNonlinEq = 70;
  obj->mIneq = mIneq;
  obj->mEq = 70;
  obj->iNonIneq0 = (mIneq - mNonlinIneq) + 1;
  obj->iNonEq0 = 1;
  obj->sqpFval = 0.0;
  obj->sqpFval_old = 0.0;
  obj->cIneq.size[0] = mIneq;
  obj->cIneq_old.size[0] = mIneq;
  obj->grad.size[0] = nVarMax;
  obj->grad_old.size[0] = nVarMax;
  obj->FunctionEvaluations = 0;
  obj->sqpIterations = 0;
  obj->sqpExitFlag = 0;
  obj->lambdasqp.size[0] = mConstrMax;
  for (int32_T i = 0; i < mConstrMax; i++) {
    obj->lambdasqp.data[i] = 0.0;
  }

  obj->lambdaStopTest.size[0] = mConstrMax;
  obj->lambdaStopTestPrev.size[0] = mConstrMax;
  obj->steplength = 1.0;
  obj->delta_x.size[0] = nVarMax;
  for (int32_T i = 0; i < nVarMax; i++) {
    obj->delta_x.data[i] = 0.0;
  }

  obj->socDirection.size[0] = nVarMax;
  obj->workingset_old.size[0] = mConstrMax;
  if (mNonlinIneq > 0) {
    obj->JacCineqTrans_old.size[0] = nVarMax;
    obj->JacCineqTrans_old.size[1] = mNonlinIneq;
  } else {
    obj->JacCineqTrans_old.size[0] = 0;
    obj->JacCineqTrans_old.size[1] = 0;
  }

  obj->JacCeqTrans_old.size[0] = nVarMax;
  obj->JacCeqTrans_old.size[1] = 70;
  obj->gradLag.size[0] = nVarMax;
  obj->delta_gradLag.size[0] = nVarMax;
  obj->xstar.size[0] = nVarMax;
  obj->fstar = 0.0;
  obj->firstorderopt = 0.0;
  obj->lambda.size[0] = mConstrMax;
  for (int32_T i = 0; i < mConstrMax; i++) {
    obj->lambda.data[i] = 0.0;
  }

  obj->state = 0;
  obj->maxConstr = 0.0;
  obj->iterations = 0;
  obj->searchDir.size[0] = nVarMax;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factoryConstruct_oogs(int32_T
  MaxVars, int32_T obj_grad_size[1], int32_T obj_Hx_size[1], boolean_T
  *obj_hasLinear, int32_T *obj_nvar, int32_T *obj_maxVar, real_T *obj_beta,
  real_T *obj_rho, int32_T *obj_objtype, int32_T *obj_prev_objtype, int32_T
  *obj_prev_nvar, boolean_T *obj_prev_hasLinear, real_T *obj_gammaScalar)
{
  obj_grad_size[0] = MaxVars;
  obj_Hx_size[0] = MaxVars - 1;
  *obj_hasLinear = false;
  *obj_nvar = 0;
  *obj_maxVar = MaxVars;
  *obj_beta = 0.0;
  *obj_rho = 0.0;
  *obj_objtype = 3;
  *obj_prev_objtype = 3;
  *obj_prev_nvar = 0;
  *obj_prev_hasLinear = false;
  *obj_gammaScalar = 0.0;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factoryConstruct_oogsz(int32_T
  mIneqMax, int32_T nVarMax, int32_T mConstrMax,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj)
{
  obj->mConstr = 0;
  obj->mConstrOrig = 0;
  obj->mConstrMax = mConstrMax;
  obj->nVar = 75;
  obj->nVarOrig = 75;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  obj->Aineq.size[0] = mIneqMax * nVarMax;
  obj->bineq.size[0] = mIneqMax;
  obj->Aeq.size[0] = 70 * nVarMax;
  obj->lb.size[0] = nVarMax;
  obj->ub.size[0] = nVarMax;
  obj->indexLB.size[0] = nVarMax;
  obj->indexUB.size[0] = nVarMax;
  obj->indexFixed.size[0] = nVarMax;
  obj->mEqRemoved = 0;
  obj->ATwset.size[0] = nVarMax * mConstrMax;
  obj->bwset.size[0] = mConstrMax;
  obj->nActiveConstr = 0;
  obj->maxConstrWorkspace.size[0] = mConstrMax;
  for (int32_T i = 0; i < 5; i++) {
    obj->sizes[i] = 0;
    obj->sizesNormal[i] = 0;
    obj->sizesPhaseOne[i] = 0;
    obj->sizesRegularized[i] = 0;
    obj->sizesRegPhaseOne[i] = 0;
  }

  for (int32_T i = 0; i < 6; i++) {
    obj->isActiveIdx[i] = 0;
    obj->isActiveIdxNormal[i] = 0;
    obj->isActiveIdxPhaseOne[i] = 0;
    obj->isActiveIdxRegularized[i] = 0;
    obj->isActiveIdxRegPhaseOne[i] = 0;
  }

  obj->isActiveConstr.size[0] = mConstrMax;
  obj->Wid.size[0] = mConstrMax;
  obj->Wlocalidx.size[0] = mConstrMax;
  for (int32_T i = 0; i < 5; i++) {
    obj->nWConstr[i] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
}

// Function for MATLAB Function: '<S15>/NLMPC'
int32_T ACCWithSensorFusionModelClass::AEBController_checkVectorNonFinite
  (int32_T N, const real_T vec_data[], int32_T iv0)
{
  int32_T idx_current;
  int32_T idx_end;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0 - 2;
  idx_end = (iv0 + N) - 1;
  while (allFinite && (idx_current + 2 <= idx_end)) {
    real_T allFinite_tmp;
    allFinite_tmp = vec_data[idx_current + 1];
    allFinite = ((!rtIsInf(allFinite_tmp)) && (!rtIsNaN(allFinite_tmp)));
    idx_current++;
  }

  if (!allFinite) {
    if (rtIsNaN(vec_data[idx_current])) {
      status = -3;
    } else if (vec_data[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }

  return status;
}

// Function for MATLAB Function: '<S15>/NLMPC'
int32_T ACCWithSensorFusionModelClass::
  AEBController_computeConstraintsAndUserJacobian_(int32_T
  obj_next_next_next_next_next_b_value, const
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
  *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const real_T
  x[75], real_T Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[70],
  real_T JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
  JacEqTrans_workspace_data[], int32_T ldJE)
{
  real_T b_x[70];
  real_T a__3_data[60];
  int32_T a__3_size[2];
  int32_T a__4_size[2];
  int32_T col;
  int32_T col_end;
  int32_T idx_mat;
  int32_T row;
  int32_T status;
  boolean_T allFinite;
  if (obj_next_next_next_next_next_b_value > 0) {
    AEBController_c4_mpclib_anonFcn2
      (obj_next_next_next_next_next_next_next_b_value_workspace_runtim->x,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->md,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters,
       x, a__3_data, a__3_size, b_x, AEBController_DW.a__4_data, a__4_size,
       AEBController_DW.JacEqTrans_tmp);
    col = static_cast<uint8_T>(obj_next_next_next_next_next_b_value);
    for (row = 0; row < col; row++) {
      Cineq_workspace_data[(ineq0 + row) - 1] = a__3_data[row];
    }

    std::memcpy(&Ceq_workspace[0], &b_x[0], 70U * sizeof(real_T));
    col_end = a__4_size[0];
    for (row = 0; row < col_end; row++) {
      idx_mat = a__4_size[1];
      for (col = 0; col < idx_mat; col++) {
        JacIneqTrans_workspace_data[row + ldJI * ((iJI_col + col) - 1)] =
          AEBController_DW.a__4_data[a__4_size[0] * col + row];
      }
    }

    for (row = 0; row < 75; row++) {
      for (col = 0; col < 70; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          AEBController_DW.JacEqTrans_tmp[75 * col + row];
      }
    }
  } else {
    AEBController_c4_mpclib_anonFcn2
      (obj_next_next_next_next_next_next_next_b_value_workspace_runtim->x,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->md,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax,
       obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters,
       x, a__3_data, a__3_size, b_x, AEBController_DW.a__4_data, a__4_size,
       AEBController_DW.JacEqTrans_tmp);
    std::memcpy(&Ceq_workspace[0], &b_x[0], 70U * sizeof(real_T));
    for (row = 0; row < 75; row++) {
      for (col = 0; col < 70; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          AEBController_DW.JacEqTrans_tmp[75 * col + row];
      }
    }
  }

  status = AEBController_checkVectorNonFinite
    (obj_next_next_next_next_next_b_value, Cineq_workspace_data, ineq0);
  if (status == 1) {
    status = checkVectorNonFinite_KkAetmxy(Ceq_workspace);
    if (status == 1) {
      allFinite = true;
      row = -1;
      col = iJI_col;
      col_end = (iJI_col + obj_next_next_next_next_next_b_value) - 1;
      while (allFinite && (col <= col_end)) {
        row = -1;
        while (allFinite && (row + 2 <= 75)) {
          idx_mat = ((col - 1) * ldJI + row) + 1;
          allFinite = ((!rtIsInf(JacIneqTrans_workspace_data[idx_mat])) &&
                       (!rtIsNaN(JacIneqTrans_workspace_data[idx_mat])));
          row++;
        }

        col++;
      }

      if (!allFinite) {
        idx_mat = (col - 2) * ldJI + row;
        if (rtIsNaN(JacIneqTrans_workspace_data[idx_mat])) {
          status = -3;
        } else if (JacIneqTrans_workspace_data[idx_mat] < 0.0) {
          status = -1;
        } else {
          status = -2;
        }
      } else {
        row = -1;
        col = -1;
        while (allFinite && (col + 2 <= 70)) {
          row = -1;
          while (allFinite && (row + 2 <= 75)) {
            col_end = ((col + 1) * ldJE + row) + 1;
            allFinite = ((!rtIsInf(JacEqTrans_workspace_data[col_end])) &&
                         (!rtIsNaN(JacEqTrans_workspace_data[col_end])));
            row++;
          }

          col++;
        }

        if (!allFinite) {
          col_end = ldJE * col + row;
          if (rtIsNaN(JacEqTrans_workspace_data[col_end])) {
            status = -3;
          } else if (JacEqTrans_workspace_data[col_end] < 0.0) {
            status = -1;
          } else {
            status = -2;
          }
        }
      }
    }
  }

  return status;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_evalObjAndConstrAndDerivatives
  (int32_T obj_next_next_next_next_next_b_value, const
   s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
   *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const
   s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
   *obj_next_next_next_next_next_next_next_next_b_value_workspace, const real_T
   x[75], real_T grad_workspace_data[], real_T Cineq_workspace_data[], int32_T
   ineq0, real_T Ceq_workspace[70], real_T JacIneqTrans_workspace_data[],
   int32_T iJI_col, int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T
   ldJE, real_T *fval, int32_T *status)
{
  real_T X[77];
  real_T b_X[77];
  real_T b_x[75];
  real_T gfX[70];
  real_T U[44];
  real_T b_U[44];
  real_T gfU[20];
  real_T gfX_0[7];
  real_T ix[7];
  real_T c[4];
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T e;
  real_T fs;
  real_T gfU_idx_0;
  real_T iu_idx_0;
  real_T iu_idx_1;
  real_T obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
  real_T umvk_idx_0;
  real_T umvk_idx_1;
  real_T wtYerr;
  int32_T b_U_tmp;
  int32_T i;
  boolean_T allFinite;
  static const real_T c_0[80] = { 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 2.26 };

  static const real_T d[21] = { 0.0, 0.0, 0.066666666666666666, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0 };

  getXUe_eNtkTHmx(x,
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.x,
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.md, X, U, &e);
  for (i = 0; i < 70; i++) {
    gfX[i] = 0.0;
  }

  for (i = 0; i < 20; i++) {
    gfU[i] = 0.0;
  }

  fs = 0.0;
  iu_idx_0 = 1.0;
  iu_idx_1 = 2.0;
  for (b_U_tmp = 0; b_U_tmp < 7; b_U_tmp++) {
    ix[b_U_tmp] = static_cast<real_T>(b_U_tmp) + 1.0;
    for (i = 0; i < 11; i++) {
      b_X[b_U_tmp + 7 * i] = X[11 * b_U_tmp + i];
    }
  }

  for (i = 0; i < 11; i++) {
    b_U_tmp = i << 2;
    b_U[b_U_tmp] = U[i];
    b_U[b_U_tmp + 1] = U[i + 11];
    b_U[b_U_tmp + 2] = U[i + 22];
    b_U[b_U_tmp + 3] = U[i + 33];
  }

  for (i = 0; i < 10; i++) {
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [i];
    b_U_tmp = (i + 1) * 7;
    umvk_idx_0 = (b_X[b_U_tmp + 2] / 15.0 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[i] / 15.0) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    wtYerr = umvk_idx_0 * umvk_idx_0;
    duk_idx_1 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r *
      umvk_idx_0;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [i + 10];
    umvk_idx_0 = (b_X[b_U_tmp + 4] / 0.5 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[i + 10] / 0.5) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    wtYerr += umvk_idx_0 * umvk_idx_0;
    umvk_idx_1 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r
      * umvk_idx_0;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [i + 20];
    umvk_idx_0 = ((b_X[b_U_tmp + 5] + b_X[b_U_tmp + 6]) / 0.5 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[i + 20] / 0.5) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r *=
      umvk_idx_0;
    fs += umvk_idx_0 * umvk_idx_0 + wtYerr;
    for (b_U_tmp = 0; b_U_tmp < 7; b_U_tmp++) {
      gfX_0[b_U_tmp] = ((d[b_U_tmp + 7] * umvk_idx_1 + d[b_U_tmp] * duk_idx_1) +
                        d[b_U_tmp + 14] *
                        obj_next_next_next_next_next_next_next_next_b_value_workspace_r)
        + gfX[static_cast<int32_T>(ix[b_U_tmp]) - 1];
    }

    for (b_U_tmp = 0; b_U_tmp < 7; b_U_tmp++) {
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
        ix[b_U_tmp];
      gfX[static_cast<int32_T>
        (obj_next_next_next_next_next_next_next_next_b_value_workspace_r) - 1] =
        gfX_0[b_U_tmp];
      ix[b_U_tmp] =
        obj_next_next_next_next_next_next_next_next_b_value_workspace_r + 7.0;
    }

    b_U_tmp = i << 2;
    umvk_idx_0 = b_U[b_U_tmp] / 6.0;
    umvk_idx_1 = b_U[b_U_tmp + 1] / 2.26;
    if (i + 1 == 1) {
      duk_idx_0 = umvk_idx_0 -
        obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.lastMV
        [0] / 6.0;
      duk_idx_1 = umvk_idx_1 -
        obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.lastMV
        [1] / 2.26;
    } else {
      b_U_tmp = (i - 1) << 2;
      duk_idx_0 = umvk_idx_0 - b_U[b_U_tmp] / 6.0;
      duk_idx_1 = umvk_idx_1 - b_U[b_U_tmp + 1] / 2.26;
    }

    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVWeights
      [i];
    wtYerr = (umvk_idx_0 -
              obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVScaledTarget
              [i]) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    umvk_idx_0 = wtYerr;
    gfU_idx_0 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r *
      wtYerr / 6.0 + gfU[static_cast<int32_T>(iu_idx_0) - 1];
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVWeights
      [i + 10];
    wtYerr = (umvk_idx_1 -
              obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVScaledTarget
              [i + 10]) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    umvk_idx_1 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r
      * wtYerr / 2.26 + gfU[static_cast<int32_T>(iu_idx_1) - 1];
    gfU[static_cast<int32_T>(iu_idx_0) - 1] = gfU_idx_0;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVRateWeights
      [i];
    duk_idx_0 *= obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    gfU_idx_0 = umvk_idx_0 * umvk_idx_0;
    duk = duk_idx_0 * duk_idx_0;
    umvk_idx_0 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r
      * duk_idx_0 / 6.0;
    gfU[static_cast<int32_T>(iu_idx_1) - 1] = umvk_idx_1;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVRateWeights
      [i + 10];
    duk_idx_0 = obj_next_next_next_next_next_next_next_next_b_value_workspace_r *
      duk_idx_1;
    obj_next_next_next_next_next_next_next_next_b_value_workspace_r =
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r *
      duk_idx_0 / 2.26;
    fs = ((wtYerr * wtYerr + gfU_idx_0) + fs) + (duk_idx_0 * duk_idx_0 + duk);
    wtYerr = gfU[static_cast<int32_T>(iu_idx_1) - 1];
    gfU[static_cast<int32_T>(iu_idx_0) - 1] += umvk_idx_0;
    gfU[static_cast<int32_T>(iu_idx_1) - 1] = wtYerr +
      obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
    if (i + 1 > 1) {
      umvk_idx_1 = gfU[static_cast<int32_T>(iu_idx_1 - 2.0) - 1] -
        obj_next_next_next_next_next_next_next_next_b_value_workspace_r;
      gfU[static_cast<int32_T>(iu_idx_0 - 2.0) - 1] -= umvk_idx_0;
      gfU[static_cast<int32_T>(iu_idx_1 - 2.0) - 1] = umvk_idx_1;
    }

    iu_idx_0 += 2.0;
    iu_idx_1 += 2.0;
  }

  *fval = 100000.0 * e * e + fs;
  for (b_U_tmp = 0; b_U_tmp < 4; b_U_tmp++) {
    fs = 0.0;
    for (i = 0; i < 20; i++) {
      fs += c_0[(i << 2) + b_U_tmp] * (2.0 * gfU[i]);
    }

    c[b_U_tmp] = fs;
  }

  for (b_U_tmp = 0; b_U_tmp < 70; b_U_tmp++) {
    b_x[b_U_tmp] = 2.0 * gfX[b_U_tmp];
  }

  b_x[70] = c[0];
  b_x[71] = c[1];
  b_x[72] = c[2];
  b_x[73] = c[3];
  b_x[74] = 200000.0 * e;
  std::memcpy(&grad_workspace_data[0], &b_x[0], 75U * sizeof(real_T));
  allFinite = rtIsNaN(*fval);
  if (rtIsInf(*fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    allFinite = true;
    i = -1;
    while (allFinite && (i + 2 <= 75)) {
      e = grad_workspace_data[i + 1];
      allFinite = ((!rtIsInf(e)) && (!rtIsNaN(e)));
      i++;
    }

    if (!allFinite) {
      if (rtIsNaN(grad_workspace_data[i])) {
        *status = -3;
      } else if (grad_workspace_data[i] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    } else {
      *status = AEBController_computeConstraintsAndUserJacobian_
        (obj_next_next_next_next_next_b_value,
         obj_next_next_next_next_next_next_next_b_value_workspace_runtim, x,
         Cineq_workspace_data, ineq0, Ceq_workspace, JacIneqTrans_workspace_data,
         iJI_col, ldJI, JacEqTrans_workspace_data, ldJE);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_modifyOverheadPhaseOne_
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj)
{
  int32_T d;
  int32_T idxEq;
  idxEq = static_cast<uint16_T>(obj->sizes[0]);
  for (int32_T idx = 0; idx < idxEq; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
  }

  for (int32_T idx = 0; idx < 70; idx++) {
    idxEq = (obj->ldA * idx + obj->nVar) - 1;
    obj->Aeq.data[idxEq] = 0.0;
    obj->ATwset.data[idxEq + obj->ldA * (obj->isActiveIdx[1] - 1)] = 0.0;
  }

  idxEq = static_cast<uint8_T>(obj->sizes[2]);
  for (int32_T idx = 0; idx < idxEq; idx++) {
    obj->Aineq.data[(obj->nVar + obj->ldA * idx) - 1] = -1.0;
  }

  obj->indexLB.data[obj->sizes[3] - 1] = obj->nVar;
  obj->lb.data[obj->nVar - 1] = 1.0E-5;
  idxEq = obj->isActiveIdx[2];
  d = obj->nActiveConstr;
  for (int32_T idx = idxEq; idx <= d; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
  }

  idxEq = obj->isActiveIdx[4] - 1;
  if (obj->nWConstr[4] > 0) {
    d = obj->sizesNormal[4];
    for (int32_T idx = d; idx >= 1; idx--) {
      int32_T tmp;
      tmp = idxEq + idx;
      obj->isActiveConstr.data[tmp] = obj->isActiveConstr.data[tmp - 1];
    }
  } else {
    obj->isActiveConstr.data[(obj->isActiveIdx[4] + obj->sizesNormal[4]) - 1] =
      false;
  }

  obj->isActiveConstr.data[obj->isActiveIdx[4] - 1] = false;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_setProblemType
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T PROBLEM_TYPE)
{
  int32_T c;
  int32_T colOffsetATw;
  int32_T colOffsetAineq;
  int32_T d;
  int32_T d_tmp;
  int32_T idxUpperExisting;
  int32_T idx_col;
  int32_T offsetEq1;
  int32_T offsetEq2;
  int32_T tmp;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 75;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      idxUpperExisting = obj->isActiveIdx[4] - 1;
      offsetEq1 = static_cast<uint16_T>(obj->sizesNormal[4]);
      for (colOffsetATw = 0; colOffsetATw < offsetEq1; colOffsetATw++) {
        offsetEq2 = idxUpperExisting + colOffsetATw;
        obj->isActiveConstr.data[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1]
          = obj->isActiveConstr.data[offsetEq2];
        obj->isActiveConstr.data[offsetEq2] = false;
      }
    }

    for (offsetEq2 = 0; offsetEq2 < 5; offsetEq2++) {
      obj->sizes[offsetEq2] = obj->sizesNormal[offsetEq2];
    }

    for (offsetEq2 = 0; offsetEq2 < 6; offsetEq2++) {
      obj->isActiveIdx[offsetEq2] = obj->isActiveIdxNormal[offsetEq2];
    }
    break;

   case 1:
    obj->nVar = 76;
    obj->mConstr = obj->mConstrOrig + 1;
    for (offsetEq2 = 0; offsetEq2 < 5; offsetEq2++) {
      obj->sizes[offsetEq2] = obj->sizesPhaseOne[offsetEq2];
    }

    AEBController_modifyOverheadPhaseOne_(obj);
    for (offsetEq2 = 0; offsetEq2 < 6; offsetEq2++) {
      obj->isActiveIdx[offsetEq2] = obj->isActiveIdxPhaseOne[offsetEq2];
    }
    break;

   case 2:
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (offsetEq2 = 0; offsetEq2 < 5; offsetEq2++) {
      obj->sizes[offsetEq2] = obj->sizesRegularized[offsetEq2];
    }

    if (obj->probType != 4) {
      idxUpperExisting = obj->sizes[2] + 145;
      offsetEq1 = obj->sizes[2] + 75;
      c = static_cast<uint16_T>(obj->sizes[0]);
      for (idx_col = 0; idx_col < c; idx_col++) {
        colOffsetATw = obj->ldA * idx_col;
        d = obj->nVar;
        for (colOffsetAineq = 76; colOffsetAineq <= d; colOffsetAineq++) {
          obj->ATwset.data[(colOffsetAineq + colOffsetATw) - 1] = 0.0;
        }
      }

      idx_col = static_cast<uint8_T>(obj->sizes[2]);
      for (colOffsetATw = 0; colOffsetATw < idx_col; colOffsetATw++) {
        colOffsetAineq = obj->ldA * colOffsetATw - 1;
        for (c = 76; c <= colOffsetATw + 75; c++) {
          obj->Aineq.data[c + colOffsetAineq] = 0.0;
        }

        obj->Aineq.data[(colOffsetATw + colOffsetAineq) + 76] = -1.0;
        d = obj->nVar;
        for (c = colOffsetATw + 77; c <= d; c++) {
          obj->Aineq.data[c + colOffsetAineq] = 0.0;
        }
      }

      for (idx_col = 0; idx_col < 70; idx_col++) {
        colOffsetAineq = obj->ldA * idx_col - 1;
        colOffsetATw = (obj->isActiveIdx[1] - 1) * obj->ldA + colOffsetAineq;
        for (c = 76; c <= offsetEq1; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        d_tmp = idxUpperExisting + idx_col;
        d = d_tmp - 70;
        for (c = idxUpperExisting - 69; c <= d; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        offsetEq2 = d_tmp + colOffsetAineq;
        obj->Aeq.data[offsetEq2 - 69] = -1.0;
        tmp = d_tmp + colOffsetATw;
        obj->ATwset.data[tmp - 69] = -1.0;
        d = d_tmp - 68;
        for (c = d; c <= idxUpperExisting; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        for (c = idxUpperExisting + 1; c <= d_tmp; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        obj->Aeq.data[offsetEq2 + 1] = 1.0;
        obj->ATwset.data[tmp + 1] = 1.0;
        d = d_tmp + 2;
        offsetEq2 = obj->nVar;
        for (c = d; c <= offsetEq2; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }
      }

      idxUpperExisting = 75;
      offsetEq1 = obj->sizesNormal[3] + 1;
      offsetEq2 = obj->sizesRegularized[3];
      for (colOffsetATw = offsetEq1; colOffsetATw <= offsetEq2; colOffsetATw++)
      {
        idxUpperExisting++;
        obj->indexLB.data[colOffsetATw - 1] = idxUpperExisting;
      }

      if (obj->nWConstr[4] > 0) {
        idxUpperExisting = static_cast<uint16_T>(obj->sizesRegularized[4]);
        for (colOffsetATw = 0; colOffsetATw < idxUpperExisting; colOffsetATw++)
        {
          obj->isActiveConstr.data[obj->isActiveIdxRegularized[4] + colOffsetATw]
            = obj->isActiveConstr.data[(obj->isActiveIdx[4] + colOffsetATw) - 1];
        }
      }

      idxUpperExisting = obj->isActiveIdx[4];
      offsetEq1 = obj->isActiveIdxRegularized[4];
      if (idxUpperExisting <= offsetEq1 - 1) {
        std::memset(&obj->isActiveConstr.data[idxUpperExisting + -1], 0,
                    static_cast<uint32_T>(offsetEq1 - idxUpperExisting) * sizeof
                    (boolean_T));
      }

      idxUpperExisting = obj->sizes[2] + 215;
      for (colOffsetATw = 76; colOffsetATw <= idxUpperExisting; colOffsetATw++)
      {
        obj->lb.data[colOffsetATw - 1] = 0.0;
      }

      offsetEq1 = obj->isActiveIdx[2];
      offsetEq2 = obj->nActiveConstr;
      for (idxUpperExisting = offsetEq1; idxUpperExisting <= offsetEq2;
           idxUpperExisting++) {
        colOffsetATw = (idxUpperExisting - 1) * obj->ldA - 1;
        if (obj->Wid.data[idxUpperExisting - 1] == 3) {
          c = obj->Wlocalidx.data[idxUpperExisting - 1];
          colOffsetAineq = c + 74;
          for (idx_col = 76; idx_col <= colOffsetAineq; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }

          obj->ATwset.data[(c + colOffsetATw) + 75] = -1.0;
          colOffsetAineq = c + 76;
          c = obj->nVar;
          for (idx_col = colOffsetAineq; idx_col <= c; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }
        } else {
          colOffsetAineq = obj->nVar;
          for (idx_col = 76; idx_col <= colOffsetAineq; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }
        }
      }
    }

    for (offsetEq2 = 0; offsetEq2 < 6; offsetEq2++) {
      obj->isActiveIdx[offsetEq2] = obj->isActiveIdxRegularized[offsetEq2];
    }
    break;

   default:
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (offsetEq2 = 0; offsetEq2 < 5; offsetEq2++) {
      obj->sizes[offsetEq2] = obj->sizesRegPhaseOne[offsetEq2];
    }

    AEBController_modifyOverheadPhaseOne_(obj);
    for (offsetEq2 = 0; offsetEq2 < 6; offsetEq2++) {
      obj->isActiveIdx[offsetEq2] = obj->isActiveIdxRegPhaseOne[offsetEq2];
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_initActiveSet
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj)
{
  int32_T colOffsetATw;
  int32_T f;
  int32_T iATw0;
  int32_T iAeq0;
  int32_T idx;
  int32_T idxFillStart;
  AEBController_setProblemType(obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  if (idxFillStart <= obj->mConstrMax) {
    std::memset(&obj->isActiveConstr.data[idxFillStart + -1], 0,
                static_cast<uint32_T>((obj->mConstrMax - idxFillStart) + 1) *
                sizeof(boolean_T));
  }

  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 70;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + 70;
  idxFillStart = static_cast<uint16_T>(obj->sizes[0]);
  for (idx = 0; idx < idxFillStart; idx++) {
    obj->Wid.data[idx] = 1;
    obj->Wlocalidx.data[idx] = idx + 1;
    obj->isActiveConstr.data[idx] = true;
    colOffsetATw = obj->ldA * idx;
    iATw0 = static_cast<uint8_T>(obj->indexFixed.data[idx] - 1);
    for (iAeq0 = 0; iAeq0 < iATw0; iAeq0++) {
      obj->ATwset.data[iAeq0 + colOffsetATw] = 0.0;
    }

    obj->ATwset.data[(obj->indexFixed.data[idx] + colOffsetATw) - 1] = 1.0;
    iATw0 = obj->indexFixed.data[idx] + 1;
    f = obj->nVar;
    for (iAeq0 = iATw0; iAeq0 <= f; iAeq0++) {
      obj->ATwset.data[(iAeq0 + colOffsetATw) - 1] = 0.0;
    }

    obj->bwset.data[idx] = obj->ub.data[obj->indexFixed.data[idx] - 1];
  }

  for (idx = 0; idx < 70; idx++) {
    colOffsetATw = obj->sizes[0] + idx;
    obj->Wid.data[colOffsetATw] = 2;
    obj->Wlocalidx.data[colOffsetATw] = idx + 1;
    obj->isActiveConstr.data[colOffsetATw] = true;
    iAeq0 = obj->ldA * idx;
    iATw0 = obj->ldA * colOffsetATw;
    f = obj->nVar;
    for (idxFillStart = 0; idxFillStart < f; idxFillStart++) {
      obj->ATwset.data[iATw0 + idxFillStart] = obj->Aeq.data[iAeq0 +
        idxFillStart];
    }

    obj->bwset.data[colOffsetATw] = obj->beq[idx];
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factoryConstruct_oo(int32_T
  maxRows, int32_T maxCols, int32_T *obj_ldq, int32_T obj_QR_size[2], real_T
  obj_Q_data[], int32_T obj_Q_size[2], int32_T obj_jpvt_data[], int32_T
  obj_jpvt_size[1], int32_T *obj_mrows, int32_T *obj_ncols, int32_T
  obj_tau_size[1], int32_T *obj_minRowCol, boolean_T *obj_usedPivoting)
{
  int32_T loop_ub_tmp;
  *obj_ldq = maxRows;
  obj_QR_size[0] = maxRows;
  obj_QR_size[1] = maxCols;
  obj_Q_size[0] = maxRows;
  obj_Q_size[1] = maxRows;
  loop_ub_tmp = maxRows * maxRows;
  for (int32_T i = 0; i < loop_ub_tmp; i++) {
    obj_Q_data[i] = 0.0;
  }

  obj_jpvt_size[0] = maxCols;
  if (maxCols - 1 >= 0) {
    std::memset(&obj_jpvt_data[0], 0, static_cast<uint32_T>(maxCols) * sizeof
                (int32_T));
  }

  *obj_mrows = 0;
  *obj_ncols = 0;
  if (maxRows <= maxCols) {
    obj_tau_size[0] = maxRows;
  } else {
    obj_tau_size[0] = maxCols;
  }

  *obj_minRowCol = 0;
  *obj_usedPivoting = false;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factoryConstruct_oog(int32_T
  MaxDims, int32_T obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims,
  int32_T *obj_info, real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T
  *obj_regTol_, real_T *obj_workspace_, real_T *obj_workspace2_)
{
  obj_FMat_size[0] = MaxDims;
  obj_FMat_size[1] = MaxDims;
  *obj_ldm = MaxDims;
  *obj_ndims = 0;
  *obj_info = 0;
  *obj_scaleFactor = 0.0;
  *obj_ConvexCheck = true;
  *obj_regTol_ = (rtInf);
  *obj_workspace_ = (rtInf);
  *obj_workspace2_ = (rtInf);
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeGradLag(real_T
  workspace_data[], int32_T ldA, int32_T nVar, const real_T grad_data[], int32_T
  mIneq, const real_T AineqTrans_data[], const real_T AeqTrans_data[], const
  int32_T finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[],
  int32_T mLB, const int32_T finiteUB_data[], int32_T mUB, const real_T
  lambda_data[])
{
  int32_T b;
  int32_T f;
  int32_T g;
  int32_T iL0;
  int32_T ix;
  int32_T tmp;
  std::memcpy(&workspace_data[0], &grad_data[0], static_cast<uint16_T>(nVar) *
              sizeof(real_T));
  b = static_cast<uint16_T>(mFixed);
  for (iL0 = 0; iL0 < b; iL0++) {
    ix = finiteFixed_data[iL0];
    workspace_data[ix - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 69 + 1;
  for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
    g = (iL0 + nVar) - 1;
    for (b = iL0; b <= g; b++) {
      tmp = b - iL0;
      workspace_data[tmp] += AeqTrans_data[b - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 70;
    f = (mIneq - 1) * ldA + 1;
    for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
      g = (iL0 + nVar) - 1;
      for (b = iL0; b <= g; b++) {
        tmp = b - iL0;
        workspace_data[tmp] += AineqTrans_data[b - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 70;
  ix = static_cast<uint16_T>(mLB) - 1;
  for (b = 0; b <= ix; b++) {
    f = finiteLB_data[b];
    workspace_data[f - 1] -= lambda_data[iL0 + b];
  }

  iL0 = static_cast<uint16_T>(mLB) - 1 < 0 ? iL0 : static_cast<uint16_T>(mLB) +
    iL0;
  ix = static_cast<uint16_T>(mUB) - 1;
  for (b = 0; b <= ix; b++) {
    f = finiteUB_data[b];
    workspace_data[f - 1] += lambda_data[iL0 + b];
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_computePrimalFeasError(const
  real_T x[75], int32_T mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[],
  const real_T cEq[70], const int32_T finiteLB_data[], int32_T mLB, const real_T
  lb[75], const int32_T finiteUB_data[], int32_T mUB)
{
  real_T feasError;
  real_T u1;
  int32_T mIneq;
  feasError = 0.0;
  mIneq = mNonlinIneq + mLinIneq;
  for (int32_T idx = 0; idx < 70; idx++) {
    u1 = std::abs(cEq[idx]);
    if ((!(feasError >= u1)) && (!rtIsNaN(u1))) {
      feasError = u1;
    }
  }

  for (int32_T idx = 0; idx < mIneq; idx++) {
    u1 = cIneq_data[idx];
    if ((!(feasError >= u1)) && (!rtIsNaN(u1))) {
      feasError = u1;
    }
  }

  mIneq = static_cast<uint16_T>(mLB);
  for (int32_T idx = 0; idx < mIneq; idx++) {
    int32_T finiteLB;
    finiteLB = finiteLB_data[idx];
    u1 = lb[finiteLB - 1] - x[finiteLB - 1];
    if ((!(feasError >= u1)) && (!rtIsNaN(u1))) {
      feasError = u1;
    }
  }

  mIneq = static_cast<uint16_T>(mUB);
  for (int32_T idx = 0; idx < mIneq; idx++) {
    u1 = x[finiteUB_data[idx] - 1] - (rtInf);
    if ((!(feasError >= u1)) && (!rtIsNaN(u1))) {
      feasError = u1;
    }
  }

  return feasError;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeDualFeasError(int32_T
  nVar, const real_T gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= static_cast<uint16_T>(nVar) - 1)) {
    *gradOK = ((!rtIsInf(gradLag_data[idx])) && (!rtIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      real_T u1;
      u1 = std::abs(gradLag_data[idx]);
      if ((!(*val >= u1)) && (!rtIsNaN(u1))) {
        *val = u1;
      }

      idx++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_test_exit
  (sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction, const
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
   s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState, const real_T lb[75],
   boolean_T *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done,
   boolean_T *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
   *Flags_stepType)
{
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T k;
  int32_T mLambda;
  int32_T nVar;
  boolean_T isFeasible;
  *Flags_fevalOK = true;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 69;
  if (mLambda >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->
                lambdasqp.data[0], static_cast<uint32_T>(mLambda + 1) * sizeof
                (real_T));
  }

  AEBController_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad.data[0]);
      for (k = 2; k <= nVar; k++) {
        s = std::abs(TrialState->grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  smax = std::abs(TrialState->grad.data[idx_max - 1]);
  if ((smax <= 1.0) || rtIsNaN(smax)) {
    smax = 1.0;
  }

  if (rtIsInf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = AEBController_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  if ((MeritFunction->nlpPrimalFeasError <= 1.0) || rtIsNaN
      (MeritFunction->nlpPrimalFeasError)) {
    MeritFunction->feasRelativeFactor = 1.0;
  } else {
    MeritFunction->feasRelativeFactor = MeritFunction->nlpPrimalFeasError;
  }

  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  AEBController_computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    Flags_gradOK, &MeritFunction->nlpDualFeasError);
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    if (MeritFunction->nlpDualFeasError >= 0.0) {
      MeritFunction->firstOrderOpt = MeritFunction->nlpDualFeasError;
    } else {
      MeritFunction->firstOrderOpt = 0.0;
    }

    if (mLambda >= 0) {
      std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                  &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                  (mLambda + 1) * sizeof(real_T));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 * smax)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      *Flags_done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        *Flags_done = true;
        TrialState->sqpExitFlag = -3;
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_saveJacobian
  (s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *obj, int32_T nVar, int32_T mIneq,
   const real_T JacCineqTrans_data[], int32_T ineqCol0, const real_T
   JacCeqTrans_data[], int32_T ldJ)
{
  int32_T b;
  int32_T iCol;
  int32_T iCol_old;
  iCol = (ineqCol0 - 1) * ldJ;
  iCol_old = 0;
  b = mIneq - ineqCol0;
  for (int32_T idx_col = 0; idx_col <= b; idx_col++) {
    int32_T c;
    int32_T loop_ub_tmp;
    loop_ub_tmp = obj->JacCineqTrans_old.size[0] * obj->JacCineqTrans_old.size[1];
    if (loop_ub_tmp - 1 >= 0) {
      std::memcpy(&AEBController_DW.y_data_c[0], &obj->JacCineqTrans_old.data[0],
                  static_cast<uint32_T>(loop_ub_tmp) * sizeof(real_T));
    }

    c = static_cast<uint16_T>(nVar);
    for (int32_T k = 0; k < c; k++) {
      AEBController_DW.y_data_c[iCol_old + k] = JacCineqTrans_data[iCol + k];
    }

    if (loop_ub_tmp - 1 >= 0) {
      std::memcpy(&obj->JacCineqTrans_old.data[0], &AEBController_DW.y_data_c[0],
                  static_cast<uint32_T>(loop_ub_tmp) * sizeof(real_T));
    }

    iCol += ldJ;
    iCol_old += ldJ;
  }

  iCol = 0;
  iCol_old = 0;
  b = static_cast<uint16_T>(nVar);
  for (int32_T idx_col = 0; idx_col < 70; idx_col++) {
    for (int32_T k = 0; k < b; k++) {
      obj->JacCeqTrans_old.data[iCol_old + k] = JacCeqTrans_data[iCol + k];
    }

    iCol += ldJ;
    iCol_old = iCol;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_updateWorkingSetForNewQP(const
  real_T xk[75], s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet, int32_T
  mIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[70],
  int32_T mLB, const real_T lb[75], int32_T mUB, int32_T mFixed)
{
  real_T WorkingSet_beq;
  int32_T i;
  int32_T iEq0;
  int32_T iw0;
  int32_T nVar;
  nVar = WorkingSet->nVar;
  iw0 = WorkingSet->ldA * mFixed;
  iEq0 = 0;
  for (int32_T idx = 0; idx < 70; idx++) {
    WorkingSet_beq = -cEq[idx];
    WorkingSet->beq[idx] = WorkingSet_beq;
    WorkingSet->bwset.data[mFixed + idx] = WorkingSet_beq;
    for (i = 0; i < nVar; i++) {
      WorkingSet->ATwset.data[iw0 + i] = WorkingSet->Aeq.data[iEq0 + i];
    }

    iw0 += WorkingSet->ldA;
    iEq0 += WorkingSet->ldA;
  }

  i = static_cast<uint8_T>(mIneq);
  for (int32_T idx = 0; idx < i; idx++) {
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }

  i = static_cast<uint16_T>(mLB);
  for (int32_T idx = 0; idx < i; idx++) {
    WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1] = -lb
      [WorkingSet->indexLB.data[idx] - 1] + xk[WorkingSet->indexLB.data[idx] - 1];
  }

  i = static_cast<uint16_T>(mUB);
  for (int32_T idx = 0; idx < i; idx++) {
    WorkingSet->ub.data[WorkingSet->indexUB.data[idx] - 1] = (rtInf) -
      xk[WorkingSet->indexUB.data[idx] - 1];
  }

  i = static_cast<uint16_T>(mFixed);
  for (int32_T idx = 0; idx < i; idx++) {
    WorkingSet_beq = (rtInf) - xk[WorkingSet->indexFixed.data[idx] - 1];
    WorkingSet->ub.data[WorkingSet->indexFixed.data[idx] - 1] = WorkingSet_beq;
    WorkingSet->bwset.data[idx] = WorkingSet_beq;
  }

  if (WorkingSet->nActiveConstr > mFixed + 70) {
    iw0 = WorkingSet->nActiveConstr;
    for (int32_T idx = mFixed + 71; idx <= iw0; idx++) {
      switch (WorkingSet->Wid.data[idx - 1]) {
       case 4:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->lb.data
          [WorkingSet->indexLB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       case 5:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->ub.data
          [WorkingSet->indexUB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       default:
        {
          i = WorkingSet->Wlocalidx.data[idx - 1];
          WorkingSet->bwset.data[idx - 1] = WorkingSet->bineq.data[i - 1];
          if ((mNonlinIneq > 0) && (i > mIneq - mNonlinIneq)) {
            int32_T g;
            int32_T ix0;
            iEq0 = (idx - 1) * WorkingSet->ldA;
            ix0 = (i - 1) * WorkingSet->ldA;
            g = static_cast<uint16_T>(nVar);
            for (i = 0; i < g; i++) {
              WorkingSet->ATwset.data[iEq0 + i] = WorkingSet->Aineq.data[ix0 + i];
            }
          }
        }
        break;
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_jty(int32_T m, int32_T n,
  const real_T A_data[], int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    int32_T iy;
    b = static_cast<uint8_T>(n);
    for (int32_T b_iy = 0; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    iy = 0;
    d = (n - 1) * lda + 1;
    for (int32_T b_iy = 1; lda < 0 ? b_iy >= d : b_iy <= d; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += A_data[b - 1] * x_data[b - b_iy];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::
  AEBController_maxConstraintViolation_AMats_nonregularized_
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[])
{
  real_T u1;
  real_T v;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size[0] != 0) {
    if (mIneq - 1 >= 0) {
      std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                  static_cast<uint32_T>(mIneq) * sizeof(real_T));
    }

    AEBController_xgemv_jty(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
      x_data, obj->maxConstrWorkspace.data);
    mIneq = static_cast<uint8_T>(obj->sizes[2]);
    for (k = 0; k < mIneq; k++) {
      u1 = obj->maxConstrWorkspace.data[k];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
              (real_T));
  AEBController_xgemv_jty(obj->nVar, 70, obj->Aeq.data, obj->ldA, x_data,
    obj->maxConstrWorkspace.data);
  for (k = 0; k < 70; k++) {
    u1 = std::abs(obj->maxConstrWorkspace.data[k]);
    if ((!(v >= u1)) && (!rtIsNaN(u1))) {
      v = u1;
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::
  AEBController_maxConstraintViolation_AMats_regularized_
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size[0] != 0) {
    if (mIneq - 1 >= 0) {
      std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                  static_cast<uint32_T>(mIneq) * sizeof(real_T));
    }

    AEBController_xgemv_jty(75, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
      obj->maxConstrWorkspace.data);
    b = static_cast<uint8_T>(obj->sizes[2]);
    for (k = 0; k < b; k++) {
      obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[k] - x_data[k + 75];
      obj->maxConstrWorkspace.data[k] = obj_maxConstrWorkspace;
      if ((!(v >= obj_maxConstrWorkspace)) && (!rtIsNaN(obj_maxConstrWorkspace)))
      {
        v = obj_maxConstrWorkspace;
      }
    }
  }

  std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
              (real_T));
  AEBController_xgemv_jty(75, 70, obj->Aeq.data, obj->ldA, x_data,
    obj->maxConstrWorkspace.data);
  for (k = 0; k < 70; k++) {
    obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] - x_data
      [(mIneq + k) + 75]) + x_data[(obj->sizes[2] + k) + 145];
    obj_maxConstrWorkspace = std::abs(obj->maxConstrWorkspace.data[k]);
    if ((!(v >= obj_maxConstrWorkspace)) && (!rtIsNaN(obj_maxConstrWorkspace)))
    {
      v = obj_maxConstrWorkspace;
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_maxConstraintViolation_dsy
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[])
{
  real_T u1;
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    v = AEBController_maxConstraintViolation_AMats_regularized_(obj, x_data);
  } else {
    v = AEBController_maxConstraintViolation_AMats_nonregularized_(obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = static_cast<uint16_T>(obj->sizes[3]);
    for (idx = 0; idx < b; idx++) {
      u1 = -x_data[obj->indexLB.data[idx] - 1] - obj->lb.data[obj->
        indexLB.data[idx] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[4] > 0) {
    b = static_cast<uint16_T>(obj->sizes[4]);
    for (idx = 0; idx < b; idx++) {
      u1 = x_data[obj->indexUB.data[idx] - 1] - obj->ub.data[obj->
        indexUB.data[idx] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[0] > 0) {
    b = static_cast<uint16_T>(obj->sizes[0]);
    for (idx = 0; idx < b; idx++) {
      u1 = std::abs(x_data[obj->indexFixed.data[idx] - 1] - obj->ub.data
                    [obj->indexFixed.data[idx] - 1]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_jtyg(int32_T m, int32_T
  n, const real_T A[5625], int32_T lda, const real_T x_data[], real_T y_data[])
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T ix;
    for (int32_T b_iy = 0; b_iy < m; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    ix = 0;
    b = (n - 1) * lda + 1;
    for (int32_T b_iy = 1; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (int32_T ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeGrad_StoreHx
  (s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *obj, const real_T H[5625], const
   real_T f_data[], const real_T x_data[])
{
  int32_T i;
  int32_T ixlast;
  int32_T iy;
  switch (obj->objtype) {
   case 5:
    ixlast = obj->nvar;
    for (i = 0; i <= ixlast - 2; i++) {
      obj->grad.data[i] = 0.0;
    }

    obj->grad.data[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    AEBController_xgemv_jtyg(obj->nvar, obj->nvar, H, obj->nvar, x_data,
      obj->Hx.data);
    if (obj->nvar - 1 >= 0) {
      std::memcpy(&obj->grad.data[0], &obj->Hx.data[0], static_cast<uint32_T>
                  (obj->nvar) * sizeof(real_T));
    }

    if (obj->hasLinear) {
      iy = obj->grad.size[0];
      i = obj->grad.size[0];
      if (i - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_b[0], &obj->grad.data[0],
                    static_cast<uint32_T>(i) * sizeof(real_T));
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (i = 0; i < ixlast; i++) {
          AEBController_DW.y_data_b[i] += f_data[i];
        }
      }

      if (iy - 1 >= 0) {
        std::memcpy(&obj->grad.data[0], &AEBController_DW.y_data_b[0],
                    static_cast<uint32_T>(iy) * sizeof(real_T));
      }
    }
    break;

   case 4:
    ixlast = obj->maxVar;
    AEBController_xgemv_jtyg(obj->nvar, obj->nvar, H, obj->nvar, x_data,
      obj->Hx.data);
    iy = obj->nvar + 1;
    for (i = iy; i < ixlast; i++) {
      obj->Hx.data[i - 1] = x_data[i - 1] * obj->beta;
    }

    std::memcpy(&obj->grad.data[0], &obj->Hx.data[0], static_cast<uint16_T>
                (obj->maxVar - 1) * sizeof(real_T));
    if (obj->hasLinear) {
      iy = obj->grad.size[0];
      i = obj->grad.size[0];
      if (i - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_b[0], &obj->grad.data[0],
                    static_cast<uint32_T>(i) * sizeof(real_T));
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (i = 0; i < ixlast; i++) {
          AEBController_DW.y_data_b[i] += f_data[i];
        }
      }

      if (iy - 1 >= 0) {
        std::memcpy(&obj->grad.data[0], &AEBController_DW.y_data_b[0],
                    static_cast<uint32_T>(iy) * sizeof(real_T));
      }
    }

    ixlast = (obj->maxVar - obj->nvar) - 1;
    if (ixlast >= 1) {
      for (i = 0; i < ixlast; i++) {
        iy = obj->nvar + i;
        obj->grad.data[iy] += obj->rho;
      }
    }
    break;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_computeFval_ReuseHx(const
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *obj, real_T workspace_data[], const
  real_T f_data[], const real_T x_data[])
{
  real_T val;
  val = 0.0;
  switch (obj->objtype) {
   case 5:
    val = x_data[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T ixlast;
        int32_T maxRegVar;
        ixlast = obj->nvar;
        for (maxRegVar = 0; maxRegVar < ixlast; maxRegVar++) {
          workspace_data[maxRegVar] = 0.5 * obj->Hx.data[maxRegVar] +
            f_data[maxRegVar];
        }

        if (obj->nvar >= 1) {
          maxRegVar = obj->nvar;
          for (ixlast = 0; ixlast < maxRegVar; ixlast++) {
            val += workspace_data[ixlast] * x_data[ixlast];
          }
        }
      } else {
        if (obj->nvar >= 1) {
          int32_T ixlast;
          ixlast = obj->nvar;
          for (int32_T maxRegVar = 0; maxRegVar < ixlast; maxRegVar++) {
            val += x_data[maxRegVar] * obj->Hx.data[maxRegVar];
          }
        }

        val *= 0.5;
      }
    }
    break;

   case 4:
    {
      int32_T maxRegVar_tmp;
      maxRegVar_tmp = obj->maxVar;
      if (obj->hasLinear) {
        int32_T ixlast;
        int32_T maxRegVar;
        if (obj->nvar - 1 >= 0) {
          std::memcpy(&workspace_data[0], &f_data[0], static_cast<uint32_T>
                      (obj->nvar) * sizeof(real_T));
        }

        ixlast = obj->maxVar - obj->nvar;
        for (maxRegVar = 0; maxRegVar <= ixlast - 2; maxRegVar++) {
          workspace_data[obj->nvar + maxRegVar] = obj->rho;
        }

        maxRegVar = static_cast<uint16_T>(obj->maxVar - 1);
        for (ixlast = 0; ixlast < maxRegVar; ixlast++) {
          workspace_data[ixlast] += 0.5 * obj->Hx.data[ixlast];
        }

        if (obj->maxVar - 1 >= 1) {
          for (ixlast = 0; ixlast <= maxRegVar_tmp - 2; ixlast++) {
            val += workspace_data[ixlast] * x_data[ixlast];
          }
        }
      } else {
        int32_T maxRegVar;
        if (obj->maxVar - 1 >= 1) {
          for (int32_T ixlast = 0; ixlast <= maxRegVar_tmp - 2; ixlast++) {
            val += x_data[ixlast] * obj->Hx.data[ixlast];
          }
        }

        val *= 0.5;
        maxRegVar = obj->nvar + 1;
        for (int32_T ixlast = maxRegVar; ixlast < maxRegVar_tmp; ixlast++) {
          val += x_data[ixlast - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_xnrm2_pf(int32_T n, const
  real_T x_data[], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_xzlarfg(int32_T n, real_T
  *alpha1, real_T x_data[], int32_T ix0)
{
  real_T a;
  real_T tau;
  real_T xnorm;
  int32_T c_tmp;
  int32_T d;
  int32_T knt;
  tau = 0.0;
  if (n > 0) {
    xnorm = AEBController_xnrm2_pf(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        c_tmp = (ix0 + n) - 2;
        do {
          knt++;
          for (d = ix0; d <= c_tmp; d++) {
            x_data[d - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

        xnorm = rt_hypotd_snf(*alpha1, AEBController_xnrm2_pf(n - 1, x_data, ix0));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        for (d = ix0; d <= c_tmp; d++) {
          x_data[d - 1] *= a;
        }

        for (d = 0; d < knt; d++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        d = (ix0 + n) - 2;
        for (knt = ix0; knt <= d; knt++) {
          x_data[knt - 1] *= a;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv(int32_T m, int32_T n,
  const real_T A_data[], int32_T ia0, int32_T lda, const real_T x_data[],
  int32_T ix0, real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T iy;
    for (int32_T b_iy = 0; b_iy < n; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    iy = 0;
    b = (n - 1) * lda + ia0;
    for (int32_T b_iy = ia0; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + m) - 1;
      for (int32_T ia = b_iy; ia <= d; ia++) {
        c += x_data[((ix0 + ia) - b_iy) - 1] * A_data[ia - 1];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgerc(int32_T m, int32_T n,
  real_T alpha1, int32_T ix0, const real_T y_data[], real_T A_data[], int32_T
  ia0, int32_T lda)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j = 0; j < n; j++) {
      real_T temp;
      temp = y_data[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = (m + jA) - 1;
        for (int32_T ijA = jA; ijA <= b; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += lda;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xzlarf(int32_T m, int32_T n,
  int32_T iv0, real_T tau, real_T C_data[], int32_T ic0, int32_T ldc, real_T
  work_data[])
{
  int32_T coltop;
  int32_T exitg1;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg2;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * ldc + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    AEBController_xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0, work_data);
    AEBController_xgerc(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_qrf(real_T A_data[], const
  int32_T A_size[2], int32_T m, int32_T n, int32_T nfxd, real_T tau_data[])
{
  real_T b_atmp;
  real_T tau;
  int32_T i;
  int32_T ii;
  int32_T lda;
  int32_T loop_ub;
  int32_T mmi;
  lda = A_size[0];
  loop_ub = A_size[1];
  for (i = 0; i < loop_ub; i++) {
    AEBController_DW.work_data[i] = 0.0;
  }

  loop_ub = static_cast<uint16_T>(nfxd);
  for (i = 0; i < loop_ub; i++) {
    ii = i * lda + i;
    mmi = m - i;
    if (i + 1 < m) {
      b_atmp = A_data[ii];
      tau = AEBController_xzlarfg(mmi, &b_atmp, A_data, ii + 2);
      tau_data[i] = tau;
      A_data[ii] = b_atmp;
    } else {
      tau = 0.0;
      tau_data[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = A_data[ii];
      A_data[ii] = 1.0;
      AEBController_xzlarf(mmi, (n - i) - 1, ii + 1, tau, A_data, (ii + lda) + 1,
                           lda, AEBController_DW.work_data);
      A_data[ii] = b_atmp;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgeqrf(real_T A_data[], const
  int32_T A_size[2], int32_T m, int32_T n, real_T tau_data[], int32_T tau_size[1])
{
  int32_T i;
  int32_T loop_ub;
  int32_T minmn;
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  if (A_size[0] <= A_size[1]) {
    loop_ub = A_size[0];
  } else {
    loop_ub = A_size[1];
  }

  tau_size[0] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    tau_data[i] = 0.0;
  }

  if (minmn >= 1) {
    AEBController_qrf(A_data, A_size, m, n, minmn, tau_data);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factorQR
  (s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj, const real_T A_data[], int32_T
   mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = static_cast<uint16_T>(mrows);
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt.data[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    AEBController_xgeqrf(obj->QR.data, obj->QR.size, mrows, ncols, obj->tau.data,
                         obj->tau.size);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_squareQ_appendCol
  (s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj, const real_T vec_data[],
   int32_T iv0)
{
  real_T b_c;
  real_T s;
  real_T temp;
  int32_T b_iy;
  int32_T ia;
  int32_T idx;
  int32_T idxRotGCol;
  int32_T iy;
  int32_T iyend;
  int32_T temp_tmp;
  if (obj->mrows <= obj->ncols + 1) {
    obj->minRowCol = obj->mrows;
  } else {
    obj->minRowCol = obj->ncols + 1;
  }

  b_iy = obj->ldq * obj->ncols;
  idx = obj->ldq;
  if (obj->mrows != 0) {
    iyend = b_iy + obj->mrows;
    for (idxRotGCol = b_iy + 1; idxRotGCol <= iyend; idxRotGCol++) {
      obj->QR.data[idxRotGCol - 1] = 0.0;
    }

    iy = (obj->mrows - 1) * obj->ldq + 1;
    for (iyend = 1; idx < 0 ? iyend >= iy : iyend <= iy; iyend += idx) {
      b_c = 0.0;
      idxRotGCol = (iyend + obj->mrows) - 1;
      for (ia = iyend; ia <= idxRotGCol; ia++) {
        b_c += vec_data[((iv0 + ia) - iyend) - 1] * obj->Q.data[ia - 1];
      }

      obj->QR.data[b_iy] += b_c;
      b_iy++;
    }
  }

  obj->ncols++;
  obj->jpvt.data[obj->ncols - 1] = obj->ncols;
  for (idx = obj->mrows - 2; idx + 2 > obj->ncols; idx--) {
    iyend = (obj->ncols - 1) * obj->ldq + idx;
    temp = obj->QR.data[iyend + 1];
    xrotg_fUOOztoj(&obj->QR.data[iyend], &temp, &b_c, &s);
    obj->QR.data[iyend + 1] = temp;
    idxRotGCol = obj->ldq * idx;
    b_iy = obj->mrows;
    if (obj->mrows >= 1) {
      iy = obj->ldq + idxRotGCol;
      for (ia = 0; ia < b_iy; ia++) {
        iyend = iy + ia;
        temp_tmp = idxRotGCol + ia;
        temp = obj->Q.data[temp_tmp] * b_c + obj->Q.data[iyend] * s;
        obj->Q.data[iyend] = obj->Q.data[iyend] * b_c - obj->Q.data[temp_tmp] *
          s;
        obj->Q.data[temp_tmp] = temp;
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_deleteColMoveEnd
  (s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj, int32_T idx)
{
  real_T b_s;
  real_T b_temp;
  real_T c_c;
  int32_T QRk0;
  int32_T b_ix;
  int32_T b_temp_tmp;
  int32_T b_temp_tmp_0;
  int32_T e;
  int32_T endIdx;
  int32_T i;
  int32_T idxRotGCol;
  int32_T ix;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt.data[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt.data[idx - 1] = obj->jpvt.data[obj->ncols - 1];
    e = obj->minRowCol;
    for (endIdx = 0; endIdx < e; endIdx++) {
      obj->QR.data[endIdx + obj->ldq * (idx - 1)] = obj->QR.data[(obj->ncols - 1)
        * obj->ldq + endIdx];
    }

    obj->ncols--;
    if (obj->mrows <= obj->ncols) {
      obj->minRowCol = obj->mrows;
    } else {
      obj->minRowCol = obj->ncols;
    }

    if (idx < obj->mrows) {
      if (obj->mrows - 1 <= obj->ncols) {
        endIdx = obj->mrows - 1;
      } else {
        endIdx = obj->ncols;
      }

      e = endIdx;
      idxRotGCol = (idx - 1) * obj->ldq;
      while (e >= idx) {
        b_temp_tmp = e + idxRotGCol;
        b_temp = obj->QR.data[b_temp_tmp];
        xrotg_fUOOztoj(&obj->QR.data[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[b_temp_tmp] = b_temp;
        obj->QR.data[e + obj->ldq * (e - 1)] = 0.0;
        QRk0 = obj->ldq * idx + e;
        b_ix = obj->ncols - idx;
        if (b_ix >= 1) {
          ix = QRk0 - 1;
          for (i = 0; i < b_ix; i++) {
            b_temp = obj->QR.data[ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[ix] *
              b_s;
            obj->QR.data[ix] = b_temp;
            QRk0 += obj->ldq;
            ix += obj->ldq;
          }
        }

        i = (e - 1) * obj->ldq;
        QRk0 = obj->mrows;
        if (obj->mrows >= 1) {
          ix = obj->ldq + i;
          for (b_ix = 0; b_ix < QRk0; b_ix++) {
            b_temp_tmp = ix + b_ix;
            b_temp_tmp_0 = i + b_ix;
            b_temp = obj->Q.data[b_temp_tmp_0] * c_c + obj->Q.data[b_temp_tmp] *
              b_s;
            obj->Q.data[b_temp_tmp] = obj->Q.data[b_temp_tmp] * c_c -
              obj->Q.data[b_temp_tmp_0] * b_s;
            obj->Q.data[b_temp_tmp_0] = b_temp;
          }
        }

        e--;
      }

      for (e = idx + 1; e <= endIdx; e++) {
        ix = (e - 1) * obj->ldq;
        b_temp_tmp = e + ix;
        b_temp = obj->QR.data[b_temp_tmp];
        xrotg_fUOOztoj(&obj->QR.data[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[b_temp_tmp] = b_temp;
        QRk0 = (obj->ldq + 1) * e;
        i = obj->ncols - e;
        if (i >= 1) {
          b_ix = QRk0 - 1;
          for (idxRotGCol = 0; idxRotGCol < i; idxRotGCol++) {
            b_temp = obj->QR.data[b_ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[b_ix] *
              b_s;
            obj->QR.data[b_ix] = b_temp;
            QRk0 += obj->ldq;
            b_ix += obj->ldq;
          }
        }

        idxRotGCol = obj->mrows;
        if (obj->mrows >= 1) {
          b_ix = obj->ldq + ix;
          for (i = 0; i < idxRotGCol; i++) {
            b_temp_tmp = b_ix + i;
            b_temp_tmp_0 = ix + i;
            b_temp = obj->Q.data[b_temp_tmp_0] * c_c + obj->Q.data[b_temp_tmp] *
              b_s;
            obj->Q.data[b_temp_tmp] = obj->Q.data[b_temp_tmp] * c_c -
              obj->Q.data[b_temp_tmp_0] * b_s;
            obj->Q.data[b_temp_tmp_0] = b_temp;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
boolean_T ACCWithSensorFusionModelClass::AEBController_strcmp(const char_T a[7])
{
  int32_T ret;
  static const char_T b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  ret = memcmp(&a[0], &b[0], 7);
  return ret == 0;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_fullColLDL2_
  (s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, int32_T LD_offset, int32_T
   NColsRemain)
{
  int32_T LDimSizeP1;
  LDimSizeP1 = obj->ldm;
  for (int32_T k = 0; k < NColsRemain; k++) {
    real_T alpha1;
    real_T y;
    int32_T LD_diagOffset;
    int32_T b_k;
    int32_T subMatrixDim;
    LD_diagOffset = (LDimSizeP1 + 1) * k + LD_offset;
    alpha1 = -1.0 / obj->FMat.data[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    for (b_k = 0; b_k <= subMatrixDim; b_k++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + b_k];
    }

    y = obj->workspace_;
    if (!(alpha1 == 0.0)) {
      int32_T jA;
      jA = LD_diagOffset + LDimSizeP1;
      for (b_k = 0; b_k <= subMatrixDim; b_k++) {
        if (y != 0.0) {
          real_T temp;
          int32_T b;
          temp = y * alpha1;
          b = (subMatrixDim + jA) + 1;
          for (int32_T ijA = jA + 1; ijA <= b; ijA++) {
            obj->FMat.data[ijA - 1] += obj->workspace_ * temp;
          }
        }

        jA += obj->ldm;
      }
    }

    alpha1 = 1.0 / obj->FMat.data[LD_diagOffset - 1];
    b_k = (LD_diagOffset + subMatrixDim) + 1;
    for (subMatrixDim = LD_diagOffset + 1; subMatrixDim <= b_k; subMatrixDim++)
    {
      obj->FMat.data[subMatrixDim - 1] *= alpha1;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_partialColLDL3_
  (s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, int32_T LD_offset, int32_T
   NColsRemain)
{
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T b_idx;
  int32_T c;
  int32_T idx;
  int32_T ix;
  int32_T lastColC;
  int32_T lda;
  int32_T subRows;
  LDimSizeP1 = obj->ldm + 1;
  for (int32_T k = 0; k < 48; k++) {
    real_T y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LDimSizeP1 * k + LD_offset) - 1;
    for (b_idx = 0; b_idx <= subRows; b_idx++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + b_idx];
    }

    for (b_idx = 0; b_idx < NColsRemain; b_idx++) {
      obj->workspace2_ = obj->workspace_;
    }

    lda = obj->ldm;
    y = obj->workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      c = (k - 1) * obj->ldm + 1;
      for (b_idx = 1; lda < 0 ? b_idx >= c : b_idx <= c; b_idx += lda) {
        lastColC = b_idx + NColsRemain;
        for (idx = b_idx; idx < lastColC; idx++) {
          y += -obj->FMat.data[ix - 1] * obj->workspace_;
        }

        ix += obj->ldm;
      }
    }

    obj->workspace2_ = y;
    for (b_idx = 0; b_idx < NColsRemain; b_idx++) {
      obj->workspace_ = y;
    }

    for (b_idx = 0; b_idx <= subRows; b_idx++) {
      obj->FMat.data[LD_diagOffset + b_idx] = obj->workspace_;
    }

    for (b_idx = 0; b_idx < subRows; b_idx++) {
      ix = (b_idx + LD_diagOffset) + 1;
      obj->FMat.data[ix] /= obj->FMat.data[LD_diagOffset];
    }
  }

  for (int32_T k = 48; k <= NColsRemain - 1; k += 48) {
    int32_T b_idx_tmp;
    int32_T br;
    br = NColsRemain - k;
    if (br >= 48) {
      subRows = 48;
    } else {
      subRows = br;
    }

    b_idx_tmp = k + subRows;
    for (LD_diagOffset = k; LD_diagOffset < b_idx_tmp; LD_diagOffset++) {
      idx = b_idx_tmp - LD_diagOffset;
      for (lda = 0; lda < 48; lda++) {
        obj->workspace2_ = obj->FMat.data[((LD_offset + LD_diagOffset) + lda *
          obj->ldm) - 1];
      }

      ix = obj->ldm;
      if (idx != 0) {
        c = (obj->ldm * 47 + LD_diagOffset) + 1;
        for (b_idx = LD_diagOffset + 1; ix < 0 ? b_idx >= c : b_idx <= c; b_idx +=
             ix) {
          lastColC = b_idx + idx;
          for (lda = b_idx; lda < lastColC; lda++) {
            // Check node always fails. would cause program termination and was eliminated 
          }
        }
      }
    }

    if (b_idx_tmp < NColsRemain) {
      LD_diagOffset = br - subRows;
      b_idx = ((LD_offset + subRows) + LDimSizeP1 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        ix = (LD_offset + k) + idx * obj->ldm;
        for (lda = 0; lda < subRows; lda++) {
          obj->workspace2_ = obj->FMat.data[(ix + lda) - 1];
        }
      }

      idx = obj->ldm;
      if ((LD_diagOffset != 0) && (subRows != 0)) {
        lastColC = (subRows - 1) * obj->ldm + b_idx;
        br = 0;
        for (subRows = b_idx; idx < 0 ? subRows >= lastColC : subRows <=
             lastColC; subRows += idx) {
          br++;
          b_idx_tmp = idx * 47 + br;
          for (ix = br; idx < 0 ? ix >= b_idx_tmp : ix <= b_idx_tmp; ix += idx)
          {
            lda = subRows + LD_diagOffset;
            for (c = subRows + 1; c <= lda; c++) {
              obj->FMat.data[c - 1] += -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factor_i
  (s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, const real_T A[5625], int32_T
   ndims, int32_T ldA)
{
  real_T s;
  real_T smax;
  int32_T A_maxDiag_idx;
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T exitg2;
  int32_T ix;
  int32_T iy0;
  boolean_T exitg1;
  LDimSizeP1 = obj->ldm + 1;
  obj->ndims = ndims;
  for (A_maxDiag_idx = 0; A_maxDiag_idx < ndims; A_maxDiag_idx++) {
    ix = ldA * A_maxDiag_idx;
    iy0 = obj->ldm * A_maxDiag_idx;
    for (LD_diagOffset = 0; LD_diagOffset < ndims; LD_diagOffset++) {
      obj->FMat.data[iy0 + LD_diagOffset] = A[LD_diagOffset + ix];
    }
  }

  if (ndims < 1) {
    A_maxDiag_idx = -1;
  } else {
    A_maxDiag_idx = 0;
    if (ndims > 1) {
      smax = std::abs(obj->FMat.data[0]);
      for (LD_diagOffset = 2; LD_diagOffset <= ndims; LD_diagOffset++) {
        s = std::abs(obj->FMat.data[(LD_diagOffset - 1) * LDimSizeP1]);
        if (s > smax) {
          A_maxDiag_idx = LD_diagOffset - 1;
          smax = s;
        }
      }
    }
  }

  smax = std::abs(obj->FMat.data[obj->ldm * A_maxDiag_idx + A_maxDiag_idx]) *
    2.2204460492503131E-16;
  if (smax >= 0.0) {
    obj->regTol_ = smax;
  } else {
    obj->regTol_ = 0.0;
  }

  if (ndims > 128) {
    A_maxDiag_idx = 0;
    exitg1 = false;
    while ((!exitg1) && (A_maxDiag_idx < ndims)) {
      LD_diagOffset = LDimSizeP1 * A_maxDiag_idx + 1;
      ix = ndims - A_maxDiag_idx;
      if (A_maxDiag_idx + 48 <= ndims) {
        AEBController_partialColLDL3_(obj, LD_diagOffset, ix);
        A_maxDiag_idx += 48;
      } else {
        AEBController_fullColLDL2_(obj, LD_diagOffset, ix);
        exitg1 = true;
      }
    }
  } else {
    AEBController_fullColLDL2_(obj, 1, ndims);
  }

  if (obj->ConvexCheck) {
    LDimSizeP1 = 0;
    do {
      exitg2 = 0;
      if (LDimSizeP1 <= ndims - 1) {
        if (obj->FMat.data[obj->ldm * LDimSizeP1 + LDimSizeP1] <= 0.0) {
          obj->info = -LDimSizeP1 - 1;
          exitg2 = 1;
        } else {
          LDimSizeP1++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
int32_T ACCWithSensorFusionModelClass::AEBController_xpotrf(int32_T n, real_T
  A_data[], int32_T lda)
{
  int32_T b_j;
  int32_T info;
  boolean_T exitg1;
  info = 0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j <= n - 1)) {
    real_T c;
    real_T ssq;
    int32_T idxA1j;
    int32_T idxAjj;
    int32_T nmj;
    idxA1j = b_j * lda;
    idxAjj = idxA1j + b_j;
    ssq = 0.0;
    if (b_j >= 1) {
      for (nmj = 0; nmj < b_j; nmj++) {
        c = A_data[idxA1j + nmj];
        ssq += c * c;
      }
    }

    ssq = A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A_data[idxAjj] = ssq;
      if (b_j + 1 < n) {
        int32_T b_ix;
        nmj = (n - b_j) - 2;
        b_ix = (idxA1j + lda) + 1;
        idxAjj += lda;
        if ((b_j != 0) && (nmj + 1 != 0)) {
          int32_T b;
          int32_T iy;
          iy = idxAjj;
          b = lda * nmj + b_ix;
          for (int32_T b_iy = b_ix; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda)
          {
            int32_T d;
            c = 0.0;
            d = (b_iy + b_j) - 1;
            for (int32_T ia = b_iy; ia <= d; ia++) {
              c += A_data[(idxA1j + ia) - b_iy] * A_data[ia - 1];
            }

            A_data[iy] -= c;
            iy += lda;
          }
        }

        ssq = 1.0 / ssq;
        idxA1j = (lda * nmj + idxAjj) + 1;
        for (nmj = idxAjj + 1; lda < 0 ? nmj >= idxA1j : nmj <= idxA1j; nmj +=
             lda) {
          A_data[nmj - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      A_data[idxAjj] = ssq;
      info = b_j + 1;
      exitg1 = true;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factor
  (s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, const real_T A[5625], int32_T
   ndims, int32_T ldA)
{
  int32_T b_k;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  obj->ndims = ndims;
  for (idx = 0; idx < ndims; idx++) {
    ix0 = ldA * idx;
    iy0 = obj->ldm * idx;
    for (b_k = 0; b_k < ndims; b_k++) {
      obj->FMat.data[iy0 + b_k] = A[b_k + ix0];
    }
  }

  obj->info = AEBController_xpotrf(ndims, obj->FMat.data, obj->ldm);
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemm(int32_T m, int32_T n,
  int32_T k, const real_T A[5625], int32_T lda, const real_T B_data[], int32_T
  ib0, int32_T ldb, real_T C_data[], int32_T ldc)
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T br;
    int32_T lastColC;
    br = ib0;
    lastColC = (n - 1) * ldc;
    for (int32_T cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      b = cr + m;
      for (int32_T ic = cr + 1; ic <= b; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    for (int32_T cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = -1;
      c = br + k;
      for (int32_T ic = br; ic < c; ic++) {
        int32_T d;
        d = cr + m;
        for (b = cr + 1; b <= d; b++) {
          C_data[b - 1] += A[(ar + b) - cr] * B_data[ic - 1];
        }

        ar += lda;
      }

      br += ldb;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemm_p(int32_T m, int32_T n,
  int32_T k, const real_T A_data[], int32_T ia0, int32_T lda, const real_T
  B_data[], int32_T ldb, real_T C_data[], int32_T ldc)
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T br;
    int32_T lastColC;
    lastColC = (n - 1) * ldc;
    for (int32_T cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      b = cr + m;
      for (int32_T ic = cr + 1; ic <= b; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    br = -1;
    for (int32_T cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = ia0;
      c = cr + m;
      for (int32_T ic = cr + 1; ic <= c; ic++) {
        real_T temp;
        temp = 0.0;
        for (b = 0; b < k; b++) {
          temp += A_data[(b + ar) - 1] * B_data[(b + br) + 1];
        }

        C_data[ic - 1] += temp;
        ar += lda;
      }

      br += ldb;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_jtygq(int32_T m, int32_T
  n, const real_T A_data[], int32_T ia0, int32_T lda, const real_T x_data[],
  real_T y_data[])
{
  if (m != 0) {
    int32_T b;
    int32_T ix;
    for (int32_T b_iy = 0; b_iy < m; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    ix = 0;
    b = (n - 1) * lda + ia0;
    for (int32_T b_iy = ia0; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (int32_T ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A_data[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_solve(const
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, real_T rhs_data[])
{
  int32_T n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    int32_T jA;
    for (int32_T j = 0; j < n_tmp; j++) {
      real_T temp;
      jA = j * obj->ldm;
      temp = rhs_data[j];
      for (int32_T i = 0; i < j; i++) {
        temp -= obj->FMat.data[jA + i] * rhs_data[i];
      }

      rhs_data[j] = temp / obj->FMat.data[jA + j];
    }

    for (int32_T j = n_tmp; j >= 1; j--) {
      jA = ((j - 1) * obj->ldm + j) - 2;
      rhs_data[j - 1] /= obj->FMat.data[jA + 1];
      for (int32_T i = 0; i <= j - 2; i++) {
        int32_T ix;
        ix = (j - i) - 2;
        rhs_data[ix] -= obj->FMat.data[jA - i] * rhs_data[j - 1];
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_solve_j(const
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *obj, real_T rhs_data[])
{
  int32_T jjA;
  int32_T n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (int32_T b_j = 0; b_j < n_tmp; b_j++) {
      int32_T c;
      jjA = b_j * obj->ldm + b_j;
      c = (n_tmp - b_j) - 2;
      for (int32_T b_i = 0; b_i <= c; b_i++) {
        int32_T ix;
        ix = (b_i + b_j) + 1;
        rhs_data[ix] -= obj->FMat.data[(b_i + jjA) + 1] * rhs_data[b_j];
      }
    }
  }

  for (int32_T b_j = 0; b_j < n_tmp; b_j++) {
    rhs_data[b_j] /= obj->FMat.data[obj->ldm * b_j + b_j];
  }

  if (obj->ndims != 0) {
    for (int32_T b_j = n_tmp; b_j >= 1; b_j--) {
      real_T temp;
      jjA = (b_j - 1) * obj->ldm;
      temp = rhs_data[b_j - 1];
      for (int32_T b_i = n_tmp; b_i >= b_j + 1; b_i--) {
        temp -= obj->FMat.data[(jjA + b_i) - 1] * rhs_data[b_i - 1];
      }

      rhs_data[b_j - 1] = temp;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_compute_deltax(const real_T H
  [5625], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager, const
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, boolean_T
  alwaysPositiveDef)
{
  real_T s;
  real_T smax;
  int32_T b_jjA;
  int32_T b_mNull;
  int32_T c_mNull;
  int32_T d_ix;
  int32_T exitg2;
  int32_T mNull_tmp;
  int32_T nVar;
  int32_T nullStart;
  int32_T nullStartIdx;
  int32_T nullStartIdx_tmp;
  boolean_T exitg1;
  nVar = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    for (c_mNull = 0; c_mNull <= nVar; c_mNull++) {
      solution->searchDir.data[c_mNull] = 0.0;
    }
  } else {
    for (nullStartIdx = 0; nullStartIdx <= nVar; nullStartIdx++) {
      solution->searchDir.data[nullStartIdx] = -objective->
        grad.data[nullStartIdx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        if (alwaysPositiveDef) {
          AEBController_factor(cholmanager, H, qrmanager->mrows,
                               qrmanager->mrows);
        } else {
          AEBController_factor_i(cholmanager, H, qrmanager->mrows,
            qrmanager->mrows);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          AEBController_solve(cholmanager, solution->searchDir.data);
        } else {
          AEBController_solve_j(cholmanager, solution->searchDir.data);
        }
        break;

       case 4:
        if (alwaysPositiveDef) {
          AEBController_factor(cholmanager, H, objective->nvar, objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            AEBController_solve(cholmanager, solution->searchDir.data);
            smax = 1.0 / objective->beta;
            nullStartIdx = objective->nvar + 1;
            b_mNull = qrmanager->mrows;
            for (c_mNull = nullStartIdx; c_mNull <= b_mNull; c_mNull++) {
              solution->searchDir.data[c_mNull - 1] *= smax;
            }
          }
        }
        break;
      }
    } else {
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (nullStart = 0; nullStart < mNull_tmp; nullStart++) {
          memspace->workspace_float.data[nullStart] = -qrmanager->Q.data
            [(qrmanager->ncols + nullStart) * qrmanager->ldq + nVar];
        }

        AEBController_xgemv_jtygq(qrmanager->mrows, mNull_tmp, qrmanager->Q.data,
          nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_float.data,
          solution->searchDir.data);
      } else {
        if (objective->objtype == 3) {
          AEBController_xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                              qrmanager->mrows, qrmanager->Q.data,
                              nullStartIdx_tmp, qrmanager->ldq,
                              memspace->workspace_float.data,
                              memspace->workspace_float.size[0]);
          AEBController_xgemm_p(mNull_tmp, mNull_tmp, qrmanager->mrows,
                                qrmanager->Q.data, nullStartIdx_tmp,
                                qrmanager->ldq, memspace->workspace_float.data,
                                memspace->workspace_float.size[0],
                                cholmanager->FMat.data, cholmanager->ldm);
        } else if (alwaysPositiveDef) {
          nullStartIdx = qrmanager->mrows;
          AEBController_xgemm(objective->nvar, mNull_tmp, objective->nvar, H,
                              objective->nvar, qrmanager->Q.data,
                              nullStartIdx_tmp, qrmanager->ldq,
                              memspace->workspace_float.data,
                              memspace->workspace_float.size[0]);
          for (b_mNull = 0; b_mNull < mNull_tmp; b_mNull++) {
            b_jjA = objective->nvar + 1;
            for (nullStart = b_jjA; nullStart <= nullStartIdx; nullStart++) {
              memspace->workspace_float.data[(nullStart +
                memspace->workspace_float.size[0] * b_mNull) - 1] =
                qrmanager->Q.data[((b_mNull + qrmanager->ncols) *
                                   qrmanager->Q.size[0] + nullStart) - 1] *
                objective->beta;
            }
          }

          AEBController_xgemm_p(mNull_tmp, mNull_tmp, qrmanager->mrows,
                                qrmanager->Q.data, nullStartIdx_tmp,
                                qrmanager->ldq, memspace->workspace_float.data,
                                memspace->workspace_float.size[0],
                                cholmanager->FMat.data, cholmanager->ldm);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = AEBController_xpotrf(mNull_tmp,
            cholmanager->FMat.data, cholmanager->ldm);
        } else {
          nullStart = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          nullStartIdx = 0;
          if (mNull_tmp > 1) {
            smax = std::abs(cholmanager->FMat.data[0]);
            for (b_mNull = 2; b_mNull <= mNull_tmp; b_mNull++) {
              s = std::abs(cholmanager->FMat.data[(b_mNull - 1) * nullStart]);
              if (s > smax) {
                nullStartIdx = b_mNull - 1;
                smax = s;
              }
            }
          }

          smax = std::abs(cholmanager->FMat.data[cholmanager->ldm * nullStartIdx
                          + nullStartIdx]) * 2.2204460492503131E-16;
          if (smax >= 0.0) {
            cholmanager->regTol_ = smax;
          } else {
            cholmanager->regTol_ = 0.0;
          }

          if (mNull_tmp > 128) {
            b_mNull = 0;
            exitg1 = false;
            while ((!exitg1) && (b_mNull < mNull_tmp)) {
              nullStartIdx = nullStart * b_mNull + 1;
              c_mNull = mNull_tmp - b_mNull;
              if (b_mNull + 48 <= mNull_tmp) {
                AEBController_partialColLDL3_(cholmanager, nullStartIdx, c_mNull);
                b_mNull += 48;
              } else {
                AEBController_fullColLDL2_(cholmanager, nullStartIdx, c_mNull);
                exitg1 = true;
              }
            }
          } else {
            AEBController_fullColLDL2_(cholmanager, 1, mNull_tmp);
          }

          if (cholmanager->ConvexCheck) {
            b_mNull = 0;
            do {
              exitg2 = 0;
              if (b_mNull <= mNull_tmp - 1) {
                if (cholmanager->FMat.data[cholmanager->ldm * b_mNull + b_mNull]
                    <= 0.0) {
                  cholmanager->info = -b_mNull - 1;
                  exitg2 = 1;
                } else {
                  b_mNull++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          c_mNull = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            for (b_mNull = 0; b_mNull < mNull_tmp; b_mNull++) {
              memspace->workspace_float.data[b_mNull] = 0.0;
            }

            b_mNull = 0;
            b_jjA = (mNull_tmp - 1) * qrmanager->ldq + nullStartIdx_tmp;
            for (nullStart = nullStartIdx_tmp; c_mNull < 0 ? nullStart >= b_jjA :
                 nullStart <= b_jjA; nullStart += c_mNull) {
              smax = 0.0;
              d_ix = nullStart + nVar;
              for (nullStartIdx = nullStart; nullStartIdx <= d_ix; nullStartIdx
                   ++) {
                smax += qrmanager->Q.data[nullStartIdx - 1] *
                  objective->grad.data[nullStartIdx - nullStart];
              }

              memspace->workspace_float.data[b_mNull] -= smax;
              b_mNull++;
            }
          }

          if (alwaysPositiveDef) {
            nVar = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nullStart = 0; nullStart < nVar; nullStart++) {
                b_mNull = nullStart * cholmanager->ldm;
                smax = memspace->workspace_float.data[nullStart];
                for (nullStartIdx = 0; nullStartIdx < nullStart; nullStartIdx++)
                {
                  smax -= cholmanager->FMat.data[b_mNull + nullStartIdx] *
                    memspace->workspace_float.data[nullStartIdx];
                }

                memspace->workspace_float.data[nullStart] = smax /
                  cholmanager->FMat.data[b_mNull + nullStart];
              }
            }

            if (cholmanager->ndims != 0) {
              for (nullStartIdx = nVar; nullStartIdx >= 1; nullStartIdx--) {
                b_jjA = ((nullStartIdx - 1) * cholmanager->ldm + nullStartIdx) -
                  2;
                memspace->workspace_float.data[nullStartIdx - 1] /=
                  cholmanager->FMat.data[b_jjA + 1];
                for (c_mNull = 0; c_mNull <= nullStartIdx - 2; c_mNull++) {
                  d_ix = (nullStartIdx - c_mNull) - 2;
                  memspace->workspace_float.data[d_ix] -=
                    memspace->workspace_float.data[nullStartIdx - 1] *
                    cholmanager->FMat.data[b_jjA - c_mNull];
                }
              }
            }
          } else {
            b_jjA = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nullStartIdx = 0; nullStartIdx < b_jjA; nullStartIdx++) {
                nullStart = nullStartIdx * cholmanager->ldm + nullStartIdx;
                d_ix = (b_jjA - nullStartIdx) - 2;
                for (c_mNull = 0; c_mNull <= d_ix; c_mNull++) {
                  nVar = (c_mNull + nullStartIdx) + 1;
                  memspace->workspace_float.data[nVar] -= cholmanager->
                    FMat.data[(c_mNull + nullStart) + 1] *
                    memspace->workspace_float.data[nullStartIdx];
                }
              }
            }

            for (nullStart = 0; nullStart < b_jjA; nullStart++) {
              memspace->workspace_float.data[nullStart] /=
                cholmanager->FMat.data[cholmanager->ldm * nullStart + nullStart];
            }

            if (cholmanager->ndims != 0) {
              for (nullStart = b_jjA; nullStart >= 1; nullStart--) {
                b_mNull = (nullStart - 1) * cholmanager->ldm;
                smax = memspace->workspace_float.data[nullStart - 1];
                for (nullStartIdx = b_jjA; nullStartIdx >= nullStart + 1;
                     nullStartIdx--) {
                  smax -= cholmanager->FMat.data[(b_mNull + nullStartIdx) - 1] *
                    memspace->workspace_float.data[nullStartIdx - 1];
                }

                memspace->workspace_float.data[nullStart - 1] = smax;
              }
            }
          }

          AEBController_xgemv_jtygq(qrmanager->mrows, mNull_tmp,
            qrmanager->Q.data, nullStartIdx_tmp, qrmanager->ldq,
            memspace->workspace_float.data, solution->searchDir.data);
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_xnrm2_pfp(int32_T n, const
  real_T x_data[])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[0]);
    } else {
      real_T scale;
      scale = 3.3121686421112381E-170;
      for (int32_T k = 0; k < n; k++) {
        real_T absxk;
        absxk = std::abs(x_data[k]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_jtygqg(int32_T m,
  int32_T n, const real_T A_data[], int32_T lda, const real_T x_data[], real_T
  y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    b = static_cast<uint8_T>(n);
    for (int32_T b_iy = 0; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    b = 0;
    d = (n - 1) * lda + 1;
    for (int32_T b_iy = 1; lda < 0 ? b_iy >= d : b_iy <= d; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (int32_T y_tmp = b_iy; y_tmp <= e; y_tmp++) {
        c += A_data[y_tmp - 1] * x_data[y_tmp - b_iy];
      }

      y_data[b] += c;
      b++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_feasibleratiotest(const real_T
  solution_xstar_data[], const real_T solution_searchDir_data[], real_T
  workspace_data[], const int32_T workspace_size[2], int32_T workingset_nVar,
  int32_T workingset_ldA, const real_T workingset_Aineq_data[], const real_T
  workingset_bineq_data[], const real_T workingset_lb_data[], const int32_T
  workingset_indexLB_data[], const int32_T workingset_sizes[5], const int32_T
  workingset_isActiveIdx[6], const boolean_T workingset_isActiveConstr_data[],
  const int32_T workingset_nWConstr[5], boolean_T isPhaseOne, real_T *alpha,
  boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx)
{
  real_T alphaTemp;
  real_T b_c;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T ratio;
  real_T u0;
  int32_T d;
  int32_T e;
  int32_T f;
  int32_T ia;
  int32_T iyend;
  int32_T ldw;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * AEBController_xnrm2_pfp(workingset_nVar,
    solution_searchDir_data);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    d = static_cast<uint16_T>(workingset_sizes[2]);
    if (d - 1 >= 0) {
      std::memcpy(&workspace_data[0], &workingset_bineq_data[0],
                  static_cast<uint32_T>(d) * sizeof(real_T));
    }

    AEBController_xgemv_jtygqg(workingset_nVar, workingset_sizes[2],
      workingset_Aineq_data, workingset_ldA, solution_xstar_data, workspace_data);
    ldw = workspace_size[0];
    if (workingset_sizes[2] != 0) {
      iyend = workspace_size[0] + workingset_sizes[2];
      for (d = ldw + 1; d <= iyend; d++) {
        workspace_data[d - 1] = 0.0;
      }

      iyend = workspace_size[0];
      e = (workingset_sizes[2] - 1) * workingset_ldA + 1;
      for (d = 1; workingset_ldA < 0 ? d >= e : d <= e; d += workingset_ldA) {
        b_c = 0.0;
        f = (d + workingset_nVar) - 1;
        for (ia = d; ia <= f; ia++) {
          b_c += workingset_Aineq_data[ia - 1] * solution_searchDir_data[ia - d];
        }

        workspace_data[iyend] += b_c;
        iyend++;
      }
    }

    iyend = static_cast<uint8_T>(workingset_sizes[2]);
    for (d = 0; d < iyend; d++) {
      b_c = workspace_data[ldw + d];
      if ((b_c > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[2] + d) - 1])) {
        u0 = std::abs(workspace_data[d]);
        ratio = 1.0E-6 - workspace_data[d];
        if ((u0 <= ratio) || rtIsNaN(ratio)) {
          ratio = u0;
        }

        alphaTemp = ratio / b_c;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = d + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    b_c = solution_xstar_data[workingset_nVar - 1] * static_cast<real_T>
      (isPhaseOne);
    phaseOneCorrectionP = solution_searchDir_data[workingset_nVar - 1] *
      static_cast<real_T>(isPhaseOne);
    d = workingset_sizes[3];
    for (ldw = 0; ldw <= d - 2; ldw++) {
      iyend = workingset_indexLB_data[ldw];
      alphaTemp = -solution_searchDir_data[iyend - 1] - phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[3] + ldw) - 1])) {
        ratio = (-solution_xstar_data[iyend - 1] - workingset_lb_data[iyend - 1])
          - b_c;
        u0 = std::abs(ratio);
        if ((!(u0 <= 1.0E-6 - ratio)) && (!rtIsNaN(1.0E-6 - ratio))) {
          u0 = 1.0E-6 - ratio;
        }

        alphaTemp = u0 / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = ldw + 1;
          *newBlocking = true;
        }
      }
    }

    e = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    b_c = -solution_searchDir_data[e];
    if ((b_c > denomTol) && (!workingset_isActiveConstr_data
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar_data[e] - workingset_lb_data[e];
      u0 = std::abs(ratio);
      if ((!(u0 <= 1.0E-6 - ratio)) && (!rtIsNaN(1.0E-6 - ratio))) {
        u0 = 1.0E-6 - ratio;
      }

      alphaTemp = u0 / b_c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (!isPhaseOne) {
    *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
    if (!(*alpha <= 1.0)) {
      *alpha = 1.0;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_compute_lambda(real_T
  workspace_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution, const
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager)
{
  int32_T nActiveConstr_tmp_tmp;
  nActiveConstr_tmp_tmp = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    real_T c;
    int32_T b;
    int32_T b_idx;
    int32_T idxQR;
    boolean_T guard1;
    guard1 = false;
    if (objective->objtype != 4) {
      boolean_T nonDegenerate;
      if (qrmanager->mrows >= qrmanager->ncols) {
        b = qrmanager->mrows;
      } else {
        b = qrmanager->ncols;
      }

      c = 2.2204460492503131E-15 * static_cast<real_T>(b);
      if (c >= 1.4901161193847656E-8) {
        c = 1.4901161193847656E-8;
      }

      nonDegenerate = ((qrmanager->mrows > 0) && (qrmanager->ncols > 0));
      if (nonDegenerate) {
        boolean_T guard2;
        b_idx = qrmanager->ncols;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = (qrmanager->ncols - 1) * qrmanager->ldq + qrmanager->mrows;
          while ((b_idx > qrmanager->mrows) && (std::abs(qrmanager->
                   QR.data[idxQR - 1]) >= c)) {
            b_idx--;
            idxQR -= qrmanager->ldq;
          }

          nonDegenerate = (b_idx == qrmanager->mrows);
          if (!nonDegenerate) {
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          idxQR = (b_idx - 1) * qrmanager->ldq + b_idx;
          while ((b_idx >= 1) && (std::abs(qrmanager->QR.data[idxQR - 1]) >= c))
          {
            b_idx--;
            idxQR = (idxQR - qrmanager->ldq) - 1;
          }

          nonDegenerate = (b_idx == 0);
        }
      }

      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      int32_T b_ix;
      int32_T jjA;
      b_idx = qrmanager->ldq;
      if (qrmanager->mrows != 0) {
        for (idxQR = 0; idxQR < nActiveConstr_tmp_tmp; idxQR++) {
          workspace_data[idxQR] = 0.0;
        }

        jjA = 0;
        b = (qrmanager->ncols - 1) * qrmanager->ldq + 1;
        for (idxQR = 1; b_idx < 0 ? idxQR >= b : idxQR <= b; idxQR += b_idx) {
          c = 0.0;
          b_ix = (idxQR + qrmanager->mrows) - 1;
          for (int32_T ia = idxQR; ia <= b_ix; ia++) {
            c += qrmanager->Q.data[ia - 1] * objective->grad.data[ia - idxQR];
          }

          workspace_data[jjA] += c;
          jjA++;
        }
      }

      if (qrmanager->ncols != 0) {
        for (idxQR = nActiveConstr_tmp_tmp; idxQR >= 1; idxQR--) {
          jjA = ((idxQR - 1) * b_idx + idxQR) - 2;
          workspace_data[idxQR - 1] /= qrmanager->QR.data[jjA + 1];
          for (int32_T ia = 0; ia <= idxQR - 2; ia++) {
            b_ix = (idxQR - ia) - 2;
            workspace_data[b_ix] -= workspace_data[idxQR - 1] *
              qrmanager->QR.data[jjA - ia];
          }
        }
      }

      for (b_idx = 0; b_idx < nActiveConstr_tmp_tmp; b_idx++) {
        solution->lambda.data[b_idx] = -workspace_data[b_idx];
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xorgqr(int32_T m, int32_T n,
  int32_T k, real_T A_data[], const int32_T A_size[2], int32_T lda, const real_T
  tau_data[])
{
  int32_T c;
  int32_T i;
  int32_T ia;
  int32_T itau;
  int32_T loop_ub;
  if (n >= 1) {
    for (itau = k; itau < n; itau++) {
      ia = itau * lda;
      for (i = 0; i < m; i++) {
        A_data[ia + i] = 0.0;
      }

      A_data[ia + itau] = 1.0;
    }

    itau = k - 1;
    loop_ub = A_size[1];
    for (i = 0; i < loop_ub; i++) {
      AEBController_DW.work_data_k[i] = 0.0;
    }

    for (i = k; i >= 1; i--) {
      ia = (i - 1) * lda + i;
      if (i < n) {
        A_data[ia - 1] = 1.0;
        AEBController_xzlarf((m - i) + 1, n - i, ia, tau_data[itau], A_data, ia
                             + lda, lda, AEBController_DW.work_data_k);
      }

      if (i < m) {
        c = (ia + m) - i;
        for (loop_ub = ia + 1; loop_ub <= c; loop_ub++) {
          A_data[loop_ub - 1] *= -tau_data[itau];
        }
      }

      A_data[ia - 1] = 1.0 - tau_data[itau];
      c = static_cast<uint16_T>(i - 1);
      for (loop_ub = 0; loop_ub < c; loop_ub++) {
        A_data[(ia - loop_ub) - 2] = 0.0;
      }

      itau--;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_checkUnboundedOrIllPosed
  (s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution, const
   s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective)
{
  if (objective->objtype == 5) {
    if (AEBController_xnrm2_pfp(objective->nvar, solution->searchDir.data) >
        100.0 * static_cast<real_T>(objective->nvar) * 1.4901161193847656E-8) {
      solution->state = 3;
    } else {
      solution->state = 4;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_addBoundToActiveSetMatrix_
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T TYPE, int32_T
   idx_local)
{
  int32_T b;
  int32_T colOffset;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  colOffset = (obj->nActiveConstr - 1) * obj->ldA - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->ub.data[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->lb.data[idx_bnd_local - 1];
  }

  b = static_cast<uint16_T>(idx_bnd_local - 1);
  for (int32_T idx = 0; idx < b; idx++) {
    obj->ATwset.data[(idx + colOffset) + 1] = 0.0;
  }

  obj->ATwset.data[idx_bnd_local + colOffset] = static_cast<real_T>(TYPE == 5) *
    2.0 - 1.0;
  b = obj->nVar;
  for (int32_T idx = idx_bnd_local + 1; idx <= b; idx++) {
    obj->ATwset.data[idx + colOffset] = 0.0;
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset.data[obj->nVar + colOffset] = -1.0;
    break;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_addAineqConstr
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T idx_local)
{
  int32_T b;
  int32_T iAineq0;
  int32_T iAw0;
  obj->nWConstr[2]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = 3;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = (idx_local - 1) * obj->ldA;
  iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
  b = obj->nVar;
  for (int32_T idx = 0; idx < b; idx++) {
    obj->ATwset.data[iAw0 + idx] = obj->Aineq.data[iAineq0 + idx];
  }

  obj->bwset.data[obj->nActiveConstr - 1] = obj->bineq.data[idx_local - 1];
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_removeConstr
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T idx_global)
{
  int32_T TYPE_tmp;
  TYPE_tmp = obj->Wid.data[idx_global - 1] - 1;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE_tmp] + obj->
    Wlocalidx.data[idx_global - 1]) - 2] = false;
  if (idx_global < obj->nActiveConstr) {
    int32_T b;
    obj->Wid.data[idx_global - 1] = obj->Wid.data[obj->nActiveConstr - 1];
    obj->Wlocalidx.data[idx_global - 1] = obj->Wlocalidx.data[obj->nActiveConstr
      - 1];
    b = static_cast<uint16_T>(obj->nVar);
    for (int32_T idx = 0; idx < b; idx++) {
      obj->ATwset.data[idx + obj->ldA * (idx_global - 1)] = obj->ATwset.data
        [(obj->nActiveConstr - 1) * obj->ldA + idx];
    }

    obj->bwset.data[idx_global - 1] = obj->bwset.data[obj->nActiveConstr - 1];
  }

  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xswap(int32_T n, real_T
  x_data[], int32_T ix0, int32_T iy0)
{
  for (int32_T k = 0; k < n; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x_data[temp_tmp];
    tmp = (iy0 + k) - 1;
    x_data[temp_tmp] = x_data[tmp];
    x_data[tmp] = temp;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_qrpf(real_T A_data[], const
  int32_T A_size[2], int32_T m, int32_T n, int32_T nfxd, real_T tau_data[],
  int32_T jpvt_data[])
{
  real_T s;
  real_T smax;
  real_T vn1;
  int32_T ii;
  int32_T ii_tmp;
  int32_T itemp;
  int32_T j;
  int32_T ma;
  int32_T minmn;
  int32_T mmi;
  int32_T nmi;
  int32_T pvt;
  ma = A_size[0];
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  pvt = A_size[1];
  for (j = 0; j < pvt; j++) {
    AEBController_DW.work_data_c[j] = 0.0;
    AEBController_DW.vn1_data[j] = 0.0;
    AEBController_DW.vn2_data[j] = 0.0;
  }

  for (j = nfxd + 1; j <= n; j++) {
    vn1 = AEBController_xnrm2_pf(m - nfxd, A_data, ((j - 1) * ma + nfxd) + 1);
    AEBController_DW.vn1_data[j - 1] = vn1;
    AEBController_DW.vn2_data[j - 1] = vn1;
  }

  for (j = nfxd + 1; j <= minmn; j++) {
    ii_tmp = (j - 1) * ma;
    ii = (ii_tmp + j) - 1;
    nmi = n - j;
    mmi = m - j;
    if (nmi + 1 < 1) {
      itemp = -2;
    } else {
      itemp = -1;
      if (nmi + 1 > 1) {
        smax = std::abs(AEBController_DW.vn1_data[j - 1]);
        for (pvt = 2; pvt <= nmi + 1; pvt++) {
          s = std::abs(AEBController_DW.vn1_data[(j + pvt) - 2]);
          if (s > smax) {
            itemp = pvt - 2;
            smax = s;
          }
        }
      }
    }

    pvt = j + itemp;
    if (pvt + 1 != j) {
      AEBController_xswap(m, A_data, pvt * ma + 1, ii_tmp + 1);
      itemp = jpvt_data[pvt];
      jpvt_data[pvt] = jpvt_data[j - 1];
      jpvt_data[j - 1] = itemp;
      AEBController_DW.vn1_data[pvt] = AEBController_DW.vn1_data[j - 1];
      AEBController_DW.vn2_data[pvt] = AEBController_DW.vn2_data[j - 1];
    }

    if (j < m) {
      smax = A_data[ii];
      vn1 = AEBController_xzlarfg(mmi + 1, &smax, A_data, ii + 2);
      tau_data[j - 1] = vn1;
      A_data[ii] = smax;
    } else {
      vn1 = 0.0;
      tau_data[j - 1] = 0.0;
    }

    if (j < n) {
      smax = A_data[ii];
      A_data[ii] = 1.0;
      AEBController_xzlarf(mmi + 1, nmi, ii + 1, vn1, A_data, (ii + ma) + 1, ma,
                           AEBController_DW.work_data_c);
      A_data[ii] = smax;
    }

    for (pvt = j + 1; pvt <= n; pvt++) {
      ii = (pvt - 1) * ma + j;
      vn1 = AEBController_DW.vn1_data[pvt - 1];
      if (vn1 != 0.0) {
        smax = std::abs(A_data[ii - 1]) / vn1;
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        s = vn1 / AEBController_DW.vn2_data[pvt - 1];
        s = s * s * smax;
        if (s <= 1.4901161193847656E-8) {
          if (j < m) {
            vn1 = AEBController_xnrm2_pf(mmi, A_data, ii + 1);
            AEBController_DW.vn1_data[pvt - 1] = vn1;
            AEBController_DW.vn2_data[pvt - 1] = vn1;
          } else {
            AEBController_DW.vn1_data[pvt - 1] = 0.0;
            AEBController_DW.vn2_data[pvt - 1] = 0.0;
          }
        } else {
          AEBController_DW.vn1_data[pvt - 1] = vn1 * std::sqrt(smax);
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgeqp3(real_T A_data[], const
  int32_T A_size[2], int32_T m, int32_T n, int32_T jpvt_data[], real_T tau_data[],
  int32_T tau_size[1])
{
  int32_T b_j;
  int32_T ma_tmp;
  int32_T minmn;
  int32_T nfxd;
  ma_tmp = A_size[0];
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  if (A_size[0] <= A_size[1]) {
    nfxd = A_size[0];
  } else {
    nfxd = A_size[1];
  }

  tau_size[0] = nfxd;
  for (b_j = 0; b_j < nfxd; b_j++) {
    tau_data[b_j] = 0.0;
  }

  if (minmn < 1) {
    for (b_j = 0; b_j < n; b_j++) {
      jpvt_data[b_j] = b_j + 1;
    }
  } else {
    nfxd = -1;
    for (b_j = 0; b_j < n; b_j++) {
      if (jpvt_data[b_j] != 0) {
        nfxd++;
        if (b_j + 1 != nfxd + 1) {
          AEBController_xswap(m, A_data, b_j * ma_tmp + 1, nfxd * ma_tmp + 1);
          jpvt_data[b_j] = jpvt_data[nfxd];
          jpvt_data[nfxd] = b_j + 1;
        } else {
          jpvt_data[b_j] = b_j + 1;
        }
      } else {
        jpvt_data[b_j] = b_j + 1;
      }
    }

    if (nfxd + 1 <= minmn) {
      nfxd++;
    } else {
      nfxd = minmn;
    }

    AEBController_qrf(A_data, A_size, m, n, nfxd, tau_data);
    if (nfxd < minmn) {
      AEBController_qrpf(A_data, A_size, m, n, nfxd, tau_data, jpvt_data);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factorQRE
  (s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj, const real_T A_data[], int32_T
   mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = static_cast<uint16_T>(mrows);
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    AEBController_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols,
                         obj->jpvt.data, obj->tau.data, obj->tau.size);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_factorQRE_i
  (s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *obj, int32_T mrows, int32_T ncols)
{
  if (mrows * ncols == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    AEBController_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols,
                         obj->jpvt.data, obj->tau.data, obj->tau.size);
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
int32_T ACCWithSensorFusionModelClass::AEBController_rank(const real_T
  qrmanager_QR_data[], const int32_T qrmanager_QR_size[2], int32_T
  qrmanager_mrows, int32_T qrmanager_ncols)
{
  int32_T minmn;
  int32_T r;
  r = 0;
  if (qrmanager_mrows <= qrmanager_ncols) {
    minmn = qrmanager_mrows;
  } else {
    minmn = qrmanager_ncols;
  }

  if (minmn > 0) {
    real_T tol;
    int32_T tmp;
    if (qrmanager_mrows >= qrmanager_ncols) {
      tmp = qrmanager_mrows;
    } else {
      tmp = qrmanager_ncols;
    }

    tol = 2.2204460492503131E-15 * static_cast<real_T>(tmp);
    if (tol >= 1.4901161193847656E-8) {
      tol = 1.4901161193847656E-8;
    }

    tol *= std::abs(qrmanager_QR_data[0]);
    while ((r < minmn) && (!(std::abs(qrmanager_QR_data[qrmanager_QR_size[0] * r
              + r]) <= tol))) {
      r++;
    }
  }

  return r;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_j(int32_T m, int32_T n,
  const real_T A_data[], int32_T lda, const real_T x_data[], real_T y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    int32_T iy;
    b = static_cast<uint8_T>(n);
    for (int32_T b_iy = 0; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    iy = 0;
    d = (n - 1) * lda + 1;
    for (int32_T b_iy = 1; lda < 0 ? b_iy >= d : b_iy <= d; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x_data[b - b_iy] * A_data[b - 1];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_maxConstraintViolation_d
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[])
{
  real_T u1;
  real_T v;
  int32_T f;
  int32_T k;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      AEBController_xgemv_j(75, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
                            obj->maxConstrWorkspace.data);
      f = static_cast<uint8_T>(obj->sizes[2]);
      for (k = 0; k < f; k++) {
        u1 = obj->maxConstrWorkspace.data[k] - x_data[k + 75];
        obj->maxConstrWorkspace.data[k] = u1;
        if ((!(v >= u1)) && (!rtIsNaN(u1))) {
          v = u1;
        }
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
                (real_T));
    AEBController_xgemv_j(75, 70, obj->Aeq.data, obj->ldA, x_data,
                          obj->maxConstrWorkspace.data);
    for (k = 0; k < 70; k++) {
      obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] -
        x_data[(mIneq + k) + 75]) + x_data[(obj->sizes[2] + k) + 145];
      u1 = std::abs(obj->maxConstrWorkspace.data[k]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      AEBController_xgemv_j(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
                            x_data, obj->maxConstrWorkspace.data);
      mIneq = static_cast<uint8_T>(obj->sizes[2]);
      for (k = 0; k < mIneq; k++) {
        u1 = obj->maxConstrWorkspace.data[k];
        if ((!(v >= u1)) && (!rtIsNaN(u1))) {
          v = u1;
        }
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
                (real_T));
    AEBController_xgemv_j(obj->nVar, 70, obj->Aeq.data, obj->ldA, x_data,
                          obj->maxConstrWorkspace.data);
    for (k = 0; k < 70; k++) {
      u1 = std::abs(obj->maxConstrWorkspace.data[k]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = -x_data[obj->indexLB.data[mIneq] - 1] - obj->lb.data
        [obj->indexLB.data[mIneq] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[4] > 0) {
    k = static_cast<uint16_T>(obj->sizes[4]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = x_data[obj->indexUB.data[mIneq] - 1] - obj->ub.data[obj->
        indexUB.data[mIneq] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[0] > 0) {
    k = static_cast<uint16_T>(obj->sizes[0]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = std::abs(x_data[obj->indexFixed.data[mIneq] - 1] - obj->ub.data
                    [obj->indexFixed.data[mIneq] - 1]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_xgemv_jt(int32_T m, int32_T n,
  const real_T A_data[], int32_T lda, const real_T x_data[], int32_T ix0, real_T
  y_data[])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    int32_T iy;
    b = static_cast<uint8_T>(n);
    for (int32_T b_iy = 0; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    iy = 0;
    d = (n - 1) * lda + 1;
    for (int32_T b_iy = 1; lda < 0 ? b_iy >= d : b_iy <= d; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x_data[((ix0 + b) - b_iy) - 1] * A_data[b - 1];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_maxConstraintViolation_ds
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[], int32_T
   ix0)
{
  real_T u1;
  real_T v;
  int32_T f;
  int32_T k;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      AEBController_xgemv_jt(75, obj->sizes[2], obj->Aineq.data, obj->ldA,
        x_data, ix0, obj->maxConstrWorkspace.data);
      f = static_cast<uint8_T>(obj->sizes[2]);
      for (k = 0; k < f; k++) {
        u1 = obj->maxConstrWorkspace.data[k] - x_data[(ix0 + k) + 74];
        obj->maxConstrWorkspace.data[k] = u1;
        if ((!(v >= u1)) && (!rtIsNaN(u1))) {
          v = u1;
        }
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
                (real_T));
    AEBController_xgemv_jt(75, 70, obj->Aeq.data, obj->ldA, x_data, ix0,
      obj->maxConstrWorkspace.data);
    for (k = 0; k < 70; k++) {
      obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] -
        x_data[((ix0 + mIneq) + k) + 74]) + x_data[((ix0 + obj->sizes[2]) + k) +
        144];
      u1 = std::abs(obj->maxConstrWorkspace.data[k]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      if (mIneq - 1 >= 0) {
        std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->bineq.data[0],
                    static_cast<uint32_T>(mIneq) * sizeof(real_T));
      }

      AEBController_xgemv_jt(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA,
        x_data, ix0, obj->maxConstrWorkspace.data);
      mIneq = static_cast<uint8_T>(obj->sizes[2]);
      for (k = 0; k < mIneq; k++) {
        u1 = obj->maxConstrWorkspace.data[k];
        if ((!(v >= u1)) && (!rtIsNaN(u1))) {
          v = u1;
        }
      }
    }

    std::memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 70U * sizeof
                (real_T));
    AEBController_xgemv_jt(obj->nVar, 70, obj->Aeq.data, obj->ldA, x_data, ix0,
      obj->maxConstrWorkspace.data);
    for (k = 0; k < 70; k++) {
      u1 = std::abs(obj->maxConstrWorkspace.data[k]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = -x_data[(ix0 + obj->indexLB.data[mIneq]) - 2] - obj->lb.data
        [obj->indexLB.data[mIneq] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[4] > 0) {
    k = static_cast<uint16_T>(obj->sizes[4]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = x_data[(ix0 + obj->indexUB.data[mIneq]) - 2] - obj->ub.data
        [obj->indexUB.data[mIneq] - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[0] > 0) {
    k = static_cast<uint16_T>(obj->sizes[0]);
    for (mIneq = 0; mIneq < k; mIneq++) {
      u1 = std::abs(x_data[(ix0 + obj->indexFixed.data[mIneq]) - 2] -
                    obj->ub.data[obj->indexFixed.data[mIneq] - 1]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
boolean_T ACCWithSensorFusionModelClass::AEBController_feasibleX0ForWorkingSet
  (real_T workspace_data[], const int32_T workspace_size[2], real_T
   xCurrent_data[], s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager)
{
  real_T constrViolation_basicX;
  real_T temp;
  int32_T b_idx;
  int32_T b_n;
  int32_T br;
  int32_T exitg1;
  int32_T iAcol;
  int32_T iQR0;
  int32_T ix;
  int32_T jBcol;
  int32_T ldq;
  int32_T mWConstr;
  int32_T n;
  int32_T nVar;
  int32_T rankQR;
  int32_T temp_tmp;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    if (workingset->nActiveConstr >= workingset->nVar) {
      temp_tmp = static_cast<uint16_T>(workingset->nVar);
      for (rankQR = 0; rankQR < temp_tmp; rankQR++) {
        ix = qrmanager->ldq * rankQR;
        for (b_idx = 0; b_idx < mWConstr; b_idx++) {
          qrmanager->QR.data[b_idx + ix] = workingset->ATwset.data
            [workingset->ldA * b_idx + rankQR];
        }
      }

      std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint16_T>
                  (workingset->nVar) * sizeof(int32_T));
      AEBController_factorQRE_i(qrmanager, workingset->nActiveConstr,
        workingset->nVar);
      ix = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < ix; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR + rankQR;
        b_n = qrmanager->mrows - rankQR;
        if (b_n - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0 + 1],
                      static_cast<uint32_T>(b_n - 1) * sizeof(real_T));
        }
      }

      AEBController_xorgqr(qrmanager->mrows, qrmanager->mrows,
                           qrmanager->minRowCol, qrmanager->Q.data,
                           qrmanager->Q.size, qrmanager->ldq,
                           qrmanager->tau.data);
      rankQR = AEBController_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (b_idx = 0; b_idx < mWConstr; b_idx++) {
        workspace_data[b_idx] = workingset->bwset.data[b_idx];
        workspace_data[b_idx + workspace_size[0]] = workingset->bwset.data[b_idx];
      }

      ix = workingset->ldA;
      iQR0 = 0;
      b_n = (workingset->nActiveConstr - 1) * workingset->ldA + 1;
      for (ldq = 1; ix < 0 ? ldq >= b_n : ldq <= b_n; ldq += ix) {
        temp = 0.0;
        jBcol = (ldq + nVar) - 1;
        for (b_idx = ldq; b_idx <= jBcol; b_idx++) {
          temp += workingset->ATwset.data[b_idx - 1] * xCurrent_data[b_idx - ldq];
        }

        workspace_data[iQR0] -= temp;
        iQR0++;
      }

      ldq = qrmanager->ldq;
      ix = workspace_size[0];
      iQR0 = workspace_size[0] * workspace_size[1];
      if (iQR0 - 1 >= 0) {
        std::memcpy(&AEBController_DW.B_data[0], &workspace_data[0],
                    static_cast<uint32_T>(iQR0) * sizeof(real_T));
      }

      for (b_idx = 0; ix < 0 ? b_idx >= ix : b_idx <= ix; b_idx += ix) {
        b_n = b_idx + nVar;
        for (iQR0 = b_idx + 1; iQR0 <= b_n; iQR0++) {
          workspace_data[iQR0 - 1] = 0.0;
        }
      }

      br = -1;
      for (b_n = 0; ix < 0 ? b_n >= ix : b_n <= ix; b_n += ix) {
        iAcol = -1;
        n = b_n + nVar;
        for (iQR0 = b_n + 1; iQR0 <= n; iQR0++) {
          temp = 0.0;
          for (jBcol = 0; jBcol < mWConstr; jBcol++) {
            temp += qrmanager->Q.data[(jBcol + iAcol) + 1] *
              AEBController_DW.B_data[(jBcol + br) + 1];
          }

          workspace_data[iQR0 - 1] += temp;
          iAcol += ldq;
        }

        br += ix;
      }

      for (b_n = 0; b_n < 2; b_n++) {
        br = ix * b_n - 1;
        for (iQR0 = rankQR; iQR0 >= 1; iQR0--) {
          iAcol = (iQR0 - 1) * ldq;
          b_idx = iQR0 + br;
          temp = workspace_data[b_idx];
          if (temp != 0.0) {
            workspace_data[b_idx] = temp / qrmanager->QR.data[(iQR0 + iAcol) - 1];
            n = static_cast<uint16_T>(iQR0 - 1);
            for (jBcol = 0; jBcol < n; jBcol++) {
              mWConstr = (jBcol + br) + 1;
              workspace_data[mWConstr] -= qrmanager->QR.data[jBcol + iAcol] *
                workspace_data[b_idx];
            }
          }
        }
      }

      for (b_idx = rankQR + 1; b_idx <= nVar; b_idx++) {
        workspace_data[b_idx - 1] = 0.0;
        workspace_data[(b_idx + workspace_size[0]) - 1] = 0.0;
      }

      for (rankQR = 0; rankQR < temp_tmp; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR];
      }

      for (rankQR = 0; rankQR < temp_tmp; rankQR++) {
        workspace_data[rankQR] = workspace_data[(workspace_size[0] << 1) +
          rankQR];
      }

      for (rankQR = 0; rankQR < temp_tmp; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR + workspace_size[0]];
      }

      for (rankQR = 0; rankQR < temp_tmp; rankQR++) {
        workspace_data[rankQR + workspace_size[0]] = workspace_data
          [(workspace_size[0] << 1) + rankQR];
      }
    } else {
      if (workingset->nActiveConstr - 1 >= 0) {
        std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint32_T>
                    (workingset->nActiveConstr) * sizeof(int32_T));
      }

      AEBController_factorQRE(qrmanager, workingset->ATwset.data,
        workingset->nVar, workingset->nActiveConstr, workingset->ldA);
      ix = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < ix; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR + rankQR;
        b_n = qrmanager->mrows - rankQR;
        if (b_n - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0 + 1],
                      static_cast<uint32_T>(b_n - 1) * sizeof(real_T));
        }
      }

      AEBController_xorgqr(qrmanager->mrows, qrmanager->minRowCol,
                           qrmanager->minRowCol, qrmanager->Q.data,
                           qrmanager->Q.size, qrmanager->ldq,
                           qrmanager->tau.data);
      rankQR = AEBController_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (b_idx = 0; b_idx < mWConstr; b_idx++) {
        ix = (qrmanager->jpvt.data[b_idx] - 1) * workingset->ldA;
        temp = 0.0;
        b_n = static_cast<uint16_T>(nVar);
        for (ldq = 0; ldq < b_n; ldq++) {
          temp += workingset->ATwset.data[ix + ldq] * xCurrent_data[ldq];
        }

        constrViolation_basicX = workingset->bwset.data[qrmanager->
          jpvt.data[b_idx] - 1];
        workspace_data[b_idx] = constrViolation_basicX - temp;
        workspace_data[b_idx + workspace_size[0]] = constrViolation_basicX;
      }

      ldq = qrmanager->ldq;
      ix = workspace_size[0];
      br = static_cast<uint16_T>(rankQR);
      for (mWConstr = 0; mWConstr < 2; mWConstr++) {
        jBcol = ix * mWConstr;
        for (b_n = 0; b_n < br; b_n++) {
          iAcol = ldq * b_n;
          temp_tmp = b_n + jBcol;
          temp = workspace_data[temp_tmp];
          for (iQR0 = 0; iQR0 < b_n; iQR0++) {
            temp -= qrmanager->QR.data[iQR0 + iAcol] * workspace_data[iQR0 +
              jBcol];
          }

          workspace_data[temp_tmp] = temp / qrmanager->QR.data[b_n + iAcol];
        }
      }

      iQR0 = workspace_size[0] * workspace_size[1];
      if (iQR0 - 1 >= 0) {
        std::memcpy(&AEBController_DW.B_data[0], &workspace_data[0],
                    static_cast<uint32_T>(iQR0) * sizeof(real_T));
      }

      for (b_idx = 0; ix < 0 ? b_idx >= ix : b_idx <= ix; b_idx += ix) {
        iQR0 = b_idx + nVar;
        for (mWConstr = b_idx + 1; mWConstr <= iQR0; mWConstr++) {
          workspace_data[mWConstr - 1] = 0.0;
        }
      }

      br = 1;
      for (b_n = 0; ix < 0 ? b_n >= ix : b_n <= ix; b_n += ix) {
        iAcol = -1;
        n = br + rankQR;
        for (iQR0 = br; iQR0 < n; iQR0++) {
          mWConstr = b_n + nVar;
          for (jBcol = b_n + 1; jBcol <= mWConstr; jBcol++) {
            workspace_data[jBcol - 1] += qrmanager->Q.data[(iAcol + jBcol) - b_n]
              * AEBController_DW.B_data[iQR0 - 1];
          }

          iAcol += ldq;
        }

        br += ix;
      }
    }

    rankQR = 0;
    do {
      exitg1 = 0;
      if (rankQR <= static_cast<uint16_T>(nVar) - 1) {
        temp = workspace_data[rankQR];
        if (rtIsInf(temp) || rtIsNaN(temp)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          temp = workspace_data[rankQR + workspace_size[0]];
          if (rtIsInf(temp) || rtIsNaN(temp)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            rankQR++;
          }
        }
      } else {
        for (rankQR = 0; rankQR < nVar; rankQR++) {
          workspace_data[rankQR] += xCurrent_data[rankQR];
        }

        temp = AEBController_maxConstraintViolation_d(workingset, workspace_data);
        constrViolation_basicX = AEBController_maxConstraintViolation_ds
          (workingset, workspace_data, workspace_size[0] + 1);
        if ((temp <= 2.2204460492503131E-16) || (temp < constrViolation_basicX))
        {
          std::memcpy(&xCurrent_data[0], &workspace_data[0],
                      static_cast<uint16_T>(nVar) * sizeof(real_T));
        } else {
          rankQR = static_cast<uint16_T>(nVar);
          for (nVar = 0; nVar < rankQR; nVar++) {
            xCurrent_data[nVar] = workspace_data[workspace_size[0] + nVar];
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_checkStoppingAndUpdateFval_p
  (int32_T *activeSetChangeID, s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
   *solution, s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
   s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective,
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager, int32_T
   runTimeOptions_MaxIterations, boolean_T *updateFval)
{
  real_T tempMaxConstr;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    tempMaxConstr = AEBController_maxConstraintViolation_dsy(workingset,
      solution->xstar.data);
    solution->maxConstr = tempMaxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      if (objective->nvar - 1 >= 0) {
        std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                    static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
      }

      nonDegenerateWset = AEBController_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = AEBController_maxConstraintViolation_dsy(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        if (objective->nvar - 1 >= 0) {
          std::memcpy(&solution->xstar.data[0], &solution->searchDir.data[0],
                      static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    *updateFval = false;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_iterate_k(const real_T H[5625],
  const real_T f_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations)
{
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  AEBController_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = AEBController_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  idxMinLambda = workingset->mConstrMax;
  for (k = 0; k < idxMinLambda; k++) {
    solution->lambda.data[k] = 0.0;
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          AEBController_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          AEBController_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          AEBController_factorQR(qrmanager, workingset->ATwset.data, nVar,
            workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            idxMinLambda = qrmanager->mrows - k;
            if (idxMinLambda - 2 >= 0) {
              std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0
                          + 1], static_cast<uint32_T>(idxMinLambda - 1) * sizeof
                          (real_T));
            }
          }

          AEBController_xorgqr(qrmanager->mrows, qrmanager->mrows,
                               qrmanager->minRowCol, qrmanager->Q.data,
                               qrmanager->Q.size, qrmanager->ldq,
                               qrmanager->tau.data);
          break;
        }

        AEBController_compute_deltax(H, solution, memspace, qrmanager,
          cholmanager, objective, AEBController_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = AEBController_xnrm2_pfp(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir.data[k] = 0.0;
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.0E-6) ||
            (workingset->nActiveConstr >= nVar)) {
          AEBController_compute_lambda(memspace->workspace_float.data, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              AEBController_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            AEBController_removeConstr(workingset, workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          AEBController_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr,
            (objective->objtype == 5), &normDelta, &updateFval, &k,
            &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              AEBController_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              AEBController_addBoundToActiveSetMatrix_(workingset, 4,
                idxMinLambda);
              break;

             default:
              AEBController_addBoundToActiveSetMatrix_(workingset, 5,
                idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            AEBController_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            for (k = 0; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          AEBController_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        AEBController_checkStoppingAndUpdateFval_p(&activeSetChangeID, solution,
          memspace, objective, workingset, qrmanager,
          runTimeOptions_MaxIterations, &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = AEBController_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_checkStoppingAndUpdateFval
  (int32_T *activeSetChangeID, const real_T f_data[],
   s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace, const
   s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective,
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager, int32_T
   runTimeOptions_MaxIterations, const boolean_T *updateFval)
{
  real_T tempMaxConstr;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    solution->maxConstr = AEBController_maxConstraintViolation_dsy(workingset,
      solution->xstar.data);
    tempMaxConstr = solution->maxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      if (objective->nvar - 1 >= 0) {
        std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                    static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
      }

      nonDegenerateWset = AEBController_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = AEBController_maxConstraintViolation_dsy(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        if (objective->nvar - 1 >= 0) {
          std::memcpy(&solution->xstar.data[0], &solution->searchDir.data[0],
                      static_cast<uint32_T>(objective->nvar) * sizeof(real_T));
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    solution->fstar = AEBController_computeFval_ReuseHx(objective,
      memspace->workspace_float.data, f_data, solution->xstar.data);
    if ((solution->fstar < 1.0E-6) && ((solution->state != 0) ||
         (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_iterate(const real_T H[5625],
  const real_T f_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
  options_SolverName[7], int32_T runTimeOptions_MaxIterations)
{
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  AEBController_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = AEBController_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  solution->state = -5;
  idxMinLambda = workingset->mConstrMax;
  for (k = 0; k < idxMinLambda; k++) {
    solution->lambda.data[k] = 0.0;
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          AEBController_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          AEBController_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          AEBController_factorQR(qrmanager, workingset->ATwset.data, nVar,
            workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            idxMinLambda = qrmanager->mrows - k;
            if (idxMinLambda - 2 >= 0) {
              std::memcpy(&qrmanager->Q.data[iQR0 + 1], &qrmanager->QR.data[iQR0
                          + 1], static_cast<uint32_T>(idxMinLambda - 1) * sizeof
                          (real_T));
            }
          }

          AEBController_xorgqr(qrmanager->mrows, qrmanager->mrows,
                               qrmanager->minRowCol, qrmanager->Q.data,
                               qrmanager->Q.size, qrmanager->ldq,
                               qrmanager->tau.data);
          break;
        }

        AEBController_compute_deltax(H, solution, memspace, qrmanager,
          cholmanager, objective, AEBController_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = AEBController_xnrm2_pfp(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir.data[k] = 0.0;
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.4901161193847657E-10) ||
            (workingset->nActiveConstr >= nVar)) {
          AEBController_compute_lambda(memspace->workspace_float.data, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              AEBController_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            AEBController_removeConstr(workingset, workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          AEBController_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr, true,
            &normDelta, &updateFval, &k, &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              AEBController_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              AEBController_addBoundToActiveSetMatrix_(workingset, 4,
                idxMinLambda);
              break;

             default:
              AEBController_addBoundToActiveSetMatrix_(workingset, 5,
                idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            AEBController_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            for (k = 0; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          AEBController_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        AEBController_checkStoppingAndUpdateFval(&activeSetChangeID, f_data,
          solution, memspace, objective, workingset, qrmanager,
          runTimeOptions_MaxIterations, &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = AEBController_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_phaseone(const real_T H[5625],
  const real_T f_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const char_T
  options_SolverName[7], const somzaGboVhDG7PNQS6E98jD_AEBController_T
  *runTimeOptions)
{
  int32_T PROBTYPE_ORIG;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T mConstr;
  int32_T nVar_tmp;
  boolean_T exitg1;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp = workingset->nVar;
  solution->xstar.data[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }

  AEBController_setProblemType(workingset, mConstr);
  idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq = idxStartIneq_tmp + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (mConstr = idxStartIneq; mConstr <= idxEndIneq; mConstr++) {
    workingset->isActiveConstr.data[(workingset->isActiveIdx
      [workingset->Wid.data[mConstr - 1] - 1] + workingset->
      Wlocalidx.data[mConstr - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = idxStartIneq_tmp;
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVar_tmp + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  solution->fstar = solution->xstar.data[nVar_tmp];
  solution->state = 5;
  AEBController_iterate(H, f_data, solution, memspace, workingset, qrmanager,
                        cholmanager, objective, options_SolverName,
                        runTimeOptions->MaxIterations);
  if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] +
       workingset->sizes[3]) - 2]) {
    mConstr = workingset->sizes[0] + 71;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      if ((workingset->Wid.data[mConstr - 1] == 4) &&
          (workingset->Wlocalidx.data[mConstr - 1] == workingset->sizes[3])) {
        AEBController_removeConstr(workingset, mConstr);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }

  mConstr = workingset->nActiveConstr;
  while ((mConstr > workingset->sizes[0] + 70) && (mConstr > nVar_tmp)) {
    AEBController_removeConstr(workingset, mConstr);
    mConstr--;
  }

  solution->maxConstr = solution->xstar.data[nVar_tmp];
  AEBController_setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_countsort(int32_T x_data[],
  int32_T xLen, int32_T workspace_data[], int32_T xMin, int32_T xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T b_tmp;
    int32_T idxEnd;
    int32_T idxFill;
    int32_T idxStart;
    int32_T maxOffset;
    b_tmp = xMax - xMin;
    if (b_tmp >= 0) {
      std::memset(&workspace_data[0], 0, static_cast<uint32_T>(b_tmp + 1) *
                  sizeof(int32_T));
    }

    maxOffset = b_tmp - 1;
    for (b_tmp = 0; b_tmp < xLen; b_tmp++) {
      idxFill = x_data[b_tmp] - xMin;
      workspace_data[idxFill]++;
    }

    for (b_tmp = 2; b_tmp <= maxOffset + 2; b_tmp++) {
      workspace_data[b_tmp - 1] += workspace_data[b_tmp - 2];
    }

    idxStart = 1;
    idxEnd = workspace_data[0];
    for (b_tmp = 0; b_tmp <= maxOffset; b_tmp++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = b_tmp + xMin;
      }

      idxStart = workspace_data[b_tmp] + 1;
      idxEnd = workspace_data[b_tmp + 1];
    }

    for (maxOffset = idxStart; maxOffset <= idxEnd; maxOffset++) {
      x_data[maxOffset - 1] = xMax;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
int32_T ACCWithSensorFusionModelClass::AEBController_RemoveDependentEq_
  (s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T c_tmp_tmp;
  int32_T ix;
  int32_T mTotalWorkingEq;
  int32_T mTotalWorkingEq_tmp;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T nVar;
  int32_T totalRank;
  boolean_T exitg1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp > 0) {
    c_tmp_tmp = static_cast<uint16_T>(workingset->nVar);
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
      for (mTotalWorkingEq = 0; mTotalWorkingEq < c_tmp_tmp; mTotalWorkingEq++)
      {
        qrmanager->QR.data[totalRank + qrmanager->ldq * mTotalWorkingEq] =
          workingset->ATwset.data[workingset->ldA * totalRank + mTotalWorkingEq];
      }
    }

    mTotalWorkingEq = mTotalWorkingEq_tmp - workingset->nVar;
    if (mTotalWorkingEq > 0) {
      nDepInd = mTotalWorkingEq;
    }

    std::memset(&qrmanager->jpvt.data[0], 0, static_cast<uint16_T>
                (workingset->nVar) * sizeof(int32_T));
    AEBController_factorQRE_i(qrmanager, mTotalWorkingEq_tmp, workingset->nVar);
    if (mTotalWorkingEq_tmp >= workingset->nVar) {
      totalRank = mTotalWorkingEq_tmp;
    } else {
      totalRank = workingset->nVar;
    }

    tol = 2.2204460492503131E-15 * static_cast<real_T>(totalRank);
    if (tol >= 1.4901161193847656E-8) {
      tol = 1.4901161193847656E-8;
    }

    if (workingset->nVar <= mTotalWorkingEq_tmp) {
      totalRank = workingset->nVar;
    } else {
      totalRank = mTotalWorkingEq_tmp;
    }

    totalRank += (totalRank - 1) * qrmanager->ldq;
    while ((totalRank > 0) && (std::abs(qrmanager->QR.data[totalRank - 1]) < std::
            abs(qrmanager->QR.data[0]) * tol)) {
      totalRank = (totalRank - qrmanager->ldq) - 1;
      nDepInd++;
    }

    if (nDepInd > 0) {
      ix = qrmanager->minRowCol;
      for (mTotalWorkingEq = 0; mTotalWorkingEq < ix; mTotalWorkingEq++) {
        totalRank = qrmanager->ldq * mTotalWorkingEq + mTotalWorkingEq;
        nVar = qrmanager->mrows - mTotalWorkingEq;
        if (nVar - 2 >= 0) {
          std::memcpy(&qrmanager->Q.data[totalRank + 1], &qrmanager->
                      QR.data[totalRank + 1], static_cast<uint32_T>(nVar - 1) *
                      sizeof(real_T));
        }
      }

      AEBController_xorgqr(qrmanager->mrows, qrmanager->mrows,
                           qrmanager->minRowCol, qrmanager->Q.data,
                           qrmanager->Q.size, qrmanager->ldq,
                           qrmanager->tau.data);
      nVar = 0;
      exitg1 = false;
      while ((!exitg1) && (nVar <= nDepInd - 1)) {
        ix = ((mTotalWorkingEq_tmp - nVar) - 1) * qrmanager->ldq;
        qtb = 0.0;
        for (mTotalWorkingEq = 0; mTotalWorkingEq < mTotalWorkingEq_tmp;
             mTotalWorkingEq++) {
          qtb += qrmanager->Q.data[ix + mTotalWorkingEq] *
            workingset->bwset.data[mTotalWorkingEq];
        }

        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          nVar++;
        }
      }
    }

    if (nDepInd > 0) {
      for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
        nVar = qrmanager->ldq * totalRank;
        ix = workingset->ldA * totalRank;
        for (mTotalWorkingEq = 0; mTotalWorkingEq < c_tmp_tmp; mTotalWorkingEq++)
        {
          qrmanager->QR.data[nVar + mTotalWorkingEq] = workingset->
            ATwset.data[ix + mTotalWorkingEq];
        }
      }

      for (nVar = 0; nVar < mWorkingFixed; nVar++) {
        qrmanager->jpvt.data[nVar] = 1;
      }

      nVar = workingset->nWConstr[0] + 1;
      if (nVar <= mTotalWorkingEq_tmp) {
        std::memset(&qrmanager->jpvt.data[nVar + -1], 0, static_cast<uint32_T>
                    ((mTotalWorkingEq_tmp - nVar) + 1) * sizeof(int32_T));
      }

      AEBController_factorQRE_i(qrmanager, workingset->nVar, mTotalWorkingEq_tmp);
      for (mWorkingFixed = 0; mWorkingFixed < nDepInd; mWorkingFixed++) {
        memspace->workspace_int.data[mWorkingFixed] = qrmanager->jpvt.data
          [(mTotalWorkingEq_tmp - nDepInd) + mWorkingFixed];
      }

      AEBController_countsort(memspace->workspace_int.data, nDepInd,
        memspace->workspace_sort.data, 1, mTotalWorkingEq_tmp);
      for (mTotalWorkingEq = nDepInd; mTotalWorkingEq >= 1; mTotalWorkingEq--) {
        totalRank = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (totalRank != 0) {
          nVar = memspace->workspace_int.data[mTotalWorkingEq - 1];
          if (nVar <= totalRank) {
            if ((totalRank == workingset->nActiveConstr) || (totalRank == nVar))
            {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx.data[nVar - 1];
              AEBController_removeConstr(workingset,
                memspace->workspace_int.data[mTotalWorkingEq - 1]);
            } else {
              workingset->mEqRemoved++;
              mTotalWorkingEq_tmp = workingset->Wid.data[nVar - 1] - 1;
              mWorkingFixed = workingset->Wlocalidx.data[nVar - 1];
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                mWorkingFixed;
              workingset->isActiveConstr.data[(workingset->
                isActiveIdx[mTotalWorkingEq_tmp] + mWorkingFixed) - 2] = false;
              workingset->Wid.data[nVar - 1] = workingset->Wid.data[totalRank -
                1];
              workingset->Wlocalidx.data[nVar - 1] = workingset->
                Wlocalidx.data[totalRank - 1];
              for (mWorkingFixed = 0; mWorkingFixed < c_tmp_tmp; mWorkingFixed++)
              {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA * (nVar
                  - 1)] = workingset->ATwset.data[(totalRank - 1) *
                  workingset->ldA + mWorkingFixed];
              }

              workingset->bwset.data[nVar - 1] = workingset->
                bwset.data[totalRank - 1];
              workingset->Wid.data[totalRank - 1] = workingset->
                Wid.data[workingset->nActiveConstr - 1];
              workingset->Wlocalidx.data[totalRank - 1] =
                workingset->Wlocalidx.data[workingset->nActiveConstr - 1];
              for (mWorkingFixed = 0; mWorkingFixed < c_tmp_tmp; mWorkingFixed++)
              {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA *
                  (totalRank - 1)] = workingset->ATwset.data
                  [(workingset->nActiveConstr - 1) * workingset->ldA +
                  mWorkingFixed];
              }

              workingset->bwset.data[totalRank - 1] = workingset->
                bwset.data[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[mTotalWorkingEq_tmp]--;
            }
          }
        }
      }
    }
  }

  return nDepInd;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_RemoveDependentIneq_
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  real_T u1;
  int32_T d;
  int32_T ix0;
  int32_T iy0;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;
  int32_T nVar_tmp_tmp;
  nVar = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar_tmp_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      nDepIneq = workingset->nVar;
    } else {
      nDepIneq = workingset->nActiveConstr;
    }

    tol = 2.2204460492503131E-15 * static_cast<real_T>(nDepIneq);
    if (tol >= 1.4901161193847656E-8) {
      tol = 1.4901161193847656E-8;
    }

    for (nDepIneq = 0; nDepIneq < nFixedConstr; nDepIneq++) {
      qrmanager->jpvt.data[nDepIneq] = 1;
    }

    if (nFixedConstr + 1 <= workingset->nActiveConstr) {
      std::memset(&qrmanager->jpvt.data[nFixedConstr], 0, static_cast<uint32_T>
                  (workingset->nActiveConstr - nFixedConstr) * sizeof(int32_T));
    }

    for (nDepIneq = 0; nDepIneq < nVar; nDepIneq++) {
      iy0 = qrmanager->ldq * nDepIneq;
      ix0 = workingset->ldA * nDepIneq;
      d = static_cast<uint16_T>(nVar_tmp_tmp);
      for (nActiveConstr = 0; nActiveConstr < d; nActiveConstr++) {
        qrmanager->QR.data[iy0 + nActiveConstr] = workingset->ATwset.data[ix0 +
          nActiveConstr];
      }
    }

    AEBController_factorQRE_i(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (nActiveConstr = workingset->nActiveConstr - 1; nActiveConstr + 1 >
         nVar_tmp_tmp; nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
        jpvt.data[nActiveConstr];
    }

    maxDiag = std::abs(qrmanager->QR.data[0]);
    for (nVar = 0; nVar < nActiveConstr; nVar++) {
      u1 = std::abs(qrmanager->QR.data[((nVar + 1) * qrmanager->ldq + nVar) + 1]);
      if ((!(maxDiag >= u1)) && (!rtIsNaN(u1))) {
        maxDiag = u1;
      }
    }

    if (nActiveConstr + 1 <= workingset->nVar) {
      nVar = qrmanager->ldq * nActiveConstr + nActiveConstr;
      while ((nActiveConstr + 1 > nFixedConstr) && (std::abs(qrmanager->
               QR.data[nVar]) < tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
          jpvt.data[nActiveConstr];
        nActiveConstr--;
        nVar = (nVar - qrmanager->ldq) - 1;
      }
    }

    AEBController_countsort(memspace->workspace_int.data, nDepIneq,
      memspace->workspace_sort.data, nFixedConstr + 1, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      AEBController_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_RemoveDependentIneq__j
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  real_T u1;
  int32_T d;
  int32_T ix0;
  int32_T iy0;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;
  int32_T nVar_tmp_tmp;
  nVar = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar_tmp_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      nDepIneq = workingset->nVar;
    } else {
      nDepIneq = workingset->nActiveConstr;
    }

    u1 = 2.2204460492503131E-15 * static_cast<real_T>(nDepIneq);
    if (u1 >= 1.4901161193847656E-8) {
      u1 = 1.4901161193847656E-8;
    }

    tol = 10.0 * u1;
    for (nDepIneq = 0; nDepIneq < nFixedConstr; nDepIneq++) {
      qrmanager->jpvt.data[nDepIneq] = 1;
    }

    if (nFixedConstr + 1 <= workingset->nActiveConstr) {
      std::memset(&qrmanager->jpvt.data[nFixedConstr], 0, static_cast<uint32_T>
                  (workingset->nActiveConstr - nFixedConstr) * sizeof(int32_T));
    }

    for (nDepIneq = 0; nDepIneq < nVar; nDepIneq++) {
      iy0 = qrmanager->ldq * nDepIneq;
      ix0 = workingset->ldA * nDepIneq;
      d = static_cast<uint16_T>(nVar_tmp_tmp);
      for (nActiveConstr = 0; nActiveConstr < d; nActiveConstr++) {
        qrmanager->QR.data[iy0 + nActiveConstr] = workingset->ATwset.data[ix0 +
          nActiveConstr];
      }
    }

    AEBController_factorQRE_i(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (nActiveConstr = workingset->nActiveConstr - 1; nActiveConstr + 1 >
         nVar_tmp_tmp; nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
        jpvt.data[nActiveConstr];
    }

    maxDiag = std::abs(qrmanager->QR.data[0]);
    for (nVar = 0; nVar < nActiveConstr; nVar++) {
      u1 = std::abs(qrmanager->QR.data[((nVar + 1) * qrmanager->ldq + nVar) + 1]);
      if ((!(maxDiag >= u1)) && (!rtIsNaN(u1))) {
        maxDiag = u1;
      }
    }

    if (nActiveConstr + 1 <= workingset->nVar) {
      nVar = qrmanager->ldq * nActiveConstr + nActiveConstr;
      while ((nActiveConstr + 1 > nFixedConstr) && (std::abs(qrmanager->
               QR.data[nVar]) < tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->
          jpvt.data[nActiveConstr];
        nActiveConstr--;
        nVar = (nVar - qrmanager->ldq) - 1;
      }
    }

    AEBController_countsort(memspace->workspace_int.data, nDepIneq,
      memspace->workspace_sort.data, nFixedConstr + 1, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      AEBController_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_PresolveWorkingSet
  (s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager)
{
  real_T constrViolation;
  int32_T b;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  boolean_T guard1;
  boolean_T okWorkingSet;
  solution->state = 82;
  b = AEBController_RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((b != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    AEBController_RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = AEBController_feasibleX0ForWorkingSet
      (memspace->workspace_float.data, memspace->workspace_float.size,
       solution->xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      AEBController_RemoveDependentIneq__j(workingset, qrmanager, memspace);
      okWorkingSet = AEBController_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (workingset->nWConstr[0] + workingset->nWConstr[1] == workingset->nVar)
      {
        constrViolation = AEBController_maxConstraintViolation_dsy(workingset,
          solution->xstar.data);
        if (constrViolation > 1.0E-6) {
          solution->state = -2;
        }
      }
    }
  } else {
    solution->state = -3;
    idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
    idxStartIneq = idxStartIneq_tmp + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (b = idxStartIneq; b <= idxEndIneq; b++) {
      workingset->isActiveConstr.data[(workingset->isActiveIdx
        [workingset->Wid.data[b - 1] - 1] + workingset->Wlocalidx.data[b - 1]) -
        2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = idxStartIneq_tmp;
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_linearForm_(boolean_T
  obj_hasLinear, int32_T obj_nvar, real_T workspace_data[], const real_T H[5625],
  const real_T f_data[], const real_T x_data[])
{
  int32_T beta1;
  beta1 = 0;
  if (obj_hasLinear) {
    if (obj_nvar - 1 >= 0) {
      std::memcpy(&workspace_data[0], &f_data[0], static_cast<uint32_T>(obj_nvar)
                  * sizeof(real_T));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if (beta1 != 1) {
      for (beta1 = 0; beta1 < obj_nvar; beta1++) {
        workspace_data[beta1] = 0.0;
      }
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar + 1;
    for (beta1 = 1; obj_nvar < 0 ? beta1 >= d : beta1 <= d; beta1 += obj_nvar) {
      real_T c;
      int32_T e;
      c = 0.5 * x_data[ix];
      e = (beta1 + obj_nvar) - 1;
      for (int32_T ia = beta1; ia <= e; ia++) {
        int32_T tmp;
        tmp = ia - beta1;
        workspace_data[tmp] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_driver_g(const real_T H[5625],
  const real_T f_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *solution,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *workingset,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *qrmanager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *cholmanager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *objective, const
  somzaGboVhDG7PNQS6E98jD_AEBController_T *options, const
  somzaGboVhDG7PNQS6E98jD_AEBController_T *runTimeOptions)
{
  real_T maxConstr_new;
  int32_T idx;
  int32_T ixlast;
  int32_T nVar;
  boolean_T guard1;
  solution->iterations = 0;
  nVar = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    ixlast = static_cast<uint16_T>(workingset->sizes[0]);
    for (idx = 0; idx < ixlast; idx++) {
      solution->xstar.data[workingset->indexFixed.data[idx] - 1] =
        workingset->ub.data[workingset->indexFixed.data[idx] - 1];
    }

    ixlast = static_cast<uint16_T>(workingset->sizes[3]);
    for (idx = 0; idx < ixlast; idx++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] + idx) - 1])
      {
        solution->xstar.data[workingset->indexLB.data[idx] - 1] =
          -workingset->lb.data[workingset->indexLB.data[idx] - 1];
      }
    }

    ixlast = static_cast<uint16_T>(workingset->sizes[4]);
    for (idx = 0; idx < ixlast; idx++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[4] + idx) - 1])
      {
        solution->xstar.data[workingset->indexUB.data[idx] - 1] =
          workingset->ub.data[workingset->indexUB.data[idx] - 1];
      }
    }

    AEBController_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state < 0) {
    } else {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = AEBController_maxConstraintViolation_dsy(workingset,
      solution->xstar.data);
    if (solution->maxConstr > 1.0E-6) {
      AEBController_phaseone(H, f_data, solution, memspace, workingset,
        qrmanager, cholmanager, objective, options->SolverName, runTimeOptions);
      if (solution->state != 0) {
        solution->maxConstr = AEBController_maxConstraintViolation_dsy
          (workingset, solution->xstar.data);
        if (solution->maxConstr > 1.0E-6) {
          idx = workingset->mConstrMax;
          for (nVar = 0; nVar < idx; nVar++) {
            solution->lambda.data[nVar] = 0.0;
          }

          maxConstr_new = 0.0;
          switch (objective->objtype) {
           case 5:
            maxConstr_new = solution->xstar.data[objective->nvar - 1] *
              objective->gammaScalar;
            break;

           case 3:
            AEBController_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            if (objective->nvar >= 1) {
              ixlast = objective->nvar;
              for (nVar = 0; nVar < ixlast; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;

           case 4:
            AEBController_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            idx = objective->nvar + 1;
            ixlast = objective->maxVar;
            for (nVar = idx; nVar < ixlast; nVar++) {
              memspace->workspace_float.data[nVar - 1] = 0.5 * objective->beta *
                solution->xstar.data[nVar - 1] + objective->rho;
            }

            if (objective->maxVar - 1 >= 1) {
              for (nVar = 0; nVar <= ixlast - 2; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;
          }

          solution->fstar = maxConstr_new;
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            if (nVar - 1 >= 0) {
              std::memcpy(&solution->searchDir.data[0], &solution->xstar.data[0],
                          static_cast<uint32_T>(nVar) * sizeof(real_T));
            }

            AEBController_PresolveWorkingSet(solution, memspace, workingset,
              qrmanager);
            maxConstr_new = AEBController_maxConstraintViolation_dsy(workingset,
              solution->xstar.data);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (nVar - 1 >= 0) {
                std::memcpy(&solution->xstar.data[0], &solution->searchDir.data
                            [0], static_cast<uint32_T>(nVar) * sizeof(real_T));
              }
            }
          }

          AEBController_iterate_k(H, f_data, solution, memspace, workingset,
            qrmanager, cholmanager, objective, options->SolverName,
            runTimeOptions->MaxIterations);
        }
      }
    } else {
      AEBController_iterate_k(H, f_data, solution, memspace, workingset,
        qrmanager, cholmanager, objective, options->SolverName,
        runTimeOptions->MaxIterations);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_addAeqConstr
  (s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, int32_T idx_local)
{
  int32_T totalEq;
  totalEq = obj->nWConstr[0] + obj->nWConstr[1];
  if ((obj->nActiveConstr == totalEq) && (idx_local > obj->nWConstr[1])) {
    int32_T b_idx;
    int32_T iAeq0;
    int32_T iAw0;
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = 2;
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
    b_idx = static_cast<uint16_T>(obj->nVar);
    for (totalEq = 0; totalEq < b_idx; totalEq++) {
      obj->ATwset.data[iAw0 + totalEq] = obj->Aeq.data[iAeq0 + totalEq];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->beq[idx_local - 1];
  } else {
    int32_T iAeq0;
    int32_T iAw0;
    int32_T iAw0_tmp;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = obj->Wid.data[totalEq];
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = obj->Wlocalidx.data[totalEq];
    iAw0_tmp = static_cast<uint16_T>(obj->nVar);
    for (iAeq0 = 0; iAeq0 < iAw0_tmp; iAeq0++) {
      obj->ATwset.data[iAeq0 + obj->ldA * (obj->nActiveConstr - 1)] =
        obj->ATwset.data[obj->ldA * totalEq + iAeq0];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->bwset.data[totalEq];
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->Wid.data[totalEq] = 2;
    obj->Wlocalidx.data[totalEq] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = obj->ldA * totalEq;
    for (int32_T b_idx = 0; b_idx < iAw0_tmp; b_idx++) {
      obj->ATwset.data[iAw0 + b_idx] = obj->Aeq.data[iAeq0 + b_idx];
    }

    obj->bwset.data[totalEq] = obj->beq[idx_local - 1];
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_sortLambdaQP(real_T
  lambda_data[], int32_T WorkingSet_nActiveConstr, const int32_T
  WorkingSet_sizes[5], const int32_T WorkingSet_isActiveIdx[6], const int32_T
  WorkingSet_Wid_data[], const int32_T WorkingSet_Wlocalidx_data[], real_T
  workspace_data[])
{
  if (WorkingSet_nActiveConstr != 0) {
    int32_T currentMplier;
    int32_T idxOffset;
    int32_T mAll;
    mAll = (((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4])
            + WorkingSet_sizes[2]) + 69;
    if (static_cast<uint16_T>(mAll + 1) - 1 >= 0) {
      std::memcpy(&workspace_data[0], &lambda_data[0], static_cast<uint16_T>
                  (mAll + 1) * sizeof(real_T));
    }

    for (currentMplier = 0; currentMplier <= mAll; currentMplier++) {
      lambda_data[currentMplier] = 0.0;
    }

    currentMplier = 0;
    mAll = 0;
    while ((mAll + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid_data[mAll] <=
            2)) {
      if (WorkingSet_Wid_data[mAll] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }

    while (mAll + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid_data[mAll]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
boolean_T ACCWithSensorFusionModelClass::AEBController_soc(const real_T Hessian
  [5625], const real_T grad_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
  *TrialState, s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const
  somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions)
{
  real_T b_c;
  int32_T idxIneqOffset;
  int32_T idxStartIneq;
  int32_T idx_Aineq;
  int32_T idx_lower;
  int32_T ix;
  int32_T iy;
  int32_T l;
  int32_T nVar;
  int32_T nWIneq_old;
  int32_T nWLower_old;
  int32_T nWUpper_old;
  boolean_T success;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  std::memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
              static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  std::memcpy(&TrialState->socDirection.data[0], &TrialState->xstar.data[0],
              static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  if (WorkingSet->mConstrMax - 1 >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->lambda.data[0],
                static_cast<uint32_T>(WorkingSet->mConstrMax) * sizeof(real_T));
  }

  idxIneqOffset = WorkingSet->isActiveIdx[2];
  for (idxStartIneq = 0; idxStartIneq < 70; idxStartIneq++) {
    WorkingSet->beq[idxStartIneq] = -TrialState->cEq[idxStartIneq];
  }

  idx_lower = WorkingSet->ldA;
  iy = 0;
  l = WorkingSet->ldA * 69 + 1;
  for (idxStartIneq = 1; idx_lower < 0 ? idxStartIneq >= l : idxStartIneq <= l;
       idxStartIneq += idx_lower) {
    b_c = 0.0;
    ix = (idxStartIneq + WorkingSet->nVar) - 1;
    for (idx_Aineq = idxStartIneq; idx_Aineq <= ix; idx_Aineq++) {
      b_c += WorkingSet->Aeq.data[idx_Aineq - 1] * TrialState->
        searchDir.data[idx_Aineq - idxStartIneq];
    }

    WorkingSet->beq[iy] += b_c;
    iy++;
  }

  for (idxStartIneq = 0; idxStartIneq < 70; idxStartIneq++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + idxStartIneq] =
      WorkingSet->beq[idxStartIneq];
  }

  if (WorkingSet->sizes[2] > 0) {
    idx_Aineq = static_cast<uint8_T>(WorkingSet->sizes[2]);
    for (idxStartIneq = 0; idxStartIneq < idx_Aineq; idxStartIneq++) {
      WorkingSet->bineq.data[idxStartIneq] = -TrialState->
        cIneq.data[idxStartIneq];
    }

    iy = 0;
    l = (WorkingSet->sizes[2] - 1) * WorkingSet->ldA + 1;
    for (idxStartIneq = 1; idx_lower < 0 ? idxStartIneq >= l : idxStartIneq <= l;
         idxStartIneq += idx_lower) {
      b_c = 0.0;
      ix = (idxStartIneq + WorkingSet->nVar) - 1;
      for (idx_Aineq = idxStartIneq; idx_Aineq <= ix; idx_Aineq++) {
        b_c += WorkingSet->Aineq.data[idx_Aineq - 1] *
          TrialState->searchDir.data[idx_Aineq - idxStartIneq];
      }

      WorkingSet->bineq.data[iy] += b_c;
      iy++;
    }

    idx_Aineq = 1;
    idx_lower = WorkingSet->sizes[2] + 1;
    iy = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
    l = WorkingSet->nActiveConstr;
    for (idxStartIneq = idxIneqOffset; idxStartIneq <= l; idxStartIneq++) {
      switch (WorkingSet->Wid.data[idxStartIneq - 1]) {
       case 3:
        ix = idx_Aineq;
        idx_Aineq++;
        WorkingSet->bwset.data[idxStartIneq - 1] = WorkingSet->
          bineq.data[WorkingSet->Wlocalidx.data[idxStartIneq - 1] - 1];
        break;

       case 4:
        ix = idx_lower;
        idx_lower++;
        break;

       default:
        ix = iy;
        iy++;
        break;
      }

      TrialState->workingset_old.data[ix - 1] = WorkingSet->
        Wlocalidx.data[idxStartIneq - 1];
    }
  }

  std::memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0], static_cast<
              uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  AEBController_driver_g(Hessian, grad_data, TrialState, memspace, WorkingSet,
    QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    AEBController_addAeqConstr(WorkingSet, WorkingSet->indexEqRemoved
      [WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  idxStartIneq = static_cast<uint16_T>(nVar);
  for (idxIneqOffset = 0; idxIneqOffset < idxStartIneq; idxIneqOffset++) {
    b_c = TrialState->socDirection.data[idxIneqOffset];
    TrialState->socDirection.data[idxIneqOffset] = TrialState->
      xstar.data[idxIneqOffset] - b_c;
    TrialState->xstar.data[idxIneqOffset] = b_c;
  }

  success = (AEBController_xnrm2_pfp(nVar, TrialState->socDirection.data) <= 2.0
             * AEBController_xnrm2_pfp(nVar, TrialState->xstar.data));
  idxIneqOffset = WorkingSet->sizes[2];
  for (nVar = 0; nVar < 70; nVar++) {
    WorkingSet->beq[nVar] = -TrialState->cEq[nVar];
  }

  for (nVar = 0; nVar < 70; nVar++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + nVar] = WorkingSet->beq[nVar];
  }

  if (WorkingSet->sizes[2] > 0) {
    idxStartIneq = static_cast<uint8_T>(WorkingSet->sizes[2]);
    for (nVar = 0; nVar < idxStartIneq; nVar++) {
      WorkingSet->bineq.data[nVar] = -TrialState->cIneq.data[nVar];
    }

    if (!success) {
      idx_lower = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = idx_lower + 1;
      idx_Aineq = WorkingSet->nActiveConstr;
      for (nVar = idxStartIneq; nVar <= idx_Aineq; nVar++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[nVar - 1] - 1] + WorkingSet->Wlocalidx.data[nVar
          - 1]) - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = idx_lower;
      for (nVar = 0; nVar < nWIneq_old; nVar++) {
        AEBController_addAineqConstr(WorkingSet, TrialState->
          workingset_old.data[nVar]);
      }

      for (nWIneq_old = 0; nWIneq_old < nWLower_old; nWIneq_old++) {
        AEBController_addBoundToActiveSetMatrix_(WorkingSet, 4,
          TrialState->workingset_old.data[nWIneq_old + idxIneqOffset]);
      }

      for (nWLower_old = 0; nWLower_old < nWUpper_old; nWLower_old++) {
        AEBController_addBoundToActiveSetMatrix_(WorkingSet, 5,
          TrialState->workingset_old.data[(nWLower_old + idxIneqOffset) +
          WorkingSet->sizes[3]]);
      }
    }
  }

  if (!success) {
    if (WorkingSet->mConstrMax - 1 >= 0) {
      std::memcpy(&TrialState->lambda.data[0], &TrialState->lambdaStopTest.data
                  [0], static_cast<uint32_T>(WorkingSet->mConstrMax) * sizeof
                  (real_T));
    }
  } else {
    AEBController_sortLambdaQP(TrialState->lambda.data,
      WorkingSet->nActiveConstr, WorkingSet->sizes, WorkingSet->isActiveIdx,
      WorkingSet->Wid.data, WorkingSet->Wlocalidx.data,
      memspace->workspace_float.data);
  }

  return success;
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_maxConstraintViolation(const
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *obj, const real_T x_data[])
{
  real_T u1;
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    AEBController_DW.b_obj = *obj;
    v = AEBController_maxConstraintViolation_AMats_regularized_
      (&AEBController_DW.b_obj, x_data);
  } else {
    AEBController_DW.b_obj = *obj;
    v = AEBController_maxConstraintViolation_AMats_nonregularized_
      (&AEBController_DW.b_obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = static_cast<uint16_T>(obj->sizes[3]);
    for (idx = 0; idx < b; idx++) {
      u1 = -x_data[AEBController_DW.b_obj.indexLB.data[idx] - 1] -
        AEBController_DW.b_obj.lb.data[AEBController_DW.b_obj.indexLB.data[idx]
        - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[4] > 0) {
    b = static_cast<uint16_T>(obj->sizes[4]);
    for (idx = 0; idx < b; idx++) {
      u1 = x_data[AEBController_DW.b_obj.indexUB.data[idx] - 1] -
        AEBController_DW.b_obj.ub.data[AEBController_DW.b_obj.indexUB.data[idx]
        - 1];
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  if (obj->sizes[0] > 0) {
    b = static_cast<uint16_T>(obj->sizes[0]);
    for (idx = 0; idx < b; idx++) {
      u1 = std::abs(x_data[AEBController_DW.b_obj.indexFixed.data[idx] - 1] -
                    AEBController_DW.b_obj.ub.data[AEBController_DW.b_obj.indexFixed.data
                    [idx] - 1]);
      if ((!(v >= u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  return v;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_normal(const real_T Hessian
  [5625], const real_T grad_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const
  somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions,
  s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *stepFlags)
{
  real_T TrialState_cIneq;
  real_T constrViolationEq;
  real_T constrViolationIneq;
  real_T penaltyParamTrial;
  int32_T b;
  int32_T k;
  boolean_T isEqAndIneqFeasible;
  AEBController_driver_g(Hessian, grad_data, TrialState, memspace, WorkingSet,
    QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  isEqAndIneqFeasible = (AEBController_maxConstraintViolation(WorkingSet,
    TrialState->xstar.data) <= 1.0E-6);
  if ((TrialState->state > 0) || ((TrialState->state == 0) &&
       isEqAndIneqFeasible)) {
    penaltyParamTrial = MeritFunction->penaltyParam;
    constrViolationEq = 0.0;
    for (k = 0; k < 70; k++) {
      constrViolationEq += std::abs(TrialState->cEq[k]);
    }

    constrViolationIneq = 0.0;
    b = static_cast<uint8_T>(WorkingSet->sizes[2]);
    for (k = 0; k < b; k++) {
      TrialState_cIneq = TrialState->cIneq.data[k];
      if (TrialState_cIneq > 0.0) {
        constrViolationIneq += TrialState_cIneq;
      }
    }

    constrViolationEq += constrViolationIneq;
    constrViolationIneq = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    constrViolationIneq += constrViolationEq;
    if ((constrViolationIneq > 2.2204460492503131E-16) && (TrialState->fstar >
         0.0)) {
      if (TrialState->sqpFval == 0.0) {
        penaltyParamTrial = 1.0;
      } else {
        penaltyParamTrial = 1.5;
      }

      penaltyParamTrial = penaltyParamTrial * TrialState->fstar /
        constrViolationIneq;
    }

    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi = penaltyParamTrial * constrViolationEq +
        TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * penaltyParamTrial +
           MeritFunction->initFval) - MeritFunction->phi > static_cast<real_T>
          (MeritFunction->nPenaltyDecreases) * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        if (penaltyParamTrial >= 1.0E-10) {
          MeritFunction->penaltyParam = penaltyParamTrial;
        } else {
          MeritFunction->penaltyParam = 1.0E-10;
        }
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
          TrialState->sqpFval;
      }
    } else {
      if (penaltyParamTrial >= 1.0E-10) {
        MeritFunction->penaltyParam = penaltyParamTrial;
      } else {
        MeritFunction->penaltyParam = 1.0E-10;
      }

      MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
        TrialState->sqpFval;
    }

    constrViolationEq = TrialState->fstar - MeritFunction->penaltyParam *
      constrViolationEq;
    if (constrViolationEq <= 0.0) {
      MeritFunction->phiPrimePlus = constrViolationEq;
    } else {
      MeritFunction->phiPrimePlus = 0.0;
    }
  } else if (TrialState->state != -6) {
    stepFlags->stepType = 2;
  }

  AEBController_sortLambdaQP(TrialState->lambda.data, WorkingSet->nActiveConstr,
    WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid.data,
    WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
  isEqAndIneqFeasible = (WorkingSet->mEqRemoved > 0);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    AEBController_addAeqConstr(WorkingSet, WorkingSet->indexEqRemoved
      [WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  if (isEqAndIneqFeasible) {
    for (k = 0; k < 70; k++) {
      WorkingSet->Wlocalidx.data[WorkingSet->sizes[0] + k] = k + 1;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_relaxed(const real_T Hessian
  [5625], const real_T grad_data[], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective,
  somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions)
{
  real_T beta;
  real_T qpfvalQuadExcess;
  real_T qpfvalQuadExcess_tmp;
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T ix;
  int32_T mFiniteLBOrig;
  int32_T mLBOrig;
  int32_T mLBOrig_tmp;
  int32_T nVarOrig;
  boolean_T b_tf;
  boolean_T tf;
  nVarOrig = WorkingSet->nVar - 1;
  beta = 0.0;
  idx_max = static_cast<uint16_T>(WorkingSet->nVar);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
    beta += Hessian[75 * mFiniteLBOrig + mFiniteLBOrig];
  }

  beta /= static_cast<real_T>(WorkingSet->nVar);
  if (TrialState->sqpIterations <= 1) {
    mLBOrig = QPObjective->nvar;
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective->nvar > 1) {
        smax = std::abs(grad_data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
          s = std::abs(grad_data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    s = std::abs(grad_data[idx_max - 1]);
    if ((s <= 1.0) || rtIsNaN(s)) {
      s = 1.0;
    }

    smax = 100.0 * s;
  } else {
    mLBOrig = WorkingSet->mConstr;
    if (WorkingSet->mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet->mConstr > 1) {
        smax = std::abs(TrialState->lambdasqp.data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
          s = std::abs(TrialState->lambdasqp.data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    smax = std::abs(TrialState->lambdasqp.data[idx_max - 1]);
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = smax;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  AEBController_DW.c_WorkingSet = *WorkingSet;
  AEBController_setProblemType(&AEBController_DW.c_WorkingSet, 2);
  idx_max = AEBController_DW.c_WorkingSet.sizes[2] + 1;
  mLBOrig = (AEBController_DW.c_WorkingSet.sizes[3] -
             AEBController_DW.c_WorkingSet.sizes[2]) - 140;
  ix = static_cast<uint16_T>(AEBController_DW.c_WorkingSet.sizes[2]);
  if (ix - 1 >= 0) {
    std::memcpy(&memspace->workspace_float.data[0],
                &AEBController_DW.c_WorkingSet.bineq.data[0],
                static_cast<uint32_T>(ix) * sizeof(real_T));
  }

  AEBController_xgemv_jtygqg(WorkingSet->nVar,
    AEBController_DW.c_WorkingSet.sizes[2],
    AEBController_DW.c_WorkingSet.Aineq.data, AEBController_DW.c_WorkingSet.ldA,
    TrialState->xstar.data, memspace->workspace_float.data);
  ix = static_cast<uint8_T>(AEBController_DW.c_WorkingSet.sizes[2]);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < ix; mFiniteLBOrig++) {
    TrialState->xstar.data[(nVarOrig + mFiniteLBOrig) + 1] = static_cast<real_T>
      (memspace->workspace_float.data[mFiniteLBOrig] > 0.0) *
      memspace->workspace_float.data[mFiniteLBOrig];
  }

  std::memcpy(&memspace->workspace_float.data[0],
              &AEBController_DW.c_WorkingSet.beq[0], 70U * sizeof(real_T));
  AEBController_xgemv_jtygqg(WorkingSet->nVar, 70,
    AEBController_DW.c_WorkingSet.Aeq.data, AEBController_DW.c_WorkingSet.ldA,
    TrialState->xstar.data, memspace->workspace_float.data);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 70; mFiniteLBOrig++) {
    ix = idx_max + mFiniteLBOrig;
    if (memspace->workspace_float.data[mFiniteLBOrig] <= 0.0) {
      TrialState->xstar.data[nVarOrig + ix] = 0.0;
      TrialState->xstar.data[(nVarOrig + ix) + 70] =
        -memspace->workspace_float.data[mFiniteLBOrig];
      AEBController_addBoundToActiveSetMatrix_(&AEBController_DW.c_WorkingSet, 4,
        mLBOrig + ix);
      if (memspace->workspace_float.data[mFiniteLBOrig] >= -1.0E-6) {
        AEBController_addBoundToActiveSetMatrix_(&AEBController_DW.c_WorkingSet,
          4, (mLBOrig + ix) + 70);
      }
    } else {
      mLBOrig_tmp = nVarOrig + ix;
      TrialState->xstar.data[mLBOrig_tmp] = memspace->
        workspace_float.data[mFiniteLBOrig];
      TrialState->xstar.data[mLBOrig_tmp + 70] = 0.0;
      AEBController_addBoundToActiveSetMatrix_(&AEBController_DW.c_WorkingSet, 4,
        (mLBOrig + ix) + 70);
      if (memspace->workspace_float.data[mFiniteLBOrig] <= 1.0E-6) {
        AEBController_addBoundToActiveSetMatrix_(&AEBController_DW.c_WorkingSet,
          4, mLBOrig + ix);
      }
    }
  }

  nVarOrig = qpoptions->MaxIterations;
  qpoptions->MaxIterations = (qpoptions->MaxIterations +
    AEBController_DW.c_WorkingSet.nVar) - WorkingSet->nVar;
  AEBController_driver_g(Hessian, grad_data, TrialState, memspace,
    &AEBController_DW.c_WorkingSet, QRManager, CholManager, QPObjective,
    qpoptions, qpoptions);
  qpoptions->MaxIterations = nVarOrig;
  idx_max = AEBController_DW.c_WorkingSet.sizes[3] - 141;
  nVarOrig = 0;
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 70; mFiniteLBOrig++) {
    mLBOrig = (AEBController_DW.c_WorkingSet.isActiveIdx[3] + idx_max) +
      mFiniteLBOrig;
    tf = AEBController_DW.c_WorkingSet.isActiveConstr.data[mLBOrig];
    b_tf = AEBController_DW.c_WorkingSet.isActiveConstr.data[mLBOrig + 70];
    memspace->workspace_int.data[mFiniteLBOrig] = tf;
    memspace->workspace_int.data[mFiniteLBOrig + 70] = b_tf;
    nVarOrig = (nVarOrig + tf) + b_tf;
  }

  mLBOrig = static_cast<uint8_T>(AEBController_DW.c_WorkingSet.sizes[2]);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < mLBOrig; mFiniteLBOrig++) {
    tf = AEBController_DW.c_WorkingSet.isActiveConstr.data
      [((AEBController_DW.c_WorkingSet.isActiveIdx[3] + idx_max) -
        AEBController_DW.c_WorkingSet.sizes[2]) + mFiniteLBOrig];
    memspace->workspace_int.data[mFiniteLBOrig + 140] = tf;
    nVarOrig += tf;
  }

  if (TrialState->state != -6) {
    idx_max = (AEBController_DW.c_WorkingSet.nVarMax - WorkingSet->nVar) - 1;
    mLBOrig_tmp = WorkingSet->nVar + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (idx_max >= 1) {
      ix = WorkingSet->nVar + idx_max;
      for (mFiniteLBOrig = mLBOrig_tmp; mFiniteLBOrig <= ix; mFiniteLBOrig++) {
        s += std::abs(TrialState->xstar.data[mFiniteLBOrig - 1]);
      }

      idx_max = static_cast<uint16_T>(idx_max);
      for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
        qpfvalQuadExcess_tmp = TrialState->xstar.data[WorkingSet->nVar +
          mFiniteLBOrig];
        qpfvalQuadExcess += qpfvalQuadExcess_tmp * qpfvalQuadExcess_tmp;
      }
    }

    beta = (TrialState->fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    mLBOrig = (WorkingSet->nVarMax - WorkingSet->nVar) - 1;
    smax = MeritFunction->penaltyParam;
    s = 0.0;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 70; mFiniteLBOrig++) {
      s += std::abs(TrialState->cEq[mFiniteLBOrig]);
    }

    qpfvalQuadExcess = 0.0;
    ix = static_cast<uint8_T>(WorkingSet->sizes[2]);
    for (mFiniteLBOrig = 0; mFiniteLBOrig < ix; mFiniteLBOrig++) {
      qpfvalQuadExcess_tmp = TrialState->cIneq.data[mFiniteLBOrig];
      if (qpfvalQuadExcess_tmp > 0.0) {
        qpfvalQuadExcess += qpfvalQuadExcess_tmp;
      }
    }

    s += qpfvalQuadExcess;
    qpfvalQuadExcess = MeritFunction->linearizedConstrViol;
    qpfvalQuadExcess_tmp = 0.0;
    if (mLBOrig >= 1) {
      mLBOrig += WorkingSet->nVar;
      for (mFiniteLBOrig = mLBOrig_tmp; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig
           ++) {
        qpfvalQuadExcess_tmp += std::abs(TrialState->xstar.data[mFiniteLBOrig -
          1]);
      }
    }

    MeritFunction->linearizedConstrViol = qpfvalQuadExcess_tmp;
    qpfvalQuadExcess = (s + qpfvalQuadExcess) - qpfvalQuadExcess_tmp;
    if ((qpfvalQuadExcess > 2.2204460492503131E-16) && (beta > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        smax = 1.0;
      } else {
        smax = 1.5;
      }

      smax = smax * beta / qpfvalQuadExcess;
    }

    if (smax < MeritFunction->penaltyParam) {
      MeritFunction->phi = smax * s + TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * smax +
           MeritFunction->initFval) - MeritFunction->phi > static_cast<real_T>
          (MeritFunction->nPenaltyDecreases) * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        if (smax >= 1.0E-10) {
          MeritFunction->penaltyParam = smax;
        } else {
          MeritFunction->penaltyParam = 1.0E-10;
        }
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * s +
          TrialState->sqpFval;
      }
    } else {
      if (smax >= 1.0E-10) {
        MeritFunction->penaltyParam = smax;
      } else {
        MeritFunction->penaltyParam = 1.0E-10;
      }

      MeritFunction->phi = MeritFunction->penaltyParam * s + TrialState->sqpFval;
    }

    beta -= MeritFunction->penaltyParam * s;
    if (beta <= 0.0) {
      MeritFunction->phiPrimePlus = beta;
    } else {
      MeritFunction->phiPrimePlus = 0.0;
    }

    idx_max = AEBController_DW.c_WorkingSet.isActiveIdx[1] - 1;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 70; mFiniteLBOrig++) {
      if (memspace->workspace_int.data[mFiniteLBOrig] != 0) {
        tf = (memspace->workspace_int.data[mFiniteLBOrig + 70] != 0);
      } else {
        tf = false;
      }

      mLBOrig_tmp = idx_max + mFiniteLBOrig;
      TrialState->lambda.data[mLBOrig_tmp] *= static_cast<real_T>(tf);
    }

    idx_max = AEBController_DW.c_WorkingSet.isActiveIdx[2];
    mLBOrig = AEBController_DW.c_WorkingSet.nActiveConstr;
    for (mFiniteLBOrig = idx_max; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++) {
      if (AEBController_DW.c_WorkingSet.Wid.data[mFiniteLBOrig - 1] == 3) {
        TrialState->lambda.data[mFiniteLBOrig - 1] *= static_cast<real_T>
          (memspace->
           workspace_int.data[AEBController_DW.c_WorkingSet.Wlocalidx.data[mFiniteLBOrig
           - 1] + 139]);
      }
    }
  }

  mFiniteLBOrig = (AEBController_DW.c_WorkingSet.sizes[3] -
                   AEBController_DW.c_WorkingSet.sizes[2]) - 140;
  idx_max = AEBController_DW.c_WorkingSet.nActiveConstr;
  while ((idx_max > AEBController_DW.c_WorkingSet.sizes[0] + 70) && (nVarOrig >
          0)) {
    if ((AEBController_DW.c_WorkingSet.Wid.data[idx_max - 1] == 4) &&
        (AEBController_DW.c_WorkingSet.Wlocalidx.data[idx_max - 1] >
         mFiniteLBOrig)) {
      beta = TrialState->lambda.data[AEBController_DW.c_WorkingSet.nActiveConstr
        - 1];
      TrialState->lambda.data[AEBController_DW.c_WorkingSet.nActiveConstr - 1] =
        0.0;
      TrialState->lambda.data[idx_max - 1] = beta;
      AEBController_removeConstr(&AEBController_DW.c_WorkingSet, idx_max);
      nVarOrig--;
    }

    idx_max--;
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  *WorkingSet = AEBController_DW.c_WorkingSet;
  AEBController_setProblemType(WorkingSet, 3);
  AEBController_sortLambdaQP(TrialState->lambda.data, WorkingSet->nActiveConstr,
    WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid.data,
    WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_step
  (s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *stepFlags, real_T Hessian[5625],
   const real_T lb[75], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
   sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
   s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
   s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
   s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective,
   somzaGboVhDG7PNQS6E98jD_AEBController_T *qpoptions)
{
  real_T nrmDirInf;
  real_T nrmGradInf;
  real_T u1;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T k;
  int32_T nVar;
  boolean_T checkBoundViolation;
  boolean_T guard1;
  stepFlags->stepAccepted = true;
  checkBoundViolation = true;
  nVar = WorkingSet->nVar - 1;
  if (stepFlags->stepType != 3) {
    std::memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0],
                static_cast<uint16_T>(WorkingSet->nVar) * sizeof(real_T));
  } else if (WorkingSet->nVar - 1 >= 0) {
    std::memcpy(&TrialState->searchDir.data[0], &TrialState->xstar.data[0],
                static_cast<uint32_T>((WorkingSet->nVar - 1) + 1) * sizeof
                (real_T));
  }

  do {
    exitg1 = 0;
    guard1 = false;
    switch (stepFlags->stepType) {
     case 1:
      AEBController_normal(Hessian, TrialState->grad.data, TrialState,
                           MeritFunction, memspace, WorkingSet, QRManager,
                           CholManager, QPObjective, qpoptions, stepFlags);
      if (stepFlags->stepType == 2) {
      } else {
        if (nVar >= 0) {
          std::memcpy(&TrialState->delta_x.data[0], &TrialState->xstar.data[0],
                      static_cast<uint32_T>(nVar + 1) * sizeof(real_T));
        }

        guard1 = true;
      }
      break;

     case 2:
      idxStartIneq_tmp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = idxStartIneq_tmp + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (k = idxStartIneq; k <= idxEndIneq; k++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[k - 1] - 1] + WorkingSet->Wlocalidx.data[k - 1])
          - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = idxStartIneq_tmp;
      idxStartIneq_tmp = TrialState->xstar.size[0];
      k = TrialState->xstar.size[0];
      if (k - 1 >= 0) {
        std::memcpy(&AEBController_DW.b_data[0], &TrialState->xstar.data[0],
                    static_cast<uint32_T>(k) * sizeof(real_T));
      }

      idxStartIneq = static_cast<uint16_T>(WorkingSet->sizes[3]);
      for (k = 0; k < idxStartIneq; k++) {
        nrmGradInf = WorkingSet->lb.data[WorkingSet->indexLB.data[k] - 1];
        if (-AEBController_DW.b_data[WorkingSet->indexLB.data[k] - 1] >
            nrmGradInf) {
          AEBController_DW.b_data[WorkingSet->indexLB.data[k] - 1] = -nrmGradInf
            + std::abs(nrmGradInf);
        }
      }

      if (idxStartIneq_tmp - 1 >= 0) {
        std::memcpy(&TrialState->xstar.data[0], &AEBController_DW.b_data[0],
                    static_cast<uint32_T>(idxStartIneq_tmp) * sizeof(real_T));
      }

      AEBController_relaxed(Hessian, TrialState->grad.data, TrialState,
                            MeritFunction, memspace, WorkingSet, QRManager,
                            CholManager, QPObjective, qpoptions);
      if (nVar >= 0) {
        std::memcpy(&TrialState->delta_x.data[0], &TrialState->xstar.data[0],
                    static_cast<uint32_T>(nVar + 1) * sizeof(real_T));
      }

      guard1 = true;
      break;

     default:
      checkBoundViolation = AEBController_soc(Hessian, TrialState->grad.data,
        TrialState, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        qpoptions);
      stepFlags->stepAccepted = checkBoundViolation;
      if (stepFlags->stepAccepted && (TrialState->state != -6)) {
        idxStartIneq = static_cast<uint16_T>(nVar + 1);
        for (k = 0; k < idxStartIneq; k++) {
          TrialState->delta_x.data[k] = TrialState->xstar.data[k] +
            TrialState->socDirection.data[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (k = 0; k < 75; k++) {
          u1 = std::abs(TrialState->grad.data[k]);
          if ((!(nrmGradInf >= u1)) && (!rtIsNaN(u1))) {
            nrmGradInf = u1;
          }

          u1 = std::abs(TrialState->xstar.data[k]);
          if ((!(nrmDirInf >= u1)) && (!rtIsNaN(u1))) {
            nrmDirInf = u1;
          }
        }

        nrmGradInf /= nrmDirInf;
        if ((nrmGradInf <= 2.2204460492503131E-16) || rtIsNaN(nrmGradInf)) {
          nrmGradInf = 2.2204460492503131E-16;
        }

        for (k = 0; k < 75; k++) {
          idxEndIneq = 75 * k;
          for (idxStartIneq = 0; idxStartIneq < k; idxStartIneq++) {
            Hessian[idxEndIneq + idxStartIneq] = 0.0;
          }

          idxEndIneq = 75 * k + k;
          Hessian[idxEndIneq] = nrmGradInf;
          idxStartIneq_tmp = 73 - k;
          for (idxStartIneq = 0; idxStartIneq <= idxStartIneq_tmp; idxStartIneq
               ++) {
            Hessian[(idxEndIneq + idxStartIneq) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    idxStartIneq_tmp = TrialState->delta_x.size[0];
    k = TrialState->delta_x.size[0];
    if (k - 1 >= 0) {
      std::memcpy(&AEBController_DW.b_data[0], &TrialState->delta_x.data[0],
                  static_cast<uint32_T>(k) * sizeof(real_T));
    }

    k = static_cast<uint16_T>(WorkingSet->sizes[3]);
    for (nVar = 0; nVar < k; nVar++) {
      nrmDirInf = AEBController_DW.b_data[WorkingSet->indexLB.data[nVar] - 1];
      nrmGradInf = (TrialState->xstarsqp[WorkingSet->indexLB.data[nVar] - 1] +
                    nrmDirInf) - lb[WorkingSet->indexLB.data[nVar] - 1];
      if (nrmGradInf < 0.0) {
        AEBController_DW.b_data[WorkingSet->indexLB.data[nVar] - 1] = nrmDirInf
          - nrmGradInf;
        TrialState->xstar.data[WorkingSet->indexLB.data[nVar] - 1] -= nrmGradInf;
      }
    }

    if (idxStartIneq_tmp - 1 >= 0) {
      std::memcpy(&TrialState->delta_x.data[0], &AEBController_DW.b_data[0],
                  static_cast<uint32_T>(idxStartIneq_tmp) * sizeof(real_T));
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_evalObjAndConstr(int32_T
  obj_next_next_next_next_next_b_value, const
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
  *obj_next_next_next_next_next_next_next_b_value_workspace_runtim, const
  s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
  *obj_next_next_next_next_next_next_next_next_b_value_workspace, const real_T
  x[75], real_T Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[70],
  real_T *fval, int32_T *status)
{
  real_T X[77];
  real_T b_X_tmp[77];
  real_T reshapes_f1[70];
  real_T c[60];
  real_T varargin_1_data[60];
  real_T U[44];
  real_T b_U_tmp[44];
  real_T ic[7];
  real_T tmp_0[7];
  real_T tmp_1[7];
  real_T b_yk_idx_0;
  real_T b_yk_idx_1;
  real_T b_yk_idx_2;
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T e;
  real_T obj_next_next_next_next_next_next_next_b_value_workspace_runt_0;
  real_T obj_next_next_next_next_next_next_next_b_value_workspace_runt_1;
  real_T umvk;
  real_T umvk_idx_0;
  real_T umvk_idx_1;
  real_T wtYerr_idx_0;
  int32_T ineqRange_data[60];
  int32_T icf_tmp[6];
  int32_T b_U_tmp_tmp;
  int32_T ineqRange_size_idx_1;
  int32_T n;
  int32_T yk;
  int8_T tmp_data[60];
  int8_T sizes[2];
  boolean_T icf[60];
  boolean_T tmp[30];
  boolean_T icf_0[6];
  boolean_T b_x[3];
  boolean_T exitg1;
  boolean_T y;
  getXUe_eNtkTHmx(x,
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.x,
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.md, X, U, &e);
  wtYerr_idx_0 = 0.0;
  for (n = 0; n < 7; n++) {
    for (yk = 0; yk < 11; yk++) {
      b_X_tmp[n + 7 * yk] = X[11 * n + yk];
    }
  }

  for (yk = 0; yk < 11; yk++) {
    b_U_tmp_tmp = yk << 2;
    b_U_tmp[b_U_tmp_tmp] = U[yk];
    b_U_tmp[b_U_tmp_tmp + 1] = U[yk + 11];
    b_U_tmp[b_U_tmp_tmp + 2] = U[yk + 22];
    b_U_tmp[b_U_tmp_tmp + 3] = U[yk + 33];
  }

  for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 10; b_U_tmp_tmp++) {
    n = (b_U_tmp_tmp + 1) * 7;
    umvk_idx_1 = (b_X_tmp[n + 2] / 15.0 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[b_U_tmp_tmp] / 15.0) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [b_U_tmp_tmp];
    duk_idx_1 = umvk_idx_1 * umvk_idx_1;
    umvk_idx_1 = (b_X_tmp[n + 4] / 0.5 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[b_U_tmp_tmp + 10] / 0.5) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [b_U_tmp_tmp + 10];
    duk_idx_1 += umvk_idx_1 * umvk_idx_1;
    umvk_idx_1 = ((b_X_tmp[n + 5] + b_X_tmp[n + 6]) / 0.5 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.ref[b_U_tmp_tmp + 20] / 0.5) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.OutputWeights
      [b_U_tmp_tmp + 20];
    wtYerr_idx_0 += umvk_idx_1 * umvk_idx_1 + duk_idx_1;
    n = b_U_tmp_tmp << 2;
    umvk_idx_0 = b_U_tmp[n] / 6.0;
    umvk_idx_1 = b_U_tmp[n + 1] / 2.26;
    if (b_U_tmp_tmp + 1 == 1) {
      duk_idx_0 = umvk_idx_0 -
        obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.lastMV
        [0] / 6.0;
      duk_idx_1 = umvk_idx_1 -
        obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.lastMV
        [1] / 2.26;
    } else {
      n = (b_U_tmp_tmp - 1) << 2;
      duk_idx_0 = umvk_idx_0 - b_U_tmp[n] / 6.0;
      duk_idx_1 = umvk_idx_1 - b_U_tmp[n + 1] / 2.26;
    }

    umvk_idx_0 = (umvk_idx_0 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.MVScaledTarget[b_U_tmp_tmp]) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVWeights
      [b_U_tmp_tmp];
    duk_idx_0 *=
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVRateWeights
      [b_U_tmp_tmp];
    umvk = umvk_idx_0 * umvk_idx_0;
    duk = duk_idx_0 * duk_idx_0;
    umvk_idx_0 = (umvk_idx_1 -
                  obj_next_next_next_next_next_next_next_next_b_value_workspace->
                  runtimedata.MVScaledTarget[b_U_tmp_tmp + 10]) *
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVWeights
      [b_U_tmp_tmp + 10];
    duk_idx_0 =
      obj_next_next_next_next_next_next_next_next_b_value_workspace->runtimedata.MVRateWeights
      [b_U_tmp_tmp + 10] * duk_idx_1;
    wtYerr_idx_0 = ((umvk_idx_0 * umvk_idx_0 + umvk) + wtYerr_idx_0) +
      (duk_idx_0 * duk_idx_0 + duk);
  }

  *fval = 100000.0 * e * e + wtYerr_idx_0;
  y = rtIsNaN(*fval);
  if (rtIsInf(*fval) || y) {
    if (y) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    if (obj_next_next_next_next_next_b_value - 1 < 0) {
      n = 0;
    } else {
      n = static_cast<uint8_T>(obj_next_next_next_next_next_b_value - 1) + 1;
    }

    ineqRange_size_idx_1 = n;
    if (n > 0) {
      ineqRange_data[0] = 0;
      yk = 0;
      for (b_U_tmp_tmp = 2; b_U_tmp_tmp <= n; b_U_tmp_tmp++) {
        yk++;
        ineqRange_data[b_U_tmp_tmp - 1] = yk;
      }
    }

    b_U_tmp_tmp = n - 1;
    for (n = 0; n <= b_U_tmp_tmp; n++) {
      ineqRange_data[n] += ineq0;
    }

    getXUe_eNtkTHmx(x,
                    obj_next_next_next_next_next_next_next_b_value_workspace_runtim
                    ->x,
                    obj_next_next_next_next_next_next_next_b_value_workspace_runtim
                    ->md, X, U, &e);
    for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 70; b_U_tmp_tmp++) {
      reshapes_f1[b_U_tmp_tmp] = 0.0;
    }

    for (n = 0; n < 7; n++) {
      ic[n] = static_cast<real_T>(n) + 1.0;
    }

    for (n = 0; n < 11; n++) {
      b_U_tmp_tmp = n << 2;
      b_U_tmp[b_U_tmp_tmp] = U[n];
      b_U_tmp[b_U_tmp_tmp + 1] = U[n + 11];
      b_U_tmp[b_U_tmp_tmp + 2] = U[n + 22];
      b_U_tmp[b_U_tmp_tmp + 3] = U[n + 33];
    }

    for (yk = 0; yk < 7; yk++) {
      for (n = 0; n < 11; n++) {
        b_X_tmp[yk + 7 * n] = X[11 * yk + n];
      }
    }

    for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 10; b_U_tmp_tmp++) {
      n = b_U_tmp_tmp << 2;
      helperNLMPCStateFcn_pC1vAxkb(&b_X_tmp[7 * b_U_tmp_tmp], &b_U_tmp[n],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [0],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [1],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [2],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [3],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [4],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [5],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [6], tmp_0);
      yk = (b_U_tmp_tmp + 1) * 7;
      helperNLMPCStateFcn_pC1vAxkb(&b_X_tmp[yk], &b_U_tmp[n],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [0],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [1],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [2],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [3],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [4],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [5],
        obj_next_next_next_next_next_next_next_b_value_workspace_runtim->Parameters
        [6], tmp_1);
      for (n = 0; n < 7; n++) {
        wtYerr_idx_0 = ic[n];
        reshapes_f1[static_cast<int32_T>(wtYerr_idx_0) - 1] = (b_X_tmp[7 *
          b_U_tmp_tmp + n] + (tmp_0[n] + tmp_1[n]) * 0.025) - b_X_tmp[yk + n];
        ic[n] = wtYerr_idx_0 + 7.0;
      }
    }

    for (n = 0; n < 30; n++) {
      tmp[n] = rtIsInf
        (obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin
         [n]);
    }

    all_WqXfAvPF(tmp, b_x);
    y = true;
    b_U_tmp_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (b_U_tmp_tmp < 3)) {
      if (!b_x[b_U_tmp_tmp]) {
        y = false;
        exitg1 = true;
      } else {
        b_U_tmp_tmp++;
      }
    }

    if (y) {
      for (n = 0; n < 30; n++) {
        tmp[n] = rtIsInf
          (obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax
           [n]);
      }

      all_WqXfAvPF(tmp, b_x);
      b_U_tmp_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (b_U_tmp_tmp < 3)) {
        if (!b_x[b_U_tmp_tmp]) {
          y = false;
          exitg1 = true;
        } else {
          b_U_tmp_tmp++;
        }
      }
    }

    if (y) {
      yk = 0;
      b_U_tmp_tmp = 0;
    } else {
      for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 60; b_U_tmp_tmp++) {
        c[b_U_tmp_tmp] = 0.0;
        icf[b_U_tmp_tmp] = true;
      }

      wtYerr_idx_0 = 1.0;
      umvk_idx_1 = 2.0;
      duk_idx_1 = 3.0;
      for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 10; b_U_tmp_tmp++) {
        umvk_idx_0 =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin
          [b_U_tmp_tmp];
        icf[static_cast<int32_T>(wtYerr_idx_0) - 1] = ((!rtIsInf(umvk_idx_0)) &&
          (!rtIsNaN(umvk_idx_0)));
        duk_idx_0 =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin
          [b_U_tmp_tmp + 10];
        icf[static_cast<int32_T>(umvk_idx_1) - 1] = ((!rtIsInf(duk_idx_0)) &&
          (!rtIsNaN(duk_idx_0)));
        umvk =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMin
          [b_U_tmp_tmp + 20];
        icf[static_cast<int32_T>(duk_idx_1) - 1] = ((!rtIsInf(umvk)) &&
          (!rtIsNaN(umvk)));
        duk =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax
          [b_U_tmp_tmp];
        icf[static_cast<int32_T>(wtYerr_idx_0 + 3.0) - 1] = ((!rtIsInf(duk)) &&
          (!rtIsNaN(duk)));
        icf_tmp[0] = static_cast<int32_T>(wtYerr_idx_0) - 1;
        icf_tmp[3] = static_cast<int32_T>(wtYerr_idx_0 + 3.0) - 1;
        obj_next_next_next_next_next_next_next_b_value_workspace_runt_0 =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax
          [b_U_tmp_tmp + 10];
        icf[static_cast<int32_T>(umvk_idx_1 + 3.0) - 1] = ((!rtIsInf
          (obj_next_next_next_next_next_next_next_b_value_workspace_runt_0)) &&
          (!rtIsNaN
           (obj_next_next_next_next_next_next_next_b_value_workspace_runt_0)));
        icf_tmp[1] = static_cast<int32_T>(umvk_idx_1) - 1;
        icf_tmp[4] = static_cast<int32_T>(umvk_idx_1 + 3.0) - 1;
        obj_next_next_next_next_next_next_next_b_value_workspace_runt_1 =
          obj_next_next_next_next_next_next_next_b_value_workspace_runtim->OutputMax
          [b_U_tmp_tmp + 20];
        icf[static_cast<int32_T>(duk_idx_1 + 3.0) - 1] = ((!rtIsInf
          (obj_next_next_next_next_next_next_next_b_value_workspace_runt_1)) &&
          (!rtIsNaN
           (obj_next_next_next_next_next_next_next_b_value_workspace_runt_1)));
        icf_tmp[2] = static_cast<int32_T>(duk_idx_1) - 1;
        icf_tmp[5] = static_cast<int32_T>(duk_idx_1 + 3.0) - 1;
        for (n = 0; n < 6; n++) {
          icf_0[n] = icf[icf_tmp[n]];
        }

        if (AEBController_any(icf_0)) {
          b_yk_idx_0 = X[b_U_tmp_tmp + 23] / 15.0;
          b_yk_idx_1 = X[b_U_tmp_tmp + 45] / 0.5;
          b_yk_idx_2 = (X[b_U_tmp_tmp + 56] + X[b_U_tmp_tmp + 67]) / 0.5;
          c[static_cast<int32_T>(wtYerr_idx_0) - 1] = (umvk_idx_0 - e) -
            b_yk_idx_0;
          c[static_cast<int32_T>(umvk_idx_1) - 1] = (duk_idx_0 - e) - b_yk_idx_1;
          c[static_cast<int32_T>(duk_idx_1) - 1] = (umvk - e) - b_yk_idx_2;
          c[static_cast<int32_T>(wtYerr_idx_0 + 3.0) - 1] = (b_yk_idx_0 - duk) -
            e;
          c[static_cast<int32_T>(umvk_idx_1 + 3.0) - 1] = (b_yk_idx_1 -
            obj_next_next_next_next_next_next_next_b_value_workspace_runt_0) - e;
          c[static_cast<int32_T>(duk_idx_1 + 3.0) - 1] = (b_yk_idx_2 -
            obj_next_next_next_next_next_next_next_b_value_workspace_runt_1) - e;
        }

        wtYerr_idx_0 += 6.0;
        umvk_idx_1 += 6.0;
        duk_idx_1 += 6.0;
      }

      n = 0;
      for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 60; b_U_tmp_tmp++) {
        if (icf[b_U_tmp_tmp]) {
          n++;
        }
      }

      yk = n;
      n = 0;
      for (b_U_tmp_tmp = 0; b_U_tmp_tmp < 60; b_U_tmp_tmp++) {
        if (icf[b_U_tmp_tmp]) {
          tmp_data[n] = static_cast<int8_T>(b_U_tmp_tmp);
          n++;
        }
      }

      b_U_tmp_tmp = 1;
      for (n = 0; n < yk; n++) {
        varargin_1_data[n] = c[tmp_data[n]];
      }
    }

    y = ((yk != 0) && (b_U_tmp_tmp != 0));
    if (!y) {
      sizes[0] = static_cast<int8_T>(yk);
    } else if (y) {
      sizes[0] = static_cast<int8_T>(yk);
    } else {
      sizes[0] = 0;
    }

    b_U_tmp_tmp = sizes[0];
    yk = y;
    for (n = 0; n < yk; n++) {
      if (b_U_tmp_tmp - 1 >= 0) {
        std::memcpy(&AEBController_DW.varargin_1_data[0], &varargin_1_data[0],
                    static_cast<uint32_T>(b_U_tmp_tmp) * sizeof(real_T));
      }
    }

    for (n = 0; n < ineqRange_size_idx_1; n++) {
      Cineq_workspace_data[ineqRange_data[n] - 1] =
        AEBController_DW.varargin_1_data[n];
    }

    std::memcpy(&Ceq_workspace[0], &reshapes_f1[0], 70U * sizeof(real_T));
    *status = AEBController_checkVectorNonFinite
      (obj_next_next_next_next_next_b_value, Cineq_workspace_data, ineq0);
    if (*status == 1) {
      *status = checkVectorNonFinite_KkAetmxy(Ceq_workspace);
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeLinearResiduals(const
  real_T x[75], int32_T nVar, real_T workspaceIneq_data[], const int32_T
  workspaceIneq_size[1], int32_T mLinIneq, const real_T AineqT_data[], const
  real_T bineq_data[], int32_T ldAi)
{
  if (mLinIneq > 0) {
    int32_T d;
    int32_T iy;
    int32_T loop_ub;
    loop_ub = workspaceIneq_size[0];
    if (loop_ub - 1 >= 0) {
      std::memcpy(&AEBController_DW.y_data_p[0], &workspaceIneq_data[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    std::memcpy(&AEBController_DW.y_data_p[0], &bineq_data[0],
                static_cast<uint32_T>(mLinIneq) * sizeof(real_T));
    if (loop_ub - 1 >= 0) {
      std::memcpy(&workspaceIneq_data[0], &AEBController_DW.y_data_p[0],
                  static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    }

    loop_ub = static_cast<uint8_T>(mLinIneq);
    for (int32_T k = 0; k < loop_ub; k++) {
      workspaceIneq_data[k] = -workspaceIneq_data[k];
    }

    iy = 0;
    d = (mLinIneq - 1) * ldAi + 1;
    for (int32_T k = 1; ldAi < 0 ? k >= d : k <= d; k += ldAi) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (k + nVar) - 1;
      for (loop_ub = k; loop_ub <= e; loop_ub++) {
        c += AineqT_data[loop_ub - 1] * x[loop_ub - k];
      }

      workspaceIneq_data[iy] += c;
      iy++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_computeMeritFcn(real_T
  obj_penaltyParam, real_T fval, const real_T Cineq_workspace_data[], int32_T
  mIneq, const real_T Ceq_workspace[70], boolean_T evalWellDefined)
{
  real_T val;
  if (evalWellDefined) {
    real_T constrViolationEq;
    real_T constrViolationIneq;
    int32_T k;
    constrViolationEq = 0.0;
    for (k = 0; k < 70; k++) {
      constrViolationEq += std::abs(Ceq_workspace[k]);
    }

    constrViolationIneq = 0.0;
    k = static_cast<uint8_T>(mIneq);
    for (int32_T idx = 0; idx < k; idx++) {
      real_T Cineq_workspace;
      Cineq_workspace = Cineq_workspace_data[idx];
      if (Cineq_workspace > 0.0) {
        constrViolationIneq += Cineq_workspace;
      }
    }

    val = (constrViolationEq + constrViolationIneq) * obj_penaltyParam + fval;
  } else {
    val = (rtInf);
  }

  return val;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_linesearch(boolean_T
  *evalWellDefined, const real_T bineq_data[], int32_T WorkingSet_nVar, int32_T
  WorkingSet_ldA, const real_T WorkingSet_Aineq_data[],
  s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState, real_T
  MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
  MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
  FcnEvaluator_next_next_next_next_next_b_value, const
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T
  *FcnEvaluator_next_next_next_next_next_next_next_b_value_workspa, const
  s_tMUA2vzIUjpbmJ8UK22wsG_AEBController_T
  *FcnEvaluator_next_next_next_next_next_next_next_next_b_value_wo, boolean_T
  socTaken, real_T *alpha, int32_T *exitflag)
{
  real_T phi_alpha;
  int32_T b_tmp;
  int32_T exitg1;
  int32_T k;
  int32_T mLinIneq;
  boolean_T exitg2;
  boolean_T tooSmallX;
  mLinIneq = TrialState->mIneq - TrialState->mNonlinIneq;
  *alpha = 1.0;
  *exitflag = 1;
  phi_alpha = MeritFunction_phiFullStep;
  if (WorkingSet_nVar - 1 >= 0) {
    std::memcpy(&TrialState->searchDir.data[0], &TrialState->delta_x.data[0],
                static_cast<uint32_T>(WorkingSet_nVar) * sizeof(real_T));
  }

  do {
    exitg1 = 0;
    if (TrialState->FunctionEvaluations < 7500) {
      if ((*evalWellDefined) && (phi_alpha <= *alpha * 0.0001 *
           MeritFunction_phiPrimePlus + MeritFunction_phi)) {
        exitg1 = 1;
      } else {
        *alpha *= 0.7;
        b_tmp = static_cast<uint16_T>(WorkingSet_nVar);
        for (k = 0; k < b_tmp; k++) {
          TrialState->delta_x.data[k] = *alpha * TrialState->xstar.data[k];
        }

        if (socTaken) {
          phi_alpha = *alpha * *alpha;
          if ((WorkingSet_nVar >= 1) && (!(phi_alpha == 0.0))) {
            for (k = 0; k < WorkingSet_nVar; k++) {
              TrialState->delta_x.data[k] += phi_alpha *
                TrialState->socDirection.data[k];
            }
          }
        }

        tooSmallX = true;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= static_cast<uint16_T>(WorkingSet_nVar) - 1)) {
          phi_alpha = std::abs(TrialState->xstarsqp[k]);
          if ((phi_alpha <= 1.0) || rtIsNaN(phi_alpha)) {
            phi_alpha = 1.0;
          }

          if (1.0E-6 * phi_alpha <= std::abs(TrialState->delta_x.data[k])) {
            tooSmallX = false;
            exitg2 = true;
          } else {
            k++;
          }
        }

        if (tooSmallX) {
          *exitflag = -2;
          exitg1 = 1;
        } else {
          for (k = 0; k < b_tmp; k++) {
            TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k] +
              TrialState->delta_x.data[k];
          }

          AEBController_evalObjAndConstr
            (FcnEvaluator_next_next_next_next_next_b_value,
             FcnEvaluator_next_next_next_next_next_next_next_b_value_workspa,
             FcnEvaluator_next_next_next_next_next_next_next_next_b_value_wo,
             TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
             TrialState->cEq, &TrialState->sqpFval, &k);
          AEBController_computeLinearResiduals(TrialState->xstarsqp,
            WorkingSet_nVar, TrialState->cIneq.data, TrialState->cIneq.size,
            mLinIneq, WorkingSet_Aineq_data, bineq_data, WorkingSet_ldA);
          TrialState->FunctionEvaluations++;
          *evalWellDefined = (k == 1);
          phi_alpha = AEBController_computeMeritFcn(MeritFunction_penaltyParam,
            TrialState->sqpFval, TrialState->cIneq.data, TrialState->mIneq,
            TrialState->cEq, *evalWellDefined);
        }
      }
    } else {
      *exitflag = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S15>/NLMPC'
real_T ACCWithSensorFusionModelClass::AEBController_computeComplError(const
  int32_T fscales_lineq_constraint_size[1], const int32_T
  fscales_cineq_constraint_size[1], const real_T xCurrent[75], int32_T mIneq,
  const real_T cIneq_data[], const int32_T finiteLB_data[], int32_T mLB, const
  real_T lb[75], const int32_T finiteUB_data[], int32_T mUB, const real_T
  lambda_data[], int32_T iL0)
{
  real_T nlpComplError;
  int32_T mLinIneq;
  nlpComplError = 0.0;
  mLinIneq = fscales_lineq_constraint_size[0];
  if ((mIneq + mLB) + mUB > 0) {
    real_T lbDelta;
    real_T lbLambda;
    real_T u0;
    int32_T iLineq0;
    int32_T idx;
    for (idx = 0; idx < mLinIneq; idx++) {
      lbDelta = cIneq_data[idx];
      lbLambda = lambda_data[(iL0 + idx) - 1];
      u0 = std::abs(lbDelta);
      if ((!(u0 <= lbLambda)) && (!rtIsNaN(lbLambda))) {
        u0 = lbLambda;
      }

      lbDelta = std::abs(lbDelta * lbLambda);
      if ((lbDelta <= u0) || rtIsNaN(u0)) {
        u0 = lbDelta;
      }

      if ((!(nlpComplError >= u0)) && (!rtIsNaN(u0))) {
        nlpComplError = u0;
      }
    }

    iLineq0 = (iL0 + fscales_lineq_constraint_size[0]) - 1;
    idx = fscales_cineq_constraint_size[0];
    for (int32_T b_idx = 0; b_idx < idx; b_idx++) {
      lbDelta = cIneq_data[mLinIneq + b_idx];
      lbLambda = lambda_data[iLineq0 + b_idx];
      u0 = std::abs(lbDelta);
      if ((!(u0 <= lbLambda)) && (!rtIsNaN(lbLambda))) {
        u0 = lbLambda;
      }

      lbDelta = std::abs(lbDelta * lbLambda);
      if ((lbDelta <= u0) || rtIsNaN(u0)) {
        u0 = lbDelta;
      }

      if ((!(nlpComplError >= u0)) && (!rtIsNaN(u0))) {
        nlpComplError = u0;
      }
    }

    iLineq0 = (iL0 + mIneq) - 1;
    mLinIneq = iLineq0 + mLB;
    idx = static_cast<uint16_T>(mLB);
    for (int32_T b_idx = 0; b_idx < idx; b_idx++) {
      int32_T finiteLB;
      finiteLB = finiteLB_data[b_idx];
      lbDelta = xCurrent[finiteLB - 1] - lb[finiteLB - 1];
      lbLambda = lambda_data[iLineq0 + b_idx];
      u0 = std::abs(lbDelta);
      if ((!(u0 <= lbLambda)) && (!rtIsNaN(lbLambda))) {
        u0 = lbLambda;
      }

      lbDelta = std::abs(lbDelta * lbLambda);
      if ((lbDelta <= u0) || rtIsNaN(u0)) {
        u0 = lbDelta;
      }

      if ((!(nlpComplError >= u0)) && (!rtIsNaN(u0))) {
        nlpComplError = u0;
      }
    }

    iLineq0 = static_cast<uint16_T>(mUB);
    for (idx = 0; idx < iLineq0; idx++) {
      lbDelta = lambda_data[mLinIneq + idx];
      u0 = (rtInf) - xCurrent[finiteUB_data[idx] - 1];
      if ((u0 <= lbDelta) || rtIsNaN(lbDelta)) {
        lbLambda = u0;
      } else {
        lbLambda = lbDelta;
      }

      u0 = std::abs(u0 * lbDelta);
      if ((u0 <= lbLambda) || rtIsNaN(lbLambda)) {
        lbLambda = u0;
      }

      if ((!(nlpComplError >= lbLambda)) && (!rtIsNaN(lbLambda))) {
        nlpComplError = lbLambda;
      }
    }
  }

  return nlpComplError;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeGradLag_b(real_T
  workspace_data[], int32_T ldA, int32_T nVar, const real_T grad_data[], int32_T
  mIneq, const real_T AineqTrans_data[], const real_T AeqTrans_data[], const
  int32_T finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[],
  int32_T mLB, const int32_T finiteUB_data[], int32_T mUB, const real_T
  lambda_data[])
{
  int32_T f;
  int32_T g;
  int32_T i;
  int32_T iL0;
  int32_T ix;
  int32_T tmp;
  std::memcpy(&workspace_data[0], &grad_data[0], static_cast<uint16_T>(nVar) *
              sizeof(real_T));
  i = static_cast<uint16_T>(mFixed);
  for (iL0 = 0; iL0 < i; iL0++) {
    ix = finiteFixed_data[iL0];
    workspace_data[ix - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 69 + 1;
  for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
    g = (iL0 + nVar) - 1;
    for (i = iL0; i <= g; i++) {
      tmp = i - iL0;
      workspace_data[tmp] += AeqTrans_data[i - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 70;
    f = (mIneq - 1) * ldA + 1;
    for (iL0 = 1; ldA < 0 ? iL0 >= f : iL0 <= f; iL0 += ldA) {
      g = (iL0 + nVar) - 1;
      for (i = iL0; i <= g; i++) {
        tmp = i - iL0;
        workspace_data[tmp] += AineqTrans_data[i - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 70;
  ix = static_cast<uint16_T>(mLB) - 1;
  for (i = 0; i <= ix; i++) {
    f = finiteLB_data[i];
    workspace_data[f - 1] -= lambda_data[iL0 + i];
  }

  iL0 = static_cast<uint16_T>(mLB) - 1 < 0 ? iL0 : static_cast<uint16_T>(mLB) +
    iL0;
  ix = static_cast<uint16_T>(mUB) - 1;
  for (i = 0; i <= ix; i++) {
    f = finiteUB_data[i];
    workspace_data[f - 1] += lambda_data[iL0 + i];
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_computeDualFeasError_c(int32_T
  nVar, const real_T gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= static_cast<uint16_T>(nVar) - 1)) {
    *gradOK = ((!rtIsInf(gradLag_data[idx])) && (!rtIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      real_T u1;
      u1 = std::abs(gradLag_data[idx]);
      if ((!(*val >= u1)) && (!rtIsNaN(u1))) {
        *val = u1;
      }

      idx++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_test_exit_n
  (s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T *Flags,
   s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
   sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction, const int32_T
   fscales_lineq_constraint_size[1], const int32_T
   fscales_cineq_constraint_size[1], s_H2r48KAwe9QhSMXfaacI5D_AEBController_T
   *WorkingSet, s_aN87P1cKlEBgtxywOa8jBC_AEBController_T *TrialState,
   s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager, const real_T lb[75])
{
  real_T nlpComplErrorTmp;
  real_T s;
  real_T smax;
  real_T tmp;
  real_T tmp_0;
  int32_T c_ix;
  int32_T iQR0;
  int32_T idx_max;
  int32_T ix;
  int32_T mLambda;
  int32_T n;
  int32_T nVar;
  int32_T rankR;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T isFeasible;
  nVar = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 69;
  if (mLambda >= 0) {
    std::memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->
                lambdasqp.data[0], static_cast<uint32_T>(mLambda + 1) * sizeof
                (real_T));
  }

  AEBController_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad.data[0]);
      for (ix = 2; ix <= nVar; ix++) {
        s = std::abs(TrialState->grad.data[ix - 1]);
        if (s > smax) {
          idx_max = ix;
          smax = s;
        }
      }
    }
  }

  smax = std::abs(TrialState->grad.data[idx_max - 1]);
  if ((smax <= 1.0) || rtIsNaN(smax)) {
    smax = 1.0;
  }

  if (rtIsInf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = AEBController_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  if (TrialState->sqpIterations == 0) {
    if ((MeritFunction->nlpPrimalFeasError <= 1.0) || rtIsNaN
        (MeritFunction->nlpPrimalFeasError)) {
      MeritFunction->feasRelativeFactor = 1.0;
    } else {
      MeritFunction->feasRelativeFactor = MeritFunction->nlpPrimalFeasError;
    }
  }

  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  AEBController_computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    &Flags->gradOK, &MeritFunction->nlpDualFeasError);
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = AEBController_computeComplError
      (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
       TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
       WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
       WorkingSet->indexUB.data, WorkingSet->sizes[4],
       TrialState->lambdaStopTest.data, WorkingSet->sizes[0] + 71);
    if ((MeritFunction->nlpDualFeasError >= MeritFunction->nlpComplError) ||
        rtIsNaN(MeritFunction->nlpComplError)) {
      MeritFunction->firstOrderOpt = MeritFunction->nlpDualFeasError;
    } else {
      MeritFunction->firstOrderOpt = MeritFunction->nlpComplError;
    }

    if (TrialState->sqpIterations > 1) {
      AEBController_computeGradLag_b(memspace->workspace_float.data,
        WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
        WorkingSet->sizes[2], WorkingSet->Aineq.data, WorkingSet->Aeq.data,
        WorkingSet->indexFixed.data, WorkingSet->sizes[0],
        WorkingSet->indexLB.data, WorkingSet->sizes[3], WorkingSet->indexUB.data,
        WorkingSet->sizes[4], TrialState->lambdaStopTestPrev.data);
      AEBController_computeDualFeasError_c(WorkingSet->nVar,
        memspace->workspace_float.data, &dxTooSmall, &s);
      nlpComplErrorTmp = AEBController_computeComplError
        (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
         TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
         WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
         WorkingSet->indexUB.data, WorkingSet->sizes[4],
         TrialState->lambdaStopTestPrev.data, WorkingSet->sizes[0] + 71);
      if ((s < MeritFunction->nlpDualFeasError) && (nlpComplErrorTmp <
           MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        if (s >= nlpComplErrorTmp) {
          MeritFunction->firstOrderOpt = s;
        } else {
          MeritFunction->firstOrderOpt = nlpComplErrorTmp;
        }

        if (mLambda >= 0) {
          std::memcpy(&TrialState->lambdaStopTest.data[0],
                      &TrialState->lambdaStopTestPrev.data[0],
                      static_cast<uint32_T>(mLambda + 1) * sizeof(real_T));
        }
      } else if (mLambda >= 0) {
        std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                    &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                    (mLambda + 1) * sizeof(real_T));
      }
    } else if (mLambda >= 0) {
      std::memcpy(&TrialState->lambdaStopTestPrev.data[0],
                  &TrialState->lambdaStopTest.data[0], static_cast<uint32_T>
                  (mLambda + 1) * sizeof(real_T));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 * smax) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * smax)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          ix = 0;
          exitg1 = false;
          while ((!exitg1) && (ix <= static_cast<uint16_T>(WorkingSet->nVar) - 1))
          {
            s = std::abs(TrialState->xstarsqp[ix]);
            if ((s <= 1.0) || rtIsNaN(s)) {
              s = 1.0;
            }

            if (1.0E-6 * s <= std::abs(TrialState->delta_x.data[ix])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              ix++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType == 2) {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              } else {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              }
            } else {
              idx_max = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr == 0) {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              } else {
                AEBController_updateWorkingSetForNewQP(TrialState->xstarsqp,
                  WorkingSet, WorkingSet->sizes[2], TrialState->mNonlinIneq,
                  TrialState->cIneq.data, TrialState->cEq, WorkingSet->sizes[3],
                  lb, WorkingSet->sizes[4], WorkingSet->sizes[0]);
                for (ix = 0; ix < idx_max; ix++) {
                  TrialState->lambda.data[ix] = 0.0;
                }

                AEBController_factorQRE(QRManager, WorkingSet->ATwset.data,
                  WorkingSet->nVar, WorkingSet->nActiveConstr, WorkingSet->ldA);
                rankR = QRManager->minRowCol;
                for (idx_max = 0; idx_max < rankR; idx_max++) {
                  iQR0 = QRManager->ldq * idx_max + idx_max;
                  n = QRManager->mrows - idx_max;
                  if (n - 2 >= 0) {
                    std::memcpy(&QRManager->Q.data[iQR0 + 1],
                                &QRManager->QR.data[iQR0 + 1],
                                static_cast<uint32_T>(n - 1) * sizeof(real_T));
                  }
                }

                AEBController_xorgqr(QRManager->mrows, QRManager->mrows,
                                     QRManager->minRowCol, QRManager->Q.data,
                                     QRManager->Q.size, QRManager->ldq,
                                     QRManager->tau.data);
                rankR = QRManager->ldq;
                idx_max = static_cast<uint16_T>(WorkingSet->nVar);
                for (ix = 0; ix < idx_max; ix++) {
                  memspace->workspace_float.data[ix] = 0.0;
                }

                iQR0 = 0;
                n = (WorkingSet->nVar - 1) * QRManager->ldq + 1;
                for (idx_max = 1; rankR < 0 ? idx_max >= n : idx_max <= n;
                     idx_max += rankR) {
                  s = 0.0;
                  c_ix = (idx_max + nVar) - 1;
                  for (ix = idx_max; ix <= c_ix; ix++) {
                    s += QRManager->Q.data[ix - 1] * TrialState->grad.data[ix -
                      idx_max];
                  }

                  memspace->workspace_float.data[iQR0] -= s;
                  iQR0++;
                }

                if (WorkingSet->nVar >= WorkingSet->nActiveConstr) {
                  ix = WorkingSet->nVar;
                } else {
                  ix = WorkingSet->nActiveConstr;
                }

                s = static_cast<real_T>(ix) * 2.2204460492503131E-16;
                if (s >= 1.4901161193847656E-8) {
                  s = 1.4901161193847656E-8;
                }

                s *= std::abs(QRManager->QR.data[0]);
                if (WorkingSet->nVar <= WorkingSet->nActiveConstr) {
                  ix = WorkingSet->nVar;
                } else {
                  ix = WorkingSet->nActiveConstr;
                }

                rankR = 0;
                nVar = 0;
                while ((rankR < ix) && (std::abs(QRManager->QR.data[nVar]) > s))
                {
                  rankR++;
                  nVar = (nVar + QRManager->ldq) + 1;
                }

                if (rankR != 0) {
                  for (nVar = rankR; nVar >= 1; nVar--) {
                    iQR0 = ((nVar - 1) * QRManager->ldq + nVar) - 2;
                    memspace->workspace_float.data[nVar - 1] /=
                      QRManager->QR.data[iQR0 + 1];
                    for (idx_max = 0; idx_max <= nVar - 2; idx_max++) {
                      c_ix = (nVar - idx_max) - 2;
                      memspace->workspace_float.data[c_ix] -=
                        memspace->workspace_float.data[nVar - 1] *
                        QRManager->QR.data[iQR0 - idx_max];
                    }
                  }
                }

                if (WorkingSet->nActiveConstr <= ix) {
                  ix = WorkingSet->nActiveConstr;
                }

                for (nVar = 0; nVar < ix; nVar++) {
                  TrialState->lambda.data[QRManager->jpvt.data[nVar] - 1] =
                    memspace->workspace_float.data[nVar];
                }

                AEBController_sortLambdaQP(TrialState->lambda.data,
                  WorkingSet->nActiveConstr, WorkingSet->sizes,
                  WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                  WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
                AEBController_computeGradLag_b(memspace->workspace_float.data,
                  WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
                  WorkingSet->sizes[2], WorkingSet->Aineq.data,
                  WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
                  WorkingSet->sizes[0], WorkingSet->indexLB.data,
                  WorkingSet->sizes[3], WorkingSet->indexUB.data,
                  WorkingSet->sizes[4], TrialState->lambda.data);
                AEBController_computeDualFeasError_c(WorkingSet->nVar,
                  memspace->workspace_float.data, &isFeasible, &s);
                nlpComplErrorTmp = AEBController_computeComplError
                  (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
                   TrialState->xstarsqp, WorkingSet->sizes[2],
                   TrialState->cIneq.data, WorkingSet->indexLB.data,
                   WorkingSet->sizes[3], lb, WorkingSet->indexUB.data,
                   WorkingSet->sizes[4], TrialState->lambda.data,
                   WorkingSet->sizes[0] + 71);
                if ((s >= nlpComplErrorTmp) || rtIsNaN(nlpComplErrorTmp)) {
                  tmp = s;
                } else {
                  tmp = nlpComplErrorTmp;
                }

                if ((MeritFunction->nlpDualFeasError >=
                     MeritFunction->nlpComplError) || rtIsNaN
                    (MeritFunction->nlpComplError)) {
                  tmp_0 = MeritFunction->nlpDualFeasError;
                } else {
                  tmp_0 = MeritFunction->nlpComplError;
                }

                if (tmp <= tmp_0) {
                  MeritFunction->nlpDualFeasError = s;
                  MeritFunction->nlpComplError = nlpComplErrorTmp;
                  MeritFunction->firstOrderOpt = tmp;
                  if (mLambda >= 0) {
                    std::memcpy(&TrialState->lambdaStopTest.data[0],
                                &TrialState->lambda.data[0],
                                static_cast<uint32_T>(mLambda + 1) * sizeof
                                (real_T));
                  }
                }

                if ((MeritFunction->nlpDualFeasError <= 1.0E-6 * smax) &&
                    (MeritFunction->nlpComplError <= 1.0E-6 * smax)) {
                  TrialState->sqpExitFlag = 1;
                } else {
                  TrialState->sqpExitFlag = 2;
                }

                Flags->done = true;
                guard1 = true;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 7500) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
boolean_T ACCWithSensorFusionModelClass::AEBController_BFGSUpdate(int32_T nvar,
  real_T Bk[5625], const real_T sk_data[], real_T yk_data[], real_T
  workspace_data[])
{
  real_T curvatureS;
  real_T dotSY;
  int32_T b_ix;
  int32_T b_jA;
  int32_T b_tmp;
  int32_T ix;
  int32_T jy;
  boolean_T success;
  dotSY = 0.0;
  if (nvar >= 1) {
    for (ix = 0; ix < nvar; ix++) {
      dotSY += sk_data[ix] * yk_data[ix];
    }
  }

  b_tmp = static_cast<uint16_T>(nvar);
  for (ix = 0; ix < b_tmp; ix++) {
    workspace_data[ix] = 0.0;
  }

  ix = 0;
  jy = (nvar - 1) * 75 + 1;
  for (int32_T b = 1; b <= jy; b += 75) {
    b_ix = (b + nvar) - 1;
    for (b_jA = b; b_jA <= b_ix; b_jA++) {
      int32_T tmp;
      tmp = b_jA - b;
      workspace_data[tmp] += Bk[b_jA - 1] * sk_data[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (int32_T b = 0; b < nvar; b++) {
      curvatureS += workspace_data[b] * sk_data[b];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    dotSY = 0.8 * curvatureS / (curvatureS - dotSY);
    for (ix = 0; ix < b_tmp; ix++) {
      yk_data[ix] *= dotSY;
    }

    if (!(1.0 - dotSY == 0.0)) {
      for (int32_T b = 0; b < nvar; b++) {
        yk_data[b] += (1.0 - dotSY) * workspace_data[b];
      }
    }

    dotSY = 0.0;
    if (nvar >= 1) {
      for (ix = 0; ix < nvar; ix++) {
        dotSY += sk_data[ix] * yk_data[ix];
      }
    }
  }

  success = ((curvatureS > 2.2204460492503131E-16) && (dotSY >
              2.2204460492503131E-16));
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 1;
      for (int32_T b = 0; b < b_tmp; b++) {
        if (workspace_data[b] != 0.0) {
          real_T temp;
          temp = workspace_data[b] * curvatureS;
          b_ix = (nvar + ix) - 1;
          for (b_jA = ix; b_jA <= b_ix; b_jA++) {
            Bk[b_jA - 1] += workspace_data[b_jA - ix] * temp;
          }
        }

        ix += 75;
      }
    }

    dotSY = 1.0 / dotSY;
    if (!(dotSY == 0.0)) {
      b_jA = 1;
      for (ix = 0; ix < b_tmp; ix++) {
        curvatureS = yk_data[ix];
        if (curvatureS != 0.0) {
          curvatureS *= dotSY;
          b_ix = (nvar + b_jA) - 1;
          for (int32_T b = b_jA; b <= b_ix; b++) {
            Bk[b - 1] += yk_data[b - b_jA] * curvatureS;
          }
        }

        b_jA += 75;
      }
    }
  }

  return success;
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_driver(const real_T
  bineq_data[], const real_T lb[75], s_aN87P1cKlEBgtxywOa8jBC_AEBController_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_AEBController_T *MeritFunction, const
  coder_internal_stickyStruct_20_AEBController_T *FcnEvaluator,
  s_5XVl0Gn0wBD7AEyOr2FgBB_AEBController_T *memspace,
  s_H2r48KAwe9QhSMXfaacI5D_AEBController_T *WorkingSet,
  s_hdPc7ahpeRZmVH2CVtYjr_AEBController_T *QRManager,
  s_NZ75DufWqh8NxBuGZX5zO_AEBController_T *CholManager,
  s_R5gKZDFXhdXkpc4uAJSolH_AEBController_T *QPObjective, const int32_T
  fscales_lineq_constraint_size[1], const int32_T fscales_cineq_constraint_size
  [1], real_T Hessian[5625])
{
  s7RdrPWkr8UPAUyTdDJkLaG_AEBController_T Flags;
  somzaGboVhDG7PNQS6E98jD_AEBController_T expl_temp;
  real_T TrialState_lambdasqp;
  int32_T b_ix;
  int32_T b_iy;
  int32_T d_ix;
  int32_T ix;
  int32_T iy;
  int32_T k;
  int32_T loop_ub;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLinIneq;
  int32_T mUB;
  int32_T n;
  int32_T qpoptions_MaxIterations;
  int32_T u1;
  static const int8_T s[5625] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const char_T r[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  somzaGboVhDG7PNQS6E98jD_AEBController_T expl_temp_0;
  int32_T iy_tmp;
  int32_T nVar_tmp_tmp;
  int32_T n_tmp;
  for (u1 = 0; u1 < 5625; u1++) {
    Hessian[u1] = s[u1];
  }

  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 70;
  mLinIneq = WorkingSet->sizes[2] - TrialState->mNonlinIneq;
  u1 = ((WorkingSet->sizes[2] + WorkingSet->sizes[3]) + WorkingSet->sizes[4]) +
    (WorkingSet->sizes[0] << 1);
  if (WorkingSet->nVar >= u1) {
    u1 = WorkingSet->nVar;
  }

  qpoptions_MaxIterations = 10 * u1;
  TrialState->steplength = 1.0;
  AEBController_test_exit(MeritFunction, WorkingSet, TrialState, lb,
    &Flags.gradOK, &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
    &Flags.failedLineSearch, &Flags.stepType);
  AEBController_saveJacobian(TrialState, WorkingSet->nVar, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, TrialState->iNonIneq0, WorkingSet->Aeq.data,
    WorkingSet->ldA);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 75; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old.data[k] = TrialState->grad.data[k];
  }

  n_tmp = TrialState->mIneq;
  d_ix = TrialState->cIneq_old.size[0];
  loop_ub = TrialState->cIneq_old.size[0];
  if (loop_ub - 1 >= 0) {
    std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq_old.data[0],
                static_cast<uint32_T>(loop_ub) * sizeof(real_T));
  }

  if (TrialState->mIneq - 1 >= 0) {
    std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq.data[0],
                static_cast<uint32_T>(TrialState->mIneq) * sizeof(real_T));
  }

  if (d_ix - 1 >= 0) {
    std::memcpy(&TrialState->cIneq_old.data[0], &AEBController_DW.y_data_m[0],
                static_cast<uint32_T>(d_ix) * sizeof(real_T));
  }

  std::memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 70U * sizeof(real_T));
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    if ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = (rtMinusInf);
      expl_temp.ConstraintTolerance = 1.0E-6;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (u1 = 0; u1 < 7; u1++) {
        expl_temp.SolverName[u1] = r[u1];
      }
    }

    while ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        AEBController_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet,
          mIneq, TrialState->mNonlinIneq, TrialState->cIneq.data,
          TrialState->cEq, mLB, lb, mUB, mFixed);
      }

      expl_temp_0 = expl_temp;
      AEBController_step(&Flags, Hessian, lb, TrialState, MeritFunction,
                         memspace, WorkingSet, QRManager, CholManager,
                         QPObjective, &expl_temp_0);
      if (Flags.stepAccepted) {
        n = static_cast<uint16_T>(nVar_tmp_tmp);
        for (k = 0; k < n; k++) {
          TrialState->xstarsqp[k] += TrialState->delta_x.data[k];
        }

        AEBController_evalObjAndConstr
          (FcnEvaluator->next.next.next.next.next.b_value,
           &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
           &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
           TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
           TrialState->cEq, &TrialState->sqpFval, &k);
        Flags.fevalOK = (k == 1);
        TrialState->FunctionEvaluations++;
        AEBController_computeLinearResiduals(TrialState->xstarsqp, nVar_tmp_tmp,
          TrialState->cIneq.data, TrialState->cIneq.size, mLinIneq,
          WorkingSet->Aineq.data, bineq_data, WorkingSet->ldA);
        MeritFunction->phiFullStep = AEBController_computeMeritFcn
          (MeritFunction->penaltyParam, TrialState->sqpFval,
           TrialState->cIneq.data, mIneq, TrialState->cEq, Flags.fevalOK);
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        AEBController_linesearch(&Flags.fevalOK, bineq_data, WorkingSet->nVar,
          WorkingSet->ldA, WorkingSet->Aineq.data, TrialState,
          MeritFunction->penaltyParam, MeritFunction->phi,
          MeritFunction->phiPrimePlus, MeritFunction->phiFullStep,
          FcnEvaluator->next.next.next.next.next.b_value,
          &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
          &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
          ((Flags.stepType == 3) && Flags.stepAccepted), &TrialState_lambdasqp,
          &k);
        TrialState->steplength = TrialState_lambdasqp;
        if (k > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      n = static_cast<uint16_T>(nVar_tmp_tmp);
      for (k = 0; k < n; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k] +
          TrialState->delta_x.data[k];
      }

      n = static_cast<uint16_T>(mConstr);
      for (k = 0; k < n; k++) {
        TrialState_lambdasqp = TrialState->lambdasqp.data[k];
        TrialState->lambdasqp.data[k] = (TrialState->lambda.data[k] -
          TrialState_lambdasqp) * TrialState->steplength + TrialState_lambdasqp;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 75; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old.data[k] = TrialState->grad.data[k];
      }

      d_ix = TrialState->cIneq_old.size[0];
      loop_ub = TrialState->cIneq_old.size[0];
      if (loop_ub - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq_old.data[0],
                    static_cast<uint32_T>(loop_ub) * sizeof(real_T));
      }

      if (n_tmp - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq.data[0],
                    static_cast<uint32_T>(n_tmp) * sizeof(real_T));
      }

      if (d_ix - 1 >= 0) {
        std::memcpy(&TrialState->cIneq_old.data[0], &AEBController_DW.y_data_m[0],
                    static_cast<uint32_T>(d_ix) * sizeof(real_T));
      }

      std::memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 70U * sizeof
                  (real_T));
      Flags.gradOK = true;
      AEBController_evalObjAndConstrAndDerivatives
        (FcnEvaluator->next.next.next.next.next.b_value,
         &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
         &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
         TrialState->xstarsqp, TrialState->grad.data, TrialState->cIneq.data,
         TrialState->iNonIneq0, TrialState->cEq, WorkingSet->Aineq.data,
         TrialState->iNonIneq0, WorkingSet->ldA, WorkingSet->Aeq.data,
         WorkingSet->ldA, &TrialState->sqpFval, &k);
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (k == 1);
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      std::memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 75U *
                  sizeof(real_T));
      d_ix = TrialState->cIneq.size[0];
      loop_ub = TrialState->cIneq.size[0];
      if (loop_ub - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq.data[0],
                    static_cast<uint32_T>(loop_ub) * sizeof(real_T));
      }

      if (n_tmp - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_m[0], &TrialState->cIneq_old.data[0],
                    static_cast<uint32_T>(n_tmp) * sizeof(real_T));
      }

      if (d_ix - 1 >= 0) {
        std::memcpy(&TrialState->cIneq.data[0], &AEBController_DW.y_data_m[0],
                    static_cast<uint32_T>(d_ix) * sizeof(real_T));
      }

      std::memcpy(&TrialState->cEq[0], &TrialState->cEq_old[0], 70U * sizeof
                  (real_T));
    }

    AEBController_test_exit_n(&Flags, memspace, MeritFunction,
      fscales_lineq_constraint_size, fscales_cineq_constraint_size, WorkingSet,
      TrialState, QRManager, lb);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      n = (mFixed + TrialState->iNonIneq0) + 69;
      k = WorkingSet->ldA;
      std::memcpy(&TrialState->delta_gradLag.data[0], &TrialState->grad.data[0],
                  static_cast<uint16_T>(nVar_tmp_tmp) * sizeof(real_T));
      d_ix = TrialState->delta_gradLag.size[0];
      loop_ub = TrialState->delta_gradLag.size[0];
      if (loop_ub - 1 >= 0) {
        std::memcpy(&AEBController_DW.y_data_m[0],
                    &TrialState->delta_gradLag.data[0], static_cast<uint32_T>
                    (loop_ub) * sizeof(real_T));
      }

      if (nVar_tmp_tmp >= 1) {
        for (loop_ub = 0; loop_ub < nVar_tmp_tmp; loop_ub++) {
          AEBController_DW.y_data_m[loop_ub] -= TrialState->
            grad_old.data[loop_ub];
        }
      }

      ix = mFixed;
      if (d_ix - 1 >= 0) {
        std::memcpy(&TrialState->delta_gradLag.data[0],
                    &AEBController_DW.y_data_m[0], static_cast<uint32_T>(d_ix) *
                    sizeof(real_T));
      }

      iy = WorkingSet->ldA * 69 + 1;
      for (loop_ub = 1; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k) {
        b_iy = (loop_ub + nVar_tmp_tmp) - 1;
        for (d_ix = loop_ub; d_ix <= b_iy; d_ix++) {
          u1 = d_ix - loop_ub;
          TrialState->delta_gradLag.data[u1] += WorkingSet->Aeq.data[d_ix - 1] *
            TrialState->lambdasqp.data[ix];
        }

        ix++;
      }

      b_ix = mFixed;
      for (loop_ub = 1; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k) {
        b_iy = (loop_ub + nVar_tmp_tmp) - 1;
        for (d_ix = loop_ub; d_ix <= b_iy; d_ix++) {
          u1 = d_ix - loop_ub;
          TrialState->delta_gradLag.data[u1] += TrialState->
            JacCeqTrans_old.data[d_ix - 1] * -TrialState->lambdasqp.data[b_ix];
        }

        b_ix++;
      }

      if (TrialState->mNonlinIneq > 0) {
        ix = (TrialState->iNonIneq0 - 1) * WorkingSet->ldA + 1;
        b_ix = n;
        iy_tmp = (TrialState->mNonlinIneq - 1) * WorkingSet->ldA;
        iy = iy_tmp + ix;
        for (loop_ub = ix; k < 0 ? loop_ub >= iy : loop_ub <= iy; loop_ub += k)
        {
          b_iy = (loop_ub + nVar_tmp_tmp) - 1;
          for (d_ix = loop_ub; d_ix <= b_iy; d_ix++) {
            u1 = d_ix - loop_ub;
            TrialState->delta_gradLag.data[u1] += WorkingSet->Aineq.data[d_ix -
              1] * TrialState->lambdasqp.data[b_ix];
          }

          b_ix++;
        }

        d_ix = n;
        b_ix = iy_tmp + 1;
        for (n = 1; k < 0 ? n >= b_ix : n <= b_ix; n += k) {
          b_iy = (n + nVar_tmp_tmp) - 1;
          for (loop_ub = n; loop_ub <= b_iy; loop_ub++) {
            u1 = loop_ub - n;
            TrialState->delta_gradLag.data[u1] +=
              TrialState->JacCineqTrans_old.data[loop_ub - 1] *
              -TrialState->lambdasqp.data[d_ix];
          }

          d_ix++;
        }
      }

      AEBController_saveJacobian(TrialState, nVar_tmp_tmp, mIneq,
        WorkingSet->Aineq.data, TrialState->iNonIneq0, WorkingSet->Aeq.data,
        WorkingSet->ldA);
      AEBController_BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x.data,
        TrialState->delta_gradLag.data, memspace->workspace_float.data);
      TrialState->sqpIterations++;
    }
  }
}

// Function for MATLAB Function: '<S15>/NLMPC'
void ACCWithSensorFusionModelClass::AEBController_fmincon(const
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T *fun_workspace_runtimedata, const
  sjqToqDl6Xs23mBXoA194vG_AEBController_T *fun_workspace_userdata, const real_T
  x0[75], const real_T Aineq_data[], const real_T bineq_data[], const int32_T
  bineq_size[1], const real_T lb[75], const
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T *nonlcon_workspace_runtimedata,
  real_T x[75], real_T *fval, real_T *exitflag, real_T *output_iterations,
  real_T *output_funcCount, char_T output_algorithm[3], real_T
  *output_constrviolation, real_T *output_stepsize, real_T *output_lssteplength,
  real_T *output_firstorderopt)
{
  sG8JZ69axY52WWR6RKyApQC_AEBController_T MeritFunction;
  real_T Ceq[70];
  real_T Cineq_data[60];
  real_T absxk;
  real_T b_c;
  real_T scale;
  real_T t;
  int32_T Cineq_size[2];
  int32_T JacCineqTrans_size[2];
  int32_T tmp_size[1];
  int32_T tmp_size_0[1];
  int32_T WorkingSet_tmp_tmp;
  int32_T iEq0;
  int32_T idx_row;
  int32_T k;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T mLinIneq;
  int32_T mLinIneq_tmp;
  int32_T mNonlinIneq;
  uint8_T WorkingSet_tmp[5];
  uint8_T WorkingSet_tmp_0;
  AEBController_c4_mpclib_anonFcn2(nonlcon_workspace_runtimedata->x,
    nonlcon_workspace_runtimedata->md, nonlcon_workspace_runtimedata->OutputMin,
    nonlcon_workspace_runtimedata->OutputMax,
    nonlcon_workspace_runtimedata->Parameters, x0, Cineq_data, Cineq_size, Ceq,
    AEBController_DW.JacCineqTrans_data, JacCineqTrans_size,
    AEBController_DW.JacCeqTrans);
  mNonlinIneq = Cineq_size[0] * Cineq_size[1];
  mLinIneq_tmp = bineq_size[0];
  mIneq = bineq_size[0] + mNonlinIneq;
  mConstrMax = (mIneq + mIneq) + 361;
  AEBController_factoryConstruct(mIneq + 216, mConstrMax, mIneq, mNonlinIneq,
    &AEBController_DW.TrialState);
  std::memcpy(&AEBController_DW.TrialState.xstarsqp[0], &x0[0], 75U * sizeof
              (real_T));
  AEBController_DW.FcnEvaluator.next.next.next.next.next.b_value = mNonlinIneq;
  AEBController_DW.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata
    = *nonlcon_workspace_runtimedata;
  AEBController_DW.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata
    = *fun_workspace_runtimedata;
  AEBController_DW.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.userdata
    = *fun_workspace_userdata;
  AEBController_factoryConstruct_oogs(mIneq + 216,
    AEBController_DW.QPObjective.grad.size, AEBController_DW.QPObjective.Hx.size,
    &AEBController_DW.QPObjective.hasLinear, &AEBController_DW.QPObjective.nvar,
    &AEBController_DW.QPObjective.maxVar, &AEBController_DW.QPObjective.beta,
    &AEBController_DW.QPObjective.rho, &AEBController_DW.QPObjective.objtype,
    &AEBController_DW.QPObjective.prev_objtype,
    &AEBController_DW.QPObjective.prev_nvar,
    &AEBController_DW.QPObjective.prev_hasLinear,
    &AEBController_DW.QPObjective.gammaScalar);
  AEBController_DW.QPObjective.nvar = 75;
  AEBController_DW.QPObjective.hasLinear = true;
  AEBController_DW.QPObjective.objtype = 3;
  AEBController_DW.memspace.workspace_float.size[0] = mConstrMax;
  AEBController_DW.memspace.workspace_float.size[1] = mIneq + 216;
  AEBController_DW.memspace.workspace_int.size[0] = mConstrMax;
  AEBController_DW.memspace.workspace_sort.size[0] = mConstrMax;
  AEBController_factoryConstruct_oogsz(mIneq, mIneq + 216, mConstrMax,
    &AEBController_DW.WorkingSet);
  k = -1;
  for (iEq0 = 0; iEq0 < 75; iEq0++) {
    b_c = lb[iEq0];
    if ((!rtIsInf(b_c)) && (!rtIsNaN(b_c))) {
      k++;
      AEBController_DW.WorkingSet.indexLB.data[k] = iEq0 + 1;
    }
  }

  AEBController_DW.WorkingSet.mConstrMax = mConstrMax;
  WorkingSet_tmp_tmp = mIneq + k;
  AEBController_DW.WorkingSet.mConstr = WorkingSet_tmp_tmp + 71;
  AEBController_DW.WorkingSet.mConstrOrig = WorkingSet_tmp_tmp + 71;
  WorkingSet_tmp[0] = 0U;
  WorkingSet_tmp[1] = 70U;
  WorkingSet_tmp[2] = static_cast<uint8_T>(mIneq);
  WorkingSet_tmp[3] = static_cast<uint8_T>(k + 1);
  WorkingSet_tmp[4] = 0U;
  for (iEq0 = 0; iEq0 < 5; iEq0++) {
    WorkingSet_tmp_0 = WorkingSet_tmp[iEq0];
    AEBController_DW.WorkingSet.sizes[iEq0] = WorkingSet_tmp_0;
    AEBController_DW.WorkingSet.sizesNormal[iEq0] = WorkingSet_tmp_0;
  }

  AEBController_DW.WorkingSet.sizesPhaseOne[0] = 0;
  AEBController_DW.WorkingSet.sizesPhaseOne[1] = 70;
  AEBController_DW.WorkingSet.sizesPhaseOne[2] = mIneq;
  AEBController_DW.WorkingSet.sizesPhaseOne[3] = k + 2;
  AEBController_DW.WorkingSet.sizesPhaseOne[4] = 0;
  AEBController_DW.WorkingSet.sizesRegularized[0] = 0;
  AEBController_DW.WorkingSet.sizesRegularized[1] = 70;
  AEBController_DW.WorkingSet.sizesRegularized[2] = mIneq;
  AEBController_DW.WorkingSet.sizesRegularized[3] = WorkingSet_tmp_tmp + 141;
  AEBController_DW.WorkingSet.sizesRegularized[4] = 0;
  AEBController_DW.WorkingSet.sizesRegPhaseOne[0] = 0;
  AEBController_DW.WorkingSet.sizesRegPhaseOne[1] = 70;
  AEBController_DW.WorkingSet.sizesRegPhaseOne[2] = mIneq;
  AEBController_DW.WorkingSet.sizesRegPhaseOne[3] = WorkingSet_tmp_tmp + 142;
  AEBController_DW.WorkingSet.sizesRegPhaseOne[4] = 0;
  AEBController_DW.WorkingSet.isActiveIdxNormal[0] = 1;
  AEBController_DW.WorkingSet.isActiveIdxNormal[1] = 0;
  AEBController_DW.WorkingSet.isActiveIdxNormal[2] = 70;
  AEBController_DW.WorkingSet.isActiveIdxNormal[3] = mIneq;
  AEBController_DW.WorkingSet.isActiveIdxNormal[4] = k + 1;
  AEBController_DW.WorkingSet.isActiveIdxNormal[5] = 0;
  for (idx_row = 0; idx_row < 6; idx_row++) {
    AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[idx_row] =
      AEBController_DW.WorkingSet.isActiveIdxNormal[idx_row];
  }

  for (iEq0 = 0; iEq0 < 5; iEq0++) {
    AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0 + 1] +=
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  for (idx_row = 0; idx_row < 6; idx_row++) {
    AEBController_DW.WorkingSet.isActiveIdx[idx_row] =
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[idx_row];
  }

  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[2] = 70;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[4] = k + 2;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (iEq0 = 0; iEq0 < 5; iEq0++) {
    AEBController_DW.WorkingSet.isActiveIdxNormal[iEq0 + 1] +=
      AEBController_DW.WorkingSet.isActiveIdxNormal[iEq0];
    AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0 + 1] +=
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  for (idx_row = 0; idx_row < 6; idx_row++) {
    AEBController_DW.WorkingSet.isActiveIdxPhaseOne[idx_row] =
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[idx_row];
  }

  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[2] = 70;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[4] = WorkingSet_tmp_tmp +
    141;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (iEq0 = 0; iEq0 < 5; iEq0++) {
    AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0 + 1] +=
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  for (idx_row = 0; idx_row < 6; idx_row++) {
    AEBController_DW.WorkingSet.isActiveIdxRegularized[idx_row] =
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[idx_row];
  }

  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[2] = 70;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[4] = WorkingSet_tmp_tmp +
    142;
  AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (iEq0 = 0; iEq0 < 5; iEq0++) {
    AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0 + 1] +=
      AEBController_DW.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  if (mIneq > 0) {
    for (WorkingSet_tmp_tmp = 0; WorkingSet_tmp_tmp < mLinIneq_tmp;
         WorkingSet_tmp_tmp++) {
      for (idx_row = 0; idx_row < 75; idx_row++) {
        AEBController_DW.WorkingSet.Aineq.data[idx_row +
          AEBController_DW.WorkingSet.ldA * WorkingSet_tmp_tmp] =
          Aineq_data[mLinIneq_tmp * idx_row + WorkingSet_tmp_tmp];
      }
    }
  }

  for (iEq0 = 0; iEq0 <= k; iEq0++) {
    WorkingSet_tmp_tmp = AEBController_DW.WorkingSet.indexLB.data[iEq0];
    b_c = lb[WorkingSet_tmp_tmp - 1];
    if ((AEBController_DW.TrialState.xstarsqp[WorkingSet_tmp_tmp - 1] >= b_c) ||
        rtIsNaN(b_c)) {
    } else {
      AEBController_DW.TrialState.xstarsqp[WorkingSet_tmp_tmp - 1] = b_c;
    }
  }

  AEBController_evalObjAndConstrAndDerivatives(mNonlinIneq,
    nonlcon_workspace_runtimedata,
    &AEBController_DW.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace,
    AEBController_DW.TrialState.xstarsqp, AEBController_DW.TrialState.grad.data,
    AEBController_DW.TrialState.cIneq.data,
    AEBController_DW.TrialState.iNonIneq0, AEBController_DW.TrialState.cEq,
    AEBController_DW.WorkingSet.Aineq.data,
    AEBController_DW.TrialState.iNonIneq0, AEBController_DW.WorkingSet.ldA,
    AEBController_DW.WorkingSet.Aeq.data, AEBController_DW.WorkingSet.ldA,
    &AEBController_DW.TrialState.sqpFval, &iEq0);
  AEBController_DW.TrialState.FunctionEvaluations = 1;
  iEq0 = AEBController_DW.WorkingSet.ldA;
  if (bineq_size[0] > 0) {
    mLinIneq = AEBController_DW.TrialState.cIneq.size[0];
    if (mLinIneq - 1 >= 0) {
      std::memcpy(&AEBController_DW.y_data[0],
                  &AEBController_DW.TrialState.cIneq.data[0],
                  static_cast<uint32_T>(mLinIneq) * sizeof(real_T));
    }

    if (mLinIneq_tmp - 1 >= 0) {
      std::memcpy(&AEBController_DW.y_data[0], &bineq_data[0],
                  static_cast<uint32_T>(mLinIneq_tmp) * sizeof(real_T));
    }

    if (mLinIneq - 1 >= 0) {
      std::memcpy(&AEBController_DW.TrialState.cIneq.data[0],
                  &AEBController_DW.y_data[0], static_cast<uint32_T>(mLinIneq) *
                  sizeof(real_T));
    }

    for (WorkingSet_tmp_tmp = 0; WorkingSet_tmp_tmp < mLinIneq_tmp;
         WorkingSet_tmp_tmp++) {
      AEBController_DW.TrialState.cIneq.data[WorkingSet_tmp_tmp] =
        -AEBController_DW.TrialState.cIneq.data[WorkingSet_tmp_tmp];
    }

    mLinIneq = 0;
    WorkingSet_tmp_tmp = (bineq_size[0] - 1) * AEBController_DW.WorkingSet.ldA +
      1;
    for (idx_row = 1; iEq0 < 0 ? idx_row >= WorkingSet_tmp_tmp : idx_row <=
         WorkingSet_tmp_tmp; idx_row += iEq0) {
      b_c = 0.0;
      for (mLinIneq_tmp = idx_row; mLinIneq_tmp <= idx_row + 74; mLinIneq_tmp++)
      {
        b_c += AEBController_DW.WorkingSet.Aineq.data[mLinIneq_tmp - 1] *
          AEBController_DW.TrialState.xstarsqp[mLinIneq_tmp - idx_row];
      }

      AEBController_DW.TrialState.cIneq.data[mLinIneq] += b_c;
      mLinIneq++;
    }
  }

  for (mLinIneq = 0; mLinIneq < 70; mLinIneq++) {
    b_c = -AEBController_DW.TrialState.cEq[mLinIneq];
    AEBController_DW.WorkingSet.beq[mLinIneq] = b_c;
    AEBController_DW.WorkingSet.bwset.data[mLinIneq] = b_c;
  }

  mLinIneq = 0;
  iEq0 = 0;
  for (WorkingSet_tmp_tmp = 0; WorkingSet_tmp_tmp < 70; WorkingSet_tmp_tmp++) {
    std::memcpy(&AEBController_DW.WorkingSet.ATwset.data[mLinIneq],
                &AEBController_DW.WorkingSet.Aeq.data[iEq0], 75U * sizeof(real_T));
    mLinIneq += AEBController_DW.WorkingSet.ldA;
    iEq0 = mLinIneq;
  }

  for (mLinIneq = 0; mLinIneq < mIneq; mLinIneq++) {
    AEBController_DW.WorkingSet.bineq.data[mLinIneq] =
      -AEBController_DW.TrialState.cIneq.data[mLinIneq];
  }

  for (mLinIneq = 0; mLinIneq <= k; mLinIneq++) {
    AEBController_DW.WorkingSet.lb.data[AEBController_DW.WorkingSet.indexLB.data[
      mLinIneq] - 1] = -lb[AEBController_DW.WorkingSet.indexLB.data[mLinIneq] -
      1] + x0[AEBController_DW.WorkingSet.indexLB.data[mLinIneq] - 1];
  }

  AEBController_initActiveSet(&AEBController_DW.WorkingSet);
  MeritFunction.initFval = AEBController_DW.TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  b_c = 0.0;
  for (k = 0; k < 70; k++) {
    b_c += std::abs(AEBController_DW.TrialState.cEq[k]);
  }

  MeritFunction.initConstrViolationEq = b_c;
  b_c = 0.0;
  for (k = 0; k < mIneq; k++) {
    scale = AEBController_DW.TrialState.cIneq.data[k];
    if (scale > 0.0) {
      b_c += scale;
    }
  }

  MeritFunction.initConstrViolationIneq = b_c;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  AEBController_factoryConstruct_oo(mConstrMax, mConstrMax,
    &AEBController_DW.QRManager.ldq, AEBController_DW.QRManager.QR.size,
    AEBController_DW.QRManager.Q.data, AEBController_DW.QRManager.Q.size,
    AEBController_DW.QRManager.jpvt.data, AEBController_DW.QRManager.jpvt.size,
    &AEBController_DW.QRManager.mrows, &AEBController_DW.QRManager.ncols,
    AEBController_DW.QRManager.tau.size, &AEBController_DW.QRManager.minRowCol,
    &AEBController_DW.QRManager.usedPivoting);
  AEBController_factoryConstruct_oog(mConstrMax,
    AEBController_DW.CholManager.FMat.size, &AEBController_DW.CholManager.ldm,
    &AEBController_DW.CholManager.ndims, &AEBController_DW.CholManager.info,
    &AEBController_DW.CholManager.scaleFactor,
    &AEBController_DW.CholManager.ConvexCheck,
    &AEBController_DW.CholManager.regTol_,
    &AEBController_DW.CholManager.workspace_,
    &AEBController_DW.CholManager.workspace2_);
  tmp_size[0] = bineq_size[0];
  tmp_size_0[0] = mNonlinIneq;
  AEBController_driver(bineq_data, lb, &AEBController_DW.TrialState,
                       &MeritFunction, &AEBController_DW.FcnEvaluator,
                       &AEBController_DW.memspace, &AEBController_DW.WorkingSet,
                       &AEBController_DW.QRManager,
                       &AEBController_DW.CholManager,
                       &AEBController_DW.QPObjective, tmp_size, tmp_size_0,
                       AEBController_DW.unusedExpr);
  *fval = AEBController_DW.TrialState.sqpFval;
  *exitflag = AEBController_DW.TrialState.sqpExitFlag;
  *output_iterations = AEBController_DW.TrialState.sqpIterations;
  *output_funcCount = AEBController_DW.TrialState.FunctionEvaluations;
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  b_c = 0.0;
  scale = 3.3121686421112381E-170;
  for (mNonlinIneq = 0; mNonlinIneq < 75; mNonlinIneq++) {
    x[mNonlinIneq] = AEBController_DW.TrialState.xstarsqp[mNonlinIneq];
    absxk = std::abs(AEBController_DW.TrialState.delta_x.data[mNonlinIneq]);
    if (absxk > scale) {
      t = scale / absxk;
      b_c = b_c * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_c += t * t;
    }
  }

  *output_stepsize = scale * std::sqrt(b_c);
  *output_lssteplength = AEBController_DW.TrialState.steplength;
  *output_firstorderopt = MeritFunction.firstOrderOpt;
}

void ACCWithSensorFusionModelClass::AEBController_helperEKFStateFcn(real_T xk[7],
  const real_T u[12])
{
  real_T b[7];
  real_T Cf;
  real_T Cr;
  real_T Iz;
  real_T delta;
  real_T lf;
  real_T lr;
  real_T m;
  real_T uk_idx_0;
  real_T uk_idx_1;
  real_T uk_idx_2;

  // Start for MATLABSystem: '<S12>/MATLAB System'
  //  State function used by the Extended Kalman Filter block.
  //
  //  Inputs:
  //            xk: Current state values
  //            u:  Current input values
  //  Outputs:
  //            xk1: Updated state values based on xk and uk
  //
  //    This is a helper function for example purposes and may be removed or
  //    modified in the future.
  //  Copyright 2021 The MathWorks, Inc.
  uk_idx_0 = u[0];
  uk_idx_1 = u[1];
  uk_idx_2 = u[2];
  m = u[5];
  Iz = u[6];
  Cf = u[7];
  Cr = u[8];
  lf = u[9];
  lr = u[10];

  //  This function uses the Euler method to discretize the augmented model
  //  used by the AEB controller. This discrete model is used by the
  //  Extended Kalman Filter to estimate the states of our ego car.
  //  Discretize in 'M' steps. Higher the value of 'M', more the accuracy
  delta = u[4] / 10.0;
  b[6] = u[3];
  for (int32_T ct = 0; ct < 10; ct++) {
    real_T b_tmp;
    real_T b_tmp_tmp;

    //  This function represents the state derivative equation for the augmented 
    //  vehicle dynamics model.
    //
    //  States x = [  lateral velocity (Vy)
    //                yaw rate (psi_dot)
    //                longitudinal velocity (Vx)
    //                longitudinal acceleration (Vx_dot)
    //                lateral deviation (e1)
    //                relative yaw angle (e2)
    //                output disturbance of relative yaw angle (xOD)];
    //
    //  Inputs u = [  acceleration
    //                steering angle
    //                road curvature * Vx (measured disturbance)
    //                white noise (unmeasured disturbance)];
    //   m            Total mass of vehicle                          (kg)
    //  Iz            Yaw moment of inertia of vehicle               (m*N*s^2)
    //  lf            Longitudinal distance from c.g. to front tires (m)
    //  lr            Longitudinal distance from c.g. to rear tires  (m)
    //  Cf            Cornering stiffness of front tires             (N/rad)
    //  Cr            Cornering stiffness of rear tires              (N/rad)
    //  tau           Longitudinal time constant (throttle)          (N/A)
    //
    //  Outputs:
    //      dxdt = state derivatives
    //
    //    This is a helper function for example purposes and may be removed or
    //    modified in the future.
    //  Copyright 2021 The MathWorks, Inc.
    //  State Equations
    b_tmp_tmp = 2.0 * Cf * lf;
    b_tmp = -(b_tmp_tmp - 2.0 * Cr * lr);
    b[0] = (-(2.0 * Cf + 2.0 * Cr) / m / xk[2] * xk[0] + (b_tmp / m / xk[2] -
             xk[2]) * xk[1]) + 2.0 * Cf / m * uk_idx_1;

    //  Vy
    b[1] = (-(2.0 * Cf * (lf * lf) + 2.0 * Cr * (lr * lr)) / Iz / xk[2] * xk[1]
            + b_tmp / Iz / xk[2] * xk[0]) + b_tmp_tmp / Iz * uk_idx_1;

    //  psi_dot
    b[2] = xk[0] * xk[1] + xk[3];

    //  Vx
    b[3] = 1.0 / u[11] * (-xk[3] + uk_idx_0);

    //  Vx_dot
    b[4] = xk[2] * xk[5] + xk[0];

    //  e1
    b[5] = xk[1] - uk_idx_2;

    //  e2
    //  xOD
    for (int32_T i = 0; i < 7; i++) {
      xk[i] += delta * b[i];
    }
  }

  // End of Start for MATLABSystem: '<S12>/MATLAB System'
}

real_T ACCWithSensorFusionModelClass::AEBController_xnrm2_po(int32_T n, const
  real_T x[98], int32_T ix0)
{
  real_T y;

  // Start for MATLABSystem: '<S12>/MATLAB System'
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<S12>/MATLAB System'
  return y;
}

void ACCWithSensorFusionModelClass::AEBController_qr_c0(const real_T A[98],
  real_T Q[98], real_T R[49])
{
  real_T b_tau[7];
  real_T work[7];
  real_T b_atmp;
  real_T beta1;
  real_T c_A;
  int32_T exitg1;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T ii;
  int32_T itau;
  int32_T ix0;
  int32_T jA;
  int32_T knt;
  int32_T lastv;
  boolean_T exitg2;
  for (i = 0; i < 7; i++) {
    // Start for MATLABSystem: '<S12>/MATLAB System'
    b_tau[i] = 0.0;
  }

  // Start for MATLABSystem: '<S12>/MATLAB System'
  std::memcpy(&Q[0], &A[0], 98U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    // Start for MATLABSystem: '<S12>/MATLAB System'
    work[i] = 0.0;
  }

  // Start for MATLABSystem: '<S12>/MATLAB System'
  for (itau = 0; itau < 7; itau++) {
    ii = itau * 14 + itau;
    ix0 = ii + 2;
    b_atmp = Q[ii];
    b_tau[itau] = 0.0;
    beta1 = AEBController_xnrm2_po(13 - itau, Q, ii + 2);
    if (beta1 != 0.0) {
      c_A = Q[ii];
      beta1 = rt_hypotd_snf(c_A, beta1);
      if (c_A >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          i = ii - itau;
          for (lastv = ix0; lastv <= i + 14; lastv++) {
            Q[lastv - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

        beta1 = rt_hypotd_snf(b_atmp, AEBController_xnrm2_po(13 - itau, Q, ii +
          2));
        if (b_atmp >= 0.0) {
          beta1 = -beta1;
        }

        b_tau[itau] = (beta1 - b_atmp) / beta1;
        b_atmp = 1.0 / (b_atmp - beta1);
        for (lastv = ix0; lastv <= i + 14; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        for (lastv = 0; lastv <= knt; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        b_atmp = beta1;
      } else {
        b_tau[itau] = (beta1 - c_A) / beta1;
        b_atmp = 1.0 / (c_A - beta1);
        i = ii - itau;
        for (lastv = ix0; lastv <= i + 14; lastv++) {
          Q[lastv - 1] *= b_atmp;
        }

        b_atmp = beta1;
      }
    }

    Q[ii] = b_atmp;
    if (itau + 1 < 7) {
      Q[ii] = 1.0;
      ix0 = ii + 15;
      if (b_tau[itau] != 0.0) {
        lastv = 14 - itau;
        i = ii - itau;
        while ((lastv > 0) && (Q[i + 13] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 5 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 14 + ii;
          ia = i + 15;
          do {
            exitg1 = 0;
            if (ia <= (i + lastv) + 14) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (14 * knt + ii) + 15;
          for (iac = ix0; iac <= i; iac += 14) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[(ii + ia) - iac] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 15, 14);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 15;
              ix0 = (lastv + jA) + 14;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 15] * beta1;
              }
            }

            jA += 14;
          }
        }
      }

      Q[ii] = b_atmp;
    }
  }

  for (ii = 0; ii < 7; ii++) {
    for (itau = 0; itau <= ii; itau++) {
      // Start for MATLABSystem: '<S12>/MATLAB System'
      R[itau + 7 * ii] = Q[14 * ii + itau];
    }

    for (itau = ii + 2; itau < 8; itau++) {
      R[(itau + 7 * ii) - 1] = 0.0;
    }

    // Start for MATLABSystem: '<S12>/MATLAB System'
    work[ii] = 0.0;
  }

  // Start for MATLABSystem: '<S12>/MATLAB System'
  for (itau = 6; itau >= 0; itau--) {
    ii = (itau * 14 + itau) + 14;
    if (itau + 1 < 7) {
      Q[ii - 14] = 1.0;
      ix0 = ii + 1;
      if (b_tau[itau] != 0.0) {
        lastv = 14 - itau;
        i = (ii - itau) - 1;
        while ((lastv > 0) && (Q[i] == 0.0)) {
          lastv--;
          i--;
        }

        knt = 5 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt + 1 > 0)) {
          i = knt * 14 + ii;
          ia = i + 1;
          do {
            exitg1 = 0;
            if (ia <= i + lastv) {
              if (Q[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = -1;
      }

      if (lastv > 0) {
        if (knt + 1 != 0) {
          for (i = 0; i <= knt; i++) {
            work[i] = 0.0;
          }

          i = (14 * knt + ii) + 1;
          for (iac = ix0; iac <= i; iac += 14) {
            beta1 = 0.0;
            jA = iac + lastv;
            for (ia = iac; ia < jA; ia++) {
              beta1 += Q[((ii + ia) - iac) - 14] * Q[ia - 1];
            }

            ia = div_nde_s32_floor((iac - ii) - 1, 14);
            work[ia] += beta1;
          }
        }

        if (!(-b_tau[itau] == 0.0)) {
          jA = ii;
          for (ia = 0; ia <= knt; ia++) {
            beta1 = work[ia];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[itau];
              i = jA + 1;
              ix0 = lastv + jA;
              for (iac = i; iac <= ix0; iac++) {
                Q[iac - 1] += Q[((ii + iac) - jA) - 15] * beta1;
              }
            }

            jA += 14;
          }
        }
      }
    }

    i = ii - itau;
    for (lastv = ii - 12; lastv <= i; lastv++) {
      Q[lastv - 1] *= -b_tau[itau];
    }

    Q[ii - 14] = 1.0 - b_tau[itau];
    for (i = 0; i < itau; i++) {
      Q[(ii - i) - 15] = 0.0;
    }
  }
}

// System initialize for referenced model: 'AEBController'
void ACCWithSensorFusionModelClass::init(void)
{
  // Start for DataStoreMemory: '<S9>/DataStoreMemory - P'
  std::memcpy(&AEBController_DW.P[0], &rtCP_DataStoreMemoryP_InitialValue[0],
              49U * sizeof(real_T));

  // Start for DataStoreMemory: '<S9>/DataStoreMemory - x'
  for (int32_T i = 0; i < 7; i++) {
    AEBController_DW.x[i] = rtCP_DataStoreMemoryx_InitialValue[i];
  }

  // End of Start for DataStoreMemory: '<S9>/DataStoreMemory - x'

  // InitializeConditions for Delay: '<S3>/Delay'
  AEBController_DW.Delay_DSTATE[0] = 0.2;
  AEBController_DW.Delay_DSTATE[1] = 0.3;

  // InitializeConditions for Delay: '<S16>/mv_Delay'
  AEBController_DW.icLoad = true;

  // InitializeConditions for Delay: '<S16>/x_Delay'
  AEBController_DW.icLoad_f = true;

  // InitializeConditions for Delay: '<S16>/slack_delay'
  AEBController_DW.icLoad_k = true;

  // SystemInitialize for Enabled SubSystem: '<S9>/Correct1'
  // Start for MATLABSystem: '<S10>/MATLAB System'
  AEBController_DW.objisempty_h = true;
  AEBController_DW.obj.isInitialized = 1;

  // End of SystemInitialize for SubSystem: '<S9>/Correct1'

  // SystemInitialize for Atomic SubSystem: '<S9>/Predict'
  // Start for MATLABSystem: '<S12>/MATLAB System'
  AEBController_DW.objisempty = true;
  AEBController_DW.obj_i.isInitialized = 1;

  // End of SystemInitialize for SubSystem: '<S9>/Predict'

  // ConstCode for Constant: '<S8>/NLMPC Reference Values'
  AEBController_DW.driver_set_velocity[0] = 10.0;
  AEBController_DW.driver_set_velocity[1] = 0.0;
  AEBController_DW.driver_set_velocity[2] = 0.0;
}

// Output and update for referenced model: 'AEBController'
void ACCWithSensorFusionModelClass::step(const real_T *rtu_LongitudinalVelocity,
  const real_T rtu_CurvatureSequence[10], const real_T *rtu_LateralDeviation,
  const real_T *rtu_RelativeYawAngle, const real_T *rtu_RelativeDistance, const
  real_T *rtu_RelativeVelocity, uint8_T *rty_AEBStatus, uint8_T *rty_FCWActivate,
  boolean_T *rty_EgoCarStop, real_T *rty_SteeringAngle, real_T *rty_Throttle)
{
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T expl_temp_2;
  s_tD85q5PFvXiFImVLjXNPSB_AEBController_T expl_temp_4;
  sjqToqDl6Xs23mBXoA194vG_AEBController_T expl_temp_3;
  real_T tmp_data[320];
  real_T a__1_1[98];
  real_T y_1[98];
  real_T B_data[80];
  real_T Bu[80];
  real_T X[77];
  real_T rtb_Selector_0[75];
  real_T tmp[75];
  real_T z[75];
  real_T a__1_0[70];
  real_T y_0[70];
  real_T rtb_Selector[63];
  real_T A[49];
  real_T y[49];
  real_T U[44];
  real_T K_0[30];
  real_T a__1[30];
  real_T C[21];
  real_T C_0[21];
  real_T K[21];
  real_T b_dHdx[21];
  real_T rtb_Selector1_0[20];
  real_T rtb_Selector1[18];
  real_T rtb_ZeroOrderHold1[12];
  real_T rtb_Product[10];
  real_T R[9];
  real_T Sy[9];
  real_T imvec[7];
  real_T z_0[7];
  real_T a[4];
  real_T expl_temp_0;
  real_T expl_temp_1;
  real_T ic_idx_0;
  real_T rtb_FCW_delTVA;
  real_T rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1;
  real_T rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2;
  real_T rtb_Vxrho;
  real_T z_idx_0;
  real_T z_idx_1;
  int32_T b_j_0[2];
  int32_T aoffset;
  int32_T b_j;
  int32_T coffset;
  int32_T i;
  int32_T idx;
  int32_T tmp_0;
  int32_T tmp_1;
  int32_T tmp_2;
  int32_T tmp_3;
  int32_T tmp_4;
  char_T expl_temp[3];
  int8_T ii_data[80];
  boolean_T x[80];
  boolean_T rtb_RelationalOperator1;
  static const int8_T g[30] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const real_T h[20] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

  static const real_T a_0[80] = { 0.16666666666666666, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.16666666666666666, 0.0, 0.0, 0.0,
    0.0, 0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T tmp_size[2];
  int32_T B_size[1];
  boolean_T exitg1;

  // Sum: '<S6>/Sum' incorporates:
  //   Constant: '<S6>/const1'

  AEBController_DW.headway = *rtu_RelativeDistance - 3.7;

  // Abs: '<S6>/Abs'
  rtb_Vxrho = std::abs(*rtu_RelativeVelocity);

  // Saturate: '<S6>/Saturation'
  if (rtb_Vxrho > 100.0) {
    rtb_Vxrho = 100.0;
  } else if (rtb_Vxrho < 0.01) {
    rtb_Vxrho = 0.01;
  }

  // Product: '<S6>/Divide3' incorporates:
  //   Saturate: '<S6>/Saturation'

  AEBController_DW.TTC = AEBController_DW.headway / rtb_Vxrho;

  // Signum: '<S6>/Sign'
  rtb_Vxrho = *rtu_RelativeVelocity;
  if (rtIsNaN(rtb_Vxrho)) {
    rtb_FCW_delTVA = (rtNaN);
  } else if (rtb_Vxrho < 0.0) {
    rtb_FCW_delTVA = -1.0;
  } else {
    rtb_FCW_delTVA = (rtb_Vxrho > 0.0);
  }

  // Product: '<S6>/Divide11' incorporates:
  //   Signum: '<S6>/Sign'

  rtb_Vxrho = AEBController_DW.TTC * rtb_FCW_delTVA;

  // Product: '<S5>/Divide12' incorporates:
  //   Constant: '<S5>/const6'

  rtb_FCW_delTVA = *rtu_LongitudinalVelocity / 4.0;

  // Sum: '<S5>/Sum1' incorporates:
  //   Constant: '<S5>/const3'

  AEBController_DW.FCWstoppingTime = rtb_FCW_delTVA + 1.2;

  // Product: '<S5>/Divide5' incorporates:
  //   Constant: '<S1>/const6'

  rtb_FCW_delTVA = *rtu_LongitudinalVelocity / 3.8;

  // Sum: '<S5>/Sum2' incorporates:
  //   Constant: '<S5>/const7'

  AEBController_DW.PB1stoppingTime = rtb_FCW_delTVA + 0.08;

  // Product: '<S5>/Divide3' incorporates:
  //   Constant: '<S1>/const4'

  rtb_FCW_delTVA = *rtu_LongitudinalVelocity / 5.3;

  // Sum: '<S5>/Sum3' incorporates:
  //   Constant: '<S5>/const7'

  AEBController_DW.PB2stoppingTime = rtb_FCW_delTVA + 0.08;

  // Product: '<S5>/Divide1' incorporates:
  //   Constant: '<S1>/const5'

  rtb_FCW_delTVA = *rtu_LongitudinalVelocity / 9.8;

  // Sum: '<S5>/Sum4' incorporates:
  //   Constant: '<S5>/const7'

  AEBController_DW.FBstoppingTime = rtb_FCW_delTVA + 0.08;

  // RelationalOperator: '<S1>/Relational Operator1' incorporates:
  //   Constant: '<S1>/const3'

  rtb_RelationalOperator1 = (*rtu_LongitudinalVelocity <= 0.27777777777777779);

  // Logic: '<S1>/AND' incorporates:
  //   Delay: '<S1>/Delay'

  *rty_EgoCarStop = (rtb_RelationalOperator1 && (AEBController_DW.Delay_DSTATE_c
    != 0));

  // Chart: '<S1>/AEBLogic' incorporates:
  //   Constant: '<S1>/const4'
  //   Constant: '<S1>/const5'
  //   Constant: '<S1>/const6'

  if (AEBController_DW.is_active_c5_AEBController == 0) {
    AEBController_DW.is_active_c5_AEBController = 1U;
    AEBController_DW.is_c5_AEBController = AEBController_IN_Default;
    *rty_AEBStatus = 0U;
    *rty_FCWActivate = 0U;
    AEBController_DW.decel = 0.0;
  } else {
    switch (AEBController_DW.is_c5_AEBController) {
     case AEBController_IN_Default:
      *rty_AEBStatus = 0U;
      *rty_FCWActivate = 0U;
      if ((std::abs(rtb_Vxrho) < AEBController_DW.FCWstoppingTime) && (rtb_Vxrho
           < 0.0)) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_FCW;
        *rty_AEBStatus = 0U;
        *rty_FCWActivate = 1U;
        AEBController_DW.decel = 0.0;
      }
      break;

     case AEBController_IN_FCW:
      *rty_AEBStatus = 0U;
      *rty_FCWActivate = 1U;
      rtb_FCW_delTVA = std::abs(rtb_Vxrho);
      if ((rtb_FCW_delTVA < AEBController_DW.PB1stoppingTime) && (rtb_Vxrho <
           0.0)) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Partial_Braking1;
        *rty_AEBStatus = 1U;
        *rty_FCWActivate = 1U;
        AEBController_DW.decel = 3.8;
      } else if (rtb_FCW_delTVA >= 1.2 * AEBController_DW.FCWstoppingTime) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Default;
        *rty_AEBStatus = 0U;
        *rty_FCWActivate = 0U;
        AEBController_DW.decel = 0.0;
      }
      break;

     case AEBController_IN_Full_Braking:
      *rty_AEBStatus = 3U;
      *rty_FCWActivate = 1U;
      if (*rty_EgoCarStop) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Default;
        *rty_AEBStatus = 0U;
        *rty_FCWActivate = 0U;
        AEBController_DW.decel = 0.0;
      }
      break;

     case AEBController_IN_Partial_Braking1:
      *rty_AEBStatus = 1U;
      *rty_FCWActivate = 1U;
      if ((std::abs(rtb_Vxrho) < AEBController_DW.PB2stoppingTime) && (rtb_Vxrho
           < 0.0)) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Partial_Braking2;
        *rty_AEBStatus = 2U;
        *rty_FCWActivate = 1U;
        AEBController_DW.decel = 5.3;
      } else if (*rty_EgoCarStop) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Default;
        *rty_AEBStatus = 0U;
        *rty_FCWActivate = 0U;
        AEBController_DW.decel = 0.0;
      }
      break;

     default:
      // case IN_Partial_Braking2:
      *rty_AEBStatus = 2U;
      *rty_FCWActivate = 1U;
      if ((std::abs(rtb_Vxrho) < AEBController_DW.FBstoppingTime) && (rtb_Vxrho <
           0.0)) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Full_Braking;
        *rty_AEBStatus = 3U;
        *rty_FCWActivate = 1U;
        AEBController_DW.decel = 9.8;
      } else if (*rty_EgoCarStop) {
        AEBController_DW.is_c5_AEBController = AEBController_IN_Default;
        *rty_AEBStatus = 0U;
        *rty_FCWActivate = 0U;
        AEBController_DW.decel = 0.0;
      }
      break;
    }
  }

  // End of Chart: '<S1>/AEBLogic'

  // Logic: '<S2>/OR1' incorporates:
  //   Delay: '<S2>/Delay2'

  rtb_RelationalOperator1 = ((*rty_AEBStatus != 0) ||
    AEBController_DW.Delay2_DSTATE);

  // Saturate: '<S7>/Saturation'
  if (*rtu_LongitudinalVelocity <= 0.2) {
    rtb_Vxrho = 0.2;
  } else {
    rtb_Vxrho = *rtu_LongitudinalVelocity;
  }

  // End of Saturate: '<S7>/Saturation'

  // Outputs for Enabled SubSystem: '<S9>/Correct1' incorporates:
  //   EnablePort: '<S10>/Enable'

  // SignalConversion generated from: '<S10>/MATLAB System'
  rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1 = *rtu_LateralDeviation;
  rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 = *rtu_RelativeYawAngle;

  // MATLABSystem: '<S10>/MATLAB System' incorporates:
  //   DataStoreRead: '<S10>/Data Store ReadP'
  //   DataStoreRead: '<S10>/Data Store ReadX'

  //  Measurement function used by the Extended Kalman Filter block.
  //
  //  Inputs:
  //            x: Current state values
  //  Outputs:
  //            y: Output vector - [Vx e1 e2+x_od]
  //
  //    This is a helper function for example purposes and may be removed or
  //    modified in the future.
  //  Copyright 2021 The MathWorks, Inc.
  z_idx_0 = AEBController_DW.x[2];
  z_idx_1 = AEBController_DW.x[4];
  ic_idx_0 = AEBController_DW.x[5] + AEBController_DW.x[6];
  for (b_j = 0; b_j < 7; b_j++) {
    for (i = 0; i < 7; i++) {
      imvec[i] = AEBController_DW.x[i];
    }

    rtb_FCW_delTVA = 1.4901161193847656E-8 * std::abs(AEBController_DW.x[b_j]);
    if ((rtb_FCW_delTVA <= 1.4901161193847656E-8) || rtIsNaN(rtb_FCW_delTVA)) {
      rtb_FCW_delTVA = 1.4901161193847656E-8;
    }

    imvec[b_j] = AEBController_DW.x[b_j] + rtb_FCW_delTVA;

    //  Measurement function used by the Extended Kalman Filter block.
    //
    //  Inputs:
    //            x: Current state values
    //  Outputs:
    //            y: Output vector - [Vx e1 e2+x_od]
    //
    //    This is a helper function for example purposes and may be removed or
    //    modified in the future.
    //  Copyright 2021 The MathWorks, Inc.
    b_dHdx[3 * b_j] = (imvec[2] - z_idx_0) / rtb_FCW_delTVA;
    b_dHdx[3 * b_j + 1] = (imvec[4] - z_idx_1) / rtb_FCW_delTVA;
    b_dHdx[3 * b_j + 2] = ((imvec[5] + imvec[6]) - ic_idx_0) / rtb_FCW_delTVA;
  }

  //  Measurement function used by the Extended Kalman Filter block.
  //
  //  Inputs:
  //            x: Current state values
  //  Outputs:
  //            y: Output vector - [Vx e1 e2+x_od]
  //
  //    This is a helper function for example purposes and may be removed or
  //    modified in the future.
  //  Copyright 2021 The MathWorks, Inc.
  for (b_j = 0; b_j < 3; b_j++) {
    coffset = b_j * 7 - 1;
    for (i = 0; i < 7; i++) {
      aoffset = i * 7 - 1;
      rtb_FCW_delTVA = 0.0;
      for (idx = 0; idx < 7; idx++) {
        rtb_FCW_delTVA += AEBController_DW.P[(aoffset + idx) + 1] * b_dHdx[idx *
          3 + b_j];
      }

      K[(coffset + i) + 1] = rtb_FCW_delTVA;
    }
  }

  for (idx = 0; idx < 7; idx++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    K_0[idx] = K[idx];
    K_0[idx + 10] = K[idx + 7];
    K_0[idx + 20] = K[idx + 14];
  }

  for (idx = 0; idx < 3; idx++) {
    // Start for MATLABSystem: '<S10>/MATLAB System' incorporates:
    //   Constant: '<S9>/R1'

    K_0[10 * idx + 7] = rtCP_R1_Value[idx];
    K_0[10 * idx + 8] = rtCP_R1_Value[idx + 3];
    K_0[10 * idx + 9] = rtCP_R1_Value[idx + 6];
  }

  // MATLABSystem: '<S10>/MATLAB System'
  AEBController_qr(K_0, a__1, R);
  for (idx = 0; idx < 3; idx++) {
    Sy[3 * idx] = R[idx];
    Sy[3 * idx + 1] = R[idx + 3];
    Sy[3 * idx + 2] = R[idx + 6];
  }

  for (idx = 0; idx < 7; idx++) {
    for (b_j = 0; b_j < 7; b_j++) {
      // Start for MATLABSystem: '<S10>/MATLAB System' incorporates:
      //   DataStoreRead: '<S10>/Data Store ReadP'

      rtb_FCW_delTVA = 0.0;
      for (i = 0; i < 7; i++) {
        rtb_FCW_delTVA += AEBController_DW.P[7 * i + idx] * AEBController_DW.P[7
          * i + b_j];
      }

      A[idx + 7 * b_j] = rtb_FCW_delTVA;
    }
  }

  for (idx = 0; idx < 3; idx++) {
    for (b_j = 0; b_j < 7; b_j++) {
      // Start for MATLABSystem: '<S10>/MATLAB System'
      rtb_FCW_delTVA = 0.0;
      for (i = 0; i < 7; i++) {
        rtb_FCW_delTVA += A[7 * i + b_j] * b_dHdx[3 * i + idx];
      }

      K[idx + 3 * b_j] = rtb_FCW_delTVA;
    }
  }

  // MATLABSystem: '<S10>/MATLAB System'
  for (b_j = 0; b_j < 7; b_j++) {
    C[3 * b_j] = K[3 * b_j];

    // Start for MATLABSystem: '<S10>/MATLAB System'
    i = 3 * b_j + 1;
    C[i] = K[i];

    // Start for MATLABSystem: '<S10>/MATLAB System'
    i = 3 * b_j + 2;
    C[i] = K[i];
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  AEBController_trisolve(Sy, C);

  // MATLABSystem: '<S10>/MATLAB System'
  for (b_j = 0; b_j < 7; b_j++) {
    C_0[3 * b_j] = C[3 * b_j];

    // Start for MATLABSystem: '<S10>/MATLAB System'
    i = 3 * b_j + 1;
    C_0[i] = C[i];

    // Start for MATLABSystem: '<S10>/MATLAB System'
    i = 3 * b_j + 2;
    C_0[i] = C[i];
  }

  for (idx = 0; idx < 3; idx++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    R[3 * idx] = Sy[idx];
    R[3 * idx + 1] = Sy[idx + 3];
    R[3 * idx + 2] = Sy[idx + 6];
  }

  // Start for MATLABSystem: '<S10>/MATLAB System'
  AEBController_trisolve_l(R, C_0);

  // MATLABSystem: '<S10>/MATLAB System'
  for (idx = 0; idx < 7; idx++) {
    K[idx] = C_0[3 * idx];
    K[idx + 7] = C_0[3 * idx + 1];
    K[idx + 14] = C_0[3 * idx + 2];
  }

  for (idx = 0; idx < 21; idx++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    C[idx] = -K[idx];
  }

  // MATLABSystem: '<S10>/MATLAB System' incorporates:
  //   DataStoreRead: '<S10>/Data Store ReadP'

  for (idx = 0; idx < 7; idx++) {
    // Start for MATLABSystem: '<S10>/MATLAB System'
    rtb_FCW_delTVA = b_dHdx[3 * idx + 1];
    z_idx_0 = b_dHdx[3 * idx];
    z_idx_1 = b_dHdx[3 * idx + 2];
    for (b_j = 0; b_j < 7; b_j++) {
      A[b_j + 7 * idx] = (C[b_j + 7] * rtb_FCW_delTVA + z_idx_0 * C[b_j]) +
        C[b_j + 14] * z_idx_1;
    }
  }

  for (i = 0; i < 7; i++) {
    b_j = 7 * i + i;
    A[b_j]++;
  }

  for (b_j = 0; b_j < 7; b_j++) {
    coffset = b_j * 7 - 1;
    for (i = 0; i < 7; i++) {
      aoffset = i * 7 - 1;
      rtb_FCW_delTVA = 0.0;
      for (idx = 0; idx < 7; idx++) {
        rtb_FCW_delTVA += AEBController_DW.P[(aoffset + idx) + 1] * A[idx * 7 +
          b_j];
      }

      y[(coffset + i) + 1] = rtb_FCW_delTVA;
    }

    // Start for MATLABSystem: '<S10>/MATLAB System' incorporates:
    //   Constant: '<S9>/R1'
    //   DataStoreRead: '<S10>/Data Store ReadP'

    rtb_FCW_delTVA = K[b_j + 7];
    z_idx_0 = K[b_j];
    z_idx_1 = K[b_j + 14];
    for (idx = 0; idx < 3; idx++) {
      C[idx + 3 * b_j] = (rtCP_R1_Value[3 * idx + 1] * rtb_FCW_delTVA +
                          rtCP_R1_Value[3 * idx] * z_idx_0) + rtCP_R1_Value[3 *
        idx + 2] * z_idx_1;
    }
  }

  for (idx = 0; idx < 7; idx++) {
    for (b_j = 0; b_j < 7; b_j++) {
      // Start for MATLABSystem: '<S10>/MATLAB System'
      y_0[b_j + 10 * idx] = y[7 * idx + b_j];
    }

    // Start for MATLABSystem: '<S10>/MATLAB System'
    y_0[10 * idx + 7] = C[3 * idx];
    y_0[10 * idx + 8] = C[3 * idx + 1];
    y_0[10 * idx + 9] = C[3 * idx + 2];
  }

  // MATLABSystem: '<S10>/MATLAB System'
  AEBController_qr_c(y_0, a__1_0, A);

  // Start for MATLABSystem: '<S10>/MATLAB System' incorporates:
  //   DataStoreRead: '<S10>/Data Store ReadX'
  //   SignalConversion generated from: '<S10>/MATLAB System'

  rtb_FCW_delTVA = rtb_Vxrho - AEBController_DW.x[2];
  rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1 -= AEBController_DW.x[4];
  ic_idx_0 = rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 - ic_idx_0;
  for (idx = 0; idx < 7; idx++) {
    // DataStoreWrite: '<S10>/Data Store WriteP' incorporates:
    //   MATLABSystem: '<S10>/MATLAB System'
    //
    for (b_j = 0; b_j < 7; b_j++) {
      AEBController_DW.P[b_j + 7 * idx] = A[7 * b_j + idx];
    }

    // End of DataStoreWrite: '<S10>/Data Store WriteP'

    // DataStoreWrite: '<S10>/Data Store WriteX' incorporates:
    //   DataStoreRead: '<S10>/Data Store ReadX'
    //   MATLABSystem: '<S10>/MATLAB System'
    //
    AEBController_DW.x[idx] += (K[idx + 7] *
      rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1 + K[idx] *
      rtb_FCW_delTVA) + K[idx + 14] * ic_idx_0;
  }

  // End of Outputs for SubSystem: '<S9>/Correct1'

  // Product: '<S8>/Product' incorporates:
  //   DataStoreRead: '<S11>/Data Store Read'

  for (i = 0; i < 10; i++) {
    rtb_Product[i] = rtu_CurvatureSequence[i] * AEBController_DW.x[2];
  }

  // End of Product: '<S8>/Product'

  // Delay: '<S16>/mv_Delay' incorporates:
  //   Constant: '<S16>/ones'
  //   Delay: '<S3>/Delay'
  //   Product: '<S16>/Product'

  if (AEBController_DW.icLoad) {
    // Delay: '<S3>/Delay' incorporates:
    //   Product: '<S16>/Product'

    rtb_FCW_delTVA = AEBController_DW.Delay_DSTATE[0];
    ic_idx_0 = AEBController_DW.Delay_DSTATE[1];
    for (idx = 0; idx < 11; idx++) {
      AEBController_DW.mv_Delay_DSTATE[idx] = rtb_FCW_delTVA;
      AEBController_DW.mv_Delay_DSTATE[idx + 11] = ic_idx_0;
    }
  }

  // Selector: '<S16>/Selector1' incorporates:
  //   Constant: '<S16>/Constant1'
  //   Delay: '<S16>/mv_Delay'

  for (idx = 0; idx < 9; idx++) {
    rtb_FCW_delTVA = rtCP_Constant1_Value_j[idx];
    rtb_Selector1[idx] = AEBController_DW.mv_Delay_DSTATE[static_cast<int32_T>
      (rtb_FCW_delTVA) - 1];
    rtb_Selector1[idx + 9] = AEBController_DW.mv_Delay_DSTATE
      [static_cast<int32_T>(rtb_FCW_delTVA) + 10];
  }

  // End of Selector: '<S16>/Selector1'

  // Delay: '<S16>/x_Delay' incorporates:
  //   Constant: '<S16>/ones'
  //   DataStoreRead: '<S11>/Data Store Read'
  //   Product: '<S16>/Product1'

  if (AEBController_DW.icLoad_f) {
    for (idx = 0; idx < 7; idx++) {
      for (b_j = 0; b_j < 11; b_j++) {
        AEBController_DW.x_Delay_DSTATE[b_j + 11 * idx] = AEBController_DW.x[idx];
      }
    }
  }

  // Selector: '<S16>/Selector' incorporates:
  //   Constant: '<S16>/Constant'
  //   Delay: '<S16>/x_Delay'

  for (idx = 0; idx < 7; idx++) {
    for (b_j = 0; b_j < 9; b_j++) {
      rtb_Selector[b_j + 9 * idx] = AEBController_DW.x_Delay_DSTATE[(11 * idx +
        static_cast<int32_T>(rtCP_Constant_Value_k[b_j])) - 1];
    }
  }

  // End of Selector: '<S16>/Selector'

  // Delay: '<S16>/slack_delay' incorporates:
  //   Constant: '<S14>/e.init_zero'

  if (AEBController_DW.icLoad_k) {
    AEBController_DW.slack_delay_DSTATE = 0.0;
  }

  // MATLAB Function: '<S15>/NLMPC' incorporates:
  //   BusCreator: '<S8>/Bus Creator'
  //   Constant: '<S8>/Constant'
  //   Constant: '<S8>/Constant1'
  //   Constant: '<S8>/Constant2'
  //   Constant: '<S8>/Constant3'
  //   Constant: '<S8>/Constant4'
  //   Constant: '<S8>/Constant5'
  //   Constant: '<S8>/Constant6'
  //   Constant: '<S8>/NLMPC Reference Values'
  //   DataStoreRead: '<S11>/Data Store Read'
  //   Delay: '<S16>/slack_delay'
  //   Delay: '<S3>/Delay'
  //   Product: '<S8>/Product'
  //   Selector: '<S16>/Selector'
  //   Selector: '<S16>/Selector1'

  expl_temp_2.Parameters[0] = 1575.0;
  expl_temp_2.Parameters[1] = 2875.0;
  expl_temp_2.Parameters[2] = 19000.0;
  expl_temp_2.Parameters[3] = 33000.0;
  expl_temp_2.Parameters[4] = 1.5130000000000001;
  expl_temp_2.Parameters[5] = 1.305;
  expl_temp_2.Parameters[6] = 0.5;
  expl_temp_2.ref[0] = 10.0;
  expl_temp_2.ref[10] = 0.0;
  expl_temp_2.ref[20] = 0.0;
  for (idx = 0; idx < 9; idx++) {
    expl_temp_2.ref[idx + 1] = 10.0;
    expl_temp_2.ref[idx + 11] = 0.0;
    expl_temp_2.ref[idx + 21] = 0.0;
  }

  std::memcpy(&expl_temp_2.md[0], &rtb_Product[0], 10U * sizeof(real_T));
  expl_temp_2.md[10] = rtb_Product[9];
  for (idx = 0; idx < 30; idx++) {
    expl_temp_2.OutputWeights[idx] = g[idx];
  }

  for (idx = 0; idx < 1600; idx++) {
    AEBController_DW.Au[idx] = 0.0;
  }

  for (i = 0; i < 80; i++) {
    Bu[i] = 0.0;
    x[i] = false;
  }

  ic_idx_0 = 1.0;
  rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 = 2.0;
  for (i = 0; i < 10; i++) {
    aoffset = static_cast<int32_T>(ic_idx_0);
    tmp_0 = aoffset;
    x[aoffset - 1] = false;
    b_j = aoffset + 2;
    aoffset = static_cast<int32_T>
      (rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2);
    x[aoffset - 1] = false;
    x[b_j - 1] = false;
    tmp_3 = static_cast<int32_T>(ic_idx_0 + 4.0);
    x[aoffset + 1] = false;
    tmp_4 = static_cast<int32_T>
      (rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 + 4.0);
    x[tmp_3 - 1] = true;
    tmp_1 = static_cast<int32_T>(ic_idx_0 + 6.0);
    x[tmp_4 - 1] = true;
    tmp_2 = static_cast<int32_T>
      (rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 + 6.0);
    x[tmp_1 - 1] = true;
    idx = 160 * i + tmp_0;
    AEBController_DW.Au[idx - 1] = -0.16666666666666666;
    coffset = 160 * i + aoffset;
    AEBController_DW.Au[coffset - 1] = -0.0;
    x[tmp_2 - 1] = true;
    AEBController_DW.Au[idx + 79] = -0.0;
    AEBController_DW.Au[coffset + 79] = -0.44247787610619471;
    idx = 160 * i + b_j;
    AEBController_DW.Au[idx - 1] = 0.16666666666666666;
    AEBController_DW.Au[coffset + 1] = 0.0;
    AEBController_DW.Au[idx + 79] = 0.0;
    AEBController_DW.Au[coffset + 81] = 0.44247787610619471;
    idx = 160 * i + tmp_3;
    AEBController_DW.Au[idx - 1] = -0.16666666666666666;
    coffset = 160 * i + tmp_4;
    AEBController_DW.Au[coffset - 1] = -0.0;
    AEBController_DW.Au[idx + 79] = -0.0;
    AEBController_DW.Au[coffset + 79] = -0.44247787610619471;
    idx = 160 * i + tmp_1;
    AEBController_DW.Au[idx - 1] = 0.16666666666666666;
    coffset = 160 * i + tmp_2;
    AEBController_DW.Au[coffset - 1] = 0.0;
    AEBController_DW.Au[idx + 79] = 0.0;
    AEBController_DW.Au[coffset + 79] = 0.44247787610619471;
    Bu[tmp_0 - 1] = (rtInf);
    Bu[aoffset - 1] = (rtInf);
    Bu[b_j - 1] = (rtInf);
    Bu[aoffset + 1] = (rtInf);
    Bu[tmp_3 - 1] = 0.5;
    Bu[tmp_4 - 1] = 0.5;
    Bu[tmp_1 - 1] = 0.5;
    Bu[tmp_2 - 1] = 0.5;
    if (i + 1 == 1) {
      rtb_FCW_delTVA = AEBController_DW.Delay_DSTATE[0] / 6.0;
      rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1 = rtb_FCW_delTVA;
      z_idx_0 = Bu[tmp_0 - 1] - rtb_FCW_delTVA;
      rtb_FCW_delTVA = AEBController_DW.Delay_DSTATE[1] / 2.26;
      z_idx_1 = Bu[aoffset - 1] - rtb_FCW_delTVA;
      Bu[tmp_0 - 1] = z_idx_0;
      Bu[aoffset - 1] = z_idx_1;
      z_idx_1 = Bu[aoffset + 1] + rtb_FCW_delTVA;
      Bu[b_j - 1] += rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1;
      Bu[aoffset + 1] = z_idx_1;
    } else {
      idx = (i - 1) * 160;
      coffset = tmp_0 + idx;
      AEBController_DW.Au[coffset - 1] = 0.16666666666666666;
      aoffset += idx;
      AEBController_DW.Au[aoffset - 1] = 0.0;
      AEBController_DW.Au[coffset + 79] = 0.0;
      AEBController_DW.Au[aoffset + 79] = 0.44247787610619471;
      idx += b_j;
      AEBController_DW.Au[idx - 1] = -0.16666666666666666;
      AEBController_DW.Au[aoffset + 1] = -0.0;
      AEBController_DW.Au[idx + 79] = -0.0;
      AEBController_DW.Au[aoffset + 81] = -0.44247787610619471;
    }

    ic_idx_0 += 8.0;
    rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 += 8.0;
  }

  idx = 0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 80)) {
    if (x[i]) {
      idx++;
      ii_data[idx - 1] = static_cast<int8_T>(i + 1);
      if (idx >= 80) {
        exitg1 = true;
      } else {
        i++;
      }
    } else {
      i++;
    }
  }

  if (idx < 1) {
    b_j = 0;
  } else {
    b_j = idx;
  }

  if (b_j > 0) {
    B_size[0] = b_j;
    for (idx = 0; idx < b_j; idx++) {
      B_data[idx] = Bu[ii_data[idx] - 1];
    }

    for (idx = 0; idx < 2; idx++) {
      for (coffset = 0; coffset < 10; coffset++) {
        for (i = 0; i < b_j; i++) {
          AEBController_DW.Auf_data[(i + b_j * idx) + (b_j << 1) * coffset] =
            AEBController_DW.Au[((80 * idx + ii_data[i]) + 160 * coffset) - 1];
        }
      }
    }

    b_j_0[0] = b_j;
    b_j_0[1] = 20;
    AEBController_mtimes(AEBController_DW.Auf_data, b_j_0, tmp_data, tmp_size);
    i = b_j * 70;
    for (idx = 0; idx < i; idx++) {
      AEBController_DW.A_data[idx] = 0.0;
    }

    i = b_j << 2;
    for (idx = 0; idx < i; idx++) {
      AEBController_DW.A_data[idx + b_j * 70] = tmp_data[idx];
    }

    for (idx = 0; idx < b_j; idx++) {
      AEBController_DW.A_data[(idx + b_j * 70) + i] = 0.0;
    }
  } else {
    B_size[0] = 0;
  }

  for (idx = 0; idx < 20; idx++) {
    expl_temp_2.MVScaledTarget[idx] = 0.0;
    expl_temp_2.MVRateMax[idx] = (rtInf);
    expl_temp_2.MVRateMin[idx] = (rtMinusInf);
    expl_temp_2.MVMax[idx] = 0.5;
    expl_temp_2.MVMin[idx] = -0.5;
  }

  for (idx = 0; idx < 70; idx++) {
    expl_temp_2.StateMax[idx] = (rtInf);
    expl_temp_2.StateMin[idx] = (rtMinusInf);
  }

  for (idx = 0; idx < 30; idx++) {
    expl_temp_2.OutputMax[idx] = (rtInf);
    expl_temp_2.OutputMin[idx] = (rtMinusInf);
  }

  expl_temp_2.ECRWeight = 100000.0;
  for (idx = 0; idx < 20; idx++) {
    expl_temp_2.MVRateWeights[idx] = h[idx];
    expl_temp_2.MVWeights[idx] = 0.0;
  }

  expl_temp_2.lastMV[0] = AEBController_DW.Delay_DSTATE[0];
  expl_temp_2.lastMV[1] = AEBController_DW.Delay_DSTATE[1];
  for (i = 0; i < 7; i++) {
    expl_temp_2.x[i] = AEBController_DW.x[i];
  }

  expl_temp_3.PassivityUsePredictedX = true;
  expl_temp_3.OutputPassivityIndex = 0.1;
  expl_temp_3.InputPassivityIndex = 0.0;
  expl_temp_3.UDIndex = 4.0;
  expl_temp_3.MDIndex = 3.0;
  expl_temp_3.MVIndex[0] = 1.0;
  expl_temp_3.MVIndex[1] = 2.0;
  expl_temp_3.NumOfInputs = 4.0;
  expl_temp_3.NumOfOutputs = 3.0;
  expl_temp_3.NumOfStates = 7.0;
  expl_temp_3.PredictionHorizon = 10.0;
  for (idx = 0; idx < 20; idx++) {
    expl_temp_3.MVTarget[idx] = 0.0;
  }

  std::memcpy(&expl_temp_3.References[0], &expl_temp_2.ref[0], 30U * sizeof
              (real_T));
  expl_temp_3.LastMV[0] = AEBController_DW.Delay_DSTATE[0];
  expl_temp_3.LastMV[1] = AEBController_DW.Delay_DSTATE[1];
  expl_temp_3.Ts = 0.05;
  for (i = 0; i < 7; i++) {
    expl_temp_3.CurrentStates[i] = AEBController_DW.x[i];
    expl_temp_4.Parameters[i] = expl_temp_2.Parameters[i];
  }

  for (idx = 0; idx < 20; idx++) {
    expl_temp_4.MVScaledTarget[idx] = 0.0;
    expl_temp_4.MVRateMax[idx] = (rtInf);
    expl_temp_4.MVRateMin[idx] = (rtMinusInf);
    expl_temp_4.MVMax[idx] = 0.5;
    expl_temp_4.MVMin[idx] = -0.5;
  }

  for (idx = 0; idx < 70; idx++) {
    expl_temp_4.StateMax[idx] = (rtInf);
    expl_temp_4.StateMin[idx] = (rtMinusInf);
  }

  for (idx = 0; idx < 30; idx++) {
    expl_temp_4.OutputMax[idx] = (rtInf);
    expl_temp_4.OutputMin[idx] = (rtMinusInf);
  }

  expl_temp_4.ECRWeight = 100000.0;
  for (idx = 0; idx < 20; idx++) {
    expl_temp_4.MVRateWeights[idx] = h[idx];
    expl_temp_4.MVWeights[idx] = 0.0;
  }

  std::memcpy(&expl_temp_4.OutputWeights[0], &expl_temp_2.OutputWeights[0], 30U *
              sizeof(real_T));
  std::memcpy(&expl_temp_4.md[0], &expl_temp_2.md[0], 11U * sizeof(real_T));
  std::memcpy(&expl_temp_4.ref[0], &expl_temp_2.ref[0], 30U * sizeof(real_T));
  expl_temp_4.lastMV[0] = AEBController_DW.Delay_DSTATE[0];
  expl_temp_4.lastMV[1] = AEBController_DW.Delay_DSTATE[1];
  for (i = 0; i < 7; i++) {
    expl_temp_4.x[i] = AEBController_DW.x[i];
  }

  for (i = 0; i < 7; i++) {
    for (idx = 0; idx < 9; idx++) {
      y_0[i + 7 * idx] = rtb_Selector[9 * i + idx];
    }
  }

  for (idx = 0; idx < 7; idx++) {
    y_0[idx + 63] = rtb_Selector[9 * idx + 8];
  }

  for (idx = 0; idx < 9; idx++) {
    b_j = idx << 1;
    rtb_Selector1_0[b_j] = rtb_Selector1[idx];
    rtb_Selector1_0[b_j + 1] = rtb_Selector1[idx + 9];
  }

  rtb_Selector1_0[18] = rtb_Selector1[8];
  rtb_Selector1_0[19] = rtb_Selector1[17];
  for (idx = 0; idx < 4; idx++) {
    rtb_FCW_delTVA = 0.0;
    for (b_j = 0; b_j < 20; b_j++) {
      rtb_FCW_delTVA += a_0[(b_j << 2) + idx] * rtb_Selector1_0[b_j];
    }

    a[idx] = rtb_FCW_delTVA;
  }

  rtb_Selector_0[70] = a[0];
  rtb_Selector_0[71] = a[1];
  rtb_Selector_0[72] = a[2];
  rtb_Selector_0[73] = a[3];
  rtb_Selector_0[74] = AEBController_DW.slack_delay_DSTATE;
  for (idx = 0; idx < 70; idx++) {
    rtb_Selector_0[idx] = y_0[idx];
    tmp[idx] = (rtMinusInf);
  }

  tmp[70] = (rtMinusInf);
  tmp[71] = (rtMinusInf);
  tmp[72] = (rtMinusInf);
  tmp[73] = (rtMinusInf);
  tmp[74] = 0.0;
  AEBController_fmincon(&expl_temp_2, &expl_temp_3, rtb_Selector_0,
                        AEBController_DW.A_data, B_data, B_size, tmp,
                        &expl_temp_4, z, &rtb_FCW_delTVA, &ic_idx_0,
                        &rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2,
                        &rtb_TmpSignalConversionAtMATLABSystemInport3_idx_1,
                        expl_temp, &z_idx_0, &z_idx_1, &expl_temp_0,
                        &expl_temp_1);
  if ((ic_idx_0 == 0.0) && (z_idx_0 > 1.0E-6)) {
    ic_idx_0 = -2.0;
  }

  // Update for Delay: '<S16>/slack_delay' incorporates:
  //   DataStoreRead: '<S11>/Data Store Read'
  //   MATLAB Function: '<S15>/NLMPC'

  getXUe_eNtkTHmx(z, AEBController_DW.x, expl_temp_2.md, X, U,
                  &AEBController_DW.slack_delay_DSTATE);

  // MATLAB Function: '<S15>/NLMPC' incorporates:
  //   Delay: '<S3>/Delay'

  if (ic_idx_0 > 0.0) {
    ic_idx_0 = U[0];
    rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 = U[11];
  } else {
    ic_idx_0 = AEBController_DW.Delay_DSTATE[0];
    rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2 =
      AEBController_DW.Delay_DSTATE[1];
  }

  // Switch: '<S2>/Switch3' incorporates:
  //   Delay: '<S2>/Delay3'
  //   Gain: '<S2>/Gain'
  //   Switch: '<S2>/Switch4'

  if (rtb_RelationalOperator1) {
    *rty_Throttle = -AEBController_DW.decel;
    *rty_SteeringAngle = AEBController_DW.Delay3_DSTATE;
  } else {
    *rty_Throttle = ic_idx_0;
    *rty_SteeringAngle = rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2;
  }

  // End of Switch: '<S2>/Switch3'

  // ZeroOrderHold: '<S7>/Zero-Order Hold1' incorporates:
  //   Constant: '<S7>/Constant'
  //   Constant: '<S7>/Noise'
  //   Constant: '<S7>/Sample Time'
  //   Product: '<S7>/Product'

  rtb_ZeroOrderHold1[2] = rtu_CurvatureSequence[0] * rtb_Vxrho;
  rtb_ZeroOrderHold1[0] = ic_idx_0;
  rtb_ZeroOrderHold1[1] = rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2;
  rtb_ZeroOrderHold1[3] = 0.0;
  rtb_ZeroOrderHold1[4] = 0.05;

  // Outputs for Atomic SubSystem: '<S9>/Predict'
  for (i = 0; i < 7; i++) {
    rtb_ZeroOrderHold1[i + 5] = rtCP_Constant_Value[i];

    // Start for MATLABSystem: '<S12>/MATLAB System' incorporates:
    //   Constant: '<S7>/Constant'
    //   DataStoreRead: '<S12>/Data Store ReadX'

    z_0[i] = AEBController_DW.x[i];
  }

  // End of ZeroOrderHold: '<S7>/Zero-Order Hold1'

  // Start for MATLABSystem: '<S12>/MATLAB System'
  AEBController_helperEKFStateFcn(z_0, rtb_ZeroOrderHold1);

  // MATLABSystem: '<S12>/MATLAB System' incorporates:
  //   DataStoreRead: '<S12>/Data Store ReadX'

  for (b_j = 0; b_j < 7; b_j++) {
    for (i = 0; i < 7; i++) {
      imvec[i] = AEBController_DW.x[i];
    }

    // Start for MATLABSystem: '<S12>/MATLAB System' incorporates:
    //   DataStoreRead: '<S12>/Data Store ReadX'

    rtb_FCW_delTVA = 1.4901161193847656E-8 * std::abs(AEBController_DW.x[b_j]);
    if ((rtb_FCW_delTVA <= 1.4901161193847656E-8) || rtIsNaN(rtb_FCW_delTVA)) {
      rtb_FCW_delTVA = 1.4901161193847656E-8;
    }

    imvec[b_j] = AEBController_DW.x[b_j] + rtb_FCW_delTVA;

    // Start for MATLABSystem: '<S12>/MATLAB System' incorporates:
    //   DataStoreRead: '<S12>/Data Store ReadX'

    AEBController_helperEKFStateFcn(imvec, rtb_ZeroOrderHold1);
    for (idx = 0; idx < 7; idx++) {
      A[idx + 7 * b_j] = (imvec[idx] - z_0[idx]) / rtb_FCW_delTVA;
    }
  }

  for (b_j = 0; b_j < 7; b_j++) {
    // Start for MATLABSystem: '<S12>/MATLAB System' incorporates:
    //   Constant: '<S9>/Q'
    //   DataStoreRead: '<S12>/Data Store ReadP'

    for (i = 0; i < 7; i++) {
      aoffset = i * 7 - 1;
      rtb_FCW_delTVA = 0.0;
      for (idx = 0; idx < 7; idx++) {
        rtb_FCW_delTVA += AEBController_DW.P[(aoffset + idx) + 1] * A[idx * 7 +
          b_j];
      }

      idx = 14 * b_j + i;
      y_1[idx] = rtb_FCW_delTVA;
      y_1[idx + 7] = rtCP_Q_Value[7 * i + b_j];
    }
  }

  AEBController_qr_c0(y_1, a__1_1, A);

  // DataStoreWrite: '<S12>/Data Store WriteX' incorporates:
  //   MATLABSystem: '<S12>/MATLAB System'
  //
  AEBController_helperEKFStateFcn(AEBController_DW.x, rtb_ZeroOrderHold1);

  // DataStoreWrite: '<S12>/Data Store WriteP' incorporates:
  //   MATLABSystem: '<S12>/MATLAB System'
  //
  for (idx = 0; idx < 7; idx++) {
    for (b_j = 0; b_j < 7; b_j++) {
      AEBController_DW.P[b_j + 7 * idx] = A[7 * b_j + idx];
    }
  }

  // End of DataStoreWrite: '<S12>/Data Store WriteP'
  // End of Outputs for SubSystem: '<S9>/Predict'

  // Update for Delay: '<S1>/Delay'
  AEBController_DW.Delay_DSTATE_c = *rty_AEBStatus;

  // Update for Delay: '<S2>/Delay2'
  AEBController_DW.Delay2_DSTATE = rtb_RelationalOperator1;

  // Update for Delay: '<S2>/Delay3'
  AEBController_DW.Delay3_DSTATE = *rty_SteeringAngle;

  // Update for Delay: '<S3>/Delay'
  AEBController_DW.Delay_DSTATE[0] = ic_idx_0;
  AEBController_DW.Delay_DSTATE[1] =
    rtb_TmpSignalConversionAtMATLABSystemInport3_idx_2;

  // Update for Delay: '<S16>/mv_Delay' incorporates:
  //   MATLAB Function: '<S15>/NLMPC'

  AEBController_DW.icLoad = false;
  std::memcpy(&AEBController_DW.mv_Delay_DSTATE[0], &U[0], 22U * sizeof(real_T));

  // Update for Delay: '<S16>/x_Delay' incorporates:
  //   MATLAB Function: '<S15>/NLMPC'

  AEBController_DW.icLoad_f = false;
  std::memcpy(&AEBController_DW.x_Delay_DSTATE[0], &X[0], 77U * sizeof(real_T));

  // Update for Delay: '<S16>/slack_delay'
  AEBController_DW.icLoad_k = false;
}

// Model initialize function
void ACCWithSensorFusionModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // states (dwork)
  {
    AEBController_DW.driver_set_velocity[0] = 10.0;
  }

  // Initialize DataMapInfo substructure containing ModelMap for C API
  AEBController_InitializeDataMapInfo((&AEBController_M), &AEBController_DW);
}

// Constructor
ACCWithSensorFusionModelClass::ACCWithSensorFusionModelClass() :
  AEBController_DW(),
  AEBController_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
ACCWithSensorFusionModelClass::~ACCWithSensorFusionModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_AEBController_T * ACCWithSensorFusionModelClass::getRTM()
{
  return (&AEBController_M);
}

// member function to set up the C-API information
void ACCWithSensorFusionModelClass::setupCAPIInfo(rtwCAPI_ModelMappingInfo
  *rt_ParentMMI, const char_T *rt_ChildPath, int_T rt_ChildMMIIdx, int_T
  rt_CSTATEIdx)
{
  // Initialize Parent model MMI
  if ((rt_ParentMMI != (NULL)) && (rt_ChildPath != (NULL))) {
    rtwCAPI_SetChildMMI(*rt_ParentMMI, rt_ChildMMIIdx, &((&AEBController_M)
      ->DataMapInfo.mmi));
    rtwCAPI_SetPath((&AEBController_M)->DataMapInfo.mmi, rt_ChildPath);
    rtwCAPI_MMISetContStateStartIndex((&AEBController_M)->DataMapInfo.mmi,
      rt_CSTATEIdx);
  }
}

RT_MODEL_AEBController_T::DataMapInfo_T RT_MODEL_AEBController_T::getDataMapInfo
  () const
{
  return DataMapInfo;
}

void RT_MODEL_AEBController_T::setDataMapInfo(RT_MODEL_AEBController_T::
  DataMapInfo_T aDataMapInfo)
{
  DataMapInfo = aDataMapInfo;
}

//
// File trailer for generated code.
//
// [EOF]
//
