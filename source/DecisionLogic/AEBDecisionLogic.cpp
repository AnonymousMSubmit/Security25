#include "AEBDecisionLogic.h"
#include "AEBDecisionLogic_types.h"
#include "rtwtypes.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rt_remd_snf.h"
#include <cmath>
#include "rt_hypotd_snf.h"
#include "AEBDecisionLogic_capi.h"
#include "AEBDecisionLogic_private.h"

real_T ACCWithSensorFusionModelClass::AEBDecisionLogic_norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

real_T ACCWithSensorFusionModelClass::AEBDecisionLogic_xnrm2(const real_T x[2])
{
  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  return std::abs(x[1]);
}

void ACCWithSensorFusionModelClass::AEBDecisionLogic_interp1(const real_T
  varargin_1_data[], const int32_T *varargin_1_size, const real_T
  varargin_2_data[], const int32_T *varargin_2_size, const real_T varargin_3[10],
  real_T Vq[10])
{
  real_T x_data[450];
  real_T y_data[450];
  real_T maxx;
  real_T xtmp;
  int32_T low_ip1;
  int32_T mid_i;
  int32_T n;
  int32_T nx;
  if (*varargin_2_size - 1 >= 0) {
    std::memcpy(&y_data[0], &varargin_2_data[0], static_cast<uint32_T>
                (*varargin_2_size) * sizeof(real_T));
  }

  if (*varargin_1_size - 1 >= 0) {
    std::memcpy(&x_data[0], &varargin_1_data[0], static_cast<uint32_T>
                (*varargin_1_size) * sizeof(real_T));
  }

  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  if (varargin_1_data[1] < varargin_1_data[0]) {
    nx = *varargin_1_size >> 1;
    for (n = 0; n < nx; n++) {
      xtmp = x_data[n];
      mid_i = (*varargin_1_size - n) - 1;
      x_data[n] = x_data[mid_i];
      x_data[mid_i] = xtmp;
    }

    if ((*varargin_2_size != 0) && (*varargin_2_size > 1)) {
      low_ip1 = *varargin_2_size >> 1;
      for (nx = 0; nx < low_ip1; nx++) {
        xtmp = y_data[nx];
        n = (*varargin_2_size - nx) - 1;
        y_data[nx] = y_data[n];
        y_data[n] = xtmp;
      }
    }
  }

  xtmp = x_data[0];
  maxx = x_data[*varargin_1_size - 1];
  for (nx = 0; nx < 10; nx++) {
    real_T r;

    // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
    Vq[nx] = 0.0;
    r = varargin_3[nx];
    if (rtIsNaN(r)) {
      Vq[nx] = (rtNaN);
    } else if (r > maxx) {
      if ((*varargin_2_size - 1) + 1 > 1) {
        Vq[nx] = (r - maxx) / (maxx - x_data[*varargin_1_size - 2]) * (y_data
          [*varargin_2_size - 1] - y_data[(*varargin_2_size - 1) - 1]) + y_data[*
          varargin_2_size - 1];
      }
    } else if (r < xtmp) {
      Vq[nx] = (r - xtmp) / (x_data[1] - xtmp) * (y_data[1] - y_data[0]) +
        y_data[0];
    } else {
      int32_T high_i;
      n = 1;
      low_ip1 = 1;
      high_i = *varargin_1_size;
      while (high_i > low_ip1 + 1) {
        mid_i = (n >> 1) + (high_i >> 1);
        if (((static_cast<uint32_T>(n) & 1U) == 1U) && ((static_cast<uint32_T>
              (high_i) & 1U) == 1U)) {
          mid_i++;
        }

        if (varargin_3[nx] >= x_data[mid_i - 1]) {
          n = mid_i;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      r = x_data[n - 1];
      r = (varargin_3[nx] - r) / (x_data[n] - r);
      if (r == 0.0) {
        Vq[nx] = y_data[n - 1];
      } else if (r == 1.0) {
        Vq[nx] = y_data[n];
      } else {
        real_T tmp;
        tmp = y_data[n - 1];
        if (tmp == y_data[n]) {
          Vq[nx] = tmp;
        } else {
          Vq[nx] = (1.0 - r) * tmp + r * y_data[n];
        }
      }
    }
  }
}

real_T ACCWithSensorFusionModelClass::AEBDecisionLogic_mod(real_T x)
{
  real_T r;

  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    boolean_T rEQ0;
    r = std::fmod(x, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      real_T q;
      q = std::abs(x / 6.2831853071795862);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = 0.0;
    } else if (r < 0.0) {
      r += 6.2831853071795862;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  return r;
}

// System initialize for referenced model: 'AEBDecisionLogic'
void ACCWithSensorFusionModelClass::init(void)
{
  // Start for MATLABSystem: '<Root>/Ego Reference Path Generator'
  AEBDecisionLogic_DW.objisempty = true;
  AEBDecisionLogic_DW.obj.isInitialized = 1;

  // InitializeConditions for MATLABSystem: '<Root>/Ego Reference Path Generator' 
  //  Initialize / reset discrete-state properties
  AEBDecisionLogic_DW.obj.CurrentIndex = 1.0;
}

// Output and update for referenced model: 'AEBDecisionLogic'
void ACCWithSensorFusionModelClass::step(const BusMultiObjectTracker1
  *rtu_Tracks, const real_T rtu_Ego_Position[3], const real_T rtu_Ego_Velocity[3],
  const real_T *rtu_Ego_Yaw, const real_T rtu_ReferencePath_time[450], const
  real_T rtu_ReferencePath_x[450], const real_T rtu_ReferencePath_y[450], const
  real_T rtu_ReferencePath_theta[450], const real_T rtu_ReferencePath_kappa[450],
  const real_T rtu_ReferencePath_speed[450], const real_T
  rtu_ReferencePath_arcLength[450], const real_T *rtu_ReferencePath_numPoints,
  real_T *rty_RelativeDistance, real_T *rty_RelativeVelocity, real_T
  rty_CurvatureSequence[10], real_T *rty_LateralDeviation, real_T
  *rty_RelativeYawAngle, boolean_T *rty_GoalReached, real_T *rty_MIOTrack)
{
  real_T tmp_data[450];
  real_T tmp_data_0[450];
  real_T ArcLengthPrediction[10];
  real_T rtb_curvature_sequence_p_0[10];
  real_T mioState[6];
  real_T b[4];
  real_T rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_0[3];
  real_T DeltaSq[2];
  real_T position[2];
  real_T ArcLengthCurr;
  real_T DeltaSq_0;
  real_T DeltaXY_idx_1_tmp;
  real_T DeltaXY_idx_1_tmp_0;
  real_T DeltaXY_idx_1_tmp_1;
  real_T RXY_idx_0;
  real_T absx;
  real_T absx_0;
  real_T maxX;
  real_T rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1;
  real_T rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
  real_T yleftLane;
  real_T yrightLane;
  int32_T i;
  int32_T knt;
  int32_T r1;
  int32_T rankA;
  uint32_T trackID;
  boolean_T exitg1;

  // MATLAB Function: '<Root>/Find Lead Car' incorporates:
  //   Constant: '<Root>/Position Selector'

  maxX = 1000.0;
  trackID = 0U;
  knt = static_cast<int32_T>(rtu_Tracks->NumTracks);
  for (r1 = 0; r1 < knt; r1++) {
    for (rankA = 0; rankA < 2; rankA++) {
      yleftLane = 0.0;
      for (i = 0; i < 6; i++) {
        yleftLane += rtCP_PositionSelector_Value[(i << 1) + rankA] *
          rtu_Tracks->Tracks[r1].State[i];
      }

      position[rankA] = yleftLane;
    }

    if ((position[0] < maxX) && (position[0] > 0.0) && (position[1] >= -1.8) &&
        (position[1] <= 1.8)) {
      maxX = position[0];
      trackID = static_cast<uint32_T>(r1) + 1U;
    }
  }

  if (trackID > 0U) {
    for (rankA = 0; rankA < 6; rankA++) {
      mioState[rankA] = rtu_Tracks->Tracks[static_cast<int32_T>(trackID) - 1].
        State[rankA];
    }
  } else {
    mioState[0] = 500.0;
    mioState[1] = 0.01;
  }

  AEBDecisionLogic_DW.x = mioState[0];
  *rty_RelativeVelocity = mioState[1];
  *rty_MIOTrack = trackID;

  // End of MATLAB Function: '<Root>/Find Lead Car'

  // SignalConversion generated from: '<Root>/Relative Distance'
  *rty_RelativeDistance = AEBDecisionLogic_DW.x;

  // BusCreator generated from: '<Root>/Ego Reference Path Generator'
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1 =
    rtu_Ego_Position[0];
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_0[0] =
    rtu_Ego_Velocity[0];
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2 =
    rtu_Ego_Position[1];
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_0[1] =
    rtu_Ego_Velocity[1];
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_0[2] =
    rtu_Ego_Velocity[2];
  maxX = *rtu_Ego_Yaw;

  // MATLABSystem: '<Root>/Ego Reference Path Generator' incorporates:
  //   BusCreator generated from: '<Root>/Ego Reference Path Generator'

  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.time[0],
              &rtu_ReferencePath_time[0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.x[0], &rtu_ReferencePath_x
              [0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.y[0], &rtu_ReferencePath_y
              [0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.theta[0],
              &rtu_ReferencePath_theta[0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.kappa[0],
              &rtu_ReferencePath_kappa[0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.speed[0],
              &rtu_ReferencePath_speed[0], 450U * sizeof(real_T));
  std::memcpy(&AEBDecisionLogic_DW.obj.ReferencePath.arcLength[0],
              &rtu_ReferencePath_arcLength[0], 450U * sizeof(real_T));

  // BusCreator generated from: '<Root>/Ego Reference Path Generator'
  yleftLane = *rtu_ReferencePath_numPoints;

  // MATLABSystem: '<Root>/Ego Reference Path Generator'
  //  Calulate reference pose information between current section of path.
  //  Initializing outputs with zeros.
  std::memset(&rtb_curvature_sequence_p_0[0], 0, 10U * sizeof(real_T));

  // SignalConversion generated from: '<Root>/Ego Reference Path Generator' incorporates:
  //   MATLABSystem: '<Root>/Ego Reference Path Generator'

  AEBDecisionLogic_DW.lateral_deviation = 0.0;

  // MATLABSystem: '<Root>/Ego Reference Path Generator' incorporates:
  //   BusCreator generated from: '<Root>/Ego Reference Path Generator'
  //
  ArcLengthCurr = 0.0;
  AEBDecisionLogic_DW.obj.ReferencePath.numPoints = yleftLane;
  AEBDecisionLogic_DW.obj.NumWaypoints = yleftLane;
  if (rtIsInf(maxX) || rtIsNaN(maxX)) {
    yleftLane = (rtNaN);
    yrightLane = (rtNaN);
  } else {
    yrightLane = rt_remd_snf(maxX, 360.0);
    yleftLane = yrightLane;
    absx_0 = std::abs(yrightLane);
    absx = absx_0;
    if (absx_0 > 180.0) {
      if (yrightLane > 0.0) {
        yleftLane = yrightLane - 360.0;
        yrightLane -= 360.0;
      } else {
        yleftLane = yrightLane + 360.0;
        yrightLane += 360.0;
      }

      absx = std::abs(yleftLane);
      absx_0 = std::abs(yrightLane);
    }

    if (absx <= 45.0) {
      yleftLane = std::cos(0.017453292519943295 * yleftLane);
    } else if (absx <= 135.0) {
      if (yleftLane > 0.0) {
        yleftLane = -std::sin((yleftLane - 90.0) * 0.017453292519943295);
      } else {
        yleftLane = std::sin((yleftLane + 90.0) * 0.017453292519943295);
      }
    } else {
      if (yleftLane > 0.0) {
        yleftLane = (yleftLane - 180.0) * 0.017453292519943295;
      } else {
        yleftLane = (yleftLane + 180.0) * 0.017453292519943295;
      }

      yleftLane = -std::cos(yleftLane);
    }

    if (absx_0 <= 45.0) {
      yrightLane = std::sin(0.017453292519943295 * yrightLane);
    } else if (absx_0 <= 135.0) {
      if (yrightLane > 0.0) {
        yrightLane = std::cos((yrightLane - 90.0) * 0.017453292519943295);
      } else {
        yrightLane = -std::cos((yrightLane + 90.0) * 0.017453292519943295);
      }
    } else {
      if (yrightLane > 0.0) {
        yrightLane = (yrightLane - 180.0) * 0.017453292519943295;
      } else {
        yrightLane = (yrightLane + 180.0) * 0.017453292519943295;
      }

      yrightLane = -std::sin(yrightLane);
    }
  }

  position[0] = 1.305 * yleftLane +
    rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1;
  position[1] = 1.305 * yrightLane +
    rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;

  //  calculate distance between the current ego position and referencePath position with next index 
  DeltaSq[0] = AEBDecisionLogic_DW.obj.ReferencePath.x[static_cast<int32_T>
    (AEBDecisionLogic_DW.obj.CurrentIndex + 1.0) - 1] - position[0];
  DeltaSq[1] = AEBDecisionLogic_DW.obj.ReferencePath.y[static_cast<int32_T>
    (AEBDecisionLogic_DW.obj.CurrentIndex + 1.0) - 1] - position[1];
  yrightLane = AEBDecisionLogic_norm(DeltaSq);
  yleftLane = AEBDecisionLogic_DW.obj.CurrentIndex;
  rankA = static_cast<int32_T>((1.0 - (AEBDecisionLogic_DW.obj.CurrentIndex +
    2.0)) + AEBDecisionLogic_DW.obj.NumWaypoints) - 1;
  r1 = 0;
  exitg1 = false;
  while ((!exitg1) && (r1 <= rankA)) {
    yleftLane = (AEBDecisionLogic_DW.obj.CurrentIndex + 2.0) +
      static_cast<real_T>(r1);
    DeltaSq[0] = AEBDecisionLogic_DW.obj.ReferencePath.x[static_cast<int32_T>
      (yleftLane) - 1] - position[0];
    DeltaSq[1] = AEBDecisionLogic_DW.obj.ReferencePath.y[static_cast<int32_T>
      (yleftLane) - 1] - position[1];
    absx = AEBDecisionLogic_norm(DeltaSq);
    if (absx > yrightLane) {
      //  when the distance starts to increase
      yleftLane--;

      //  this point is the index for the current segment
      exitg1 = true;
    } else {
      yrightLane = absx;
      r1++;
    }
  }

  AEBDecisionLogic_DW.obj.CurrentIndex = yleftLane - 1.0;

  //  Distance between section start and end points
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1 =
    AEBDecisionLogic_DW.obj.ReferencePath.x[static_cast<int32_T>(yleftLane - 1.0)
    - 1];
  rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2 =
    AEBDecisionLogic_DW.obj.ReferencePath.x[static_cast<int32_T>(yleftLane) - 1]
    - rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1;
  DeltaXY_idx_1_tmp = AEBDecisionLogic_DW.obj.ReferencePath.y
    [static_cast<int32_T>(yleftLane - 1.0) - 1];
  DeltaXY_idx_1_tmp_0 = AEBDecisionLogic_DW.obj.ReferencePath.y
    [static_cast<int32_T>(yleftLane) - 1];
  DeltaXY_idx_1_tmp_1 = DeltaXY_idx_1_tmp_0 - DeltaXY_idx_1_tmp;

  //  Distance between section starting point and current position
  //  Normalized distance between section starting point and current position
  DeltaSq[0] = rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2 *
    rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
  DeltaSq[1] = DeltaXY_idx_1_tmp_1 * DeltaXY_idx_1_tmp_1;
  if (AEBDecisionLogic_norm(DeltaSq) < 0.1) {
    absx = 1.0;
  } else {
    RXY_idx_0 = (position[0] -
                 rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1)
      * rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
    yrightLane = 0.0;
    for (r1 = 0; r1 < 1; r1++) {
      absx = DeltaSq[0];
      yrightLane = 0.0;
      absx_0 = AEBDecisionLogic_xnrm2(DeltaSq);
      if (absx_0 != 0.0) {
        absx_0 = rt_hypotd_snf(DeltaSq[0], absx_0);
        if (DeltaSq[0] >= 0.0) {
          absx_0 = -absx_0;
        }

        if (std::abs(absx_0) < 1.0020841800044864E-292) {
          knt = -1;
          do {
            knt++;
            DeltaSq_0 = 9.9792015476736E+291 * DeltaSq[1];
            DeltaSq[1] *= 9.9792015476736E+291;
            absx_0 *= 9.9792015476736E+291;
            absx *= 9.9792015476736E+291;
          } while ((std::abs(absx_0) < 1.0020841800044864E-292) && (knt + 1 < 20));

          absx_0 = rt_hypotd_snf(absx, AEBDecisionLogic_xnrm2(DeltaSq));
          if (absx >= 0.0) {
            absx_0 = -absx_0;
          }

          yrightLane = (absx_0 - absx) / absx_0;
          DeltaSq[1] = 1.0 / (absx - absx_0) * DeltaSq_0;
          for (rankA = 0; rankA <= knt; rankA++) {
            absx_0 *= 1.0020841800044864E-292;
          }

          absx = absx_0;
        } else {
          yrightLane = (absx_0 - DeltaSq[0]) / absx_0;
          DeltaSq[1] *= 1.0 / (DeltaSq[0] - absx_0);
          absx = absx_0;
        }
      }

      DeltaSq[0] = absx;
    }

    rankA = 0;
    absx = std::abs(DeltaSq[0]);
    if (!(absx <= 4.4408920985006262E-15 * absx)) {
      rankA = 1;
    }

    absx = 0.0;
    if (yrightLane != 0.0) {
      absx_0 = ((position[1] - DeltaXY_idx_1_tmp) * DeltaXY_idx_1_tmp_1 *
                DeltaSq[1] + RXY_idx_0) * yrightLane;
      if (absx_0 != 0.0) {
        RXY_idx_0 -= absx_0;
      }
    }

    for (r1 = 0; r1 < rankA; r1++) {
      absx = RXY_idx_0;
    }

    for (r1 = rankA; r1 >= 1; r1--) {
      absx /= DeltaSq[0];
    }
  }

  if (yleftLane < AEBDecisionLogic_DW.obj.NumWaypoints - 2.0) {
    // SignalConversion generated from: '<Root>/Ego Reference Path Generator'
    *rty_GoalReached = false;

    //  Target States at start point of current section
    //  Target States at end point of current section
    //  Target States on the current section corresponding to current pose
    //  Calculate curavture sequence based on current arc length and
    //  velocity
    ArcLengthCurr = AEBDecisionLogic_DW.obj.ReferencePath.arcLength[static_cast<
      int32_T>(yleftLane - 1.0) - 1];
    ArcLengthCurr += (AEBDecisionLogic_DW.obj.ReferencePath.arcLength[
                      static_cast<int32_T>(yleftLane) - 1] - ArcLengthCurr) *
      absx;

    //  calculate the curvature sequence based on the current arc
    //  length and current ego velocity.
    yrightLane = AEBDecisionLogic_norm
      (&rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_0[0]);
    std::memset(&ArcLengthPrediction[0], 0, 10U * sizeof(real_T));
    ArcLengthPrediction[0] = ArcLengthCurr;
    for (r1 = 0; r1 < 9; r1++) {
      ArcLengthPrediction[r1 + 1] = ((static_cast<real_T>(r1) + 2.0) - 1.0) *
        0.1 * yrightLane + ArcLengthCurr;
    }

    if (AEBDecisionLogic_DW.obj.NumWaypoints < 1.0) {
      r1 = 0;
    } else {
      r1 = static_cast<int32_T>(AEBDecisionLogic_DW.obj.NumWaypoints);
    }

    rankA = r1;
    if (r1 - 1 >= 0) {
      std::memcpy(&tmp_data[0],
                  &AEBDecisionLogic_DW.obj.ReferencePath.arcLength[0],
                  static_cast<uint32_T>(r1) * sizeof(real_T));
    }

    if (AEBDecisionLogic_DW.obj.NumWaypoints < 1.0) {
      r1 = 0;
    } else {
      r1 = static_cast<int32_T>(AEBDecisionLogic_DW.obj.NumWaypoints);
    }

    if (r1 - 1 >= 0) {
      std::memcpy(&tmp_data_0[0], &AEBDecisionLogic_DW.obj.ReferencePath.kappa[0],
                  static_cast<uint32_T>(r1) * sizeof(real_T));
    }

    AEBDecisionLogic_interp1(tmp_data, &rankA, tmp_data_0, &r1,
      ArcLengthPrediction, rtb_curvature_sequence_p_0);

    //  Distance between current position and target position
    //  Calculate the distance between the current position from the
    //  reference path
    DeltaSq[0] = rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2
      * -position[0] - DeltaXY_idx_1_tmp_1 * position[1];
    DeltaSq[1] = rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2
      * -DeltaXY_idx_1_tmp + DeltaXY_idx_1_tmp_1 *
      rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1;
    b[0] = rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
    b[2] = DeltaXY_idx_1_tmp_1;
    b[1] = DeltaXY_idx_1_tmp - DeltaXY_idx_1_tmp_0;
    b[3] = rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
    if (std::abs(b[1]) > std::abs
        (rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2)) {
      r1 = 1;
      rankA = 0;
    } else {
      r1 = 0;
      rankA = 1;
    }

    ArcLengthCurr = b[rankA] / b[r1];
    yrightLane = b[r1 + 2];
    ArcLengthCurr = (DeltaSq[rankA] - DeltaSq[r1] * ArcLengthCurr) / (b[rankA +
      2] - yrightLane * ArcLengthCurr);
    DeltaSq[0] = -((DeltaSq[r1] - yrightLane * ArcLengthCurr) / b[r1]) -
      position[0];
    DeltaSq[1] = -ArcLengthCurr - position[1];
    ArcLengthCurr = (position[0] -
                     (rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2
                      * absx +
                      rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_1))
      * DeltaXY_idx_1_tmp_1 - (position[1] - (DeltaXY_idx_1_tmp_1 * absx +
      DeltaXY_idx_1_tmp)) *
      rtb_BusConversion_InsertedFor_EgoReferencePathGenerator_at_in_2;
    if (rtIsNaN(ArcLengthCurr)) {
      ArcLengthCurr = (rtNaN);
    } else if (ArcLengthCurr < 0.0) {
      ArcLengthCurr = -1.0;
    } else {
      ArcLengthCurr = (ArcLengthCurr > 0.0);
    }

    // SignalConversion generated from: '<Root>/Ego Reference Path Generator'
    AEBDecisionLogic_DW.lateral_deviation = AEBDecisionLogic_norm(DeltaSq) *
      ArcLengthCurr;
    ArcLengthCurr = AEBDecisionLogic_DW.obj.ReferencePath.theta
      [static_cast<int32_T>(yleftLane - 1.0) - 1];
    ArcLengthCurr = AEBDecisionLogic_mod((((AEBDecisionLogic_mod
      ((AEBDecisionLogic_DW.obj.ReferencePath.theta[static_cast<int32_T>
        (yleftLane) - 1] - ArcLengthCurr) + 3.1415926535897931) -
      3.1415926535897931) * absx + ArcLengthCurr) - 0.017453292519943295 * maxX)
      + 3.1415926535897931) - 3.1415926535897931;
  } else {
    // SignalConversion generated from: '<Root>/Ego Reference Path Generator'
    *rty_GoalReached = true;
  }

  // SignalConversion generated from: '<Root>/Ego Reference Path Generator' incorporates:
  //   MATLABSystem: '<Root>/Ego Reference Path Generator'

  std::memcpy(&rty_CurvatureSequence[0], &rtb_curvature_sequence_p_0[0], 10U *
              sizeof(real_T));

  // SignalConversion generated from: '<Root>/Lateral Deviation'
  *rty_LateralDeviation = AEBDecisionLogic_DW.lateral_deviation;

  // SignalConversion generated from: '<Root>/Ego Reference Path Generator' incorporates:
  //   MATLABSystem: '<Root>/Ego Reference Path Generator'
  //
  *rty_RelativeYawAngle = ArcLengthCurr;
}

// Model initialize function
void ACCWithSensorFusionModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // Initialize DataMapInfo substructure containing ModelMap for C API
  AEBDecisionLogic_InitializeDataMapInfo((&AEBDecisionLogic_M),
    &AEBDecisionLogic_DW);
}

// Constructor
ACCWithSensorFusionModelClass::ACCWithSensorFusionModelClass() :
  AEBDecisionLogic_DW(),
  AEBDecisionLogic_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
ACCWithSensorFusionModelClass::~ACCWithSensorFusionModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_AEBDecisionLogic_T * ACCWithSensorFusionModelClass::getRTM()
{
  return (&AEBDecisionLogic_M);
}

// member function to set up the C-API information
void ACCWithSensorFusionModelClass::setupCAPIInfo(rtwCAPI_ModelMappingInfo
  *rt_ParentMMI, const char_T *rt_ChildPath, int_T rt_ChildMMIIdx, int_T
  rt_CSTATEIdx)
{
  // Initialize Parent model MMI
  if ((rt_ParentMMI != (NULL)) && (rt_ChildPath != (NULL))) {
    rtwCAPI_SetChildMMI(*rt_ParentMMI, rt_ChildMMIIdx, &((&AEBDecisionLogic_M)
      ->DataMapInfo.mmi));
    rtwCAPI_SetPath((&AEBDecisionLogic_M)->DataMapInfo.mmi, rt_ChildPath);
    rtwCAPI_MMISetContStateStartIndex((&AEBDecisionLogic_M)->DataMapInfo.mmi,
      rt_CSTATEIdx);
  }
}

RT_MODEL_AEBDecisionLogic_T::DataMapInfo_T RT_MODEL_AEBDecisionLogic_T::
  getDataMapInfo() const
{
  return DataMapInfo;
}

void RT_MODEL_AEBDecisionLogic_T::setDataMapInfo(RT_MODEL_AEBDecisionLogic_T::
  DataMapInfo_T aDataMapInfo)
{
  DataMapInfo = aDataMapInfo;
}

//
// File trailer for generated code.
//
// [EOF]
//
