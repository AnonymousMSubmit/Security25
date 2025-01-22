#include "AEBSensorFusion.h"
#include "rtwtypes.h"
#include "AEBSensorFusion_types.h"
#include <cmath>
#include <cstring>
#include "rt_roundd_snf.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rt_remd_snf.h"
#include "rt_hypotd_snf.h"
#include "coder_bounded_array.h"
#include "rt_atan2d_snf.h"
#include "rt_powd_snf.h"
#include "AEBSensorFusion_capi.h"
#include "div_nde_s32_floor.h"

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_DetectionConcatenation_setupImpl
  (matlabshared_tracking_internal_DetectionConcatenation_AEBSens_T *obj, const
   BusVisionDetections varargin_1_Detections[20])
{
  BusDetectionConcatenation1Detections det0;

  // Start for MATLABSystem: '<Root>/Detection Concatenation'
  det0.MeasurementParameters.Frame = varargin_1_Detections[0].
    MeasurementParameters.Frame;
  for (int32_T b_n = 0; b_n < 70; b_n++) {
    // Start for MATLABSystem: '<Root>/Detection Concatenation'
    obj->pOutTemp.Detections[b_n].Time = 0.0;
    for (int32_T i = 0; i < 6; i++) {
      // Start for MATLABSystem: '<Root>/Detection Concatenation'
      obj->pOutTemp.Detections[b_n].Measurement[i] = 0.0;
    }

    // Start for MATLABSystem: '<Root>/Detection Concatenation'
    std::memset(&obj->pOutTemp.Detections[b_n].MeasurementNoise[0], 0, 36U *
                sizeof(real_T));
    obj->pOutTemp.Detections[b_n].SensorIndex = 0.0;
    obj->pOutTemp.Detections[b_n].ObjectClassID = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.Frame =
      det0.MeasurementParameters.Frame;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginPosition[0] = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginPosition[1] = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginPosition[2] = 0.0;
    std::memset(&obj->pOutTemp.Detections[b_n]
                .MeasurementParameters.Orientation[0], 0, 9U * sizeof(real_T));
    obj->pOutTemp.Detections[b_n].MeasurementParameters.HasVelocity = false;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginVelocity[0] = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginVelocity[1] = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.OriginVelocity[2] = 0.0;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.IsParentToChild = false;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.HasAzimuth = false;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.HasElevation = false;
    obj->pOutTemp.Detections[b_n].MeasurementParameters.HasRange = false;
    obj->pOutTemp.Detections[b_n].ObjectAttributes.TargetIndex = 0.0;
    obj->pOutTemp.Detections[b_n].ObjectAttributes.SNR = 0.0;
  }

  obj->pOutTemp.NumDetections = 0.0;
  obj->pOutTemp.IsValidTime = false;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_char
  (drivingCoordinateFrameType varargin_1, char_T y_data[], int32_T y_size[2])
{
  h_cell_wrap_AEBSensorFusion_T b_0[3];
  h_cell_wrap_AEBSensorFusion_T b_1[3];
  h_cell_wrap_AEBSensorFusion_T b;
  h_cell_wrap_AEBSensorFusion_T c;
  h_cell_wrap_AEBSensorFusion_T d;
  int32_T b_k;
  int32_T enumIdx;
  int32_T loop_ub;
  static const char_T tmp[7] = { 'I', 'n', 'v', 'a', 'l', 'i', 'd' };

  static const char_T tmp_0[11] = { 'R', 'e', 'c', 't', 'a', 'n', 'g', 'u', 'l',
    'a', 'r' };

  static const char_T tmp_1[9] = { 'S', 'p', 'h', 'e', 'r', 'i', 'c', 'a', 'l' };

  static const drivingCoordinateFrameType enumVals[3] = { 0U, 1U, 2U };

  boolean_T exitg1;
  b.f1.size[0] = 1;
  b.f1.size[1] = 7;
  for (b_k = 0; b_k < 7; b_k++) {
    b.f1.data[b_k] = tmp[b_k];
  }

  c.f1.size[0] = 1;
  c.f1.size[1] = 11;
  for (b_k = 0; b_k < 11; b_k++) {
    c.f1.data[b_k] = tmp_0[b_k];
  }

  d.f1.size[0] = 1;
  d.f1.size[1] = 9;
  for (b_k = 0; b_k < 9; b_k++) {
    d.f1.data[b_k] = tmp_1[b_k];
  }

  enumIdx = -1;
  b_k = 1;
  exitg1 = false;
  while ((!exitg1) && (b_k - 1 < 3)) {
    if (enumVals[b_k - 1] == varargin_1) {
      enumIdx = b_k - 1;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  y_size[0] = 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_0[0] = b;
  b_0[1] = c;
  b_0[2] = d;
  y_size[1] = b_0[enumIdx].f1.size[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_0[0] = b;
  b_0[1] = c;
  b_0[2] = d;
  b_1[0] = b;
  b_1[1] = c;
  b_1[2] = d;
  loop_ub = b_1[enumIdx].f1.size[1];
  for (b_k = 0; b_k < loop_ub; b_k++) {
    y_data[b_k] = b_0[enumIdx].f1.data[b_k];
  }
}

boolean_T ACCWithSensorFusionModelClass::AEBSensorFusion_strcmp(const char_T
  a_data[], const int32_T a_size[2])
{
  boolean_T b_bool;
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  static const char_T tmp_0[11] = { 'r', 'e', 'c', 't', 'a', 'n', 'g', 'u', 'l',
    'a', 'r' };

  b_bool = false;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (a_size[1] == 11) {
    int32_T b_kstr;
    b_kstr = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 11) {
        if (tmp[static_cast<int32_T>(a_data[b_kstr - 1])] != tmp
            [static_cast<int32_T>(tmp_0[b_kstr - 1])]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return b_bool;
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  absxk = std::abs(x[2]);
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

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_cosd(real_T x)
{
  real_T absx;
  real_T b_x;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (rtIsInf(x) || rtIsNaN(x)) {
    b_x = (rtNaN);
  } else {
    b_x = rt_remd_snf(x, 360.0);
    absx = std::abs(b_x);
    if (absx > 180.0) {
      if (b_x > 0.0) {
        b_x -= 360.0;
      } else {
        b_x += 360.0;
      }

      absx = std::abs(b_x);
    }

    if (absx <= 45.0) {
      b_x *= 0.017453292519943295;
      b_x = std::cos(b_x);
    } else if (absx <= 135.0) {
      if (b_x > 0.0) {
        b_x = (b_x - 90.0) * 0.017453292519943295;
        b_x = -std::sin(b_x);
      } else {
        b_x = (b_x + 90.0) * 0.017453292519943295;
        b_x = std::sin(b_x);
      }
    } else {
      if (b_x > 0.0) {
        b_x = (b_x - 180.0) * 0.017453292519943295;
      } else {
        b_x = (b_x + 180.0) * 0.017453292519943295;
      }

      b_x = -std::cos(b_x);
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return b_x;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_parseDetectionForInitFcn(
  const real_T detection_Measurement[6], const real_T
  detection_MeasurementNoise[36], drivingCoordinateFrameType
  detection_MeasurementParameters_Frame, const real_T
  detection_MeasurementParameters_OriginPosition[3], const real_T
  detection_MeasurementParameters_Orientation[9], boolean_T
  detection_MeasurementParameters_HasVelocity, const real_T
  detection_MeasurementParameters_OriginVelocity[3], boolean_T
  detection_MeasurementParameters_IsParentToChild, boolean_T
  detection_MeasurementParameters_HasElevation, real_T posMeas[3], real_T
  velMeas[3], real_T posCov[9], real_T velCov[9], boolean_T *invalidDet)
{
  real_T lclCov_data[49];
  real_T Rpos[9];
  real_T b_argsinCell_f4_tmp[9];
  real_T rot[9];
  real_T rot_0[9];
  real_T thisOrient[9];
  real_T velLclCov[9];
  real_T originPosition[3];
  real_T originVelocity[3];
  real_T lclCoord_idx_0_tmp;
  real_T lclCoord_idx_1_tmp;
  real_T lclMeas_idx_0;
  real_T lclMeas_idx_1;
  real_T lclMeas_idx_2;
  real_T lclRect_idx_0;
  real_T lclRect_idx_0_tmp;
  real_T rcoselev;
  real_T rcoselev_tmp;
  int32_T b_argsinCell_f4_tmp_tmp;
  int32_T b_argsinCell_f4_tmp_tmp_0;
  int32_T b_kstr;
  int32_T velLclCov_0;
  int32_T velLclCov_tmp;
  char_T a_tmp_data[11];
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  static const char_T tmp_0[7] = { 'I', 'n', 'v', 'a', 'l', 'i', 'd' };

  static const int8_T tmp_1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_2[9] = { 100, 0, 0, 0, 100, 0, 0, 0, 100 };

  int32_T a_tmp_size[2];
  int32_T exitg1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_char(detection_MeasurementParameters_Frame, a_tmp_data,
                       a_tmp_size);
  *invalidDet = false;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (a_tmp_size[1] == 7) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 7) {
        if (tmp[static_cast<int32_T>(a_tmp_data[b_kstr - 1])] != tmp[
            static_cast<int32_T>(tmp_0[b_kstr - 1])]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        *invalidDet = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (*invalidDet) {
    posMeas[0] = 0.0;
    velMeas[0] = 0.0;
    posMeas[1] = 0.0;
    velMeas[1] = 0.0;
    posMeas[2] = 0.0;
    velMeas[2] = 0.0;
    std::memset(&posCov[0], 0, 9U * sizeof(real_T));
    posCov[0] = 1.0;
    posCov[4] = 1.0;
    posCov[8] = 1.0;
    std::memset(&velCov[0], 0, 9U * sizeof(real_T));
    velCov[0] = 1.0;
    velCov[4] = 1.0;
    velCov[8] = 1.0;
  } else {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    std::memcpy(&thisOrient[0], &detection_MeasurementParameters_Orientation[0],
                9U * sizeof(real_T));
    if (detection_MeasurementParameters_IsParentToChild) {
      for (b_kstr = 0; b_kstr < 3; b_kstr++) {
        thisOrient[3 * b_kstr] =
          detection_MeasurementParameters_Orientation[b_kstr];
        thisOrient[3 * b_kstr + 1] =
          detection_MeasurementParameters_Orientation[b_kstr + 3];
        thisOrient[3 * b_kstr + 2] =
          detection_MeasurementParameters_Orientation[b_kstr + 6];
      }
    }

    for (b_kstr = 0; b_kstr < 3; b_kstr++) {
      lclMeas_idx_0 = 0.0;
      lclMeas_idx_1 = 0.0;
      for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
        velLclCov_0 = 3 * velLclCov_tmp + b_kstr;
        b_argsinCell_f4_tmp_tmp = tmp_1[velLclCov_0];
        lclMeas_idx_0 += static_cast<real_T>(b_argsinCell_f4_tmp_tmp) *
          detection_MeasurementParameters_OriginPosition[velLclCov_tmp];
        lclMeas_idx_1 += static_cast<real_T>(b_argsinCell_f4_tmp_tmp) *
          detection_MeasurementParameters_OriginVelocity[velLclCov_tmp];
        b_argsinCell_f4_tmp[velLclCov_0] = (thisOrient[3 * velLclCov_tmp + 1] *
          static_cast<real_T>(tmp_1[b_kstr + 3]) + thisOrient[3 * velLclCov_tmp]
          * static_cast<real_T>(tmp_1[b_kstr])) + thisOrient[3 * velLclCov_tmp +
          2] * static_cast<real_T>(tmp_1[b_kstr + 6]);
      }

      originPosition[b_kstr] = lclMeas_idx_0;
      originVelocity[b_kstr] = lclMeas_idx_1;
    }

    if (AEBSensorFusion_strcmp(a_tmp_data, a_tmp_size)) {
      rcoselev_tmp = detection_Measurement[0];
      lclRect_idx_0_tmp = detection_Measurement[3];
      lclCoord_idx_0_tmp = detection_Measurement[1];
      lclRect_idx_0 = detection_Measurement[4];
      lclCoord_idx_1_tmp = detection_Measurement[2];
      rcoselev = detection_Measurement[5];
      for (b_kstr = 0; b_kstr < 3; b_kstr++) {
        lclMeas_idx_2 = b_argsinCell_f4_tmp[b_kstr];
        lclMeas_idx_0 = lclMeas_idx_2 * rcoselev_tmp;
        lclMeas_idx_1 = lclMeas_idx_2 * lclRect_idx_0_tmp;
        thisOrient[3 * b_kstr] = detection_MeasurementNoise[6 * b_kstr];
        velLclCov_tmp = (b_kstr + 3) * 6;
        velLclCov[3 * b_kstr] = detection_MeasurementNoise[velLclCov_tmp + 3];
        lclMeas_idx_2 = b_argsinCell_f4_tmp[b_kstr + 3];
        lclMeas_idx_0 += lclMeas_idx_2 * lclCoord_idx_0_tmp;
        lclMeas_idx_1 += lclMeas_idx_2 * lclRect_idx_0;
        velLclCov_0 = 3 * b_kstr + 1;
        thisOrient[velLclCov_0] = detection_MeasurementNoise[6 * b_kstr + 1];
        velLclCov[velLclCov_0] = detection_MeasurementNoise[velLclCov_tmp + 4];
        lclMeas_idx_2 = b_argsinCell_f4_tmp[b_kstr + 6];
        velLclCov_0 = 3 * b_kstr + 2;
        thisOrient[velLclCov_0] = detection_MeasurementNoise[6 * b_kstr + 2];
        velLclCov[velLclCov_0] = detection_MeasurementNoise[velLclCov_tmp + 5];
        posMeas[b_kstr] = (lclMeas_idx_2 * lclCoord_idx_1_tmp + lclMeas_idx_0) +
          originPosition[b_kstr];
        velMeas[b_kstr] = (lclMeas_idx_2 * rcoselev + lclMeas_idx_1) +
          originVelocity[b_kstr];
      }
    } else {
      if (detection_MeasurementParameters_HasElevation) {
        lclMeas_idx_0 = detection_Measurement[0];
        lclMeas_idx_1 = detection_Measurement[1];
        lclMeas_idx_2 = detection_Measurement[2];
        velLclCov_tmp = 6;
        std::memcpy(&lclCov_data[0], &detection_MeasurementNoise[0], 36U *
                    sizeof(real_T));
      } else {
        lclMeas_idx_0 = detection_Measurement[0];
        lclMeas_idx_1 = 0.0;
        lclMeas_idx_2 = detection_Measurement[1];
        std::memset(&lclCov_data[0], 0, 49U * sizeof(real_T));
        lclCov_data[0] = detection_MeasurementNoise[0];
        lclCov_data[8] = 2704.0;
        for (b_kstr = 0; b_kstr < 5; b_kstr++) {
          for (velLclCov_tmp = 0; velLclCov_tmp < 5; velLclCov_tmp++) {
            lclCov_data[(velLclCov_tmp + 7 * (b_kstr + 2)) + 2] =
              detection_MeasurementNoise[((b_kstr + 1) * 6 + velLclCov_tmp) + 1];
          }
        }

        velLclCov_tmp = 7;
      }

      lclCoord_idx_0_tmp = 0.017453292519943295 * lclMeas_idx_0;
      lclCoord_idx_1_tmp = 0.017453292519943295 * lclMeas_idx_1;
      if ((lclMeas_idx_2 < 0.0) || rtIsNaN(lclMeas_idx_2)) {
        lclMeas_idx_0 = 0.0;
      } else {
        lclMeas_idx_0 = lclMeas_idx_2;
      }

      rcoselev_tmp = std::cos(lclCoord_idx_1_tmp);
      rcoselev = lclMeas_idx_0 * rcoselev_tmp;
      lclRect_idx_0_tmp = std::cos(lclCoord_idx_0_tmp);
      lclRect_idx_0 = rcoselev * lclRect_idx_0_tmp;
      lclCoord_idx_0_tmp = std::sin(lclCoord_idx_0_tmp);
      rcoselev *= lclCoord_idx_0_tmp;
      lclCoord_idx_1_tmp = std::sin(lclCoord_idx_1_tmp);
      lclMeas_idx_0 *= lclCoord_idx_1_tmp;
      for (b_kstr = 0; b_kstr < 3; b_kstr++) {
        posMeas[b_kstr] = ((b_argsinCell_f4_tmp[b_kstr + 3] * rcoselev +
                            b_argsinCell_f4_tmp[b_kstr] * lclRect_idx_0) +
                           b_argsinCell_f4_tmp[b_kstr + 6] * lclMeas_idx_0) +
          originPosition[b_kstr];
      }

      if (detection_MeasurementParameters_HasVelocity) {
        originPosition[0] = posMeas[0] - originPosition[0];
        originPosition[1] = posMeas[1] - originPosition[1];
        originPosition[2] = posMeas[2] - originPosition[2];
        lclMeas_idx_0 = AEBSensorFusion_norm(originPosition);
        velMeas[0] = originPosition[0] * detection_Measurement[5] /
          lclMeas_idx_0 + originVelocity[0];
        velMeas[1] = originPosition[1] * detection_Measurement[5] /
          lclMeas_idx_0 + originVelocity[1];
        velMeas[2] = originPosition[2] * detection_Measurement[5] /
          lclMeas_idx_0 + originVelocity[2];
        rcoselev = std::sqrt(lclCov_data[(velLclCov_tmp << 1) + 2]);
        if ((lclMeas_idx_2 < rcoselev) || (rtIsNaN(lclMeas_idx_2) && (!rtIsNaN
              (rcoselev)))) {
          lclMeas_idx_2 = rcoselev;
        }

        lclMeas_idx_1 = lclMeas_idx_2 * AEBSensorFusion_cosd(lclMeas_idx_1) *
          (0.017453292519943295 * std::sqrt(lclCov_data[0]));
        lclMeas_idx_0 = std::sqrt(lclCov_data[1 + velLclCov_tmp]) *
          0.017453292519943295 * lclMeas_idx_2;
        std::memset(&Rpos[0], 0, 9U * sizeof(real_T));
        Rpos[0] = rcoselev * rcoselev;
        Rpos[4] = lclMeas_idx_1 * lclMeas_idx_1;
        Rpos[8] = lclMeas_idx_0 * lclMeas_idx_0;
        std::memset(&thisOrient[0], 0, 9U * sizeof(real_T));
        thisOrient[4] = 1.0;
        thisOrient[0] = rcoselev_tmp;
        thisOrient[6] = lclCoord_idx_1_tmp;
        thisOrient[2] = -lclCoord_idx_1_tmp;
        thisOrient[8] = rcoselev_tmp;
        std::memset(&velLclCov[0], 0, 9U * sizeof(real_T));
        velLclCov[8] = 1.0;
        velLclCov[0] = lclRect_idx_0_tmp;
        velLclCov[3] = -lclCoord_idx_0_tmp;
        velLclCov[1] = lclCoord_idx_0_tmp;
        velLclCov[4] = lclRect_idx_0_tmp;
        for (b_kstr = 0; b_kstr < 3; b_kstr++) {
          lclMeas_idx_2 = velLclCov[b_kstr + 3];
          lclMeas_idx_0 = velLclCov[b_kstr];
          velLclCov_0 = static_cast<int32_T>(velLclCov[b_kstr + 6]);
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            rot[b_kstr + 3 * velLclCov_tmp] = (thisOrient[velLclCov_tmp + 3] *
              lclMeas_idx_2 + lclMeas_idx_0 * thisOrient[velLclCov_tmp]) +
              thisOrient[velLclCov_tmp + 6] * static_cast<real_T>(velLclCov_0);
          }
        }

        for (b_kstr = 0; b_kstr < 9; b_kstr++) {
          velLclCov[b_kstr] = tmp_2[b_kstr];
        }

        for (b_kstr = 0; b_kstr < 3; b_kstr++) {
          lclMeas_idx_2 = rot[b_kstr + 3];
          lclMeas_idx_0 = rot[b_kstr];
          lclMeas_idx_1 = rot[b_kstr + 6];
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            rot_0[b_kstr + 3 * velLclCov_tmp] = (Rpos[3 * velLclCov_tmp + 1] *
              lclMeas_idx_2 + Rpos[3 * velLclCov_tmp] * lclMeas_idx_0) + Rpos[3 *
              velLclCov_tmp + 2] * lclMeas_idx_1;
          }

          lclMeas_idx_2 = rot_0[b_kstr + 3];
          lclMeas_idx_0 = rot_0[b_kstr];
          lclMeas_idx_1 = rot_0[b_kstr + 6];
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            thisOrient[b_kstr + 3 * velLclCov_tmp] = (rot[velLclCov_tmp + 3] *
              lclMeas_idx_2 + lclMeas_idx_0 * rot[velLclCov_tmp]) +
              rot[velLclCov_tmp + 6] * lclMeas_idx_1;
          }
        }
      } else {
        velMeas[0] = 0.0;
        velMeas[1] = 0.0;
        velMeas[2] = 0.0;
        rcoselev = std::sqrt(lclCov_data[(velLclCov_tmp << 1) + 2]);
        if ((lclMeas_idx_2 < rcoselev) || (rtIsNaN(lclMeas_idx_2) && (!rtIsNaN
              (rcoselev)))) {
          lclMeas_idx_2 = rcoselev;
        }

        lclMeas_idx_1 = lclMeas_idx_2 * AEBSensorFusion_cosd(lclMeas_idx_1) *
          (0.017453292519943295 * std::sqrt(lclCov_data[0]));
        lclMeas_idx_0 = std::sqrt(lclCov_data[1 + velLclCov_tmp]) *
          0.017453292519943295 * lclMeas_idx_2;
        std::memset(&Rpos[0], 0, 9U * sizeof(real_T));
        Rpos[0] = rcoselev * rcoselev;
        Rpos[4] = lclMeas_idx_1 * lclMeas_idx_1;
        Rpos[8] = lclMeas_idx_0 * lclMeas_idx_0;
        std::memset(&thisOrient[0], 0, 9U * sizeof(real_T));
        thisOrient[4] = 1.0;
        thisOrient[0] = rcoselev_tmp;
        thisOrient[6] = lclCoord_idx_1_tmp;
        thisOrient[2] = -lclCoord_idx_1_tmp;
        thisOrient[8] = rcoselev_tmp;
        std::memset(&velLclCov[0], 0, 9U * sizeof(real_T));
        velLclCov[8] = 1.0;
        velLclCov[0] = lclRect_idx_0_tmp;
        velLclCov[3] = -lclCoord_idx_0_tmp;
        velLclCov[1] = lclCoord_idx_0_tmp;
        velLclCov[4] = lclRect_idx_0_tmp;
        for (b_kstr = 0; b_kstr < 3; b_kstr++) {
          lclMeas_idx_2 = velLclCov[b_kstr + 3];
          lclMeas_idx_0 = velLclCov[b_kstr];
          velLclCov_0 = static_cast<int32_T>(velLclCov[b_kstr + 6]);
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            rot[b_kstr + 3 * velLclCov_tmp] = (thisOrient[velLclCov_tmp + 3] *
              lclMeas_idx_2 + lclMeas_idx_0 * thisOrient[velLclCov_tmp]) +
              thisOrient[velLclCov_tmp + 6] * static_cast<real_T>(velLclCov_0);
          }

          lclMeas_idx_2 = rot[b_kstr + 3];
          lclMeas_idx_0 = rot[b_kstr];
          lclMeas_idx_1 = rot[b_kstr + 6];
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            rot_0[b_kstr + 3 * velLclCov_tmp] = (Rpos[3 * velLclCov_tmp + 1] *
              lclMeas_idx_2 + Rpos[3 * velLclCov_tmp] * lclMeas_idx_0) + Rpos[3 *
              velLclCov_tmp + 2] * lclMeas_idx_1;
          }
        }

        for (b_kstr = 0; b_kstr < 3; b_kstr++) {
          lclMeas_idx_2 = rot_0[b_kstr + 3];
          lclMeas_idx_0 = rot_0[b_kstr];
          lclMeas_idx_1 = rot_0[b_kstr + 6];
          for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
            thisOrient[b_kstr + 3 * velLclCov_tmp] = (rot[velLclCov_tmp + 3] *
              lclMeas_idx_2 + lclMeas_idx_0 * rot[velLclCov_tmp]) +
              rot[velLclCov_tmp + 6] * lclMeas_idx_1;
          }
        }

        for (b_kstr = 0; b_kstr < 9; b_kstr++) {
          velLclCov[b_kstr] = tmp_2[b_kstr];
        }
      }
    }

    for (b_kstr = 0; b_kstr < 3; b_kstr++) {
      for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
        velLclCov_0 = 3 * velLclCov_tmp + b_kstr;
        Rpos[velLclCov_tmp + 3 * b_kstr] = b_argsinCell_f4_tmp[velLclCov_0];
        rot[velLclCov_0] = (thisOrient[3 * velLclCov_tmp + 1] *
                            b_argsinCell_f4_tmp[b_kstr + 3] + thisOrient[3 *
                            velLclCov_tmp] * b_argsinCell_f4_tmp[b_kstr]) +
          thisOrient[3 * velLclCov_tmp + 2] * b_argsinCell_f4_tmp[b_kstr + 6];
      }
    }

    for (b_kstr = 0; b_kstr < 3; b_kstr++) {
      lclMeas_idx_2 = b_argsinCell_f4_tmp[b_kstr + 3];
      lclMeas_idx_0 = b_argsinCell_f4_tmp[b_kstr];
      lclMeas_idx_1 = b_argsinCell_f4_tmp[b_kstr + 6];
      rcoselev_tmp = rot[b_kstr + 3];
      lclRect_idx_0_tmp = rot[b_kstr];
      lclCoord_idx_0_tmp = rot[b_kstr + 6];
      for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
        velLclCov_0 = 3 * velLclCov_tmp + 1;
        b_argsinCell_f4_tmp_tmp = 3 * velLclCov_tmp + 2;
        b_argsinCell_f4_tmp_tmp_0 = 3 * velLclCov_tmp + b_kstr;
        thisOrient[b_argsinCell_f4_tmp_tmp_0] = (velLclCov[3 * velLclCov_tmp] *
          lclMeas_idx_0 + velLclCov[velLclCov_0] * lclMeas_idx_2) +
          velLclCov[b_argsinCell_f4_tmp_tmp] * lclMeas_idx_1;
        posCov[b_argsinCell_f4_tmp_tmp_0] = (Rpos[3 * velLclCov_tmp] *
          lclRect_idx_0_tmp + Rpos[velLclCov_0] * rcoselev_tmp) +
          Rpos[b_argsinCell_f4_tmp_tmp] * lclCoord_idx_0_tmp;
      }

      lclMeas_idx_2 = thisOrient[b_kstr + 3];
      lclMeas_idx_0 = thisOrient[b_kstr];
      lclMeas_idx_1 = thisOrient[b_kstr + 6];
      for (velLclCov_tmp = 0; velLclCov_tmp < 3; velLclCov_tmp++) {
        velCov[b_kstr + 3 * velLclCov_tmp] = (Rpos[3 * velLclCov_tmp + 1] *
          lclMeas_idx_2 + Rpos[3 * velLclCov_tmp] * lclMeas_idx_0) + Rpos[3 *
          velLclCov_tmp + 2] * lclMeas_idx_1;
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlascl(real_T cfrom, real_T
  cto, real_T A[36])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (int32_T i = 0; i < 36; i++) {
      A[i] *= mul;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2(int32_T n, const
  real_T x[36], int32_T ix0)
{
  real_T y;
  y = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return y;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlarfg(int32_T n, real_T
  alpha1, real_T x[36], int32_T ix0, real_T *b_alpha1, real_T *tau)
{
  real_T beta1;
  int32_T b_tmp;
  int32_T k;
  int32_T knt;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  *b_alpha1 = alpha1;
  *tau = 0.0;
  if (n > 0) {
    beta1 = AEBSensorFusion_xnrm2(n - 1, x, ix0);
    if (beta1 != 0.0) {
      beta1 = rt_hypotd_snf(alpha1, beta1);
      if (alpha1 >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          b_tmp = ix0 + n;
          for (k = ix0; k <= b_tmp - 2; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          *b_alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

        beta1 = rt_hypotd_snf(*b_alpha1, AEBSensorFusion_xnrm2(n - 1, x, ix0));
        if (*b_alpha1 >= 0.0) {
          beta1 = -beta1;
        }

        *tau = (beta1 - *b_alpha1) / beta1;
        *b_alpha1 = 1.0 / (*b_alpha1 - beta1);
        for (k = ix0; k <= b_tmp - 2; k++) {
          x[k - 1] *= *b_alpha1;
        }

        for (k = 0; k <= knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }

        *b_alpha1 = beta1;
      } else {
        *tau = (beta1 - alpha1) / beta1;
        *b_alpha1 = 1.0 / (alpha1 - beta1);
        knt = ix0 + n;
        for (k = ix0; k <= knt - 2; k++) {
          x[k - 1] *= *b_alpha1;
        }

        *b_alpha1 = beta1;
      }
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzgehrd(real_T a[36],
  int32_T ilo, int32_T ihi)
{
  real_T work[6];
  real_T tau[5];
  real_T b_alpha1;
  real_T temp;
  real_T tmp;
  int32_T exitg1;
  int32_T i;
  int32_T iac;
  int32_T im1n_tmp;
  int32_T in;
  int32_T ix;
  int32_T jA;
  int32_T jy;
  int32_T lastc;
  int32_T lastv;
  int32_T rowleft;
  int32_T tmp_0;
  int32_T work_tmp;
  boolean_T exitg2;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((ihi - ilo) + 1 > 1) {
    if (static_cast<uint8_T>(ilo - 1) - 1 >= 0) {
      std::memset(&tau[0], 0, static_cast<uint8_T>(ilo - 1) * sizeof(real_T));
    }

    for (i = ihi; i < 6; i++) {
      tau[i - 1] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      work[i] = 0.0;
    }

    for (i = ilo; i < ihi; i++) {
      im1n_tmp = (i - 1) * 6;
      in = i * 6 + 1;
      if (i + 2 <= 6) {
        rowleft = i + 2;
      } else {
        rowleft = 6;
      }

      tmp_0 = i + im1n_tmp;
      jy = ihi - i;
      AEBSensorFusion_xzlarfg(jy, a[tmp_0], a, rowleft + im1n_tmp, &b_alpha1,
        &tau[i - 1]);
      a[tmp_0] = 1.0;
      lastv = jy;
      tmp = tau[i - 1];
      if (tmp != 0.0) {
        lastc = (tmp_0 + jy) - 1;
        while ((lastv > 0) && (a[lastc] == 0.0)) {
          lastv--;
          lastc--;
        }

        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          rowleft = (in + lastc) - 1;
          im1n_tmp = rowleft;
          do {
            exitg1 = 0;
            if (im1n_tmp <= (lastv - 1) * 6 + rowleft) {
              if (a[im1n_tmp - 1] != 0.0) {
                exitg1 = 1;
              } else {
                im1n_tmp += 6;
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
        if (lastc != 0) {
          std::memset(&work[0], 0, static_cast<uint8_T>(lastc) * sizeof(real_T));
          ix = tmp_0;
          rowleft = (lastv - 1) * 6 + in;
          for (iac = in; iac <= rowleft; iac += 6) {
            jA = iac + lastc;
            for (im1n_tmp = iac; im1n_tmp < jA; im1n_tmp++) {
              work_tmp = im1n_tmp - iac;
              work[work_tmp] += a[im1n_tmp - 1] * a[ix];
            }

            ix++;
          }
        }

        if (!(-tmp == 0.0)) {
          jA = in - 1;
          im1n_tmp = static_cast<uint8_T>(lastv);
          for (iac = 0; iac < im1n_tmp; iac++) {
            temp = a[tmp_0 + iac];
            if (temp != 0.0) {
              temp *= -tmp;
              lastv = jA + 1;
              rowleft = lastc + jA;
              for (work_tmp = lastv; work_tmp <= rowleft; work_tmp++) {
                a[work_tmp - 1] += work[(work_tmp - jA) - 1] * temp;
              }
            }

            jA += 6;
          }
        }
      }

      lastv = jy;
      in += i;
      if (tmp != 0.0) {
        lastc = (tmp_0 + jy) - 1;
        while ((lastv > 0) && (a[lastc] == 0.0)) {
          lastv--;
          lastc--;
        }

        lastc = 6 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          jy = (lastc - 1) * 6 + in;
          im1n_tmp = jy;
          do {
            exitg1 = 0;
            if (im1n_tmp <= (jy + lastv) - 1) {
              if (a[im1n_tmp - 1] != 0.0) {
                exitg1 = 1;
              } else {
                im1n_tmp++;
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
        if (lastc != 0) {
          std::memset(&work[0], 0, static_cast<uint8_T>(lastc) * sizeof(real_T));
          rowleft = (lastc - 1) * 6 + in;
          for (iac = in; iac <= rowleft; iac += 6) {
            temp = 0.0;
            jA = iac + lastv;
            for (im1n_tmp = iac; im1n_tmp < jA; im1n_tmp++) {
              temp += a[(tmp_0 + im1n_tmp) - iac] * a[im1n_tmp - 1];
            }

            work_tmp = div_nde_s32_floor(iac - in, 6);
            work[work_tmp] += temp;
          }
        }

        if (!(-tmp == 0.0)) {
          jA = in;
          im1n_tmp = static_cast<uint8_T>(lastc) - 1;
          for (iac = 0; iac <= im1n_tmp; iac++) {
            temp = work[iac];
            if (temp != 0.0) {
              temp *= -tmp;
              rowleft = (lastv + jA) - 1;
              for (work_tmp = jA; work_tmp <= rowleft; work_tmp++) {
                a[work_tmp - 1] += a[(tmp_0 + work_tmp) - jA] * temp;
              }
            }

            jA += 6;
          }
        }
      }

      a[tmp_0] = b_alpha1;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xdlanv2(real_T a, real_T b,
  real_T c, real_T d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i,
  real_T *b_a, real_T *b_b, real_T *b_c, real_T *b_d, real_T *cs, real_T *sn)
{
  real_T bcmax;
  real_T bcmis;
  real_T p;
  real_T scale;
  real_T temp;
  real_T z;
  int32_T count;
  int32_T tmp;
  boolean_T tmp_0;
  *b_d = d;
  *b_c = c;
  *b_b = b;
  *b_a = a;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    *b_d = a;
    *b_a = d;
    *b_b = -c;
    *b_c = 0.0;
  } else {
    temp = a - d;
    if ((temp == 0.0) && ((b < 0.0) != (c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      p = 0.5 * temp;
      scale = std::abs(b);
      z = std::abs(c);
      tmp_0 = rtIsNaN(z);
      if ((scale >= z) || tmp_0) {
        bcmax = scale;
      } else {
        bcmax = z;
      }

      if ((scale <= z) || tmp_0) {
        z = scale;
      }

      if (!(b < 0.0)) {
        count = 1;
      } else {
        count = -1;
      }

      if (!(c < 0.0)) {
        tmp = 1;
      } else {
        tmp = -1;
      }

      bcmis = z * static_cast<real_T>(count) * static_cast<real_T>(tmp);
      scale = std::abs(p);
      if ((!(scale >= bcmax)) && (!rtIsNaN(bcmax))) {
        scale = bcmax;
      }

      z = p / scale * p + bcmax / scale * bcmis;
      if (z >= 8.8817841970012523E-16) {
        scale = std::sqrt(scale) * std::sqrt(z);
        if (p < 0.0) {
          scale = -scale;
        }

        z = p + scale;
        *b_a = d + z;
        *b_d = d - bcmax / z * bcmis;
        scale = rt_hypotd_snf(c, z);
        *cs = z / scale;
        *sn = c / scale;
        *b_b = b - c;
        *b_c = 0.0;
      } else {
        p = b + c;
        z = std::abs(temp);
        scale = std::abs(p);
        if ((z >= scale) || rtIsNaN(scale)) {
          scale = z;
        }

        count = 0;
        while ((scale >= 7.4428285367870146E+137) && (count <= 20)) {
          p *= 1.3435752215134178E-138;
          temp *= 1.3435752215134178E-138;
          z = std::abs(temp);
          scale = std::abs(p);
          if ((z >= scale) || rtIsNaN(scale)) {
            scale = z;
          }

          count++;
        }

        while ((scale <= 1.3435752215134178E-138) && (count <= 20)) {
          p *= 7.4428285367870146E+137;
          temp *= 7.4428285367870146E+137;
          z = std::abs(temp);
          scale = std::abs(p);
          if ((z >= scale) || rtIsNaN(scale)) {
            scale = z;
          }

          count++;
        }

        scale = rt_hypotd_snf(p, temp);
        *cs = std::sqrt((std::abs(p) / scale + 1.0) * 0.5);
        if (!(p < 0.0)) {
          count = 1;
        } else {
          count = -1;
        }

        *sn = -(0.5 * temp / (scale * *cs)) * static_cast<real_T>(count);
        temp = a * *cs + b * *sn;
        p = -a * *sn + b * *cs;
        scale = c * *cs + d * *sn;
        bcmax = -c * *sn + d * *cs;
        *b_b = p * *cs + bcmax * *sn;
        *b_c = -temp * *sn + scale * *cs;
        temp = ((temp * *cs + scale * *sn) + (-p * *sn + bcmax * *cs)) * 0.5;
        *b_a = temp;
        *b_d = temp;
        if (*b_c != 0.0) {
          if (*b_b != 0.0) {
            if ((*b_b < 0.0) == (*b_c < 0.0)) {
              z = std::sqrt(std::abs(*b_b));
              bcmax = std::sqrt(std::abs(*b_c));
              if (!(*b_c < 0.0)) {
                p = z * bcmax;
              } else {
                p = -(z * bcmax);
              }

              scale = 1.0 / std::sqrt(std::abs(*b_b + *b_c));
              *b_a = temp + p;
              *b_d = temp - p;
              *b_b -= *b_c;
              *b_c = 0.0;
              p = z * scale;
              scale *= bcmax;
              temp = *cs * p - *sn * scale;
              *sn = *cs * scale + *sn * p;
              *cs = temp;
            }
          } else {
            *b_b = -*b_c;
            *b_c = 0.0;
            temp = *cs;
            *cs = -*sn;
            *sn = temp;
          }
        }
      }
    }
  }

  *rt1r = *b_a;
  *rt2r = *b_d;
  if (*b_c == 0.0) {
    *rt1i = 0.0;
    *rt2i = 0.0;
  } else {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    *rt1i = std::sqrt(std::abs(*b_b)) * std::sqrt(std::abs(*b_c));
    *rt2i = -*rt1i;
  }
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_p(int32_T n, const
  real_T x[3])
{
  real_T y;
  y = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[1]);
    } else {
      real_T absxk;
      real_T scale;
      real_T t;
      scale = 3.3121686421112381E-170;
      absxk = std::abs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
      }

      absxk = std::abs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return y;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xdlahqr(int32_T ilo, int32_T
  ihi, real_T h[36], int32_T *info, real_T *b_z, real_T wr[6], real_T wi[6])
{
  real_T v[3];
  real_T aa;
  real_T bb;
  real_T h11;
  real_T h12;
  real_T h21;
  real_T h_0;
  real_T smlnum;
  real_T tst;
  real_T tst_tmp;
  real_T tst_tmp_tmp;
  int32_T c_i;
  int32_T h_tmp;
  int32_T h_tmp_0;
  int32_T i;
  int32_T ih;
  int32_T its;
  int32_T k;
  int32_T kdefl;
  int32_T knt;
  int32_T l;
  int32_T m;
  int32_T nr;
  boolean_T converged;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T tmp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  *b_z = 1.0;
  *info = 0;
  kdefl = static_cast<uint8_T>(ilo - 1);
  for (c_i = 0; c_i < kdefl; c_i++) {
    wr[c_i] = h[6 * c_i + c_i];
    wi[c_i] = 0.0;
  }

  for (i = ihi + 1; i < 7; i++) {
    wr[i - 1] = h[((i - 1) * 6 + i) - 1];
    wi[i - 1] = 0.0;
  }

  if (ilo == ihi) {
    wr[ilo - 1] = h[((ilo - 1) * 6 + ilo) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    for (i = ilo; i <= ihi - 3; i++) {
      ih = (i - 1) * 6 + i;
      h[ih + 1] = 0.0;
      h[ih + 2] = 0.0;
    }

    if (ilo <= ihi - 2) {
      h[(ihi + 6 * (ihi - 3)) - 1] = 0.0;
    }

    smlnum = static_cast<real_T>((ihi - ilo) + 1) / 2.2204460492503131E-16 *
      2.2250738585072014E-308;
    kdefl = 0;
    i = ihi - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= ilo)) {
      l = ilo;
      converged = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its < 301)) {
        k = i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > l)) {
          ih = (k - 1) * 6 + k;
          if (std::abs(h[ih]) <= smlnum) {
            exitg3 = true;
          } else {
            c_i = 6 * k + k;
            tst = std::abs(h[ih - 1]) + std::abs(h[c_i]);
            if (tst == 0.0) {
              if (k - 1 >= ilo) {
                tst = std::abs(h[((k - 2) * 6 + k) - 1]);
              }

              if (k + 2 <= ihi) {
                tst += std::abs(h[c_i + 1]);
              }
            }

            if (std::abs(h[ih]) <= 2.2204460492503131E-16 * tst) {
              tst = std::abs(h[ih - 1] - h[c_i]);
              aa = std::abs(h[c_i]);
              tmp = !rtIsNaN(tst);
              if ((!(aa >= tst)) && tmp) {
                aa = tst;
              }

              bb = std::abs(h[c_i]);
              if ((!(bb <= tst)) && tmp) {
                bb = tst;
              }

              tst = aa + bb;
              h11 = std::abs(h[ih]);
              h21 = std::abs(h[c_i - 1]);
              bb = aa / tst * bb * 2.2204460492503131E-16;
              tmp = rtIsNaN(h21);
              if ((h11 <= h21) || tmp) {
                h12 = h11;
              } else {
                h12 = h21;
              }

              if ((h11 >= h21) || tmp) {
                h21 = h11;
              }

              if ((smlnum >= bb) || rtIsNaN(bb)) {
                bb = smlnum;
              }

              if (h21 / tst * h12 <= bb) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }

        l = k + 1;
        if (k + 1 > ilo) {
          h[k + 6 * (k - 1)] = 0.0;
        }

        if (k + 1 >= i) {
          converged = true;
          exitg2 = true;
        } else {
          kdefl++;
          if (kdefl - kdefl / 20 * 20 == 0) {
            tst = std::abs(h[((i - 2) * 6 + i) - 1]) + std::abs(h[(i - 1) * 6 +
              i]);
            h11 = h[6 * i + i] + 0.75 * tst;
            h12 = -0.4375 * tst;
            h21 = tst;
            aa = h11;
          } else if (kdefl - kdefl / 10 * 10 == 0) {
            c_i = 6 * k + k;
            tst = std::abs(h[((k + 1) * 6 + k) + 2]) + std::abs(h[c_i + 1]);
            h11 = 0.75 * tst + h[c_i];
            h12 = -0.4375 * tst;
            h21 = tst;
            aa = h11;
          } else {
            m = (i - 1) * 6 + i;
            h11 = h[m - 1];
            h21 = h[m];
            m = 6 * i + i;
            h12 = h[m - 1];
            aa = h[m];
          }

          tst = ((std::abs(h11) + std::abs(h12)) + std::abs(h21)) + std::abs(aa);
          if (tst == 0.0) {
            h11 = 0.0;
            aa = 0.0;
            bb = 0.0;
            h21 = 0.0;
          } else {
            h11 /= tst;
            aa /= tst;
            bb = (h11 + aa) / 2.0;
            h11 = (h11 - bb) * (aa - bb) - h12 / tst * (h21 / tst);
            h21 = std::sqrt(std::abs(h11));
            if (h11 >= 0.0) {
              h11 = bb * tst;
              bb = h11;
              aa = h21 * tst;
              h21 = -aa;
            } else {
              h11 = bb + h21;
              bb -= h21;
              if (std::abs(h11 - aa) <= std::abs(bb - aa)) {
                h11 *= tst;
                bb = h11;
              } else {
                bb *= tst;
                h11 = bb;
              }

              aa = 0.0;
              h21 = 0.0;
            }
          }

          m = i - 2;
          exitg3 = false;
          while ((!exitg3) && (m + 1 >= k + 1)) {
            c_i = 6 * m + m;
            tst_tmp_tmp = h[c_i];
            tst_tmp = tst_tmp_tmp - bb;
            h12 = h[c_i + 1];
            tst = (std::abs(tst_tmp) + std::abs(h21)) + std::abs(h12);
            h12 /= tst;
            nr = (m + 1) * 6 + m;
            v[0] = (tst_tmp / tst * tst_tmp + h[nr] * h12) - h21 / tst * aa;
            v[1] = (((h[nr + 1] + tst_tmp_tmp) - h11) - bb) * h12;
            v[2] = h[nr + 2] * h12;
            tst = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            v[0] /= tst;
            v[1] /= tst;
            v[2] /= tst;
            if (m + 1 == k + 1) {
              exitg3 = true;
            } else {
              ih = (m - 1) * 6 + m;
              if ((std::abs(v[1]) + std::abs(v[2])) * std::abs(h[ih]) <= ((std::
                    abs(h[ih - 1]) + std::abs(h[c_i])) + std::abs(h[nr + 1])) *
                  (2.2204460492503131E-16 * std::abs(v[0]))) {
                exitg3 = true;
              } else {
                m--;
              }
            }
          }

          for (c_i = m + 1; c_i <= i; c_i++) {
            nr = (i - c_i) + 2;
            if (nr >= 3) {
              nr = 3;
            }

            if (c_i > m + 1) {
              ih = ((c_i - 2) * 6 + c_i) - 2;
              for (knt = 0; knt < nr; knt++) {
                v[knt] = h[(ih + knt) + 1];
              }
            }

            aa = v[0];
            tst = 0.0;
            if (nr > 0) {
              bb = AEBSensorFusion_xnrm2_p(nr - 1, v);
              if (bb != 0.0) {
                bb = rt_hypotd_snf(v[0], bb);
                if (v[0] >= 0.0) {
                  bb = -bb;
                }

                if (std::abs(bb) < 1.0020841800044864E-292) {
                  knt = -1;
                  do {
                    knt++;
                    for (ih = 2; ih <= nr; ih++) {
                      v[ih - 1] *= 9.9792015476736E+291;
                    }

                    bb *= 9.9792015476736E+291;
                    aa *= 9.9792015476736E+291;
                  } while ((std::abs(bb) < 1.0020841800044864E-292) && (knt + 1 <
                            20));

                  bb = rt_hypotd_snf(aa, AEBSensorFusion_xnrm2_p(nr - 1, v));
                  if (aa >= 0.0) {
                    bb = -bb;
                  }

                  tst = (bb - aa) / bb;
                  aa = 1.0 / (aa - bb);
                  for (ih = 2; ih <= nr; ih++) {
                    v[ih - 1] *= aa;
                  }

                  for (ih = 0; ih <= knt; ih++) {
                    bb *= 1.0020841800044864E-292;
                  }

                  aa = bb;
                } else {
                  tst = (bb - v[0]) / bb;
                  aa = 1.0 / (v[0] - bb);
                  for (ih = 2; ih <= nr; ih++) {
                    v[ih - 1] *= aa;
                  }

                  aa = bb;
                }
              }
            }

            if (c_i > m + 1) {
              ih = (c_i - 2) * 6 + c_i;
              h[ih - 1] = aa;
              h[ih] = 0.0;
              if (c_i < i) {
                h[ih + 1] = 0.0;
              }
            } else if (m + 1 > k + 1) {
              ih = ((c_i - 2) * 6 + c_i) - 1;
              h[ih] *= 1.0 - tst;
            }

            h21 = v[1];
            bb = tst * v[1];
            if (nr == 3) {
              tst_tmp = v[2];
              h12 = tst * v[2];
              for (ih = c_i; ih <= i + 1; ih++) {
                knt = (ih - 1) * 6 + c_i;
                aa = h[knt - 1];
                tst_tmp_tmp = h[knt];
                h_0 = h[knt + 1];
                h11 = (h21 * tst_tmp_tmp + aa) + tst_tmp * h_0;
                h[knt - 1] = aa - h11 * tst;
                h[knt] = tst_tmp_tmp - h11 * bb;
                h[knt + 1] = h_0 - h11 * h12;
              }

              if (c_i + 3 <= i + 1) {
                nr = c_i + 3;
              } else {
                nr = i + 1;
              }

              for (ih = k + 1; ih <= nr; ih++) {
                knt = ((c_i - 1) * 6 + ih) - 1;
                aa = h[knt];
                h_tmp = (6 * c_i + ih) - 1;
                tst_tmp_tmp = h[h_tmp];
                h_tmp_0 = ((c_i + 1) * 6 + ih) - 1;
                h_0 = h[h_tmp_0];
                h11 = (h21 * tst_tmp_tmp + aa) + tst_tmp * h_0;
                h[knt] = aa - h11 * tst;
                h[h_tmp] = tst_tmp_tmp - h11 * bb;
                h[h_tmp_0] = h_0 - h11 * h12;
              }
            } else if (nr == 2) {
              for (ih = c_i; ih <= i + 1; ih++) {
                knt = (ih - 1) * 6 + c_i;
                aa = h[knt - 1];
                tst_tmp_tmp = h[knt];
                h11 = h21 * tst_tmp_tmp + aa;
                h[knt - 1] = aa - h11 * tst;
                h[knt] = tst_tmp_tmp - h11 * bb;
              }

              for (ih = k + 1; ih <= i + 1; ih++) {
                knt = ((c_i - 1) * 6 + ih) - 1;
                aa = h[knt];
                h_tmp = (6 * c_i + ih) - 1;
                tst_tmp_tmp = h[h_tmp];
                h11 = h21 * tst_tmp_tmp + aa;
                h[knt] = aa - h11 * tst;
                h[h_tmp] = tst_tmp_tmp - h11 * bb;
              }
            }
          }

          its++;
        }
      }

      if (!converged) {
        *info = i + 1;
        exitg1 = true;
      } else {
        if (i + 1 == l) {
          wr[i] = h[6 * i + i];
          wi[i] = 0.0;
        } else if (l == i) {
          ih = (i - 1) * 6 + i;
          kdefl = 6 * i + i;
          AEBSensorFusion_xdlanv2(h[ih - 1], h[kdefl - 1], h[ih], h[kdefl],
            &wr[i - 1], &wi[i - 1], &tst, &aa, &bb, &h11, &h21, &h12,
            &tst_tmp_tmp, &tst_tmp);
          wr[i] = tst;
          wi[i] = aa;
          h[ih - 1] = bb;
          h[kdefl - 1] = h11;
          h[ih] = h21;
          h[kdefl] = h12;
        }

        kdefl = 0;
        i = l - 2;
      }
    }

    if (*info != 0) {
      for (i = 0; i < 4; i++) {
        for (c_i = i + 3; c_i < 7; c_i++) {
          h[(c_i + 6 * i) - 1] = 0.0;
        }
      }
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlascl_i(real_T cfrom,
  real_T cto, int32_T m, real_T A[6], int32_T iA0)
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (int32_T b_i = 0; b_i < m; b_i++) {
      int32_T tmp;
      tmp = (b_i + iA0) - 1;
      A[tmp] *= mul;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_eigStandard(const real_T A
  [36], creal_T V[6])
{
  real_T A_0[36];
  real_T wi[6];
  real_T wr[6];
  real_T absxk;
  real_T anrm;
  real_T c;
  real_T cscale;
  real_T f;
  real_T g;
  real_T r;
  real_T scale;
  real_T t;
  real_T y;
  real_T y_0;
  int32_T c_tmp;
  int32_T exitg2;
  int32_T exitg3;
  int32_T exitg4;
  int32_T exitg5;
  int32_T i;
  int32_T ira;
  int32_T ix;
  int32_T ix_0;
  int32_T ix_tmp;
  int32_T k;
  int32_T kend;
  int32_T l;
  boolean_T exitg1;
  boolean_T exitg6;
  boolean_T exitg7;
  boolean_T notdone;
  boolean_T scalea;
  boolean_T skipThisColumn;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&A_0[0], &A[0], 36U * sizeof(real_T));
  anrm = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    absxk = std::abs(A[k]);
    if (rtIsNaN(absxk)) {
      anrm = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      k++;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    for (i = 0; i < 6; i++) {
      V[i].re = (rtNaN);
      V[i].im = 0.0;
    }
  } else {
    cscale = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      cscale = 6.7178761075670888E-139;
      AEBSensorFusion_xzlascl(anrm, cscale, A_0);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      AEBSensorFusion_xzlascl(anrm, cscale, A_0);
    }

    for (i = 0; i < 6; i++) {
      wr[i] = 1.0;
    }

    k = 1;
    l = 6;
    notdone = true;
    do {
      exitg5 = 0;
      if (notdone) {
        notdone = false;
        ix_0 = l;
        do {
          exitg4 = 0;
          if (ix_0 > 0) {
            skipThisColumn = false;
            ix = 1;
            exitg6 = false;
            while ((!exitg6) && (ix - 1 <= static_cast<uint8_T>(l) - 1)) {
              if ((ix == ix_0) || (!(A_0[((ix - 1) * 6 + ix_0) - 1] != 0.0))) {
                ix++;
              } else {
                skipThisColumn = true;
                exitg6 = true;
              }
            }

            if (skipThisColumn) {
              ix_0--;
            } else {
              wr[l - 1] = ix_0;
              if (ix_0 != l) {
                ix = (ix_0 - 1) * 6;
                ira = (l - 1) * 6;
                kend = static_cast<uint8_T>(l);
                for (i = 0; i < kend; i++) {
                  c_tmp = ix + i;
                  c = A_0[c_tmp];
                  ix_tmp = ira + i;
                  A_0[c_tmp] = A_0[ix_tmp];
                  A_0[ix_tmp] = c;
                }

                for (i = 0; i < 6; i++) {
                  c_tmp = (i * 6 + ix_0) - 1;
                  c = A_0[c_tmp];
                  ix_tmp = (i * 6 + l) - 1;
                  A_0[c_tmp] = A_0[ix_tmp];
                  A_0[ix_tmp] = c;
                }
              }

              exitg4 = 1;
            }
          } else {
            exitg4 = 2;
          }
        } while (exitg4 == 0);

        if (exitg4 == 1) {
          if (l == 1) {
            exitg5 = 1;
          } else {
            l--;
            notdone = true;
          }
        }
      } else {
        notdone = true;
        while (notdone) {
          notdone = false;
          ix_0 = k;
          exitg6 = false;
          while ((!exitg6) && (ix_0 <= l)) {
            skipThisColumn = false;
            i = k;
            exitg7 = false;
            while ((!exitg7) && (i <= l)) {
              if ((i == ix_0) || (!(A_0[((ix_0 - 1) * 6 + i) - 1] != 0.0))) {
                i++;
              } else {
                skipThisColumn = true;
                exitg7 = true;
              }
            }

            if (skipThisColumn) {
              ix_0++;
            } else {
              wr[k - 1] = ix_0;
              if (ix_0 != k) {
                ix = (ix_0 - 1) * 6;
                ira = (k - 1) * 6;
                kend = static_cast<uint8_T>(l);
                for (i = 0; i < kend; i++) {
                  c_tmp = ix + i;
                  c = A_0[c_tmp];
                  ix_tmp = ira + i;
                  A_0[c_tmp] = A_0[ix_tmp];
                  A_0[ix_tmp] = c;
                }

                ix = (ira + ix_0) - 1;
                ira = (ira + k) - 1;
                kend = static_cast<uint8_T>(7 - k);
                for (i = 0; i < kend; i++) {
                  c_tmp = i * 6 + ix;
                  c = A_0[c_tmp];
                  ix_tmp = i * 6 + ira;
                  A_0[c_tmp] = A_0[ix_tmp];
                  A_0[ix_tmp] = c;
                }
              }

              k++;
              notdone = true;
              exitg6 = true;
            }
          }
        }

        exitg5 = 2;
      }
    } while (exitg5 == 0);

    if (exitg5 == 1) {
    } else {
      exitg1 = false;
      while ((!exitg1) && (!notdone)) {
        notdone = true;
        ix_0 = k - 1;
        do {
          exitg3 = 0;
          if (ix_0 + 1 <= l) {
            c_tmp = l - k;
            c = AEBSensorFusion_xnrm2(c_tmp + 1, A_0, ix_0 * 6 + k);
            ix = (k - 1) * 6 + ix_0;
            ix_tmp = ix + 1;
            r = 0.0;
            if (c_tmp + 1 >= 1) {
              if (c_tmp + 1 == 1) {
                r = std::abs(A_0[ix]);
              } else {
                scale = 3.3121686421112381E-170;
                kend = (c_tmp * 6 + ix) + 1;
                for (i = ix_tmp; i <= kend; i += 6) {
                  absxk = std::abs(A_0[i - 1]);
                  if (absxk > scale) {
                    t = scale / absxk;
                    r = r * t * t + 1.0;
                    scale = absxk;
                  } else {
                    t = absxk / scale;
                    r += t * t;
                  }
                }

                r = scale * std::sqrt(r);
              }
            }

            c_tmp = ix_0 * 6;
            kend = 0;
            if (l > 1) {
              t = std::abs(A_0[c_tmp]);
              for (i = 2; i <= l; i++) {
                absxk = std::abs(A_0[(c_tmp + i) - 1]);
                if (absxk > t) {
                  kend = i - 1;
                  t = absxk;
                }
              }
            }

            scale = std::abs(A_0[6 * ix_0 + kend]);
            kend = 7 - k;
            if (7 - k < 1) {
              ira = -2;
            } else {
              ira = -1;
              if (7 - k > 1) {
                t = std::abs(A_0[ix]);
                for (i = 2; i <= kend; i++) {
                  absxk = std::abs(A_0[(i - 1) * 6 + ix]);
                  if (absxk > t) {
                    ira = i - 2;
                    t = absxk;
                  }
                }
              }
            }

            t = std::abs(A_0[(ira + k) * 6 + ix_0]);
            if ((c == 0.0) || (r == 0.0)) {
              ix_0++;
            } else {
              g = r / 2.0;
              f = 1.0;
              absxk = c + r;
              do {
                exitg2 = 0;
                if (c < g) {
                  if ((c >= scale) || rtIsNaN(scale)) {
                    y = c;
                  } else {
                    y = scale;
                  }

                  if (f >= y) {
                    y = f;
                  }

                  if (y < 4.9896007738368E+291) {
                    if ((g <= t) || rtIsNaN(t)) {
                      y = g;
                    } else {
                      y = t;
                    }

                    if (r <= y) {
                      y = r;
                    }

                    if (y > 2.0041683600089728E-292) {
                      if (rtIsNaN(((((c + f) + scale) + r) + g) + t)) {
                        exitg2 = 1;
                      } else {
                        f *= 2.0;
                        c *= 2.0;
                        scale *= 2.0;
                        r /= 2.0;
                        g /= 2.0;
                        t /= 2.0;
                      }
                    } else {
                      exitg2 = 2;
                    }
                  } else {
                    exitg2 = 2;
                  }
                } else {
                  exitg2 = 2;
                }
              } while (exitg2 == 0);

              if (exitg2 == 1) {
                exitg3 = 2;
              } else {
                g = c / 2.0;
                exitg6 = false;
                while ((!exitg6) && (g >= r)) {
                  if ((r >= t) || rtIsNaN(t)) {
                    y = r;
                  } else {
                    y = t;
                  }

                  if (y < 4.9896007738368E+291) {
                    if ((f <= c) || rtIsNaN(c)) {
                      y = f;
                    } else {
                      y = c;
                    }

                    if ((g <= scale) || rtIsNaN(scale)) {
                      y_0 = g;
                    } else {
                      y_0 = scale;
                    }

                    if (y <= y_0) {
                      y_0 = y;
                    }

                    if (y_0 > 2.0041683600089728E-292) {
                      f /= 2.0;
                      c /= 2.0;
                      g /= 2.0;
                      scale /= 2.0;
                      r *= 2.0;
                      t *= 2.0;
                    } else {
                      exitg6 = true;
                    }
                  } else {
                    exitg6 = true;
                  }
                }

                if ((c + r >= 0.95 * absxk) || ((f < 1.0) && (wr[ix_0] < 1.0) &&
                     (f * wr[ix_0] <= 1.0020841800044864E-292)) || ((f > 1.0) &&
                     (wr[ix_0] > 1.0) && (wr[ix_0] >= 9.9792015476736E+291 / f)))
                {
                } else {
                  g = 1.0 / f;
                  wr[ix_0] *= f;
                  kend = ((6 - k) * 6 + ix) + 1;
                  for (i = ix_tmp; i <= kend; i += 6) {
                    A_0[i - 1] *= g;
                  }

                  kend = c_tmp + l;
                  for (i = c_tmp + 1; i <= kend; i++) {
                    A_0[i - 1] *= f;
                  }

                  notdone = false;
                }

                ix_0++;
              }
            }
          } else {
            exitg3 = 1;
          }
        } while (exitg3 == 0);

        if (exitg3 == 1) {
        } else {
          exitg1 = true;
        }
      }
    }

    AEBSensorFusion_xzgehrd(A_0, k, l);
    AEBSensorFusion_xdlahqr(k, l, A_0, &ix, &c, wr, wi);
    if (scalea) {
      AEBSensorFusion_xzlascl_i(cscale, anrm, 6 - ix, wr, ix + 1);
      AEBSensorFusion_xzlascl_i(cscale, anrm, 6 - ix, wi, ix + 1);
      if (ix != 0) {
        AEBSensorFusion_xzlascl_i(cscale, anrm, k - 1, wr, 1);
        AEBSensorFusion_xzlascl_i(cscale, anrm, k - 1, wi, 1);
      }
    }

    if (ix != 0) {
      for (i = k; i <= ix; i++) {
        wr[i - 1] = (rtNaN);
        wi[i - 1] = 0.0;
      }
    }

    for (i = 0; i < 6; i++) {
      V[i].re = wr[i];
      V[i].im = wi[i];
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_insertionsort(real_T x[6],
  int32_T xstart, int32_T xend)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (int32_T k = xstart + 1; k <= xend; k++) {
    real_T xc;
    int32_T idx;
    boolean_T exitg1;
    xc = x[k - 1];
    idx = k - 1;
    exitg1 = false;
    while ((!exitg1) && (idx >= xstart)) {
      real_T tmp;
      tmp = x[idx - 1];
      if (xc < tmp) {
        x[idx] = tmp;
        idx--;
      } else {
        exitg1 = true;
      }
    }

    x[idx] = xc;
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlascl_iv(real_T cfrom,
  real_T cto, int32_T m, real_T A[5], int32_T iA0)
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (int32_T b_i = 0; b_i < m; b_i++) {
      int32_T tmp;
      tmp = (b_i + iA0) - 1;
      A[tmp] *= mul;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xdlaev2(real_T a, real_T b,
  real_T c, real_T *rt1, real_T *rt2)
{
  real_T ab;
  real_T acmn;
  real_T acmx;
  real_T adf;
  real_T sm;
  sm = a + c;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  adf = std::abs(a - c);
  ab = std::abs(b + b);
  if (std::abs(a) > std::abs(c)) {
    acmx = a;
    acmn = c;
  } else {
    acmx = c;
    acmn = a;
  }

  if (adf > ab) {
    real_T a_0;
    a_0 = ab / adf;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    adf *= std::sqrt(a_0 * a_0 + 1.0);
  } else if (adf < ab) {
    real_T a_0;
    a_0 = adf / ab;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    adf = std::sqrt(a_0 * a_0 + 1.0) * ab;
  } else {
    adf = ab * 1.4142135623730951;
  }

  if (sm < 0.0) {
    *rt1 = (sm - adf) * 0.5;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else if (sm > 0.0) {
    *rt1 = (sm + adf) * 0.5;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else {
    *rt1 = 0.5 * adf;
    *rt2 = -0.5 * adf;
  }
}

int32_T ACCWithSensorFusionModelClass::AEBSensorFusion_xdsterf(real_T d[6],
  real_T e[5])
{
  real_T anorm;
  real_T anorm_0;
  real_T b_gamma;
  real_T e_0;
  real_T oldc;
  real_T p;
  real_T r;
  real_T rte;
  real_T s;
  int32_T anorm_tmp;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exitg4;
  int32_T info;
  int32_T iscale;
  int32_T jtot;
  int32_T l;
  int32_T l1;
  int32_T lend;
  int32_T lendsv_tmp;
  int32_T lsv;
  int32_T m;
  int32_T n_tmp;
  boolean_T exitg2;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  info = 0;
  jtot = 0;
  l1 = 1;
  do {
    exitg1 = 0;
    if (l1 > 6) {
      AEBSensorFusion_insertionsort(d, 1, 6);
      exitg1 = 1;
    } else {
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }

      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 6)) {
        if (std::abs(e[m - 1]) <= std::sqrt(std::abs(d[m - 1])) * std::sqrt(std::
             abs(d[m])) * 2.2204460492503131E-16) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }

      l = l1;
      lsv = l1;
      lend = m;
      lendsv_tmp = m + 1;
      l1 = m + 1;
      if (m == l) {
      } else {
        n_tmp = m - l;
        if (n_tmp + 1 <= 0) {
          anorm = 0.0;
        } else {
          anorm = std::abs(d[(l + n_tmp) - 1]);
          iscale = -1;
          exitg2 = false;
          while ((!exitg2) && (iscale + 1 <= n_tmp - 1)) {
            anorm_tmp = l + iscale;
            anorm_0 = std::abs(d[anorm_tmp]);
            if (rtIsNaN(anorm_0)) {
              anorm = (rtNaN);
              exitg2 = true;
            } else {
              if (anorm_0 > anorm) {
                anorm = anorm_0;
              }

              anorm_0 = std::abs(e[anorm_tmp]);
              if (rtIsNaN(anorm_0)) {
                anorm = (rtNaN);
                exitg2 = true;
              } else {
                if (anorm_0 > anorm) {
                  anorm = anorm_0;
                }

                iscale++;
              }
            }
          }
        }

        if (anorm == 0.0) {
        } else {
          iscale = 0;
          if (anorm > 2.2346346549904327E+153) {
            iscale = 1;
            AEBSensorFusion_xzlascl_i(anorm, 2.2346346549904327E+153, n_tmp + 1,
              d, l);
            AEBSensorFusion_xzlascl_iv(anorm, 2.2346346549904327E+153, n_tmp, e,
              l);
          } else if (anorm < 3.02546243347603E-123) {
            iscale = 2;
            AEBSensorFusion_xzlascl_i(anorm, 3.02546243347603E-123, n_tmp + 1, d,
              l);
            AEBSensorFusion_xzlascl_iv(anorm, 3.02546243347603E-123, n_tmp, e, l);
          }

          for (n_tmp = l; n_tmp < m; n_tmp++) {
            e_0 = e[n_tmp - 1];
            e[n_tmp - 1] = e_0 * e_0;
          }

          if (std::abs(d[m - 1]) < std::abs(d[l - 1])) {
            lend = lsv;
            l = m;
          }

          if (lend >= l) {
            do {
              exitg4 = 0;
              if (l != lend) {
                m = l;
                while ((m < lend) && (!(std::abs(e[m - 1]) <= std::abs(d[m - 1])
                         * 4.9303806576313238E-32 * std::abs(d[m])))) {
                  m++;
                }
              } else {
                m = lend;
              }

              if (m < lend) {
                e[m - 1] = 0.0;
              }

              if (m == l) {
                l++;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (l + 1 == m) {
                AEBSensorFusion_xdlaev2(d[l - 1], std::sqrt(e[l - 1]), d[l],
                  &d[l - 1], &anorm_0);
                d[l] = anorm_0;
                e[l - 1] = 0.0;
                l += 2;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (jtot == 180) {
                exitg4 = 1;
              } else {
                jtot++;
                rte = std::sqrt(e[l - 1]);
                e_0 = d[l - 1];
                anorm_0 = (d[l] - e_0) / (2.0 * rte);
                if (anorm_0 >= 0.0) {
                  s = rt_hypotd_snf(anorm_0, 1.0);
                } else {
                  s = -rt_hypotd_snf(anorm_0, 1.0);
                }

                anorm_0 = e_0 - rte / (anorm_0 + s);
                rte = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - anorm_0;
                p = b_gamma * b_gamma;
                for (n_tmp = m - 1; n_tmp >= l; n_tmp--) {
                  e_0 = e[n_tmp - 1];
                  r = p + e_0;
                  if (m - 1 != n_tmp) {
                    e[n_tmp] = s * r;
                  }

                  oldc = rte;
                  rte = p / r;
                  s = e_0 / r;
                  r = b_gamma;
                  p = d[n_tmp - 1];
                  b_gamma = (p - anorm_0) * rte - s * b_gamma;
                  d[n_tmp] = (p - b_gamma) + r;
                  if (rte != 0.0) {
                    p = b_gamma * b_gamma / rte;
                  } else {
                    p = oldc * e_0;
                  }
                }

                e[l - 1] = s * p;
                d[l - 1] = anorm_0 + b_gamma;
              }
            } while (exitg4 == 0);
          } else {
            do {
              exitg3 = 0;
              m = l;
              while ((m > lend) && (!(std::abs(e[m - 2]) <= std::abs(d[m - 1]) *
                       4.9303806576313238E-32 * std::abs(d[m - 2])))) {
                m--;
              }

              if (m > lend) {
                e[m - 2] = 0.0;
              }

              if (m == l) {
                l--;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (l - 1 == m) {
                AEBSensorFusion_xdlaev2(d[l - 1], std::sqrt(e[l - 2]), d[l - 2],
                  &d[l - 1], &anorm_0);
                d[l - 2] = anorm_0;
                e[l - 2] = 0.0;
                l -= 2;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (jtot == 180) {
                exitg3 = 1;
              } else {
                jtot++;
                rte = std::sqrt(e[l - 2]);
                e_0 = d[l - 1];
                anorm_0 = (d[l - 2] - e_0) / (2.0 * rte);
                if (anorm_0 >= 0.0) {
                  s = rt_hypotd_snf(anorm_0, 1.0);
                } else {
                  s = -rt_hypotd_snf(anorm_0, 1.0);
                }

                anorm_0 = e_0 - rte / (anorm_0 + s);
                rte = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - anorm_0;
                p = b_gamma * b_gamma;
                for (n_tmp = m; n_tmp < l; n_tmp++) {
                  e_0 = e[n_tmp - 1];
                  r = p + e_0;
                  if (n_tmp != m) {
                    e[n_tmp - 2] = s * r;
                  }

                  oldc = rte;
                  rte = p / r;
                  s = e_0 / r;
                  r = b_gamma;
                  b_gamma = (d[n_tmp] - anorm_0) * rte - s * b_gamma;
                  d[n_tmp - 1] = (d[n_tmp] - b_gamma) + r;
                  if (rte != 0.0) {
                    p = b_gamma * b_gamma / rte;
                  } else {
                    p = oldc * e_0;
                  }
                }

                e[l - 2] = s * p;
                d[l - 1] = anorm_0 + b_gamma;
              }
            } while (exitg3 == 0);
          }

          if (iscale == 1) {
            AEBSensorFusion_xzlascl_i(2.2346346549904327E+153, anorm, lendsv_tmp
              - lsv, d, lsv);
          } else if (iscale == 2) {
            AEBSensorFusion_xzlascl_i(3.02546243347603E-123, anorm, lendsv_tmp -
              lsv, d, lsv);
          }

          if (jtot >= 180) {
            for (jtot = 0; jtot < 5; jtot++) {
              if ((e[jtot] != 0.0) && (info <= 2147483646)) {
                info++;
              }
            }

            exitg1 = 1;
          }
        }
      }
    }
  } while (exitg1 == 0);

  return info;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_eig(const real_T A[36],
  creal_T V[6])
{
  real_T b_A[36];
  real_T a__3[6];
  real_T wi[6];
  real_T e[5];
  real_T tau[5];
  real_T absx;
  real_T anrm;
  real_T tau_tmp;
  real_T temp1;
  real_T temp2;
  int32_T b;
  int32_T b_j;
  int32_T c_tmp;
  int32_T exitg1;
  int32_T i;
  int32_T ii;
  int32_T iv;
  int32_T iy_tmp;
  int32_T tau_tmp_tmp;
  int32_T temp1_tmp;
  boolean_T exitg2;
  boolean_T iscale;
  iscale = true;
  for (i = 0; i < 36; i++) {
    if (iscale) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      anrm = A[i];
      if ((!rtIsInf(anrm)) && (!rtIsNaN(anrm))) {
      } else {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!iscale) {
    for (i = 0; i < 6; i++) {
      V[i].re = (rtNaN);
      V[i].im = 0.0;
    }
  } else {
    b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (b_j < 6)) {
      i = 0;
      do {
        exitg1 = 0;
        if (i <= b_j) {
          if (!(A[6 * b_j + i] == A[6 * i + b_j])) {
            iscale = false;
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (iscale) {
      std::memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
      anrm = 0.0;
      b_j = 1;
      exitg2 = false;
      while ((!exitg2) && (b_j - 1 < 6)) {
        i = 0;
        do {
          exitg1 = 0;
          if (i <= static_cast<uint8_T>(b_j) - 1) {
            absx = std::abs(A[(b_j - 1) * 6 + i]);
            if (rtIsNaN(absx)) {
              anrm = (rtNaN);
              exitg1 = 1;
            } else {
              if (absx > anrm) {
                anrm = absx;
              }

              i++;
            }
          } else {
            b_j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }

      if (rtIsInf(anrm) || rtIsNaN(anrm)) {
        for (i = 0; i < 6; i++) {
          a__3[i] = (rtNaN);
        }
      } else {
        iscale = false;
        if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
          iscale = true;
          anrm = 1.0010415475915505E-146 / anrm;
          AEBSensorFusion_xzlascl(1.0, anrm, b_A);
        } else if (anrm > 9.9895953610111751E+145) {
          iscale = true;
          anrm = 9.9895953610111751E+145 / anrm;
          AEBSensorFusion_xzlascl(1.0, anrm, b_A);
        }

        for (i = 0; i < 5; i++) {
          if (i + 3 <= 6) {
            c_tmp = i + 3;
          } else {
            c_tmp = 6;
          }

          b_j = 6 * i + i;
          AEBSensorFusion_xzlarfg(5 - i, b_A[b_j + 1], b_A, i * 6 + c_tmp, &e[i],
            &absx);
          if (absx != 0.0) {
            b_A[b_j + 1] = 1.0;
            for (b = i + 1; b < 6; b++) {
              tau[b - 1] = 0.0;
            }

            c_tmp = 5 - i;
            iy_tmp = 6 - i;
            for (iv = 0; iv < c_tmp; iv++) {
              temp1_tmp = i + iv;
              temp1 = b_A[(6 * i + temp1_tmp) + 1] * absx;
              temp2 = 0.0;
              b = (temp1_tmp + 1) * 6 + i;
              tau[temp1_tmp] += b_A[(b + iv) + 1] * temp1;
              for (ii = iv + 2; ii < iy_tmp; ii++) {
                tau_tmp_tmp = i + ii;
                tau_tmp = b_A[b + ii];
                tau[tau_tmp_tmp - 1] += tau_tmp * temp1;
                temp2 += b_A[6 * i + tau_tmp_tmp] * tau_tmp;
              }

              tau[temp1_tmp] += absx * temp2;
            }

            temp1 = 0.0;
            for (ii = 0; ii < c_tmp; ii++) {
              temp1 += b_A[(b_j + ii) + 1] * tau[i + ii];
            }

            temp1 *= -0.5 * absx;
            if (!(temp1 == 0.0)) {
              for (ii = 0; ii < c_tmp; ii++) {
                b = i + ii;
                tau[b] += b_A[(b_j + ii) + 1] * temp1;
              }
            }

            for (iv = 0; iv < c_tmp; iv++) {
              b = iv + 1;
              temp1_tmp = iv + i;
              temp1 = b_A[(6 * i + temp1_tmp) + 1];
              temp2 = tau[temp1_tmp];
              tau_tmp = temp2 * temp1;
              tau_tmp_tmp = (temp1_tmp + 1) * 6;
              b_A[(temp1_tmp + tau_tmp_tmp) + 1] = (b_A[((tau_tmp_tmp + iv) + i)
                + 1] - tau_tmp) - tau_tmp;
              for (ii = b + 1; ii < iy_tmp; ii++) {
                temp1_tmp = i + ii;
                b_A[temp1_tmp + tau_tmp_tmp] = (b_A[(tau_tmp_tmp + i) + ii] -
                  tau[temp1_tmp - 1] * temp1) - b_A[6 * i + temp1_tmp] * temp2;
              }
            }
          }

          b_A[b_j + 1] = e[i];
          a__3[i] = b_A[b_j];
          tau[i] = absx;
        }

        a__3[5] = b_A[35];
        i = AEBSensorFusion_xdsterf(a__3, e);
        if (i != 0) {
          for (i = 0; i < 6; i++) {
            a__3[i] = (rtNaN);
          }
        } else if (iscale) {
          anrm = 1.0 / anrm;
          for (i = 0; i < 6; i++) {
            a__3[i] *= anrm;
          }
        }
      }

      for (i = 0; i < 6; i++) {
        V[i].re = a__3[i];
        V[i].im = 0.0;
      }
    } else {
      iscale = true;
      b_j = 0;
      exitg2 = false;
      while ((!exitg2) && (b_j < 6)) {
        i = 0;
        do {
          exitg1 = 0;
          if (i <= b_j) {
            if (!(A[6 * b_j + i] == -A[6 * i + b_j])) {
              iscale = false;
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            b_j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }

      if (iscale) {
        std::memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
        AEBSensorFusion_xzgehrd(b_A, 1, 6);
        AEBSensorFusion_xdlahqr(1, 6, b_A, &b_j, &anrm, a__3, wi);
        b = static_cast<uint8_T>(b_j);
        for (i = 0; i < b; i++) {
          V[i].re = (rtNaN);
          V[i].im = 0.0;
        }

        for (i = b_j + 1; i < 7; i++) {
          V[i - 1].re = 0.0;
          V[i - 1].im = wi[i - 1];
        }
      } else {
        AEBSensorFusion_eigStandard(A, V);
      }
    }
  }
}

int32_T ACCWithSensorFusionModelClass::AEBSensorFusion_xpotrf_ba(real_T A[36])
{
  int32_T b_j;
  int32_T info;
  boolean_T exitg1;
  info = 0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_j = 1;
  exitg1 = false;
  while ((!exitg1) && (b_j - 1 < 6)) {
    real_T c;
    real_T ssq;
    int32_T b;
    int32_T idxA1j;
    int32_T idxA1jp1;
    int32_T idxAjj;
    idxA1j = (b_j - 1) * 6 + 1;
    idxAjj = (b_j + idxA1j) - 2;
    ssq = 0.0;
    if (b_j - 1 >= 1) {
      b = static_cast<uint8_T>(b_j - 1);
      for (idxA1jp1 = 0; idxA1jp1 < b; idxA1jp1++) {
        c = A[(idxA1j + idxA1jp1) - 1];
        ssq += c * c;
      }
    }

    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (b_j < 6) {
        idxA1jp1 = idxA1j + 6;
        if (b_j - 1 != 0) {
          b = ((5 - b_j) * 6 + idxA1j) + 6;
          for (int32_T iac = idxA1jp1; iac <= b; iac += 6) {
            int32_T d;
            int32_T ia;
            c = 0.0;
            d = (b_j + iac) - 1;
            for (ia = iac; ia < d; ia++) {
              c += A[((idxA1j + ia) - iac) - 1] * A[ia - 1];
            }

            ia = (div_nde_s32_floor((iac - idxA1j) - 6, 6) * 6 + idxAjj) + 6;
            A[ia] -= c;
          }
        }

        ssq = 1.0 / ssq;
        b = (5 - b_j) * 6 + idxAjj;
        for (idxA1jp1 = idxAjj + 7; idxA1jp1 <= b + 7; idxA1jp1 += 6) {
          A[idxA1jp1 - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      A[idxAjj] = ssq;
      info = b_j;
      exitg1 = true;
    }
  }

  return info;
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xzlangeM(const real_T x[36])
{
  real_T y;
  int32_T b_k;
  boolean_T exitg1;
  y = 0.0;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 36)) {
    real_T absxk;
    absxk = std::abs(x[b_k]);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      b_k++;
    }
  }

  return y;
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xdotc(int32_T n, const
  real_T x[36], int32_T ix0, const real_T y[36], int32_T iy0)
{
  real_T d;
  d = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (n >= 1) {
    int32_T b;
    b = static_cast<uint8_T>(n);
    for (int32_T k = 0; k < b; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return d;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy(int32_T n, real_T a,
  int32_T ix0, real_T y[36], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1(int32_T n, const
  real_T x[6], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return scale * std::sqrt(y);
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy_g(int32_T n, real_T a,
  const real_T x[36], int32_T ix0, real_T y[6], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy_go(int32_T n, real_T a,
  const real_T x[6], int32_T ix0, real_T y[36], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xrotg(real_T a, real_T b,
  real_T *b_a, real_T *b_b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  roe = b;
  absa = std::abs(a);
  absb = std::abs(b);
  if (absa > absb) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    roe = a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    *b_a = std::sqrt(ads * ads + bds * bds) * scale;
    if (roe < 0.0) {
      *b_a = -*b_a;
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    *c = a / *b_a;
    *s = b / *b_a;
    if (absa > absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xrot_ps(real_T x[36],
  int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  for (int32_T k = 0; k < 6; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xswap_dj(real_T x[36],
  int32_T ix0, int32_T iy0)
{
  for (int32_T k = 0; k < 6; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_svd(const real_T A[36],
  real_T U[36], real_T s[6], real_T V[36])
{
  real_T A_0[36];
  real_T e[6];
  real_T s_0[6];
  real_T work[6];
  real_T anrm;
  real_T cscale;
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  real_T ztest0;
  real_T ztest0_tmp_tmp;
  int32_T colqp1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  boolean_T apply_transform;
  boolean_T doscale;
  boolean_T exitg1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&A_0[0], &A[0], 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    s_0[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  std::memset(&U[0], 0, 36U * sizeof(real_T));
  std::memset(&V[0], 0, 36U * sizeof(real_T));
  doscale = false;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  anrm = AEBSensorFusion_xzlangeM(A);
  cscale = anrm;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    AEBSensorFusion_xzlascl(anrm, cscale, A_0);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    AEBSensorFusion_xzlascl(anrm, cscale, A_0);
  }

  for (i = 0; i < 5; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    qp1 = i + 2;
    colqp1 = 6 * i + i;
    qq = colqp1 + 1;
    apply_transform = false;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    nrm = AEBSensorFusion_xnrm2(6 - i, A_0, colqp1 + 1);
    if (nrm > 0.0) {
      apply_transform = true;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      if (A_0[colqp1] < 0.0) {
        ztest = -nrm;
        s_0[i] = -nrm;
      } else {
        ztest = nrm;
        s_0[i] = nrm;
      }

      if (std::abs(ztest) >= 1.0020841800044864E-292) {
        nrm = 1.0 / ztest;
        qp1jj = (colqp1 - i) + 1;
        for (qjj = qq; qjj <= qp1jj + 5; qjj++) {
          A_0[qjj - 1] *= nrm;
        }
      } else {
        qp1jj = (colqp1 - i) + 1;
        for (qjj = qq; qjj <= qp1jj + 5; qjj++) {
          A_0[qjj - 1] /= s_0[i];
        }
      }

      A_0[colqp1]++;
      s_0[i] = -s_0[i];
    } else {
      s_0[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 7; qp1jj++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      qjj = ((qp1jj - 1) * 6 + i) + 1;
      if (apply_transform) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        AEBSensorFusion_xaxpy(6 - i, -(AEBSensorFusion_xdotc(6 - i, A_0, colqp1
          + 1, A_0, qjj) / A_0[colqp1]), colqp1 + 1, A_0, qjj);
      }

      e[qp1jj - 1] = A_0[qjj - 1];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    for (qq = i + 1; qq < 7; qq++) {
      qp1jj = (6 * i + qq) - 1;
      U[qp1jj] = A_0[qp1jj];
    }

    if (i + 1 <= 4) {
      nrm = AEBSensorFusion_xnrm2_po1(5 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (std::abs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          for (qjj = qp1; qjj < 7; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          for (qjj = qp1; qjj < 7; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 7; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 7; qq++) {
          AEBSensorFusion_xaxpy_g(5 - i, e[qq - 1], A_0, (i + 6 * (qq - 1)) + 2,
            work, i + 2);
        }

        for (qq = qp1; qq < 7; qq++) {
          AEBSensorFusion_xaxpy_go(5 - i, -e[qq - 1] / e[i + 1], work, i + 2,
            A_0, (i + 6 * (qq - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 7; qq++) {
        V[(qq + 6 * i) - 1] = e[qq - 1];
      }
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  i = 5;
  s_0[5] = A_0[35];
  e[4] = A_0[34];
  e[5] = 0.0;
  for (colqp1 = 0; colqp1 < 6; colqp1++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    U[colqp1 + 30] = 0.0;
  }

  U[35] = 1.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (colqp1 = 4; colqp1 >= 0; colqp1--) {
    qq = 6 * colqp1 + colqp1;
    if (s_0[colqp1] != 0.0) {
      for (qp1jj = colqp1 + 2; qp1jj < 7; qp1jj++) {
        qjj = ((qp1jj - 1) * 6 + colqp1) + 1;
        AEBSensorFusion_xaxpy(6 - colqp1, -(AEBSensorFusion_xdotc(6 - colqp1, U,
          qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
      }

      for (qp1 = colqp1 + 1; qp1 < 7; qp1++) {
        qp1jj = (6 * colqp1 + qp1) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qq = 0; qq < colqp1; qq++) {
        U[qq + 6 * colqp1] = 0.0;
      }
    } else {
      for (qp1 = 0; qp1 < 6; qp1++) {
        U[qp1 + 6 * colqp1] = 0.0;
      }

      U[qq] = 1.0;
    }
  }

  for (colqp1 = 5; colqp1 >= 0; colqp1--) {
    if ((colqp1 + 1 <= 4) && (e[colqp1] != 0.0)) {
      qq = (6 * colqp1 + colqp1) + 2;
      for (qjj = colqp1 + 2; qjj < 7; qjj++) {
        qp1jj = ((qjj - 1) * 6 + colqp1) + 2;
        AEBSensorFusion_xaxpy(5 - colqp1, -(AEBSensorFusion_xdotc(5 - colqp1, V,
          qq, V, qp1jj) / V[qq - 1]), qq, V, qp1jj);
      }
    }

    for (qp1 = 0; qp1 < 6; qp1++) {
      V[qp1 + 6 * colqp1] = 0.0;
    }

    V[colqp1 + 6 * colqp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 6; qp1++) {
    ztest = s_0[qp1];
    if (ztest != 0.0) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      rt = std::abs(ztest);
      nrm = ztest / rt;
      s_0[qp1] = rt;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      if (qp1 + 1 < 6) {
        e[qp1] /= nrm;
      }

      qq = 6 * qp1;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      for (qjj = qq + 1; qjj <= qq + 6; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (qp1 + 1 < 6) {
      emm1 = e[qp1];
      if (emm1 != 0.0) {
        rt = std::abs(emm1);
        nrm = rt / emm1;
        e[qp1] = rt;
        s_0[qp1 + 1] *= nrm;
        colqp1 = (qp1 + 1) * 6;
        for (qjj = colqp1 + 1; qjj <= colqp1 + 6; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  rt = 0.0;
  nrm = 0.0;
  for (colqp1 = 0; colqp1 < 6; colqp1++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    ztest = std::abs(s_0[colqp1]);
    ztest0 = std::abs(e[colqp1]);
    if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
      ztest0 = ztest;
    }

    if ((!(nrm >= ztest0)) && (!rtIsNaN(ztest0))) {
      nrm = ztest0;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  while ((i + 1 > 0) && (!(rt >= 75.0))) {
    colqp1 = i;
    qp1 = i;
    exitg1 = false;
    while ((!exitg1) && (qp1 > -1)) {
      colqp1 = qp1;
      if (qp1 == 0) {
        exitg1 = true;
      } else {
        ztest0 = std::abs(e[qp1 - 1]);
        if ((ztest0 <= (std::abs(s_0[qp1 - 1]) + std::abs(s_0[qp1])) *
             2.2204460492503131E-16) || ((ztest0 <= 1.0020841800044864E-292) ||
             ((rt > 20.0) && (ztest0 <= 2.2204460492503131E-16 * nrm)))) {
          e[qp1 - 1] = 0.0;
          exitg1 = true;
        } else {
          qp1--;
        }
      }
    }

    if (colqp1 == i) {
      ztest0 = 4.0;
    } else {
      qp1 = i + 1;
      qq = i + 1;
      exitg1 = false;
      while ((!exitg1) && (qq >= colqp1)) {
        qp1 = qq;
        if (qq == colqp1) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qq < i + 1) {
            ztest0 = std::abs(e[qq - 1]);
          }

          if (qq > colqp1 + 1) {
            ztest0 += std::abs(e[qq - 2]);
          }

          ztest = std::abs(s_0[qq - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s_0[qq - 1] = 0.0;
            exitg1 = true;
          } else {
            qq--;
          }
        }
      }

      if (qp1 == colqp1) {
        ztest0 = 3.0;
      } else if (i + 1 == qp1) {
        ztest0 = 1.0;
      } else {
        ztest0 = 2.0;
        colqp1 = qp1;
      }
    }

    switch (static_cast<int32_T>(ztest0)) {
     case 1:
      ztest0 = e[i - 1];
      e[i - 1] = 0.0;
      for (qq = i; qq >= colqp1 + 1; qq--) {
        AEBSensorFusion_xrotg(s_0[qq - 1], ztest0, &s_0[qq - 1], &ztest0, &ztest,
                              &sqds);
        if (qq > colqp1 + 1) {
          emm1 = e[qq - 2];
          ztest0 = -sqds * emm1;
          e[qq - 2] = emm1 * ztest;
        }

        AEBSensorFusion_xrot_ps(V, 6 * (qq - 1) + 1, 6 * i + 1, ztest, sqds);
      }
      break;

     case 2:
      ztest0 = e[colqp1 - 1];
      e[colqp1 - 1] = 0.0;
      for (qp1 = colqp1 + 1; qp1 <= i + 1; qp1++) {
        AEBSensorFusion_xrotg(s_0[qp1 - 1], ztest0, &s_0[qp1 - 1], &ztest, &sqds,
                              &smm1);
        emm1 = e[qp1 - 1];
        ztest0 = -smm1 * emm1;
        e[qp1 - 1] = emm1 * sqds;
        AEBSensorFusion_xrot_ps(U, 6 * (qp1 - 1) + 1, 6 * (colqp1 - 1) + 1, sqds,
          smm1);
      }
      break;

     case 3:
      ztest = std::abs(s_0[i]);
      smm1 = s_0[i - 1];
      ztest0 = std::abs(smm1);
      if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
        ztest0 = ztest;
      }

      sqds = e[i - 1];
      ztest = std::abs(sqds);
      if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
        ztest = ztest0;
      }

      ztest0 = std::abs(s_0[colqp1]);
      if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
        ztest0 = ztest;
      }

      ztest = std::abs(e[colqp1]);
      if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
        ztest = ztest0;
      }

      ztest0 = s_0[i] / ztest;
      smm1 /= ztest;
      emm1 = sqds / ztest;
      sqds = s_0[colqp1] / ztest;
      smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
      emm1 *= ztest0;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      ztest0 = (sqds + ztest0) * (sqds - ztest0) + shift;
      ztest = e[colqp1] / ztest * sqds;
      for (qq = colqp1 + 1; qq <= i; qq++) {
        AEBSensorFusion_xrotg(ztest0, ztest, &sqds, &smm1, &emm1, &shift);
        if (qq > colqp1 + 1) {
          e[qq - 2] = sqds;
        }

        ztest0_tmp_tmp = e[qq - 1];
        ztest0 = s_0[qq - 1];
        e[qq - 1] = ztest0_tmp_tmp * emm1 - ztest0 * shift;
        ztest = shift * s_0[qq];
        s_0[qq] *= emm1;
        qp1jj = (qq - 1) * 6 + 1;
        qjj = 6 * qq + 1;
        AEBSensorFusion_xrot_ps(V, qp1jj, qjj, emm1, shift);
        AEBSensorFusion_xrotg(ztest0 * emm1 + ztest0_tmp_tmp * shift, ztest,
                              &s_0[qq - 1], &sqds, &smm1, &emm1);
        ztest = e[qq - 1];
        ztest0 = ztest * smm1 + emm1 * s_0[qq];
        s_0[qq] = ztest * -emm1 + smm1 * s_0[qq];
        ztest = emm1 * e[qq];
        e[qq] *= smm1;
        AEBSensorFusion_xrot_ps(U, qp1jj, qjj, smm1, emm1);
      }

      e[i - 1] = ztest0;
      rt++;
      break;

     default:
      if (s_0[colqp1] < 0.0) {
        s_0[colqp1] = -s_0[colqp1];
        qq = 6 * colqp1;
        for (qjj = qq + 1; qjj <= qq + 6; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = colqp1 + 1;
      while ((colqp1 + 1 < 6) && (s_0[colqp1] < s_0[qp1])) {
        rt = s_0[colqp1];
        s_0[colqp1] = s_0[qp1];
        s_0[qp1] = rt;
        qp1jj = 6 * colqp1 + 1;
        qjj = (colqp1 + 1) * 6 + 1;
        AEBSensorFusion_xswap_dj(V, qp1jj, qjj);
        AEBSensorFusion_xswap_dj(U, qp1jj, qjj);
        colqp1 = qp1;
        qp1++;
      }

      rt = 0.0;
      i--;
      break;
    }
  }

  for (i = 0; i < 6; i++) {
    s[i] = s_0[i];
  }

  if (doscale) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_xzlascl_i(cscale, anrm, 6, s, 1);
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_cholPSD(const real_T A[36],
  real_T b_value[36])
{
  real_T Ss[36];
  real_T b_V[36];
  real_T s[6];
  real_T x;
  int32_T b_j;
  int32_T i;
  int32_T info;
  int32_T jmax;
  boolean_T p;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&Ss[0], &A[0], 36U * sizeof(real_T));
  info = AEBSensorFusion_xpotrf_ba(Ss);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (info == 0) {
    std::memcpy(&Ss[0], &A[0], 36U * sizeof(real_T));
    info = AEBSensorFusion_xpotrf_ba(Ss);
    if (info == 0) {
      jmax = 6;
    } else {
      jmax = info - 1;
    }

    info = jmax - 2;
    for (b_j = 0; b_j <= info; b_j++) {
      for (i = b_j + 2; i <= jmax; i++) {
        Ss[(i + 6 * b_j) - 1] = 0.0;
      }
    }

    for (info = 0; info < 6; info++) {
      for (jmax = 0; jmax < 6; jmax++) {
        b_value[jmax + 6 * info] = Ss[6 * jmax + info];
      }
    }
  } else {
    p = true;
    for (jmax = 0; jmax < 36; jmax++) {
      if (p) {
        x = A[jmax];
        if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
        } else {
          p = false;
        }
      } else {
        p = false;
      }
    }

    if (p) {
      AEBSensorFusion_svd(A, Ss, s, b_V);
    } else {
      for (i = 0; i < 6; i++) {
        s[i] = (rtNaN);
      }

      for (info = 0; info < 36; info++) {
        b_V[info] = (rtNaN);
      }
    }

    std::memset(&Ss[0], 0, 36U * sizeof(real_T));
    for (info = 0; info < 6; info++) {
      Ss[info + 6 * info] = s[info];
    }

    for (jmax = 0; jmax < 36; jmax++) {
      Ss[jmax] = std::sqrt(Ss[jmax]);
    }

    for (info = 0; info < 6; info++) {
      for (jmax = 0; jmax < 6; jmax++) {
        x = 0.0;
        for (i = 0; i < 6; i++) {
          x += b_V[6 * i + jmax] * Ss[6 * info + i];
        }

        b_value[jmax + 6 * info] = x;
      }
    }
  }
}

c_trackingEKF_AEBSensorFusion_T *ACCWithSensorFusionModelClass::
  AEBSensorFusion_initcvekf(const real_T Detection_Measurement[6], const real_T
  Detection_MeasurementNoise[36], drivingCoordinateFrameType
  Detection_MeasurementParameters_Frame, const real_T
  Detection_MeasurementParameters_OriginPosition[3], const real_T
  Detection_MeasurementParameters_Orientation[9], boolean_T
  Detection_MeasurementParameters_HasVelocity, const real_T
  Detection_MeasurementParameters_OriginVelocity[3], boolean_T
  Detection_MeasurementParameters_IsParentToChild, boolean_T
  Detection_MeasurementParameters_HasElevation, c_trackingEKF_AEBSensorFusion_T *
  iobj_0)
{
  c_trackingEKF_AEBSensorFusion_T *EKF;
  real_T stateCov[36];
  real_T stateCov_0[36];
  real_T tmp_1[36];
  real_T tmp[18];
  real_T tmp_0[18];
  real_T posCov[9];
  real_T velCov[9];
  real_T posMeas[3];
  real_T velMeas[3];
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  real_T tmp_5;
  real_T tmp_6;
  real_T tmp_7;
  int32_T i;
  int32_T i_0;
  int32_T stateCov_tmp;
  int32_T tmp_e;
  int32_T tmp_f;
  int8_T tmp_8;
  int8_T tmp_9;
  int8_T tmp_a;
  int8_T tmp_b;
  int8_T tmp_c;
  int8_T tmp_d;
  boolean_T invalidDet;
  static const int8_T tmp_g[18] = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1 };

  static const int8_T tmp_h[18] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 1, 0 };

  static const int8_T tmp_i[18] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 1 };

  static const int8_T tmp_j[18] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0 };

  static const int8_T tmp_k[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  AEBSensorFusion_parseDetectionForInitFcn(Detection_Measurement,
    Detection_MeasurementNoise, Detection_MeasurementParameters_Frame,
    Detection_MeasurementParameters_OriginPosition,
    Detection_MeasurementParameters_Orientation,
    Detection_MeasurementParameters_HasVelocity,
    Detection_MeasurementParameters_OriginVelocity,
    Detection_MeasurementParameters_IsParentToChild,
    Detection_MeasurementParameters_HasElevation, posMeas, velMeas, posCov,
    velCov, &invalidDet);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (i = 0; i < 6; i++) {
    tmp_8 = tmp_g[i + 6];
    tmp_9 = tmp_g[i];
    tmp_a = tmp_g[i + 12];
    tmp_b = tmp_h[i + 6];
    tmp_c = tmp_h[i];
    tmp_d = tmp_h[i + 12];
    for (i_0 = 0; i_0 < 3; i_0++) {
      stateCov_tmp = 3 * i_0 + 1;
      tmp_e = 3 * i_0 + 2;
      tmp_f = 6 * i_0 + i;
      tmp_0[tmp_f] = (velCov[3 * i_0] * static_cast<real_T>(tmp_9) +
                      velCov[stateCov_tmp] * static_cast<real_T>(tmp_8)) +
        velCov[tmp_e] * static_cast<real_T>(tmp_a);
      tmp[tmp_f] = (posCov[3 * i_0] * static_cast<real_T>(tmp_c) +
                    posCov[stateCov_tmp] * static_cast<real_T>(tmp_b)) +
        posCov[tmp_e] * static_cast<real_T>(tmp_d);
    }

    tmp_2 = tmp_0[i + 6];
    tmp_3 = tmp_0[i];
    tmp_4 = tmp_0[i + 12];
    tmp_5 = tmp[i + 6];
    tmp_6 = tmp[i];
    tmp_7 = tmp[i + 12];
    for (i_0 = 0; i_0 < 6; i_0++) {
      stateCov_tmp = 3 * i_0 + 1;
      tmp_e = 3 * i_0 + 2;
      tmp_f = 6 * i_0 + i;
      tmp_1[tmp_f] = (static_cast<real_T>(tmp_i[3 * i_0]) * tmp_3 +
                      static_cast<real_T>(tmp_i[stateCov_tmp]) * tmp_2) +
        static_cast<real_T>(tmp_i[tmp_e]) * tmp_4;
      stateCov_0[tmp_f] = (static_cast<real_T>(tmp_j[3 * i_0]) * tmp_6 +
                           static_cast<real_T>(tmp_j[stateCov_tmp]) * tmp_5) +
        static_cast<real_T>(tmp_j[tmp_e]) * tmp_7;
    }
  }

  for (i = 0; i < 36; i++) {
    stateCov[i] = stateCov_0[i] + tmp_1[i];
  }

  if (invalidDet) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0[0].pIsFirstCallPredict = true;
    iobj_0[0].pIsFirstCallCorrect = true;
    iobj_0[0].pSqrtStateCovarianceScalar = 1.0;
    for (i = 0; i < 6; i++) {
      iobj_0[0].pState[i] = ((static_cast<real_T>(tmp_h[i + 6]) * posMeas[1] +
        static_cast<real_T>(tmp_h[i]) * posMeas[0]) + static_cast<real_T>
        (tmp_h[i + 12]) * posMeas[2]) + ((static_cast<real_T>(tmp_g[i + 6]) *
        velMeas[1] + static_cast<real_T>(tmp_g[i]) * velMeas[0]) +
        static_cast<real_T>(tmp_g[i + 12]) * velMeas[2]);
      for (i_0 = 0; i_0 < 6; i_0++) {
        stateCov_tmp = 6 * i + i_0;
        stateCov_0[stateCov_tmp] = (stateCov[6 * i_0 + i] +
          stateCov[stateCov_tmp]) / 2.0;
      }
    }

    AEBSensorFusion_eig(stateCov_0, unusedExpr);

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_cholPSD(stateCov, iobj_0[0].pSqrtStateCovariance);
    iobj_0[0].pIsSetStateCovariance = true;
    iobj_0[0].pSqrtStateCovarianceScalar = -1.0;
    iobj_0[0].pIsValidStateTransitionFcn = false;
    iobj_0[0].pIsValidMeasurementFcn = false;
    iobj_0[0].pIsValidMeasurementFcn = false;
    iobj_0[0].pIsValidStateTransitionFcn = false;
    iobj_0[0].pSqrtProcessNoiseScalar = 1.0;
    for (i = 0; i < 9; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      iobj_0[0].pSqrtProcessNoise[i] = tmp_k[i];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0[0].pIsSetProcessNoise = true;
    iobj_0[0].pSqrtProcessNoiseScalar = -1.0;
    iobj_0[0].pSqrtMeasurementNoiseScalar = 1.0;
    iobj_0[0].pHasPrediction = false;
    iobj_0[0].pWasRetrodicted = false;
    iobj_0[0].IsLastJacobianInitialized = false;
    iobj_0[0].pIsDistributionsSetup = false;
    iobj_0[0].pIsInitialized = false;
    EKF = &iobj_0[0];
  } else {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0[1].pIsFirstCallPredict = true;
    iobj_0[1].pIsFirstCallCorrect = true;
    iobj_0[1].pSqrtStateCovarianceScalar = 1.0;
    for (i = 0; i < 6; i++) {
      iobj_0[1].pState[i] = ((static_cast<real_T>(tmp_h[i + 6]) * posMeas[1] +
        static_cast<real_T>(tmp_h[i]) * posMeas[0]) + static_cast<real_T>
        (tmp_h[i + 12]) * posMeas[2]) + ((static_cast<real_T>(tmp_g[i + 6]) *
        velMeas[1] + static_cast<real_T>(tmp_g[i]) * velMeas[0]) +
        static_cast<real_T>(tmp_g[i + 12]) * velMeas[2]);
      for (i_0 = 0; i_0 < 6; i_0++) {
        stateCov_tmp = 6 * i + i_0;
        stateCov_0[stateCov_tmp] = (stateCov[6 * i_0 + i] +
          stateCov[stateCov_tmp]) / 2.0;
      }
    }

    AEBSensorFusion_eig(stateCov_0, unusedExpr);

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_cholPSD(stateCov, iobj_0[1].pSqrtStateCovariance);
    iobj_0[1].pIsSetStateCovariance = true;
    iobj_0[1].pSqrtStateCovarianceScalar = -1.0;
    iobj_0[1].pIsValidStateTransitionFcn = false;
    iobj_0[1].pIsValidMeasurementFcn = false;
    iobj_0[1].pIsValidMeasurementFcn = false;
    iobj_0[1].pIsValidStateTransitionFcn = false;
    iobj_0[1].pSqrtProcessNoiseScalar = 1.0;
    for (i = 0; i < 9; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      iobj_0[1].pSqrtProcessNoise[i] = tmp_k[i];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0[1].pIsSetProcessNoise = true;
    iobj_0[1].pSqrtProcessNoiseScalar = -1.0;
    AEBSensorFusion_cholPSD(Detection_MeasurementNoise, iobj_0[1].
      pSqrtMeasurementNoise);
    iobj_0[1].pSqrtMeasurementNoiseScalar = -1.0;
    iobj_0[1].pHasPrediction = false;
    iobj_0[1].pWasRetrodicted = false;
    iobj_0[1].IsLastJacobianInitialized = false;
    iobj_0[1].pIsDistributionsSetup = false;
    iobj_0[1].pIsInitialized = false;
    EKF = &iobj_0[1];
  }

  return EKF;
}

c_trackingEKF_AEBSensorFusion_T *ACCWithSensorFusionModelClass::
  AEBSensorFusion_ExtendedKalmanFilter_clone(const
  c_trackingEKF_AEBSensorFusion_T *EKF, c_trackingEKF_AEBSensorFusion_T *iobj_0)
{
  c_trackingEKF_AEBSensorFusion_T *newEKF;
  real_T b_value[36];
  real_T b_value_0[9];
  real_T EKF_0[6];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iobj_0->pIsFirstCallPredict = true;
  iobj_0->pIsFirstCallCorrect = true;
  iobj_0->pSqrtStateCovarianceScalar = 1.0;
  iobj_0->pIsValidMeasurementFcn = false;
  iobj_0->pIsValidStateTransitionFcn = false;
  iobj_0->pSqrtProcessNoiseScalar = 1.0;
  iobj_0->pSqrtMeasurementNoiseScalar = 1.0;
  iobj_0->pHasPrediction = false;
  iobj_0->pWasRetrodicted = false;
  iobj_0->IsLastJacobianInitialized = false;
  iobj_0->pIsDistributionsSetup = false;
  iobj_0->pIsInitialized = false;
  newEKF = iobj_0;
  iobj_0->pIsSetStateCovariance = EKF->pIsSetStateCovariance;
  iobj_0->pIsSetProcessNoise = EKF->pIsSetProcessNoise;
  for (int32_T i = 0; i < 6; i++) {
    EKF_0[i] = EKF->pState[i];
  }

  for (int32_T i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0->pState[i] = EKF_0[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iobj_0->pIsValidStateTransitionFcn = false;
  iobj_0->pIsValidMeasurementFcn = false;
  for (int32_T i = 0; i < 36; i++) {
    b_value[i] = EKF->pSqrtStateCovariance[i];
  }

  for (int32_T i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0->pSqrtStateCovariance[i] = b_value[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iobj_0->pIsSetStateCovariance = true;
  iobj_0->pSqrtStateCovarianceScalar = -1.0;
  iobj_0->pSqrtStateCovarianceScalar = EKF->pSqrtStateCovarianceScalar;
  for (int32_T i = 0; i < 9; i++) {
    b_value_0[i] = EKF->pSqrtProcessNoise[i];
  }

  for (int32_T i = 0; i < 9; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0->pSqrtProcessNoise[i] = b_value_0[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iobj_0->pIsSetProcessNoise = true;
  iobj_0->pSqrtProcessNoiseScalar = -1.0;
  iobj_0->pSqrtProcessNoiseScalar = EKF->pSqrtProcessNoiseScalar;
  for (int32_T i = 0; i < 36; i++) {
    b_value[i] = EKF->pSqrtMeasurementNoise[i];
  }

  for (int32_T i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_0->pSqrtMeasurementNoise[i] = b_value[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iobj_0->pSqrtMeasurementNoiseScalar = EKF->pSqrtMeasurementNoiseScalar;
  iobj_0->pHasPrediction = EKF->pHasPrediction;
  iobj_0->pIsValidStateTransitionFcn = EKF->pIsValidStateTransitionFcn;
  iobj_0->pIsValidMeasurementFcn = EKF->pIsValidMeasurementFcn;
  return newEKF;
}

c_trackingEKF_AEBSensorFusion_T *ACCWithSensorFusionModelClass::
  AEBSensorFusion_trackingEKF_clone(c_trackingEKF_AEBSensorFusion_T *obj,
  c_trackingEKF_AEBSensorFusion_T *iobj_0)
{
  c_trackingEKF_AEBSensorFusion_T *obj2;
  obj2 = AEBSensorFusion_ExtendedKalmanFilter_clone(obj, iobj_0);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj2->IsLastJacobianInitialized = obj->IsLastJacobianInitialized;
  if (obj->pIsDistributionsSetup) {
    obj2->pIsDistributionsSetup = obj->pIsDistributionsSetup;
  }

  obj2->pShouldWarn = obj->pShouldWarn;
  obj2->pWasRetrodicted = obj->pWasRetrodicted;

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return obj2;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_ObjectTrack_nullify
  (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track)
{
  c_trackHistoryLogic_AEBSensorFusion_T *obj;
  c_trackingEKF_AEBSensorFusion_T *EKF;
  real_T b_I[36];
  int32_T i;
  track->BranchID = 0U;
  track->TrackID = 0U;
  track->IsConfirmed = false;
  track->ObjectClassID = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  track->pObjectAttributes[0].TargetIndex = 0.0;
  track->pObjectAttributes[0].SNR = 0.0;
  track->pObjectAttributes[1].TargetIndex = 0.0;
  track->pObjectAttributes[1].SNR = 0.0;
  track->pUsedObjectAttributes[0] = false;
  track->pUsedObjectAttributes[1] = false;
  track->UpdateTime = 0.0;
  track->Time = 0.0;
  track->pAge = 0U;
  track->pIsCoasted = true;
  obj = track->TrackLogic;
  for (i = 0; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }

  obj->pIsFirstUpdate = true;
  EKF = track->Filter;
  for (i = 0; i < 6; i++) {
    EKF->pState[i] = 0.0;
  }

  std::memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    b_I[i + 6 * i] = 1.0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cholPSD(b_I, EKF->pSqrtStateCovariance);
  EKF->pIsSetStateCovariance = true;
  EKF->pSqrtStateCovarianceScalar = -1.0;
  EKF = track->pDistanceFilter;
  for (i = 0; i < 6; i++) {
    EKF->pState[i] = 0.0;
  }

  std::memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    b_I[i + 6 * i] = 1.0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cholPSD(b_I, EKF->pSqrtStateCovariance);
  EKF->pIsSetStateCovariance = true;
  EKF->pSqrtStateCovarianceScalar = -1.0;
}

b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
  *ACCWithSensorFusionModelClass::AEBSensorFusion_TrackManager_createSampleTrack
  (const multiObjectTracker_AEBSensorFusion_T *obj,
   c_trackingEKF_AEBSensorFusion_T *iobj_0,
   c_trackHistoryLogic_AEBSensorFusion_T *iobj_1,
   b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *iobj_2)
{
  BusRadarDetectionsObjectAttributes detection_ObjectAttributes;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track;
  c_trackingEKF_AEBSensorFusion_T *filter;
  real_T detection_MeasurementNoise[36];
  real_T detection_MeasurementNoise_0[36];
  real_T detection_MeasurementParameters_Orientation[9];
  real_T detection_Measurement[6];
  real_T detection_MeasurementParameters_OriginPosition[3];
  real_T detection_MeasurementParameters_OriginVelocity[3];
  real_T b_value;
  real_T detection_ObjectClassID;
  int32_T detection_MeasurementNoise_tmp;
  int32_T i;
  int32_T i_0;
  drivingCoordinateFrameType detection_MeasurementParameters_Frame;
  boolean_T detection_MeasurementParameters_HasElevation;
  boolean_T detection_MeasurementParameters_HasVelocity;
  boolean_T detection_MeasurementParameters_IsParentToChild;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  for (i = 0; i < 6; i++) {
    detection_Measurement[i] = obj->pSampleDetection.Measurement[i];
  }

  for (i = 0; i < 36; i++) {
    detection_MeasurementNoise[i] = obj->pSampleDetection.MeasurementNoise[i];
  }

  detection_ObjectClassID = obj->pSampleDetection.ObjectClassID;
  detection_MeasurementParameters_Frame =
    obj->pSampleDetection.MeasurementParameters.Frame;
  detection_MeasurementParameters_OriginPosition[0] =
    obj->pSampleDetection.MeasurementParameters.OriginPosition[0];
  detection_MeasurementParameters_OriginPosition[1] =
    obj->pSampleDetection.MeasurementParameters.OriginPosition[1];
  detection_MeasurementParameters_OriginPosition[2] =
    obj->pSampleDetection.MeasurementParameters.OriginPosition[2];
  for (i = 0; i < 9; i++) {
    detection_MeasurementParameters_Orientation[i] =
      obj->pSampleDetection.MeasurementParameters.Orientation[i];
  }

  detection_MeasurementParameters_HasVelocity =
    obj->pSampleDetection.MeasurementParameters.HasVelocity;
  detection_MeasurementParameters_OriginVelocity[0] =
    obj->pSampleDetection.MeasurementParameters.OriginVelocity[0];
  detection_MeasurementParameters_OriginVelocity[1] =
    obj->pSampleDetection.MeasurementParameters.OriginVelocity[1];
  detection_MeasurementParameters_OriginVelocity[2] =
    obj->pSampleDetection.MeasurementParameters.OriginVelocity[2];
  detection_MeasurementParameters_IsParentToChild =
    obj->pSampleDetection.MeasurementParameters.IsParentToChild;
  detection_MeasurementParameters_HasElevation =
    obj->pSampleDetection.MeasurementParameters.HasElevation;
  detection_ObjectAttributes = obj->pSampleDetection.ObjectAttributes;
  for (i = 0; i < 50; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_1->pRecentHistory[i] = false;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  filter = AEBSensorFusion_initcvekf(detection_Measurement,
    detection_MeasurementNoise, detection_MeasurementParameters_Frame,
    detection_MeasurementParameters_OriginPosition,
    detection_MeasurementParameters_Orientation,
    detection_MeasurementParameters_HasVelocity,
    detection_MeasurementParameters_OriginVelocity,
    detection_MeasurementParameters_IsParentToChild,
    detection_MeasurementParameters_HasElevation, &iobj_0[0]);
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      detection_MeasurementNoise_tmp = 6 * i + i_0;
      detection_MeasurementNoise_0[detection_MeasurementNoise_tmp] =
        (detection_MeasurementNoise[6 * i_0 + i] +
         detection_MeasurementNoise[detection_MeasurementNoise_tmp]) / 2.0;
    }
  }

  AEBSensorFusion_eig(detection_MeasurementNoise_0, unusedExpr);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  filter->pSqrtMeasurementNoiseScalar = -1.0;
  if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
    b_value = filter->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 36; i++) {
      filter->pSqrtMeasurementNoise[i] = b_value * static_cast<real_T>(tmp[i]);
    }

    filter->pSqrtMeasurementNoiseScalar = -1.0;
  }

  AEBSensorFusion_cholPSD(detection_MeasurementNoise,
    filter->pSqrtMeasurementNoise);
  filter->pShouldWarn = false;
  track = iobj_2;
  iobj_2->BranchID = 1U;
  iobj_2->TrackID = 0U;
  iobj_2->Filter = filter;
  iobj_2->pDistanceFilter = AEBSensorFusion_trackingEKF_clone(filter, &iobj_0[2]);
  iobj_2->UpdateTime = 0.0;
  iobj_2->Time = 0.0;
  iobj_2->ObjectClassID = detection_ObjectClassID;
  iobj_2->TrackLogic = iobj_1;
  iobj_2->IsConfirmed = false;
  iobj_2->pUsedObjectAttributes[0] = false;
  iobj_2->pUsedObjectAttributes[1] = false;
  iobj_2->pObjectAttributes[0] = detection_ObjectAttributes;
  iobj_2->pObjectAttributes[1] = detection_ObjectAttributes;
  iobj_2->pUsedObjectAttributes[0] = true;
  AEBSensorFusion_ObjectTrack_nullify(iobj_2);
  return track;
}

int32_T ACCWithSensorFusionModelClass::AEBSensorFusion_xpotrf_ban(real_T A[9])
{
  int32_T b_j;
  int32_T info;
  boolean_T exitg1;
  info = 0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_j = 1;
  exitg1 = false;
  while ((!exitg1) && (b_j - 1 < 3)) {
    real_T c;
    real_T ssq;
    int32_T b;
    int32_T idxA1j;
    int32_T idxA1jp1;
    int32_T idxAjj;
    idxA1j = (b_j - 1) * 3 + 1;
    idxAjj = (b_j + idxA1j) - 2;
    ssq = 0.0;
    if (b_j - 1 >= 1) {
      b = static_cast<uint8_T>(b_j - 1);
      for (idxA1jp1 = 0; idxA1jp1 < b; idxA1jp1++) {
        c = A[(idxA1j + idxA1jp1) - 1];
        ssq += c * c;
      }
    }

    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (b_j < 3) {
        idxA1jp1 = idxA1j + 3;
        if (b_j - 1 != 0) {
          b = ((2 - b_j) * 3 + idxA1j) + 3;
          for (int32_T iac = idxA1jp1; iac <= b; iac += 3) {
            int32_T d;
            int32_T ia;
            c = 0.0;
            d = (b_j + iac) - 1;
            for (ia = iac; ia < d; ia++) {
              c += A[((idxA1j + ia) - iac) - 1] * A[ia - 1];
            }

            ia = (div_nde_s32_floor((iac - idxA1j) - 3, 3) * 3 + idxAjj) + 3;
            A[ia] -= c;
          }
        }

        ssq = 1.0 / ssq;
        b = (2 - b_j) * 3 + idxAjj;
        for (idxA1jp1 = idxAjj + 4; idxA1jp1 <= b + 4; idxA1jp1 += 3) {
          A[idxA1jp1 - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      A[idxAjj] = ssq;
      info = b_j;
      exitg1 = true;
    }
  }

  return info;
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xzlangeM_k(const real_T x
  [9])
{
  real_T y;
  int32_T b_k;
  boolean_T exitg1;
  y = 0.0;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 9)) {
    real_T absxk;
    absxk = std::abs(x[b_k]);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      b_k++;
    }
  }

  return y;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlascl_ivk(real_T cfrom,
  real_T cto, real_T A[9])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (int32_T b_j = 0; b_j < 3; b_j++) {
      int32_T offset;
      offset = b_j * 3 - 1;
      A[offset + 1] *= mul;
      A[offset + 2] *= mul;
      A[offset + 3] *= mul;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1g(int32_T n,
  const real_T x[9], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return scale * std::sqrt(y);
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xdotc_o(int32_T n, const
  real_T x[9], int32_T ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  d = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (n >= 1) {
    int32_T b;
    b = static_cast<uint8_T>(n);
    for (int32_T k = 0; k < b; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return d;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy_gop(int32_T n, real_T
  a, int32_T ix0, real_T y[9], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1g3(const real_T
  x[3], int32_T ix0)
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (int32_T k = ix0; k <= ix0 + 1; k++) {
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return scale * std::sqrt(y);
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy_goph(int32_T n, real_T
  a, const real_T x[9], int32_T ix0, real_T y[3], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xaxpy_gopha(int32_T n,
  real_T a, const real_T x[3], int32_T ix0, real_T y[9], int32_T iy0)
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzlascl_ivky(real_T cfrom,
  real_T cto, real_T A[3])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    A[0] *= mul;
    A[1] *= mul;
    A[2] *= mul;
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xrot_psj(real_T x[9],
  int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  real_T temp;
  real_T temp_tmp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = temp * c - temp_tmp * s;
  x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = x[ix0] * c + x[iy0] * s;
  x[iy0] = x[iy0] * c - x[ix0] * s;
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = temp * c - temp_tmp * s;
  x[ix0 + 1] = temp_tmp * c + temp * s;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xswap_dj5(real_T x[9],
  int32_T ix0, int32_T iy0)
{
  real_T temp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_svd_a(const real_T A[9],
  real_T U[9], real_T s[3], real_T V[9])
{
  real_T A_0[9];
  real_T e[3];
  real_T s_0[3];
  real_T work[3];
  real_T anrm;
  real_T cscale;
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  real_T ztest0;
  real_T ztest0_tmp_tmp;
  int32_T b;
  int32_T colqp1;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  boolean_T apply_transform;
  boolean_T doscale;
  boolean_T exitg1;
  s_0[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s_0[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s_0[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (b = 0; b < 9; b++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    A_0[b] = A[b];
    U[b] = 0.0;
    V[b] = 0.0;
  }

  doscale = false;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  anrm = AEBSensorFusion_xzlangeM_k(A);
  cscale = anrm;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    AEBSensorFusion_xzlascl_ivk(anrm, cscale, A_0);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    AEBSensorFusion_xzlascl_ivk(anrm, cscale, A_0);
  }

  for (m = 0; m < 2; m++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    qp1 = m + 2;
    colqp1 = 3 * m + m;
    qq = colqp1 + 1;
    apply_transform = false;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    nrm = AEBSensorFusion_xnrm2_po1g(3 - m, A_0, colqp1 + 1);
    if (nrm > 0.0) {
      apply_transform = true;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      if (A_0[colqp1] < 0.0) {
        ztest = -nrm;
        s_0[m] = -nrm;
      } else {
        ztest = nrm;
        s_0[m] = nrm;
      }

      if (std::abs(ztest) >= 1.0020841800044864E-292) {
        nrm = 1.0 / ztest;
        b = (colqp1 - m) + 1;
        for (qjj = qq; qjj <= b + 2; qjj++) {
          A_0[qjj - 1] *= nrm;
        }
      } else {
        b = (colqp1 - m) + 1;
        for (qjj = qq; qjj <= b + 2; qjj++) {
          A_0[qjj - 1] /= s_0[m];
        }
      }

      A_0[colqp1]++;
      s_0[m] = -s_0[m];
    } else {
      s_0[m] = 0.0;
    }

    for (b = qp1; b < 4; b++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      qjj = ((b - 1) * 3 + m) + 1;
      if (apply_transform) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        AEBSensorFusion_xaxpy_gop(3 - m, -(AEBSensorFusion_xdotc_o(3 - m, A_0,
          colqp1 + 1, A_0, qjj) / A_0[colqp1]), colqp1 + 1, A_0, qjj);
      }

      e[b - 1] = A_0[qjj - 1];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    for (qq = m + 1; qq < 4; qq++) {
      b = (3 * m + qq) - 1;
      U[b] = A_0[b];
    }

    if (m + 1 <= 1) {
      nrm = AEBSensorFusion_xnrm2_po1g3(e, m + 2);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[m + 1] < 0.0) {
          nrm = -nrm;
        }

        e[0] = nrm;
        if (std::abs(nrm) >= 1.0020841800044864E-292) {
          nrm = 1.0 / nrm;
          for (qjj = qp1; qjj <= m + 3; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          for (qjj = qp1; qjj <= m + 3; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[m + 1]++;
        e[0] = -e[0];
        for (qq = qp1; qq < 4; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 4; qq++) {
          AEBSensorFusion_xaxpy_goph(2 - m, e[qq - 1], A_0, (m + 3 * (qq - 1)) +
            2, work, m + 2);
        }

        for (qq = qp1; qq < 4; qq++) {
          AEBSensorFusion_xaxpy_gopha(2 - m, -e[qq - 1] / e[m + 1], work, m + 2,
            A_0, (m + 3 * (qq - 1)) + 2);
        }
      }

      for (colqp1 = qp1; colqp1 < 4; colqp1++) {
        V[colqp1 - 1] = e[colqp1 - 1];
      }
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  m = 2;
  s_0[2] = A_0[8];
  e[1] = A_0[7];
  e[2] = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (colqp1 = 1; colqp1 >= 0; colqp1--) {
    qq = 3 * colqp1 + colqp1;
    if (s_0[colqp1] != 0.0) {
      for (b = colqp1 + 2; b < 4; b++) {
        qjj = ((b - 1) * 3 + colqp1) + 1;
        AEBSensorFusion_xaxpy_gop(3 - colqp1, -(AEBSensorFusion_xdotc_o(3 -
          colqp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
      }

      for (qp1 = colqp1 + 1; qp1 < 4; qp1++) {
        b = (3 * colqp1 + qp1) - 1;
        U[b] = -U[b];
      }

      U[qq]++;
      if (colqp1 - 1 >= 0) {
        U[3 * colqp1] = 0.0;
      }
    } else {
      U[3 * colqp1] = 0.0;
      U[3 * colqp1 + 1] = 0.0;
      U[3 * colqp1 + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (colqp1 = 2; colqp1 >= 0; colqp1--) {
    if ((colqp1 + 1 <= 1) && (e[0] != 0.0)) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      AEBSensorFusion_xaxpy_gop(2, -(AEBSensorFusion_xdotc_o(2, V, 2, V, 5) / V
        [1]), 2, V, 5);
      AEBSensorFusion_xaxpy_gop(2, -(AEBSensorFusion_xdotc_o(2, V, 2, V, 8) / V
        [1]), 2, V, 8);
    }

    V[3 * colqp1] = 0.0;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    V[3 * colqp1 + 1] = 0.0;
    V[3 * colqp1 + 2] = 0.0;
    V[colqp1 + 3 * colqp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 3; qp1++) {
    ztest = s_0[qp1];
    if (ztest != 0.0) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      rt = std::abs(ztest);
      nrm = ztest / rt;
      s_0[qp1] = rt;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      if (qp1 + 1 < 3) {
        e[qp1] /= nrm;
      }

      qq = 3 * qp1;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      for (qjj = qq + 1; qjj <= qq + 3; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (qp1 + 1 < 3) {
      ztest = e[qp1];
      if (ztest != 0.0) {
        rt = std::abs(ztest);
        nrm = rt / ztest;
        e[qp1] = rt;
        s_0[qp1 + 1] *= nrm;
        colqp1 = (qp1 + 1) * 3;
        for (qjj = colqp1 + 1; qjj <= colqp1 + 3; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  rt = 0.0;
  nrm = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  ztest = std::abs(s_0[0]);
  ztest0 = std::abs(e[0]);
  if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
    ztest0 = ztest;
  }

  if ((!(ztest0 <= 0.0)) && (!rtIsNaN(ztest0))) {
    nrm = ztest0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  ztest = std::abs(s_0[1]);
  ztest0 = std::abs(e[1]);
  if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
    ztest0 = ztest;
  }

  if ((!(nrm >= ztest0)) && (!rtIsNaN(ztest0))) {
    nrm = ztest0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  ztest = std::abs(s_0[2]);
  ztest0 = std::abs(e[2]);
  if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
    ztest0 = ztest;
  }

  if ((!(nrm >= ztest0)) && (!rtIsNaN(ztest0))) {
    nrm = ztest0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  while ((m + 1 > 0) && (!(rt >= 75.0))) {
    colqp1 = m;
    qp1 = m;
    exitg1 = false;
    while ((!exitg1) && (qp1 > -1)) {
      colqp1 = qp1;
      if (qp1 == 0) {
        exitg1 = true;
      } else {
        ztest0 = std::abs(e[qp1 - 1]);
        if ((ztest0 <= (std::abs(s_0[qp1 - 1]) + std::abs(s_0[qp1])) *
             2.2204460492503131E-16) || ((ztest0 <= 1.0020841800044864E-292) ||
             ((rt > 20.0) && (ztest0 <= 2.2204460492503131E-16 * nrm)))) {
          e[qp1 - 1] = 0.0;
          exitg1 = true;
        } else {
          qp1--;
        }
      }
    }

    if (colqp1 == m) {
      ztest0 = 4.0;
    } else {
      qp1 = m + 1;
      qq = m + 1;
      exitg1 = false;
      while ((!exitg1) && (qq >= colqp1)) {
        qp1 = qq;
        if (qq == colqp1) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qq < m + 1) {
            ztest0 = std::abs(e[qq - 1]);
          }

          if (qq > colqp1 + 1) {
            ztest0 += std::abs(e[qq - 2]);
          }

          ztest = std::abs(s_0[qq - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s_0[qq - 1] = 0.0;
            exitg1 = true;
          } else {
            qq--;
          }
        }
      }

      if (qp1 == colqp1) {
        ztest0 = 3.0;
      } else if (m + 1 == qp1) {
        ztest0 = 1.0;
      } else {
        ztest0 = 2.0;
        colqp1 = qp1;
      }
    }

    switch (static_cast<int32_T>(ztest0)) {
     case 1:
      ztest0 = e[m - 1];
      e[m - 1] = 0.0;
      for (qq = m; qq >= colqp1 + 1; qq--) {
        AEBSensorFusion_xrotg(s_0[qq - 1], ztest0, &s_0[qq - 1], &ztest0, &ztest,
                              &sqds);
        if (qq > colqp1 + 1) {
          ztest0 = -sqds * e[0];
          e[0] *= ztest;
        }

        AEBSensorFusion_xrot_psj(V, 3 * (qq - 1) + 1, 3 * m + 1, ztest, sqds);
      }
      break;

     case 2:
      ztest0 = e[colqp1 - 1];
      e[colqp1 - 1] = 0.0;
      for (qp1 = colqp1 + 1; qp1 <= m + 1; qp1++) {
        AEBSensorFusion_xrotg(s_0[qp1 - 1], ztest0, &s_0[qp1 - 1], &ztest, &sqds,
                              &smm1);
        ztest = e[qp1 - 1];
        ztest0 = -smm1 * ztest;
        e[qp1 - 1] = ztest * sqds;
        AEBSensorFusion_xrot_psj(U, 3 * (qp1 - 1) + 1, 3 * (colqp1 - 1) + 1,
          sqds, smm1);
      }
      break;

     case 3:
      ztest = std::abs(s_0[m]);
      smm1 = s_0[m - 1];
      ztest0 = std::abs(smm1);
      if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
        ztest0 = ztest;
      }

      sqds = e[m - 1];
      ztest = std::abs(sqds);
      if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
        ztest = ztest0;
      }

      ztest0 = std::abs(s_0[colqp1]);
      if ((ztest >= ztest0) || rtIsNaN(ztest0)) {
        ztest0 = ztest;
      }

      ztest = std::abs(e[colqp1]);
      if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
        ztest = ztest0;
      }

      ztest0 = s_0[m] / ztest;
      smm1 /= ztest;
      emm1 = sqds / ztest;
      sqds = s_0[colqp1] / ztest;
      smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
      emm1 *= ztest0;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      ztest0 = (sqds + ztest0) * (sqds - ztest0) + shift;
      ztest = e[colqp1] / ztest * sqds;
      for (qq = colqp1 + 1; qq <= m; qq++) {
        AEBSensorFusion_xrotg(ztest0, ztest, &sqds, &smm1, &emm1, &shift);
        if (qq > colqp1 + 1) {
          e[0] = sqds;
        }

        ztest0_tmp_tmp = e[qq - 1];
        ztest0 = s_0[qq - 1];
        e[qq - 1] = ztest0_tmp_tmp * emm1 - ztest0 * shift;
        ztest = shift * s_0[qq];
        s_0[qq] *= emm1;
        b = (qq - 1) * 3 + 1;
        qjj = 3 * qq + 1;
        AEBSensorFusion_xrot_psj(V, b, qjj, emm1, shift);
        AEBSensorFusion_xrotg(ztest0 * emm1 + ztest0_tmp_tmp * shift, ztest,
                              &s_0[qq - 1], &sqds, &smm1, &emm1);
        ztest = e[qq - 1];
        ztest0 = ztest * smm1 + emm1 * s_0[qq];
        s_0[qq] = ztest * -emm1 + smm1 * s_0[qq];
        ztest = emm1 * e[qq];
        e[qq] *= smm1;
        AEBSensorFusion_xrot_psj(U, b, qjj, smm1, emm1);
      }

      e[m - 1] = ztest0;
      rt++;
      break;

     default:
      if (s_0[colqp1] < 0.0) {
        s_0[colqp1] = -s_0[colqp1];
        qq = 3 * colqp1;
        for (qjj = qq + 1; qjj <= qq + 3; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = colqp1 + 1;
      while ((colqp1 + 1 < 3) && (s_0[colqp1] < s_0[qp1])) {
        rt = s_0[colqp1];
        s_0[colqp1] = s_0[qp1];
        s_0[qp1] = rt;
        b = 3 * colqp1 + 1;
        qjj = (colqp1 + 1) * 3 + 1;
        AEBSensorFusion_xswap_dj5(V, b, qjj);
        AEBSensorFusion_xswap_dj5(U, b, qjj);
        colqp1 = qp1;
        qp1++;
      }

      rt = 0.0;
      m--;
      break;
    }
  }

  s[0] = s_0[0];
  s[1] = s_0[1];
  s[2] = s_0[2];
  if (doscale) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_xzlascl_ivky(cscale, anrm, s);
  }
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_ExtendedKalmanFilter_set_pProcessNoise
  (c_trackingEKF_AEBSensorFusion_T *obj, const real_T b_value[9])
{
  real_T Ss[9];
  real_T b_V[9];
  real_T s[3];
  real_T Ss_0;
  real_T Ss_1;
  real_T x;
  int32_T b_k;
  int32_T i;
  int32_T info;
  int32_T jmax;
  boolean_T p;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&Ss[0], &b_value[0], 9U * sizeof(real_T));
  info = AEBSensorFusion_xpotrf_ban(Ss);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (info == 0) {
    std::memcpy(&Ss[0], &b_value[0], 9U * sizeof(real_T));
    info = AEBSensorFusion_xpotrf_ban(Ss);
    if (info == 0) {
      jmax = 3;
    } else {
      jmax = info - 1;
    }

    info = jmax - 2;
    for (b_k = 0; b_k <= info; b_k++) {
      for (i = b_k + 2; i <= jmax; i++) {
        Ss[(i + 3 * b_k) - 1] = 0.0;
      }
    }

    for (b_k = 0; b_k < 3; b_k++) {
      obj->pSqrtProcessNoise[3 * b_k] = Ss[b_k];
      obj->pSqrtProcessNoise[3 * b_k + 1] = Ss[b_k + 3];
      obj->pSqrtProcessNoise[3 * b_k + 2] = Ss[b_k + 6];
    }
  } else {
    p = true;
    for (b_k = 0; b_k < 9; b_k++) {
      if (p) {
        x = b_value[b_k];
        if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
        } else {
          p = false;
        }
      } else {
        p = false;
      }
    }

    if (p) {
      AEBSensorFusion_svd_a(b_value, Ss, s, b_V);
    } else {
      s[0] = (rtNaN);
      s[1] = (rtNaN);
      s[2] = (rtNaN);
      for (b_k = 0; b_k < 9; b_k++) {
        b_V[b_k] = (rtNaN);
      }
    }

    std::memset(&Ss[0], 0, 9U * sizeof(real_T));
    Ss[0] = s[0];
    Ss[4] = s[1];
    Ss[8] = s[2];
    for (b_k = 0; b_k < 9; b_k++) {
      Ss[b_k] = std::sqrt(Ss[b_k]);
    }

    for (b_k = 0; b_k < 3; b_k++) {
      x = Ss[3 * b_k + 1];
      Ss_0 = Ss[3 * b_k];
      Ss_1 = Ss[3 * b_k + 2];
      for (i = 0; i < 3; i++) {
        obj->pSqrtProcessNoise[i + 3 * b_k] = (b_V[i + 3] * x + Ss_0 * b_V[i]) +
          b_V[i + 6] * Ss_1;
      }
    }
  }

  obj->pIsSetProcessNoise = true;
  obj->pSqrtProcessNoiseScalar = -1.0;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_trackingEKF_sync
  (c_trackingEKF_AEBSensorFusion_T *EKF, c_trackingEKF_AEBSensorFusion_T *EKF2)
{
  real_T a[36];
  real_T a_1[36];
  real_T valueSqrt[36];
  real_T a_0[9];
  real_T a_2[9];
  real_T b[9];
  real_T b_value[6];
  real_T a_3;
  real_T a_4;
  real_T b_value_0;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  for (i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_value[i] = EKF2->pState[i];
  }

  for (i = 0; i < 6; i++) {
    EKF->pState[i] = b_value[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((!EKF2->pIsSetStateCovariance) || (EKF2->pSqrtStateCovarianceScalar !=
       -1.0)) {
    b_value_0 = EKF2->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      EKF2->pSqrtStateCovariance[i] = b_value_0 * static_cast<real_T>(tmp[i]);
    }

    EKF2->pIsSetStateCovariance = true;
    EKF2->pSqrtStateCovarianceScalar = -1.0;
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    valueSqrt[i] = EKF2->pSqrtStateCovariance[i];
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    a[i] = EKF2->pSqrtStateCovariance[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      b_value_0 = 0.0;
      for (i_1 = 0; i_1 < 6; i_1++) {
        b_value_0 += a[6 * i_1 + i] * valueSqrt[6 * i_1 + i_0];
      }

      a_1[i + 6 * i_0] = b_value_0;
    }
  }

  AEBSensorFusion_cholPSD(a_1, EKF->pSqrtStateCovariance);
  EKF->pIsSetStateCovariance = true;
  EKF->pSqrtStateCovarianceScalar = -1.0;
  if (EKF2->pSqrtMeasurementNoiseScalar > 0.0) {
    b_value_0 = EKF2->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 36; i++) {
      EKF2->pSqrtMeasurementNoise[i] = b_value_0 * static_cast<real_T>(tmp[i]);
    }

    EKF2->pSqrtMeasurementNoiseScalar = -1.0;
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    valueSqrt[i] = EKF2->pSqrtMeasurementNoise[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  EKF->pSqrtMeasurementNoiseScalar = -1.0;
  if (EKF->pSqrtMeasurementNoiseScalar > 0.0) {
    b_value_0 = EKF->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 36; i++) {
      EKF->pSqrtMeasurementNoise[i] = b_value_0 * static_cast<real_T>(tmp[i]);
    }

    EKF->pSqrtMeasurementNoiseScalar = -1.0;
  }

  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      b_value_0 = 0.0;
      for (i_1 = 0; i_1 < 6; i_1++) {
        b_value_0 += valueSqrt[6 * i_1 + i] * valueSqrt[6 * i_1 + i_0];
      }

      a[i + 6 * i_0] = b_value_0;
    }
  }

  AEBSensorFusion_cholPSD(a, EKF->pSqrtMeasurementNoise);
  if ((!EKF2->pIsSetProcessNoise) || (EKF2->pSqrtProcessNoiseScalar != -1.0)) {
    b_value_0 = EKF2->pSqrtProcessNoiseScalar;
    for (i = 0; i < 9; i++) {
      EKF2->pSqrtProcessNoise[i] = b_value_0 * static_cast<real_T>(tmp_0[i]);
    }

    EKF2->pIsSetProcessNoise = true;
    EKF2->pSqrtProcessNoiseScalar = -1.0;
  }

  for (i = 0; i < 9; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b[i] = EKF2->pSqrtProcessNoise[i];
  }

  for (i = 0; i < 9; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    a_0[i] = EKF2->pSqrtProcessNoise[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (i = 0; i < 3; i++) {
    b_value_0 = a_0[i + 3];
    a_3 = a_0[i];
    a_4 = a_0[i + 6];
    for (i_0 = 0; i_0 < 3; i_0++) {
      a_2[i + 3 * i_0] = (b[i_0 + 3] * b_value_0 + a_3 * b[i_0]) + b[i_0 + 6] *
        a_4;
    }
  }

  AEBSensorFusion_ExtendedKalmanFilter_set_pProcessNoise(EKF, a_2);
  EKF->pIsFirstCallCorrect = EKF2->pIsFirstCallCorrect;
  EKF->pIsFirstCallPredict = EKF2->pIsFirstCallPredict;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  EKF->pWasRetrodicted = EKF2->pWasRetrodicted;
}

b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T
  *ACCWithSensorFusionModelClass::AEBSensorFusion_ObjectTrack_copy
  (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track,
   b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *iobj_0,
   c_trackHistoryLogic_AEBSensorFusion_T *iobj_1,
   c_trackingEKF_AEBSensorFusion_T *iobj_2)
{
  BusRadarDetectionsObjectAttributes c;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *newTrack;
  c_trackHistoryLogic_AEBSensorFusion_T *obj;
  c_trackingEKF_AEBSensorFusion_T *filter;
  real_T varargin_10;
  real_T varargin_6;
  int32_T i;
  uint32_T varargin_2;
  uint32_T varargin_4;
  boolean_T track_idx_1;
  boolean_T varargin_22;
  filter = AEBSensorFusion_trackingEKF_clone(track->Filter, &iobj_2[0]);
  obj = track->TrackLogic;
  for (i = 0; i < 50; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    iobj_1->pRecentHistory[i] = false;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  *iobj_1 = *obj;
  c = track->pObjectAttributes[0];
  varargin_2 = track->BranchID;
  varargin_4 = track->TrackID;
  varargin_6 = track->Time;
  varargin_10 = track->ObjectClassID;
  varargin_22 = track->IsConfirmed;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  newTrack = iobj_0;
  iobj_0->BranchID = varargin_2;
  iobj_0->TrackID = varargin_4;
  iobj_0->Filter = filter;
  iobj_0->pDistanceFilter = AEBSensorFusion_trackingEKF_clone(filter, &iobj_2[1]);
  iobj_0->UpdateTime = varargin_6;
  iobj_0->Time = varargin_6;
  iobj_0->ObjectClassID = varargin_10;
  iobj_0->TrackLogic = iobj_1;
  iobj_0->IsConfirmed = varargin_22;
  iobj_0->pUsedObjectAttributes[0] = false;
  iobj_0->pUsedObjectAttributes[1] = false;
  iobj_0->pObjectAttributes[0] = c;
  iobj_0->pObjectAttributes[1] = c;
  iobj_0->pUsedObjectAttributes[0] = true;
  iobj_0->pObjectAttributes[0] = track->pObjectAttributes[0];
  iobj_0->pObjectAttributes[1] = track->pObjectAttributes[1];
  iobj_0->UpdateTime = track->UpdateTime;
  iobj_0->pIsCoasted = track->pIsCoasted;
  varargin_22 = track->pUsedObjectAttributes[0];
  track_idx_1 = track->pUsedObjectAttributes[1];
  iobj_0->pUsedObjectAttributes[0] = varargin_22;
  iobj_0->pUsedObjectAttributes[1] = track_idx_1;
  iobj_0->pAge = track->pAge;
  AEBSensorFusion_trackingEKF_sync(iobj_0->Filter, track->Filter);
  AEBSensorFusion_trackingEKF_sync(iobj_0->pDistanceFilter,
    track->pDistanceFilter);
  return newTrack;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_ObjectTrack_trackToStruct
  (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track,
   sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T *trackStruct)
{
  c_trackHistoryLogic_AEBSensorFusion_T *obj;
  c_trackingEKF_AEBSensorFusion_T *obj_0;
  real_T a[36];
  real_T b[36];
  real_T b_value;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  trackStruct->TrackID = track->TrackID;
  trackStruct->BranchID = track->BranchID;
  trackStruct->UpdateTime = track->Time;
  trackStruct->Age = track->pAge;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj_0 = track->Filter;
  for (int32_T i = 0; i < 6; i++) {
    trackStruct->State[i] = obj_0->pState[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj_0 = track->Filter;
  if ((!obj_0->pIsSetStateCovariance) || (obj_0->pSqrtStateCovarianceScalar !=
       -1.0)) {
    b_value = obj_0->pSqrtStateCovarianceScalar;
    for (int32_T i = 0; i < 36; i++) {
      obj_0->pSqrtStateCovariance[i] = b_value * static_cast<real_T>(tmp[i]);
    }

    obj_0->pIsSetStateCovariance = true;
    obj_0->pSqrtStateCovarianceScalar = -1.0;
  }

  for (int32_T i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b[i] = obj_0->pSqrtStateCovariance[i];
  }

  for (int32_T i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    a[i] = obj_0->pSqrtStateCovariance[i];
  }

  trackStruct->ObjectClassID = track->ObjectClassID;
  obj = track->TrackLogic;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  trackStruct->TrackLogicState[0] = obj->pRecentHistory[0];
  trackStruct->TrackLogicState[1] = obj->pRecentHistory[1];
  trackStruct->TrackLogicState[2] = obj->pRecentHistory[2];
  trackStruct->IsConfirmed = track->IsConfirmed;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  trackStruct->IsCoasted = track->pIsCoasted;
  trackStruct->SourceIndex = 0U;
  for (int32_T i = 0; i < 6; i++) {
    for (int32_T i_0 = 0; i_0 < 6; i_0++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b_value = 0.0;
      for (int32_T i_1 = 0; i_1 < 6; i_1++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        b_value += a[6 * i_1 + i] * b[6 * i_1 + i_0];
      }

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      trackStruct->StateCovariance[i + 6 * i_0] = b_value;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  trackStruct->ObjectClassProbabilities = 1.0;
  trackStruct->TrackLogic = trackLogicType_History;
  trackStruct->IsSelfReported = true;
  trackStruct->ObjectAttributes[0] = track->pObjectAttributes[0];
  trackStruct->ObjectAttributes[1] = track->pObjectAttributes[1];
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_SystemCore_setup
  (multiObjectTracker_AEBSensorFusion_T *obj, real_T varargin_1_NumDetections,
   const BusDetectionConcatenation1Detections varargin_1_Detections[70])
{
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *tracks[20];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *tracks_0[20];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T track;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T track_0;
  c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T *obj_0;
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj_1;
  c_trackHistoryLogic_AEBSensorFusion_T lobj_1_0;
  c_trackHistoryLogic_AEBSensorFusion_T lobj_2;
  c_trackingEKF_AEBSensorFusion_T lobj_1[3];
  sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T unusedExpr;
  real_T varargin_2;
  real_T varargin_6_idx_1;
  int32_T b_i;
  obj->isSetupComplete = false;
  obj->isInitialized = 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->_pobj3.isInitialized = 0;
  obj->cDetectionManager = &obj->_pobj3;
  obj_1 = obj->cDetectionManager;
  obj_1->isInitialized = 1;
  for (b_i = 0; b_i < 70; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj_1->pDetections[b_i] = varargin_1_Detections[0];
    obj_1->pOriginatingSensor[b_i] = 0.0;
    obj_1->pIsOOSM[b_i] = false;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj_1->pNumDetections = varargin_1_NumDetections;
  obj->pSampleDetection = varargin_1_Detections[0];
  obj->pLastTimeStamp = -2.2204460492503131E-16;
  AEBSensorFusion_initcvekf(obj->pSampleDetection.Measurement,
    obj->pSampleDetection.MeasurementNoise,
    obj->pSampleDetection.MeasurementParameters.Frame,
    obj->pSampleDetection.MeasurementParameters.OriginPosition,
    obj->pSampleDetection.MeasurementParameters.Orientation,
    obj->pSampleDetection.MeasurementParameters.HasVelocity,
    obj->pSampleDetection.MeasurementParameters.OriginVelocity,
    obj->pSampleDetection.MeasurementParameters.IsParentToChild,
    obj->pSampleDetection.MeasurementParameters.HasElevation,
    &AEBSensorFusion_DW.lobj_2[0]);
  AEBSensorFusion_TrackManager_createSampleTrack(obj,
    &AEBSensorFusion_DW.lobj_2[2], &lobj_1_0, &track_0);
  tracks[0] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[0],
    &obj->_pobj1[0], &obj->_pobj0[0]);
  tracks[1] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[1],
    &obj->_pobj1[1], &obj->_pobj0[2]);
  tracks[2] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[2],
    &obj->_pobj1[2], &obj->_pobj0[4]);
  tracks[3] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[3],
    &obj->_pobj1[3], &obj->_pobj0[6]);
  tracks[4] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[4],
    &obj->_pobj1[4], &obj->_pobj0[8]);
  tracks[5] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[5],
    &obj->_pobj1[5], &obj->_pobj0[10]);
  tracks[6] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[6],
    &obj->_pobj1[6], &obj->_pobj0[12]);
  tracks[7] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[7],
    &obj->_pobj1[7], &obj->_pobj0[14]);
  tracks[8] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[8],
    &obj->_pobj1[8], &obj->_pobj0[16]);
  tracks[9] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[9],
    &obj->_pobj1[9], &obj->_pobj0[18]);
  tracks[10] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[10],
    &obj->_pobj1[10], &obj->_pobj0[20]);
  tracks[11] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[11],
    &obj->_pobj1[11], &obj->_pobj0[22]);
  tracks[12] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[12],
    &obj->_pobj1[12], &obj->_pobj0[24]);
  tracks[13] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[13],
    &obj->_pobj1[13], &obj->_pobj0[26]);
  tracks[14] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[14],
    &obj->_pobj1[14], &obj->_pobj0[28]);
  tracks[15] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[15],
    &obj->_pobj1[15], &obj->_pobj0[30]);
  tracks[16] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[16],
    &obj->_pobj1[16], &obj->_pobj0[32]);
  tracks[17] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[17],
    &obj->_pobj1[17], &obj->_pobj0[34]);
  tracks[18] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[18],
    &obj->_pobj1[18], &obj->_pobj0[36]);
  tracks[19] = AEBSensorFusion_ObjectTrack_copy(&track_0, &obj->_pobj2[19],
    &obj->_pobj1[19], &obj->_pobj0[38]);
  for (b_i = 0; b_i < 20; b_i++) {
    tracks_0[b_i] = tracks[b_i];
  }

  for (b_i = 0; b_i < 20; b_i++) {
    tracks[b_i] = tracks_0[b_i];
  }

  for (b_i = 0; b_i < 20; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pTracksList[b_i] = tracks[b_i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->pDistFilter = AEBSensorFusion_trackingEKF_clone(obj->pTracksList[0]
    ->Filter, &obj->_pobj0[40]);
  obj->pNumLiveTracks = 0;
  obj->pUsedSensors[0] = 0U;
  obj->pUsedSensors[1] = 0U;
  for (b_i = 0; b_i < 20; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pTrackIDs[b_i] = 0U;
  }

  for (b_i = 0; b_i < 20; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pConfirmedTracks[b_i] = false;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  varargin_2 = obj->AssignmentThreshold[1];
  obj->cCostCalculator.isInitialized = 0;
  obj->cCostCalculator.MaxAssignmentCost = varargin_2;
  varargin_2 = obj->AssignmentThreshold[0];
  varargin_6_idx_1 = obj->AssignmentThreshold[1];
  obj->_pobj4.isInitialized = 0;
  obj->_pobj4.AssignmentThreshold[0] = varargin_2;
  obj->_pobj4.AssignmentThreshold[1] = varargin_6_idx_1;
  obj->cAssigner = &obj->_pobj4;
  obj_0 = obj->cAssigner;
  obj_0->isSetupComplete = false;
  obj_0->isInitialized = 1;
  obj_0->isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_TrackManager_createSampleTrack(obj, &lobj_1[0], &lobj_2,
    &track);
  AEBSensorFusion_ObjectTrack_trackToStruct(&track, &unusedExpr);
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_GNNTracker_resetImpl
  (multiObjectTracker_AEBSensorFusion_T *obj)
{
  c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T *obj_0;
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj_1;
  int32_T i;
  for (i = 0; i < 20; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_ObjectTrack_nullify(obj->pTracksList[i]);
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->pNumLiveTracks = 0;
  for (i = 0; i < 20; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pTrackIDs[i] = 0U;
  }

  for (i = 0; i < 20; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pConfirmedTracks[i] = false;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->pUsedSensors[0] = 0U;
  obj->pUsedSensors[1] = 0U;
  obj_1 = obj->cDetectionManager;
  if (obj_1->isInitialized == 1) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj_1->pNumDetections = 0.0;
    for (i = 0; i < 70; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      obj_1->pOriginatingSensor[i] = 0.0;
      obj_1->pIsOOSM[i] = false;
    }
  }

  obj->pLastTimeStamp = -0.05;
  obj->pLastTrackID = 0U;
  for (i = 0; i < 20; i++) {
    obj->pWasDetectable[i] = true;
  }

  obj->pCostOfNonAssignment = obj->AssignmentThreshold[0] / 2.0;
  obj_0 = obj->cAssigner;
  if (obj_0->isInitialized == 1) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj_0->pCostOfNonAssignment = obj_0->AssignmentThreshold[0] / 2.0;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_merge(int32_T idx_data[],
  uint32_T x_data[], int32_T offset, int32_T np, int32_T nq, int32_T iwork_data[],
  uint32_T xwork_data[])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (nq != 0) {
    int32_T iout;
    int32_T n;
    int32_T q;
    int32_T qend;
    qend = np + nq;
    for (q = 0; q < qend; q++) {
      iout = offset + q;
      iwork_data[q] = idx_data[iout];
      xwork_data[q] = x_data[iout];
    }

    n = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[n] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[n];
        x_data[iout] = xwork_data[n];
        if (n + 1 < np) {
          n++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < qend) {
          q++;
        } else {
          qend = iout - n;
          for (q = n + 1; q <= np; q++) {
            iout = qend + q;
            idx_data[iout] = iwork_data[q - 1];
            x_data[iout] = xwork_data[q - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_sort(const uint32_T x_data[],
  const int32_T x_size[2], uint32_T b_x_data[], int32_T b_x_size[2])
{
  int32_T c_idx_data[70];
  int32_T c_iwork_data[70];
  int32_T bLen;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T loop_ub;
  int32_T nQuartets_tmp;
  int32_T perm_0;
  int32_T perm_1;
  uint32_T xwork_data[70];
  uint32_T x4[4];
  uint32_T b_x;
  uint32_T b_x_0;
  uint32_T b_x_1;
  uint32_T b_x_2;
  int8_T perm[4];
  uint8_T idx4[4];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_x_size[0] = 1;
  loop_ub = x_size[1];
  b_x_size[1] = x_size[1];
  if (loop_ub - 1 >= 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    std::memcpy(&b_x_data[0], &x_data[0], static_cast<uint32_T>(loop_ub) *
                sizeof(uint32_T));
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (x_size[1] != 0) {
    std::memset(&c_idx_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof
                (int32_T));
    x4[0] = 0U;
    idx4[0] = 0U;
    x4[1] = 0U;
    idx4[1] = 0U;
    x4[2] = 0U;
    idx4[2] = 0U;
    x4[3] = 0U;
    idx4[3] = 0U;
    nQuartets_tmp = x_size[1] >> 2;
    for (bLen = 0; bLen < nQuartets_tmp; bLen++) {
      i = (bLen << 2) + 1;
      idx4[0] = static_cast<uint8_T>(i);
      idx4[1] = static_cast<uint8_T>(i + 1);
      idx4[2] = static_cast<uint8_T>(i + 2);
      idx4[3] = static_cast<uint8_T>(i + 3);
      b_x = b_x_data[i - 1];
      x4[0] = b_x;
      b_x_0 = b_x_data[i];
      x4[1] = b_x_0;
      b_x_1 = b_x_data[i + 1];
      x4[2] = b_x_1;
      b_x_2 = b_x_data[i + 2];
      x4[3] = b_x_2;
      if (b_x <= b_x_0) {
        i1 = 1;
        i2 = 2;
      } else {
        i1 = 2;
        i2 = 1;
      }

      if (b_x_1 <= b_x_2) {
        i3 = 3;
        i4 = 4;
      } else {
        i3 = 4;
        i4 = 3;
      }

      b_x = x4[i1 - 1];
      b_x_0 = x4[i3 - 1];
      if (b_x <= b_x_0) {
        b_x = x4[i2 - 1];
        if (b_x <= b_x_0) {
          perm_0 = i1;
          perm_1 = i2;
          i1 = i3;
          i2 = i4;
        } else if (b_x <= x4[i4 - 1]) {
          perm_0 = i1;
          perm_1 = i3;
          i1 = i2;
          i2 = i4;
        } else {
          perm_0 = i1;
          perm_1 = i3;
          i1 = i4;
        }
      } else {
        b_x_0 = x4[i4 - 1];
        if (b_x <= b_x_0) {
          if (x4[i2 - 1] <= b_x_0) {
            perm_0 = i3;
            perm_1 = i1;
            i1 = i2;
            i2 = i4;
          } else {
            perm_0 = i3;
            perm_1 = i1;
            i1 = i4;
          }
        } else {
          perm_0 = i3;
          perm_1 = i4;
        }
      }

      c_idx_data[i - 1] = idx4[perm_0 - 1];
      c_idx_data[i] = idx4[perm_1 - 1];
      c_idx_data[i + 1] = idx4[i1 - 1];
      c_idx_data[i + 2] = idx4[i2 - 1];
      b_x_data[i - 1] = x4[perm_0 - 1];
      b_x_data[i] = x4[perm_1 - 1];
      b_x_data[i + 1] = x4[i1 - 1];
      b_x_data[i + 2] = x4[i2 - 1];
    }

    i1 = (nQuartets_tmp << 2) - 1;
    i = (x_size[1] - i1) - 1;
    if (i > 0) {
      for (i2 = 0; i2 < i; i2++) {
        perm_0 = i1 + i2;
        idx4[i2] = static_cast<uint8_T>(perm_0 + 2);
        x4[i2] = b_x_data[perm_0 + 1];
      }

      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (i == 1) {
        perm[0] = 1;
      } else if (i == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }

      i2 = i;
      for (i = 0; i < i2; i++) {
        perm_0 = perm[i];
        bLen = (i1 + i) + 1;
        c_idx_data[bLen] = idx4[perm_0 - 1];
        b_x_data[bLen] = x4[perm_0 - 1];
      }
    }

    if (x_size[1] > 1) {
      std::memset(&c_iwork_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof
                  (int32_T));
      i = nQuartets_tmp;
      bLen = 4;
      while (i > 1) {
        if ((static_cast<uint32_T>(i) & 1U) != 0U) {
          i--;
          i1 = bLen * i;
          i2 = x_size[1] - i1;
          if (i2 > bLen) {
            AEBSensorFusion_merge(c_idx_data, b_x_data, i1, bLen, i2 - bLen,
                                  c_iwork_data, xwork_data);
          }
        }

        i1 = bLen << 1;
        i >>= 1;
        for (i2 = 0; i2 < i; i2++) {
          AEBSensorFusion_merge(c_idx_data, b_x_data, i2 * i1, bLen, bLen,
                                c_iwork_data, xwork_data);
        }

        bLen = i1;
      }

      if (x_size[1] > bLen) {
        AEBSensorFusion_merge(c_idx_data, b_x_data, 0, bLen, x_size[1] - bLen,
                              c_iwork_data, xwork_data);
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_sort_m(const uint32_T x[2],
  uint32_T b_x[2])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_x[0] = x[0];
  b_x[1] = x[1];
  if (x[0] > x[1]) {
    b_x[0] = x[1];
    b_x[1] = x[0];
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_do_vectors(const uint32_T
  a_data[], const int32_T a_size[2], const uint32_T b[2], uint32_T c_data[],
  int32_T c_size[2], int32_T ia_data[], int32_T ia_size[1], int32_T ib_size[1])
{
  int32_T iafirst;
  int32_T ialast;
  int32_T iblast;
  int32_T na_tmp;
  int32_T nc;
  int32_T nia;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  na_tmp = a_size[1];
  c_size[0] = 1;
  c_size[1] = a_size[1];
  ia_size[0] = a_size[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  ib_size[0] = 0;
  nc = -1;
  nia = -1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iafirst = 0;
  ialast = 0;
  iblast = 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  while ((ialast + 1 <= na_tmp) && (iblast <= 2)) {
    int32_T b_ialast;
    uint32_T ak;
    uint32_T bk;
    b_ialast = ialast + 1;
    ak = a_data[ialast];
    while ((b_ialast < a_size[1]) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast - 1;
    bk = b[iblast - 1];
    while ((iblast < 2) && (b[1] == bk)) {
      iblast = 2;
    }

    if (ak == bk) {
      ialast = b_ialast;
      iafirst = b_ialast;
      iblast++;
    } else if (ak < bk) {
      ialast = nc + 1;
      nia = nc + 1;
      nc++;
      c_data[ialast] = ak;
      ia_data[ialast] = iafirst + 1;
      ialast = b_ialast;
      iafirst = b_ialast;
    } else {
      iblast++;
    }
  }

  while (ialast + 1 <= na_tmp) {
    iblast = ialast + 1;
    while ((iblast < a_size[1]) && (a_data[iblast] == a_data[ialast])) {
      iblast++;
    }

    nia = nc + 1;
    nc++;
    c_data[nia] = a_data[ialast];
    ia_data[nia] = iafirst + 1;
    ialast = iblast;
    iafirst = iblast;
  }

  if (a_size[1] > 0) {
    if (nia + 1 < 1) {
      ia_size[0] = 0;
    } else {
      ia_size[0] = nia + 1;
    }

    if (nc + 1 < 1) {
      c_size[1] = 0;
    } else {
      c_size[1] = nc + 1;
    }
  }
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_TrackManager_getLiveTrackIndices(const
  multiObjectTracker_AEBSensorFusion_T *obj, int32_T indices_data[], int32_T
  indices_size[2])
{
  int32_T n;
  if (obj->pNumLiveTracks < 1) {
    n = 0;
  } else {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    n = static_cast<uint8_T>(obj->pNumLiveTracks - 1) + 1;
  }

  indices_size[0] = 1;
  indices_size[1] = n;
  if (n > 0) {
    int32_T yk;
    indices_data[0] = 1;
    yk = 1;
    for (int32_T k = 2; k <= n; k++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      yk++;
      indices_data[k - 1] = yk;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_unique_vector(const real_T
  a_data[], const int32_T a_size[2], real_T b_data[], int32_T b_size[2])
{
  real_T x;
  int32_T idx_data[70];
  int32_T iwork_data[70];
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T k;
  int32_T loop_ub;
  int32_T n;
  int32_T qEnd;
  boolean_T exitg1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  n = a_size[1] + 1;
  loop_ub = a_size[1];
  if (loop_ub - 1 >= 0) {
    std::memset(&idx_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(int32_T));
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (a_size[1] != 0) {
    std::memset(&idx_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(int32_T));
    for (i = 1; i <= loop_ub - 1; i += 2) {
      x = a_data[i];
      if ((a_data[i - 1] <= x) || rtIsNaN(x)) {
        idx_data[i - 1] = i;
        idx_data[i] = i + 1;
      } else {
        idx_data[i - 1] = i + 1;
        idx_data[i] = i;
      }
    }

    if ((static_cast<uint32_T>(a_size[1]) & 1U) != 0U) {
      idx_data[a_size[1] - 1] = a_size[1];
    }

    i = 2;
    while (i < n - 1) {
      int32_T pEnd;
      i2 = i << 1;
      j = 1;
      pEnd = i + 1;
      while (pEnd < n) {
        int32_T kEnd;
        int32_T p;
        int32_T q;
        p = j - 1;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          int32_T tmp;
          int32_T x_tmp;
          x_tmp = idx_data[q];
          x = a_data[x_tmp - 1];
          tmp = idx_data[p];
          if ((a_data[tmp - 1] <= x) || rtIsNaN(x)) {
            iwork_data[k] = tmp;
            p++;
            if (p + 1 == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = x_tmp;
            q++;
            if (q + 1 == qEnd) {
              while (p + 1 < pEnd) {
                k++;
                iwork_data[k] = idx_data[p];
                p++;
              }
            }
          }

          k++;
        }

        for (pEnd = 0; pEnd < kEnd; pEnd++) {
          idx_data[(j + pEnd) - 1] = iwork_data[pEnd];
        }

        j = qEnd;
        pEnd = qEnd + i;
      }

      i = i2;
    }
  }

  b_size[0] = 1;
  for (i = 0; i < loop_ub; i++) {
    b_data[i] = a_data[idx_data[i] - 1];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j + 1 <= a_size[1])) {
    x = b_data[j];
    if (rtIsInf(x) && (x < 0.0)) {
      j++;
    } else {
      exitg1 = true;
    }
  }

  k = j;
  j = a_size[1];
  while ((j >= 1) && rtIsNaN(b_data[j - 1])) {
    j--;
  }

  i = a_size[1] - j;
  exitg1 = false;
  while ((!exitg1) && (j >= 1)) {
    x = b_data[j - 1];
    if (rtIsInf(x) && (x > 0.0)) {
      j--;
    } else {
      exitg1 = true;
    }
  }

  qEnd = (a_size[1] - j) - i;
  i2 = -1;
  if (k > 0) {
    i2 = 0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  while (k + 1 <= j) {
    x = b_data[k];
    do {
      k++;
    } while (!((k + 1 > j) || (b_data[k] != x)));

    i2++;
    b_data[i2] = x;
  }

  if (qEnd > 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    i2++;
    b_data[i2] = b_data[j];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  k = j + qEnd;
  for (j = 0; j < i; j++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_data[(i2 + j) + 1] = b_data[k + j];
  }

  i2 = i - 1 < 0 ? i2 + 1 : (i2 + i) + 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (i2 < 1) {
    b_size[1] = 0;
  } else {
    b_size[1] = i2;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_1(boolean_T
  in1_data[], int32_T in1_size[2], const real_T in2_data[], const int32_T
  in2_size[2], real_T in3, const boolean_T in4_data[], const int32_T in4_size[2])
{
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  in1_size[0] = 1;
  loop_ub = in4_size[1] == 1 ? in2_size[1] : in4_size[1];
  in1_size[1] = loop_ub;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in4_size[1] != 1);
  for (int32_T i = 0; i < loop_ub; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    in1_data[i] = ((in2_data[i * stride_0_1] == in3) && in4_data[i * stride_1_1]);
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_GNNTracker_selectDetections(
  const real_T origSen_data[], const int32_T origSen_size[2], real_T sensorID,
  const boolean_T insequence_data[], const int32_T insequence_size[2], int32_T
  idx_data[], int32_T idx_size[2])
{
  int32_T b_size[2];
  int32_T b;
  int32_T b_ii;
  int32_T idx;
  boolean_T b_data[70];
  boolean_T exitg1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (origSen_size[1] == insequence_size[1]) {
    idx = origSen_size[1];
    b_size[1] = origSen_size[1];
    for (b = 0; b < idx; b++) {
      b_data[b] = ((origSen_data[b] == sensorID) && insequence_data[b]);
    }
  } else {
    AEBSensorFusion_binary_expand_op_1(b_data, b_size, origSen_data,
      origSen_size, sensorID, insequence_data, insequence_size);
  }

  idx = 0;
  idx_size[0] = 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  idx_size[1] = b_size[1];
  b = b_size[1] - 1;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 <= b)) {
    if (b_data[b_ii - 1]) {
      idx++;
      idx_data[idx - 1] = b_ii;
      if (idx >= b_size[1]) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (b_size[1] == 1) {
    if (idx == 0) {
      idx_size[0] = 1;
      idx_size[1] = 0;
    }
  } else if (idx < 1) {
    idx_size[1] = 0;
  } else {
    idx_size[1] = idx;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_merge_e(int32_T idx_data[],
  real_T x_data[], int32_T offset, int32_T np, int32_T nq, int32_T iwork_data[],
  real_T xwork_data[])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (nq != 0) {
    int32_T iout;
    int32_T n;
    int32_T q;
    int32_T qend;
    qend = np + nq;
    for (q = 0; q < qend; q++) {
      iout = offset + q;
      iwork_data[q] = idx_data[iout];
      xwork_data[q] = x_data[iout];
    }

    n = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[n] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[n];
        x_data[iout] = xwork_data[n];
        if (n + 1 < np) {
          n++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < qend) {
          q++;
        } else {
          qend = iout - n;
          for (q = n + 1; q <= np; q++) {
            iout = qend + q;
            idx_data[iout] = iwork_data[q - 1];
            x_data[iout] = xwork_data[q - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_sort_mh(const real_T x_data[],
  const int32_T x_size[2], real_T b_x_data[], int32_T b_x_size[2], int32_T
  idx_data[], int32_T idx_size[2])
{
  real_T xwork_data[70];
  real_T x4[4];
  real_T tmp;
  real_T tmp_0;
  int32_T c_iwork_data[70];
  int32_T bLen;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T ib;
  int32_T loop_ub;
  int32_T perm_0;
  int32_T perm_1;
  int32_T quartetOffset;
  int32_T wOffset;
  int8_T idx4[4];
  int8_T perm[4];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_x_size[0] = 1;
  loop_ub = x_size[1];
  b_x_size[1] = x_size[1];
  idx_size[0] = 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  idx_size[1] = x_size[1];
  for (bLen = 0; bLen < loop_ub; bLen++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_x_data[bLen] = x_data[bLen];
    idx_data[bLen] = 0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (x_size[1] != 0) {
    idx_size[0] = 1;
    idx_size[1] = x_size[1];
    std::memset(&idx_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(int32_T));
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    bLen = 0;
    ib = 0;
    for (wOffset = 0; wOffset < loop_ub; wOffset++) {
      tmp_0 = b_x_data[wOffset];
      if (rtIsNaN(tmp_0)) {
        quartetOffset = (loop_ub - bLen) - 1;
        idx_data[quartetOffset] = wOffset + 1;
        xwork_data[quartetOffset] = tmp_0;
        bLen++;
      } else {
        ib++;
        idx4[ib - 1] = static_cast<int8_T>(wOffset + 1);
        x4[ib - 1] = tmp_0;
        if (ib == 4) {
          quartetOffset = wOffset - bLen;
          if (x4[0] <= x4[1]) {
            ib = 1;
            i2 = 2;
          } else {
            ib = 2;
            i2 = 1;
          }

          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }

          tmp_0 = x4[ib - 1];
          tmp = x4[i3 - 1];
          if (tmp_0 <= tmp) {
            tmp_0 = x4[i2 - 1];
            if (tmp_0 <= tmp) {
              perm_0 = ib;
              perm_1 = i2;
              ib = i3;
              i2 = i4;
            } else if (tmp_0 <= x4[i4 - 1]) {
              perm_0 = ib;
              perm_1 = i3;
              ib = i2;
              i2 = i4;
            } else {
              perm_0 = ib;
              perm_1 = i3;
              ib = i4;
            }
          } else {
            tmp = x4[i4 - 1];
            if (tmp_0 <= tmp) {
              if (x4[i2 - 1] <= tmp) {
                perm_0 = i3;
                perm_1 = ib;
                ib = i2;
                i2 = i4;
              } else {
                perm_0 = i3;
                perm_1 = ib;
                ib = i4;
              }
            } else {
              perm_0 = i3;
              perm_1 = i4;
            }
          }

          idx_data[quartetOffset - 3] = idx4[perm_0 - 1];
          idx_data[quartetOffset - 2] = idx4[perm_1 - 1];
          idx_data[quartetOffset - 1] = idx4[ib - 1];
          idx_data[quartetOffset] = idx4[i2 - 1];
          b_x_data[quartetOffset - 3] = x4[perm_0 - 1];
          b_x_data[quartetOffset - 2] = x4[perm_1 - 1];
          b_x_data[quartetOffset - 1] = x4[ib - 1];
          b_x_data[quartetOffset] = x4[i2 - 1];
          ib = 0;
        }
      }
    }

    i3 = x_size[1] - bLen;
    if (ib > 0) {
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }

      i4 = static_cast<uint8_T>(ib);
      for (i2 = 0; i2 < i4; i2++) {
        perm_0 = perm[i2];
        wOffset = (i3 - ib) + i2;
        idx_data[wOffset] = idx4[perm_0 - 1];
        b_x_data[wOffset] = x4[perm_0 - 1];
      }
    }

    i4 = bLen >> 1;
    for (ib = 0; ib < i4; ib++) {
      wOffset = ib + i3;
      i2 = idx_data[wOffset];
      quartetOffset = (loop_ub - ib) - 1;
      idx_data[wOffset] = idx_data[quartetOffset];
      idx_data[quartetOffset] = i2;
      b_x_data[wOffset] = xwork_data[quartetOffset];
      b_x_data[quartetOffset] = xwork_data[wOffset];
    }

    if ((static_cast<uint32_T>(bLen) & 1U) != 0U) {
      bLen = i3 + i4;
      b_x_data[bLen] = xwork_data[bLen];
    }

    if (i3 > 1) {
      std::memset(&c_iwork_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof
                  (int32_T));
      quartetOffset = i3 >> 2;
      bLen = 4;
      while (quartetOffset > 1) {
        if ((static_cast<uint32_T>(quartetOffset) & 1U) != 0U) {
          quartetOffset--;
          wOffset = bLen * quartetOffset;
          i4 = i3 - wOffset;
          if (i4 > bLen) {
            AEBSensorFusion_merge_e(idx_data, b_x_data, wOffset, bLen, i4 - bLen,
              c_iwork_data, xwork_data);
          }
        }

        i4 = bLen << 1;
        quartetOffset >>= 1;
        for (wOffset = 0; wOffset < quartetOffset; wOffset++) {
          AEBSensorFusion_merge_e(idx_data, b_x_data, wOffset * i4, bLen, bLen,
            c_iwork_data, xwork_data);
        }

        bLen = i4;
      }

      if (i3 > bLen) {
        AEBSensorFusion_merge_e(idx_data, b_x_data, 0, bLen, i3 - bLen,
          c_iwork_data, xwork_data);
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_merge_ef(int32_T idx_data[],
  real_T x_data[], int32_T offset, int32_T np, int32_T nq, int32_T iwork_data[],
  real_T xwork_data[])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (nq != 0) {
    int32_T iout;
    int32_T n;
    int32_T q;
    int32_T qend;
    qend = np + nq;
    for (q = 0; q < qend; q++) {
      iout = offset + q;
      iwork_data[q] = idx_data[iout];
      xwork_data[q] = x_data[iout];
    }

    n = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[n] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[n];
        x_data[iout] = xwork_data[n];
        if (n + 1 < np) {
          n++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < qend) {
          q++;
        } else {
          qend = iout - n;
          for (q = n + 1; q <= np; q++) {
            iout = qend + q;
            idx_data[iout] = iwork_data[q - 1];
            x_data[iout] = xwork_data[q - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_sort_mhq(real_T x_data[],
  const int32_T x_size[1], int32_T idx_data[], int32_T idx_size[1])
{
  real_T vwork_data[70];
  real_T xwork_data[70];
  real_T x4[4];
  real_T tmp;
  real_T tmp_0;
  int32_T c_idx_data[70];
  int32_T c_iwork_data[70];
  int32_T idx4[4];
  int32_T perm[4];
  int32_T b;
  int32_T bLen;
  int32_T dim;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T ib;
  int32_T perm_0;
  int32_T perm_1;
  int32_T quartetOffset;
  int32_T vstride;
  int32_T wOffset;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  dim = 1;
  if (x_size[0] != 1) {
    dim = 0;
    b = x_size[0];
  } else {
    b = 1;
  }

  idx_size[0] = x_size[0];
  vstride = 1;
  for (quartetOffset = 0; quartetOffset < dim; quartetOffset++) {
    vstride *= x_size[0];
  }

  for (dim = 0; dim < vstride; dim++) {
    for (wOffset = 0; wOffset < b; wOffset++) {
      vwork_data[wOffset] = x_data[wOffset * vstride + dim];
    }

    if (b - 1 >= 0) {
      std::memset(&c_idx_data[0], 0, static_cast<uint32_T>(b) * sizeof(int32_T));
    }

    if (b != 0) {
      if (b - 1 >= 0) {
        std::memset(&c_idx_data[0], 0, static_cast<uint32_T>(b) * sizeof(int32_T));
      }

      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      bLen = 0;
      ib = 0;
      for (wOffset = 0; wOffset < b; wOffset++) {
        if (rtIsNaN(vwork_data[wOffset])) {
          quartetOffset = (b - bLen) - 1;
          c_idx_data[quartetOffset] = wOffset + 1;
          xwork_data[quartetOffset] = vwork_data[wOffset];
          bLen++;
        } else {
          ib++;
          idx4[ib - 1] = wOffset + 1;
          x4[ib - 1] = vwork_data[wOffset];
          if (ib == 4) {
            quartetOffset = wOffset - bLen;
            if (x4[0] <= x4[1]) {
              ib = 1;
              i2 = 2;
            } else {
              ib = 2;
              i2 = 1;
            }

            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }

            tmp = x4[ib - 1];
            tmp_0 = x4[i3 - 1];
            if (tmp <= tmp_0) {
              tmp = x4[i2 - 1];
              if (tmp <= tmp_0) {
                perm_0 = ib;
                perm_1 = i2;
                ib = i3;
                i2 = i4;
              } else if (tmp <= x4[i4 - 1]) {
                perm_0 = ib;
                perm_1 = i3;
                ib = i2;
                i2 = i4;
              } else {
                perm_0 = ib;
                perm_1 = i3;
                ib = i4;
              }
            } else {
              tmp_0 = x4[i4 - 1];
              if (tmp <= tmp_0) {
                if (x4[i2 - 1] <= tmp_0) {
                  perm_0 = i3;
                  perm_1 = ib;
                  ib = i2;
                  i2 = i4;
                } else {
                  perm_0 = i3;
                  perm_1 = ib;
                  ib = i4;
                }
              } else {
                perm_0 = i3;
                perm_1 = i4;
              }
            }

            c_idx_data[quartetOffset - 3] = idx4[perm_0 - 1];
            c_idx_data[quartetOffset - 2] = idx4[perm_1 - 1];
            c_idx_data[quartetOffset - 1] = idx4[ib - 1];
            c_idx_data[quartetOffset] = idx4[i2 - 1];
            vwork_data[quartetOffset - 3] = x4[perm_0 - 1];
            vwork_data[quartetOffset - 2] = x4[perm_1 - 1];
            vwork_data[quartetOffset - 1] = x4[ib - 1];
            vwork_data[quartetOffset] = x4[i2 - 1];
            ib = 0;
          }
        }
      }

      i3 = b - bLen;
      if (ib > 0) {
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (ib == 1) {
          perm[0] = 1;
        } else if (ib == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }

        i2 = static_cast<uint8_T>(ib);
        for (i4 = 0; i4 < i2; i4++) {
          perm_0 = perm[i4];
          quartetOffset = (i3 - ib) + i4;
          c_idx_data[quartetOffset] = idx4[perm_0 - 1];
          vwork_data[quartetOffset] = x4[perm_0 - 1];
        }
      }

      ib = bLen >> 1;
      for (i4 = 0; i4 < ib; i4++) {
        wOffset = i4 + i3;
        i2 = c_idx_data[wOffset];
        quartetOffset = (b - i4) - 1;
        c_idx_data[wOffset] = c_idx_data[quartetOffset];
        c_idx_data[quartetOffset] = i2;
        vwork_data[wOffset] = xwork_data[quartetOffset];
        vwork_data[quartetOffset] = xwork_data[wOffset];
      }

      if ((static_cast<uint32_T>(bLen) & 1U) != 0U) {
        quartetOffset = i3 + ib;
        vwork_data[quartetOffset] = xwork_data[quartetOffset];
      }

      if (i3 > 1) {
        if (b - 1 >= 0) {
          std::memset(&c_iwork_data[0], 0, static_cast<uint32_T>(b) * sizeof
                      (int32_T));
        }

        quartetOffset = i3 >> 2;
        bLen = 4;
        while (quartetOffset > 1) {
          if ((static_cast<uint32_T>(quartetOffset) & 1U) != 0U) {
            quartetOffset--;
            wOffset = bLen * quartetOffset;
            i4 = i3 - wOffset;
            if (i4 > bLen) {
              AEBSensorFusion_merge_ef(c_idx_data, vwork_data, wOffset, bLen, i4
                - bLen, c_iwork_data, xwork_data);
            }
          }

          i4 = bLen << 1;
          quartetOffset >>= 1;
          for (wOffset = 0; wOffset < quartetOffset; wOffset++) {
            AEBSensorFusion_merge_ef(c_idx_data, vwork_data, wOffset * i4, bLen,
              bLen, c_iwork_data, xwork_data);
          }

          bLen = i4;
        }

        if (i3 > bLen) {
          AEBSensorFusion_merge_ef(c_idx_data, vwork_data, 0, bLen, i3 - bLen,
            c_iwork_data, xwork_data);
        }
      }
    }

    for (i4 = 0; i4 < b; i4++) {
      quartetOffset = i4 * vstride + dim;
      x_data[quartetOffset] = vwork_data[i4];
      idx_data[quartetOffset] = c_idx_data[i4];
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_measmodelsvalidateoptionalstruct(drivingCoordinateFrameType
  candidateStructs_Frame, const real_T candidateStructs_OriginPosition[3], const
  real_T candidateStructs_Orientation[9], boolean_T candidateStructs_HasVelocity,
  const real_T candidateStructs_OriginVelocity[3], boolean_T
  candidateStructs_IsParentToChild, boolean_T candidateStructs_HasAzimuth,
  boolean_T candidateStructs_HasElevation, boolean_T candidateStructs_HasRange,
  n_cell_AEBSensorFusion_T *argsinCell)
{
  real_T thisOrient[9];
  real_T tmp;
  real_T tmp_0;
  int32_T i;
  int32_T i_0;
  int32_T tmp_1;
  int32_T tmp_2;
  static const int8_T tmp_3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  std::memcpy(&thisOrient[0], &candidateStructs_Orientation[0], 9U * sizeof
              (real_T));

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (candidateStructs_IsParentToChild) {
    for (i = 0; i < 3; i++) {
      thisOrient[3 * i] = candidateStructs_Orientation[i];
      thisOrient[3 * i + 1] = candidateStructs_Orientation[i + 3];
      thisOrient[3 * i + 2] = candidateStructs_Orientation[i + 6];
    }
  }

  AEBSensorFusion_char(candidateStructs_Frame, argsinCell->f1.data,
                       argsinCell->f1.size);
  for (i = 0; i < 3; i++) {
    tmp = 0.0;
    tmp_0 = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      tmp_2 = 3 * i_0 + i;
      tmp_1 = tmp_3[tmp_2];
      tmp += static_cast<real_T>(tmp_1) * candidateStructs_OriginPosition[i_0];
      tmp_0 += static_cast<real_T>(tmp_1) * candidateStructs_OriginVelocity[i_0];
      argsinCell->f4[tmp_2] = (thisOrient[3 * i_0 + 1] * static_cast<real_T>
        (tmp_3[i + 3]) + thisOrient[3 * i_0] * static_cast<real_T>(tmp_3[i])) +
        thisOrient[3 * i_0 + 2] * static_cast<real_T>(tmp_3[i + 6]);
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    argsinCell->f2[i] = tmp;
    argsinCell->f3[i] = tmp_0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  argsinCell->f5 = candidateStructs_HasAzimuth;
  argsinCell->f6 = candidateStructs_HasElevation;
  argsinCell->f7 = candidateStructs_HasVelocity;
  argsinCell->f8 = candidateStructs_HasRange;
  argsinCell->f9 = false;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_get_match(const char_T
  str_data[], const int32_T str_size[2], char_T match_data[], int32_T
  match_size[2], int32_T *nmatched)
{
  int32_T lenstr_tmp;
  int32_T minnanb;
  boolean_T b_bool;
  boolean_T matched;
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  static const char_T b[7] = { 'I', 'n', 'v', 'a', 'l', 'i', 'd' };

  static const char_T b_vstr[9] = { 's', 'p', 'h', 'e', 'r', 'i', 'c', 'a', 'l'
  };

  static const char_T c_vstr[11] = { 'r', 'e', 'c', 't', 'a', 'n', 'g', 'u', 'l',
    'a', 'r' };

  int32_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  match_size[0] = 1;
  match_size[1] = 7;
  for (minnanb = 0; minnanb < 7; minnanb++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    match_data[minnanb] = ' ';
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  lenstr_tmp = str_size[1];
  *nmatched = 0;
  matched = false;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (str_size[1] <= 7) {
    b_bool = false;
    minnanb = 1;
    do {
      exitg1 = 0;
      if (minnanb - 1 < 7) {
        if (tmp[static_cast<int32_T>(str_data[minnanb - 1])] != tmp[static_cast<
            int32_T>(b[minnanb - 1])]) {
          exitg1 = 1;
        } else {
          minnanb++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      *nmatched = 1;
      match_size[0] = 1;
      match_size[1] = 7;
      for (minnanb = 0; minnanb < 7; minnanb++) {
        match_data[minnanb] = b[minnanb];
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3) {
    if (str_size[1] <= 9) {
      b_bool = false;
      minnanb = 1;
      do {
        exitg1 = 0;
        if (minnanb - 1 <= lenstr_tmp - 1) {
          if (tmp[static_cast<int32_T>(str_data[minnanb - 1])] != tmp[
              static_cast<int32_T>(b_vstr[minnanb - 1])]) {
            exitg1 = 1;
          } else {
            minnanb++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      if (b_bool) {
        if (str_size[1] == 9) {
          *nmatched = 1;
          match_size[0] = 1;
          match_size[1] = 9;
          for (minnanb = 0; minnanb < 9; minnanb++) {
            match_data[minnanb] = b_vstr[minnanb];
          }
        } else {
          match_size[0] = 1;
          match_size[1] = 9;
          for (minnanb = 0; minnanb < 9; minnanb++) {
            match_data[minnanb] = b_vstr[minnanb];
          }

          matched = true;
          *nmatched = 1;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    b_bool = false;
    minnanb = 1;
    do {
      exitg1 = 0;
      if (minnanb - 1 <= lenstr_tmp - 1) {
        if (tmp[static_cast<int32_T>(str_data[minnanb - 1])] != tmp
            [static_cast<int32_T>(c_vstr[minnanb - 1])]) {
          exitg1 = 1;
        } else {
          minnanb++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      if (str_size[1] == 11) {
        *nmatched = 1;
        match_size[0] = 1;
        match_size[1] = 11;
        for (minnanb = 0; minnanb < 11; minnanb++) {
          match_data[minnanb] = c_vstr[minnanb];
        }
      } else {
        if (!matched) {
          match_size[0] = 1;
          match_size[1] = 11;
          for (minnanb = 0; minnanb < 11; minnanb++) {
            match_data[minnanb] = c_vstr[minnanb];
          }
        }

        (*nmatched)++;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    if (*nmatched == 0) {
      match_size[0] = 1;
      match_size[1] = 7;
      for (minnanb = 0; minnanb < 7; minnanb++) {
        match_data[minnanb] = ' ';
      }
    }
  }
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_radialspeed(const real_T
  tgtpos[3], const real_T tgtvel[3], const real_T refpos[3], const real_T
  refvel[3])
{
  real_T rn;
  real_T rspeed;
  real_T sn;
  real_T tgtdirec_idx_0;
  real_T tgtdirec_idx_1;
  real_T tgtdirec_idx_2;
  real_T veldirec_idx_0;
  real_T veldirec_idx_1;
  real_T veldirec_idx_2;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  tgtdirec_idx_0 = tgtpos[0] - refpos[0];
  veldirec_idx_0 = tgtvel[0] - refvel[0];
  tgtdirec_idx_1 = tgtpos[1] - refpos[1];
  veldirec_idx_1 = tgtvel[1] - refvel[1];
  tgtdirec_idx_2 = tgtpos[2] - refpos[2];
  veldirec_idx_2 = tgtvel[2] - refvel[2];
  rn = std::sqrt((tgtdirec_idx_0 * tgtdirec_idx_0 + tgtdirec_idx_1 *
                  tgtdirec_idx_1) + tgtdirec_idx_2 * tgtdirec_idx_2);
  sn = std::sqrt((veldirec_idx_0 * veldirec_idx_0 + veldirec_idx_1 *
                  veldirec_idx_1) + veldirec_idx_2 * veldirec_idx_2);
  rspeed = -(((veldirec_idx_0 * tgtdirec_idx_0 + veldirec_idx_1 * tgtdirec_idx_1)
              + veldirec_idx_2 * tgtdirec_idx_2) / rn);
  if (sn == 0.0) {
    rspeed = 0.0;
  }

  if (rn == 0.0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    rspeed = -sn;
  }

  return rspeed;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_cvmeas(const real_T state[6],
  drivingCoordinateFrameType varargin_1_Frame, const real_T
  varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
  boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
  boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
  boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
  measurement_data[], int32_T measurement_size[1])
{
  n_cell_AEBSensorFusion_T expl_temp;
  real_T measurement_tmp[9];
  real_T lclRect[3];
  real_T meas[3];
  real_T tgtpos[3];
  real_T tgtvel[3];
  real_T hypotxy;
  real_T tgtpos_idx_1;
  real_T tgtpos_idx_2;
  int32_T frame_size[2];
  int32_T measSize;
  int32_T tgtpos_tmp;
  int32_T trueCount;
  char_T frame_data[11];
  int8_T tmp_data[3];
  boolean_T tmp[3];
  boolean_T hasRangeRate;
  AEBSensorFusion_measmodelsvalidateoptionalstruct(varargin_1_Frame,
    varargin_1_OriginPosition, varargin_1_Orientation, varargin_1_HasVelocity,
    varargin_1_OriginVelocity, varargin_1_IsParentToChild, varargin_1_HasAzimuth,
    varargin_1_HasElevation, varargin_1_HasRange, &expl_temp);
  AEBSensorFusion_get_match(expl_temp.f1.data, expl_temp.f1.size, frame_data,
    frame_size, &measSize);
  if (measSize == 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    frame_size[0] = 1;
    frame_size[1] = 7;
    for (tgtpos_tmp = 0; tgtpos_tmp < 7; tgtpos_tmp++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      frame_data[tgtpos_tmp] = ' ';
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (measSize = 0; measSize < 3; measSize++) {
    hypotxy = AEBSensorFusion_norm(&expl_temp.f4[3 * measSize]);
    expl_temp.f4[3 * measSize] /= hypotxy;
    tgtpos_tmp = 3 * measSize + 1;
    expl_temp.f4[tgtpos_tmp] /= hypotxy;
    tgtpos_tmp = 3 * measSize + 2;
    expl_temp.f4[tgtpos_tmp] /= hypotxy;
    tgtpos_tmp = measSize << 1;
    tgtpos[measSize] = state[tgtpos_tmp];
    tgtvel[measSize] = state[tgtpos_tmp + 1];
  }

  if (AEBSensorFusion_strcmp(frame_data, frame_size)) {
    if (!expl_temp.f7) {
      hypotxy = tgtpos[0] - expl_temp.f2[0];
      tgtpos_idx_1 = tgtpos[1] - expl_temp.f2[1];
      tgtpos_idx_2 = tgtpos[2] - expl_temp.f2[2];
      measurement_size[0] = 3;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = (expl_temp.f4[3 * tgtpos_tmp + 1] *
          tgtpos_idx_1 + expl_temp.f4[3 * tgtpos_tmp] * hypotxy) + expl_temp.f4
          [3 * tgtpos_tmp + 2] * tgtpos_idx_2;
      }
    } else {
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_tmp[3 * tgtpos_tmp] = expl_temp.f4[tgtpos_tmp];
        measurement_tmp[3 * tgtpos_tmp + 1] = expl_temp.f4[tgtpos_tmp + 3];
        measurement_tmp[3 * tgtpos_tmp + 2] = expl_temp.f4[tgtpos_tmp + 6];
        lclRect[tgtpos_tmp] = tgtpos[tgtpos_tmp] - expl_temp.f2[tgtpos_tmp];
      }

      hypotxy = lclRect[1];
      tgtpos_idx_1 = lclRect[0];
      tgtpos_idx_2 = lclRect[2];
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        tgtpos[tgtpos_tmp] = (measurement_tmp[tgtpos_tmp + 3] * hypotxy +
                              measurement_tmp[tgtpos_tmp] * tgtpos_idx_1) +
          measurement_tmp[tgtpos_tmp + 6] * tgtpos_idx_2;
        lclRect[tgtpos_tmp] = tgtvel[tgtpos_tmp] - expl_temp.f3[tgtpos_tmp];
      }

      measurement_size[0] = 6;
      hypotxy = lclRect[1];
      tgtpos_idx_1 = lclRect[0];
      tgtpos_idx_2 = lclRect[2];
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = tgtpos[tgtpos_tmp];
        measurement_data[tgtpos_tmp + 3] = (measurement_tmp[tgtpos_tmp + 3] *
          hypotxy + measurement_tmp[tgtpos_tmp] * tgtpos_idx_1) +
          measurement_tmp[tgtpos_tmp + 6] * tgtpos_idx_2;
      }
    }
  } else {
    hasRangeRate = (expl_temp.f7 && expl_temp.f8);
    measSize = ((expl_temp.f5 + expl_temp.f6) + expl_temp.f8) + hasRangeRate;
    measurement_size[0] = measSize;
    if (measSize - 1 >= 0) {
      std::memset(&measurement_data[0], 0, static_cast<uint32_T>(measSize) *
                  sizeof(real_T));
    }

    hypotxy = tgtpos[0] - expl_temp.f2[0];
    tgtpos_idx_1 = tgtpos[1] - expl_temp.f2[1];
    tgtpos_idx_2 = tgtpos[2] - expl_temp.f2[2];
    for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
      lclRect[tgtpos_tmp] = (expl_temp.f4[3 * tgtpos_tmp + 1] * tgtpos_idx_1 +
        expl_temp.f4[3 * tgtpos_tmp] * hypotxy) + expl_temp.f4[3 * tgtpos_tmp +
        2] * tgtpos_idx_2;
    }

    hypotxy = rt_hypotd_snf(lclRect[0], lclRect[1]);
    meas[0] = rt_atan2d_snf(lclRect[1], lclRect[0]);
    meas[1] = rt_atan2d_snf(lclRect[2], hypotxy);
    meas[2] = rt_hypotd_snf(hypotxy, lclRect[2]);
    meas[0] *= 57.295779513082323;
    meas[1] *= 57.295779513082323;
    if (!hasRangeRate) {
      tmp[0] = expl_temp.f5;
      tmp[1] = expl_temp.f6;
      tmp[2] = expl_temp.f8;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          trueCount++;
        }
      }

      measSize = trueCount;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          tmp_data[trueCount] = static_cast<int8_T>(tgtpos_tmp);
          trueCount++;
        }
      }

      measurement_size[0] = measSize;
      for (tgtpos_tmp = 0; tgtpos_tmp < measSize; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = meas[tmp_data[tgtpos_tmp]];
      }
    } else {
      if (measSize - 1 < 1) {
        measSize = 0;
      } else {
        measSize--;
      }

      tmp[0] = expl_temp.f5;
      tmp[1] = expl_temp.f6;
      tmp[2] = expl_temp.f8;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          tmp_data[trueCount] = static_cast<int8_T>(tgtpos_tmp);
          trueCount++;
        }
      }

      for (tgtpos_tmp = 0; tgtpos_tmp < measSize; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = meas[tmp_data[tgtpos_tmp]];
      }

      measurement_data[measurement_size[0] - 1] = -AEBSensorFusion_radialspeed
        (tgtpos, tgtvel, expl_temp.f2, expl_temp.f3);
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op(real_T in1
  [6], const BusDetectionConcatenation1Detections *in2)
{
  real_T tmp_data[6];
  int32_T tmp_size[1];
  int32_T i;
  int32_T stride_0_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cvmeas(in1, in2->MeasurementParameters.Frame,
    in2->MeasurementParameters.OriginPosition,
    in2->MeasurementParameters.Orientation,
    in2->MeasurementParameters.HasVelocity,
    in2->MeasurementParameters.OriginVelocity,
    in2->MeasurementParameters.IsParentToChild,
    in2->MeasurementParameters.HasAzimuth,
    in2->MeasurementParameters.HasElevation, in2->MeasurementParameters.HasRange,
    tmp_data, tmp_size);
  stride_0_0 = (tmp_size[0] != 1);
  for (i = 0; i < 6; i++) {
    in1[i] = tmp_data[i * stride_0_0] - in2->Measurement[i];
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_xzgetrf(real_T A[36],
  int32_T ipiv[6], int32_T *info)
{
  int32_T ix;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (ix = 0; ix < 6; ix++) {
    ipiv[ix] = ix + 1;
  }

  *info = 0;
  for (int32_T b_j = 0; b_j < 5; b_j++) {
    real_T smax;
    int32_T c;
    int32_T iy;
    int32_T jj;
    int32_T jp1j;
    int32_T jy;
    int32_T k;
    c = b_j * 7 + 2;
    jj = b_j * 7;
    iy = 6 - b_j;
    ix = 1;
    smax = std::abs(A[jj]);
    for (k = 2; k <= iy; k++) {
      real_T s;
      s = std::abs(A[(c + k) - 3]);
      if (s > smax) {
        ix = k;
        smax = s;
      }
    }

    if (A[(c + ix) - 3] != 0.0) {
      if (ix - 1 != 0) {
        iy = b_j + ix;
        ipiv[b_j] = iy;
        for (k = 0; k < 6; k++) {
          jp1j = k * 6 + b_j;
          smax = A[jp1j];
          ix = (k * 6 + iy) - 1;
          A[jp1j] = A[ix];
          A[ix] = smax;
        }
      }

      k = c - b_j;
      for (ix = c; ix <= k + 4; ix++) {
        A[ix - 1] /= A[jj];
      }
    } else {
      *info = b_j + 1;
    }

    jp1j = jj;
    jy = jj + 6;
    k = 5 - b_j;
    for (jj = 0; jj < k; jj++) {
      smax = A[jj * 6 + jy];
      if (smax != 0.0) {
        int32_T d;
        iy = jp1j + 8;
        d = (jp1j - b_j) + 12;
        for (ix = iy; ix <= d; ix++) {
          A[ix - 1] += A[((c + ix) - jp1j) - 9] * -smax;
        }
      }

      jp1j += 6;
    }
  }

  if ((*info == 0) && (!(A[35] != 0.0))) {
    *info = 6;
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_mrdiv_b(real_T A[6], const
  real_T B[36])
{
  real_T c_A[36];
  real_T temp;
  int32_T b_ipiv[6];
  int32_T b_info;
  int32_T b_k;
  int32_T jAcol;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&c_A[0], &B[0], 36U * sizeof(real_T));
  AEBSensorFusion_xzgetrf(c_A, b_ipiv, &b_info);
  for (b_info = 0; b_info < 6; b_info++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    jAcol = 6 * b_info - 1;
    for (b_k = 0; b_k < b_info; b_k++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      temp = c_A[(b_k + jAcol) + 1];
      if (temp != 0.0) {
        A[b_info] -= temp * A[b_k];
      }
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    A[b_info] *= 1.0 / c_A[(b_info + jAcol) + 1];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (b_info = 5; b_info >= 0; b_info--) {
    jAcol = 6 * b_info - 1;
    for (b_k = b_info + 2; b_k < 7; b_k++) {
      temp = c_A[b_k + jAcol];
      if (temp != 0.0) {
        A[b_info] -= A[b_k - 1] * temp;
      }
    }
  }

  for (b_info = 4; b_info >= 0; b_info--) {
    jAcol = b_ipiv[b_info];
    if (b_info + 1 != jAcol) {
      temp = A[b_info];
      A[b_info] = A[jAcol - 1];
      A[jAcol - 1] = temp;
    }
  }
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1g3p(int32_T n,
  const real_T x[54], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return scale * std::sqrt(y);
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_EKFPredictorNonAdditive_predict_k(const real_T Qs[9], real_T
  x[6], const real_T S[36], real_T varargin_1, real_T b_S[36], real_T dFdx[36])
{
  real_T c_A[54];
  real_T y[36];
  real_T dFdw[18];
  real_T dFdw_0[18];
  real_T b_tau[6];
  real_T work[6];
  real_T B_idx_0;
  real_T dFdw_1;
  real_T s;
  int32_T aoffset;
  int32_T b_i;
  int32_T coffset;
  int32_T e;
  int32_T exitg1;
  int32_T ic0;
  int32_T ii;
  int32_T ix;
  int32_T jy;
  int32_T lastv;
  boolean_T exitg2;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  s = varargin_1 * varargin_1;
  B_idx_0 = s / 2.0;
  std::memset(&dFdx[0], 0, 36U * sizeof(real_T));

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  dFdx[0] = 1.0;
  dFdx[14] = 1.0;
  dFdx[28] = 1.0;
  dFdx[1] = 0.0;
  dFdx[15] = 0.0;
  dFdx[29] = 0.0;
  dFdx[6] = varargin_1;
  dFdx[20] = varargin_1;
  dFdx[34] = varargin_1;
  dFdx[7] = 1.0;
  dFdx[21] = 1.0;
  dFdx[35] = 1.0;
  std::memset(&dFdw[0], 0, 18U * sizeof(real_T));

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  dFdw[0] = B_idx_0;
  dFdw[8] = B_idx_0;
  dFdw[16] = B_idx_0;
  dFdw[1] = varargin_1;
  dFdw[9] = varargin_1;
  dFdw[17] = varargin_1;
  s = s * 0.5 * 0.0;
  x[0] = (x[1] * varargin_1 + x[0]) + s;
  x[1] += 0.0 * varargin_1;
  x[2] = (x[3] * varargin_1 + x[2]) + s;
  x[3] += 0.0 * varargin_1;
  x[4] = (x[5] * varargin_1 + x[4]) + s;
  x[5] += 0.0 * varargin_1;
  for (ii = 0; ii < 6; ii++) {
    coffset = ii * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      aoffset = b_i * 6 - 1;
      s = 0.0;
      for (ix = 0; ix < 6; ix++) {
        s += S[(aoffset + ix) + 1] * dFdx[ix * 6 + ii];
      }

      y[(coffset + b_i) + 1] = s;
    }

    b_tau[ii] = 0.0;
    s = dFdw[ii + 6];
    B_idx_0 = dFdw[ii];
    dFdw_1 = dFdw[ii + 12];
    for (b_i = 0; b_i < 3; b_i++) {
      dFdw_0[b_i + 3 * ii] = (Qs[3 * b_i + 1] * s + Qs[3 * b_i] * B_idx_0) + Qs
        [3 * b_i + 2] * dFdw_1;
    }
  }

  for (ii = 0; ii < 6; ii++) {
    for (b_i = 0; b_i < 6; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      c_A[b_i + 9 * ii] = y[6 * ii + b_i];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    c_A[9 * ii + 6] = dFdw_0[3 * ii];
    c_A[9 * ii + 7] = dFdw_0[3 * ii + 1];
    c_A[9 * ii + 8] = dFdw_0[3 * ii + 2];
    work[ii] = 0.0;
  }

  for (b_i = 0; b_i < 6; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    ii = b_i * 9 + b_i;
    ix = ii + 2;
    s = c_A[ii];
    b_tau[b_i] = 0.0;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    B_idx_0 = AEBSensorFusion_xnrm2_po1g3p(8 - b_i, c_A, ii + 2);
    if (B_idx_0 != 0.0) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      dFdw_1 = c_A[ii];
      B_idx_0 = rt_hypotd_snf(dFdw_1, B_idx_0);
      if (dFdw_1 >= 0.0) {
        B_idx_0 = -B_idx_0;
      }

      if (std::abs(B_idx_0) < 1.0020841800044864E-292) {
        aoffset = -1;
        do {
          aoffset++;
          coffset = ii - b_i;
          for (lastv = ix; lastv <= coffset + 9; lastv++) {
            c_A[lastv - 1] *= 9.9792015476736E+291;
          }

          B_idx_0 *= 9.9792015476736E+291;
          s *= 9.9792015476736E+291;
        } while ((std::abs(B_idx_0) < 1.0020841800044864E-292) && (aoffset + 1 <
                  20));

        B_idx_0 = rt_hypotd_snf(s, AEBSensorFusion_xnrm2_po1g3p(8 - b_i, c_A, ii
          + 2));
        if (s >= 0.0) {
          B_idx_0 = -B_idx_0;
        }

        b_tau[b_i] = (B_idx_0 - s) / B_idx_0;
        s = 1.0 / (s - B_idx_0);
        for (lastv = ix; lastv <= coffset + 9; lastv++) {
          c_A[lastv - 1] *= s;
        }

        for (lastv = 0; lastv <= aoffset; lastv++) {
          B_idx_0 *= 1.0020841800044864E-292;
        }

        s = B_idx_0;
      } else {
        b_tau[b_i] = (B_idx_0 - dFdw_1) / B_idx_0;
        s = 1.0 / (dFdw_1 - B_idx_0);
        coffset = ii - b_i;
        for (lastv = ix; lastv <= coffset + 9; lastv++) {
          c_A[lastv - 1] *= s;
        }

        s = B_idx_0;
      }
    }

    c_A[ii] = s;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (b_i + 1 < 6) {
      c_A[ii] = 1.0;
      ic0 = ii + 10;
      if (b_tau[b_i] != 0.0) {
        lastv = 9 - b_i;
        coffset = ii - b_i;
        while ((lastv > 0) && (c_A[coffset + 8] == 0.0)) {
          lastv--;
          coffset--;
        }

        ix = 5 - b_i;
        exitg2 = false;
        while ((!exitg2) && (ix > 0)) {
          aoffset = (ix - 1) * 9 + ii;
          coffset = aoffset + 10;
          do {
            exitg1 = 0;
            if (coffset <= (aoffset + lastv) + 9) {
              if (c_A[coffset - 1] != 0.0) {
                exitg1 = 1;
              } else {
                coffset++;
              }
            } else {
              ix--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        ix = 0;
      }

      if (lastv > 0) {
        if (ix != 0) {
          std::memset(&work[0], 0, static_cast<uint32_T>(ix) * sizeof(real_T));
          aoffset = ((ix - 1) * 9 + ii) + 10;
          for (jy = ic0; jy <= aoffset; jy += 9) {
            B_idx_0 = 0.0;
            e = jy + lastv;
            for (coffset = jy; coffset < e; coffset++) {
              B_idx_0 += c_A[(ii + coffset) - jy] * c_A[coffset - 1];
            }

            coffset = div_nde_s32_floor((jy - ii) - 10, 9);
            work[coffset] += B_idx_0;
          }
        }

        if (!(-b_tau[b_i] == 0.0)) {
          jy = ii;
          coffset = ix - 1;
          for (ix = 0; ix <= coffset; ix++) {
            B_idx_0 = work[ix];
            if (B_idx_0 != 0.0) {
              B_idx_0 *= -b_tau[b_i];
              ic0 = jy + 10;
              aoffset = (lastv + jy) + 9;
              for (e = ic0; e <= aoffset; e++) {
                c_A[e - 1] += c_A[((ii + e) - jy) - 10] * B_idx_0;
              }
            }

            jy += 9;
          }
        }
      }

      c_A[ii] = s;
    }
  }

  for (ii = 0; ii < 6; ii++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    std::memcpy(&y[ii * 6], &c_A[ii * 9], static_cast<uint32_T>(ii + 1) * sizeof
                (real_T));
    for (b_i = ii + 2; b_i < 7; b_i++) {
      y[(b_i + 6 * ii) - 1] = 0.0;
    }
  }

  for (b_i = 0; b_i < 6; b_i++) {
    for (ii = 0; ii < 6; ii++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b_S[ii + 6 * b_i] = y[6 * ii + b_i];
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_predictTrackFilter
  (c_trackingEKF_AEBSensorFusion_T *filter, real_T dt)
{
  int32_T i;
  int32_T i_0;
  int32_T tmp;
  static const int8_T tmp_0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_1[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  real_T filter_0[36];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!filter->IsLastJacobianInitialized) {
    filter->IsLastJacobianInitialized = true;
  }

  if (!filter->pIsDistributionsSetup) {
    filter->pIsDistributionsSetup = true;
  }

  if ((!filter->pIsSetStateCovariance) || (filter->pSqrtStateCovarianceScalar !=
       -1.0)) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      for (i = 0; i < 6; i++) {
        tmp = 6 * i_0 + i;
        filter->pSqrtStateCovariance[tmp] = static_cast<real_T>(tmp_1[tmp]) *
          filter->pSqrtStateCovarianceScalar;
      }
    }
  }

  if ((!filter->pIsSetProcessNoise) || (filter->pSqrtProcessNoiseScalar != -1.0))
  {
    for (i_0 = 0; i_0 < 3; i_0++) {
      filter->pSqrtProcessNoise[3 * i_0] = static_cast<real_T>(tmp_0[3 * i_0]) *
        filter->pSqrtProcessNoiseScalar;
      i = 3 * i_0 + 1;
      filter->pSqrtProcessNoise[i] = static_cast<real_T>(tmp_0[i]) *
        filter->pSqrtProcessNoiseScalar;
      i = 3 * i_0 + 2;
      filter->pSqrtProcessNoise[i] = static_cast<real_T>(tmp_0[i]) *
        filter->pSqrtProcessNoiseScalar;
    }

    filter->pIsSetProcessNoise = true;
    filter->pSqrtProcessNoiseScalar = -1.0;
  }

  if (filter->pIsFirstCallPredict) {
    if (!filter->pIsValidStateTransitionFcn) {
      filter->pIsValidStateTransitionFcn = true;
    }

    filter->pIsFirstCallPredict = false;
  }

  std::memcpy(&filter_0[0], &filter->pSqrtStateCovariance[0], 36U * sizeof
              (real_T));
  AEBSensorFusion_EKFPredictorNonAdditive_predict_k(filter->pSqrtProcessNoise,
    filter->pState, filter_0, dt, filter->pSqrtStateCovariance,
    filter->pJacobian);
  filter->pIsSetStateCovariance = true;
  filter->pSqrtStateCovarianceScalar = -1.0;
  filter->pHasPrediction = true;

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_cvmeasjac(const real_T
  state[6], drivingCoordinateFrameType varargin_1_Frame, const real_T
  varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
  boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
  boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
  boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
  jacobian_data[], int32_T jacobian_size[2])
{
  real_T localAxes[9];
  real_T thisOrient[9];
  real_T tmp[9];
  real_T relposlocal[3];
  real_T relposlocal_tmp[3];
  real_T state_0[3];
  real_T tgtpos[3];
  real_T localAxes_0;
  real_T localAxes_1;
  real_T localAxes_2;
  real_T measSize;
  real_T relposlocal_idx_0;
  real_T relposlocal_idx_1;
  real_T relposlocal_tmp_0;
  real_T thisOrient_0;
  real_T varargin_1_OriginPosition_0;
  real_T xysq;
  real_T xyzsq;
  int32_T i;
  int32_T localAxes_tmp;
  int32_T nmatched;
  int32_T trueCount;
  char_T frame_data[11];
  int8_T tmp_data[3];
  int8_T tmp_0;
  int8_T tmp_1;
  int8_T tmp_2;
  boolean_T measLogicalIndex[3];
  boolean_T hasRangeRate;
  static const int8_T tmp_3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_4[9] = { 0, 0, 1, 0, 0, 1, 0, 0, 1 };

  static const int8_T tmp_5[9] = { 0, 1, 0, 0, 1, 0, 0, 1, 0 };

  static const int8_T tmp_6[6] = { 0, 1, 3, 4, 6, 7 };

  int32_T frame_size[2];
  int32_T tmp_size[2];
  char_T tmp_data_0[11];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&thisOrient[0], &varargin_1_Orientation[0], 9U * sizeof(real_T));
  if (varargin_1_IsParentToChild) {
    for (i = 0; i < 3; i++) {
      thisOrient[3 * i] = varargin_1_Orientation[i];
      thisOrient[3 * i + 1] = varargin_1_Orientation[i + 3];
      thisOrient[3 * i + 2] = varargin_1_Orientation[i + 6];
    }
  }

  AEBSensorFusion_char(varargin_1_Frame, tmp_data_0, tmp_size);
  AEBSensorFusion_get_match(tmp_data_0, tmp_size, frame_data, frame_size,
    &nmatched);
  if (nmatched == 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    frame_size[0] = 1;
    frame_size[1] = 7;
    for (i = 0; i < 7; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      frame_data[i] = ' ';
    }
  }

  for (i = 0; i < 3; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    measSize = thisOrient[3 * i + 1];
    xysq = thisOrient[3 * i];
    thisOrient_0 = thisOrient[3 * i + 2];
    for (trueCount = 0; trueCount < 3; trueCount++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      localAxes[trueCount + 3 * i] = (static_cast<real_T>(tmp_3[trueCount + 3]) *
        measSize + xysq * static_cast<real_T>(tmp_3[trueCount])) +
        static_cast<real_T>(tmp_3[trueCount + 6]) * thisOrient_0;
    }
  }

  for (nmatched = 0; nmatched < 3; nmatched++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    xysq = AEBSensorFusion_norm(&localAxes[3 * nmatched]);
    localAxes[3 * nmatched] /= xysq;
    localAxes_tmp = 3 * nmatched + 1;
    localAxes[localAxes_tmp] /= xysq;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    localAxes_tmp = 3 * nmatched + 2;
    localAxes[localAxes_tmp] /= xysq;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (AEBSensorFusion_strcmp(frame_data, frame_size)) {
    if (!varargin_1_HasVelocity) {
      jacobian_size[0] = 3;
      jacobian_size[1] = 6;
      std::memset(&jacobian_data[0], 0, 18U * sizeof(real_T));
      for (nmatched = 0; nmatched < 3; nmatched++) {
        trueCount = (((nmatched + 1) << 1) - 2) * 3;
        jacobian_data[trueCount] = localAxes[nmatched];
        jacobian_data[1 + trueCount] = localAxes[nmatched + 3];
        jacobian_data[2 + trueCount] = localAxes[nmatched + 6];
      }
    } else {
      jacobian_size[0] = 6;
      jacobian_size[1] = 6;
      std::memset(&jacobian_data[0], 0, 36U * sizeof(real_T));
      for (nmatched = 0; nmatched < 3; nmatched++) {
        i = (nmatched + 1) << 1;
        measSize = localAxes[nmatched];
        trueCount = (i - 2) * 6;
        jacobian_data[trueCount] = measSize;
        i = (i - 1) * 6;
        jacobian_data[3 + i] = measSize;
        measSize = localAxes[nmatched + 3];
        jacobian_data[1 + trueCount] = measSize;
        jacobian_data[4 + i] = measSize;
        measSize = localAxes[nmatched + 6];
        jacobian_data[2 + trueCount] = measSize;
        jacobian_data[5 + i] = measSize;
      }
    }
  } else {
    hasRangeRate = (varargin_1_HasVelocity && varargin_1_HasRange);
    measSize = ((static_cast<real_T>(varargin_1_HasAzimuth) + static_cast<real_T>
                 (varargin_1_HasElevation)) + static_cast<real_T>
                (varargin_1_HasRange)) + static_cast<real_T>(hasRangeRate);
    localAxes_tmp = static_cast<int32_T>(measSize);
    jacobian_size[0] = localAxes_tmp;
    jacobian_size[1] = 6;
    nmatched = localAxes_tmp * 6;
    if (nmatched - 1 >= 0) {
      std::memset(&jacobian_data[0], 0, static_cast<uint32_T>(nmatched) * sizeof
                  (real_T));
    }

    xysq = varargin_1_OriginPosition[1];
    thisOrient_0 = varargin_1_OriginPosition[0];
    varargin_1_OriginPosition_0 = varargin_1_OriginPosition[2];
    for (i = 0; i < 3; i++) {
      relposlocal_tmp_0 = (static_cast<real_T>(tmp_3[i + 3]) * xysq +
                           static_cast<real_T>(tmp_3[i]) * thisOrient_0) +
        static_cast<real_T>(tmp_3[i + 6]) * varargin_1_OriginPosition_0;
      relposlocal_tmp[i] = relposlocal_tmp_0;
      state_0[i] = state[i << 1] - relposlocal_tmp_0;
    }

    xysq = state_0[1];
    thisOrient_0 = state_0[0];
    varargin_1_OriginPosition_0 = state_0[2];
    for (i = 0; i < 3; i++) {
      relposlocal[i] = (localAxes[3 * i + 1] * xysq + localAxes[3 * i] *
                        thisOrient_0) + localAxes[3 * i + 2] *
        varargin_1_OriginPosition_0;
    }

    relposlocal_tmp_0 = relposlocal[0] * relposlocal[0] + relposlocal[1] *
      relposlocal[1];
    xyzsq = relposlocal[2] * relposlocal[2] + relposlocal_tmp_0;
    std::memset(&thisOrient[0], 0, 9U * sizeof(real_T));
    if (xyzsq == 0.0) {
      for (i = 0; i < 3; i++) {
        tmp_0 = tmp_4[3 * i + 1];
        tmp_1 = tmp_4[3 * i];
        tmp_2 = tmp_4[3 * i + 2];
        for (trueCount = 0; trueCount < 3; trueCount++) {
          thisOrient[trueCount + 3 * i] = (localAxes[trueCount + 3] *
            static_cast<real_T>(tmp_0) + static_cast<real_T>(tmp_1) *
            localAxes[trueCount]) + localAxes[trueCount + 6] *
            static_cast<real_T>(tmp_2);
        }
      }
    } else if (relposlocal_tmp_0 == 0.0) {
      xysq = -1.0 / relposlocal[2] * 180.0 / 3.1415926535897931;
      tmp[1] = xysq;
      tmp[4] = xysq;
      tmp[7] = 0.0;
      tmp[0] = 0.0;
      tmp[2] = 0.0;
      tmp[3] = 0.0;
      tmp[5] = 0.0;
      tmp[6] = 0.0;
      tmp[8] = 1.0;
      for (i = 0; i < 3; i++) {
        xysq = tmp[3 * i + 1];
        thisOrient_0 = tmp[3 * i];
        varargin_1_OriginPosition_0 = tmp[3 * i + 2];
        for (trueCount = 0; trueCount < 3; trueCount++) {
          thisOrient[trueCount + 3 * i] = (localAxes[trueCount + 3] * xysq +
            thisOrient_0 * localAxes[trueCount]) + localAxes[trueCount + 6] *
            varargin_1_OriginPosition_0;
        }
      }
    } else {
      xysq = -relposlocal[1];
      thisOrient_0 = relposlocal[0];
      varargin_1_OriginPosition_0 = std::sqrt(relposlocal_tmp_0);
      relposlocal_idx_0 = -relposlocal[0] * relposlocal[2];
      relposlocal_idx_1 = -relposlocal[1] * relposlocal[2];
      for (i = 0; i < 3; i++) {
        localAxes_0 = localAxes[i + 3];
        localAxes_1 = localAxes[i];
        localAxes_2 = localAxes[i + 6];
        thisOrient[3 * i] = ((localAxes_0 * thisOrient_0 + localAxes_1 * xysq) +
                             localAxes_2 * 0.0) / relposlocal_tmp_0;
        thisOrient[3 * i + 1] = ((localAxes_0 * relposlocal_idx_1 + localAxes_1 *
          relposlocal_idx_0) + localAxes_2 * relposlocal_tmp_0) /
          varargin_1_OriginPosition_0 / xyzsq;
      }

      varargin_1_OriginPosition_0 = std::sqrt(xyzsq);
      xysq = relposlocal[0];
      thisOrient_0 = relposlocal[1];
      relposlocal_tmp_0 = relposlocal[2];
      for (i = 0; i < 3; i++) {
        thisOrient[3 * i + 2] = ((localAxes[i + 3] * thisOrient_0 + localAxes[i]
          * xysq) + localAxes[i + 6] * relposlocal_tmp_0) /
          varargin_1_OriginPosition_0;
        thisOrient[3 * i] = thisOrient[3 * i] * 180.0 / 3.1415926535897931;
        nmatched = 3 * i + 1;
        thisOrient[nmatched] = thisOrient[nmatched] * 180.0 / 3.1415926535897931;
      }
    }

    measLogicalIndex[0] = varargin_1_HasAzimuth;
    measLogicalIndex[1] = varargin_1_HasElevation;
    measLogicalIndex[2] = varargin_1_HasRange;
    if (!hasRangeRate) {
      trueCount = 0;
      for (i = 0; i < 3; i++) {
        if (measLogicalIndex[i]) {
          tmp_data[trueCount] = static_cast<int8_T>(i);
          trueCount++;
        }
      }

      for (i = 0; i < 3; i++) {
        for (trueCount = 0; trueCount < localAxes_tmp; trueCount++) {
          jacobian_data[trueCount + localAxes_tmp * (i << 1)] = thisOrient[3 * i
            + tmp_data[trueCount]];
        }
      }
    } else {
      if (measSize - 1.0 < 1.0) {
        nmatched = 0;
      } else {
        nmatched = localAxes_tmp - 1;
      }

      trueCount = 0;
      for (i = 0; i < 3; i++) {
        if (measLogicalIndex[i]) {
          tmp_data[trueCount] = static_cast<int8_T>(i);
          trueCount++;
        }
      }

      tgtpos[0] = state[0] - relposlocal_tmp[0];
      tgtpos[1] = state[2] - relposlocal_tmp[1];
      tgtpos[2] = state[4] - relposlocal_tmp[2];
      state_0[0] = state[1];
      state_0[1] = state[3];
      state_0[2] = state[5];
      for (i = 0; i < 3; i++) {
        for (trueCount = 0; trueCount < nmatched; trueCount++) {
          jacobian_data[trueCount + localAxes_tmp * (i << 1)] = thisOrient[3 * i
            + tmp_data[trueCount]];
        }

        relposlocal[i] = state_0[i] - ((static_cast<real_T>(tmp_3[i + 3]) *
          varargin_1_OriginVelocity[1] + static_cast<real_T>(tmp_3[i]) *
          varargin_1_OriginVelocity[0]) + static_cast<real_T>(tmp_3[i + 6]) *
          varargin_1_OriginVelocity[2]);
      }

      measSize = AEBSensorFusion_norm(tgtpos);
      xysq = AEBSensorFusion_norm(relposlocal);
      if (measSize > 0.0) {
        xysq = (tgtpos[0] * relposlocal[0] + tgtpos[1] * relposlocal[1]) +
          tgtpos[2] * relposlocal[2];
        thisOrient_0 = measSize * measSize;
        thisOrient[0] = (relposlocal[0] * measSize - xysq * tgtpos[0] / measSize)
          / thisOrient_0;
        thisOrient[1] = tgtpos[0] / measSize;
        thisOrient[2] = 0.0;
        thisOrient[3] = (relposlocal[1] * measSize - xysq * tgtpos[1] / measSize)
          / thisOrient_0;
        thisOrient[4] = tgtpos[1] / measSize;
        thisOrient[5] = 0.0;
        thisOrient[6] = (relposlocal[2] * measSize - xysq * tgtpos[2] / measSize)
          / thisOrient_0;
        thisOrient[7] = tgtpos[2] / measSize;
        thisOrient[8] = 0.0;
      } else if (xysq != 0.0) {
        thisOrient[0] = 0.0;
        thisOrient[1] = relposlocal[0] / xysq;
        thisOrient[2] = 0.0;
        thisOrient[3] = 0.0;
        thisOrient[4] = relposlocal[1] / xysq;
        thisOrient[5] = 0.0;
        thisOrient[6] = 0.0;
        thisOrient[7] = relposlocal[2] / xysq;
        thisOrient[8] = 0.0;
      } else {
        for (i = 0; i < 9; i++) {
          thisOrient[i] = tmp_5[i];
        }
      }

      for (i = 0; i < 6; i++) {
        jacobian_data[(localAxes_tmp + localAxes_tmp * i) - 1] =
          thisOrient[tmp_6[i]];
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_cvmeas_h(const real_T state
  [6], drivingCoordinateFrameType varargin_1_Frame, const real_T
  varargin_1_OriginPosition[3], const real_T varargin_1_Orientation[9],
  boolean_T varargin_1_HasVelocity, const real_T varargin_1_OriginVelocity[3],
  boolean_T varargin_1_IsParentToChild, boolean_T varargin_1_HasAzimuth,
  boolean_T varargin_1_HasElevation, boolean_T varargin_1_HasRange, real_T
  measurement_data[], int32_T measurement_size[1], real_T bounds_data[], int32_T
  bounds_size[2])
{
  n_cell_AEBSensorFusion_T expl_temp;
  real_T measurement_tmp[9];
  real_T lclRect[3];
  real_T meas[3];
  real_T tgtpos[3];
  real_T tgtvel[3];
  real_T hypotxy;
  real_T tgtpos_idx_1;
  real_T tgtpos_idx_2;
  int32_T measSize;
  int32_T tgtpos_tmp;
  int32_T trueCount;
  char_T frame_data[11];
  int8_T tmp_data_0[4];
  int8_T tmp_data[3];
  boolean_T tmp_0[4];
  boolean_T tmp[3];
  boolean_T hasRangeRate_tmp;
  static real_T tmp_1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static real_T tmp_2[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0 };

  static real_T tmp_3[8] = { -180.0, -90.0, 0.0, 0.0, 180.0, 90.0, 0.0, 0.0 };

  int32_T frame_size[2];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  tmp_3[2U] = (rtMinusInf);
  tmp_3[3U] = (rtMinusInf);
  tmp_3[6U] = (rtInf);
  tmp_3[7U] = (rtInf);
  tmp_2[0U] = (rtMinusInf);
  tmp_2[1U] = (rtMinusInf);
  tmp_2[2U] = (rtMinusInf);
  tmp_2[3U] = (rtMinusInf);
  tmp_2[4U] = (rtMinusInf);
  tmp_2[5U] = (rtMinusInf);
  tmp_2[6U] = (rtInf);
  tmp_2[7U] = (rtInf);
  tmp_2[8U] = (rtInf);
  tmp_2[9U] = (rtInf);
  tmp_2[10U] = (rtInf);
  tmp_2[11U] = (rtInf);
  tmp_1[0U] = (rtMinusInf);
  tmp_1[1U] = (rtMinusInf);
  tmp_1[2U] = (rtMinusInf);
  tmp_1[3U] = (rtInf);
  tmp_1[4U] = (rtInf);
  tmp_1[5U] = (rtInf);
  AEBSensorFusion_measmodelsvalidateoptionalstruct(varargin_1_Frame,
    varargin_1_OriginPosition, varargin_1_Orientation, varargin_1_HasVelocity,
    varargin_1_OriginVelocity, varargin_1_IsParentToChild, varargin_1_HasAzimuth,
    varargin_1_HasElevation, varargin_1_HasRange, &expl_temp);
  AEBSensorFusion_get_match(expl_temp.f1.data, expl_temp.f1.size, frame_data,
    frame_size, &measSize);
  if (measSize == 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    frame_size[0] = 1;
    frame_size[1] = 7;
    for (tgtpos_tmp = 0; tgtpos_tmp < 7; tgtpos_tmp++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      frame_data[tgtpos_tmp] = ' ';
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (measSize = 0; measSize < 3; measSize++) {
    hypotxy = AEBSensorFusion_norm(&expl_temp.f4[3 * measSize]);
    expl_temp.f4[3 * measSize] /= hypotxy;
    tgtpos_tmp = 3 * measSize + 1;
    expl_temp.f4[tgtpos_tmp] /= hypotxy;
    tgtpos_tmp = 3 * measSize + 2;
    expl_temp.f4[tgtpos_tmp] /= hypotxy;
    tgtpos_tmp = measSize << 1;
    tgtpos[measSize] = state[tgtpos_tmp];
    tgtvel[measSize] = state[tgtpos_tmp + 1];
  }

  if (AEBSensorFusion_strcmp(frame_data, frame_size)) {
    if (!expl_temp.f7) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      hypotxy = tgtpos[0] - expl_temp.f2[0];
      tgtpos_idx_1 = tgtpos[1] - expl_temp.f2[1];
      tgtpos_idx_2 = tgtpos[2] - expl_temp.f2[2];
      measurement_size[0] = 3;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = (expl_temp.f4[3 * tgtpos_tmp + 1] *
          tgtpos_idx_1 + expl_temp.f4[3 * tgtpos_tmp] * hypotxy) + expl_temp.f4
          [3 * tgtpos_tmp + 2] * tgtpos_idx_2;
      }
    } else {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_tmp[3 * tgtpos_tmp] = expl_temp.f4[tgtpos_tmp];
        measurement_tmp[3 * tgtpos_tmp + 1] = expl_temp.f4[tgtpos_tmp + 3];
        measurement_tmp[3 * tgtpos_tmp + 2] = expl_temp.f4[tgtpos_tmp + 6];
        lclRect[tgtpos_tmp] = tgtpos[tgtpos_tmp] - expl_temp.f2[tgtpos_tmp];
      }

      hypotxy = lclRect[1];
      tgtpos_idx_1 = lclRect[0];
      tgtpos_idx_2 = lclRect[2];
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        tgtpos[tgtpos_tmp] = (measurement_tmp[tgtpos_tmp + 3] * hypotxy +
                              measurement_tmp[tgtpos_tmp] * tgtpos_idx_1) +
          measurement_tmp[tgtpos_tmp + 6] * tgtpos_idx_2;
        lclRect[tgtpos_tmp] = tgtvel[tgtpos_tmp] - expl_temp.f3[tgtpos_tmp];
      }

      measurement_size[0] = 6;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      hypotxy = lclRect[1];
      tgtpos_idx_1 = lclRect[0];
      tgtpos_idx_2 = lclRect[2];
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = tgtpos[tgtpos_tmp];
        measurement_data[tgtpos_tmp + 3] = (measurement_tmp[tgtpos_tmp + 3] *
          hypotxy + measurement_tmp[tgtpos_tmp] * tgtpos_idx_1) +
          measurement_tmp[tgtpos_tmp + 6] * tgtpos_idx_2;
      }
    }

    if (expl_temp.f7) {
      bounds_size[0] = 6;
      bounds_size[1] = 2;
      std::memcpy(&bounds_data[0], &tmp_2[0], 12U * sizeof(real_T));
    } else {
      bounds_size[0] = 3;
      bounds_size[1] = 2;
      for (tgtpos_tmp = 0; tgtpos_tmp < 6; tgtpos_tmp++) {
        bounds_data[tgtpos_tmp] = tmp_1[tgtpos_tmp];
      }
    }
  } else {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    hasRangeRate_tmp = (expl_temp.f7 && expl_temp.f8);
    measSize = ((expl_temp.f5 + expl_temp.f6) + expl_temp.f8) + hasRangeRate_tmp;
    measurement_size[0] = measSize;
    if (measSize - 1 >= 0) {
      std::memset(&measurement_data[0], 0, static_cast<uint32_T>(measSize) *
                  sizeof(real_T));
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    hypotxy = tgtpos[0] - expl_temp.f2[0];
    tgtpos_idx_1 = tgtpos[1] - expl_temp.f2[1];
    tgtpos_idx_2 = tgtpos[2] - expl_temp.f2[2];
    for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      lclRect[tgtpos_tmp] = (expl_temp.f4[3 * tgtpos_tmp + 1] * tgtpos_idx_1 +
        expl_temp.f4[3 * tgtpos_tmp] * hypotxy) + expl_temp.f4[3 * tgtpos_tmp +
        2] * tgtpos_idx_2;
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    hypotxy = rt_hypotd_snf(lclRect[0], lclRect[1]);
    meas[0] = rt_atan2d_snf(lclRect[1], lclRect[0]);
    meas[1] = rt_atan2d_snf(lclRect[2], hypotxy);
    meas[2] = rt_hypotd_snf(hypotxy, lclRect[2]);
    meas[0] *= 57.295779513082323;
    meas[1] *= 57.295779513082323;
    if (!hasRangeRate_tmp) {
      tmp[0] = expl_temp.f5;
      tmp[1] = expl_temp.f6;
      tmp[2] = expl_temp.f8;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          trueCount++;
        }
      }

      measSize = trueCount;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          tmp_data[trueCount] = static_cast<int8_T>(tgtpos_tmp);
          trueCount++;
        }
      }

      measurement_size[0] = measSize;
      for (tgtpos_tmp = 0; tgtpos_tmp < measSize; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = meas[tmp_data[tgtpos_tmp]];
      }
    } else {
      if (measSize - 1 < 1) {
        measSize = 0;
      } else {
        measSize--;
      }

      tmp[0] = expl_temp.f5;
      tmp[1] = expl_temp.f6;
      tmp[2] = expl_temp.f8;
      trueCount = 0;
      for (tgtpos_tmp = 0; tgtpos_tmp < 3; tgtpos_tmp++) {
        if (tmp[tgtpos_tmp]) {
          tmp_data[trueCount] = static_cast<int8_T>(tgtpos_tmp);
          trueCount++;
        }
      }

      for (tgtpos_tmp = 0; tgtpos_tmp < measSize; tgtpos_tmp++) {
        measurement_data[tgtpos_tmp] = meas[tmp_data[tgtpos_tmp]];
      }

      measurement_data[measurement_size[0] - 1] = -AEBSensorFusion_radialspeed
        (tgtpos, tgtvel, expl_temp.f2, expl_temp.f3);
    }

    tmp_0[0] = expl_temp.f5;
    tmp_0[1] = expl_temp.f6;
    tmp_0[2] = expl_temp.f8;
    tmp_0[3] = hasRangeRate_tmp;
    trueCount = 0;
    for (tgtpos_tmp = 0; tgtpos_tmp < 4; tgtpos_tmp++) {
      if (tmp_0[tgtpos_tmp]) {
        trueCount++;
      }
    }

    measSize = trueCount;
    trueCount = 0;
    for (tgtpos_tmp = 0; tgtpos_tmp < 4; tgtpos_tmp++) {
      if (tmp_0[tgtpos_tmp]) {
        tmp_data_0[trueCount] = static_cast<int8_T>(tgtpos_tmp);
        trueCount++;
      }
    }

    bounds_size[0] = measSize;
    bounds_size[1] = 2;
    for (tgtpos_tmp = 0; tgtpos_tmp < 2; tgtpos_tmp++) {
      for (trueCount = 0; trueCount < measSize; trueCount++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        bounds_data[trueCount + measSize * tgtpos_tmp] = tmp_3[(tgtpos_tmp << 2)
          + tmp_data_0[trueCount]];
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_13(real_T
  in1[36], const real_T in2_data[], const int32_T in2_size[2], const real_T in3
  [36])
{
  real_T in3_0[36];
  int32_T aux_0_1;
  int32_T i_1;
  int32_T stride_0_0;
  int32_T stride_0_1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (int32_T i = 0; i < 6; i++) {
    for (int32_T i_0 = 0; i_0 < 6; i_0++) {
      real_T in3_1;
      in3_1 = 0.0;
      for (i_1 = 0; i_1 < 6; i_1++) {
        in3_1 += in3[6 * i_1 + i] * in3[6 * i_1 + i_0];
      }

      in3_0[i + 6 * i_0] = in3_1;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_0 = (in2_size[0] != 1);
  stride_0_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  for (int32_T i = 0; i < 6; i++) {
    for (int32_T i_0 = 0; i_0 < 6; i_0++) {
      i_1 = 6 * i + i_0;
      in1[i_1] = in2_data[i_0 * stride_0_0 + in2_size[0] * aux_0_1] + in3_0[i_1];
    }

    aux_0_1 += stride_0_1;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_12(real_T
  in1[6], const real_T in2[6], const real_T in3_data[], const int32_T in3_size[1])
{
  int32_T stride_0_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_0 = (in3_size[0] != 1);
  for (int32_T i = 0; i < 6; i++) {
    in1[i] = in2[i] - in3_data[i * stride_0_0];
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_all(const boolean_T x_data[],
  const int32_T x_size[2], boolean_T y_data[], int32_T y_size[1])
{
  int32_T b;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  loop_ub = x_size[0];
  y_size[0] = x_size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    y_data[i1] = true;
  }

  i1 = 0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  i2 = x_size[0];
  b = x_size[0] - 1;
  for (int32_T j = 0; j <= b; j++) {
    int32_T ix;
    boolean_T exitg1;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((loop_ub > 0) && (ix <= i2))) {
      if (!x_data[ix - 1]) {
        y_data[j] = false;
        exitg1 = true;
      } else {
        ix += loop_ub;
      }
    }
  }
}

boolean_T ACCWithSensorFusionModelClass::AEBSensorFusion_any(const boolean_T
  x_data[], const int32_T x_size[1])
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x_size[0])) {
    if (x_data[ix]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_plus(real_T in1[6], const
  real_T in2_data[], const int32_T in2_size[1])
{
  int32_T stride_0_0;
  stride_0_0 = (in2_size[0] != 1);
  for (int32_T i = 0; i < 6; i++) {
    in1[i] += in2_data[i * stride_0_0];
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_2(real_T
  in1[6], const real_T in2_data[], const int32_T in2_size[2], const real_T
  in3_data[], const int32_T in3_size[1])
{
  real_T in2_data_0[36];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  loop_ub = in3_size[0] == 1 ? in2_size[0] : in3_size[0];
  stride_0_1 = (in2_size[0] != 1);
  stride_1_1 = (in3_size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (int32_T i_0 = 0; i_0 < loop_ub; i_0++) {
    for (int32_T i = 0; i < 6; i++) {
      in2_data_0[i + 6 * i_0] = in2_data[in2_size[0] * i + aux_0_1] +
        in3_data[aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  for (int32_T i_0 = 0; i_0 < 6; i_0++) {
    in1[i_0] = in2_data_0[i_0];
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_minus(real_T in1_data[],
  int32_T in1_size[1], const real_T in2_data[], const int32_T in2_size[1], const
  real_T in3_data[], const int32_T in3_size[1])
{
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  loop_ub = in3_size[0] == 1 ? in2_size[0] : in3_size[0];
  in1_size[0] = loop_ub;
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (in3_size[0] != 1);
  for (int32_T i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] - in3_data[i * stride_1_0];
  }
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_normalizedDistance(const
  real_T z_data[], const int32_T z_size[1], const real_T mu_data[], const
  int32_T mu_size[1], const real_T sigma[36])
{
  real_T c_A[36];
  real_T b_X_data[6];
  real_T d[6];
  real_T zd_data[6];
  real_T b_X;
  real_T f;
  real_T temp;
  int32_T b_ipiv[6];
  int32_T zd_size[1];
  int32_T b_info;
  int32_T b_k;
  int32_T jAcol;
  boolean_T isodd;
  if (z_size[0] == mu_size[0]) {
    jAcol = z_size[0];
    zd_size[0] = z_size[0];
    for (b_info = 0; b_info < jAcol; b_info++) {
      zd_data[b_info] = z_data[b_info] - mu_data[b_info];
    }
  } else {
    AEBSensorFusion_minus(zd_data, zd_size, z_data, z_size, mu_data, mu_size);
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  std::memcpy(&c_A[0], &sigma[0], 36U * sizeof(real_T));
  AEBSensorFusion_xzgetrf(c_A, b_ipiv, &b_info);
  jAcol = zd_size[0];
  if (jAcol - 1 >= 0) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    std::memcpy(&b_X_data[0], &zd_data[0], static_cast<uint32_T>(jAcol) * sizeof
                (real_T));
  }

  for (b_info = 0; b_info < 6; b_info++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    jAcol = 6 * b_info - 1;
    for (b_k = 0; b_k < b_info; b_k++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      temp = c_A[(b_k + jAcol) + 1];
      if (temp != 0.0) {
        b_X_data[b_info] -= temp * b_X_data[b_k];
      }
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_X_data[b_info] *= 1.0 / c_A[(b_info + jAcol) + 1];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (b_info = 5; b_info >= 0; b_info--) {
    jAcol = 6 * b_info - 1;
    for (b_k = b_info + 2; b_k < 7; b_k++) {
      temp = c_A[b_k + jAcol];
      if (temp != 0.0) {
        b_X_data[b_info] -= b_X_data[b_k - 1] * temp;
      }
    }
  }

  for (b_info = 4; b_info >= 0; b_info--) {
    jAcol = b_ipiv[b_info];
    if (b_info + 1 != jAcol) {
      temp = b_X_data[b_info];
      b_X_data[b_info] = b_X_data[jAcol - 1];
      b_X_data[jAcol - 1] = temp;
    }
  }

  std::memcpy(&c_A[0], &sigma[0], 36U * sizeof(real_T));
  AEBSensorFusion_xzgetrf(c_A, b_ipiv, &b_info);
  temp = c_A[0];
  isodd = false;
  for (b_k = 0; b_k < 5; b_k++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    temp *= c_A[((b_k + 1) * 6 + b_k) + 1];
    if (b_ipiv[b_k] > b_k + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    temp = -temp;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  temp = std::log(temp);
  if (rtIsInf(temp) || rtIsNaN(temp)) {
    AEBSensorFusion_cholPSD(sigma, c_A);
    for (b_k = 0; b_k < 6; b_k++) {
      d[b_k] = std::log(c_A[6 * b_k + b_k]);
    }

    temp = d[0];
    for (b_k = 0; b_k < 5; b_k++) {
      temp += d[b_k + 1];
    }

    temp *= 2.0;
  }

  b_X = 0.0;
  for (b_info = 0; b_info < 6; b_info++) {
    b_X += b_X_data[b_info] * zd_data[b_info];
  }

  f = b_X + temp;
  return f;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_10(real_T
  in1[6], const real_T in2_data[], const int32_T in2_size[2])
{
  int32_T stride_0_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_0 = (in2_size[0] != 1);
  for (int32_T i = 0; i < 6; i++) {
    in1[i] /= in2_data[i * stride_0_0 + in2_size[0]];
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_9(boolean_T
  in1[6], const boolean_T in2_data[], const int32_T in2_size[1], const real_T
  in3[6])
{
  int32_T stride_0_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_0 = (in2_size[0] != 1);
  for (int32_T i = 0; i < 6; i++) {
    in1[i] = (in2_data[i * stride_0_0] && (in3[i] > 0.001));
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_6(real_T
  in1_data[], int32_T in1_size[1], const real_T in2_data[], const int32_T
  in2_size[2])
{
  real_T in1_data_0[6];
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  loop_ub = in2_size[0] == 1 ? in1_size[0] : in2_size[0];
  stride_0_0 = (in1_size[0] != 1);
  stride_1_0 = (in2_size[0] != 1);
  for (int32_T i = 0; i < loop_ub; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    in1_data_0[i] = in1_data[i * stride_0_0] / in2_data[i * stride_1_0 +
      in2_size[0]];
  }

  in1_size[0] = loop_ub;
  if (loop_ub - 1 >= 0) {
    std::memcpy(&in1_data[0], &in1_data_0[0], static_cast<uint32_T>(loop_ub) *
                sizeof(real_T));
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_5(boolean_T
  in1_data[], int32_T in1_size[1], const boolean_T in2_data[], const int32_T
  in2_size[1], const real_T in3_data[], const int32_T in3_size[1])
{
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  loop_ub = in3_size[0] == 1 ? in2_size[0] : in3_size[0];
  in1_size[0] = loop_ub;
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (in3_size[0] != 1);
  for (int32_T i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] && (in3_data[i * stride_1_0] > 0.001));
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_mod(real_T x, real_T y)
{
  real_T r;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  r = x;
  if (y == 0.0) {
    if (x == 0.0) {
      r = y;
    }
  } else if (rtIsNaN(x) || rtIsNaN(y) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0 / y;
  } else if (rtIsInf(y)) {
    if ((y < 0.0) != (x < 0.0)) {
      r = y;
    }
  } else {
    boolean_T rEQ0;
    r = std::fmod(x, y);
    rEQ0 = (r == 0.0);
    if ((!rEQ0) && (y > std::floor(y))) {
      real_T q;
      q = std::abs(x / y);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = y * 0.0;
    } else if (((r < 0.0) && (!(y < 0.0))) || ((!(r < 0.0)) && (y < 0.0))) {
      r += y;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return r;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_8(real_T
  in1[6], const int32_T in2_data[], const int32_T in2_size[1], const real_T
  in3_data[], const int32_T in3_size[1], const real_T in4_data[])
{
  int32_T in2_idx_0;
  int32_T stride_0_0;
  int32_T stride_1_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  in2_idx_0 = in2_size[0];
  stride_0_0 = (in3_size[0] != 1);
  stride_1_0 = (in2_size[0] != 1);
  for (int32_T i = 0; i < in2_idx_0; i++) {
    in1[in2_data[i]] = in3_data[i * stride_0_0] + in4_data[in2_data[i *
      stride_1_0]];
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_expand_mod(const real_T
  a_data[], const int32_T a_size[2], const real_T b_data[], const int32_T
  b_size[1], real_T c_data[], int32_T c_size[2])
{
  int32_T csz_idx_0;
  int32_T f;
  int32_T k;
  boolean_T d;
  boolean_T e;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (b_size[0] == 1) {
    csz_idx_0 = a_size[0];
  } else if (a_size[0] == 1) {
    csz_idx_0 = b_size[0];
  } else if (a_size[0] <= b_size[0]) {
    csz_idx_0 = a_size[0];
  } else {
    csz_idx_0 = b_size[0];
  }

  c_size[0] = csz_idx_0;
  c_size[1] = 6;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (csz_idx_0 != 0) {
    d = (a_size[0] != 1);
    e = (b_size[0] != 1);
    for (csz_idx_0 = 0; csz_idx_0 < 6; csz_idx_0++) {
      f = c_size[0];
      for (k = 0; k < f; k++) {
        c_data[k + c_size[0] * csz_idx_0] = AEBSensorFusion_mod(a_data[d * k +
          a_size[0] * csz_idx_0], b_data[e * k]);
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_3(real_T
  in1_data[], int32_T in1_size[2], const real_T in2_data[], const int32_T
  in2_size[2], const int32_T in3_data[], const int32_T in3_size[1], const real_T
  in4_data[], const int32_T in4_size[2])
{
  real_T in2_data_0[36];
  real_T in4_data_0[6];
  int32_T in2_size_0[2];
  int32_T in4_size_0[1];
  int32_T i;
  int32_T i_0;
  int32_T in2_tmp;
  int32_T in3_tmp;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  in3_tmp = in3_size[0];
  in2_size_0[0] = in3_size[0];
  in2_size_0[1] = 6;
  stride_0_0_tmp = (in3_size[0] != 1);
  loop_ub = in3_size[0];
  for (i_0 = 0; i_0 < 6; i_0++) {
    for (i = 0; i < loop_ub; i++) {
      in2_tmp = in3_data[i * stride_0_0_tmp];
      in2_data_0[i + in2_size_0[0] * i_0] = in2_data[in2_size[0] * i_0 + in2_tmp]
        - in4_data[in2_tmp];
    }
  }

  in4_size_0[0] = in3_size[0];
  for (i_0 = 0; i_0 < in3_tmp; i_0++) {
    in4_data_0[i_0] = in4_data[in3_data[i_0] + in4_size[0]] -
      in4_data[in3_data[i_0]];
  }

  AEBSensorFusion_expand_mod(in2_data_0, in2_size_0, in4_data_0, in4_size_0,
    in1_data, in1_size);

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_4(real_T
  in1_data[], const int32_T in1_size[2], const int32_T in2_data[], const int32_T
  in2_size[1], const real_T in3_data[], const int32_T in3_size[2])
{
  real_T in1_data_0[36];
  real_T tmp_data[36];
  real_T in3_data_0[6];
  int32_T in1_size_0[2];
  int32_T tmp_size[2];
  int32_T in3_size_0[1];
  int32_T i;
  int32_T i_0;
  int32_T in1_tmp;
  int32_T in2_idx_0_tmp;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  in2_idx_0_tmp = in2_size[0];
  in1_size_0[0] = in2_size[0];
  in1_size_0[1] = 6;
  stride_0_0_tmp = (in2_size[0] != 1);
  loop_ub = in2_size[0];
  for (i_0 = 0; i_0 < 6; i_0++) {
    for (i = 0; i < loop_ub; i++) {
      in1_tmp = in2_data[i * stride_0_0_tmp];
      in1_data_0[i + in1_size_0[0] * i_0] = in1_data[in1_size[0] * i_0 + in1_tmp]
        - in3_data[in1_tmp];
    }
  }

  in3_size_0[0] = in2_size[0];
  for (i_0 = 0; i_0 < in2_idx_0_tmp; i_0++) {
    in3_data_0[i_0] = in3_data[in2_data[i_0] + in3_size[0]] -
      in3_data[in2_data[i_0]];
  }

  AEBSensorFusion_expand_mod(in1_data_0, in1_size_0, in3_data_0, in3_size_0,
    tmp_data, tmp_size);
  loop_ub = (tmp_size[0] != 1);
  for (i_0 = 0; i_0 < 6; i_0++) {
    for (i = 0; i < in2_idx_0_tmp; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      in1_data[in2_data[i] + in1_size[0] * i_0] = tmp_data[i * loop_ub +
        tmp_size[0] * i_0] + in3_data[in2_data[i * stride_0_0_tmp]];
    }
  }
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_ObjectTrack_calcCostOneDetection
  (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track, const
   real_T detection_Measurement[6], drivingCoordinateFrameType
   detection_MeasurementParameters_Frame, const real_T
   detection_MeasurementParameters_OriginPosition[3], const real_T
   detection_MeasurementParameters_Orientation[9], boolean_T
   detection_MeasurementParameters_HasVelocity, const real_T
   detection_MeasurementParameters_OriginVelocity[3], boolean_T
   detection_MeasurementParameters_IsParentToChild, boolean_T
   detection_MeasurementParameters_HasAzimuth, boolean_T
   detection_MeasurementParameters_HasElevation, boolean_T
   detection_MeasurementParameters_HasRange, real_T costValue_data[], int32_T
   costValue_size[2])
{
  c_trackingEKF_AEBSensorFusion_T *EKF;
  real_T b[36];
  real_T dHdx_data[36];
  real_T resToWrap_data[36];
  real_T residualCovariance[36];
  real_T wrapping_data[12];
  real_T wrapping_data_0[12];
  real_T ma[6];
  real_T ma_data[6];
  real_T resToWrap_data_0[6];
  real_T residual[6];
  real_T zEstimated_data[6];
  real_T s;
  int32_T b_tmp_data[6];
  int32_T m_data[6];
  int32_T b_i;
  int32_T b_j;
  int32_T boffset;
  int32_T coffset;
  int32_T i;
  int32_T m_tmp;
  boolean_T tmp_data_0[12];
  boolean_T isf_data[6];
  boolean_T tmp_data[6];
  boolean_T p;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T b_size[2];
  int32_T dHdx_size[2];
  int32_T resToWrap_size[2];
  int32_T resToWrap_size_1[2];
  int32_T tmp_size_0[2];
  int32_T wrapping_size[2];
  int32_T b_tmp_size[1];
  int32_T isf_size[1];
  int32_T m_size[1];
  int32_T ma_size[1];
  int32_T resToWrap_size_0[1];
  int32_T tmp_size[1];
  int32_T zEstimated_size[1];
  EKF = track->pDistanceFilter;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((!EKF->pIsSetStateCovariance) || (EKF->pSqrtStateCovarianceScalar != -1.0))
  {
    s = EKF->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      EKF->pSqrtStateCovariance[i] = s * static_cast<real_T>(tmp[i]);
    }

    EKF->pIsSetStateCovariance = true;
    EKF->pSqrtStateCovarianceScalar = -1.0;
  }

  if (EKF->pIsFirstCallCorrect) {
    if (!EKF->pIsValidMeasurementFcn) {
      EKF->pIsValidMeasurementFcn = true;
    }

    EKF->pIsFirstCallCorrect = false;
  }

  if (EKF->pSqrtMeasurementNoiseScalar > 0.0) {
    s = EKF->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 36; i++) {
      EKF->pSqrtMeasurementNoise[i] = s * static_cast<real_T>(tmp[i]);
    }

    EKF->pSqrtMeasurementNoiseScalar = -1.0;
  }

  for (i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    ma_data[i] = EKF->pState[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cvmeasjac(ma_data, detection_MeasurementParameters_Frame,
    detection_MeasurementParameters_OriginPosition,
    detection_MeasurementParameters_Orientation,
    detection_MeasurementParameters_HasVelocity,
    detection_MeasurementParameters_OriginVelocity,
    detection_MeasurementParameters_IsParentToChild,
    detection_MeasurementParameters_HasAzimuth,
    detection_MeasurementParameters_HasElevation,
    detection_MeasurementParameters_HasRange, dHdx_data, dHdx_size);
  for (i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    ma_data[i] = EKF->pState[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cvmeas_h(ma_data, detection_MeasurementParameters_Frame,
    detection_MeasurementParameters_OriginPosition,
    detection_MeasurementParameters_Orientation,
    detection_MeasurementParameters_HasVelocity,
    detection_MeasurementParameters_OriginVelocity,
    detection_MeasurementParameters_IsParentToChild,
    detection_MeasurementParameters_HasAzimuth,
    detection_MeasurementParameters_HasElevation,
    detection_MeasurementParameters_HasRange, zEstimated_data, zEstimated_size,
    wrapping_data, wrapping_size);
  if ((!EKF->pIsSetStateCovariance) || (EKF->pSqrtStateCovarianceScalar != -1.0))
  {
    s = EKF->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      EKF->pSqrtStateCovariance[i] = s * static_cast<real_T>(tmp[i]);
    }

    EKF->pIsSetStateCovariance = true;
    EKF->pSqrtStateCovarianceScalar = -1.0;
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    resToWrap_data[i] = EKF->pSqrtStateCovariance[i];
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    residualCovariance[i] = EKF->pSqrtStateCovariance[i];
  }

  for (i = 0; i < 6; i++) {
    for (boffset = 0; boffset < 6; boffset++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      s = 0.0;
      for (b_i = 0; b_i < 6; b_i++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        s += residualCovariance[6 * b_i + i] * resToWrap_data[6 * b_i + boffset];
      }

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b[i + 6 * boffset] = s;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  m_tmp = dHdx_size[0];
  resToWrap_size[0] = dHdx_size[0];
  for (b_j = 0; b_j < 6; b_j++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    coffset = b_j * m_tmp - 1;
    boffset = b_j * 6 - 1;
    for (b_i = 0; b_i < m_tmp; b_i++) {
      s = 0.0;
      for (i = 0; i < 6; i++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        s += dHdx_data[i * dHdx_size[0] + b_i] * b[(boffset + i) + 1];
      }

      resToWrap_data[(coffset + b_i) + 1] = s;
    }
  }

  b_size[0] = dHdx_size[0];
  b_size[1] = dHdx_size[0];
  for (b_j = 0; b_j < m_tmp; b_j++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    coffset = b_j * m_tmp - 1;
    for (b_i = 0; b_i < m_tmp; b_i++) {
      s = 0.0;
      for (i = 0; i < 6; i++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        boffset = i * resToWrap_size[0];
        s += resToWrap_data[boffset + b_i] * dHdx_data[boffset + b_j];
      }

      b[(coffset + b_i) + 1] = s;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (EKF->pSqrtMeasurementNoiseScalar > 0.0) {
    s = EKF->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 36; i++) {
      EKF->pSqrtMeasurementNoise[i] = s * static_cast<real_T>(tmp[i]);
    }

    EKF->pSqrtMeasurementNoiseScalar = -1.0;
  }

  for (i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    resToWrap_data[i] = EKF->pSqrtMeasurementNoise[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (dHdx_size[0] == 6) {
    for (i = 0; i < 6; i++) {
      for (boffset = 0; boffset < 6; boffset++) {
        s = 0.0;
        for (b_i = 0; b_i < 6; b_i++) {
          s += resToWrap_data[6 * b_i + i] * resToWrap_data[6 * b_i + boffset];
        }

        residualCovariance[i + 6 * boffset] = b[b_size[0] * boffset + i] + s;
      }
    }
  } else {
    AEBSensorFusion_binary_expand_op_13(residualCovariance, b, b_size,
      resToWrap_data);
  }

  if (zEstimated_size[0] == 1) {
    resToWrap_size[0] = 1;
    resToWrap_size[1] = 6;
    for (i = 0; i < 6; i++) {
      resToWrap_data[i] = detection_Measurement[i] - zEstimated_data[0];
    }

    tmp_size_0[0] = wrapping_size[0];
    tmp_size_0[1] = 2;
    b_i = wrapping_size[0] << 1;
    for (i = 0; i < b_i; i++) {
      tmp_data_0[i] = ((!rtIsInf(wrapping_data[i])) && (!rtIsNaN(wrapping_data[i])));
    }

    AEBSensorFusion_all(tmp_data_0, tmp_size_0, isf_data, isf_size);
    if (AEBSensorFusion_any(isf_data, isf_size)) {
      b_i = isf_size[0];
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (isf_data[i]) {
          b_j++;
        }
      }

      b_tmp_size[0] = b_j;
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (isf_data[i]) {
          b_tmp_data[b_j] = i;
          b_j++;
        }
      }

      b_j = b_tmp_size[0];
      for (i = 0; i < b_j; i++) {
        boffset = b_tmp_data[i];
        ma_data[i] = (wrapping_data[boffset + wrapping_size[0]] +
                      wrapping_data[boffset]) / 2.0;
      }

      m_tmp = b_tmp_size[0];
      for (i = 0; i < 2; i++) {
        for (boffset = 0; boffset < b_j; boffset++) {
          wrapping_data_0[boffset + m_tmp * i] = wrapping_data[wrapping_size[0] *
            i + b_tmp_data[boffset]] - ma_data[boffset];
        }
      }

      for (i = 0; i < 2; i++) {
        for (boffset = 0; boffset < b_j; boffset++) {
          wrapping_data[b_tmp_data[boffset] + wrapping_size[0] * i] =
            wrapping_data_0[m_tmp * i + boffset];
        }
      }

      dHdx_size[0] = 1;
      dHdx_size[1] = 6;
      for (i = 0; i < 6; i++) {
        dHdx_data[i] = std::abs(resToWrap_data[i]);
      }

      ma_size[0] = 1;
      ma_data[0] = dHdx_data[0];
      for (boffset = 0; boffset < 5; boffset++) {
        s = dHdx_data[boffset + 1];
        for (b_j = 0; b_j < 1; b_j++) {
          if (rtIsNaN(s)) {
            p = false;
          } else {
            p = (rtIsNaN(ma_data[0]) || (ma_data[0] < s));
          }

          if (p) {
            ma_data[0] = s;
          }
        }
      }

      if (wrapping_size[0] == 1) {
        s = ma_data[0];
        for (i = 0; i < 1; i++) {
          s /= wrapping_data[wrapping_size[0]];
        }

        ma_data[0] = s;
      } else {
        AEBSensorFusion_binary_expand_op_6(ma_data, ma_size, wrapping_data,
          wrapping_size);
      }

      b_j = ma_size[0];
      resToWrap_size_0[0] = ma_size[0];
      for (i = 0; i < b_j; i++) {
        resToWrap_data_0[i] = std::abs(ma_data[i]);
      }

      if (isf_size[0] == ma_size[0]) {
        tmp_size[0] = isf_size[0];
        for (i = 0; i < b_i; i++) {
          tmp_data[i] = (isf_data[i] && (resToWrap_data_0[i] > 0.001));
        }
      } else {
        AEBSensorFusion_binary_expand_op_5(tmp_data, tmp_size, isf_data,
          isf_size, resToWrap_data_0, resToWrap_size_0);
      }

      b_i = tmp_size[0];
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (tmp_data[i]) {
          b_j++;
        }
      }

      m_size[0] = b_j;
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (tmp_data[i]) {
          m_data[b_j] = i;
          b_j++;
        }
      }

      AEBSensorFusion_binary_expand_op_3(dHdx_data, dHdx_size, resToWrap_data,
        resToWrap_size, m_data, m_size, wrapping_data, wrapping_size);
      if (dHdx_size[0] == m_size[0]) {
        b_j = m_size[0];
        resToWrap_size_1[0] = m_size[0];
        resToWrap_size_1[1] = 6;
        for (i = 0; i < 6; i++) {
          for (boffset = 0; boffset < b_j; boffset++) {
            b_i = m_data[boffset];
            b[boffset + resToWrap_size_1[0] * i] = resToWrap_data[b_i + i] -
              wrapping_data[b_i];
          }
        }

        ma_size[0] = m_size[0];
        for (i = 0; i < b_j; i++) {
          boffset = m_data[i];
          ma_data[i] = wrapping_data[boffset + wrapping_size[0]] -
            wrapping_data[boffset];
        }

        AEBSensorFusion_expand_mod(b, resToWrap_size_1, ma_data, ma_size,
          dHdx_data, dHdx_size);
        for (i = 0; i < 6; i++) {
          for (boffset = 0; boffset < b_j; boffset++) {
            b_i = m_data[boffset];
            resToWrap_data[b_i + i] = dHdx_data[dHdx_size[0] * i + boffset] +
              wrapping_data[b_i];
          }
        }
      } else {
        AEBSensorFusion_binary_expand_op_4(resToWrap_data, resToWrap_size,
          m_data, m_size, wrapping_data, wrapping_size);
      }
    }

    if (resToWrap_size[0] == 1) {
      for (i = 0; i < 6; i++) {
        b[i] = resToWrap_data[resToWrap_size[0] * i] + zEstimated_data[0];
      }

      for (i = 0; i < 6; i++) {
        residual[i] = b[i];
      }
    } else {
      AEBSensorFusion_binary_expand_op_2(residual, resToWrap_data,
        resToWrap_size, zEstimated_data, zEstimated_size);
    }
  } else {
    if (zEstimated_size[0] == 6) {
      for (i = 0; i < 6; i++) {
        residual[i] = detection_Measurement[i] - zEstimated_data[i];
      }
    } else {
      AEBSensorFusion_binary_expand_op_12(residual, detection_Measurement,
        zEstimated_data, zEstimated_size);
    }

    tmp_size_0[0] = wrapping_size[0];
    tmp_size_0[1] = 2;
    b_i = wrapping_size[0] << 1;
    for (i = 0; i < b_i; i++) {
      tmp_data_0[i] = ((!rtIsInf(wrapping_data[i])) && (!rtIsNaN(wrapping_data[i])));
    }

    AEBSensorFusion_all(tmp_data_0, tmp_size_0, isf_data, isf_size);
    if (AEBSensorFusion_any(isf_data, isf_size)) {
      b_i = isf_size[0];
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (isf_data[i]) {
          b_j++;
        }
      }

      b_tmp_size[0] = b_j;
      b_j = 0;
      for (i = 0; i < b_i; i++) {
        if (isf_data[i]) {
          b_tmp_data[b_j] = i;
          b_j++;
        }
      }

      b_j = b_tmp_size[0];
      for (i = 0; i < b_j; i++) {
        boffset = b_tmp_data[i];
        ma_data[i] = (wrapping_data[boffset + wrapping_size[0]] +
                      wrapping_data[boffset]) / 2.0;
      }

      m_tmp = b_tmp_size[0];
      for (i = 0; i < 2; i++) {
        for (boffset = 0; boffset < b_j; boffset++) {
          wrapping_data_0[boffset + m_tmp * i] = wrapping_data[wrapping_size[0] *
            i + b_tmp_data[boffset]] - ma_data[boffset];
        }
      }

      for (i = 0; i < 2; i++) {
        for (boffset = 0; boffset < b_j; boffset++) {
          wrapping_data[b_tmp_data[boffset] + wrapping_size[0] * i] =
            wrapping_data_0[m_tmp * i + boffset];
        }
      }

      for (i = 0; i < 6; i++) {
        ma[i] = std::abs(residual[i]);
      }

      if (wrapping_size[0] == 6) {
        for (i = 0; i < 6; i++) {
          ma[i] /= wrapping_data[i + wrapping_size[0]];
        }
      } else {
        AEBSensorFusion_binary_expand_op_10(ma, wrapping_data, wrapping_size);
      }

      for (i = 0; i < 6; i++) {
        ma_data[i] = std::abs(ma[i]);
      }

      if (isf_size[0] == 6) {
        for (i = 0; i < 6; i++) {
          tmp_data[i] = (isf_data[i] && (ma_data[i] > 0.001));
        }
      } else {
        AEBSensorFusion_binary_expand_op_9(tmp_data, isf_data, isf_size, ma_data);
      }

      b_j = 0;
      for (i = 0; i < 6; i++) {
        if (tmp_data[i]) {
          b_j++;
        }
      }

      m_size[0] = b_j;
      b_j = 0;
      for (i = 0; i < 6; i++) {
        if (tmp_data[i]) {
          m_data[b_j] = i;
          b_j++;
        }
      }

      b_j = m_size[0];
      for (i = 0; i < b_j; i++) {
        b_i = m_data[i];
        s = wrapping_data[b_i];
        ma_data[i] = residual[b_i] - s;
        ma[i] = wrapping_data[b_i + wrapping_size[0]] - s;
      }

      resToWrap_size_0[0] = m_size[0];
      for (i = 0; i < b_j; i++) {
        resToWrap_data_0[i] = AEBSensorFusion_mod(ma_data[i], ma[i]);
      }

      if (resToWrap_size_0[0] == m_size[0]) {
        for (i = 0; i < b_j; i++) {
          b_i = m_data[i];
          residual[b_i] = resToWrap_data_0[i] + wrapping_data[b_i];
        }
      } else {
        AEBSensorFusion_binary_expand_op_8(residual, m_data, m_size,
          resToWrap_data_0, resToWrap_size_0, wrapping_data);
      }
    }

    if (zEstimated_size[0] == 6) {
      for (i = 0; i < 6; i++) {
        residual[i] += zEstimated_data[i];
      }
    } else {
      AEBSensorFusion_plus(residual, zEstimated_data, zEstimated_size);
    }
  }

  b_j = zEstimated_size[0];
  if (zEstimated_size[0] == 1) {
    costValue_size[0] = 1;
    costValue_size[1] = 6;
    for (i = 0; i < 6; i++) {
      costValue_data[i] = 0.0;
    }

    for (boffset = 0; boffset < 6; boffset++) {
      ma_size[0] = b_j;
      for (i = 0; i < b_j; i++) {
        ma_data[i] = residual[boffset];
      }

      costValue_data[boffset] = AEBSensorFusion_normalizedDistance(ma_data,
        ma_size, zEstimated_data, zEstimated_size, residualCovariance);
    }
  } else {
    costValue_size[0] = 1;
    costValue_size[1] = 1;
    isf_size[0] = zEstimated_size[0];
    costValue_data[0] = AEBSensorFusion_normalizedDistance(residual, isf_size,
      zEstimated_data, zEstimated_size, residualCovariance);
  }
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_GNNTracker_calcCostMatrixAllSensors
  (multiObjectTracker_AEBSensorFusion_T *obj, const
   BusDetectionConcatenation1Detections dets_data[], const int32_T dets_size[1],
   real_T overallCostMatrix_data[], int32_T overallCostMatrix_size[2])
{
  BusDetectionConcatenation1Detections expl_temp;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *obj_2[20];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *obj_3[20];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *varargin_1[20];
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj_0;
  c_trackingEKF_AEBSensorFusion_T *obj_1;
  cell_wrap_AEBSensorFusion_T varSizes[4];
  real_T detTimes_data[70];
  real_T detectionTimes_data[70];
  real_T origSen_data[70];
  real_T sensorIDs_data[70];
  real_T c_A[36];
  real_T stateCol[6];
  real_T Y;
  real_T dt;
  real_T lastTrackTime;
  real_T stateCol_tmp;
  int32_T SensorDetections_data[70];
  int32_T iidx_data[70];
  int32_T iidx_data_0[70];
  int32_T relevantinSeqDets_tmp_data[70];
  int32_T sorting_data[70];
  int32_T b_ipiv[6];
  int32_T b;
  int32_T b_0;
  int32_T d_k;
  int32_T end;
  int32_T expl_temp_tmp;
  int32_T expl_temp_tmp_0;
  int32_T i;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  int32_T nLiveTracks_tmp;
  int32_T nTracks;
  int32_T partialTrueCount;
  int32_T trueCount;
  int8_T inSize[8];
  boolean_T allToCalculate[1400];
  boolean_T toCalculate_data[1400];
  boolean_T insequence_data[70];
  boolean_T tmp_data[70];
  boolean_T isodd;
  static const int8_T inSize_0[8] = { 1, 20, 1, 1, 1, 1, 1, 1 };

  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  real_T tmp_data_0[6];
  int32_T detectionTimes_size[2];
  int32_T insequence_size[2];
  int32_T origSen_size[2];
  int32_T sensorIDs_size[2];
  int32_T sorting_size[2];
  int32_T detTimes_size[1];
  int32_T tmp_size[1];
  int32_T relevantinSeqDets_tmp_size_idx_1;
  int32_T toCalculate_size_idx_0;
  boolean_T exitg1;
  nTracks = obj->pNumLiveTracks;
  obj_0 = obj->cDetectionManager;
  lastTrackTime = obj_0->pNumDetections;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (lastTrackTime < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int32_T>(lastTrackTime);
  }

  insequence_size[0] = 1;
  insequence_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    insequence_data[i] = !obj_0->pIsOOSM[i];
  }

  overallCostMatrix_size[0] = nTracks;
  overallCostMatrix_size[1] = 70;
  loop_ub = nTracks * 70;
  if (loop_ub - 1 >= 0) {
    std::memset(&overallCostMatrix_data[0], 0, static_cast<uint32_T>(loop_ub) *
                sizeof(real_T));
  }

  obj_0 = obj->cDetectionManager;
  lastTrackTime = obj_0->pNumDetections;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (lastTrackTime < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int32_T>(lastTrackTime);
  }

  origSen_size[0] = 1;
  origSen_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    origSen_data[i] = obj_0->pOriginatingSensor[i];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_unique_vector(origSen_data, origSen_size, sensorIDs_data,
    sensorIDs_size);
  end = sensorIDs_size[1];
  trueCount = 0;
  for (i = 0; i < end; i++) {
    if (sensorIDs_data[i] > 0.0) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  for (i = 0; i < end; i++) {
    lastTrackTime = sensorIDs_data[i];
    if (lastTrackTime > 0.0) {
      sensorIDs_data[partialTrueCount] = lastTrackTime;
      partialTrueCount++;
    }
  }

  for (end = 0; end < trueCount; end++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_GNNTracker_selectDetections(origSen_data, origSen_size,
      sensorIDs_data[end], insequence_data, insequence_size,
      SensorDetections_data, sensorIDs_size);
    if (sensorIDs_size[1] != 0) {
      nLiveTracks_tmp = obj->pNumLiveTracks;
      for (i = 0; i < 20; i++) {
        obj_3[i] = obj->pTracksList[i];
      }

      for (i = 0; i < 20; i++) {
        obj_2[i] = obj_3[i];
      }

      for (i = 0; i < 20; i++) {
        varargin_1[i] = obj_2[i];
      }

      if (obj->cCostCalculator.isInitialized != 1) {
        obj->cCostCalculator.isInitialized = 1;
        for (i = 0; i < 8; i++) {
          varSizes[0].f1[i] = static_cast<uint32_T>(inSize_0[i]);
        }

        varSizes[1].f1[0] = static_cast<uint32_T>(dets_size[0]);
        varSizes[1].f1[1] = 1U;
        for (i = 0; i < 6; i++) {
          varSizes[1].f1[i + 2] = 1U;
        }

        for (i = 0; i < 8; i++) {
          varSizes[2].f1[i] = 1U;
        }

        varSizes[3].f1[0] = 1U;
        varSizes[3].f1[1] = static_cast<uint32_T>(sensorIDs_size[1]);
        for (i = 0; i < 6; i++) {
          varSizes[3].f1[i + 2] = 1U;
        }

        obj->cCostCalculator.inputVarSize[0] = varSizes[0];
        obj->cCostCalculator.inputVarSize[1] = varSizes[1];
        obj->cCostCalculator.inputVarSize[2] = varSizes[2];
        obj->cCostCalculator.inputVarSize[3] = varSizes[3];
        obj->cCostCalculator.pMaxAssignmentCost =
          obj->cCostCalculator.MaxAssignmentCost;
      }

      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 8)) {
        if (obj->cCostCalculator.inputVarSize[0].f1[loop_ub] != static_cast<
            uint32_T>(inSize_0[loop_ub])) {
          for (i = 0; i < 8; i++) {
            obj->cCostCalculator.inputVarSize[0].f1[i] = static_cast<uint32_T>
              (inSize_0[i]);
          }

          exitg1 = true;
        } else {
          loop_ub++;
        }
      }

      inSize[0] = static_cast<int8_T>(dets_size[0]);
      inSize[1] = 1;
      for (i = 0; i < 6; i++) {
        inSize[i + 2] = 1;
      }

      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 8)) {
        if (obj->cCostCalculator.inputVarSize[1].f1[loop_ub] !=
            static_cast<uint32_T>(inSize[loop_ub])) {
          for (i = 0; i < 8; i++) {
            obj->cCostCalculator.inputVarSize[1].f1[i] = static_cast<uint32_T>
              (inSize[i]);
          }

          exitg1 = true;
        } else {
          loop_ub++;
        }
      }

      d_k = 0;
      exitg1 = false;
      while ((!exitg1) && (d_k < 8)) {
        if (obj->cCostCalculator.inputVarSize[2].f1[d_k] != 1U) {
          for (i = 0; i < 8; i++) {
            obj->cCostCalculator.inputVarSize[2].f1[i] = 1U;
          }

          exitg1 = true;
        } else {
          d_k++;
        }
      }

      inSize[0] = 1;
      inSize[1] = static_cast<int8_T>(sensorIDs_size[1]);
      for (i = 0; i < 6; i++) {
        inSize[i + 2] = 1;
      }

      d_k = 0;
      exitg1 = false;
      while ((!exitg1) && (d_k < 8)) {
        if (obj->cCostCalculator.inputVarSize[3].f1[d_k] != static_cast<uint32_T>
            (inSize[d_k])) {
          for (i = 0; i < 8; i++) {
            obj->cCostCalculator.inputVarSize[3].f1[i] = static_cast<uint32_T>
              (inSize[i]);
          }

          exitg1 = true;
        } else {
          d_k++;
        }
      }

      obj->cCostCalculator.pMaxAssignmentCost =
        obj->cCostCalculator.MaxAssignmentCost;
      b_0 = sensorIDs_size[1];
      loop_ub_tmp = nLiveTracks_tmp * sensorIDs_size[1];
      for (i = 0; i < loop_ub_tmp; i++) {
        AEBSensorFusion_DW.costMatrix_data_m[i] = (rtInf);
      }

      lastTrackTime = obj->cCostCalculator.MaxAssignmentCost;
      if (rtIsInf(lastTrackTime)) {
        if (nLiveTracks_tmp < 1) {
          loop_ub = 0;
        } else {
          loop_ub = nLiveTracks_tmp;
        }

        toCalculate_size_idx_0 = loop_ub;
        for (i = 0; i < b_0; i++) {
          for (partialTrueCount = 0; partialTrueCount < loop_ub;
               partialTrueCount++) {
            toCalculate_data[partialTrueCount + loop_ub * i] = true;
          }
        }
      } else {
        for (i = 0; i < 1400; i++) {
          allToCalculate[i] = true;
        }

        detectionTimes_size[0] = 1;
        detectionTimes_size[1] = sensorIDs_size[1];
        for (loop_ub = 0; loop_ub < b_0; loop_ub++) {
          detectionTimes_data[loop_ub] = dets_data[SensorDetections_data[loop_ub]
            - 1].Time;
        }

        AEBSensorFusion_sort_mh(detectionTimes_data, detectionTimes_size,
          detTimes_data, sensorIDs_size, sorting_data, sorting_size);
        d_k = sorting_size[1];
        for (i = 0; i < d_k; i++) {
          detectionTimes_data[i] = sorting_data[i];
        }

        lastTrackTime = varargin_1[0]->UpdateTime;
        partialTrueCount = static_cast<uint8_T>(nLiveTracks_tmp);
        for (b = 0; b < b_0; b++) {
          expl_temp_tmp = SensorDetections_data[static_cast<int32_T>
            (detectionTimes_data[b]) - 1] - 1;
          expl_temp = dets_data[expl_temp_tmp];
          dt = dets_data[expl_temp_tmp].Time - lastTrackTime;
          for (d_k = 0; d_k < partialTrueCount; d_k++) {
            obj_1 = varargin_1[d_k]->Filter;
            for (i = 0; i < 6; i++) {
              stateCol[i] = obj_1->pState[i];
            }

            stateCol_tmp = dt * dt * 0.5 * 0.0;
            stateCol[0] = (stateCol[1] * dt + stateCol[0]) + stateCol_tmp;
            stateCol[1] += 0.0 * dt;
            stateCol[2] = (stateCol[3] * dt + stateCol[2]) + stateCol_tmp;
            stateCol[3] += 0.0 * dt;
            stateCol[4] = (stateCol[5] * dt + stateCol[4]) + stateCol_tmp;
            stateCol[5] += 0.0 * dt;
            AEBSensorFusion_cvmeas(stateCol, dets_data[expl_temp_tmp].
              MeasurementParameters.Frame, dets_data[expl_temp_tmp].
              MeasurementParameters.OriginPosition, dets_data[expl_temp_tmp].
              MeasurementParameters.Orientation, dets_data[expl_temp_tmp].
              MeasurementParameters.HasVelocity, dets_data[expl_temp_tmp].
              MeasurementParameters.OriginVelocity, dets_data[expl_temp_tmp].
              MeasurementParameters.IsParentToChild, dets_data[expl_temp_tmp].
              MeasurementParameters.HasAzimuth, dets_data[expl_temp_tmp].
              MeasurementParameters.HasElevation, dets_data[expl_temp_tmp].
              MeasurementParameters.HasRange, tmp_data_0, tmp_size);
            if (tmp_size[0] == 6) {
              AEBSensorFusion_cvmeas(stateCol,
                expl_temp.MeasurementParameters.Frame,
                expl_temp.MeasurementParameters.OriginPosition,
                expl_temp.MeasurementParameters.Orientation,
                expl_temp.MeasurementParameters.HasVelocity,
                expl_temp.MeasurementParameters.OriginVelocity,
                expl_temp.MeasurementParameters.IsParentToChild,
                expl_temp.MeasurementParameters.HasAzimuth,
                expl_temp.MeasurementParameters.HasElevation,
                expl_temp.MeasurementParameters.HasRange, tmp_data_0, tmp_size);
              loop_ub = tmp_size[0];
              for (i = 0; i < loop_ub; i++) {
                stateCol[i] = tmp_data_0[i] - expl_temp.Measurement[i];
              }
            } else {
              AEBSensorFusion_binary_expand_op(stateCol, &expl_temp);
            }

            for (i = 0; i < 6; i++) {
              tmp_data_0[i] = stateCol[i];
            }

            AEBSensorFusion_mrdiv_b(tmp_data_0, expl_temp.MeasurementNoise);
            std::memcpy(&c_A[0], &expl_temp.MeasurementNoise[0], 36U * sizeof
                        (real_T));
            AEBSensorFusion_xzgetrf(c_A, b_ipiv, &loop_ub);
            stateCol_tmp = c_A[0];
            isodd = false;
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              stateCol_tmp *= c_A[((loop_ub + 1) * 6 + loop_ub) + 1];
              if (b_ipiv[loop_ub] > loop_ub + 1) {
                isodd = !isodd;
              }
            }

            if (isodd) {
              stateCol_tmp = -stateCol_tmp;
            }

            Y = 0.0;
            for (i = 0; i < 6; i++) {
              Y += tmp_data_0[i] * stateCol[i];
            }

            allToCalculate[d_k + 20 * b] = (Y + std::log(stateCol_tmp) <
              obj->cCostCalculator.pMaxAssignmentCost);
          }
        }

        if (nLiveTracks_tmp < 1) {
          loop_ub = 0;
        } else {
          loop_ub = nLiveTracks_tmp;
        }

        toCalculate_size_idx_0 = loop_ub;
        for (i = 0; i < b_0; i++) {
          for (partialTrueCount = 0; partialTrueCount < loop_ub;
               partialTrueCount++) {
            toCalculate_data[partialTrueCount + loop_ub * i] = allToCalculate[20
              * i + partialTrueCount];
          }
        }
      }

      b = static_cast<uint8_T>(nLiveTracks_tmp);
      for (d_k = 0; d_k < b; d_k++) {
        partialTrueCount = 0;
        for (i = 0; i < b_0; i++) {
          tmp_data[i] = toCalculate_data[toCalculate_size_idx_0 * i + d_k];
          if (tmp_data[i]) {
            partialTrueCount++;
          }
        }

        relevantinSeqDets_tmp_size_idx_1 = partialTrueCount;
        partialTrueCount = 0;
        for (i = 0; i < b_0; i++) {
          if (tmp_data[i]) {
            relevantinSeqDets_tmp_data[partialTrueCount] = i;
            partialTrueCount++;
          }
        }

        if (relevantinSeqDets_tmp_size_idx_1 - 1 >= 0) {
          std::memcpy(&sorting_data[0], &relevantinSeqDets_tmp_data[0],
                      static_cast<uint32_T>(relevantinSeqDets_tmp_size_idx_1) *
                      sizeof(int32_T));
        }

        if ((dets_size[0] == 0) || (relevantinSeqDets_tmp_size_idx_1 == 0)) {
        } else {
          detTimes_size[0] = relevantinSeqDets_tmp_size_idx_1;
          for (loop_ub = 0; loop_ub < relevantinSeqDets_tmp_size_idx_1; loop_ub
               ++) {
            detTimes_data[loop_ub] =
              dets_data[SensorDetections_data[relevantinSeqDets_tmp_data[loop_ub]]
              - 1].Time;
            detectionTimes_data[loop_ub] = (rtInf);
          }

          AEBSensorFusion_sort_mhq(detTimes_data, detTimes_size, iidx_data,
            tmp_size);
          lastTrackTime = varargin_1[d_k]->Time;
          AEBSensorFusion_trackingEKF_sync(varargin_1[d_k]->pDistanceFilter,
            varargin_1[d_k]->Filter);
          for (loop_ub = 0; loop_ub < relevantinSeqDets_tmp_size_idx_1; loop_ub
               ++) {
            expl_temp_tmp =
              SensorDetections_data[relevantinSeqDets_tmp_data[iidx_data[loop_ub]
              - 1]] - 1;
            expl_temp = dets_data[expl_temp_tmp];
            obj_1 = varargin_1[d_k]->pDistanceFilter;
            for (i = 0; i < 6; i++) {
              for (partialTrueCount = 0; partialTrueCount < 6; partialTrueCount
                   ++) {
                expl_temp_tmp_0 = 6 * i + partialTrueCount;
                c_A[expl_temp_tmp_0] = (expl_temp.MeasurementNoise[6 *
                  partialTrueCount + i] +
                  expl_temp.MeasurementNoise[expl_temp_tmp_0]) / 2.0;
              }
            }

            AEBSensorFusion_eig(c_A, unusedExpr);
            obj_1->pSqrtMeasurementNoiseScalar = -1.0;
            if (obj_1->pSqrtMeasurementNoiseScalar > 0.0) {
              dt = obj_1->pSqrtMeasurementNoiseScalar;
              for (i = 0; i < 36; i++) {
                obj_1->pSqrtMeasurementNoise[i] = dt * static_cast<real_T>(tmp[i]);
              }

              obj_1->pSqrtMeasurementNoiseScalar = -1.0;
            }

            AEBSensorFusion_cholPSD(expl_temp.MeasurementNoise,
              obj_1->pSqrtMeasurementNoise);
            if ((varargin_1[d_k]->ObjectClassID == 0.0) ||
                (dets_data[expl_temp_tmp].ObjectClassID == 0.0) ||
                (dets_data[expl_temp_tmp].ObjectClassID == varargin_1[d_k]
                 ->ObjectClassID)) {
              dt = expl_temp.Time - lastTrackTime;
              if (dt > 0.0) {
                AEBSensorFusion_predictTrackFilter(varargin_1[d_k]
                  ->pDistanceFilter, dt);
              }

              lastTrackTime = expl_temp.Time;
              AEBSensorFusion_ObjectTrack_calcCostOneDetection(varargin_1[d_k],
                dets_data[expl_temp_tmp].Measurement, dets_data[expl_temp_tmp].
                MeasurementParameters.Frame, dets_data[expl_temp_tmp].
                MeasurementParameters.OriginPosition, dets_data[expl_temp_tmp].
                MeasurementParameters.Orientation, dets_data[expl_temp_tmp].
                MeasurementParameters.HasVelocity, dets_data[expl_temp_tmp].
                MeasurementParameters.OriginVelocity, dets_data[expl_temp_tmp].
                MeasurementParameters.IsParentToChild, dets_data[expl_temp_tmp].
                MeasurementParameters.HasAzimuth, dets_data[expl_temp_tmp].
                MeasurementParameters.HasElevation, dets_data[expl_temp_tmp].
                MeasurementParameters.HasRange, stateCol, sensorIDs_size);
              detectionTimes_data[loop_ub] = stateCol[0];
            }
          }

          for (i = 0; i < relevantinSeqDets_tmp_size_idx_1; i++) {
            iidx_data_0[i] = iidx_data[i] - 1;
          }

          std::memcpy(&iidx_data[0], &iidx_data_0[0], static_cast<uint32_T>
                      (relevantinSeqDets_tmp_size_idx_1) * sizeof(int32_T));
          std::memcpy(&detTimes_data[0], &detectionTimes_data[0], static_cast<
                      uint32_T>(relevantinSeqDets_tmp_size_idx_1) * sizeof
                      (real_T));
          for (i = 0; i < relevantinSeqDets_tmp_size_idx_1; i++) {
            detectionTimes_data[iidx_data[i]] = detTimes_data[i];
          }
        }

        for (i = 0; i < relevantinSeqDets_tmp_size_idx_1; i++) {
          AEBSensorFusion_DW.costMatrix_data_m[d_k + nLiveTracks_tmp *
            sorting_data[i]] = detectionTimes_data[i];
        }
      }

      lastTrackTime = obj->AssignmentThreshold[0];
      for (i = 0; i < loop_ub_tmp; i++) {
        toCalculate_data[i] = (AEBSensorFusion_DW.costMatrix_data_m[i] >
          lastTrackTime);
        if (toCalculate_data[i]) {
          AEBSensorFusion_DW.costMatrix_data_m[i] = (rtInf);
        }
      }

      if (nTracks < 1) {
        d_k = 0;
      } else {
        d_k = nTracks;
      }

      for (i = 0; i < b_0; i++) {
        iidx_data[i] = SensorDetections_data[i] - 1;
      }

      if (nTracks < 1) {
        loop_ub = 0;
      } else {
        loop_ub = nTracks;
      }

      for (i = 0; i < b_0; i++) {
        for (partialTrueCount = 0; partialTrueCount < d_k; partialTrueCount++) {
          overallCostMatrix_data[partialTrueCount + nTracks * iidx_data[i]] =
            AEBSensorFusion_DW.costMatrix_data_m[loop_ub * i + partialTrueCount];
        }
      }
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_14
  (boolean_T in1_data[], int32_T in1_size[1], real_T in2, real_T in3)
{
  int32_T aux_1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_1_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (in2 < 1.0) {
    i = 0;
  } else {
    i = static_cast<int32_T>(in2);
  }

  if (in3 < 1.0) {
    stride_1_0 = 0;
  } else {
    stride_1_0 = static_cast<int32_T>(in3);
  }

  loop_ub = stride_1_0 == 1 ? i : stride_1_0;
  in1_size[0] = loop_ub;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_1_0 = (stride_1_0 != 1);
  aux_1_0 = 0;
  for (i = 0; i < loop_ub; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    in1_data[i] = (static_cast<uint32_T>(aux_1_0) + 1U > 0U);
    aux_1_0 += stride_1_0;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_minPriorityQueue_percUp
  (matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T *obj, int32_T i,
   const real_T dist_data[])
{
  int32_T iparent;
  boolean_T exitg1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  iparent = i / 2 - 1;
  exitg1 = false;
  while ((!exitg1) && (iparent + 1 > 0)) {
    real_T tmp;
    real_T tmp_0;
    int32_T obj_idx_1;
    obj_idx_1 = obj->heap.data[i - 1];
    tmp = dist_data[obj_idx_1 - 1];
    tmp_0 = dist_data[obj->heap.data[iparent] - 1];
    if ((tmp < tmp_0) || ((tmp == tmp_0) && (obj_idx_1 <= obj->heap.data[iparent])))
    {
      int32_T obj_idx_1_tmp;
      obj->heap.data[i - 1] = obj->heap.data[iparent];
      obj->heap.data[iparent] = obj_idx_1;
      obj_idx_1_tmp = obj->heap.data[i - 1] - 1;
      obj_idx_1 = obj->indexToHeap.data[obj_idx_1_tmp];
      obj->indexToHeap.data[obj_idx_1_tmp] = obj->indexToHeap.data
        [obj->heap.data[iparent] - 1];
      obj->indexToHeap.data[obj->heap.data[iparent] - 1] = obj_idx_1;
      i = iparent + 1;
      iparent = (iparent + 1) / 2 - 1;
    } else {
      exitg1 = true;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_perfectMatching(const real_T
  A_data[], const int32_T A_size[2], int32_T m1_data[], int32_T m1_size[1],
  int32_T m2_data[], int32_T m2_size[1])
{
  matlab_internal_coder_minPriorityQueue_AEBSensorFusion_T queue;
  real_T a__2_data[90];
  real_T b_ex_data[90];
  real_T distancesR_data[90];
  real_T minIndices_data[90];
  real_T pairWeightR_data[90];
  real_T dnew;
  real_T edge_weight;
  real_T edge_weight_shifted;
  real_T last_weight_sap;
  real_T lsap;
  int32_T queue_heap_data[90];
  int32_T b_i;
  int32_T c;
  int32_T clast;
  int32_T edge_weight_shifted_tmp;
  int32_T exitg2;
  int32_T exitg3;
  int32_T exitg4;
  int32_T lc;
  int32_T n;
  int32_T n_tmp;
  int32_T rlast;
  int32_T rnext;
  int32_T y;
  int8_T b_idx_data[90];
  uint8_T colorsR_data[90];
  uint8_T tmp;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T p;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  n_tmp = A_size[0];
  m1_size[0] = A_size[0];
  m2_size[0] = A_size[0];
  for (n = 0; n < n_tmp; n++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    m1_data[n] = 0;
    m2_data[n] = 0;
    a__2_data[n] = (rtInf);
  }

  if (A_size[0] == 0) {
  } else {
    n = A_size[1];
    for (b_i = 0; b_i < n_tmp; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b_idx_data[b_i] = 1;
      b_ex_data[b_i] = A_data[b_i];
    }

    for (rlast = 2; rlast <= n; rlast++) {
      for (clast = 0; clast < n_tmp; clast++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        lsap = A_data[(rlast - 1) * A_size[0] + clast];
        if (rtIsNaN(lsap)) {
          p = false;
        } else {
          last_weight_sap = b_ex_data[clast];
          p = (rtIsNaN(last_weight_sap) || (last_weight_sap > lsap));
        }

        if (p) {
          // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
          b_ex_data[clast] = lsap;
          b_idx_data[clast] = static_cast<int8_T>(rlast);
        }
      }
    }

    p = true;
    for (b_i = 0; b_i < n_tmp; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      minIndices_data[b_i] = b_idx_data[b_i];
      if (p) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        edge_weight_shifted = b_ex_data[b_i];
        if ((!rtIsInf(edge_weight_shifted)) && (!rtIsNaN(edge_weight_shifted)))
        {
        } else {
          p = false;
        }
      } else {
        p = false;
      }
    }

    guard1 = false;
    if (!p) {
      guard1 = true;
    } else {
      for (rnext = 0; rnext < n_tmp; rnext++) {
        pairWeightR_data[rnext] = 0.0;

        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        b_i = static_cast<int32_T>(minIndices_data[rnext]);
        if (m1_data[b_i - 1] == 0) {
          m2_data[rnext] = b_i;
          m1_data[b_i - 1] = rnext + 1;
          pairWeightR_data[rnext] = b_ex_data[rnext];
        }

        for (clast = 0; clast < n_tmp; clast++) {
          // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
          edge_weight_shifted = A_data[A_size[0] * rnext + clast] -
            b_ex_data[clast];
          if (edge_weight_shifted < a__2_data[rnext]) {
            a__2_data[rnext] = edge_weight_shifted;
          }
        }
      }

      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i <= n_tmp - 1)) {
        if (m1_data[b_i] != 0) {
          b_i++;
        } else {
          queue.heap.size[0] = n_tmp;
          queue.indexToHeap.size[0] = n_tmp;

          // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
          for (n = 0; n < n_tmp; n++) {
            minIndices_data[n] = 0.0;
            b_idx_data[n] = 0;
            distancesR_data[n] = (rtInf);
            colorsR_data[n] = 0U;
            queue.heap.data[n] = queue_heap_data[n];
            queue.indexToHeap.data[n] = queue_heap_data[n];
          }

          b_idx_data[b_i] = 0;
          queue.len = 0;
          edge_weight_shifted = 0.0;
          lsap = (rtInf);
          rlast = 0;

          // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
          clast = -1;
          last_weight_sap = (rtInf);

          // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
          c = b_i;
          guard2 = false;
          do {
            exitg3 = 0;
            rnext = 1;
            do {
              exitg2 = 0;
              if (rnext - 1 <= n_tmp - 1) {
                tmp = colorsR_data[rnext - 1];
                if (tmp == 2) {
                  rnext++;
                } else {
                  edge_weight = A_data[(A_size[0] * c + rnext) - 1];
                  dnew = ((edge_weight - b_ex_data[rnext - 1]) - a__2_data[c]) +
                    edge_weight_shifted;
                  if (dnew < lsap) {
                    n = m2_data[rnext - 1];
                    if (n == 0) {
                      lsap = dnew;
                      rlast = rnext;
                      clast = c;
                      last_weight_sap = edge_weight;
                    } else if (dnew < distancesR_data[rnext - 1]) {
                      distancesR_data[rnext - 1] = dnew;
                      b_idx_data[n - 1] = static_cast<int8_T>(c + 1);
                      minIndices_data[rnext - 1] = edge_weight;
                      if (tmp == 0) {
                        queue.len++;
                        queue.heap.data[queue.len - 1] = rnext;
                        queue.indexToHeap.data[rnext - 1] = queue.len;
                        AEBSensorFusion_minPriorityQueue_percUp(&queue,
                          queue.len, distancesR_data);
                        colorsR_data[rnext - 1] = 1U;
                      } else {
                        AEBSensorFusion_minPriorityQueue_percUp(&queue,
                          queue.indexToHeap.data[rnext - 1], distancesR_data);
                      }
                    }

                    rnext++;
                  } else if (((!(dnew >= -1.7976931348623157E+308)) || (!(dnew <=
                    1.7976931348623157E+308))) && (edge_weight >=
                              -1.7976931348623157E+308) && (edge_weight <=
                              1.7976931348623157E+308)) {
                    p = false;
                    exitg2 = 2;
                  } else {
                    rnext++;
                  }
                }
              } else {
                exitg2 = 1;
              }
            } while (exitg2 == 0);

            if (exitg2 == 1) {
              if (queue.len == 0) {
                guard2 = true;
                exitg3 = 1;
              } else {
                c = queue.heap.data[0] - 1;
                n = queue.len - 1;
                queue.heap.data[0] = queue.heap.data[queue.len - 1];
                queue.indexToHeap.data[queue.heap.data[0] - 1] = 1;
                queue.len--;
                rnext = 0;
                do {
                  exitg4 = 0;
                  y = (rnext + 1) << 1;
                  if (y <= n) {
                    lc = y - 1;
                    if (y + 1 > n) {
                    } else {
                      edge_weight_shifted_tmp = queue.heap.data[y - 1];
                      edge_weight_shifted =
                        distancesR_data[edge_weight_shifted_tmp - 1];
                      edge_weight = distancesR_data[queue.heap.data[y] - 1];
                      if ((edge_weight_shifted < edge_weight) ||
                          ((edge_weight_shifted == edge_weight) &&
                           (edge_weight_shifted_tmp <= queue.heap.data[y]))) {
                      } else {
                        lc = y;
                      }
                    }

                    edge_weight_shifted = distancesR_data[queue.heap.data[rnext]
                      - 1];
                    edge_weight = distancesR_data[queue.heap.data[lc] - 1];
                    if ((edge_weight_shifted < edge_weight) ||
                        ((edge_weight_shifted == edge_weight) &&
                         (queue.heap.data[rnext] <= queue.heap.data[lc]))) {
                      exitg4 = 1;
                    } else {
                      y = queue.heap.data[rnext];
                      queue.heap.data[rnext] = queue.heap.data[lc];
                      queue.heap.data[lc] = y;
                      y = queue.indexToHeap.data[queue.heap.data[rnext] - 1];
                      queue.indexToHeap.data[queue.heap.data[rnext] - 1] =
                        queue.indexToHeap.data[queue.heap.data[lc] - 1];
                      queue.indexToHeap.data[queue.heap.data[lc] - 1] = y;
                      rnext = lc;
                    }
                  } else {
                    exitg4 = 1;
                  }
                } while (exitg4 == 0);

                edge_weight_shifted = distancesR_data[c];
                if (lsap <= distancesR_data[c]) {
                  guard2 = true;
                  exitg3 = 1;
                } else {
                  colorsR_data[c] = 2U;
                  c = m2_data[c] - 1;
                  guard2 = false;
                }
              }
            } else {
              exitg3 = 1;
            }
          } while (exitg3 == 0);

          if (guard2) {
            p = (lsap < (rtInf));
            if (p) {
              do {
                exitg2 = 0;
                rnext = m1_data[clast];
                m2_data[rlast - 1] = clast + 1;
                m1_data[clast] = rlast;
                pairWeightR_data[rlast - 1] = last_weight_sap;
                if (b_idx_data[clast] == 0) {
                  exitg2 = 1;
                } else {
                  rlast = rnext;
                  clast = b_idx_data[clast] - 1;
                  last_weight_sap = minIndices_data[rnext - 1];
                }
              } while (exitg2 == 0);

              for (clast = 0; clast < n_tmp; clast++) {
                if (colorsR_data[clast] == 2) {
                  b_ex_data[clast] = (b_ex_data[clast] - lsap) +
                    distancesR_data[clast];
                }
              }

              for (n = 0; n < n_tmp; n++) {
                rlast = m1_data[n];
                if (rlast != 0) {
                  a__2_data[n] = pairWeightR_data[rlast - 1] - b_ex_data[rlast -
                    1];
                }
              }
            }
          }

          if (!p) {
            guard1 = true;
            exitg1 = true;
          } else {
            b_i++;
          }
        }
      }
    }

    if (guard1) {
      m1_size[0] = 0;
      m2_size[0] = 0;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_SystemCore_step
  (c_matlabshared_tracking_internal_fusion_AssignerGNN_AEBSensor_T *obj, const
   real_T varargin_1_data[], const int32_T varargin_1_size[2], uint32_T
   varargout_1_data[], int32_T varargout_1_size[2])
{
  real_T absNewVal;
  real_T costUnmatched;
  real_T maxVal;
  int32_T b_matchings_data[140];
  int32_T b_rowToCol_data[90];
  int32_T colToRow_data[90];
  int32_T paddedCost_size[2];
  int32_T paddedCost_size_0[2];
  int32_T b_rowToCol_size[1];
  int32_T colToRow_size[1];
  int32_T b_ii;
  int32_T b_jj;
  int32_T b_tmp;
  int32_T d;
  int32_T loop_ub_tmp;
  int32_T m_tmp;
  int32_T n;
  int32_T nr;
  int32_T varargin_1_tmp;
  if (obj->isInitialized != 1) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->isInitialized = 1;
    obj->isSetupComplete = true;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->pCostOfNonAssignment = obj->AssignmentThreshold[0] / 2.0;
  costUnmatched = obj->pCostOfNonAssignment;
  m_tmp = varargin_1_size[0];
  n = varargin_1_size[1] - 1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  varargin_1_tmp = varargin_1_size[0] + varargin_1_size[1];
  paddedCost_size[0] = varargin_1_tmp;
  paddedCost_size[1] = varargin_1_tmp;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  loop_ub_tmp = varargin_1_tmp * varargin_1_tmp;
  for (b_jj = 0; b_jj < loop_ub_tmp; b_jj++) {
    AEBSensorFusion_DW.paddedCost_data[b_jj] = (rtInf);
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  b_tmp = varargin_1_size[1];
  for (b_jj = 0; b_jj < b_tmp; b_jj++) {
    for (b_ii = 0; b_ii < m_tmp; b_ii++) {
      AEBSensorFusion_DW.paddedCost_data[b_ii + varargin_1_tmp * b_jj] =
        varargin_1_data[varargin_1_size[0] * b_jj + b_ii];
    }
  }

  for (b_ii = 0; b_ii < m_tmp; b_ii++) {
    d = n + 1;
    for (b_jj = 0; b_jj < d; b_jj++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      AEBSensorFusion_DW.paddedCost_data[(m_tmp + b_jj) + varargin_1_tmp * ((n +
        b_ii) + 1)] = varargin_1_data[varargin_1_size[0] * b_jj + b_ii];
    }
  }

  for (b_jj = 0; b_jj < m_tmp; b_jj++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_DW.paddedCost_data[b_jj + varargin_1_tmp * ((n + b_jj) + 1)]
      = 2.0 * costUnmatched;
  }

  for (b_jj = 0; b_jj < b_tmp; b_jj++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_DW.paddedCost_data[(m_tmp + b_jj) + varargin_1_tmp * b_jj] =
      2.0 * costUnmatched;
  }

  AEBSensorFusion_perfectMatching(AEBSensorFusion_DW.paddedCost_data,
    paddedCost_size, colToRow_data, colToRow_size, b_rowToCol_data,
    b_rowToCol_size);
  if ((colToRow_size[0] == 0) && (varargin_1_tmp > 0)) {
    maxVal = 0.0;
    for (b_jj = 0; b_jj < loop_ub_tmp; b_jj++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      absNewVal = std::abs(AEBSensorFusion_DW.paddedCost_data[b_jj]);
      if ((absNewVal > maxVal) && (!rtIsInf(absNewVal))) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        maxVal = absNewVal;
      }
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (!rtIsInf(maxVal)) {
      absNewVal = std::frexp(maxVal, &nr);
      maxVal = nr;
      if (absNewVal == 0.5) {
        maxVal = static_cast<real_T>(nr) - 1.0;
      }
    }

    maxVal = rt_powd_snf(2.0, -maxVal);
    paddedCost_size_0[0] = varargin_1_tmp;
    paddedCost_size_0[1] = varargin_1_tmp;
    for (b_jj = 0; b_jj < loop_ub_tmp; b_jj++) {
      AEBSensorFusion_DW.paddedCost_data_k[b_jj] =
        AEBSensorFusion_DW.paddedCost_data[b_jj] * maxVal;
    }

    AEBSensorFusion_perfectMatching(AEBSensorFusion_DW.paddedCost_data_k,
      paddedCost_size_0, colToRow_data, colToRow_size, b_rowToCol_data,
      b_rowToCol_size);
  }

  for (nr = 0; nr < b_tmp; nr++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    loop_ub_tmp = colToRow_data[nr];
    if ((loop_ub_tmp <= m_tmp) && (varargin_1_data[(varargin_1_size[0] * nr +
          loop_ub_tmp) - 1] == 2.0 * costUnmatched)) {
      colToRow_data[nr] = (m_tmp + n) + 2;
    }
  }

  nr = 0;
  for (n = 0; n < b_tmp; n++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (colToRow_data[n] <= m_tmp) {
      nr++;
    }
  }

  b_jj = nr;
  nr = -1;
  for (n = 0; n < b_tmp; n++) {
    loop_ub_tmp = colToRow_data[n];
    if (loop_ub_tmp <= m_tmp) {
      nr++;
      b_matchings_data[nr] = loop_ub_tmp;
      b_matchings_data[nr + b_jj] = n + 1;
    }
  }

  varargout_1_size[0] = b_jj;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  varargout_1_size[1] = 2;
  loop_ub_tmp = b_jj << 1;
  for (b_jj = 0; b_jj < loop_ub_tmp; b_jj++) {
    n = b_matchings_data[b_jj];
    if (n < 0) {
      n = 0;
    }

    varargout_1_data[b_jj] = static_cast<uint32_T>(n);
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_binary_expand_op_15
  (boolean_T in1_data[], int32_T in1_size[1], const boolean_T in2[70], real_T
   in3, real_T in4, const boolean_T in5_data[], const int32_T in5_size[2])
{
  int32_T aux_1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  int32_T stride_2_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (in3 < 1.0) {
    i = 0;
  } else {
    i = static_cast<int32_T>(in3);
  }

  if (in4 < 1.0) {
    stride_1_0 = 0;
  } else {
    stride_1_0 = static_cast<int32_T>(in4);
  }

  loop_ub = in5_size[1] == 1 ? stride_1_0 == 1 ? i : stride_1_0 : in5_size[1];
  in1_size[0] = loop_ub;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  stride_0_0 = (i != 1);
  stride_1_0 = (stride_1_0 != 1);
  stride_2_0 = (in5_size[1] != 1);
  aux_1_0 = 0;
  for (i = 0; i < loop_ub; i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    in1_data[i] = ((!in2[i * stride_0_0]) && (static_cast<uint32_T>(aux_1_0) +
      1U > 0U) && in5_data[i * stride_2_0]);
    aux_1_0 += stride_1_0;
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_GNNTracker_associate
  (multiObjectTracker_AEBSensorFusion_T *obj, const real_T costMatrix_data[],
   const int32_T costMatrix_size[2], uint32_T assigned_data[], int32_T
   assigned_size[2], uint32_T unassignedTrs_data[], int32_T unassignedTrs_size[1],
   uint32_T unassignedDets_data[], int32_T unassignedDets_size[1])
{
  coder::bounded_array<int32_T, 20U, 2U> obj_1;
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj_0;
  real_T origSen_data[70];
  real_T sensorIDs_data[70];
  real_T b;
  real_T db;
  int32_T SensorDetections_data[70];
  int32_T m_data[70];
  int32_T costMatrix_size_0[2];
  int32_T insequence_size[2];
  int32_T origSen_size[2];
  int32_T sensorIDs_size[2];
  int32_T tmp_size[1];
  int32_T costMatrix;
  int32_T i;
  int32_T lastAssigned;
  int32_T loop_ub;
  int32_T nTracks;
  int32_T p_idx_0;
  int32_T partialTrueCount;
  int32_T trueCount;
  uint32_T assignments_data[140];
  uint32_T b_OverallAssignments[140];
  int8_T tmp_data_0[70];
  int8_T tmp_data_1[70];
  boolean_T insequence_data[70];
  boolean_T isDetectionAssociated[70];
  boolean_T tmp[70];
  boolean_T tmp_data[70];
  boolean_T isTrackAssociated[20];
  boolean_T tmp_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  nTracks = obj->pNumLiveTracks;
  lastAssigned = 1;
  obj_0 = obj->cDetectionManager;
  b = obj_0->pNumDetections;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (b < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int32_T>(b);
  }

  insequence_size[0] = 1;
  insequence_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    insequence_data[i] = !obj_0->pIsOOSM[i];
  }

  std::memset(&b_OverallAssignments[0], 0, 140U * sizeof(uint32_T));
  for (i = 0; i < 20; i++) {
    isTrackAssociated[i] = true;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (nTracks < 1) {
    nTracks = 0;
  }

  if (nTracks - 1 >= 0) {
    std::memset(&isTrackAssociated[0], 0, static_cast<uint32_T>(nTracks) *
                sizeof(boolean_T));
  }

  std::memset(&isDetectionAssociated[0], 0, 70U * sizeof(boolean_T));

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (obj->pNumDetections == 0.0) {
    assigned_size[0] = 0;
    assigned_size[1] = 2;
    trueCount = 0;
    for (i = 0; i < 20; i++) {
      if (!isTrackAssociated[i]) {
        trueCount++;
      }
    }

    unassignedTrs_size[0] = trueCount;
    partialTrueCount = 0;
    for (i = 0; i < 20; i++) {
      if (!isTrackAssociated[i]) {
        unassignedTrs_data[partialTrueCount] = static_cast<uint32_T>(i) + 1U;
        partialTrueCount++;
      }
    }

    b = obj->pNumDetections;
    db = obj->pNumDetections;
    if (b < 1.0) {
      i = 0;
    } else {
      i = static_cast<int32_T>(b);
    }

    if (db < 1.0) {
      partialTrueCount = 0;
    } else {
      partialTrueCount = static_cast<int32_T>(db);
    }

    if (i == partialTrueCount) {
      if (b < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int32_T>(b);
      }

      tmp_size[0] = loop_ub;
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = true;
      }
    } else {
      AEBSensorFusion_binary_expand_op_14(tmp_data, tmp_size, b, db);
    }

    nTracks = tmp_size[0];
    trueCount = 0;
    for (i = 0; i < nTracks; i++) {
      if (tmp_data[i]) {
        trueCount++;
      }
    }

    partialTrueCount = 0;
    for (i = 0; i < nTracks; i++) {
      if (tmp_data[i]) {
        tmp_data_0[partialTrueCount] = static_cast<int8_T>(i);
        partialTrueCount++;
      }
    }

    unassignedDets_size[0] = trueCount;
    for (i = 0; i < trueCount; i++) {
      unassignedDets_data[i] = static_cast<uint32_T>(tmp_data_0[i]) + 1U;
    }
  } else {
    obj_0 = obj->cDetectionManager;
    b = obj_0->pNumDetections;
    if (b < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int32_T>(b);
    }

    origSen_size[0] = 1;
    origSen_size[1] = loop_ub;
    for (i = 0; i < loop_ub; i++) {
      origSen_data[i] = obj_0->pOriginatingSensor[i];
    }

    AEBSensorFusion_unique_vector(origSen_data, origSen_size, sensorIDs_data,
      sensorIDs_size);
    nTracks = sensorIDs_size[1];
    trueCount = 0;
    for (i = 0; i < nTracks; i++) {
      if (sensorIDs_data[i] > 0.0) {
        trueCount++;
      }
    }

    partialTrueCount = 0;
    for (i = 0; i < nTracks; i++) {
      b = sensorIDs_data[i];
      if (b > 0.0) {
        sensorIDs_data[partialTrueCount] = b;
        partialTrueCount++;
      }
    }

    AEBSensorFusion_TrackManager_getLiveTrackIndices(obj, obj_1.data, obj_1.size);
    for (nTracks = 0; nTracks < trueCount; nTracks++) {
      AEBSensorFusion_GNNTracker_selectDetections(origSen_data, origSen_size,
        sensorIDs_data[nTracks], insequence_data, insequence_size,
        SensorDetections_data, sensorIDs_size);
      if (sensorIDs_size[1] != 0) {
        costMatrix = costMatrix_size[0];
        costMatrix_size_0[0] = costMatrix_size[0];
        loop_ub = sensorIDs_size[1];
        costMatrix_size_0[1] = sensorIDs_size[1];
        for (i = 0; i < loop_ub; i++) {
          for (partialTrueCount = 0; partialTrueCount < costMatrix;
               partialTrueCount++) {
            AEBSensorFusion_DW.costMatrix_data_c[partialTrueCount +
              costMatrix_size_0[0] * i] = costMatrix_data
              [(SensorDetections_data[i] - 1) * costMatrix_size[0] +
              partialTrueCount];
          }
        }

        AEBSensorFusion_SystemCore_step(obj->cAssigner,
          AEBSensorFusion_DW.costMatrix_data_c, costMatrix_size_0,
          assignments_data, sensorIDs_size);
        if (sensorIDs_size[0] != 0) {
          loop_ub = lastAssigned + sensorIDs_size[0];
          i = loop_ub - 1;
          if (lastAssigned > loop_ub - 1) {
            costMatrix = 0;
            i = 0;
          } else {
            costMatrix = lastAssigned - 1;
          }

          p_idx_0 = i - costMatrix;
          for (i = 0; i < p_idx_0; i++) {
            b_OverallAssignments[costMatrix + i] = assignments_data[i];
          }

          i = loop_ub - 1;
          if (lastAssigned > loop_ub - 1) {
            costMatrix = 0;
            i = 0;
          } else {
            costMatrix = lastAssigned - 1;
          }

          p_idx_0 = i - costMatrix;
          for (i = 0; i < p_idx_0; i++) {
            partialTrueCount = SensorDetections_data[static_cast<int32_T>
              (assignments_data[i + sensorIDs_size[0]]) - 1];
            if (partialTrueCount < 0) {
              partialTrueCount = 0;
            }

            b_OverallAssignments[(costMatrix + i) + 70] = static_cast<uint32_T>
              (partialTrueCount);
          }

          lastAssigned = loop_ub;
          loop_ub = sensorIDs_size[0];
          for (i = 0; i < loop_ub; i++) {
            m_data[i] = static_cast<int32_T>(assignments_data[i]);
            isTrackAssociated[m_data[i] - 1] = true;
          }
        }

        partialTrueCount = sensorIDs_size[0];
        for (i = 0; i < partialTrueCount; i++) {
          isDetectionAssociated[SensorDetections_data[static_cast<int32_T>
            (assignments_data[i + sensorIDs_size[0]]) - 1] - 1] = true;
        }
      }
    }

    trueCount = 0;
    for (i = 0; i < 70; i++) {
      tmp_0 = (b_OverallAssignments[i] > 0U);
      tmp[i] = tmp_0;
      if (tmp_0) {
        trueCount++;
      }
    }

    partialTrueCount = 0;
    for (i = 0; i < 70; i++) {
      if (tmp[i]) {
        tmp_data_0[partialTrueCount] = static_cast<int8_T>(i);
        partialTrueCount++;
      }
    }

    assigned_size[0] = trueCount;
    assigned_size[1] = 2;
    for (i = 0; i < 2; i++) {
      for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount
           ++) {
        assigned_data[partialTrueCount + trueCount * i] = b_OverallAssignments
          [70 * i + tmp_data_0[partialTrueCount]];
      }
    }

    trueCount = 0;
    for (i = 0; i < 20; i++) {
      if (!isTrackAssociated[i]) {
        trueCount++;
      }
    }

    unassignedTrs_size[0] = trueCount;
    partialTrueCount = 0;
    for (i = 0; i < 20; i++) {
      if (!isTrackAssociated[i]) {
        unassignedTrs_data[partialTrueCount] = static_cast<uint32_T>(i) + 1U;
        partialTrueCount++;
      }
    }

    b = obj->pNumDetections;
    db = obj->pNumDetections;
    if (db < 1.0) {
      i = 0;
      partialTrueCount = 0;
    } else {
      i = static_cast<int32_T>(db);
      partialTrueCount = static_cast<int32_T>(db);
    }

    if (b < 1.0) {
      nTracks = 0;
      lastAssigned = 0;
    } else {
      nTracks = static_cast<int32_T>(b);
      lastAssigned = static_cast<int32_T>(b);
    }

    if ((lastAssigned == i) && ((nTracks == 1 ? partialTrueCount : nTracks) ==
         insequence_size[1])) {
      if (b < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int32_T>(b);
      }

      tmp_size[0] = loop_ub;
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = ((!isDetectionAssociated[i]) && insequence_data[i]);
      }
    } else {
      AEBSensorFusion_binary_expand_op_15(tmp_data, tmp_size,
        isDetectionAssociated, b, db, insequence_data, insequence_size);
    }

    nTracks = tmp_size[0];
    trueCount = 0;
    for (i = 0; i < nTracks; i++) {
      if (tmp_data[i]) {
        trueCount++;
      }
    }

    partialTrueCount = 0;
    for (i = 0; i < nTracks; i++) {
      if (tmp_data[i]) {
        tmp_data_1[partialTrueCount] = static_cast<int8_T>(i);
        partialTrueCount++;
      }
    }

    unassignedDets_size[0] = trueCount;
    for (i = 0; i < trueCount; i++) {
      unassignedDets_data[i] = static_cast<uint32_T>(tmp_data_1[i]) + 1U;
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_eml_find(const boolean_T x[2],
  int32_T i_data[], int32_T i_size[2])
{
  int32_T b_ii;
  int32_T idx;
  boolean_T exitg1;
  idx = 0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  i_size[0] = 1;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 2)) {
    if (x[b_ii - 1]) {
      idx++;
      i_data[idx - 1] = b_ii;
      if (idx >= 2) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (idx < 1) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

boolean_T ACCWithSensorFusionModelClass::
  AEBSensorFusion_TrackManager_initiateTrack
  (multiObjectTracker_AEBSensorFusion_T *obj, uint32_T newTrackID, real_T
   Det_Time, const real_T Det_Measurement[6], const real_T Det_MeasurementNoise
   [36], real_T Det_SensorIndex, real_T Det_ObjectClassID,
   drivingCoordinateFrameType Det_MeasurementParameters_Frame, const real_T
   Det_MeasurementParameters_OriginPosition[3], const real_T
   Det_MeasurementParameters_Orientation[9], boolean_T
   Det_MeasurementParameters_HasVelocity, const real_T
   Det_MeasurementParameters_OriginVelocity[3], boolean_T
   Det_MeasurementParameters_IsParentToChild, boolean_T
   Det_MeasurementParameters_HasElevation, const
   BusRadarDetectionsObjectAttributes Det_ObjectAttributes)
{
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track_0;
  c_trackHistoryLogic_AEBSensorFusion_T *obj_0;
  c_trackingEKF_AEBSensorFusion_T lobj_0[2];
  c_trackingEKF_AEBSensorFusion_T *filter;
  real_T Det_MeasurementNoise_0[36];
  real_T b_value;
  int32_T ind_data[2];
  int32_T Det_MeasurementNoise_tmp;
  int32_T i;
  int32_T loop_ub;
  int32_T newTrackIndex;
  uint32_T tmp_0;
  boolean_T tmp[50];
  boolean_T in[2];
  boolean_T tf;
  boolean_T x_idx_0;
  boolean_T x_idx_1;
  boolean_T x_idx_2;
  static const int8_T tmp_1[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  int32_T ind_size[2];
  newTrackIndex = obj->pNumLiveTracks;
  if (newTrackIndex + 1 <= 20) {
    obj->pTrackIDs[newTrackIndex] = newTrackID;
    obj->pNumLiveTracks++;
    track = obj->pTracksList[newTrackIndex];
    track->TrackID = newTrackID;
    b_value = rt_roundd_snf(Det_SensorIndex);
    if (b_value < 4.294967296E+9) {
      if (b_value >= 0.0) {
        tmp_0 = static_cast<uint32_T>(b_value);
      } else {
        tmp_0 = 0U;
      }
    } else {
      tmp_0 = MAX_uint32_T;
    }

    in[0] = (tmp_0 == obj->pUsedSensors[0]);
    if (b_value < 4.294967296E+9) {
      if (b_value >= 0.0) {
        tmp_0 = static_cast<uint32_T>(b_value);
      } else {
        tmp_0 = 0U;
      }
    } else {
      tmp_0 = MAX_uint32_T;
    }

    in[1] = (tmp_0 == obj->pUsedSensors[1]);

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    AEBSensorFusion_eml_find(in, ind_data, ind_size);
    track->pObjectAttributes[ind_data[0] - 1] = Det_ObjectAttributes;
    loop_ub = ind_size[1];
    for (i = 0; i < loop_ub; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      track->pUsedObjectAttributes[ind_data[i] - 1] = true;
    }

    track->Time = Det_Time;
    track->UpdateTime = Det_Time;
    track->pAge = 1U;
    obj_0 = track->TrackLogic;
    for (i = 0; i < 50; i++) {
      obj_0->pRecentHistory[i] = false;
    }

    obj_0->pIsFirstUpdate = true;
    obj_0 = track->TrackLogic;
    tmp[0] = true;
    for (i = 0; i < 49; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      tmp[i + 1] = obj_0->pRecentHistory[i];
    }

    for (i = 0; i < 50; i++) {
      obj_0->pRecentHistory[i] = tmp[i];
    }

    obj_0->pIsFirstUpdate = false;
    filter = AEBSensorFusion_initcvekf(Det_Measurement, Det_MeasurementNoise,
      Det_MeasurementParameters_Frame, Det_MeasurementParameters_OriginPosition,
      Det_MeasurementParameters_Orientation,
      Det_MeasurementParameters_HasVelocity,
      Det_MeasurementParameters_OriginVelocity,
      Det_MeasurementParameters_IsParentToChild,
      Det_MeasurementParameters_HasElevation, &lobj_0[0]);
    for (i = 0; i < 6; i++) {
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        Det_MeasurementNoise_tmp = 6 * i + loop_ub;
        Det_MeasurementNoise_0[Det_MeasurementNoise_tmp] =
          (Det_MeasurementNoise[6 * loop_ub + i] +
           Det_MeasurementNoise[Det_MeasurementNoise_tmp]) / 2.0;
      }
    }

    AEBSensorFusion_eig(Det_MeasurementNoise_0, unusedExpr);

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    filter->pSqrtMeasurementNoiseScalar = -1.0;
    if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
      b_value = filter->pSqrtMeasurementNoiseScalar;
      for (i = 0; i < 36; i++) {
        filter->pSqrtMeasurementNoise[i] = b_value * static_cast<real_T>(tmp_1[i]);
      }

      filter->pSqrtMeasurementNoiseScalar = -1.0;
    }

    AEBSensorFusion_cholPSD(Det_MeasurementNoise, filter->pSqrtMeasurementNoise);
    track_0 = obj->pTracksList[newTrackIndex];
    AEBSensorFusion_trackingEKF_sync(track_0->Filter, filter);
    AEBSensorFusion_trackingEKF_sync(track_0->pDistanceFilter, filter);

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (Det_ObjectClassID != 0.0) {
      track->ObjectClassID = Det_ObjectClassID;
    }

    if (track->ObjectClassID != 0.0) {
      x_idx_0 = true;
    } else {
      obj_0 = track->TrackLogic;
      if (obj_0->pIsFirstUpdate) {
        x_idx_0 = false;
      } else {
        x_idx_0 = obj_0->pRecentHistory[0];
        x_idx_1 = obj_0->pRecentHistory[1];
        x_idx_2 = obj_0->pRecentHistory[2];
        x_idx_0 = ((x_idx_0 + x_idx_1) + x_idx_2 >= 2);
      }
    }

    track->IsConfirmed = x_idx_0;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    track->pIsCoasted = false;
    obj->pConfirmedTracks[newTrackIndex] = track->IsConfirmed;
    tf = true;
  } else {
    tf = false;
  }

  return tf;
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1g3pk(int32_T n,
  const real_T x_data[], int32_T ix0)
{
  real_T y;
  y = 0.0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return y;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_qrFactor(const real_T
  A_data[], const int32_T A_size[2], const real_T S[36], const real_T Ns[36],
  real_T b_S[36])
{
  real_T M_data[72];
  real_T y_data[36];
  real_T b_tau_data[6];
  real_T work[6];
  real_T beta1;
  real_T s;
  int32_T aoffset;
  int32_T b_i;
  int32_T coffset;
  int32_T e;
  int32_T exitg1;
  int32_T jA;
  int32_T jy;
  int32_T knt;
  int32_T lastc;
  int32_T m;
  int32_T m_tmp;
  int32_T mmip1;
  int8_T y[2];
  boolean_T exitg2;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  knt = A_size[0];
  for (m = 0; m < knt; m++) {
    coffset = m * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      aoffset = b_i * 6 - 1;
      s = 0.0;
      for (mmip1 = 0; mmip1 < 6; mmip1++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        s += S[(aoffset + mmip1) + 1] * A_data[mmip1 * A_size[0] + m];
      }

      y_data[(coffset + b_i) + 1] = s;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (A_size[0] != 0) {
    y[0] = 6;
  } else {
    y[0] = 0;
  }

  m_tmp = y[0] + 6;
  b_i = y[0];
  for (mmip1 = 0; mmip1 < 6; mmip1++) {
    for (aoffset = 0; aoffset < b_i; aoffset++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      M_data[aoffset + m_tmp * mmip1] = y_data[y[0] * mmip1 + aoffset];
    }

    for (aoffset = 0; aoffset < 6; aoffset++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      M_data[(aoffset + y[0]) + m_tmp * mmip1] = Ns[6 * aoffset + mmip1];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_tau_data[mmip1] = 0.0;
    work[mmip1] = 0.0;
  }

  for (b_i = 0; b_i < 6; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    aoffset = b_i * m_tmp + b_i;
    mmip1 = (m_tmp - b_i) - 1;
    if (b_i + 1 < m_tmp) {
      coffset = aoffset + 2;
      s = M_data[aoffset];
      b_tau_data[b_i] = 0.0;
      beta1 = AEBSensorFusion_xnrm2_po1g3pk(mmip1, M_data, aoffset + 2);
      if (beta1 != 0.0) {
        beta1 = rt_hypotd_snf(M_data[aoffset], beta1);
        if (M_data[aoffset] >= 0.0) {
          beta1 = -beta1;
        }

        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = -1;
          do {
            knt++;
            lastc = (aoffset + mmip1) + 1;
            for (m = coffset; m <= lastc; m++) {
              M_data[m - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            s *= 9.9792015476736E+291;
          } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

          beta1 = rt_hypotd_snf(s, AEBSensorFusion_xnrm2_po1g3pk(mmip1, M_data,
            aoffset + 2));
          if (s >= 0.0) {
            beta1 = -beta1;
          }

          b_tau_data[b_i] = (beta1 - s) / beta1;
          s = 1.0 / (s - beta1);
          for (m = coffset; m <= lastc; m++) {
            M_data[m - 1] *= s;
          }

          for (m = 0; m <= knt; m++) {
            beta1 *= 1.0020841800044864E-292;
          }

          s = beta1;
        } else {
          b_tau_data[b_i] = (beta1 - M_data[aoffset]) / beta1;
          s = 1.0 / (M_data[aoffset] - beta1);
          knt = (aoffset + mmip1) + 1;
          for (m = coffset; m <= knt; m++) {
            M_data[m - 1] *= s;
          }

          s = beta1;
        }
      }

      M_data[aoffset] = s;
    } else {
      b_tau_data[5] = 0.0;
    }

    if (b_i + 1 < 6) {
      s = M_data[aoffset];
      M_data[aoffset] = 1.0;
      coffset = (aoffset + m_tmp) + 1;
      if (b_tau_data[b_i] != 0.0) {
        m = mmip1;
        mmip1 += aoffset;
        while ((m + 1 > 0) && (M_data[mmip1] == 0.0)) {
          m--;
          mmip1--;
        }

        lastc = 5 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          mmip1 = (lastc - 1) * m_tmp + coffset;
          knt = mmip1;
          do {
            exitg1 = 0;
            if (knt <= mmip1 + m) {
              if (M_data[knt - 1] != 0.0) {
                exitg1 = 1;
              } else {
                knt++;
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
        m = -1;
        lastc = 0;
      }

      if (m + 1 > 0) {
        if (lastc != 0) {
          std::memset(&work[0], 0, static_cast<uint32_T>(lastc) * sizeof(real_T));
          jA = 0;
          mmip1 = (lastc - 1) * m_tmp + coffset;
          for (jy = coffset; m_tmp < 0 ? jy >= mmip1 : jy <= mmip1; jy += m_tmp)
          {
            beta1 = 0.0;
            e = jy + m;
            for (knt = jy; knt <= e; knt++) {
              beta1 += M_data[(aoffset + knt) - jy] * M_data[knt - 1];
            }

            work[jA] += beta1;
            jA++;
          }
        }

        if (!(-b_tau_data[b_i] == 0.0)) {
          jA = coffset;
          knt = lastc - 1;
          for (coffset = 0; coffset <= knt; coffset++) {
            beta1 = work[coffset];
            if (beta1 != 0.0) {
              beta1 *= -b_tau_data[b_i];
              mmip1 = m + jA;
              for (e = jA; e <= mmip1; e++) {
                M_data[e - 1] += M_data[(aoffset + e) - jA] * beta1;
              }
            }

            jA += m_tmp;
          }
        }
      }

      M_data[aoffset] = s;
    }
  }

  for (m = 0; m < 6; m++) {
    knt = m + 1;
    for (b_i = 0; b_i < knt; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      y_data[b_i + 6 * m] = M_data[m_tmp * m + b_i];
    }

    for (b_i = m + 2; b_i < 7; b_i++) {
      y_data[(b_i + 6 * m) - 1] = 0.0;
    }

    for (aoffset = 0; aoffset < 6; aoffset++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b_S[m + 6 * aoffset] = y_data[6 * m + aoffset];
    }
  }
}

void ACCWithSensorFusionModelClass::
  AEBSen_EKFCorrectorAdditive_getMeasurementJacobianAndCovariance(const real_T
  Rs[36], const real_T x[6], const real_T S[36], drivingCoordinateFrameType
  varargin_1_Frame, const real_T varargin_1_OriginPosition[3], const real_T
  varargin_1_Orientation[9], boolean_T varargin_1_HasVelocity, const real_T
  varargin_1_OriginVelocity[3], boolean_T varargin_1_IsParentToChild, boolean_T
  varargin_1_HasAzimuth, boolean_T varargin_1_HasElevation, boolean_T
  varargin_1_HasRange, real_T zEstimated_data[], int32_T zEstimated_size[1],
  real_T Pxy_data[], int32_T Pxy_size[2], real_T Sy[36], real_T dHdx_data[],
  int32_T dHdx_size[2], real_T wrapping_data[], int32_T wrapping_size[2])
{
  real_T y[36];
  real_T s;
  int32_T b;
  int32_T b_i;
  int32_T b_j;
  int32_T b_k;
  int32_T coffset;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_cvmeasjac(x, varargin_1_Frame, varargin_1_OriginPosition,
    varargin_1_Orientation, varargin_1_HasVelocity, varargin_1_OriginVelocity,
    varargin_1_IsParentToChild, varargin_1_HasAzimuth, varargin_1_HasElevation,
    varargin_1_HasRange, dHdx_data, dHdx_size);
  AEBSensorFusion_cvmeas_h(x, varargin_1_Frame, varargin_1_OriginPosition,
    varargin_1_Orientation, varargin_1_HasVelocity, varargin_1_OriginVelocity,
    varargin_1_IsParentToChild, varargin_1_HasAzimuth, varargin_1_HasElevation,
    varargin_1_HasRange, zEstimated_data, zEstimated_size, wrapping_data,
    wrapping_size);
  for (b_j = 0; b_j < 6; b_j++) {
    for (b_i = 0; b_i < 6; b_i++) {
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        s += S[6 * b_k + b_j] * S[6 * b_k + b_i];
      }

      y[b_j + 6 * b_i] = s;
    }
  }

  Pxy_size[0] = 6;
  b = dHdx_size[0];
  Pxy_size[1] = dHdx_size[0];
  for (b_j = 0; b_j < b; b_j++) {
    coffset = b_j * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        s += y[b_k * 6 + b_i] * dHdx_data[b_k * dHdx_size[0] + b_j];
      }

      Pxy_data[(coffset + b_i) + 1] = s;
    }
  }

  AEBSensorFusion_qrFactor(dHdx_data, dHdx_size, S, Rs, Sy);

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_wrapResidual(real_T
  residual[6], real_T bounds_data[], const int32_T bounds_size[2])
{
  real_T bounds_data_0[12];
  real_T e_data[6];
  real_T ma[6];
  real_T resToWrap_data[6];
  real_T e_data_tmp;
  int32_T b_tmp_data[6];
  int32_T m_data[6];
  int32_T tmp_size[2];
  int32_T b_tmp_size[1];
  int32_T isf_size[1];
  int32_T m_size[1];
  int32_T resToWrap_size[1];
  int32_T bounds_size_idx_0;
  int32_T i;
  int32_T loop_ub_tmp;
  int32_T trueCount;
  boolean_T tmp_data[12];
  boolean_T isf_data[6];
  boolean_T tmp[6];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  tmp_size[0] = bounds_size[0];
  tmp_size[1] = 2;
  loop_ub_tmp = bounds_size[0] << 1;
  for (i = 0; i < loop_ub_tmp; i++) {
    tmp_data[i] = ((!rtIsInf(bounds_data[i])) && (!rtIsNaN(bounds_data[i])));
  }

  AEBSensorFusion_all(tmp_data, tmp_size, isf_data, isf_size);
  if (AEBSensorFusion_any(isf_data, isf_size)) {
    loop_ub_tmp = isf_size[0];
    trueCount = 0;
    for (i = 0; i < loop_ub_tmp; i++) {
      if (isf_data[i]) {
        trueCount++;
      }
    }

    b_tmp_size[0] = trueCount;
    trueCount = 0;
    for (i = 0; i < loop_ub_tmp; i++) {
      if (isf_data[i]) {
        b_tmp_data[trueCount] = i;
        trueCount++;
      }
    }

    loop_ub_tmp = b_tmp_size[0];
    for (i = 0; i < loop_ub_tmp; i++) {
      trueCount = b_tmp_data[i];
      e_data[i] = (bounds_data[trueCount + bounds_size[0]] +
                   bounds_data[trueCount]) / 2.0;
    }

    bounds_size_idx_0 = b_tmp_size[0];
    for (i = 0; i < 2; i++) {
      for (trueCount = 0; trueCount < loop_ub_tmp; trueCount++) {
        bounds_data_0[trueCount + bounds_size_idx_0 * i] =
          bounds_data[bounds_size[0] * i + b_tmp_data[trueCount]] -
          e_data[trueCount];
      }
    }

    for (i = 0; i < 2; i++) {
      for (trueCount = 0; trueCount < loop_ub_tmp; trueCount++) {
        bounds_data[b_tmp_data[trueCount] + bounds_size[0] * i] =
          bounds_data_0[bounds_size_idx_0 * i + trueCount];
      }
    }

    for (i = 0; i < 6; i++) {
      ma[i] = std::abs(residual[i]);
    }

    if (bounds_size[0] == 6) {
      for (i = 0; i < 6; i++) {
        ma[i] /= bounds_data[i + bounds_size[0]];
      }
    } else {
      AEBSensorFusion_binary_expand_op_10(ma, bounds_data, bounds_size);
    }

    for (i = 0; i < 6; i++) {
      e_data[i] = std::abs(ma[i]);
    }

    if (isf_size[0] == 6) {
      for (i = 0; i < 6; i++) {
        tmp[i] = (isf_data[i] && (e_data[i] > 0.001));
      }
    } else {
      AEBSensorFusion_binary_expand_op_9(tmp, isf_data, isf_size, e_data);
    }

    trueCount = 0;
    for (i = 0; i < 6; i++) {
      if (tmp[i]) {
        trueCount++;
      }
    }

    m_size[0] = trueCount;
    trueCount = 0;
    for (i = 0; i < 6; i++) {
      if (tmp[i]) {
        m_data[trueCount] = i;
        trueCount++;
      }
    }

    loop_ub_tmp = m_size[0];
    for (i = 0; i < loop_ub_tmp; i++) {
      trueCount = m_data[i];
      e_data_tmp = bounds_data[trueCount];
      e_data[i] = residual[trueCount] - e_data_tmp;
      ma[i] = bounds_data[trueCount + bounds_size[0]] - e_data_tmp;
    }

    resToWrap_size[0] = m_size[0];
    for (i = 0; i < loop_ub_tmp; i++) {
      resToWrap_data[i] = AEBSensorFusion_mod(e_data[i], ma[i]);
    }

    if (resToWrap_size[0] == m_size[0]) {
      for (i = 0; i < loop_ub_tmp; i++) {
        trueCount = m_data[i];
        residual[trueCount] = resToWrap_data[i] + bounds_data[trueCount];
      }
    } else {
      AEBSensorFusion_binary_expand_op_8(residual, m_data, m_size,
        resToWrap_data, resToWrap_size, bounds_data);
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_trackingEKF_likelihood
  (c_trackingEKF_AEBSensorFusion_T *EKF, const real_T z[6], const
   BusDetectionConcatenation1DetectionsMeasurementParameters *varargin_1)
{
  real_T Sy[36];
  real_T a__1_data[36];
  real_T a__2_data[36];
  real_T val[36];
  real_T wrapping_data[12];
  real_T b_value[6];
  real_T zEstimated_data[6];
  real_T Y;
  real_T l;
  real_T val_0;
  int32_T b_ipiv[6];
  int32_T i;
  int32_T i_0;
  int32_T info;
  boolean_T isodd;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T a__1_size[2];
  int32_T a__2_size[2];
  int32_T wrapping_size[2];
  int32_T zEstimated_size[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((!EKF->pIsSetStateCovariance) || (EKF->pSqrtStateCovarianceScalar != -1.0))
  {
    for (info = 0; info < 6; info++) {
      for (i = 0; i < 6; i++) {
        i_0 = 6 * info + i;
        EKF->pSqrtStateCovariance[i_0] = static_cast<real_T>(tmp[i_0]) *
          EKF->pSqrtStateCovarianceScalar;
      }
    }

    EKF->pIsSetStateCovariance = true;
    EKF->pSqrtStateCovarianceScalar = -1.0;
  }

  if (EKF->pIsFirstCallCorrect) {
    if (!EKF->pIsValidMeasurementFcn) {
      EKF->pIsValidMeasurementFcn = true;
    }

    EKF->pIsFirstCallCorrect = false;
  }

  if (EKF->pSqrtMeasurementNoiseScalar > 0.0) {
    for (info = 0; info < 6; info++) {
      for (i = 0; i < 6; i++) {
        i_0 = 6 * info + i;
        EKF->pSqrtMeasurementNoise[i_0] = static_cast<real_T>(tmp[i_0]) *
          EKF->pSqrtMeasurementNoiseScalar;
      }
    }

    EKF->pSqrtMeasurementNoiseScalar = -1.0;
  }

  std::memcpy(&val[0], &EKF->pSqrtMeasurementNoise[0], 36U * sizeof(real_T));
  AEBSen_EKFCorrectorAdditive_getMeasurementJacobianAndCovariance(val,
    EKF->pState, EKF->pSqrtStateCovariance, varargin_1->Frame,
    varargin_1->OriginPosition, varargin_1->Orientation, varargin_1->HasVelocity,
    varargin_1->OriginVelocity, varargin_1->IsParentToChild,
    varargin_1->HasAzimuth, varargin_1->HasElevation, varargin_1->HasRange,
    zEstimated_data, zEstimated_size, a__1_data, a__1_size, Sy, a__2_data,
    a__2_size, wrapping_data, wrapping_size);
  if (zEstimated_size[0] == 6) {
    for (info = 0; info < 6; info++) {
      b_value[info] = z[info] - zEstimated_data[info];
    }
  } else {
    AEBSensorFusion_binary_expand_op_12(b_value, z, zEstimated_data,
      zEstimated_size);
  }

  AEBSensorFusion_wrapResidual(b_value, wrapping_data, wrapping_size);
  for (info = 0; info < 6; info++) {
    for (i = 0; i < 6; i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      val_0 = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        val_0 += Sy[6 * i_0 + info] * Sy[6 * i_0 + i];
      }

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      val[info + 6 * i] = val_0;
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    zEstimated_data[info] = b_value[info];
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_mrdiv_b(zEstimated_data, val);
  AEBSensorFusion_xzgetrf(val, b_ipiv, &info);
  val_0 = val[0];
  isodd = false;
  for (info = 0; info < 5; info++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    val_0 *= val[((info + 1) * 6 + info) + 1];
    if (b_ipiv[info] > info + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    val_0 = -val_0;
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  Y = 0.0;
  for (info = 0; info < 6; info++) {
    Y += zEstimated_data[info] * b_value[info];
  }

  l = std::exp(-Y / 2.0) / 248.05021344239853 / std::sqrt(val_0);
  if (!(l >= 2.2250738585072014E-308)) {
    l = 2.2250738585072014E-308;
  }

  return l;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_trisolve(const real_T A[36],
  real_T B[36])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (int32_T b_j = 0; b_j < 6; b_j++) {
    int32_T jBcol;
    jBcol = 6 * b_j - 1;
    for (int32_T b_k = 0; b_k < 6; b_k++) {
      real_T B_0;
      int32_T B_tmp;
      int32_T k;
      int32_T kAcol;
      k = b_k + 1;
      kAcol = b_k * 6 - 1;
      B_tmp = (b_k + jBcol) + 1;
      B_0 = B[B_tmp];
      if (B_0 != 0.0) {
        B[B_tmp] = B_0 / A[(b_k + kAcol) + 1];
        for (int32_T i = k + 1; i < 7; i++) {
          int32_T tmp;
          tmp = i + jBcol;
          B[tmp] -= A[i + kAcol] * B[B_tmp];
        }
      }
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_trisolve_l(const real_T A[36],
  real_T B[36])
{
  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (int32_T b_j = 0; b_j < 6; b_j++) {
    int32_T jBcol;
    jBcol = 6 * b_j;
    for (int32_T k = 5; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = 6 * k;
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

real_T ACCWithSensorFusionModelClass::AEBSensorFusion_xnrm2_po1g3pki(int32_T n,
  const real_T x[72], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
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

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  return scale * std::sqrt(y);
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_qrFactor_g(const real_T A[36],
  const real_T S[36], const real_T Ns[36], real_T b_S[36])
{
  real_T c_A[72];
  real_T y[36];
  real_T b_tau[6];
  real_T work[6];
  real_T beta1;
  real_T c_A_0;
  real_T s;
  int32_T aoffset;
  int32_T b_i;
  int32_T coffset;
  int32_T coffset_tmp;
  int32_T e;
  int32_T exitg1;
  int32_T ii;
  int32_T ix;
  int32_T jy;
  int32_T lastv;
  boolean_T exitg2;
  for (ii = 0; ii < 6; ii++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    b_tau[ii] = 0.0;
    for (b_i = 0; b_i < 6; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      aoffset = b_i * 6 - 1;
      s = 0.0;
      for (ix = 0; ix < 6; ix++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        s += S[(aoffset + ix) + 1] * A[ix * 6 + ii];
      }

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      aoffset = 12 * ii + b_i;
      c_A[aoffset] = s;

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      c_A[aoffset + 6] = Ns[6 * b_i + ii];
    }

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    work[ii] = 0.0;
  }

  for (b_i = 0; b_i < 6; b_i++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    ii = b_i * 12 + b_i;
    ix = ii + 2;
    s = c_A[ii];
    b_tau[b_i] = 0.0;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    beta1 = AEBSensorFusion_xnrm2_po1g3pki(11 - b_i, c_A, ii + 2);
    if (beta1 != 0.0) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      c_A_0 = c_A[ii];
      beta1 = rt_hypotd_snf(c_A_0, beta1);
      if (c_A_0 >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        aoffset = -1;
        do {
          aoffset++;
          coffset_tmp = ii - b_i;
          for (lastv = ix; lastv <= coffset_tmp + 12; lastv++) {
            c_A[lastv - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          s *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (aoffset + 1 <
                  20));

        beta1 = rt_hypotd_snf(s, AEBSensorFusion_xnrm2_po1g3pki(11 - b_i, c_A,
          ii + 2));
        if (s >= 0.0) {
          beta1 = -beta1;
        }

        b_tau[b_i] = (beta1 - s) / beta1;
        s = 1.0 / (s - beta1);
        for (lastv = ix; lastv <= coffset_tmp + 12; lastv++) {
          c_A[lastv - 1] *= s;
        }

        for (lastv = 0; lastv <= aoffset; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        s = beta1;
      } else {
        b_tau[b_i] = (beta1 - c_A_0) / beta1;
        s = 1.0 / (c_A_0 - beta1);
        coffset = ii - b_i;
        for (lastv = ix; lastv <= coffset + 12; lastv++) {
          c_A[lastv - 1] *= s;
        }

        s = beta1;
      }
    }

    c_A[ii] = s;

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    if (b_i + 1 < 6) {
      c_A[ii] = 1.0;
      coffset_tmp = ii + 13;
      if (b_tau[b_i] != 0.0) {
        lastv = 12 - b_i;
        coffset = ii - b_i;
        while ((lastv > 0) && (c_A[coffset + 11] == 0.0)) {
          lastv--;
          coffset--;
        }

        ix = 5 - b_i;
        exitg2 = false;
        while ((!exitg2) && (ix > 0)) {
          aoffset = (ix - 1) * 12 + ii;
          coffset = aoffset + 13;
          do {
            exitg1 = 0;
            if (coffset <= (aoffset + lastv) + 12) {
              if (c_A[coffset - 1] != 0.0) {
                exitg1 = 1;
              } else {
                coffset++;
              }
            } else {
              ix--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        ix = 0;
      }

      if (lastv > 0) {
        if (ix != 0) {
          std::memset(&work[0], 0, static_cast<uint32_T>(ix) * sizeof(real_T));
          aoffset = ((ix - 1) * 12 + ii) + 13;
          for (jy = coffset_tmp; jy <= aoffset; jy += 12) {
            beta1 = 0.0;
            e = jy + lastv;
            for (coffset = jy; coffset < e; coffset++) {
              beta1 += c_A[(ii + coffset) - jy] * c_A[coffset - 1];
            }

            coffset = div_nde_s32_floor((jy - ii) - 13, 12);
            work[coffset] += beta1;
          }
        }

        if (!(-b_tau[b_i] == 0.0)) {
          jy = ii;
          coffset = ix - 1;
          for (ix = 0; ix <= coffset; ix++) {
            beta1 = work[ix];
            if (beta1 != 0.0) {
              beta1 *= -b_tau[b_i];
              coffset_tmp = jy + 13;
              aoffset = (lastv + jy) + 12;
              for (e = coffset_tmp; e <= aoffset; e++) {
                c_A[e - 1] += c_A[((ii + e) - jy) - 13] * beta1;
              }
            }

            jy += 12;
          }
        }
      }

      c_A[ii] = s;
    }
  }

  for (ii = 0; ii < 6; ii++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    std::memcpy(&y[ii * 6], &c_A[ii * 12], static_cast<uint32_T>(ii + 1) *
                sizeof(real_T));
    for (b_i = ii + 2; b_i < 7; b_i++) {
      y[(b_i + 6 * ii) - 1] = 0.0;
    }
  }

  for (ii = 0; ii < 6; ii++) {
    for (b_i = 0; b_i < 6; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      b_S[b_i + 6 * ii] = y[6 * b_i + ii];
    }
  }
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_trackingEKF_correct
  (c_trackingEKF_AEBSensorFusion_T *filter, const real_T varargin_1[6],
   drivingCoordinateFrameType varargin_2_Frame, const real_T
   varargin_2_OriginPosition[3], const real_T varargin_2_Orientation[9],
   boolean_T varargin_2_HasVelocity, const real_T varargin_2_OriginVelocity[3],
   boolean_T varargin_2_IsParentToChild, boolean_T varargin_2_HasAzimuth,
   boolean_T varargin_2_HasElevation, boolean_T varargin_2_HasRange)
{
  real_T B_data[36];
  real_T C[36];
  real_T K[36];
  real_T Pxy_data[36];
  real_T dHdx_data[36];
  real_T wrapping_data[12];
  real_T residue[6];
  real_T zEstimated_data[6];
  real_T C_0;
  real_T K_0;
  int32_T C_tmp;
  int32_T b_i;
  int32_T b_j;
  int32_T loop_ub;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T Pxy_size[2];
  int32_T dHdx_size[2];
  int32_T wrapping_size[2];
  int32_T zEstimated_size[1];
  int32_T B_size_idx_0;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if ((!filter->pIsSetStateCovariance) || (filter->pSqrtStateCovarianceScalar !=
       -1.0)) {
    for (C_tmp = 0; C_tmp < 6; C_tmp++) {
      for (b_j = 0; b_j < 6; b_j++) {
        loop_ub = 6 * C_tmp + b_j;
        filter->pSqrtStateCovariance[loop_ub] = static_cast<real_T>(tmp[loop_ub])
          * filter->pSqrtStateCovarianceScalar;
      }
    }
  }

  if (filter->pIsFirstCallCorrect) {
    if (!filter->pIsValidMeasurementFcn) {
      filter->pIsValidMeasurementFcn = true;
    }

    filter->pIsFirstCallCorrect = false;
  }

  if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
    for (C_tmp = 0; C_tmp < 6; C_tmp++) {
      for (b_j = 0; b_j < 6; b_j++) {
        loop_ub = 6 * C_tmp + b_j;
        filter->pSqrtMeasurementNoise[loop_ub] = static_cast<real_T>(tmp[loop_ub])
          * filter->pSqrtMeasurementNoiseScalar;
      }
    }

    filter->pSqrtMeasurementNoiseScalar = -1.0;
  }

  AEBSen_EKFCorrectorAdditive_getMeasurementJacobianAndCovariance
    (filter->pSqrtMeasurementNoise, filter->pState, filter->pSqrtStateCovariance,
     varargin_2_Frame, varargin_2_OriginPosition, varargin_2_Orientation,
     varargin_2_HasVelocity, varargin_2_OriginVelocity,
     varargin_2_IsParentToChild, varargin_2_HasAzimuth, varargin_2_HasElevation,
     varargin_2_HasRange, zEstimated_data, zEstimated_size, Pxy_data, Pxy_size,
     K, dHdx_data, dHdx_size, wrapping_data, wrapping_size);
  if (zEstimated_size[0] == 6) {
    for (C_tmp = 0; C_tmp < 6; C_tmp++) {
      residue[C_tmp] = varargin_1[C_tmp] - zEstimated_data[C_tmp];
    }
  } else {
    AEBSensorFusion_binary_expand_op_12(residue, varargin_1, zEstimated_data,
      zEstimated_size);
  }

  AEBSensorFusion_wrapResidual(residue, wrapping_data, wrapping_size);
  loop_ub = Pxy_size[1];
  B_size_idx_0 = Pxy_size[1];
  for (b_j = 0; b_j < 6; b_j++) {
    for (C_tmp = 0; C_tmp < loop_ub; C_tmp++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      B_data[C_tmp + B_size_idx_0 * b_j] = Pxy_data[6 * C_tmp + b_j];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      C[b_i + 6 * b_j] = B_data[B_size_idx_0 * b_j + b_i];
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_trisolve(K, C);
  for (b_j = 0; b_j < 6; b_j++) {
    for (b_i = 0; b_i < 6; b_i++) {
      C_tmp = 6 * b_j + b_i;
      B_data[C_tmp] = C[C_tmp];
      Pxy_data[C_tmp] = K[6 * b_i + b_j];
    }
  }

  AEBSensorFusion_trisolve_l(Pxy_data, B_data);
  for (C_tmp = 0; C_tmp < 6; C_tmp++) {
    for (b_j = 0; b_j < 6; b_j++) {
      K[b_j + 6 * C_tmp] = B_data[6 * b_j + C_tmp];
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (C_tmp = 0; C_tmp < 36; C_tmp++) {
    Pxy_data[C_tmp] = -K[C_tmp];
  }

  for (C_tmp = 0; C_tmp < 6; C_tmp++) {
    for (b_j = 0; b_j < 6; b_j++) {
      C_0 = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
        C_0 += Pxy_data[6 * loop_ub + b_j] * dHdx_data[dHdx_size[0] * C_tmp +
          loop_ub];
      }

      C[b_j + 6 * C_tmp] = C_0;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (b_i = 0; b_i < 6; b_i++) {
    C_tmp = 6 * b_i + b_i;
    C[C_tmp]++;
    C_0 = 0.0;
    for (C_tmp = 0; C_tmp < 6; C_tmp++) {
      b_j = 6 * C_tmp + b_i;
      C_0 += K[b_j] * residue[C_tmp];
      K_0 = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        K_0 += K[6 * loop_ub + b_i] * filter->pSqrtMeasurementNoise[6 * C_tmp +
          loop_ub];
      }

      Pxy_data[b_j] = K_0;
    }

    filter->pState[b_i] += C_0;
  }

  std::memcpy(&K[0], &filter->pSqrtStateCovariance[0], 36U * sizeof(real_T));
  AEBSensorFusion_qrFactor_g(C, K, Pxy_data, filter->pSqrtStateCovariance);
  filter->pIsSetStateCovariance = true;
  filter->pSqrtStateCovarianceScalar = -1.0;
  filter->pHasPrediction = false;
  if (!filter->pIsInitialized) {
    filter->pIsDistributionsSetup = true;
  }

  filter->pWasRetrodicted = false;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_GNNTracker_initiateTracks
  (multiObjectTracker_AEBSensorFusion_T *obj, uint32_T OverallUnassigned_data[],
   int32_T OverallUnassigned_size[1], const BusDetectionConcatenation1Detections
   dets_data[], const int32_T dets_size[1], uint32_T initTrIDs_data[], int32_T
   initTrIDs_size[2])
{
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track;
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj_0;
  c_trackHistoryLogic_AEBSensorFusion_T *obj_1;
  c_trackingEKF_AEBSensorFusion_T *filter;
  real_T costMatrix_data[70];
  real_T detTimes_data[70];
  real_T h_data[70];
  real_T origSen_data[70];
  real_T dets_0[36];
  real_T tmp[36];
  real_T costValue_data[6];
  real_T b_value;
  real_T trackTime;
  real_T trackTime_tmp;
  int32_T iidx_data[70];
  int32_T iidx_data_0[70];
  int32_T tmp_data[70];
  int32_T idx_data[69];
  int32_T NewTrackNumber;
  int32_T e;
  int32_T i;
  int32_T i_0;
  int32_T lastTrackInd;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T m;
  int32_T trueCount;
  uint32_T checkedUnassigned_data[70];
  uint32_T ind_data[2];
  uint32_T OverallUnassigned_tmp;
  uint32_T q0;
  uint32_T qY;
  boolean_T sameSensor_data[70];
  boolean_T tmp_0[50];
  boolean_T dets[2];
  boolean_T tf;
  boolean_T x_idx_1;
  boolean_T x_idx_2;
  static const int8_T tmp_1[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  int32_T costValue_size[2];
  int32_T tmp_size[2];
  int32_T detTimes_size[1];
  int32_T iidx_size[1];
  boolean_T exitg1;
  boolean_T guard1;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  lastTrackInd = obj->pNumLiveTracks + 1;
  obj_0 = obj->cDetectionManager;
  trackTime = obj_0->pNumDetections;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (trackTime < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int32_T>(trackTime);
  }

  for (trueCount = 0; trueCount < loop_ub; trueCount++) {
    origSen_data[trueCount] = obj_0->pOriginatingSensor[trueCount];
  }

  exitg1 = false;
  while ((!exitg1) && (OverallUnassigned_size[0] != 0)) {
    NewTrackNumber = obj->pNumLiveTracks;
    q0 = obj->pLastTrackID;
    qY = q0 + /*MW:OvSatOk*/ 1U;
    if (q0 + 1U < q0) {
      qY = MAX_uint32_T;
    }

    trackTime = dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      SensorIndex;
    tf = AEBSensorFusion_TrackManager_initiateTrack(obj, qY, dets_data[
      static_cast<int32_T>(OverallUnassigned_data[0]) - 1].Time, dets_data[
      static_cast<int32_T>(OverallUnassigned_data[0]) - 1].Measurement,
      dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      MeasurementNoise, trackTime, dets_data[static_cast<int32_T>
      (OverallUnassigned_data[0]) - 1].ObjectClassID, dets_data[static_cast<
      int32_T>(OverallUnassigned_data[0]) - 1].MeasurementParameters.Frame,
      dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      MeasurementParameters.OriginPosition, dets_data[static_cast<int32_T>
      (OverallUnassigned_data[0]) - 1].MeasurementParameters.Orientation,
      dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      MeasurementParameters.HasVelocity, dets_data[static_cast<int32_T>
      (OverallUnassigned_data[0]) - 1].MeasurementParameters.OriginVelocity,
      dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      MeasurementParameters.IsParentToChild, dets_data[static_cast<int32_T>
      (OverallUnassigned_data[0]) - 1].MeasurementParameters.HasElevation,
      dets_data[static_cast<int32_T>(OverallUnassigned_data[0]) - 1].
      ObjectAttributes);
    if (tf) {
      obj->pLastTrackID = qY;
      loop_ub = OverallUnassigned_size[0];
      std::memset(&costMatrix_data[0], 0, static_cast<uint32_T>(loop_ub) *
                  sizeof(real_T));
      track = obj->pTracksList[NewTrackNumber];
      for (i = 0; i < loop_ub; i++) {
        sameSensor_data[i] = (origSen_data[static_cast<int32_T>
                              (OverallUnassigned_data[i]) - 1] == trackTime);
        if (sameSensor_data[i]) {
          costMatrix_data[i] = (rtInf);
        }
      }

      if (dets_size[0] == 0) {
      } else {
        trueCount = 0;
        for (i = 0; i < loop_ub; i++) {
          if (!sameSensor_data[i]) {
            trueCount++;
          }
        }

        m = trueCount;
        trueCount = 0;
        for (i = 0; i < loop_ub; i++) {
          if (!sameSensor_data[i]) {
            tmp_data[trueCount] = i;
            trueCount++;
          }
        }

        if (m == 0) {
        } else {
          detTimes_size[0] = m;
          for (i = 0; i < m; i++) {
            detTimes_data[i] = dets_data[static_cast<int32_T>
              (OverallUnassigned_data[tmp_data[i]]) - 1].Time;
            h_data[i] = (rtInf);
          }

          AEBSensorFusion_sort_mhq(detTimes_data, detTimes_size, iidx_data,
            iidx_size);
          trackTime = track->Time;
          AEBSensorFusion_trackingEKF_sync(track->pDistanceFilter, track->Filter);
          for (i = 0; i < m; i++) {
            filter = track->pDistanceFilter;
            for (trueCount = 0; trueCount < 36; trueCount++) {
              dets_0[trueCount] = dets_data[static_cast<int32_T>
                (OverallUnassigned_data[tmp_data[iidx_data[i] - 1]]) - 1].
                MeasurementNoise[trueCount];
            }

            for (trueCount = 0; trueCount < 6; trueCount++) {
              for (i_0 = 0; i_0 < 6; i_0++) {
                e = 6 * trueCount + i_0;
                tmp[e] = (dets_0[6 * i_0 + trueCount] + dets_0[e]) / 2.0;
              }
            }

            AEBSensorFusion_eig(tmp, unusedExpr);
            filter->pSqrtMeasurementNoiseScalar = -1.0;
            if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
              b_value = filter->pSqrtMeasurementNoiseScalar;
              for (trueCount = 0; trueCount < 36; trueCount++) {
                filter->pSqrtMeasurementNoise[trueCount] = b_value *
                  static_cast<real_T>(tmp_1[trueCount]);
              }

              filter->pSqrtMeasurementNoiseScalar = -1.0;
            }

            AEBSensorFusion_cholPSD(dets_0, filter->pSqrtMeasurementNoise);
            guard1 = false;
            if (track->ObjectClassID == 0.0) {
              guard1 = true;
            } else {
              trueCount = static_cast<int32_T>
                (OverallUnassigned_data[tmp_data[iidx_data[i] - 1]]) - 1;
              if ((dets_data[trueCount].ObjectClassID == 0.0) ||
                  (dets_data[trueCount].ObjectClassID == track->ObjectClassID))
              {
                guard1 = true;
              }
            }

            if (guard1) {
              trueCount = static_cast<int32_T>
                (OverallUnassigned_data[tmp_data[iidx_data[i] - 1]]) - 1;
              trackTime_tmp = dets_data[trueCount].Time;
              trackTime = trackTime_tmp - trackTime;
              if (trackTime > 0.0) {
                AEBSensorFusion_predictTrackFilter(track->pDistanceFilter,
                  trackTime);
              }

              trackTime = trackTime_tmp;
              AEBSensorFusion_ObjectTrack_calcCostOneDetection(track,
                dets_data[trueCount].Measurement, dets_data[trueCount].
                MeasurementParameters.Frame, dets_data[trueCount].
                MeasurementParameters.OriginPosition, dets_data[trueCount].
                MeasurementParameters.Orientation, dets_data[trueCount].
                MeasurementParameters.HasVelocity, dets_data[trueCount].
                MeasurementParameters.OriginVelocity, dets_data[trueCount].
                MeasurementParameters.IsParentToChild, dets_data[trueCount].
                MeasurementParameters.HasAzimuth, dets_data[trueCount].
                MeasurementParameters.HasElevation, dets_data[trueCount].
                MeasurementParameters.HasRange, costValue_data, costValue_size);
              h_data[i] = costValue_data[0];
            }
          }

          for (trueCount = 0; trueCount < m; trueCount++) {
            iidx_data_0[trueCount] = iidx_data[trueCount] - 1;
          }

          std::memcpy(&iidx_data[0], &iidx_data_0[0], static_cast<uint32_T>(m) *
                      sizeof(int32_T));
          std::memcpy(&detTimes_data[0], &h_data[0], static_cast<uint32_T>(m) *
                      sizeof(real_T));
          for (trueCount = 0; trueCount < m; trueCount++) {
            h_data[iidx_data[trueCount]] = detTimes_data[trueCount];
          }
        }
      }

      trueCount = 0;
      for (i = 0; i < loop_ub; i++) {
        if (!sameSensor_data[i]) {
          costMatrix_data[i] = h_data[trueCount];
          trueCount++;
        }

        checkedUnassigned_data[i] = 0U;
      }

      m = 0;
      e = OverallUnassigned_size[0] - 1;
      for (i = 0; i < e; i++) {
        if (costMatrix_data[i + 1] <= obj->pCostOfNonAssignment) {
          track = obj->pTracksList[NewTrackNumber];
          q0 = obj->pUsedSensors[0];
          qY = obj->pUsedSensors[1];
          OverallUnassigned_tmp = OverallUnassigned_data[i + 1];
          trackTime = dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1]
            .SensorIndex;
          dets[0] = (trackTime == q0);
          dets[1] = (trackTime == qY);
          AEBSensorFusion_eml_find(dets, costValue_size, tmp_size);
          loop_ub_0 = tmp_size[1];
          for (trueCount = 0; trueCount < loop_ub_0; trueCount++) {
            i_0 = costValue_size[trueCount];
            if (i_0 < 0) {
              i_0 = 0;
            }

            ind_data[trueCount] = static_cast<uint32_T>(i_0);
          }

          track->pObjectAttributes[static_cast<int32_T>(ind_data[0]) - 1] =
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            ObjectAttributes;
          for (trueCount = 0; trueCount < loop_ub_0; trueCount++) {
            costValue_size[trueCount] = static_cast<int32_T>(ind_data[trueCount]);
          }

          for (trueCount = 0; trueCount < loop_ub_0; trueCount++) {
            track->pUsedObjectAttributes[costValue_size[trueCount] - 1] = true;
          }

          trackTime_tmp = dets_data[static_cast<int32_T>(OverallUnassigned_tmp)
            - 1].Time;
          trackTime = trackTime_tmp - track->Time;
          if (!(trackTime <= 0.0)) {
            AEBSensorFusion_predictTrackFilter(track->Filter, trackTime);
            track->Time = trackTime_tmp;
          }

          filter = track->Filter;
          for (trueCount = 0; trueCount < 6; trueCount++) {
            for (i_0 = 0; i_0 < 6; i_0++) {
              loop_ub_0 = 6 * trueCount + i_0;
              dets_0[loop_ub_0] = (dets_data[static_cast<int32_T>
                                   (OverallUnassigned_tmp) - 1]
                                   .MeasurementNoise[6 * i_0 + trueCount] +
                                   dets_data[static_cast<int32_T>
                                   (OverallUnassigned_tmp) - 1]
                                   .MeasurementNoise[loop_ub_0]) / 2.0;
            }
          }

          AEBSensorFusion_eig(dets_0, unusedExpr);
          filter->pSqrtMeasurementNoiseScalar = -1.0;
          if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
            b_value = filter->pSqrtMeasurementNoiseScalar;
            for (trueCount = 0; trueCount < 36; trueCount++) {
              filter->pSqrtMeasurementNoise[trueCount] = b_value *
                static_cast<real_T>(tmp_1[trueCount]);
            }

            filter->pSqrtMeasurementNoiseScalar = -1.0;
          }

          AEBSensorFusion_cholPSD(dets_data[static_cast<int32_T>
            (OverallUnassigned_tmp) - 1].MeasurementNoise,
            filter->pSqrtMeasurementNoise);
          AEBSensorFusion_trackingEKF_likelihood(track->Filter, dets_data[
            static_cast<int32_T>(OverallUnassigned_tmp) - 1].Measurement,
            &dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters);
          AEBSensorFusion_trackingEKF_correct(track->Filter, dets_data[
            static_cast<int32_T>(OverallUnassigned_tmp) - 1].Measurement,
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters.Frame, dets_data[static_cast<int32_T>
            (OverallUnassigned_tmp) - 1].MeasurementParameters.OriginPosition,
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters.Orientation, dets_data[static_cast<int32_T>
            (OverallUnassigned_tmp) - 1].MeasurementParameters.HasVelocity,
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters.OriginVelocity, dets_data[static_cast<int32_T>
            (OverallUnassigned_tmp) - 1].MeasurementParameters.IsParentToChild,
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters.HasAzimuth, dets_data[static_cast<int32_T>
            (OverallUnassigned_tmp) - 1].MeasurementParameters.HasElevation,
            dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
            MeasurementParameters.HasRange);
          track->UpdateTime = trackTime_tmp;
          obj_1 = track->TrackLogic;
          tmp_0[0] = true;
          for (trueCount = 0; trueCount < 49; trueCount++) {
            tmp_0[trueCount + 1] = obj_1->pRecentHistory[trueCount];
          }

          for (trueCount = 0; trueCount < 50; trueCount++) {
            obj_1->pRecentHistory[trueCount] = tmp_0[trueCount];
          }

          obj_1->pIsFirstUpdate = false;
          q0 = track->pAge;
          qY = q0 + /*MW:OvSatOk*/ 1U;
          if (q0 + 1U < q0) {
            qY = MAX_uint32_T;
          }

          track->pAge = qY;
          if (dets_data[static_cast<int32_T>(OverallUnassigned_tmp) - 1].
              ObjectClassID != 0.0) {
            track->ObjectClassID = dets_data[static_cast<int32_T>
              (OverallUnassigned_tmp) - 1].ObjectClassID;
          }

          track->pIsCoasted = false;
          if (!track->IsConfirmed) {
            if (track->ObjectClassID != 0.0) {
              tf = true;
            } else {
              obj_1 = track->TrackLogic;
              if (obj_1->pIsFirstUpdate) {
                tf = false;
              } else {
                tf = obj_1->pRecentHistory[0];
                x_idx_1 = obj_1->pRecentHistory[1];
                x_idx_2 = obj_1->pRecentHistory[2];
                tf = ((tf + x_idx_1) + x_idx_2 >= 2);
              }
            }

            track->IsConfirmed = tf;
          }
        } else {
          m++;
          checkedUnassigned_data[m - 1] = OverallUnassigned_data[i + 1];
        }
      }

      obj->pConfirmedTracks[NewTrackNumber] = obj->pTracksList[NewTrackNumber]
        ->IsConfirmed;
      if (m > 0) {
        loop_ub_0 = OverallUnassigned_size[0] - m;
        for (trueCount = 0; trueCount < loop_ub_0; trueCount++) {
          idx_data[trueCount] = (m + trueCount) + 1;
        }

        for (trueCount = 0; trueCount < loop_ub; trueCount++) {
          OverallUnassigned_data[trueCount] = checkedUnassigned_data[trueCount];
          sameSensor_data[trueCount] = false;
        }

        trueCount = static_cast<uint8_T>(loop_ub_0);
        for (i = 0; i < trueCount; i++) {
          sameSensor_data[idx_data[i] - 1] = true;
        }

        NewTrackNumber = 0;
        m = -1;
        for (i = 0; i < loop_ub; i++) {
          tf = sameSensor_data[i];
          NewTrackNumber += tf;
          if ((i + 1 > loop_ub) || (!tf)) {
            m++;
            OverallUnassigned_data[m] = OverallUnassigned_data[i];
          }
        }

        NewTrackNumber = OverallUnassigned_size[0] - NewTrackNumber;
        if (NewTrackNumber < 1) {
          OverallUnassigned_size[0] = 0;
        } else {
          OverallUnassigned_size[0] = NewTrackNumber;
        }
      } else {
        exitg1 = true;
      }
    } else {
      exitg1 = true;
    }
  }

  i = obj->pNumLiveTracks;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (lastTrackInd > i) {
    NewTrackNumber = 0;
    i = 0;
  } else {
    NewTrackNumber = lastTrackInd - 1;
  }

  initTrIDs_size[0] = 1;
  loop_ub = i - NewTrackNumber;
  initTrIDs_size[1] = loop_ub;

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (trueCount = 0; trueCount < loop_ub; trueCount++) {
    initTrIDs_data[trueCount] = obj->pTrackIDs[NewTrackNumber + trueCount];
  }
}

void ACCWithSensorFusionModelClass::
  AEBSensorFusion_GNNTracker_updateAssignedTracks
  (multiObjectTracker_AEBSensorFusion_T *obj, const uint32_T
   OverallAssignments_data[], const int32_T OverallAssignments_size[2], const
   BusDetectionConcatenation1Detections dets_data[])
{
  BusDetectionConcatenation1Detections trackDetections_data[2];
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track;
  c_trackHistoryLogic_AEBSensorFusion_T *obj_0;
  c_trackingEKF_AEBSensorFusion_T *filter;
  real_T b_y_data[70];
  real_T trackDetections_0[36];
  real_T dt;
  real_T trackDetections_tmp;
  int32_T idx_data[70];
  int32_T iwork_data[70];
  int32_T tmp_data_1[2];
  int32_T b_idx_0_tmp;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T k;
  int32_T kEnd;
  int32_T n;
  int32_T p;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  uint32_T UniqueTracks_data[70];
  uint32_T ind_data[2];
  uint32_T q0;
  uint32_T qY;
  uint32_T usedSensors_idx_1;
  uint32_T x;
  int8_T tmp_data[70];
  boolean_T tmp_data_0[70];
  boolean_T tmp[50];
  boolean_T trackDetections[2];
  boolean_T x_idx_0;
  boolean_T x_idx_1;
  boolean_T x_idx_2;
  static const int8_T tmp_0[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  creal_T unusedExpr[6];
  int32_T tmp_size[2];
  int32_T b_y_size[1];
  int32_T idx_size[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (OverallAssignments_size[0] != 0) {
    n = OverallAssignments_size[0] + 1;
    b_idx_0_tmp = OverallAssignments_size[0];
    std::memset(&idx_data[0], 0, static_cast<uint32_T>(b_idx_0_tmp) * sizeof
                (int32_T));
    for (pEnd = 1; pEnd <= b_idx_0_tmp - 1; pEnd += 2) {
      if (OverallAssignments_data[pEnd - 1] <= OverallAssignments_data[pEnd]) {
        idx_data[pEnd - 1] = pEnd;
        idx_data[pEnd] = pEnd + 1;
      } else {
        idx_data[pEnd - 1] = pEnd + 1;
        idx_data[pEnd] = pEnd;
      }
    }

    if ((static_cast<uint32_T>(OverallAssignments_size[0]) & 1U) != 0U) {
      idx_data[OverallAssignments_size[0] - 1] = OverallAssignments_size[0];
    }

    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      pEnd = i + 1;
      while (pEnd < n) {
        p = j - 1;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          if (OverallAssignments_data[idx_data[p] - 1] <=
              OverallAssignments_data[idx_data[q] - 1]) {
            iwork_data[k] = idx_data[p];
            p++;
            if (p + 1 == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p + 1 < pEnd) {
                k++;
                iwork_data[k] = idx_data[p];
                p++;
              }
            }
          }

          k++;
        }

        for (pEnd = 0; pEnd < kEnd; pEnd++) {
          idx_data[(j + pEnd) - 1] = iwork_data[pEnd];
        }

        j = qEnd;
        pEnd = qEnd + i;
      }

      i = i2;
    }

    for (pEnd = 0; pEnd < b_idx_0_tmp; pEnd++) {
      UniqueTracks_data[pEnd] = OverallAssignments_data[idx_data[pEnd] - 1];
    }

    pEnd = -1;
    k = 0;
    while (k + 1 <= b_idx_0_tmp) {
      x = UniqueTracks_data[k];
      do {
        k++;
      } while (!((k + 1 > b_idx_0_tmp) || (UniqueTracks_data[k] != x)));

      pEnd++;
      UniqueTracks_data[pEnd] = x;
    }

    pEnd++;
    if (pEnd < 1) {
      pEnd = 0;
    }

    for (n = 0; n < pEnd; n++) {
      x = UniqueTracks_data[n];
      k = 0;
      for (i = 0; i < b_idx_0_tmp; i++) {
        tmp_data_0[i] = (OverallAssignments_data[i] == x);
        if (tmp_data_0[i]) {
          k++;
        }
      }

      j = k;
      k = 0;
      for (i = 0; i < b_idx_0_tmp; i++) {
        if (tmp_data_0[i]) {
          tmp_data[k] = static_cast<int8_T>(i);
          k++;
        }
      }

      if (j != 0) {
        for (k = 0; k < j; k++) {
          trackDetections_data[k] = dets_data[static_cast<int32_T>
            (OverallAssignments_data[tmp_data[0] + OverallAssignments_size[0]])
            - 1];
        }
      }

      if (j - 2 >= 0) {
        trackDetections_data[1] = dets_data[static_cast<int32_T>
          (OverallAssignments_data[tmp_data[1] + OverallAssignments_size[0]]) -
          1];
      }

      if (j != 0) {
        track = obj->pTracksList[static_cast<int32_T>(UniqueTracks_data[n]) - 1];
        x = obj->pUsedSensors[0];
        usedSensors_idx_1 = obj->pUsedSensors[1];
        b_y_size[0] = j;
        for (k = 0; k < j; k++) {
          b_y_data[k] = trackDetections_data[k].Time;
        }

        AEBSensorFusion_sort_mhq(b_y_data, b_y_size, idx_data, idx_size);
        q = idx_size[0];
        for (i = 0; i < q; i++) {
          b_y_data[i] = idx_data[i];
        }

        for (k = 0; k < j; k++) {
          p = static_cast<int32_T>(b_y_data[k]);
          trackDetections_tmp = trackDetections_data[p - 1].SensorIndex;
          trackDetections[0] = (trackDetections_tmp == x);
          trackDetections[1] = (trackDetections_tmp == usedSensors_idx_1);
          AEBSensorFusion_eml_find(trackDetections, tmp_data_1, tmp_size);
          q = tmp_size[1];
          for (i = 0; i < q; i++) {
            i2 = tmp_data_1[i];
            if (i2 < 0) {
              i2 = 0;
            }

            ind_data[i] = static_cast<uint32_T>(i2);
          }

          track->pObjectAttributes[static_cast<int32_T>(ind_data[0]) - 1] =
            trackDetections_data[p - 1].ObjectAttributes;
          for (i = 0; i < q; i++) {
            tmp_data_1[i] = static_cast<int32_T>(ind_data[i]);
          }

          for (i = 0; i < q; i++) {
            track->pUsedObjectAttributes[tmp_data_1[i] - 1] = true;
          }

          trackDetections_tmp = trackDetections_data[p - 1].Time;
          dt = trackDetections_tmp - track->Time;
          if (!(dt <= 0.0)) {
            AEBSensorFusion_predictTrackFilter(track->Filter, dt);
            track->Time = trackDetections_tmp;
          }

          filter = track->Filter;
          for (i = 0; i < 6; i++) {
            for (i2 = 0; i2 < 6; i2++) {
              q = 6 * i + i2;
              trackDetections_0[q] = (trackDetections_data[p - 1].
                MeasurementNoise[6 * i2 + i] + trackDetections_data[p - 1].
                MeasurementNoise[q]) / 2.0;
            }
          }

          AEBSensorFusion_eig(trackDetections_0, unusedExpr);
          filter->pSqrtMeasurementNoiseScalar = -1.0;
          if (filter->pSqrtMeasurementNoiseScalar > 0.0) {
            dt = filter->pSqrtMeasurementNoiseScalar;
            for (i = 0; i < 36; i++) {
              filter->pSqrtMeasurementNoise[i] = dt * static_cast<real_T>
                (tmp_0[i]);
            }

            filter->pSqrtMeasurementNoiseScalar = -1.0;
          }

          AEBSensorFusion_cholPSD(trackDetections_data[p - 1].MeasurementNoise,
            filter->pSqrtMeasurementNoise);
          AEBSensorFusion_trackingEKF_likelihood(track->Filter,
            trackDetections_data[p - 1].Measurement, &trackDetections_data[p - 1]
            .MeasurementParameters);
          AEBSensorFusion_trackingEKF_correct(track->Filter,
            trackDetections_data[p - 1].Measurement, trackDetections_data[p - 1]
            .MeasurementParameters.Frame, trackDetections_data[p - 1].
            MeasurementParameters.OriginPosition, trackDetections_data[p - 1].
            MeasurementParameters.Orientation, trackDetections_data[p - 1].
            MeasurementParameters.HasVelocity, trackDetections_data[p - 1].
            MeasurementParameters.OriginVelocity, trackDetections_data[p - 1].
            MeasurementParameters.IsParentToChild, trackDetections_data[p - 1].
            MeasurementParameters.HasAzimuth, trackDetections_data[p - 1].
            MeasurementParameters.HasElevation, trackDetections_data[p - 1].
            MeasurementParameters.HasRange);
          track->UpdateTime = trackDetections_tmp;
          obj_0 = track->TrackLogic;
          tmp[0] = true;
          for (i = 0; i < 49; i++) {
            tmp[i + 1] = obj_0->pRecentHistory[i];
          }

          for (i = 0; i < 50; i++) {
            obj_0->pRecentHistory[i] = tmp[i];
          }

          obj_0->pIsFirstUpdate = false;
          q0 = track->pAge;
          qY = q0 + /*MW:OvSatOk*/ 1U;
          if (q0 + 1U < q0) {
            qY = MAX_uint32_T;
          }

          track->pAge = qY;
        }

        for (k = 0; k < j; k++) {
          trackDetections_tmp = trackDetections_data[k].ObjectClassID;
          if (trackDetections_tmp != 0.0) {
            track->ObjectClassID = trackDetections_tmp;
          }
        }

        track->pIsCoasted = false;
        if (!track->IsConfirmed) {
          if (track->ObjectClassID != 0.0) {
            x_idx_0 = true;
          } else {
            obj_0 = track->TrackLogic;
            if (obj_0->pIsFirstUpdate) {
              x_idx_0 = false;
            } else {
              x_idx_0 = obj_0->pRecentHistory[0];
              x_idx_1 = obj_0->pRecentHistory[1];
              x_idx_2 = obj_0->pRecentHistory[2];
              x_idx_0 = ((x_idx_0 + x_idx_1) + x_idx_2 >= 2);
            }
          }

          track->IsConfirmed = x_idx_0;
        }
      }

      x = UniqueTracks_data[n];
      obj->pConfirmedTracks[static_cast<int32_T>(x) - 1] = obj->pTracksList[
        static_cast<int32_T>(x) - 1]->IsConfirmed;
    }
  }

  // End of Start for MATLABSystem: '<Root>/Multi-Object Tracker'
}

boolean_T ACCWithSensorFusionModelClass::
  AEBSensorFusion_ObjectTrack_checkDeletion
  (b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *track)
{
  c_trackHistoryLogic_AEBSensorFusion_T *obj;
  uint32_T age;
  boolean_T x_data[3];
  boolean_T tentativeTrack;
  boolean_T toDelete;
  obj = track->TrackLogic;
  tentativeTrack = !track->IsConfirmed;
  age = track->pAge;
  if (obj->pIsFirstUpdate) {
    toDelete = false;
  } else if (!tentativeTrack) {
    if (age > 3U) {
      boolean_T x_idx_1;
      boolean_T x_idx_2;
      tentativeTrack = !obj->pRecentHistory[0];
      x_idx_1 = !obj->pRecentHistory[1];
      x_idx_2 = !obj->pRecentHistory[2];

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      toDelete = ((tentativeTrack + x_idx_1) + x_idx_2 >= 3);
    } else {
      int32_T nz;
      int32_T x_size_idx_1_tmp;
      x_size_idx_1_tmp = static_cast<int32_T>(age);
      for (int32_T k = 0; k < x_size_idx_1_tmp; k++) {
        x_data[k] = !obj->pRecentHistory[k];
      }

      // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
      if (static_cast<int32_T>(age) == 0) {
        nz = 0;
      } else {
        nz = x_data[0];
        for (int32_T k = 2; k <= x_size_idx_1_tmp; k++) {
          nz += x_data[k - 1];
        }
      }

      toDelete = (nz >= 3);
    }
  } else {
    int32_T nz;
    uint32_T qY;
    uint32_T qY_0;
    boolean_T x_idx_1;
    boolean_T x_idx_2;
    tentativeTrack = obj->pRecentHistory[0];
    x_idx_1 = obj->pRecentHistory[1];
    x_idx_2 = obj->pRecentHistory[2];

    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    nz = (tentativeTrack + x_idx_1) + x_idx_2;
    qY = 2U - static_cast<uint32_T>(nz);
    if (2U - static_cast<uint32_T>(nz) > 2U) {
      qY = 0U;
    }

    qY_0 = 3U -
      /*MW:operator MISRA2012:D4.1 CERT-C:INT30-C 'Justifying MISRA C rule violation'*/
      /*MW:OvSatOk*/ age;
    if (3U - age > 3U) {
      qY_0 = 0U;
    }

    toDelete = (qY > qY_0);
  }

  return toDelete;
}

void ACCWithSensorFusionModelClass::AEBSensorFusion_GNNTracker_coreAlgorithm
  (multiObjectTracker_AEBSensorFusion_T *obj, const
   BusDetectionConcatenation1Detections dets_data[], const int32_T dets_size[1],
   real_T b_time)
{
  coder::bounded_array<int32_T, 20U, 2U> obj_1;
  coder::bounded_array<uint32_T, 20U, 2U> obj_2;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *temp;
  c_trackHistoryLogic_AEBSensorFusion_T *obj_0;
  real_T dt;
  int32_T liveTrackIdx_data[20];
  int32_T assigned_size[2];
  int32_T costMatrix_size[2];
  int32_T unassignedDets_size[1];
  int32_T unassignedTrs_size[1];
  int32_T NumberOfTracks;
  int32_T c;
  int32_T i;
  int32_T idx;
  int32_T liveTrackIdx_tmp;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  int32_T nz;
  uint32_T assigned_data[140];
  uint32_T unassignedDets_data[70];
  uint32_T m_data[21];
  uint32_T unassignedTrs_data[20];
  uint32_T q0;
  uint32_T qY;
  uint32_T unassignedTrs;
  boolean_T tmp[50];
  boolean_T g_data[21];
  boolean_T toDelete[20];
  boolean_T exitg1;
  boolean_T x_idx_0;
  boolean_T x_idx_1;
  boolean_T x_idx_2;
  obj->cCostCalculator.MaxAssignmentCost = obj->AssignmentThreshold[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_TrackManager_getLiveTrackIndices(obj, obj_1.data, obj_1.size);
  AEBSensorFusion_GNNTracker_calcCostMatrixAllSensors(obj, dets_data, dets_size,
    AEBSensorFusion_DW.costMatrix_data, costMatrix_size);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_TrackManager_getLiveTrackIndices(obj, obj_1.data, obj_1.size);
  AEBSensorFusion_GNNTracker_associate(obj, AEBSensorFusion_DW.costMatrix_data,
    costMatrix_size, assigned_data, assigned_size, unassignedTrs_data,
    unassignedTrs_size, unassignedDets_data, unassignedDets_size);

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_TrackManager_getLiveTrackIndices(obj, liveTrackIdx_data,
    costMatrix_size);
  loop_ub = costMatrix_size[1];
  for (c = 0; c < loop_ub; c++) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    obj->pWasDetectable[liveTrackIdx_data[c] - 1] = true;
  }

  AEBSensorFusion_GNNTracker_initiateTracks(obj, unassignedDets_data,
    unassignedDets_size, dets_data, dets_size, obj_2.data, obj_2.size);
  AEBSensorFusion_GNNTracker_updateAssignedTracks(obj, assigned_data,
    assigned_size, dets_data);
  idx = unassignedTrs_size[0];
  for (i = idx; i >= 1; i--) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    unassignedTrs = unassignedTrs_data[i - 1];
    if (obj->pWasDetectable[static_cast<int32_T>(unassignedTrs) - 1]) {
      temp = obj->pTracksList[static_cast<int32_T>(unassignedTrs) - 1];
      temp->pIsCoasted = true;
      obj_0 = temp->TrackLogic;
      tmp[0] = false;
      for (c = 0; c < 49; c++) {
        tmp[c + 1] = obj_0->pRecentHistory[c];
      }

      for (c = 0; c < 50; c++) {
        obj_0->pRecentHistory[c] = tmp[c];
      }

      obj_0->pIsFirstUpdate = false;
      q0 = temp->pAge;
      qY = q0 + /*MW:OvSatOk*/ 1U;
      if (q0 + 1U < q0) {
        qY = MAX_uint32_T;
      }

      temp->pAge = qY;
    }

    if (obj->pConfThreshChanged && (!obj->pTracksList[static_cast<int32_T>
         (unassignedTrs) - 1]->IsConfirmed)) {
      temp = obj->pTracksList[static_cast<int32_T>(unassignedTrs) - 1];
      if (temp->ObjectClassID != 0.0) {
        x_idx_0 = true;
      } else {
        obj_0 = temp->TrackLogic;
        if (obj_0->pIsFirstUpdate) {
          x_idx_0 = false;
        } else {
          x_idx_0 = obj_0->pRecentHistory[0];
          x_idx_1 = obj_0->pRecentHistory[1];
          x_idx_2 = obj_0->pRecentHistory[2];
          x_idx_0 = ((x_idx_0 + x_idx_1) + x_idx_2 >= 2);
        }
      }

      obj->pTracksList[static_cast<int32_T>(unassignedTrs) - 1]->IsConfirmed =
        x_idx_0;
      obj->pConfirmedTracks[static_cast<int32_T>(unassignedTrs) - 1] =
        obj->pTracksList[static_cast<int32_T>(unassignedTrs) - 1]->IsConfirmed;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  obj->pConfThreshChanged = false;
  for (c = 0; c < 20; c++) {
    toDelete[c] = false;
  }

  for (i = idx; i >= 1; i--) {
    // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
    unassignedTrs = unassignedTrs_data[i - 1];
    toDelete[static_cast<int32_T>(unassignedTrs) - 1] =
      AEBSensorFusion_ObjectTrack_checkDeletion(obj->pTracksList
      [static_cast<int32_T>(unassignedTrs) - 1]);
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (obj->pDelThreshChanged) {
    NumberOfTracks = assigned_size[0];
    for (idx = NumberOfTracks; idx >= 1; idx--) {
      unassignedTrs = assigned_data[idx - 1];
      toDelete[static_cast<int32_T>(unassignedTrs) - 1] =
        AEBSensorFusion_ObjectTrack_checkDeletion(obj->pTracksList
        [static_cast<int32_T>(unassignedTrs) - 1]);
    }

    obj->pDelThreshChanged = false;
  }

  nz = toDelete[0];
  for (i = 0; i < 19; i++) {
    nz += toDelete[i + 1];
  }

  idx = 0;
  i = 1;
  exitg1 = false;
  while ((!exitg1) && (i - 1 < 20)) {
    if (toDelete[i - 1]) {
      idx++;
      liveTrackIdx_data[idx - 1] = i;
      if (idx >= 20) {
        exitg1 = true;
      } else {
        i++;
      }
    } else {
      i++;
    }
  }

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  for (idx = 0; idx < nz; idx++) {
    AEBSensorFusion_ObjectTrack_nullify(obj->pTracksList[liveTrackIdx_data[idx]
      - 1]);
  }

  for (idx = nz; idx >= 1; idx--) {
    liveTrackIdx_tmp = liveTrackIdx_data[idx - 1];
    temp = obj->pTracksList[liveTrackIdx_tmp - 1];
    c = obj->pNumLiveTracks;
    for (i = liveTrackIdx_tmp + 1; i <= c; i++) {
      obj->pTracksList[i - 2] = obj->pTracksList[i - 1];
    }

    obj->pTracksList[obj->pNumLiveTracks - 1] = temp;
    obj->pNumLiveTracks--;
    if (liveTrackIdx_tmp + 1 > 20) {
      i = 0;
      NumberOfTracks = -1;
    } else {
      i = liveTrackIdx_tmp;
      NumberOfTracks = 19;
    }

    loop_ub = NumberOfTracks - i;
    loop_ub_tmp = loop_ub + 2;
    for (c = 0; c <= loop_ub; c++) {
      g_data[c] = obj->pConfirmedTracks[i + c];
    }

    g_data[loop_ub + 1] = false;
    for (c = 0; c < loop_ub_tmp; c++) {
      obj->pConfirmedTracks[(liveTrackIdx_tmp + c) - 1] = g_data[c];
    }

    if (liveTrackIdx_tmp + 1 > 20) {
      NumberOfTracks = 0;
      i = -1;
    } else {
      NumberOfTracks = liveTrackIdx_tmp;
      i = 19;
    }

    loop_ub = i - NumberOfTracks;
    loop_ub_tmp = loop_ub + 2;
    for (c = 0; c <= loop_ub; c++) {
      m_data[c] = obj->pTrackIDs[NumberOfTracks + c];
    }

    m_data[loop_ub + 1] = 0U;
    for (c = 0; c < loop_ub_tmp; c++) {
      obj->pTrackIDs[(liveTrackIdx_tmp + c) - 1] = m_data[c];
    }
  }

  nz = obj->pNumLiveTracks;
  if (b_time > obj->pLastTimeStamp) {
    for (idx = 0; idx < nz; idx++) {
      if (b_time > obj->pTracksList[idx]->Time) {
        temp = obj->pTracksList[idx];
        dt = b_time - temp->Time;
        if (!(dt <= 0.0)) {
          AEBSensorFusion_predictTrackFilter(temp->Filter, dt);
          temp->Time = b_time;
        }
      }
    }
  }

  obj->pLastTimeStamp = b_time;
}

// System initialize for referenced model: 'AEBSensorFusion'
void ACCWithSensorFusionModelClass::init(void)
{
  real_T x;
  int32_T i;
  boolean_T flag;
  static const BusVisionDetections tmp = { 0.0,// Time
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // Measurement

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // MeasurementNoise
    0.0,                               // SensorIndex
    0.0,                               // ObjectClassID

    { 0U,                              // Frame
      { 0.0, 0.0, 0.0 },               // OriginPosition

      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },// Orientation
      false                            // HasVelocity
    },                                 // MeasurementParameters

    { 0.0                              // TargetIndex
    }                                  // ObjectAttributes
  };

  // Start for MATLABSystem: '<Root>/Detection Concatenation'
  AEBSensorFusion_DW.objisempty_m = true;
  AEBSensorFusion_DW.obj_c.isInitialized = 1;
  for (i = 0; i < 20; i++) {
    AEBSensorFusion_DW.rv[i] = tmp;
  }

  AEBSensorFusion_DetectionConcatenation_setupImpl(&AEBSensorFusion_DW.obj_c,
    AEBSensorFusion_DW.rv);

  // End of Start for MATLABSystem: '<Root>/Detection Concatenation'

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker' incorporates:
  //   MATLABSystem: '<Root>/Detection Concatenation'

  AEBSensorFusion_DW.obj.pConfThreshChanged = false;
  AEBSensorFusion_DW.obj.pDelThreshChanged = false;
  AEBSensorFusion_DW.obj.isInitialized = 0;
  AEBSensorFusion_DW.obj.tunablePropertyChanged[0] = false;
  AEBSensorFusion_DW.obj.tunablePropertyChanged[1] = false;
  AEBSensorFusion_DW.obj.matlabCodegenIsDeleted = false;
  AEBSensorFusion_DW.objisempty = true;
  flag = (AEBSensorFusion_DW.obj.isInitialized == 1);
  if (flag) {
    AEBSensorFusion_DW.obj.TunablePropsChanged = true;
    AEBSensorFusion_DW.obj.tunablePropertyChanged[0] = true;
  }

  AEBSensorFusion_DW.obj.AssignmentThreshold[0] = 50.0;
  AEBSensorFusion_DW.obj.AssignmentThreshold[1] = (rtInf);
  x = AEBSensorFusion_DW.obj.AssignmentThreshold[1];
  AEBSensorFusion_DW.obj.AssignmentThreshold[1] = std::abs(x);
  AEBSensorFusion_SystemCore_setup(&AEBSensorFusion_DW.obj,
    AEBSensorFusion_DW.DetectionConcatenation.NumDetections,
    AEBSensorFusion_DW.DetectionConcatenation.Detections);

  // InitializeConditions for MATLABSystem: '<Root>/Multi-Object Tracker'
  AEBSensorFusion_GNNTracker_resetImpl(&AEBSensorFusion_DW.obj);
}

// Output and update for referenced model: 'AEBSensorFusion'
void ACCWithSensorFusionModelClass::step(const BusVision *rtu_Vision, const
  BusRadar *rtu_Radar, const real_T *rtu_SystemTime, BusMultiObjectTracker1
  *rty_Tracks)
{
  BusDetectionConcatenation1Detections expl_temp_1;
  BusDetectionConcatenation1Detections expl_temp_2;
  BusMultiObjectTracker1Tracks expl_temp_3;
  BusMultiObjectTracker1Tracks track;
  BusRadarDetections expl_temp;
  BusVisionDetections expl_temp_0;
  b_matlabshared_tracking_internal_fusion_ObjectTrack_AEBSensor_T *tracksList[20];
  c_matlabshared_tracking_internal_fusion_SimulinkDetectionMana_T *obj;
  sMHza4QfTveUgeWCFWTBlDH_AEBSensorFusion_T expl_temp_4;
  real_T sensors_data[70];
  real_T n_0;
  real_T tmp;
  int32_T ia_data[70];
  int32_T liveTracks_data[20];
  int32_T tmp_data[20];
  int32_T sensors_size[2];
  int32_T tmp_size[2];
  int32_T b_dets_size[1];
  int32_T ib_size[1];
  int32_T ii_data[1];
  int32_T b_n;
  int32_T idx;
  int32_T ii_size_idx_1;
  int32_T n;
  int32_T numDetsIn;
  uint32_T sensors_data_0[70];
  uint32_T tmp_data_0[70];
  uint32_T tmp_0[2];
  uint32_T x_0[2];
  boolean_T list[20];
  boolean_T x[2];
  boolean_T exitg1;
  boolean_T y;

  // MATLABSystem: '<Root>/Detection Concatenation'
  AEBSensorFusion_DW.r = AEBSensorFusion_DW.obj_c.pOutTemp;
  numDetsIn = static_cast<int32_T>(rtu_Vision->NumDetections) - 1;
  for (b_n = 0; b_n <= numDetsIn; b_n++) {
    expl_temp_0 = rtu_Vision->Detections[b_n];
    for (n = 0; n < 6; n++) {
      expl_temp_2.Measurement[n] = expl_temp_0.Measurement[n];
    }

    std::memcpy(&expl_temp_2.MeasurementNoise[0], &expl_temp_0.MeasurementNoise
                [0], 36U * sizeof(real_T));
    expl_temp_2.MeasurementParameters.OriginPosition[0] = rtu_Vision->
      Detections[b_n].MeasurementParameters.OriginPosition[0];
    expl_temp_2.MeasurementParameters.OriginPosition[1] = rtu_Vision->
      Detections[b_n].MeasurementParameters.OriginPosition[1];
    expl_temp_2.MeasurementParameters.OriginPosition[2] = rtu_Vision->
      Detections[b_n].MeasurementParameters.OriginPosition[2];
    std::memcpy(&expl_temp_2.MeasurementParameters.Orientation[0],
                &rtu_Vision->Detections[b_n].MeasurementParameters.Orientation[0],
                9U * sizeof(real_T));
    expl_temp_2.Time = expl_temp_0.Time;
    expl_temp_2.SensorIndex = expl_temp_0.SensorIndex;
    expl_temp_2.ObjectClassID = expl_temp_0.ObjectClassID;
    expl_temp_2.MeasurementParameters.Frame = rtu_Vision->Detections[b_n].
      MeasurementParameters.Frame;
    expl_temp_2.MeasurementParameters.HasVelocity = rtu_Vision->Detections[b_n].
      MeasurementParameters.HasVelocity;
    expl_temp_2.MeasurementParameters.OriginVelocity[0] =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.OriginVelocity
      [0];
    expl_temp_2.MeasurementParameters.OriginVelocity[1] =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.OriginVelocity
      [1];
    expl_temp_2.MeasurementParameters.OriginVelocity[2] =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.OriginVelocity
      [2];
    expl_temp_2.MeasurementParameters.IsParentToChild =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.IsParentToChild;
    expl_temp_2.MeasurementParameters.HasAzimuth =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.HasAzimuth;
    expl_temp_2.MeasurementParameters.HasElevation =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.HasElevation;
    expl_temp_2.MeasurementParameters.HasRange =
      AEBSensorFusion_DW.r.Detections[b_n].MeasurementParameters.HasRange;
    expl_temp_2.ObjectAttributes.TargetIndex = rtu_Vision->Detections[b_n].
      ObjectAttributes.TargetIndex;
    expl_temp_2.ObjectAttributes.SNR = AEBSensorFusion_DW.r.Detections[b_n].
      ObjectAttributes.SNR;
    AEBSensorFusion_DW.r.Detections[b_n] = expl_temp_2;
  }

  idx = (static_cast<int32_T>(rtu_Vision->NumDetections) - 1 < 0 ? 1 :
         static_cast<int32_T>(rtu_Vision->NumDetections) + 1) - 1;
  numDetsIn = static_cast<int32_T>(rtu_Radar->NumDetections);
  for (b_n = 0; b_n < numDetsIn; b_n++) {
    expl_temp = rtu_Radar->Detections[b_n];
    for (n = 0; n < 6; n++) {
      expl_temp_1.Measurement[n] = expl_temp.Measurement[n];
    }

    std::memcpy(&expl_temp_1.MeasurementNoise[0], &expl_temp.MeasurementNoise[0],
                36U * sizeof(real_T));
    expl_temp_1.MeasurementParameters.OriginPosition[0] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginPosition[0];
    expl_temp_1.MeasurementParameters.OriginVelocity[0] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginVelocity[0];
    expl_temp_1.MeasurementParameters.OriginPosition[1] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginPosition[1];
    expl_temp_1.MeasurementParameters.OriginVelocity[1] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginVelocity[1];
    expl_temp_1.MeasurementParameters.OriginPosition[2] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginPosition[2];
    expl_temp_1.MeasurementParameters.OriginVelocity[2] = rtu_Radar->
      Detections[b_n].MeasurementParameters.OriginVelocity[2];
    std::memcpy(&expl_temp_1.MeasurementParameters.Orientation[0],
                &rtu_Radar->Detections[b_n].MeasurementParameters.Orientation[0],
                9U * sizeof(real_T));
    expl_temp_1.Time = expl_temp.Time;
    expl_temp_1.SensorIndex = expl_temp.SensorIndex;
    expl_temp_1.ObjectClassID = expl_temp.ObjectClassID;
    expl_temp_1.MeasurementParameters.Frame = rtu_Radar->Detections[b_n].
      MeasurementParameters.Frame;
    expl_temp_1.MeasurementParameters.HasVelocity = rtu_Radar->Detections[b_n].
      MeasurementParameters.HasVelocity;
    expl_temp_1.MeasurementParameters.IsParentToChild = rtu_Radar->
      Detections[b_n].MeasurementParameters.IsParentToChild;
    expl_temp_1.MeasurementParameters.HasAzimuth = rtu_Radar->Detections[b_n].
      MeasurementParameters.HasAzimuth;
    expl_temp_1.MeasurementParameters.HasElevation = rtu_Radar->Detections[b_n].
      MeasurementParameters.HasElevation;
    expl_temp_1.MeasurementParameters.HasRange = rtu_Radar->Detections[b_n].
      MeasurementParameters.HasRange;
    expl_temp_1.ObjectAttributes = rtu_Radar->Detections[b_n].ObjectAttributes;
    AEBSensorFusion_DW.r.Detections[idx + b_n] = expl_temp_1;
  }

  AEBSensorFusion_DW.r.NumDetections = static_cast<int32_T>
    (rtu_Vision->NumDetections) + static_cast<int32_T>(rtu_Radar->NumDetections);
  AEBSensorFusion_DW.r.IsValidTime = (rtu_Vision->IsValidTime ||
    rtu_Radar->IsValidTime);

  // MATLABSystem: '<Root>/Detection Concatenation'
  std::memcpy(&AEBSensorFusion_DW.DetectionConcatenation, &AEBSensorFusion_DW.r,
              sizeof(BusDetectionConcatenation1));

  // MATLABSystem: '<Root>/Multi-Object Tracker' incorporates:
  //   MATLABSystem: '<Root>/Detection Concatenation'

  x[0] = (AEBSensorFusion_DW.obj.AssignmentThreshold[0] != 50.0);
  x[1] = (AEBSensorFusion_DW.obj.AssignmentThreshold[1] != 50.0);
  y = true;
  b_n = 0;
  exitg1 = false;
  while ((!exitg1) && (b_n < 2)) {
    if (!x[b_n]) {
      y = false;
      exitg1 = true;
    } else {
      b_n++;
    }
  }

  if (y) {
    y = (AEBSensorFusion_DW.obj.isInitialized == 1);
    if (y) {
      AEBSensorFusion_DW.obj.TunablePropsChanged = true;
      AEBSensorFusion_DW.obj.tunablePropertyChanged[0] = true;
    }

    AEBSensorFusion_DW.obj.AssignmentThreshold[0] = 50.0;
    AEBSensorFusion_DW.obj.AssignmentThreshold[1] = (rtInf);
    n_0 = AEBSensorFusion_DW.obj.AssignmentThreshold[1];
    AEBSensorFusion_DW.obj.AssignmentThreshold[1] = std::abs(n_0);
  }

  if (AEBSensorFusion_DW.obj.TunablePropsChanged) {
    AEBSensorFusion_DW.obj.TunablePropsChanged = false;
    y = AEBSensorFusion_DW.obj.tunablePropertyChanged[0];
    if (y) {
      n_0 = AEBSensorFusion_DW.obj.AssignmentThreshold[0];
      tmp = AEBSensorFusion_DW.obj.AssignmentThreshold[1];
      AEBSensorFusion_DW.obj.cAssigner->AssignmentThreshold[0] = n_0;
      AEBSensorFusion_DW.obj.cAssigner->AssignmentThreshold[1] = tmp;
    }

    AEBSensorFusion_DW.obj.tunablePropertyChanged[0] = false;
    AEBSensorFusion_DW.obj.tunablePropertyChanged[1] = false;
  }

  obj = AEBSensorFusion_DW.obj.cDetectionManager;
  n_0 = AEBSensorFusion_DW.obj.pLastTimeStamp;
  if (AEBSensorFusion_DW.obj.cDetectionManager->isInitialized != 1) {
    AEBSensorFusion_DW.obj.cDetectionManager->isInitialized = 1;
    AEBSensorFusion_DW.obj.cDetectionManager->pNumDetections =
      AEBSensorFusion_DW.DetectionConcatenation.NumDetections;
    AEBSensorFusion_DW.obj.cDetectionManager->pNumDetections = 0.0;
    for (b_n = 0; b_n < 70; b_n++) {
      obj->pDetections[b_n] =
        AEBSensorFusion_DW.DetectionConcatenation.Detections[0];
      obj->pOriginatingSensor[b_n] = 0.0;
      obj->pIsOOSM[b_n] = false;
      obj->pOriginatingSensor[b_n] = 0.0;
      obj->pIsOOSM[b_n] = false;
    }
  }

  obj->pNumDetections = AEBSensorFusion_DW.DetectionConcatenation.NumDetections;
  if (AEBSensorFusion_DW.DetectionConcatenation.NumDetections > 0.0) {
    tmp = obj->pNumDetections;
    numDetsIn = static_cast<int32_T>(tmp);
    for (b_n = 0; b_n < numDetsIn; b_n++) {
      obj->pDetections[b_n] =
        AEBSensorFusion_DW.DetectionConcatenation.Detections[b_n];
    }
  }

  for (n = 0; n < 70; n++) {
    obj->pOriginatingSensor[n] = 0.0;
    obj->pIsOOSM[n] = false;
  }

  tmp = obj->pNumDetections;
  numDetsIn = static_cast<int32_T>(tmp);
  for (b_n = 0; b_n < numDetsIn; b_n++) {
    obj->pOriginatingSensor[b_n] = obj->pDetections[b_n].SensorIndex;
    tmp = obj->pDetections[b_n].Time;
    obj->pIsOOSM[b_n] = (tmp <= n_0);
  }

  AEBSensorFusion_DW.obj.pNumDetections =
    AEBSensorFusion_DW.DetectionConcatenation.NumDetections;
  tmp = AEBSensorFusion_DW.obj.cDetectionManager->pNumDetections;
  if (tmp < 1.0) {
    idx = 0;
  } else {
    idx = static_cast<int32_T>(tmp);
  }

  for (n = 0; n < idx; n++) {
    sensors_data[n] =
      AEBSensorFusion_DW.obj.cDetectionManager->pOriginatingSensor[n];
  }

  x_0[0] = AEBSensorFusion_DW.obj.pUsedSensors[0];
  x_0[1] = AEBSensorFusion_DW.obj.pUsedSensors[1];

  // Start for MATLABSystem: '<Root>/Multi-Object Tracker'
  sensors_size[0] = 1;
  sensors_size[1] = idx;
  for (n = 0; n < idx; n++) {
    n_0 = rt_roundd_snf(sensors_data[n]);
    if (n_0 < 4.294967296E+9) {
      if (n_0 >= 0.0) {
        sensors_data_0[n] = static_cast<uint32_T>(n_0);
      } else {
        sensors_data_0[n] = 0U;
      }
    } else {
      sensors_data_0[n] = MAX_uint32_T;
    }
  }

  AEBSensorFusion_sort(sensors_data_0, sensors_size, tmp_data_0, tmp_size);
  AEBSensorFusion_sort_m(x_0, tmp_0);

  // MATLABSystem: '<Root>/Multi-Object Tracker' incorporates:
  //   SignalConversion generated from: '<Root>/Multi-Object Tracker'

  AEBSensorFusion_do_vectors(tmp_data_0, tmp_size, tmp_0, sensors_data_0,
    sensors_size, ia_data, ii_data, ib_size);
  numDetsIn = sensors_size[1];
  for (b_n = 0; b_n < numDetsIn; b_n++) {
    x[0] = (AEBSensorFusion_DW.obj.pUsedSensors[0] == 0U);
    x[1] = (AEBSensorFusion_DW.obj.pUsedSensors[1] == 0U);
    idx = 0;
    ii_size_idx_1 = 1;
    n = 1;
    exitg1 = false;
    while ((!exitg1) && (n - 1 < 2)) {
      if (x[n - 1]) {
        idx = 1;
        ii_data[0] = n;
        exitg1 = true;
      } else {
        n++;
      }
    }

    if (idx == 0) {
      ii_size_idx_1 = 0;
    }

    for (n = 0; n < ii_size_idx_1; n++) {
      AEBSensorFusion_DW.obj.pUsedSensors[ii_data[0] - 1] = sensors_data_0[b_n];
    }
  }

  if (AEBSensorFusion_DW.obj.cDetectionManager->pNumDetections > 0.0) {
    n_0 = AEBSensorFusion_DW.obj.cDetectionManager->pNumDetections;
    n = static_cast<int32_T>(n_0);
    b_dets_size[0] = static_cast<int32_T>(n_0);
    for (b_n = 0; b_n < n; b_n++) {
      AEBSensorFusion_DW.b_dets_data[b_n] =
        AEBSensorFusion_DW.obj.cDetectionManager->pDetections[b_n];
    }
  } else {
    b_dets_size[0] = 0;
  }

  AEBSensorFusion_GNNTracker_coreAlgorithm(&AEBSensorFusion_DW.obj,
    AEBSensorFusion_DW.b_dets_data, b_dets_size, *rtu_SystemTime);
  for (n = 0; n < 20; n++) {
    tracksList[n] = AEBSensorFusion_DW.obj.pTracksList[n];
  }

  for (n = 0; n < 20; n++) {
    list[n] = AEBSensorFusion_DW.obj.pConfirmedTracks[n];
  }

  idx = list[0];
  for (b_n = 0; b_n < 19; b_n++) {
    idx += list[b_n + 1];
  }

  AEBSensorFusion_TrackManager_getLiveTrackIndices(&AEBSensorFusion_DW.obj,
    liveTracks_data, tmp_size);
  AEBSensorFusion_ObjectTrack_trackToStruct(tracksList[0], &expl_temp_4);
  track.TrackID = expl_temp_4.TrackID;
  track.BranchID = expl_temp_4.BranchID;
  track.SourceIndex = 0U;
  track.UpdateTime = expl_temp_4.UpdateTime;
  track.Age = expl_temp_4.Age;
  for (n = 0; n < 6; n++) {
    track.State[n] = expl_temp_4.State[n];
  }

  std::memcpy(&track.StateCovariance[0], &expl_temp_4.StateCovariance[0], 36U *
              sizeof(real_T));
  track.ObjectClassID = expl_temp_4.ObjectClassID;
  track.TrackLogic = trackLogicType_History;
  track.TrackLogicState[0] = expl_temp_4.TrackLogicState[0];
  track.TrackLogicState[1] = expl_temp_4.TrackLogicState[1];
  track.TrackLogicState[2] = expl_temp_4.TrackLogicState[2];
  track.IsConfirmed = expl_temp_4.IsConfirmed;
  track.IsCoasted = expl_temp_4.IsCoasted;
  track.IsSelfReported = true;
  track.ObjectAttributes[0] = expl_temp_4.ObjectAttributes[0];
  track.ObjectAttributes[1] = expl_temp_4.ObjectAttributes[1];
  for (n = 0; n < idx; n++) {
    AEBSensorFusion_DW.tracks_data[n] = track;
  }

  if (idx > 0) {
    expl_temp_3.SourceIndex = 0U;
    expl_temp_3.TrackLogic = trackLogicType_History;
    expl_temp_3.IsSelfReported = true;
    for (b_n = 0; b_n < idx; b_n++) {
      numDetsIn = 0;
      for (n = 0; n < 20; n++) {
        if (list[n]) {
          tmp_data[numDetsIn] = n;
          numDetsIn++;
        }
      }

      AEBSensorFusion_ObjectTrack_trackToStruct
        (tracksList[liveTracks_data[tmp_data[b_n]] - 1], &expl_temp_4);
      expl_temp_3.TrackID = expl_temp_4.TrackID;
      expl_temp_3.BranchID = expl_temp_4.BranchID;
      expl_temp_3.UpdateTime = expl_temp_4.UpdateTime;
      expl_temp_3.Age = expl_temp_4.Age;
      for (n = 0; n < 6; n++) {
        expl_temp_3.State[n] = expl_temp_4.State[n];
      }

      std::memcpy(&expl_temp_3.StateCovariance[0], &expl_temp_4.StateCovariance
                  [0], 36U * sizeof(real_T));
      expl_temp_3.ObjectClassID = expl_temp_4.ObjectClassID;
      expl_temp_3.TrackLogicState[0] = expl_temp_4.TrackLogicState[0];
      expl_temp_3.TrackLogicState[1] = expl_temp_4.TrackLogicState[1];
      expl_temp_3.TrackLogicState[2] = expl_temp_4.TrackLogicState[2];
      expl_temp_3.IsConfirmed = expl_temp_4.IsConfirmed;
      expl_temp_3.IsCoasted = expl_temp_4.IsCoasted;
      expl_temp_3.ObjectAttributes[0] = expl_temp_4.ObjectAttributes[0];
      expl_temp_3.ObjectAttributes[1] = expl_temp_4.ObjectAttributes[1];
      AEBSensorFusion_DW.tracks_data[b_n] = expl_temp_3;
    }
  }

  for (n = 0; n < 20; n++) {
    AEBSensorFusion_DW.TmpMLSysMemLayoutBufferAtMultiObjectTrackerOutport1.Tracks
      [n] = track;
  }

  for (n = 0; n < idx; n++) {
    AEBSensorFusion_DW.TmpMLSysMemLayoutBufferAtMultiObjectTrackerOutport1.Tracks
      [n] = AEBSensorFusion_DW.tracks_data[n];
  }

  AEBSensorFusion_DW.TmpMLSysMemLayoutBufferAtMultiObjectTrackerOutport1.NumTracks
    = idx;

  // SignalConversion generated from: '<Root>/Tracks' incorporates:
  //   SignalConversion generated from: '<Root>/Multi-Object Tracker'

  *rty_Tracks =
    AEBSensorFusion_DW.TmpMLSysMemLayoutBufferAtMultiObjectTrackerOutport1;
}

// Termination for referenced model: 'AEBSensorFusion'
void ACCWithSensorFusionModelClass::terminate(void)
{
  // Terminate for MATLABSystem: '<Root>/Multi-Object Tracker'
  if (!AEBSensorFusion_DW.obj.matlabCodegenIsDeleted) {
    AEBSensorFusion_DW.obj.matlabCodegenIsDeleted = true;
    if ((AEBSensorFusion_DW.obj.isInitialized == 1) &&
        AEBSensorFusion_DW.obj.isSetupComplete) {
      AEBSensorFusion_DW.obj.pNumLiveTracks = 0;
      std::memset(&AEBSensorFusion_DW.obj.pTrackIDs[0], 0, 20U * sizeof(uint32_T));
      for (int32_T i = 0; i < 20; i++) {
        AEBSensorFusion_DW.obj.pConfirmedTracks[i] = false;
      }

      AEBSensorFusion_DW.obj.pLastTrackID = 0U;
      AEBSensorFusion_DW.obj.pLastTimeStamp = -2.2204460492503131E-16;
      if (AEBSensorFusion_DW.obj.cAssigner->isInitialized == 1) {
        AEBSensorFusion_DW.obj.cAssigner->isInitialized = 2;
      }
    }
  }

  // End of Terminate for MATLABSystem: '<Root>/Multi-Object Tracker'
}

// Model initialize function
void ACCWithSensorFusionModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // Initialize DataMapInfo substructure containing ModelMap for C API
  AEBSensorFusion_InitializeDataMapInfo((&AEBSensorFusion_M),
    &AEBSensorFusion_DW);
}

// Constructor
ACCWithSensorFusionModelClass::ACCWithSensorFusionModelClass() :
  AEBSensorFusion_DW(),
  AEBSensorFusion_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
ACCWithSensorFusionModelClass::~ACCWithSensorFusionModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_AEBSensorFusion_T * ACCWithSensorFusionModelClass::getRTM()
{
  return (&AEBSensorFusion_M);
}

// member function to set up the C-API information
void ACCWithSensorFusionModelClass::setupCAPIInfo(rtwCAPI_ModelMappingInfo
  *rt_ParentMMI, const char_T *rt_ChildPath, int_T rt_ChildMMIIdx, int_T
  rt_CSTATEIdx)
{
  // Initialize Parent model MMI
  if ((rt_ParentMMI != (NULL)) && (rt_ChildPath != (NULL))) {
    rtwCAPI_SetChildMMI(*rt_ParentMMI, rt_ChildMMIIdx, &((&AEBSensorFusion_M)
      ->DataMapInfo.mmi));
    rtwCAPI_SetPath((&AEBSensorFusion_M)->DataMapInfo.mmi, rt_ChildPath);
    rtwCAPI_MMISetContStateStartIndex((&AEBSensorFusion_M)->DataMapInfo.mmi,
      rt_CSTATEIdx);
  }
}

RT_MODEL_AEBSensorFusion_T::DataMapInfo_T RT_MODEL_AEBSensorFusion_T::
  getDataMapInfo() const
{
  return DataMapInfo;
}

void RT_MODEL_AEBSensorFusion_T::setDataMapInfo(RT_MODEL_AEBSensorFusion_T::
  DataMapInfo_T aDataMapInfo)
{
  DataMapInfo = aDataMapInfo;
}

//
// File trailer for generated code.
//
// [EOF]
//
