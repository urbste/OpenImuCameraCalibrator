/* Copyright (C) 2021 Steffen Urban
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <unordered_map>

#include "OpenCameraCalibrator/utils/types.h"

#include "OpenCameraCalibrator/basalt_spline/calib_helpers.h"
#include "OpenCameraCalibrator/basalt_spline/ceres_calib_spline_split.h"

namespace OpenICC {
namespace core {

const int SPLINE_N = 5;
const bool USE_OLD_TIME_DERIV = false;

class ImuCameraCalibrator {
public:
  ImuCameraCalibrator(const bool reestimate_biases) {
    reestimate_biases_ = reestimate_biases;
  }
  void InitSpline(const theia::Reconstruction &calib_dataset,
                  const Sophus::SE3<double> &T_i_c_init,
                  const OpenICC::SplineWeightingData &spline_weight_data,
                  const double time_offset_imu_to_cam,
                  const Eigen::Vector3d &gyro_bias,
                  const Eigen::Vector3d &accl_bias,
                  const OpenICC::CameraTelemetryData &telemetry_data,
                  const double initial_line_delay);

  void InitializeGravity(const OpenICC::CameraTelemetryData &telemetry_data,
                         const Eigen::Vector3d &accl_bias);

  double Optimize(const int iterations,
                  const bool fix_so3_spline,
                  const bool fix_r3_spline,
                  const bool fix_T_i_c,
                  const bool fix_line_delay);

  void ToTheiaReconDataset(theia::Reconstruction &output_recon);

  void ClearSpline();

  CeresCalibrationSplineSplit<SPLINE_N, USE_OLD_TIME_DERIV> trajectory_;

  //! camera timestamps in seconds
  std::vector<double> GetCamTimestamps() { return cam_timestamps_; }

  //! get gyroscope measurements
  aligned_map<double, Eigen::Vector3d> GetGyroMeasurements() {
    return gyro_measurements_;
  }

  //! get accelerometer measurements
  aligned_map<double, Eigen::Vector3d> GetAcclMeasurements() {
    return accl_measurements_;
  }

  void SetCalibrateRSLineDelay() { calibrate_cam_line_delay_ = true; }
  bool GetCalibrateRSLineDelay() { return calibrate_cam_line_delay_; }
  void SetRSLineDelay(const double line_delay) {
    inital_cam_line_delay_s_ = line_delay;
  }
  double GetCalibratedRSLineDelay() {
    return trajectory_.GetOptimizedRSLineDelay();
  }
  double GetInitialRSLineDelay() { return inital_cam_line_delay_s_; }

private:
  //! camera timestamps
  std::vector<double> cam_timestamps_;

  //! accl measurements
  aligned_map<double, Eigen::Vector3d> gyro_measurements_;

  //! gyro measurements
  aligned_map<double, Eigen::Vector3d> accl_measurements_;

  //! imu update rate in hz
  double imu_update_rate_hz_;

  //! spline know spacing in R3 and SO3 in seconds
  SplineWeightingData spline_weight_data_;

  //! spline start and end time in seconds
  double t0_s_;
  double tend_s_;

  //! number of knots in SO3 and R3
  uint64_t nr_knots_so3_;
  uint64_t nr_knots_r3_;

  //! camera line delay, init global shutter = 0.0
  double inital_cam_line_delay_s_ = 0.0;
  bool calibrate_cam_line_delay_ = false;

  //! is gravity direction in sensor frame is initialized
  bool gravity_initialized_ = false;

  //! gravity in sensor frame
  Eigen::Vector3d gravity_init_;

  Sophus::SE3<double> T_i_c_init_;

  //! is gravity direction in sensor frame is initialized
  bool reestimate_biases_ = false;

  //! -
  std::unordered_map<TimeCamId, CalibCornerData> calib_corners_;
  std::unordered_map<TimeCamId, CalibInitPoseData> calib_init_poses_;
  std::unordered_map<TimeCamId, CalibInitPoseData> spline_init_poses_;
};

} // namespace core
} // namespace OpenICC
