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

#include "OpenCameraCalibrator/core/spline_trajectory_estimator.h"

namespace OpenICC {
namespace core {

const int SPLINE_N = 6;

class ImuCameraCalibrator {
 public:
  ImuCameraCalibrator() {}
  void BatchInitSpline(
      const theia::Reconstruction& vision_dataset,
      const Sophus::SE3<double>& T_i_c_init,
      const OpenICC::SplineWeightingData& spline_weight_data,
      const double time_offset_imu_to_cam,
      const OpenICC::CameraTelemetryData& telemetry_data,
      const double initial_line_delay,
      const ThreeAxisSensorCalibParams<double> accl_intrinsics,
      const ThreeAxisSensorCalibParams<double> gyro_intrinsics);

  double Optimize(const int iterations, const int optim_flags);

  void ToTheiaReconDataset(theia::Reconstruction& output_recon);

  void ClearSpline();

  SplineTrajectoryEstimator<SPLINE_N> trajectory_;

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

  //! Use this function if we really know the gravity direction of
  //! the calibration board (e.g. flat on the ground -> [0,0,9.81])
  void SetKnownGravityDir(const Eigen::Vector3d& gravity);

  void SetCalibrateRSLineDelay() { calibrate_cam_line_delay_ = true; }
  bool GetCalibrateRSLineDelay() { return calibrate_cam_line_delay_; }
  void SetRSLineDelay(const double line_delay) {
    inital_cam_line_delay_s_ = line_delay;
  }
  double GetCalibratedRSLineDelay() { return trajectory_.GetRSLineDelay(); }
  double GetInitialRSLineDelay() { return inital_cam_line_delay_s_; }

  void GetIMUIntrinsics(ThreeAxisSensorCalibParams<double>& acc_intrinsics,
                        ThreeAxisSensorCalibParams<double>& gyr_intrinsics,
                        const int64_t time_ns = 0);

 private:
  void InitializeGravity(const OpenICC::CameraTelemetryData& telemetry_data);

  //! camera timestamps
  std::vector<double> cam_timestamps_;

  //! accl measurements
  aligned_map<double, Eigen::Vector3d> gyro_measurements_;

  //! gyro measurements
  aligned_map<double, Eigen::Vector3d> accl_measurements_;

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

  theia::Reconstruction image_data_;
};

}  // namespace core
}  // namespace OpenICC
