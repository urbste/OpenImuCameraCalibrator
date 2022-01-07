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

#include "OpenCameraCalibrator/utils/types.h"

namespace OpenICC {
namespace core {

class ImuToCameraRotationEstimator {
 public:
  ImuToCameraRotationEstimator() {}
  ImuToCameraRotationEstimator(const quat_map& visual_rotations,
                               const vec3_map& imu_angular_vel)
      : visual_rotations_(visual_rotations),
        imu_angular_vel_(imu_angular_vel) {}

  void SetVisualRotations(const quat_map& visual_rotations) {
    visual_rotations_ = visual_rotations;
  }

  void SetAngularVelocities(const vec3_map& imu_angular_vel) {
    imu_angular_vel_ = imu_angular_vel;
  }

  bool EstimateCameraImuRotation(const double dt_vis,
                                 const double dt_imu,
                                 Eigen::Matrix3d& R_imu_to_camera,
                                 double& time_offset_imu_to_camera,
                                 Eigen::Vector3d& gyro_bias,
                                 vec3_vector& smoothed_ang_imu,
                                 vec3_vector& smoothed_vis_vel);

  double SolveClosedForm(const vec3_vector& angVis,
                         const vec3_vector& angImu,
                         const std::vector<double> timestamps_s,
                         const double td,
                         const double dt_imu,
                         Eigen::Matrix3d& Rs,
                         Eigen::Vector3d& bias);

  void EnableGyroBiasEstimation() { estimate_gyro_bias_ = true; }

 private:
  //! visual rotations
  quat_map visual_rotations_;

  //! imu angular velocities
  vec3_map imu_angular_vel_;

  //! estimate bias
  bool estimate_gyro_bias_ = false;
};

}  // namespace core
}  // namespace OpenICC
