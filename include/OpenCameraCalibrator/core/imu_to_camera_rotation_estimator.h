#pragma once

#include "OpenCameraCalibrator/utils/types.h"

namespace OpenCamCalib {
namespace core {

class ImuToCameraRotationEstimator {
public:
  ImuToCameraRotationEstimator() {}
  ImuToCameraRotationEstimator(const QuatMap &visual_rotations,
                               const Vec3Map &imu_angular_vel)
      : visual_rotations_(visual_rotations), imu_angular_vel_(imu_angular_vel) {
  }

  void SetVisualRotations(const QuatMap &visual_rotations) { visual_rotations_ = visual_rotations; }

  void SetAngularVelocities(const Vec3Map &imu_angular_vel) { imu_angular_vel_ = imu_angular_vel; }

  bool EstimateCameraImuRotation(const double dt_vis,
                                 const double dt_imu,
                                 Eigen::Matrix3d &R_imu_to_camera,
                                 double &time_offset_imu_to_camera,
                                 Eigen::Vector3d &gyro_bias,
                                 Vec3Vector &smoothed_ang_imu,
                                 Vec3Vector &smoothed_vis_vel);

  double SolveClosedForm(const Vec3Vector &angVis, const Vec3Vector &angImu,
                         const std::vector<double> timestamps_s,
                         const double td, const double dt_imu,
                         Eigen::Matrix3d &Rs, Eigen::Vector3d &bias);

  void EnableGyroBiasEstimation() { estimate_gyro_bias_ = true; }

private:
  //! visual rotations
  QuatMap visual_rotations_;
  //! imu angular velocities
  Vec3Map imu_angular_vel_;
  //! estimate bias
  bool estimate_gyro_bias_ = false;
};

} // namespace core
} // namespace OpenCamCalib
