// created by Steffen Urban 2019, January

#include "OpenCameraCalibrator/filter/kalmanRTS.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

// C++ port from https://github.com/jannemus/InertialScale

namespace filter {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Matrix31d = Eigen::Matrix<double, 3, 1>;
using Matrix13d = Eigen::Matrix<double, 1, 3>;
using Eigen::MatrixXd;
using Matrix63d = Eigen::Matrix<double, 6, 3>;
using Eigen::Quaterniond;

void UpsampleAndInterpolateRotations(
    const std::map<double, Eigen::Quaterniond> time_to_rotations_fast_sensor,
    const std::map<double, Eigen::Quaterniond> time_to_rotations_slow_sensor,
    std::map<double, Eigen::Quaterniond>* interpolated_time_to_rotations) {
  for (auto const& t_to_rot_fast : time_to_rotations_fast_sensor) {
    std::vector<double> time_differences;
    //        const double interp_time_0;
    //        const double interp_time_1;
    //        // first get time differences
    //        for (auto const& t_to_rot_slow : time_to_rotations_slow_sensor) {
    //            const double abs_time_diff = std::abs(t_to_rot_fast.first -
    //            t_to_rot_slow.first);
    //            time_differences.push_back(abs_time_diff);
    //        }

    //        auto min_element = std::min_element(time_differences.begin(),
    //        time_differences.end());
  }
}

void EstimateCameraImuAlignment(
    const std::map<double, Eigen::Quaterniond>& visual_rotations,
    const std::map<double, Eigen::Quaterniond>& imu_rotatons,
    Eigen::Quaterniond* R_imu_to_camera, double time_offset_imu_to_camera,
    Eigen::Vector3d* imu_bias) {
  // find start and end points of camera and imu
  const double start_time_cam = visual_rotations.begin()->first;
  const double end_time_cam = visual_rotations.end()->first;
  const double start_time_imu = imu_rotatons.begin()->first;
  const double end_time_imu = imu_rotatons.end()->first;

  double t0 =
      (start_time_cam >= start_time_imu) ? start_time_cam : start_time_imu;
  double tend = (end_time_cam >= end_time_imu) ? end_time_cam : end_time_imu;

  // create zero-based maps
  std::map<double, Eigen::Quaterniond> visual_rotations_clamped_interpolated;
  std::map<double, Eigen::Quaterniond> imu_rotatons_clamped;

  for (auto const& imu_rot : imu_rotatons) {
    if (imu_rot.first >= t0 && imu_rot.first <= tend) {
      imu_rotatons_clamped[imu_rot.first - t0] = imu_rot.second;
    }
  }

  // interpolate visual rotations
  for (auto const& v_rot : visual_rotations) {
    if (v_rot.first >= t0 && v_rot.first <= tend) {
      visual_rotations_clamped_interpolated[v_rot.first - t0] = v_rot.second;
    }
  }
}
}
