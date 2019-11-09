// created by Steffen Urban 2019, January

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <vector>

namespace filter {

void EstimateCameraImuAlignment(
    const std::map<double, Eigen::Quaterniond>& visual_rotations,
    const std::map<double, Eigen::Quaterniond>& imu_rotatons,
    Eigen::Quaterniond* R_imu_to_camera, double time_offset_imu_to_camera,
    Eigen::Vector3d* imu_bias);
}
