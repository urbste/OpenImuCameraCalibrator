// created by Steffen Urban 2019, January

#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace filter {

using Eigen::Quaterniond;

bool EstimateCameraImuAlignment(
    const std::vector<Eigen::Quaterniond>& world_2_camera_rotations,
    const std::vector<double>& visual_timestamps,
    const std::vector<Eigen::Vector3d> imu_angular_velocities,
    const std::vector<Eigen::Vector3d>& imu_timestamps,
    Eigen::Quaterniond* imu_to_camera_rotation, double* time_offset,
    Eigen::Vector3d* gyroscope_bias);
}
