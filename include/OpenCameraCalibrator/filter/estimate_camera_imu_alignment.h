// created by Steffen Urban 2019, January

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <vector>

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Matrix31d = Eigen::Matrix<double, 3, 1>;
using Matrix13d = Eigen::Matrix<double, 1, 3>;
using Eigen::MatrixXd;
using Matrix63d = Eigen::Matrix<double, 6, 3>;
using Eigen::Quaterniond;

// alignment stuff
using QuatVector = std::vector<Quaterniond, Eigen::aligned_allocator<Quaterniond>>;
using Vec3Vector = std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>>;
using QuatMap = std::map<double, Quaterniond, std::less<double>, Eigen::aligned_allocator<Quaterniond>>;
using Vec3Map = std::map<double, Vector3d, std::less<double>, Eigen::aligned_allocator<Vector3d>>;

namespace OpenCamCalib {
namespace filter {

void EstimateCameraImuAlignment(const QuatMap& visual_rotations,
        const Vec3Map& imu_angular_vel,
        const double dt_vis,
        const double dt_imu,
        Eigen::Matrix3d &R_imu_to_camera,
        double& time_offset_imu_to_camera,
        Vector3d& gyro_bias);
}
}
