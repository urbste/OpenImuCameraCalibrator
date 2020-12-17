// created by Steffen Urban November 2019
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

namespace OpenCamCalib {

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                             Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                       Eigen::aligned_allocator<std::pair<K const, V>>>;

// alignment stuff
using QuatVector = aligned_vector<Eigen::Quaterniond>;
using Vec3Vector = aligned_vector<Eigen::Vector3d>;
using QuatMap = aligned_map<double, Eigen::Quaterniond>;
using Vec3Map = aligned_map<double, Eigen::Vector3d>;

struct GPXData {
  std::vector<Eigen::Vector3d> lle;
  std::vector<double> timestamp_utc_unixtime;
  std::vector<double> timestamp_ms;
  std::vector<double> precision;
  std::vector<double> geoid_height;
  std::vector<Eigen::Vector2d> vel2d_vel3d;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraGyroData {
  aligned_vector<Eigen::Vector3d> gyro_measurement;
  std::vector<double> timestamp_ms;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraAccData {
  aligned_vector<Eigen::Vector3d> acc_measurement;
  std::vector<double> timestamp_ms;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SplineWeightingData {
  // create this with get_sew_for_dataset.py
  double dt_r3;
  double dt_so3;
  double var_r3;
  double var_so3;
  double cam_fps;
};

struct CameraTelemetryData {
  // IMU
  CameraAccData accelerometer;
  CameraGyroData gyroscope;
  // GPS
  GPXData gps;
};

struct IMUCalibData {
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d acc_bias;
  double rate;
  Eigen::Matrix3d R_imu_to_camera;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace OpenCamCalib
