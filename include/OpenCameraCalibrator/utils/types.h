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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include "sophus/so3.hpp"

namespace OpenICC {

const double NS_TO_S = 1e-9; ///< Nanosecond to second conversion
const double S_TO_NS = 1e9;  ///< Second to nanosecond conversion

const double US_TO_S = 1e-6; ///< Museconds to second conversion
const double S_TO_US = 1e6;  ///< Second to museconds conversion

const double MS_TO_S = 1e-3; ///< Milliseconds to second conversion
const double S_TO_MS = 1e3;  ///< Second to milliseconds conversion

// alignment stuff
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

using quat_vector = aligned_vector<Eigen::Quaterniond>;
using vec4_vector = aligned_vector<Eigen::Vector4d>;
using vec3_vector = aligned_vector<Eigen::Vector3d>;
using vec2_vector = aligned_vector<Eigen::Vector2d>;
using quat_map = aligned_map<double, Eigen::Quaterniond>;
using vec3_map = aligned_map<double, Eigen::Vector3d>;
using so3_vector = aligned_vector<Sophus::SO3d>;

struct GPXData {
  vec3_vector lle;
  std::vector<double> timestamp_utc_unixtime;
  std::vector<double> timestamp_ms;
  std::vector<double> precision;
  std::vector<double> geoid_height;
  vec2_vector vel2d_vel3d;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraGyroData {
  vec3_vector measurement;
  std::vector<double> timestamp_ms;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraAccData {
  vec3_vector measurement;
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

class Data {
public:
  Data() : v(0.0), t(0.0) {}
  Data(double data, double time) : v(data), t(time) {}

  double v;
  double t;
};

class AccData {
public:
  AccData() : a(0.0), t(0.0) {}
  AccData(double data, double time) : a(data), t(time) {}

  double a;
  double t;
};

class GyrData {
public:
  GyrData() : w(0.0), t(0.0) {}
  GyrData(double data, double time) : w(data), t(time) {}

  double w;
  double t;
};

} // namespace OpenICC
