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

template <typename T>
class ImuReading {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuReading() {}

  ImuReading(T timestamp_s, T x, T y, T z)
      : timestamp_s_(timestamp_s), xyz_(x, y, z) {}

  ImuReading(T timestamp_s, const  Eigen::Matrix<T, 3, 1> &data)
      : timestamp_s_(timestamp_s), xyz_(data) {}

  ImuReading(T timestamp_s, const T *data)
      : timestamp_s_(timestamp_s), xyz_(data[0], data[1], data[2]) {}

  inline const T &timestamp_s() const { return timestamp_s_; }
  inline const Eigen::Matrix<T, 3, 1> &data() const { return xyz_; }
  inline const T &operator()(int index) const { return xyz_[index]; }
  inline const T &x() const { return xyz_[0]; }
  inline const T &y() const { return xyz_[1]; }
  inline const T &z() const { return xyz_[2]; }
  inline const T *data_ptr() const { return xyz_.data(); }
private:
  //! imu reading
  Eigen::Matrix<T, 3, 1> xyz_;
  //! timestamp in seconds
  T timestamp_s_;
};

using CameraGyroData = std::vector<ImuReading<double>>;
using CameraAccData = std::vector<ImuReading<double>>;

struct SplineWeightingData {
  // create this with get_sew_for_dataset.py
  double dt_r3;
  double dt_so3;
  double var_r3;
  double var_so3;
  double cam_fps;
};

using ImuReadings = std::vector<ImuReading<double>>;


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

template <typename _T> class ThreeAxisSensorCalibParams {
  /** @brief This object contains the calibration parameters (misalignment,
   * scale factors, ...) of a generic orthogonal sensor triad (accelerometers,
   * gyroscopes, etc.)
   *
   * Triad model:
   *
   * -Misalignment matrix:
   *
   * general case:
   *
   *     [    1     -mis_yz   mis_zy  ]
   * T = [  mis_xz     1     -mis_zx  ]
   *     [ -mis_xy   mis_yx     1     ]
   *
   * "body" frame spacial case:
   *
   *     [  1     -mis_yz   mis_zy  ]
   * T = [  0        1     -mis_zx  ]
   *     [  0        0        1     ]
   *
   * Scale matrix:
   *
   *     [  s_x      0        0  ]
   * K = [   0      s_y       0  ]
   *     [   0       0       s_z ]
   *
   * Bias vector:
   *
   *     [ b_x ]
   * B = [ b_y ]
   *     [ b_z ]
   *
   * Given a raw sensor reading X (e.g., the acceleration ), the calibrated
   * "unbiased" reading X' is obtained
   *
   * X' = T*K*(X - B)
   *
   * with B the bias (variable) + offset (constant, possibbly 0), or,
   * equivalently:
   *
   * X' = T*K*X - B'
   *
   * with B' = T*K*B
   *
   * Without knowing the value of the bias (and with offset == 0), the
   * calibrated reading X'' is simply:
   *
   * X'' = T*K*X
   */
public:
  /** @brief Basic "default" constructor: without any parameter, it initilizes
   * the calibration parameter with default values (zero scaling factors and
   * biases, identity misalignment matrix)
   */
  ThreeAxisSensorCalibParams(const _T &mis_yz = _T(0), const _T &mis_zy = _T(0),
                             const _T &mis_zx = _T(0), const _T &mis_xz = _T(0),
                             const _T &mis_xy = _T(0), const _T &mis_yx = _T(0),
                             const _T &s_x = _T(1), const _T &s_y = _T(1),
                             const _T &s_z = _T(1), const _T &b_x = _T(0),
                             const _T &b_y = _T(0), const _T &b_z = _T(0)) {
    mis_mat_ << _T(1), -mis_yz, mis_zy, mis_xz, _T(1), -mis_zx, -mis_xy, mis_yx,
        _T(1);

    scale_mat_ << s_x, _T(0), _T(0), _T(0), s_y, _T(0), _T(0), _T(0), s_z;

    bias_vec_ << b_x, b_y, b_z;

    Update();
  }

  ~ThreeAxisSensorCalibParams() {}

  inline _T misYZ() const { return -mis_mat_(0, 1); }
  inline _T misZY() const { return mis_mat_(0, 2); }
  inline _T misZX() const { return -mis_mat_(1, 2); }
  inline _T misXZ() const { return mis_mat_(1, 0); }
  inline _T misXY() const { return -mis_mat_(2, 0); }
  inline _T misYX() const { return mis_mat_(2, 1); }

  inline _T scaleX() const { return scale_mat_(0, 0); }
  inline _T scaleY() const { return scale_mat_(1, 1); }
  inline _T scaleZ() const { return scale_mat_(2, 2); }

  inline _T biasX() const { return bias_vec_(0); }
  inline _T biasY() const { return bias_vec_(1); }
  inline _T biasZ() const { return bias_vec_(2); }

  inline const Eigen::Matrix<_T, 3, 3> &GetMisalignmentMatrix() const {
    return mis_mat_;
  }
  inline const Eigen::Matrix<_T, 3, 3> &GetScaleMatrix() const {
    return scale_mat_;
  }
  inline const Eigen::Matrix<_T, 3, 1> &GetBiasVector() const {
    return bias_vec_;
  }

  inline void SetScale(const Eigen::Matrix<_T, 3, 1> &s_vec) {
    scale_mat_(0, 0) = s_vec(0);
    scale_mat_(1, 1) = s_vec(1);
    scale_mat_(2, 2) = s_vec(2);
    Update();
  }

  inline void SetBias(const Eigen::Matrix<_T, 3, 1> &b_vec) {
    bias_vec_ = b_vec;
    Update();
  }

  /** @brief Normalize a raw data X by correcting the misalignment and the
   * scale, i.e., by applying the equation  X'' = T*K*X
   */
  inline Eigen::Matrix<_T, 3, 1>
  Normalize(const Eigen::Matrix<_T, 3, 1> &raw_data) const {
    return ms_mat_ * raw_data;
  }

  /** @brief Normalize a raw data X by removing the biases and
   *         correcting the misalignment and the scale,
   *         i.e., by applying the equation  X' = T*K*(X - B)
   */
  inline Eigen::Matrix<_T, 3, 1>
  UnbiasNormalize(const Eigen::Matrix<_T, 3, 1> &raw_data) const {
    return ms_mat_ * (raw_data - bias_vec_);
  }

  /** @brief Remove the biases from a raw data */
  inline Eigen::Matrix<_T, 3, 1>
  Unbias(const Eigen::Matrix<_T, 3, 1> &raw_data) const {
    return raw_data - bias_vec_;
  }

private:
  /** @brief Update internal data (e.g., compute Misalignment * scale matrix)
   *         after a parameter is changed */
  void Update() { ms_mat_ = mis_mat_ * scale_mat_; }

  /** @brief Misalignment matrix */
  Eigen::Matrix<_T, 3, 3> mis_mat_;
  /** @brief Scale matrix */
  Eigen::Matrix<_T, 3, 3> scale_mat_;
  /** @brief Bias vector */
  Eigen::Matrix<_T, 3, 1> bias_vec_;
  /** @brief Misalignment * scale matrix */
  Eigen::Matrix<_T, 3, 3> ms_mat_;
};
} // namespace OpenICC
