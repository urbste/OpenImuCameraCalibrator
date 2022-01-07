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

#include "OpenCameraCalibrator/io/read_misc.h"

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <istream>

namespace OpenICC {
namespace io {
using json = nlohmann::json;

bool ReadSplineErrorWeighting(
    const std::string &path_to_spline_error_weighting_json,
    SplineWeightingData &spline_weighting) {
  std::ifstream file;
  file.open(path_to_spline_error_weighting_json.c_str());
  if (!file.is_open()) {
    return false;
  }
  json j;
  file >> j;
  spline_weighting.cam_fps = j["camera_fps"];
  spline_weighting.dt_r3 = j["r3"]["knot_spacing"];
  spline_weighting.dt_so3 = j["so3"]["knot_spacing"];
  spline_weighting.var_r3 = j["r3"]["weighting_factor"];
  spline_weighting.var_so3 = j["so3"]["weighting_factor"];

  return true;
}

bool ReadIMUBias(const std::string &path_to_imu_bias,
                 Eigen::Vector3d &gyro_bias, Eigen::Vector3d &accl_bias) {
  std::ifstream file;
  file.open(path_to_imu_bias.c_str());
  if (!file.is_open()) {
    return false;
  }
  json j;
  file >> j;
  accl_bias << j["accl_bias"]["x"], j["accl_bias"]["y"], j["accl_bias"]["z"];
  gyro_bias << j["gyro_bias"]["x"], j["gyro_bias"]["y"], j["gyro_bias"]["z"];

  return true;
}

bool ReadIMU2CamInit(const std::string &path_to_file,
                     Eigen::Quaterniond &imu_to_cam_rotation,
                     double &time_offset_imu_to_cam) {
  std::ifstream file;
  file.open(path_to_file.c_str());
  if (!file.is_open()) {
    return false;
  }
  json j;
  file >> j;
  imu_to_cam_rotation = Eigen::Quaterniond(
      j["gyro_to_camera_rotation"]["w"], j["gyro_to_camera_rotation"]["x"],
      j["gyro_to_camera_rotation"]["y"], j["gyro_to_camera_rotation"]["z"]);
  time_offset_imu_to_cam = j["time_offset_gyro_to_cam"];

  return true;
}

bool ReadIMUIntrinsics(const std::string &path_to_imu_intrinsics,
                       const std::string &path_to_initial_imu_bias,
                       ThreeAxisSensorCalibParamsd &acc_params,
                       ThreeAxisSensorCalibParamsd &gyro_params) {
  if (path_to_initial_imu_bias != "") {
    LOG(INFO) << "Initial IMU biases supplied.";
    Eigen::Vector3d gyr_bias, acc_bias;
    if (ReadIMUBias(path_to_initial_imu_bias, gyr_bias, acc_bias)) {
      acc_params.SetBias(acc_bias);
      gyro_params.SetBias(gyr_bias);
    }
  } else {
    LOG(INFO) << "No initial IMU biases supplied. Setting bias to zero.";
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    acc_params.SetBias(zero);
    gyro_params.SetBias(zero);
  }

  if (path_to_imu_intrinsics != "") {
    LOG(INFO) << "Loading IMU intrinsics.";
    std::ifstream file;
    file.open(path_to_imu_intrinsics.c_str());
    if (!file.is_open()) {
      return false;
    }
    json j;
    file >> j;

    auto m_acc = j["accelerometer"]["misalignment_matrix"];
    auto s_acc = j["accelerometer"]["scale_matrix"];
    Eigen::Matrix3d acc_misalignment = Eigen::Matrix3d::Identity();
    acc_misalignment(0, 1) = m_acc[0][1];
    acc_misalignment(0, 2) = m_acc[0][2];
    acc_misalignment(1, 2) = m_acc[1][2];
    const Eigen::Vector3d acc_scale(s_acc[0][0], s_acc[1][1], s_acc[2][2]);

    auto m_gyr = j["gyroscope"]["misalignment_matrix"];
    auto g_acc = j["gyroscope"]["scale_matrix"];
    Eigen::Matrix3d gyr_misalignment = Eigen::Matrix3d::Identity();
    gyr_misalignment(0, 1) = m_gyr[0][1];
    gyr_misalignment(0, 2) = m_gyr[0][2];
    gyr_misalignment(1, 2) = m_gyr[1][2];
    gyr_misalignment(1, 0) = m_gyr[1][0];
    gyr_misalignment(2, 0) = m_gyr[2][0];
    gyr_misalignment(2, 1) = m_gyr[2][1];
    const Eigen::Vector3d gyr_scale(g_acc[0][0], g_acc[1][1], g_acc[2][2]);

    acc_params.SetScale(acc_scale);
    acc_params.SetMisalignmentMatrix(acc_misalignment);

    gyro_params.SetScale(gyr_scale);
    gyro_params.SetMisalignmentMatrix(gyr_misalignment);
  } else {
    LOG(INFO) << "No IMU intrinsic supplied. Setting misalignment and scale to "
                 "identity.";
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    acc_params.SetScale(I3.diagonal());
    acc_params.SetMisalignmentMatrix(I3);

    gyro_params.SetScale(I3.diagonal());
    gyro_params.SetMisalignmentMatrix(I3);
  }
  return true;
}

} // namespace io
} // namespace OpenICC
