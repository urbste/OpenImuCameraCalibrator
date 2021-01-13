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

#include "OpenCameraCalibrator/io/read_gopro_imu_json.h"

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include <fstream>
#include <iostream>
#include <istream>

namespace OpenICC {
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
  spline_weighting.cam_fps = j["fps"];
  spline_weighting.dt_r3 = j["r3"]["knot_spacing"];
  spline_weighting.dt_so3 = j["so3"]["knot_spacing"];
  spline_weighting.var_r3 = j["r3"]["weighting_factor"];
  spline_weighting.var_so3 = j["so3"]["weighting_factor"];

  return true;
}

bool ReadGoProTelemetry(const std::string &path_to_telemetry_file,
                        CameraTelemetryData &telemetry) {
  std::ifstream file;
  file.open(path_to_telemetry_file.c_str());
  if (!file.is_open()) {
    return false;
  }
  json j;
  file >> j;
  const auto accl = j["1"]["streams"]["ACCL"]["samples"];
  const auto gyro = j["1"]["streams"]["GYRO"]["samples"];
  const auto gps5 = j["1"]["streams"]["GPS5"]["samples"];
  for (const auto &e : accl) {
    Eigen::Vector3d v;
    v << e["value"][1], e["value"][2], e["value"][0];
    telemetry.accelerometer.acc_measurement.emplace_back(v);
    telemetry.accelerometer.timestamp_ms.emplace_back(e["cts"]);
  }
  for (const auto &e : gyro) {
    Eigen::Vector3d v;
    v << e["value"][1], e["value"][2], e["value"][0];
    telemetry.gyroscope.gyro_measurement.emplace_back(v);
    telemetry.gyroscope.timestamp_ms.emplace_back(e["cts"]);
  }
  for (const auto &e : gps5) {
    Eigen::Vector3d v;
    Eigen::Vector2d vel2d_vel3d;
    v << e["value"][0], e["value"][1], e["value"][2];
    vel2d_vel3d << e["value"][3], e["value"][4];
    telemetry.gps.lle.emplace_back(v);
    telemetry.gps.timestamp_ms.emplace_back(e["cts"]);
    telemetry.gps.precision.emplace_back(e["precision"]);
    telemetry.gps.vel2d_vel3d.emplace_back(vel2d_vel3d);
  }

  file.close();
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
                     double& time_offset_imu_to_cam) {
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

} // namespace OpenICC
