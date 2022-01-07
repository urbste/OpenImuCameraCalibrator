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
namespace io {
using json = nlohmann::json;

bool ReadGoProTelemetry(const std::string& path_to_telemetry_file,
                        CameraTelemetryData& telemetry) {
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
  for (const auto& e : accl) {
    Eigen::Vector3d v;
    v << e["value"][1], e["value"][2], e["value"][0];
    ImuReading<double> reading((double)e["cts"] * MS_TO_S, v);
    telemetry.accelerometer.push_back(reading);
  }
  for (const auto& e : gyro) {
    Eigen::Vector3d v;
    v << e["value"][1], e["value"][2], e["value"][0];
    ImuReading<double> reading((double)e["cts"] * MS_TO_S, v);
    telemetry.accelerometer.push_back(reading);
  }
  for (const auto& e : gps5) {
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

}  // namespace io
}  // namespace OpenICC
