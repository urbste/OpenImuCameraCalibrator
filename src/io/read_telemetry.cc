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

#include "OpenCameraCalibrator/io/read_telemetry.h"

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include <fstream>
#include <iostream>
#include <istream>

namespace OpenICC {
namespace io {
using json = nlohmann::json;

bool ReadTelemetryJSON(const std::string &path_to_telemetry_file,
                       CameraTelemetryData &telemetry) {
  std::ifstream file;
  file.open(path_to_telemetry_file.c_str());
  if (!file.is_open()) {
    return false;
  }
  json j;
  file >> j;
  const auto accl = j["accelerometer"];
  const auto gyro = j["gyroscope"];
  const auto timestamps_ns = j["timestamps_ns"];

  const int nr_datapoints = timestamps_ns.size();
  if (gyro.size() != nr_datapoints || accl.size() != nr_datapoints) {
    std::cerr << "Telemetry should have the same amount of timestamps, "
                 "accelerometer and gyroscope values.\n";
    return false;
  }

  for (const auto& t : timestamps_ns) {
      telemetry.accelerometer.timestamp_ms.emplace_back((double)t * US_TO_S);
      telemetry.gyroscope.timestamp_ms.emplace_back((double)t * US_TO_S);
  }
  for (const auto& a : accl) {
      Eigen::Vector3d v;
      v << a[0], a[1], a[2];
      telemetry.accelerometer.measurement.emplace_back(v);
  }
  for (const auto& g : gyro) {
      Eigen::Vector3d v;
      v << g[0], g[1], g[2];
      telemetry.gyroscope.measurement.emplace_back(v);
  }

  file.close();
  return true;
}

} // namespace io
} // namespace OpenICC
