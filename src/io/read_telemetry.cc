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

bool ReadTelemetryJSON(const std::string& path_to_telemetry_file,
                       CameraTelemetryData& telemetry) {
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

  const size_t nr_datapoints = timestamps_ns.size();
  if (gyro.size() != nr_datapoints || accl.size() != nr_datapoints) {
    std::cerr << "Telemetry should have the same amount of timestamps, "
                 "accelerometer and gyroscope values.\n";
    return false;
  }

  for (size_t i = 0; i < timestamps_ns.size(); ++i) {
    double t_s = (double)timestamps_ns[i] * NS_TO_S;
    ImuReading<double> acc_reading(t_s, accl[i][0], accl[i][1], accl[i][2]);
    ImuReading<double> gyr_reading(t_s, gyro[i][0], gyro[i][1], gyro[i][2]);
    telemetry.accelerometer.push_back(acc_reading);
    telemetry.gyroscope.push_back(gyr_reading);
  }

  // if we have accurate image timestamps
  // otherwise video timestamps will be used
  const auto accurate_timestamps_ns = j["img_timestamps_ns"];
  if (accurate_timestamps_ns.size() > 0) {
    for (size_t i = 0; i < accurate_timestamps_ns.size(); ++i) {
      telemetry.img_timestamps_s.emplace_back(
                  (double)accurate_timestamps_ns[i]*NS_TO_S);
    }
  }

  file.close();
  return true;
}

}  // namespace io
}  // namespace OpenICC
