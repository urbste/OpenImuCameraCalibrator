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
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "OpenCameraCalibrator/core/allan_variance_fitter.h"
#include "OpenCameraCalibrator/io/read_telemetry.h"
#include "OpenCameraCalibrator/utils/json.h"

using namespace OpenICC;
using namespace OpenICC::core;

DEFINE_string(telemetry_json,
              "/media/Data/Sparsenet/"
              "CameraCalibrationStudy/AllanVariance/merged_telemetry.json",
              "Path to the telemetry json.");

DEFINE_bool(verbose, false, "If more stuff should be printed");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  // read telemetry
  CameraTelemetryData telemetry_data;
  CHECK(io::ReadTelemetryJSON(FLAGS_telemetry_json, telemetry_data))
      << "Could not read: " << FLAGS_telemetry_json;

  AllanVarianceFitter fitter(telemetry_data, 10000);
  fitter.RunFit();

  return 0;
}
