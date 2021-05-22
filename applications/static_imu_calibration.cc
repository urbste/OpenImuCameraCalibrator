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

#include <fstream>

#include "OpenCameraCalibrator/core/static_imu_calibrator.h"
#include "OpenCameraCalibrator/io/read_telemetry.h"
#include "OpenCameraCalibrator/utils/json.h"

using namespace OpenICC;
using namespace OpenICC::core;

DEFINE_string(
    telemetry_json,
    "/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/"
    "StaticMultiPose/GoPro9/dataset1/GX016705_gen.json",
    "Path to the telemetry json.");

DEFINE_double(gravity_magnitude, 9.811107, "Gravity magnitude.");

DEFINE_double(initial_static_interval_s, 29.0,
              "Length of the initial static interval for bias estimation.");

DEFINE_string(output_calibration_path, "", "path to output calibration json");
DEFINE_bool(verbose, false, "If more stuff should be printed");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  // read telemetry
  CameraTelemetryData telemetry_data;
  CHECK(io::ReadTelemetryJSON(FLAGS_telemetry_json, telemetry_data))
      << "Could not read: " << FLAGS_telemetry_json;

  StaticImuCalibrator multi_pose_calibrator;
  multi_pose_calibrator.SetGravityMagnitude(FLAGS_gravity_magnitude);
  multi_pose_calibrator.SetInitStaticIntervalDuration(
      FLAGS_initial_static_interval_s);
  multi_pose_calibrator.EnableVerboseOutput(FLAGS_verbose);
  multi_pose_calibrator.CalibrateAccGyro(telemetry_data.accelerometer,
                                         telemetry_data.gyroscope);

  // write result
  ThreeAxisSensorCalibParams<double> acc_calib =
      multi_pose_calibrator.getAccCalib();
  ThreeAxisSensorCalibParams<double> gyr_calib =
      multi_pose_calibrator.getGyroCalib();
  nlohmann::json output;

  Eigen::Matrix3d acc_m_mat = acc_calib.GetMisalignmentMatrix();
  output["accelerometer"]["misalignment_matrix"] = {
      {1.0, acc_m_mat(0, 1), acc_m_mat(0, 2)},
      {0.0, 1.0, acc_m_mat(1, 2)},
      {0.0, 0.0, 1.0}};
  output["accelerometer"]["scale_matrix"] = {{acc_calib.scaleX(), 0.0, 0.0},
                                             {0.0, acc_calib.scaleY(), 0.0},
                                             {0.0, 0.0, acc_calib.scaleZ()}};
  output["accelerometer"]["bias"] = {acc_calib.biasX(), acc_calib.biasY(),
                                     acc_calib.biasZ()};

  Eigen::Matrix3d gyr_m_mat = gyr_calib.GetMisalignmentMatrix();
  output["gyroscope"]["misalignment_matrix"] = {
      {1.0, gyr_m_mat(0, 1), gyr_m_mat(0, 2)},
      {gyr_m_mat(1, 0), 1.0, gyr_m_mat(1, 2)},
      {gyr_m_mat(2, 0), gyr_m_mat(2, 1), 1.0}};
  output["gyroscope"]["scale_matrix"] = {{gyr_calib.scaleX(), 0.0, 0.0},
                                         {0.0, gyr_calib.scaleY(), 0.0},
                                         {0.0, 0.0, gyr_calib.scaleZ()}};
  output["gyroscope"]["bias"] = {gyr_calib.biasX(), gyr_calib.biasY(),
                                 gyr_calib.biasZ()};
  std::ofstream output_json(FLAGS_output_calibration_path);
  output_json << std::setw(4) << output << std::endl;
  output_json.close();

  return 0;
}
