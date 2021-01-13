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

#include <fstream>
#include <gflags/gflags.h>

#include "OpenCameraCalibrator/core/imu_to_camera_rotation_estimator.h"
#include "OpenCameraCalibrator/io/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/reconstruction.h"

#include "OpenCameraCalibrator/utils/json.h"

using json = nlohmann::json;

using namespace OpenICC;
using namespace OpenICC::utils;
using namespace OpenICC::core;

// Input/output files.
DEFINE_string(input_pose_calibration_dataset, "",
              "Path to input calibration dataset.");
DEFINE_string(
    gopro_telemetry_json, "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(imu_bias_estimate, "",
              "Estimate to imu bias values. If empty, we will estimate this.");
DEFINE_string(imu_rotation_init_output, "gyro_to_cam_calibration.json",
              "Gyroscope to camera calibration output path.");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  // Load camera calibration reconstuction.
  theia::Reconstruction pose_dataset;
  CHECK(theia::ReadReconstruction(FLAGS_input_pose_calibration_dataset,
                                  &pose_dataset));

  ImuToCameraRotationEstimator rotation_estimator;

  Eigen::Vector3d accl_bias, gyro_bias;
  accl_bias.setZero();
  gyro_bias.setZero();
  if (FLAGS_imu_bias_estimate != "") {
    // IMU Bias
    LOG(INFO) << "Load IMU bias file: " << FLAGS_imu_bias_estimate << std::endl;
    OpenICC::ReadIMUBias(FLAGS_imu_bias_estimate, gyro_bias, accl_bias);
  } else {
    // if no bias is given we can also estimate it here
    rotation_estimator.EnableGyroBiasEstimation();
  }

  // read gopro telemetry
  OpenICC::CameraTelemetryData telemetry_data;
  if (!OpenICC::ReadGoProTelemetry(FLAGS_gopro_telemetry_json,
                                        telemetry_data)) {
    std::cout << "Could not read: " << FLAGS_gopro_telemetry_json << std::endl;
  }

  // fill measurementes
  vec3_map angular_velocities, acclerations;
  for (size_t i = 0; i < telemetry_data.gyroscope.gyro_measurement.size();
       ++i) {
    angular_velocities[telemetry_data.gyroscope.timestamp_ms[i] * 1e-3] =
        telemetry_data.gyroscope.gyro_measurement[i] + gyro_bias;
    acclerations[telemetry_data.gyroscope.timestamp_ms[i] * 1e-3] =
        telemetry_data.accelerometer.acc_measurement[i] + accl_bias;
  }

  // get mean hz imu
  double imu_dt_s = 0.0;
  for (size_t i = 1; i < telemetry_data.gyroscope.timestamp_ms.size(); ++i) {
    imu_dt_s += telemetry_data.gyroscope.timestamp_ms[i] -
                telemetry_data.gyroscope.timestamp_ms[i - 1];
  }
  imu_dt_s /=
      static_cast<double>(telemetry_data.gyroscope.timestamp_ms.size() - 1);
  imu_dt_s *= 1e-3;

  quat_map visual_rotations;
  for (size_t i = 0; i < pose_dataset.ViewIds().size(); ++i) {
    const theia::View *view = pose_dataset.View(pose_dataset.ViewIds()[i]);
    const double timestamp_s = view->GetTimestamp();
    // cam to world trafo, so transposed rotation matrix
    Eigen::Quaterniond vis_quat(
        view->Camera().GetOrientationAsRotationMatrix());
    visual_rotations[timestamp_s] = vis_quat;
  }
  std::vector<double> timestamps_images;
  for (const auto &vis_rot : visual_rotations) {
    timestamps_images.push_back(vis_rot.first);
  }

  // get mean hz camera
  std::vector<double> cams_dt_s;
  for (size_t i = 1; i < timestamps_images.size(); ++i) {
    cams_dt_s.push_back(timestamps_images[i] - timestamps_images[i - 1]);
  }
  // we take the median as some images might not have been estimated
  const double cam_dt_s = OpenICC::utils::MedianOfDoubleVec(cams_dt_s);

  std::vector<double> tVis_all_frames, tVis_missing_frames;
  for (double t = visual_rotations.begin()->first;
       t < visual_rotations.rbegin()->first; t += cam_dt_s) {
    tVis_all_frames.push_back(t);
  }
  quat_vector visual_rotations_missing_frames;
  for (auto const &vis : visual_rotations) {
    tVis_missing_frames.push_back(vis.first);
    visual_rotations_missing_frames.push_back(vis.second);
  }
  // interpolate visual rotations as some views might be missing
  quat_vector visual_rotations_interpolated_vec;
  OpenICC::utils::InterpolateQuaternions(
      tVis_missing_frames, tVis_all_frames, visual_rotations_missing_frames,
      visual_rotations_interpolated_vec);
  quat_map visual_rotations_interpolated;
  for (int i = 0; i < visual_rotations_interpolated_vec.size(); ++i) {
    visual_rotations_interpolated[tVis_all_frames[i]] =
        visual_rotations_interpolated_vec[i];
  }

  Eigen::Matrix3d R_gyro_to_camera;
  double time_offset_gyro_to_camera;
  vec3_vector ang_vel, imu_vel;

  rotation_estimator.SetAngularVelocities(angular_velocities);
  rotation_estimator.SetVisualRotations(visual_rotations_interpolated);
  rotation_estimator.EstimateCameraImuRotation(
      cam_dt_s, imu_dt_s, R_gyro_to_camera, time_offset_gyro_to_camera,
      gyro_bias, imu_vel, ang_vel);

  json output_json;
  output_json["gyro_bias"] = {gyro_bias[0], gyro_bias[1], gyro_bias[2]};
  Eigen::Quaterniond q_gyro_to_cam(R_gyro_to_camera);
  output_json["gyro_to_camera_rotation"]["w"] = q_gyro_to_cam.w();
  output_json["gyro_to_camera_rotation"]["x"] = q_gyro_to_cam.x();
  output_json["gyro_to_camera_rotation"]["y"] = q_gyro_to_cam.y();
  output_json["gyro_to_camera_rotation"]["z"] = q_gyro_to_cam.z();
  output_json["time_offset_gyro_to_cam"] = time_offset_gyro_to_camera;

  // write prettified JSON to another file
  std::ofstream out_file(FLAGS_imu_rotation_init_output);
  out_file << std::setw(4) << output_json << std::endl;

  // write to txt for testing
  // std::ofstream
  // vis_out("/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/poses.txt");
  // std::ofstream
  // acc_out("/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/accelerometer.txt");
  // std::ofstream
  // gyr_out("/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/gyroscope_original.txt");
  std::ofstream gyr_out_trafo("/media/steffen/0F78151A1CEDE4A2/Sparsenet/"
                              "SparsnetTests2020/gyroscope_transformed.txt");
  std::ofstream vis_interp_out("/media/steffen/0F78151A1CEDE4A2/Sparsenet/"
                               "SparsnetTests2020/visual_gyroscope.txt");
  std::ofstream vis_interp_all_out("/media/steffen/0F78151A1CEDE4A2/Sparsenet/"
                               "SparsnetTests2020/visual_gyroscope_all.txt");
  //  for (auto v : visual_rotations) {
  //    Eigen::Vector3d pos = visual_translation.find(v.first)->second;
  //    vis_out << v.first * 1e9 << " " << pos[0] <<" "<<pos[1]<<" "<<
  //               pos[2]<<" "<<v.second.w()<< " " <<
  //               v.second.x()<< " " << v.second.y()<< " " <<
  //               v.second.z()<<"\n";
  //  }
  //  vis_out.close();
  //  for (auto a : acclerations) {
  //    acc_out << a.first * 1e9 << " " << a.second[0] <<" "<<a.second[1]<<"
  //    "<<a.second[2]<<"\n";
  //  }
  //  acc_out.close();
  //  for (auto v : angular_velocities) {
  //    gyr_out << v.first * 1e9 << " " << v.second[0] <<" "<<v.second[1]<<"
  //    "<<v.second[2]<<"\n";
  //  }
  //  gyr_out.close();
  //  for (auto v : imu_vel) {
  //    gyr_out << 1 << " " << v[0] <<" "<<v[1]<<" "<<v[2]<<"\n";
  //  }
  //  gyr_out.close();
  for (auto v : imu_vel) {
    Eigen::Vector3d a = R_gyro_to_camera * v;
    gyr_out_trafo << 1 << " " << a[0] << " " << a[1] << " " << a[2] << "\n";
  }
  gyr_out_trafo.close();
  for (auto v : ang_vel) {
    vis_interp_out << 1 << " " << v[0] << " " << v[1] << " " << v[2] << "\n";
  }
  vis_interp_out.close();
  for (auto q : visual_rotations) {
    vis_interp_all_out << 1 << " " << q.second.w() << " " << q.second.x() << " " << q.second.y() << " " <<q.second.z()<< "\n";
  }
  vis_interp_all_out.close();
  return 0;
}
