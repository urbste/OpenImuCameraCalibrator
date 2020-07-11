#include <dirent.h>
#include <gflags/gflags.h>
#include <time.h>
#include <algorithm>
#include <chrono>  // NOLINT
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include "OpenCameraCalibrator/imu/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"
#include "OpenCameraCalibrator/filter/estimate_camera_imu_alignment.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/reconstruction.h"

#include "OpenCameraCalibrator/utils/json.h"

using json = nlohmann::json;

// Input/output files.
DEFINE_string(
    input_pose_calibration_dataset, "",
    "Path to input calibration dataset.");
DEFINE_string(
    gopro_telemetry_json, "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(
    gyro_calibration_output, "gyro_to_cam_calibration.json",
    "Gyroscope to camera calibration output path.");

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load camera calibration reconstuction.
  theia::Reconstruction pose_dataset;
  CHECK(theia::ReadReconstruction(
        FLAGS_input_pose_calibration_dataset, &pose_dataset));

  // read gopro telemetry
  OpenCamCalib::CameraTelemetryData telemetry_data;
  if (!OpenCamCalib::ReadGoProTelemetry(FLAGS_gopro_telemetry_json,
                                        telemetry_data)) {
    std::cout << "Could not read: " << FLAGS_gopro_telemetry_json << std::endl;
  }

  Vec3Map angular_velocities;
  for (size_t i = 0; i < telemetry_data.gyroscope.gyro_measurement.size(); ++i) {
    angular_velocities[telemetry_data.gyroscope.timestamp_ms[i] / 1000.0] =
                              telemetry_data.gyroscope.gyro_measurement[i];
  }
  // get mean hz imu
  double imu_dt_s = 0.0;
  for (size_t i = 1; i < telemetry_data.gyroscope.timestamp_ms.size(); ++i) {
      imu_dt_s += telemetry_data.gyroscope.timestamp_ms[i] - telemetry_data.gyroscope.timestamp_ms[i-1];
  }
  imu_dt_s /= static_cast<double>(telemetry_data.gyroscope.timestamp_ms.size()-1);
  imu_dt_s /= 1000.0;

  QuatMap visual_rotations;
  for (size_t i = 0; i < pose_dataset.ViewIds().size(); ++i) {
    const theia::View* view = pose_dataset.View(pose_dataset.ViewIds()[i]);
    const double timestamp_s = std::stod(view->Name()); // holds timestamp
    // cam to world trafo, so transposed rotation matrix
    Eigen::Quaterniond vis_quat(view->Camera().GetOrientationAsRotationMatrix().transpose());
    visual_rotations[timestamp_s] = vis_quat;
  }
  std::vector<double> timestamps_images;
  for (const auto& vis_rot : visual_rotations) {
      timestamps_images.push_back(vis_rot.first);
  }

  // get mean hz camera
  std::vector<double> cams_dt_s;
  for (size_t i = 1; i < timestamps_images.size(); ++i) {
      cams_dt_s.push_back(timestamps_images[i] - timestamps_images[i-1]);
  }
  // we take the median as some images might not have been estimated
  const double cam_dt_s = OpenCamCalib::MedianOfDoubleVec(cams_dt_s);

  Eigen::Matrix3d R_gyro_to_camera;
  double time_offset_gyro_to_camera;
  Eigen::Vector3d gyro_bias;
  OpenCamCalib::filter::EstimateCameraImuAlignment(
      visual_rotations,
      angular_velocities,
      cam_dt_s,
      imu_dt_s,
      R_gyro_to_camera,
      time_offset_gyro_to_camera,
      gyro_bias);

  json output_json;
  output_json["gyro_bias"] = {gyro_bias[0], gyro_bias[1], gyro_bias[2]};
  Eigen::Quaterniond q_gyro_to_cam(R_gyro_to_camera);
  output_json["gyro_to_camera_rotation"]["w"] = q_gyro_to_cam.w();
  output_json["gyro_to_camera_rotation"]["x"] = q_gyro_to_cam.x();
  output_json["gyro_to_camera_rotation"]["y"] = q_gyro_to_cam.y();
  output_json["gyro_to_camera_rotation"]["z"] = q_gyro_to_cam.z();
  output_json["time_offset_gyro_to_cam"] = time_offset_gyro_to_camera;

  // write prettified JSON to another file
  std::ofstream out_file(FLAGS_gyro_calibration_output);
  out_file << std::setw(4) << output_json << std::endl;

  return 0;
}
