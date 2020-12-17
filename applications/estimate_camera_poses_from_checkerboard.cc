#include <algorithm>
#include <chrono> // NOLINT
#include <dirent.h>
#include <fstream>
#include <gflags/gflags.h>
#include <iostream>
#include <ostream>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/core/pose_estimator.h"
#include "OpenCameraCalibrator/imu/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/io/read_camera_calibration.h"
#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include <theia/io/reconstruction_writer.h>

using namespace cv;
using namespace OpenCamCalib::core;
using namespace OpenCamCalib::io;

// Input/output files.
DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_calibration_json, "", "Path to camera calibration json.");
DEFINE_string(output_pose_dataset, "",
              "Path to write the pose calibration dataset to.");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  nlohmann::json scene_json;
  CHECK(read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  // read camera calibration
  theia::Camera camera;
  double fps;
  CHECK(read_camera_calibration(FLAGS_camera_calibration_json, camera, fps))
      << "Could not read camera calibration: " << FLAGS_camera_calibration_json;

  LOG(INFO) << "Start pose estimation.\n";
  PoseEstimator pose_estimator;
  pose_estimator.EstimatePosesFromJson(scene_json, camera);
  LOG(INFO) << "Finished pose estimation.\n";

  theia::Reconstruction pose_dataset;
  pose_estimator.GetPoseDataset(pose_dataset);
  theia::WriteReconstruction(pose_dataset, FLAGS_output_pose_dataset);

  return 0;
}
