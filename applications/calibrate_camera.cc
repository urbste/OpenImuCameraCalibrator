#include <algorithm>
#include <chrono> // NOLINT
#include <fstream>
#include <gflags/gflags.h>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <theia/io/reconstruction_writer.h>
#include <theia/sfm/bundle_adjustment/bundle_adjuster.h>
#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/core/camera_calibrator.h"
#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/io/write_camera_calibration.h"
#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;
using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
using namespace OpenCamCalib::core;
using namespace OpenCamCalib::io;

DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_model_to_calibrate, "DOUBLE_SPHERE",
              "What camera model do you want to calibrate. Options:"
              "LINEAR_PINHOLE,DIVISION_UNDISTORTION,DOUBLE_SPHERE");
DEFINE_string(save_path_calib_dataset, "",
              "Where to save the recon dataset to.");
DEFINE_double(grid_size, 0.04,
              "Only take images that are at least grid_size apart");
DEFINE_bool(optimize_scene_points, false,
            "If in the end also the scene points should be adjusted. (if the "
            "board is not planar)");
DEFINE_bool(verbose, false, "If more stuff should be printed");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  nlohmann::json scene_json;
  CHECK(read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  CameraCalibrator camera_calibrator(FLAGS_camera_model_to_calibrate);
  camera_calibrator.SetGridSize(FLAGS_grid_size);
  if (FLAGS_verbose) {
    camera_calibrator.SetVerbose();
  }
  camera_calibrator.CalibrateCameraFromJson(scene_json,
                                            FLAGS_save_path_calib_dataset);

  return 0;
}
