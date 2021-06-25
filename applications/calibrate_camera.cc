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

#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/core/camera_calibrator.h"
#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/json.h"

using namespace OpenICC;
using namespace OpenICC::core;

DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_model_to_calibrate, "DOUBLE_SPHERE",
              "What camera model do you want to calibrate. Options:"
              "PINHOLE,PINHOLE_RADIAL_TANGENTIAL,DIVISION_UNDISTORTION,DOUBLE_SPHERE,EXTENDED_UNIFIED,FISHEYE");
DEFINE_string(save_path_calib_dataset, "",
              "Where to save the recon dataset to.");
DEFINE_double(grid_size, 0.04,
              "Only take images that are at least grid_size apart");
DEFINE_bool(optimize_board_points, false,
            "If in the end also the scene points should be adjusted. (if the "
            "board is not planar)");
DEFINE_bool(verbose, false, "If more stuff should be printed");

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  nlohmann::json scene_json;
  CHECK(io::read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  CameraCalibrator camera_calibrator(FLAGS_camera_model_to_calibrate, FLAGS_optimize_board_points);
  camera_calibrator.SetGridSize(FLAGS_grid_size);
  if (FLAGS_verbose) {
    camera_calibrator.SetVerbose();
  }
  camera_calibrator.CalibrateCameraFromJson(scene_json,
                                            FLAGS_save_path_calib_dataset);
  camera_calibrator.PrintResult();

  return 0;
}
