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
#include <iostream>
#include <string>
#include <vector>

#include "OpenCameraCalibrator/core/pose_estimator.h"
#include "OpenCameraCalibrator/io/read_camera_calibration.h"
#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include <theia/io/reconstruction_writer.h>
#include <theia/io/write_ply_file.h>

using namespace OpenICC::core;
using namespace OpenICC::io;

// Input/output files.
DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_calibration_json, "", "Path to camera calibration json.");
DEFINE_string(output_pose_dataset,
              "",
              "Path to write the pose calibration dataset to.");
DEFINE_bool(optimize_board_points,
            false,
            "If board points should be optimized.");

int main(int argc, char* argv[]) {
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
  if (FLAGS_optimize_board_points) {
    LOG(INFO) << "Optimizing board points.\n";
    pose_estimator.OptimizeBoardPoints();
    pose_estimator.OptimizeAllPoses();
  }

  theia::Reconstruction pose_dataset;
  pose_estimator.GetPoseDataset(pose_dataset);
  theia::WriteReconstruction(pose_dataset, FLAGS_output_pose_dataset);
  theia::WritePlyFile(FLAGS_output_pose_dataset + ".ply",
                      pose_dataset,
                      Eigen::Vector3i(255, 0, 0),
                      2);

  return 0;
}
