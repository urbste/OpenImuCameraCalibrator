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
#include <ios>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "OpenCameraCalibrator/core/board_extractor.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

DEFINE_string(input_path, "", "Input path.");
DEFINE_string(board_type, "charuco", "Board type. (charuco, radon, apriltag)");
DEFINE_string(aruco_detector_params, "", "Path detector yaml.");
DEFINE_double(downsample_factor,
              1.0,
              "Downsample factor for images. I_new = 1/factor * I");
DEFINE_string(save_corners_json_path,
              "",
              "Where to save the recon dataset to.");
DEFINE_double(checker_square_length_m,
              0.022,
              "Size of one square on the checkerboard in [m]. Needed to only "
              "take far away poses!");
DEFINE_int32(num_squares_x, 9, "Number of squares in x.");
DEFINE_int32(num_squares_y, 7, "Number of squares in y");
DEFINE_int32(aruco_dict,
             cv::aruco::DICT_ARUCO_ORIGINAL,
             "Aruco dictionary id.");
DEFINE_bool(recompute_corners, false, "If corners should be extracted again.");
DEFINE_bool(verbose, false, "If more stuff should be printed");

using namespace OpenICC;
using namespace OpenICC::utils;
using namespace OpenICC::core;
using nlohmann::json;

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  if (DoesFileExist(FLAGS_save_corners_json_path) && !FLAGS_recompute_corners) {
    LOG(INFO) << "Skipping corner extraction. Already extracted for: "
              << FLAGS_input_path << "\n";
    return 0;
  }

  BoardExtractor board_extractor;
  if (FLAGS_verbose) {
    board_extractor.SetVerbosePlot();
  }
  BoardType board_type = StringToBoardType(FLAGS_board_type);
  if (board_type == BoardType::CHARUCO) {
    const float aruco_marker_length = FLAGS_checker_square_length_m / 2.0f;
    board_extractor.InitializeCharucoBoard(FLAGS_aruco_detector_params,
                                           aruco_marker_length,
                                           FLAGS_checker_square_length_m,
                                           FLAGS_num_squares_x,
                                           FLAGS_num_squares_y,
                                           FLAGS_aruco_dict);
  } else if (board_type == BoardType::RADON) {
    board_extractor.InitializeRadonBoard(FLAGS_checker_square_length_m,
                                         FLAGS_num_squares_x,
                                         FLAGS_num_squares_y);
  } else if (board_type == BoardType::APRILTAG) {
    board_extractor.InitializeAprilBoard(FLAGS_checker_square_length_m,
                                         0.3,
                                         FLAGS_num_squares_x,
                                         FLAGS_num_squares_y);
  } else {
    LOG(ERROR) << "This board type does not exist! Choose Charuco or Radon";
  }

  LOG(INFO) << "Starting board extraction. This might take a while...";
  if (IsPathAFile(FLAGS_input_path)) {
    board_extractor.ExtractVideoToJson(FLAGS_input_path,
                                       FLAGS_save_corners_json_path,
                                       FLAGS_downsample_factor);

  } else {
    board_extractor.ExtractImageFolderToJson(FLAGS_input_path,
                                             FLAGS_save_corners_json_path,
                                             FLAGS_downsample_factor);
  }
  return 0;
}
