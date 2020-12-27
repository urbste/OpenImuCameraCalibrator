#include <algorithm>
#include <chrono> // NOLINT
#include <fstream>
#include <gflags/gflags.h>
#include <ios>
#include <iostream>
#include <string>
#include <time.h>
#include <vector>

#include "OpenCameraCalibrator/core/board_extractor.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

DEFINE_string(input_video, "", "Path to save charuco board to.");
DEFINE_string(board_type, "charuco", "Board type. (charuco, radon)");
DEFINE_string(aruco_detector_params, "", "Path detector yaml.");
DEFINE_double(downsample_factor, 2.0,
              "Downsample factor for images. I_new = 1/factor * I");
DEFINE_string(save_corners_json_path, "",
              "Where to save the recon dataset to.");
DEFINE_double(checker_square_length_m, 0.017,
              "Size of one square on the checkerbaord in [m]. Needed to only "
              "take far away poses!");
DEFINE_int32(num_squared_x, 14, "Number of squares in x.");
DEFINE_int32(num_squared_y, 9, "Number of squares in y");
DEFINE_int32(aruco_dict, cv::aruco::DICT_ARUCO_ORIGINAL,
             "Aruco dictionary id.");
DEFINE_bool(recompute_corners, false, "If corners should be extracted again.");
DEFINE_bool(verbose, false, "If more stuff should be printed");

using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
using namespace OpenCamCalib::core;
using nlohmann::json;

bool DoesFileExist(const std::string &path) {
  std::ifstream file(path);
  if (file.is_open()) {
    file.close();
    return true;
  }
  return false;
}

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  ::google::InitGoogleLogging(argv[0]);

  if (DoesFileExist(FLAGS_save_corners_json_path) && !FLAGS_recompute_corners) {
    LOG(INFO) << "Skipping corner extraction. Already extracted for: "
              << FLAGS_input_video << "\n";
    return 0;
  }

  BoardExtractor board_extractor;
  if (FLAGS_verbose) {
    board_extractor.SetVerbosePlot();
  }
  BoardType board_type = StringToBoardType(FLAGS_board_type);
  if (board_type == BoardType::CHARUCO) {
    const float aruco_marker_length = FLAGS_checker_square_length_m / 2.0f;

    board_extractor.InitializeCharucoBoard(
        FLAGS_aruco_detector_params, aruco_marker_length,
        FLAGS_checker_square_length_m, FLAGS_num_squared_x, FLAGS_num_squared_y,
        FLAGS_aruco_dict);
  } else if (board_type == BoardType::RADON) {
     board_extractor.InitializeRadonBoard(FLAGS_checker_square_length_m, FLAGS_num_squared_x, FLAGS_num_squared_y);
  }

  LOG(INFO) << "Starting board extraction. This might take a while...";
  board_extractor.ExtractVideoToJson(
      FLAGS_input_video, FLAGS_save_corners_json_path, FLAGS_downsample_factor);

  return 0;
}
