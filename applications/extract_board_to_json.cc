#include <algorithm>
#include <ios>
#include <iostream>
#include <chrono> // NOLINT
#include <fstream>
#include <gflags/gflags.h>
#include <string>
#include <time.h>
#include <vector>

#include "OpenCameraCalibrator/utils/board_extractor.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

DEFINE_string(input_video, "", "Path to save charuco board to.");
DEFINE_string(board_type, "charuco", "Board type. (charuco, radon)");
DEFINE_string(aruco_detector_params, "", "Path detector yaml.");
DEFINE_double(downsample_factor, 2, "Downsample factor for images.");
DEFINE_string(save_corners_json_path, "",
              "Where to save the recon dataset to.");
DEFINE_double(checker_size_m, 0.022,
              "Size of one square on the checkerbaord in [m]. Needed to only "
              "take far away poses!");
DEFINE_bool(verbose, false, "If more stuff should be printed");

using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
using nlohmann::json;

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  json output_json;

  int squaresX = 10;
  int squaresY = 8;
  float square_length = FLAGS_checker_size_m;
  float marker_length = FLAGS_checker_size_m / 2.0;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;

  BoardExtractor board_extractor;

  BoardType board_type = StringToBoardType(FLAGS_board_type);

  if (board_type == BoardType::CHARUCO) {
    board_extractor.InitializeCharucoBoard(FLAGS_aruco_detector_params,
                                           marker_length, square_length,
                                           squaresX, squaresY, dictionaryId);
  } else if (board_type == BoardType::RADON) {

  }

  VideoCapture input_video;
  input_video.open(FLAGS_input_video);
  int cnt_wrong = 0;
  double fps = input_video.get(cv::CAP_PROP_FPS);

  output_json["camera_fps"] = fps;
  output_json["calibration_board_type"] = FLAGS_board_type;
  output_json["square_size_meter"] = square_length;

  std::vector<cv::Point3f> board_pts = board_extractor.GetBoardPts()[0];
  for (size_t i = 0; i < board_pts.size(); ++i) {
    output_json["scene_pts"][std::to_string(i)] = {
        board_pts[i].x, board_pts[i].y, board_pts[i].z};
  }

  int frame_cnt = 0;
  bool set_img_size = false;
  while (true) {
    Mat image;
    if (!input_video.read(image)) {
      cnt_wrong++;
      if (cnt_wrong > 500)
        break;
      continue;
    }
    const double timstamp_s = input_video.get(cv::CAP_PROP_POS_MSEC) * 1000.0;
    const std::string view_s = std::to_string(timstamp_s);
    ++frame_cnt;

    const double fxfy = 1. / FLAGS_downsample_factor;
    cv::resize(image, image, cv::Size(), fxfy, fxfy);

    aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> ids;
    board_extractor.ExtractBoard(image, corners, ids);

    for (size_t c = 0; c < ids.size(); ++c) {
      output_json["views"][view_s]["image_points"][std::to_string(ids[c])] = {corners[c][0],corners[c][1]};
    }
   if (!set_img_size) {
    output_json["image_width"] = image.cols;
    output_json["image_height"] = image.rows;
    set_img_size = true;
   }
  }

  std::vector<std::uint8_t> v_bson = json::to_ubjson(output_json);

  std::ofstream calib_txt_output(FLAGS_save_corners_json_path, std::ios::out | std::ios::binary);
  calib_txt_output.write(reinterpret_cast<const char*>(&v_bson[0]), v_bson.size()*sizeof(std::uint8_t));

  return 0;
}
