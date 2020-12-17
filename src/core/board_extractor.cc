
#include "OpenCameraCalibrator/core/board_extractor.h"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <algorithm>
#include <vector>
#include <ios>
#include <fstream>

#include "OpenCameraCalibrator/utils/utils.h"
#include "OpenCameraCalibrator/utils/json.h"

using namespace cv;

namespace OpenCamCalib {
namespace core {

BoardExtractor::BoardExtractor() {}

bool BoardExtractor::InitializeCharucoBoard(std::string path_to_detector_params,
                                            float marker_length, float square_length,
                                            int squaresX, int squaresY,
                                            int dictionaryId) {
  // load images from folder
  detector_params_ = aruco::DetectorParameters::create();

  if (!OpenCamCalib::utils::ReadDetectorParameters(path_to_detector_params,
                                                   detector_params_)) {
    LOG(ERROR) << "Invalid detector parameters file\n";
    return 0;
  }

  dictionary_ = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // create charuco board object
  charucoboard_ = aruco::CharucoBoard::create(squaresX, squaresY, square_length,
                                              marker_length, dictionary_);
  board_ = charucoboard_.staticCast<aruco::Board>();

  board_pts3d_.push_back(charucoboard_->chessboardCorners);
  board_type_ = BoardType::CHARUCO;

  square_length_m_ = square_length;

  board_initialized_ = true;
  return true;
}

bool BoardExtractor::InitializeRadonBoard(int square_length, int squaresX,
                                          int squaresY) {
  radon_pattern_size_ = cv::Size(squaresX, squaresY);
  radon_flags_ =
      cv::CALIB_CB_LARGER | cv::CALIB_CB_ACCURACY | cv::CALIB_CB_MARKER;
  std::vector<cv::Point3f> board_pts;
  for (int i = 0; i < radon_pattern_size_.height; ++i) {
    for (int j = 0; j < radon_pattern_size_.width; ++j) {
      board_pts.push_back(
          cv::Point3f((float)i * square_length, (float)j * square_length, 0.f));
    }
  }
  board_pts3d_.push_back(board_pts);
  square_length_m_ = square_length;

  board_initialized_ = true;
  return true;
}

bool BoardExtractor::ExtractBoard(const Mat &image,
                                  aligned_vector<Eigen::Vector2d> &corners,
                                  std::vector<int> &object_pt_ids) {

  if (board_type_ == BoardType::CHARUCO) {
    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<Point2f>> marker_corners, rejected_markers;
    std::vector<Point2f> charuco_corners;

    aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids,
                         detector_params_, rejected_markers);

    // refind strategy to detect more markers
    aruco::refineDetectedMarkers(image, board_, marker_corners, marker_ids,
                                 rejected_markers);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (marker_ids.size() > 0) {
      interpolatedCorners = aruco::interpolateCornersCharuco(
          marker_corners, marker_ids, image, charucoboard_, charuco_corners,
          charuco_ids);

      object_pt_ids = charuco_ids;
      for (int i=0; i < charuco_corners.size(); ++i) {
          corners.push_back(
                Eigen::Vector2d(charuco_corners[i].x, charuco_corners[i].y));
      }
      return true;
    } else {
        return false;
    }

  } else if (board_type_ == BoardType::RADON) {
    std::vector<Point2d> corners;
    cv::Mat meta;
    cv::findChessboardCornersSB(image, radon_pattern_size_, corners, radon_flags_, meta);
  } else {
      LOG(WARNING) << " Board type does not exist.";
      return false;
  }

  return true;
}

bool BoardExtractor::ExtractVideoToJson(const std::string& video_path,
                                        const std::string& save_path,
                                        const double img_downsample_factor) {
    if (!board_initialized_) {
        LOG(ERROR) << "No board initialized.\n";
        return false;
    }
    if (video_path == "") {
        LOG(ERROR) << "Video path is empty.\n";
        return false;
    }

    nlohmann::json output_json;
    VideoCapture input_video;
    input_video.open(video_path);
    int cnt_wrong = 0;
    double fps = input_video.get(cv::CAP_PROP_FPS);

    output_json["camera_fps"] = fps;
    output_json["calibration_board_type"] = board_type_;
    output_json["square_size_meter"] = square_length_m_;

    std::vector<cv::Point3f> board_pts = GetBoardPts()[0];
    for (size_t i = 0; i < board_pts.size(); ++i) {
      output_json["scene_pts"][std::to_string(i)] = {
          board_pts[i].x, board_pts[i].y, board_pts[i].z};
    }

    const int total_nr_frames = input_video.get(cv::CAP_PROP_FRAME_COUNT);
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

      const double fxfy = 1. / img_downsample_factor;
      cv::resize(image, image, cv::Size(), fxfy, fxfy);

      aligned_vector<Eigen::Vector2d> corners;
      std::vector<int> ids;
      ExtractBoard(image, corners, ids);


      for (size_t c = 0; c < ids.size(); ++c) {
        output_json["views"][view_s]["image_points"][std::to_string(ids[c])] = {corners[c][0],corners[c][1]};
      }
     if (!set_img_size) {
      output_json["image_width"] = image.cols;
      output_json["image_height"] = image.rows;
      set_img_size = true;
     }

     LOG_IF(INFO, frame_cnt % 60==0) << "Extracting corners from frame "<<frame_cnt<<" / "<<total_nr_frames<<"\n";
    }

    std::vector<std::uint8_t> v_bson = nlohmann::json::to_ubjson(output_json);

    std::ofstream calib_txt_output(save_path, std::ios::out | std::ios::binary);
    calib_txt_output.write(reinterpret_cast<const char*>(&v_bson[0]), v_bson.size()*sizeof(std::uint8_t));
}


} // namespace utils
} // namespace OpenCamCalib
