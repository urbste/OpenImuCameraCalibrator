
#include "OpenCameraCalibrator/core/board_extractor.h"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <algorithm>
#include <fstream>
#include <ios>
#include <vector>

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

namespace OpenCamCalib {
namespace core {

BoardExtractor::BoardExtractor() {}

bool BoardExtractor::InitializeCharucoBoard(std::string path_to_detector_params,
                                            float marker_length,
                                            float square_length, int squaresX,
                                            int squaresY, int dictionaryId) {
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

bool BoardExtractor::InitializeRadonBoard(float square_length, int squaresX,
                                          int squaresY) {
  radon_pattern_size_ = cv::Size(squaresX, squaresY);
  radon_flags_ =
      cv::CALIB_CB_LARGER | cv::CALIB_CB_MARKER | cv::CALIB_CB_EXHAUSTIVE;
  std::vector<cv::Point3f> board_pts;
  int cont_idx = 0;
  for (int i = 0; i < radon_pattern_size_.height; ++i) {
    for (int j = 0; j < radon_pattern_size_.width; ++j) {
      radon_board_indices_.push_back(cont_idx);
      ++cont_idx;
      board_pts.push_back(
          cv::Point3f((float)i * square_length, (float)j * square_length, 0.f));
    }
  }
  board_pts3d_.push_back(board_pts);
  square_length_m_ = square_length;
  board_type_ = BoardType::RADON;
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
      for (int i = 0; i < charuco_corners.size(); ++i) {
        corners.push_back(
            Eigen::Vector2d(charuco_corners[i].x, charuco_corners[i].y));
      }
      return true;
    } else {
      return false;
    }

  } else if (board_type_ == BoardType::RADON) {
    std::vector<Point2d> radon_corners;
    cv::Mat meta;
    bool success = cv::findChessboardCornersSB(
        image, radon_pattern_size_, radon_corners, radon_flags_, meta);
    if (!success)
      return false;
    int lf = 0;
    for (int i = 0; i < radon_pattern_size_.height; ++i) {
      for (int j = 0; j < radon_pattern_size_.width; ++j) {
        if ((int)meta.at<uchar>(i, j) >= 0) {
          object_pt_ids.push_back(radon_board_indices_[lf]);
        }
        lf++;
      }
    }
    for (int i = 0; i < radon_corners.size(); ++i) {
      corners.push_back(
          Eigen::Vector2d(radon_corners[i].x, radon_corners[i].y));
    }
  } else {
    LOG(WARNING) << " Board type does not exist.";
    return false;
  }

  return true;
}

bool BoardExtractor::ExtractVideoToJson(const std::string &video_path,
                                        const std::string &save_path,
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
  if (board_type_ == BoardType::CHARUCO) {
    for (size_t i = 0; i < board_pts.size(); ++i) {
      output_json["scene_pts"][std::to_string(i)] = {
          board_pts[i].x, board_pts[i].y, board_pts[i].z};
    }
  } else if (board_type_ == BoardType::RADON) {
    std::vector<int> board_ids = GetRadonBoardIDs();
    for (size_t i = 0; i < board_ids.size(); ++i) {
      output_json["scene_pts"][board_ids[i]] = {
          board_pts[i].x, board_pts[i].y, board_pts[i].z};
    }
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

    const double timstamp_s = input_video.get(cv::CAP_PROP_POS_MSEC) * 1e-3;
    const std::string view_us = std::to_string(timstamp_s * 1e6);
    ++frame_cnt;

    const double fxfy = 1. / img_downsample_factor;
    cv::resize(image, image, cv::Size(), fxfy, fxfy);

    aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> ids;
    ExtractBoard(image, corners, ids);

    for (size_t c = 0; c < ids.size(); ++c) {
      output_json["views"][view_us]["image_points"][std::to_string(ids[c])] = {
          corners[c][0], corners[c][1]};
    }
    if (!set_img_size) {
      output_json["image_width"] = image.cols;
      output_json["image_height"] = image.rows;
      set_img_size = true;
    }

    LOG_IF(INFO, frame_cnt % 60 == 0)
        << "Extracting corners from frame " << frame_cnt << " / "
        << total_nr_frames << "\n";

    if (verbose_plot_) {
      for (int i = 0; i < corners.size(); ++i) {
        cv::drawMarker(
            image, cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
            cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 3);

        cv::putText(image, std::to_string(ids[i]),
                    cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
      }
      cv::putText(image, "Number corners: " + std::to_string(corners.size()),
                  cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                  cv::Scalar(0, 0, 255));
      cv::imshow("corners", image);
      cv::waitKey(1);
    }
  }

  std::vector<std::uint8_t> v_bson = nlohmann::json::to_ubjson(output_json);

  std::ofstream calib_txt_output(save_path, std::ios::out | std::ios::binary);
  calib_txt_output.write(reinterpret_cast<const char *>(&v_bson[0]),
                         v_bson.size() * sizeof(std::uint8_t));
}

} // namespace core
} // namespace OpenCamCalib
