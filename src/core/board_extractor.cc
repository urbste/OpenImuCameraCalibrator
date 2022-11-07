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

#include "OpenCameraCalibrator/core/board_extractor.h"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <third_party/apriltag/ethz_apriltag2/include/apriltags/TagDetection.h>

#include <algorithm>
#include <fstream>
#include <ios>
#include <vector>

#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

namespace OpenICC {
namespace core {

BoardExtractor::BoardExtractor() {}

bool BoardExtractor::InitializeCharucoBoard(std::string path_to_detector_params,
                                            float marker_length,
                                            float square_length,
                                            int squaresX,
                                            int squaresY,
                                            int dictionaryId) {
  // load images from folder
  detector_params_ = aruco::DetectorParameters::create();

  if (!OpenICC::utils::ReadDetectorParameters(path_to_detector_params,
                                              detector_params_)) {
    LOG(ERROR) << "Invalid detector parameters file\n";
    return 0;
  }

  dictionary_ = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // create charuco board object
  charucoboard_ = aruco::CharucoBoard::create(
      squaresX, squaresY, square_length, marker_length, dictionary_);
  board_ = charucoboard_.staticCast<aruco::Board>();

  board_pts3d_.push_back(charucoboard_->chessboardCorners);
  board_type_ = BoardType::CHARUCO;

  square_length_m_ = square_length;

  board_initialized_ = true;
  return true;
}

bool BoardExtractor::InitializeRadonBoard(float square_length,
                                          int squaresX,
                                          int squaresY) {
  radon_pattern_size_ = cv::Size(squaresX, squaresY);
  radon_flags_ =
      cv::CALIB_CB_LARGER | cv::CALIB_CB_MARKER | cv::CALIB_CB_EXHAUSTIVE;
  std::vector<cv::Point3f> board_pts;
  int cont_idx = 0;
  for (int i = 0; i < radon_pattern_size_.height; ++i) {
    for (int j = 0; j < radon_pattern_size_.width; ++j) {
      continuous_board_indices_.push_back(cont_idx);
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

bool BoardExtractor::InitializeAprilBoard(double marker_length,
                                          double tag_spacing,
                                          int squaresX,
                                          int squaresY) {
  double x_corner_offsets[4] = {0, marker_length, marker_length, 0};
  double y_corner_offsets[4] = {0, 0, marker_length, marker_length};

  std::vector<cv::Point3f> board_pts;
  board_pts.resize(squaresX * squaresY * NUM_PTS_MARKER);

  for (int y = 0; y < squaresY; y++) {
    for (int x = 0; x < squaresX; x++) {
      int tag_id = squaresX * y + x;
      double x_offset = x * marker_length * (1 + tag_spacing);
      double y_offset = y * marker_length * (1 + tag_spacing);

      for (int i = 0; i < NUM_PTS_MARKER; i++) {
        int corner_id = (tag_id << 2) + i;
        board_pts[corner_id].x = x_offset + x_corner_offsets[i];
        board_pts[corner_id].y = y_offset + y_corner_offsets[i];
        board_pts[corner_id].z = 0;
      }
    }
  }
  board_pts3d_.push_back(board_pts);
  square_length_m_ = marker_length;
  board_type_ = BoardType::APRILTAG;
  board_initialized_ = true;
  return true;
}

bool BoardExtractor::ExtractBoard(const Mat& image,
                                  aligned_vector<Eigen::Vector2d>& corners,
                                  std::vector<int>& object_pt_ids) {
  if (board_type_ == BoardType::CHARUCO) {
    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<Point2f>> marker_corners, rejected_markers;
    std::vector<Point2f> charuco_corners;

    aruco::detectMarkers(image,
                         dictionary_,
                         marker_corners,
                         marker_ids,
                         detector_params_,
                         rejected_markers);

    // refind strategy to detect more markers
    aruco::refineDetectedMarkers(image,
                                 board_,
                                 marker_corners,
                                 marker_ids,
                                 rejected_markers,
                                 cv::noArray(),
                                 cv::noArray(),
                                 10);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (marker_ids.size() > 0) {
      interpolatedCorners = aruco::interpolateCornersCharuco(marker_corners,
                                                             marker_ids,
                                                             image,
                                                             charucoboard_,
                                                             charuco_corners,
                                                             charuco_ids,
                                                             cv::noArray(),
                                                             cv::noArray(),
                                                             1);

      if (charuco_corners.size() > 4) {
        cv::cornerSubPix(
            image,
            charuco_corners,
            cv::Size(detector_params_->cornerRefinementWinSize,
                     detector_params_->cornerRefinementWinSize),
            cv::Size(-1, -1),
            cv::TermCriteria(
                cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.001));

        //          if (marker_ids.size() > 0) {
        //              cv::Mat imageCopy;
        //              image.copyTo(imageCopy);
        //              cv::cvtColor(imageCopy,imageCopy, cv::COLOR_GRAY2BGR);
        //              cv::aruco::drawDetectedMarkers(imageCopy,
        //              marker_corners, marker_ids);
        //              // if at least one charuco corner detected
        //              if (charuco_ids.size() > 0)
        //                  cv::aruco::drawDetectedCornersCharuco(imageCopy,
        //                  charuco_corners, charuco_ids, cv::Scalar(255, 0,
        //                  0));
        //              cv::imshow("out", imageCopy);
        //              cv::waitKey(0);
        //          }

        object_pt_ids = charuco_ids;
        for (const auto& c : charuco_corners) {
          corners.push_back(Eigen::Vector2d(c.x, c.y));
        }
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }

  } else if (board_type_ == BoardType::RADON) {
    std::vector<Point2d> radon_corners;
    cv::Mat meta;
    bool success = cv::findChessboardCornersSB(
        image, radon_pattern_size_, radon_corners, radon_flags_, meta);
    if (!success) {
      return false;
    }
    int lf = 0;
    for (int i = 0; i < radon_pattern_size_.height; ++i) {
      for (int j = 0; j < radon_pattern_size_.width; ++j) {
        if ((int)meta.at<uchar>(i, j) >= 0) {
          object_pt_ids.push_back(continuous_board_indices_[lf]);
        }
        lf++;
      }
    }
    for (const auto& c : radon_corners) {
      corners.push_back(Eigen::Vector2d(c.x, c.y));
    }
  } else if (board_type_ == BoardType::APRILTAG) {
    std::vector<int> marker_ids, rejected_ids;
    std::vector<double> radii, rejected_radii;
    std::vector<Point2f> marker_corners, rejected_markers;
    april_detector_.detectTags(image,
                               marker_corners,
                               marker_ids,
                               radii,
                               rejected_markers,
                               rejected_ids,
                               rejected_radii);
    object_pt_ids = marker_ids;
    for (const auto& c : marker_corners) {
      corners.push_back(Eigen::Vector2d(c.x, c.y));
    }
  } else {
    LOG(WARNING) << "Board type does not exist.";
    return false;
  }

  return true;
}

void BoardExtractor::BoardToJson(nlohmann::json& output_json) {
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
  } else if (board_type_ == BoardType::APRILTAG) {
    for (size_t i = 0; i < board_pts.size(); ++i) {
      output_json["scene_pts"][std::to_string(i)] = {
          board_pts[i].x, board_pts[i].y, board_pts[i].z};
    }
  } else {
    LOG(ERROR) << "Board type does not exist.";
  }
}

bool BoardExtractor::ExtractImageFolderToJson(
    const std::string& image_folder,
    const std::string& save_path,
    const double img_downsample_factor,
    const std::string img_file_ext) {
  if (!board_initialized_) {
    LOG(ERROR) << "No board initialized.\n";
    return false;
  }
  if (image_folder == "") {
    LOG(ERROR) << "Image file path is empty.\n";
    return false;
  }

  // get filenames
  std::vector<std::string> filenames;
  cv::glob(image_folder + "/*."+img_file_ext, filenames, false);
  std::sort(filenames.begin(), filenames.end());

  if (filenames.size() <= 0) {
    LOG(ERROR)
        << "No image files found in folder. Must be timestamp_in_ns."<<img_file_ext<<"!";
    return false;
  }

  nlohmann::json output_json;

  output_json["calibration_board_type"] = board_type_;
  output_json["square_size_meter"] = square_length_m_;
  BoardToJson(output_json);

  const size_t total_nr_frames = filenames.size();
  std::cout << "Total number of frames: " << total_nr_frames << "\n";
  int frame_cnt = 0;
  bool set_img_size = false;
  std::set<double> timestamps_s;
  for (size_t i = 0; i < total_nr_frames; ++i) {
    const std::string image_path = filenames[i];

    // get timestamp in nanoseconds
    std::size_t slash = image_path.find_last_of("/\\");
    std::size_t ending = image_path.find_last_of(".");

    int64_t timestamp_ns = std::stoul(image_path.substr(slash + 1, ending));
    Mat image = cv::imread(image_path);
    const double timestamp_s = timestamp_ns * NS_TO_S;
    timestamps_s.insert(timestamp_s);
    const std::string view_us = std::to_string(timestamp_s * S_TO_US);
    ++frame_cnt;

    const double fxfy = 1. / img_downsample_factor;
    cv::resize(image, image, cv::Size(), fxfy, fxfy);

    aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> ids;
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    if (!ExtractBoard(image, corners, ids)) {
        continue;
    }

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
      cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      for (size_t i = 0; i < corners.size(); ++i) {
        cv::drawMarker(
            image,
            cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
            cv::Scalar(0, 0, 255),
            cv::MARKER_CROSS,
            10,
            3);

        cv::putText(image,
                    std::to_string(ids[i]),
                    cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
                    cv::FONT_HERSHEY_PLAIN,
                    1,
                    cv::Scalar(0, 0, 255));
        cv::putText(image,
                    "Number corners: " + std::to_string(corners.size()),
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL,
                    2,
                    cv::Scalar(0, 0, 255));
        cv::imshow("corners", image);
        cv::waitKey(1);
      }
    }
  }
  std::vector<double> times, delta_ts;
  for (const auto& t : timestamps_s) {
    times.push_back(t);
  }
  for (size_t i = 0; i < times.size() - 2; ++i) {
    delta_ts.push_back(times[i + 1] - times[i]);
  }

  output_json["camera_fps"] = 1. / utils::MedianOfDoubleVec(delta_ts);

  std::vector<std::uint8_t> v_bson = nlohmann::json::to_ubjson(output_json);
  std::ofstream calib_txt_output(save_path, std::ios::out | std::ios::binary);
  calib_txt_output.write(reinterpret_cast<const char*>(&v_bson[0]),
                         v_bson.size() * sizeof(std::uint8_t));

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
  const double fps = input_video.get(cv::CAP_PROP_FPS);

  output_json["camera_fps"] = fps;
  output_json["calibration_board_type"] = board_type_;
  output_json["square_size_meter"] = square_length_m_;

  BoardToJson(output_json);

  const int total_nr_frames = input_video.get(cv::CAP_PROP_FRAME_COUNT);
  std::cout << "Total number of frames: " << total_nr_frames << "\n";
  int frame_cnt = 0;
  bool set_img_size = false;
  while (true) {
    Mat image;
    if (!input_video.read(image)) {
      cnt_wrong++;
      if (cnt_wrong > 500) break;
      continue;
    }

    const double timstamp_s = input_video.get(cv::CAP_PROP_POS_MSEC) * 1e-3;
    const std::string view_us = std::to_string(timstamp_s * S_TO_US);
    ++frame_cnt;

    const double fxfy = 1. / img_downsample_factor;
    cv::resize(image, image, cv::Size(), fxfy, fxfy);

    aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> ids;
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    ExtractBoard(image, corners, ids);
    
    std::cout<<"Extracted "<<ids.size()<<" corners\n";
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
      cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      for (size_t i = 0; i < corners.size(); ++i) {
        cv::drawMarker(
            image,
            cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
            cv::Scalar(0, 0, 255),
            cv::MARKER_CROSS,
            10,
            3);

        cv::putText(image,
                    std::to_string(ids[i]),
                    cv::Point(cvRound(corners[i][0]), cvRound(corners[i][1])),
                    cv::FONT_HERSHEY_PLAIN,
                    1,
                    cv::Scalar(0, 0, 255));
      }
      cv::putText(image,
                  "Number corners: " + std::to_string(corners.size()),
                  cv::Point(10, 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  2,
                  cv::Scalar(0, 0, 255));
      //cv::imshow("corners", image);
      cv::imwrite("/mnt/c/Users/zosurban/Downloads/TestCalib/corners"+std::to_string(frame_cnt)+".jpg", image);
      //cv::waitKey(1);
    }
  }

  std::vector<std::uint8_t> v_bson = nlohmann::json::to_ubjson(output_json);

  std::ofstream calib_txt_output(save_path, std::ios::out | std::ios::binary);
  calib_txt_output.write(reinterpret_cast<const char*>(&v_bson[0]),
                         v_bson.size() * sizeof(std::uint8_t));

  return true;
}

}  // namespace core
}  // namespace OpenICC
