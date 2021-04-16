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

#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/utils/types.h"

#include <algorithm>
#include <dirent.h>
#include <vector>

namespace OpenICC {
namespace core {

enum BoardType { CHARUCO = 0, RADON = 1 };

inline BoardType StringToBoardType(const std::string &board_type) {
  if (board_type == "charuco") {
    return BoardType::CHARUCO;
  } else if (board_type == "radon") {
    return BoardType::RADON;
  }
  return BoardType::CHARUCO;
}

class BoardExtractor {
public:
  BoardExtractor();

  //! Extracts an initialized board type from an image
  bool ExtractBoard(const cv::Mat &image,
                    aligned_vector<Eigen::Vector2d> &corners,
                    std::vector<int> &object_pt_ids);

  //! Extracts a board from a video file to a json file and saves it to disk
  bool ExtractVideoToJson(const std::string& video_path,
                          const std::string& save_path,
                          const double img_downsample_factor);

  //! Initializes a Charuco board
  bool InitializeCharucoBoard(std::string path_to_detector_params,
                              float marker_length, float square_length,
                              int squaresX, int squaresY, int dictionaryId);

  //! Initializes a Radon checkerboard
  bool InitializeRadonBoard(float square_length, int squaresX, int squaresY);

  //! Returns the 3d board points
  std::vector<std::vector<cv::Point3f>> GetBoardPts() { return board_pts3d_; }

  std::vector<int> GetRadonBoardIDs() { return radon_board_indices_; }

  //! Set verbose plot
  void SetVerbosePlot() { verbose_plot_ = true; }

private:
  //! Board type
  BoardType board_type_;

  //! 3D Board points
  std::vector<std::vector<cv::Point3f>> board_pts3d_;

  //! Aruco board detector parameters
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  //! Aruco board dictionary
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  //! Charuco board
  cv::Ptr<cv::aruco::CharucoBoard> charucoboard_;
  //! Aruco board
  cv::Ptr<cv::aruco::Board> board_;

  //! radon board extraction flags
  int radon_flags_;
  //! radon board size
  cv::Size radon_pattern_size_;
  //! radon board pt continuous index
  std::vector<int> radon_board_indices_;

  //! if a board is already initialized
  bool board_initialized_ = false;

  //! square size in meter
  double square_length_m_;

  //! display extracted corners
  bool verbose_plot_ = false;
};

}
}
