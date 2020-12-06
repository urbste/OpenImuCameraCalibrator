// created by Steffen Urban November 2019
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/utils/types.h"

#include <algorithm>
#include <dirent.h>
#include <vector>

namespace OpenCamCalib {
namespace utils {

enum BoardType { CHARUCO = 0, RADON = 1 };

inline BoardType StringToBoardType(const std::string &board_type) {
  if (board_type == "charuco") {
    return BoardType::CHARUCO;
  } else if (board_type == "radon") {
    return BoardType::RADON;
  }
}

class BoardExtractor {
public:
  BoardExtractor();

  bool ExtractBoard(const cv::Mat &image,
                    aligned_vector<Eigen::Vector2d> &corners,
                    std::vector<int> &object_pt_ids);

  bool InitializeCharucoBoard(std::string path_to_detector_params,
                              float marker_length, float square_length,
                              int squaresX, int squaresY, int dictionaryId);

  bool InitializeRadonBoard(int square_length, int squaresX, int squaresY);

  std::vector<std::vector<cv::Point3f>> GetBoardPts() { return board_pts3d_; }

private:
  BoardType board_type_;

  std::vector<std::vector<cv::Point3f>> board_pts3d_;

  // Charco parameters
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> charucoboard_;
  cv::Ptr<cv::aruco::Board> board_;

  // Radon board parameters
  int radon_flags_;
  cv::Size radon_pattern_size_;
};

} // namespace utils
} // namespace OpenCamCalib
