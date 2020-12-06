
#include "OpenCameraCalibrator/utils/board_extractor.h"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <algorithm>
#include <vector>

#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

namespace OpenCamCalib {
namespace utils {

BoardExtractor::BoardExtractor() {}

bool BoardExtractor::InitializeCharucoBoard(std::string path_to_detector_params,
                                            float marker_length, float square_length,
                                            int squaresX, int squaresY,
                                            int dictionaryId) {
  // load images from folder
  detector_params_ = aruco::DetectorParameters::create();

  if (!OpenCamCalib::utils::ReadDetectorParameters(path_to_detector_params,
                                                   detector_params_)) {
    std::cerr << "Invalid detector parameters file\n";
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
  }
}

} // namespace utils
} // namespace OpenCamCalib
