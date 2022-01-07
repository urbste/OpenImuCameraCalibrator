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

#include <opencv2/aruco/charuco.hpp>

#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/utils.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include <theia/sfm/pose/four_point_focal_length_radial_distortion.h>

#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>

#include <opencv2/core/eigen.hpp>

const size_t MIN_NUM_POINTS = 20;

namespace OpenICC {
namespace utils {

bool initialize_pinhole_camera(
    const std::vector<theia::FeatureCorrespondence2D3D> &correspondences,
    const theia::RansacParameters &ransac_params,
    theia::RansacSummary &ransac_summary, Eigen::Matrix3d &rotation,
    Eigen::Vector3d &position, double &focal_length, const bool verbose) {

  if (correspondences.size() <= MIN_NUM_POINTS) {
    return false;
  }

  theia::UncalibratedAbsolutePose pose_linear;
  // use -> cv::initCameraMatrix2D()
  const bool success = theia::EstimateUncalibratedAbsolutePose(
      ransac_params, theia::RansacType::RANSAC, correspondences, &pose_linear,
      &ransac_summary);
  if (verbose) {
    std::cout << "Estimated focal length: " << pose_linear.focal_length
              << std::endl;
    std::cout << "Number of Ransac inliers: " << ransac_summary.inliers.size()
              << std::endl;
  }
  rotation = pose_linear.rotation;
  position = pose_linear.position;
  focal_length = pose_linear.focal_length;
  if (ransac_summary.inliers.size() < MIN_NUM_POINTS)
    return false;
  return success;
}

bool initialize_radial_undistortion_camera(
    const std::vector<theia::FeatureCorrespondence2D3D> &correspondences,
    const theia::RansacParameters &ransac_params,
    theia::RansacSummary &ransac_summary, const cv::Size &img_size,
    Eigen::Matrix3d &rotation, Eigen::Vector3d &position, double &focal_length,
    double &radial_distortion, const bool verbose) {

  if (correspondences.size() <= MIN_NUM_POINTS) {
    return false;
  }
  theia::RadialDistUncalibratedAbsolutePoseMetaData meta_data;
  theia::RadialDistUncalibratedAbsolutePose pose_division_undist;
  meta_data.max_focal_length = 0.75 * img_size.width + 0.5 * img_size.width;
  meta_data.min_focal_length = 0.75 * img_size.width - 0.5 * img_size.width;

  const bool success = theia::EstimateRadialDistUncalibratedAbsolutePose(
      ransac_params, theia::RansacType::RANSAC, correspondences, meta_data,
      &pose_division_undist, &ransac_summary);

  rotation = pose_division_undist.rotation;
  position = -pose_division_undist.rotation.transpose() *
             pose_division_undist.translation;
  radial_distortion = pose_division_undist.radial_distortion;
  focal_length = pose_division_undist.focal_length;

  theia::Camera cam;
  cam.SetCameraIntrinsicsModelType(
      theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
  std::shared_ptr<theia::CameraIntrinsicsModel> intrinsics =
      cam.MutableCameraIntrinsics();
  intrinsics->SetFocalLength(focal_length);
  intrinsics->SetPrincipalPoint(0.0,0.0);
  intrinsics->SetParameter(
      theia::DivisionUndistortionCameraModel::InternalParametersIndex::RADIAL_DISTORTION_1,
      radial_distortion);
  cam.SetPosition(position);
  cam.SetOrientationFromRotationMatrix(rotation);
  // calculate reprojection error
  double repro_error = 0.0;
  for (int i = 0; i < correspondences.size(); ++i) {
    Eigen::Vector2d pixel;
    cam.ProjectPoint(correspondences[i].world_point.homogeneous(),
                     &pixel);
      repro_error += (pixel - correspondences[i].feature).norm();
  }
  repro_error /= (double)correspondences.size();
  if (verbose) {
    std::cout << "Estimated focal length: " << pose_division_undist.focal_length
              << std::endl;
    std::cout << "Estimated radial distortion: "
              << pose_division_undist.radial_distortion
              << std::endl;
    std::cout << "Number of Ransac inliers: " << ransac_summary.inliers.size()
              << std::endl;
    std::cout << "Reprojection error: " << repro_error
              << std::endl<< std::endl;
  }
  if (ransac_summary.inliers.size() < MIN_NUM_POINTS || repro_error > 4.0)
    return false;
  return success;
}

// took that initialization from basalt
// https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/src/calibration/calibraiton_helper.cpp
bool initialize_doublesphere_model(
    const std::vector<theia::FeatureCorrespondence2D3D> &correspondences,
    const std::vector<int> board_ids, const cv::Size &board_size,
    const theia::RansacParameters &ransac_params, const int img_cols,
    const int img_rows, theia::RansacSummary &ransac_summary,
    Eigen::Matrix3d &rotation, Eigen::Vector3d &position, double &focal_length,
    const bool verbose) {
  // First, initialize the image center at the center of the image.

  aligned_map<int, Eigen::Vector2d> id_to_corner;
  for (size_t i = 0; i < board_ids.size(); i++) {
    id_to_corner[board_ids[i]] = correspondences[i].feature;
  }

  const double _xi = 1.0;

  // Initialize some temporaries needed.
  double gamma0 = 0.0;
  double min_reproj_error = std::numeric_limits<double>::max();
  bool success = false;

  // Now we try to find a non-radial line to initialize the focal length
  const size_t target_cols = board_size.width;
  const size_t target_rows = board_size.height;

  for (int r = 0; r < target_rows; ++r) {
    aligned_vector<Eigen::Vector4d> P;

    for (int c = 0; c < target_cols; ++c) {
      int corner_id = (r * target_cols + c);

      if (id_to_corner.find(corner_id) != id_to_corner.end()) {
        const Eigen::Vector2d imagePoint = id_to_corner[corner_id];
        P.emplace_back(imagePoint[0], imagePoint[1], 0.5,
                       -0.5 * (imagePoint[0] * imagePoint[0] +
                               imagePoint[1] * imagePoint[1]));
      }
    }
    const int MIN_CORNERS = 8;
    // MIN_CORNERS is an arbitrary threshold for the number of corners
    if (P.size() > MIN_CORNERS) {
      // Resize P to fit with the count of valid points.

      Eigen::Map<Eigen::Matrix4Xd> P_mat((double *)P.data(), 4, P.size());

      // std::cerr << "P_mat\n" << P_mat.transpose() << std::endl;

      Eigen::MatrixXd P_mat_t = P_mat.transpose();

      Eigen::JacobiSVD<Eigen::MatrixXd> svd(P_mat_t, Eigen::ComputeThinU |
                                                         Eigen::ComputeThinV);

      // std::cerr << "U\n" << svd.matrixU() << std::endl;
      // std::cerr << "V\n" << svd.matrixV() << std::endl;
      // std::cerr << "singularValues\n" << svd.singularValues() <<
      // std::endl;

      Eigen::Vector4d C = svd.matrixV().col(3);
      // std::cerr << "C\n" << C.transpose() << std::endl;
      // std::cerr << "P*res\n" << P_mat.transpose() * C << std::endl;

      double t = C(0) * C(0) + C(1) * C(1) + C(2) * C(3);
      if (t < 0) {
        continue;
      }

      // check that line image is not radial
      double d = sqrt(1.0 / t);
      double nx = C(0) * d;
      double ny = C(1) * d;
      if (hypot(nx, ny) > 0.95) {
        // std::cerr << "hypot(nx, ny) " << hypot(nx, ny) << std::endl;
        continue;
      }

      double nz = sqrt(1.0 - nx * nx - ny * ny);
      double gamma = fabs(C(2) * d / nz);

      // undistort points with intrinsic guess
      theia::Camera cam;

      cam.SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
      std::shared_ptr<theia::CameraIntrinsicsModel> intrinsics =
          cam.MutableCameraIntrinsics();
      intrinsics->SetFocalLength(0.5 * gamma);
      intrinsics->SetPrincipalPoint(0.0, 0.0);
      intrinsics->SetParameter(
          theia::DoubleSphereCameraModel::InternalParametersIndex::XI,
          0.5 * _xi);
      intrinsics->SetParameter(
          theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA, 0.0);

      std::vector<theia::FeatureCorrespondence2D3D> correspondences_new =
          correspondences;

      for (auto &cor : correspondences_new) {
        cor.feature =
            cam.PixelToNormalizedCoordinates(cor.feature).hnormalized();
      }

      theia::CalibratedAbsolutePose pose;
      theia::RansacParameters params = ransac_params;
      params.error_thresh = 0.5 / img_cols;
      theia::PnPType pnpr = theia::PnPType::DLS;
      theia::EstimateCalibratedAbsolutePose(
          params, theia::RansacType::RANSAC, pnpr, correspondences_new, &pose,
          &ransac_summary);
      cam.SetPosition(pose.position);
      cam.SetOrientationFromRotationMatrix(pose.rotation);

      double repro_error = 0.0;
      int in_image = 0;
      for (int i = 0; i < correspondences.size(); ++i) {
        Eigen::Vector2d pixel;
        cam.ProjectPoint(correspondences[i].world_point.homogeneous(),
                         &pixel);
        if (pixel(0) >= 0.0 && pixel(1) >= 0.0 && pixel(0) < img_cols &&
            pixel(1) < img_rows) {
          repro_error += (pixel - correspondences[i].feature).norm();
          in_image++;
        }
      }
//      if (verbose) {
//        std::cout << "numReprojected " << in_image << " reprojErr "
//                  << repro_error / in_image << std::endl;
//      }
      if (in_image > MIN_CORNERS) {
        double avg_reproj_error = repro_error / in_image;

        if (avg_reproj_error < min_reproj_error && avg_reproj_error < 5.0 && ransac_summary.inliers.size() > 10) {
          min_reproj_error = avg_reproj_error;
          gamma0 = gamma;
          success = true;
          rotation = pose.rotation;
          position = pose.position;
          focal_length = 0.5 * gamma0;
          if (verbose) {
            std::cout << "new min_reproj_error: " << min_reproj_error
                      << std::endl;
          }
          return success;
        } // if clause
      }   // if in_image
    }     // if P.size()
  }       // for target cols
  return success;
} // for target rows

} // namespace utils
} // namespace OpenICC
