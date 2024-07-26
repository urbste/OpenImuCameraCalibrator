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
#include "theia/sfm/pose/orthographic_four_point.h"

#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>

#include <opencv2/core/eigen.hpp>

#include <unsupported/Eigen/Polynomials>


const size_t MIN_NUM_POINTS = 12;

namespace OpenICC {
namespace utils {



bool initialize_pinhole_camera(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const theia::RansacParameters& ransac_params,
    theia::RansacSummary& ransac_summary,
    Eigen::Matrix3d& rotation,
    Eigen::Vector3d& position,
    double& focal_length,
    const bool verbose) {
  if (correspondences.size() <= MIN_NUM_POINTS) {
    return false;
  }

  theia::UncalibratedAbsolutePose pose_linear;
  // use -> cv::initCameraMatrix2D()
  const bool success =
      theia::EstimateUncalibratedAbsolutePose(ransac_params,
                                              theia::RansacType::RANSAC,
                                              correspondences,
                                              &pose_linear,
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
  if (ransac_summary.inliers.size() < MIN_NUM_POINTS) return false;
  return success;
}

bool initialize_radial_undistortion_camera(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const theia::RansacParameters& ransac_params,
    theia::RansacSummary& ransac_summary,
    const cv::Size& img_size,
    Eigen::Matrix3d& rotation,
    Eigen::Vector3d& position,
    double& focal_length,
    double& radial_distortion,
    const bool verbose) {
  if (correspondences.size() <= MIN_NUM_POINTS) {
    return false;
  }
  theia::RadialDistUncalibratedAbsolutePoseMetaData meta_data;
  theia::RadialDistUncalibratedAbsolutePose pose_division_undist;
  meta_data.max_focal_length = 0.75 * img_size.width + 0.5 * img_size.width;
  meta_data.min_focal_length = 0.75 * img_size.width - 0.5 * img_size.width;

  const bool success = theia::EstimateRadialDistUncalibratedAbsolutePose(
      ransac_params,
      theia::RansacType::RANSAC,
      correspondences,
      meta_data,
      &pose_division_undist,
      &ransac_summary);

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
  intrinsics->SetPrincipalPoint(0.0, 0.0);
  intrinsics->SetParameter(theia::DivisionUndistortionCameraModel::
                               InternalParametersIndex::RADIAL_DISTORTION_1,
                           radial_distortion);
  cam.SetPosition(position);
  cam.SetOrientationFromRotationMatrix(rotation);
  // calculate reprojection error
  double repro_error = 0.0;
  for (size_t i = 0; i < correspondences.size(); ++i) {
    Eigen::Vector2d pixel;
    cam.ProjectPoint(correspondences[i].world_point.homogeneous(), &pixel);
    repro_error += (pixel - correspondences[i].feature).norm();
  }
  repro_error /= (double)correspondences.size();
  if (verbose) {
    std::cout << "Estimated focal length: " << pose_division_undist.focal_length
              << std::endl;
    std::cout << "Estimated radial distortion: "
              << pose_division_undist.radial_distortion << std::endl;
    std::cout << "Number of Ransac inliers: " << ransac_summary.inliers.size()
              << std::endl;
    std::cout << "Reprojection error: " << repro_error << std::endl
              << std::endl;
  }
  if (ransac_summary.inliers.size() < MIN_NUM_POINTS || repro_error > 4.0)
    return false;
  return success;
}

// took that initialization from basalt
// https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/src/calibration/calibraiton_helper.cpp
bool initialize_doublesphere_model(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const std::vector<int> board_ids,
    const cv::Size& board_size,
    const theia::RansacParameters& ransac_params,
    const int img_cols,
    const int img_rows,
    theia::RansacSummary& ransac_summary,
    Eigen::Matrix3d& rotation,
    Eigen::Vector3d& position,
    double& focal_length,
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

  for (size_t r = 0; r < target_rows; ++r) {
    aligned_vector<Eigen::Vector4d> P;

    for (size_t c = 0; c < target_cols; ++c) {
      int corner_id = (r * target_cols + c);

      if (id_to_corner.find(corner_id) != id_to_corner.end()) {
        const Eigen::Vector2d imagePoint = id_to_corner[corner_id];
        P.emplace_back(imagePoint[0],
                       imagePoint[1],
                       0.5,
                       -0.5 * (imagePoint[0] * imagePoint[0] +
                               imagePoint[1] * imagePoint[1]));
      }
    }
    const int MIN_CORNERS = 8;
    // MIN_CORNERS is an arbitrary threshold for the number of corners
    if (P.size() > MIN_CORNERS) {
      // Resize P to fit with the count of valid points.

      Eigen::Map<Eigen::Matrix4Xd> P_mat((double*)P.data(), 4, P.size());

      // std::cerr << "P_mat\n" << P_mat.transpose() << std::endl;

      Eigen::MatrixXd P_mat_t = P_mat.transpose();

      Eigen::JacobiSVD<Eigen::MatrixXd> svd(
          P_mat_t, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

      for (auto& cor : correspondences_new) {
        cor.feature =
            cam.PixelToNormalizedCoordinates(cor.feature).hnormalized();
      }

      theia::CalibratedAbsolutePose pose;
      theia::RansacParameters params = ransac_params;
      params.error_thresh = 0.5 / img_cols;
      theia::PnPType type = theia::PnPType::DLS;
      theia::EstimateCalibratedAbsolutePose(params,
                                            theia::RansacType::RANSAC,
                                            type,
                                            correspondences_new,
                                            &pose,
                                            &ransac_summary);

      cam.SetPosition(pose.position);
      cam.SetOrientationFromRotationMatrix(pose.rotation);

      double repro_error = 0.0;
      int in_image = 0;
      for (size_t i = 0; i < correspondences.size(); ++i) {
        Eigen::Vector2d pixel;
        cam.ProjectPoint(correspondences[i].world_point.homogeneous(), &pixel);
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

        if (avg_reproj_error < min_reproj_error && avg_reproj_error < 5.0 &&
            ransac_summary.inliers.size() > 10) {
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
        }  // if clause
      }    // if in_image
    }      // if P.size()
  }        // for target cols
  return success;
}  // for target rows


// cf. https://github.com/Seagate/telecentric-calibration/blob/main/src/estimate_params.m
bool initialize_orthographic_camera_model(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& p,
    Eigen::Matrix3d& H,
    double& focal_length,
    const Eigen::Vector2d& principal_pt,
    const bool verbose) {
      
    if (correspondences.size() <= MIN_NUM_POINTS) {
      return false;
    }

    Eigen::MatrixXd pts2d;
    Eigen::MatrixXd pts3d;

    const size_t num_pts = correspondences.size();
    pts2d.resize(2*num_pts,1);
    pts3d.resize(2*num_pts,6);
    pts2d.setZero();
    pts3d.setZero();

    for (size_t i = 0; i < num_pts; ++i) {
        // add principal point because it was substracted for other cam models
        pts2d(i,0) = correspondences[i].feature[0] + principal_pt[0];
        pts2d(i+num_pts,0) = correspondences[i].feature[1] + principal_pt[1];
        Eigen::Vector3d world_pt = correspondences[i].world_point;
        world_pt[2] = 1.0;
        pts3d.block<1,3>(i,0) = world_pt;
        pts3d.block<1,3>(i+num_pts,3) = world_pt;
    }

    Eigen::MatrixXd h = pseudoInverse(pts3d) * pts2d;
    // ortho homography
    H << h(0,0), h(1,0), h(2,0),
         h(3,0), h(4,0), h(5,0),
         0, 0, 1.;

    // get magnification of orthographic camera
    const double c1 = h(0,0)*h(0,0) + h(1,0)*h(1,0) + h(3,0)*h(3,0) + h(4,0)*h(4,0);
    const double c2 = std::pow((h(0,0)*h(4,0) - h(1,0)*h(3,0)),2);
    Eigen::Matrix<double, 5, 1> coeff;
    // order of coeffs are swapped in Eigen compared to numpy and Matlab
    coeff << c2, 0., -c1, 0., 1;
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    bool has_real_root = false;
    const double magnification = solver.greatestRealRoot(has_real_root);
    if (!has_real_root || std::isinf(magnification) ||std::isnan(magnification) || magnification <= 0.0) {
        LOG(INFO) << "Magnification could not be estimated for this view!";
        return false;
    }

    Eigen::Matrix3d K_init;
    K_init << magnification, 0, principal_pt[0],
              0., magnification, principal_pt[1],
              0., 0., 1.;
    // focal length in this case is basically f = magnification / pixel_pitch
    focal_length = magnification;

    const Eigen::MatrixXd E = pseudoInverseSquare(K_init) * H;
    // get rotation
    const double term1 = std::pow(E(0,0),2) + std::pow(E(1,0),2);
    const double term2 = std::pow(E(0,1),2) + std::pow(E(1,1),2);
    // Avoid getting imaginary values in sqrt
    if (term1 > 1.0 || term2 > 1.0) {
        return false;
    }
    const double r13 = std::sqrt(1.0 - term1);
    const double r23 = std::sqrt(1.0 - term2);
    const Eigen::Vector3d r1(E(0,0), E(1,0), r13);
    const Eigen::Vector3d r2(E(0,1), E(1,1), r23);
    const Eigen::Vector3d r3 = r1.cross(r2);

    Eigen::Matrix3d Rtmp;
    Rtmp.col(0) = r1;
    Rtmp.col(1) = r2;
    Rtmp.col(2) = r3;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(Rtmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();

    // return translation
    Eigen::Vector3d t(
      (H(0,2) - principal_pt[0]) / magnification,
      (H(1,2) - principal_pt[1]) / magnification,
      0.0);
    p = -R.transpose()*t;

    if (std::abs(R.determinant() - 1.) > 1e-5) {
        LOG(INFO) << "Rotation matrix is not orthogonal!";
        return false;
    }
    if (verbose) {
      VLOG(1) << "Estimated magnification: " << magnification
                << std::endl;
    }

    // check reprojection error
    double repro_error = 0.0;
    Eigen::Matrix<double,3,4> T_w_c = Eigen::Matrix<double,3,4>::Identity();
    T_w_c.block<3,3>(0,0) = R;
    T_w_c.block<3,1>(0,3) = t;
    T_w_c(2,3) = 0.;

    for (size_t i = 0; i < correspondences.size(); ++i) {
      Eigen::Vector3d pt_in_cam = T_w_c * correspondences[i].world_point.homogeneous();

      repro_error += (pt_in_cam.head<2>()*magnification - 
        (correspondences[i].feature)).norm();
      
    }
    repro_error /= (double)correspondences.size();

    if (repro_error > 20.) {
      VLOG(1) << "Reprojection error too large: " << repro_error
              << std::endl;
      return false;
    }

    return true;
}

// cf. https://github.com/Seagate/telecentric-calibration/blob/main/src/estimate_params.m
bool initialize_orthographic_focal_length(
    const aligned_vector<Eigen::Matrix3d>& ortho_homographies,
    double& alpha,
    double& beta) {
    Eigen::MatrixXd G, w;
    const size_t num_calib_views = ortho_homographies.size();
    G.resize(num_calib_views, 4);
    G.setZero();
    G.col(0).setOnes();
    w.resize(num_calib_views, 1);
    w.setZero();
    for (size_t i=0; i < num_calib_views; ++i) {
        const auto H = ortho_homographies[i];
        G(i,1) = -std::pow(H(1,0),2) - std::pow(H(1,1),2);
        G(i,2) = -std::pow(H(0,0),2) - std::pow(H(0,1),2);
        G(i,3) = 2.0 * (H(0,0)*H(1,0) + H(0,1)*H(1,1));
        w(i)   = -std::pow(H(0,0)*H(1,1) - H(0,1)* H(1,0),2);
    }
    Eigen::MatrixXd l = pseudoInverse(G) * w;
    alpha = std::sqrt( (l(1)*l(2)-l(3)*l(3)) / l(2));
    beta = std::sqrt(l(2));

    return true;
}
}  // namespace utils
}  // namespace OpenICC
