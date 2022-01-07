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

#include "OpenCameraCalibrator/core/pose_estimator.h"

#include "OpenCameraCalibrator/io/read_scene.h"

#include <theia/io/reconstruction_reader.h>
#include <theia/io/reconstruction_writer.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/extended_unified_camera_model.h>
#include <theia/sfm/camera/fisheye_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/camera/pinhole_radial_tangential_camera_model.h>

#include <thread>

namespace OpenICC {
namespace core {

PoseEstimator::PoseEstimator() {
  ransac_params_.failure_probability = 0.001;
  ransac_params_.use_mle = true;
  ransac_params_.max_iterations = 1000;
  ransac_params_.min_iterations = 10;
  ransac_params_.error_thresh = 0.01;

  // bundle adjustment options
  ba_options_.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options_.robust_loss_width = 1.345;
  ba_options_.intrinsics_to_optimize = theia::OptimizeIntrinsicsType::NONE;
}

bool PoseEstimator::EstimatePosePinhole(
    const theia::ViewId& view_id,
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences_undist,
    const std::vector<int>& board_pts3_ids) {
  // Estimate camera pose using
  theia::CalibratedAbsolutePose pose;
  theia::RansacSummary ransac_summary;
  theia::EstimateCalibratedAbsolutePose(ransac_params_,
                                        theia::RansacType::RANSAC,
                                        pnp_type_,
                                        correspondences_undist,
                                        &pose,
                                        &ransac_summary);

  if (ransac_summary.inliers.size() < min_num_points_) {
    return false;
  }

  theia::View* theia_view = pose_dataset_.MutableView(view_id);
  theia_view->SetEstimated(true);

  theia::Camera* cam = theia_view->MutableCamera();
  cam->SetPosition(pose.position);
  cam->SetOrientationFromRotationMatrix(pose.rotation);

  for (size_t i = 0; i < ransac_summary.inliers.size(); ++i) {
    int inlier = ransac_summary.inliers[i];

    pose_dataset_.AddObservation(
        view_id,
        board_pts3_ids[inlier],
        theia::Feature(correspondences_undist[inlier].feature));
  }

  // optimize pose
  theia::BundleAdjustmentSummary summary =
      theia::BundleAdjustView(ba_options_, view_id, &pose_dataset_);

  return summary.success;
}

bool PoseEstimator::EstimatePosesFromJson(const nlohmann::json& scene_json,
                                          const theia::Camera camera,
                                          const double max_reproj_error) {
  const double image_diag =
      std::sqrt(camera.ImageWidth() * camera.ImageWidth() +
                camera.ImageHeight() * camera.ImageHeight());
  ransac_params_.error_thresh = max_reproj_error / image_diag;
  // get scene points and fill them into
  io::scene_points_to_calib_dataset(scene_json, pose_dataset_);
  // init the number of observations per point to zero.
  // It might happen, that some point are not seen at all and
  // then BA will crash as no observations are available
  for (const auto t_id : pose_dataset_.TrackIds()) {
    tracks_to_nr_obs_[t_id] = 0;
  }

  const auto views = scene_json["views"];
  double total_repro_error = 0.0;
  int processed_frames = 0;

  for (const auto& view : views.items()) {
    const double timestamp_us = std::stod(view.key());
    const double timestamp_s = timestamp_us * US_TO_S;  // to seconds
    const auto image_points = view.value()["image_points"];
    std::vector<int> board_pts3_ids;
    aligned_vector<Eigen::Vector2d> corners;
    std::vector<theia::FeatureCorrespondence2D3D> correspondences_undist;

    for (const auto& img_pts : image_points.items()) {
      const int board_pt3_id = std::stoi(img_pts.key());
      board_pts3_ids.push_back(std::stoi(img_pts.key()));
      const Eigen::Vector2d corner(
          Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
      corners.push_back(corner);
      Eigen::Vector3d undist_pt = camera.PixelToNormalizedCoordinates(corner);
      undist_pt /= undist_pt[2];

      const Eigen::Vector4d track = pose_dataset_.Track(board_pt3_id)->Point();

      theia::FeatureCorrespondence2D3D corr_undist;
      corr_undist.world_point = track.hnormalized();
      corr_undist.feature[0] = undist_pt[0];
      corr_undist.feature[1] = undist_pt[1];
      correspondences_undist.push_back(corr_undist);
    }
    if (correspondences_undist.size() < min_num_points_) {
      LOG(INFO) << "Skipping view at timestamp : " << timestamp_s
                << "s. Not enough points found.";
      continue;
    }
    std::string view_name = std::to_string((uint64_t)(timestamp_s * S_TO_US));
    theia::ViewId view_id = pose_dataset_.AddView(view_name, 0, timestamp_s);

    theia::Camera* cam = pose_dataset_.MutableView(view_id)->MutableCamera();
    cam->SetCameraIntrinsicsModelType(
        theia::CameraIntrinsicsModelType::PINHOLE);
    cam->SetFocalLength(1.0);
    cam->SetPrincipalPoint(0.0, 0.0);
    cam->SetImageSize(1.0, 1.0);
    if (!EstimatePosePinhole(view_id, correspondences_undist, board_pts3_ids)) {
      LOG(INFO) << "Pose estimation failed for view at timestamp "
                << timestamp_s << "s from " << correspondences_undist.size()
                << " points. Max reproj error was: "
                << ransac_params_.error_thresh;
      pose_dataset_.RemoveView(view_id);
      continue;
    }
    // test back projection
    double reproj_error = 0;
    for (size_t i = 0; i < pose_dataset_.View(view_id)->TrackIds().size();
         ++i) {
      theia::TrackId track_id = pose_dataset_.View(view_id)->TrackIds()[i];
      tracks_to_nr_obs_[track_id] += 1;

      const theia::Track* track = pose_dataset_.Track(track_id);
      Eigen::Vector2d reproj_point;
      pose_dataset_.View(view_id)->Camera().ProjectPoint(track->Point(),
                                                         &reproj_point);

      reproj_error +=
          (reproj_point -
           (*pose_dataset_.View(view_id)->GetFeature(track_id)).point_)
              .norm();
    }
    const double repro_error_n =
        reproj_error / pose_dataset_.View(view_id)->TrackIds().size();
    if (repro_error_n > max_reproj_error) {
      LOG(INFO) << "Removing view " << view_id
                << " due to large reprojection error: " << repro_error_n
                << "px > " << max_reproj_error << " px\n";
      pose_dataset_.RemoveView(view_id);
    }
    total_repro_error += repro_error_n;
    ++processed_frames;
  }
  return true;
}

void PoseEstimator::OptimizeBoardPoints() {
  ba_options_.constant_camera_orientation = true;
  ba_options_.constant_camera_position = true;
  ba_options_.verbose = true;

  std::map<theia::TrackId, Eigen::Matrix3d> emp_covariance_matrices;
  double empirical_variance_factor;
  // only use track ids that have actually been observed more than 3 times
  std::vector<theia::TrackId> track_ids_to_optimize;
  for (const auto& pair : tracks_to_nr_obs_) {
    if (pair.second > min_num_obs_for_optim_) {
      track_ids_to_optimize.push_back(pair.first);
    }
  }
  theia::BundleAdjustTracks(ba_options_,
                            track_ids_to_optimize,
                            &pose_dataset_,
                            &emp_covariance_matrices,
                            &empirical_variance_factor);
  std::cout << "Empirical variance factor after board point optimization: "
            << empirical_variance_factor << "\n";
  Eigen::Vector3d mean_std(0.0, 0.0, 0.0);
  for (const auto& c : emp_covariance_matrices) {
    Eigen::Vector3d stddev = c.second.diagonal().array().sqrt() * 1e3;
    mean_std += stddev;
    std::cout << "Track Id: " << c.first << " std dev: " << stddev.transpose()
              << " mm\n";
  }
  mean_std /= (double)emp_covariance_matrices.size();
  std::cout << "Mean board point standard deviation after optimization: "
            << mean_std.transpose() << " mm\n";
}

void PoseEstimator::OptimizeAllPoses() {
  ba_options_.constant_camera_orientation = false;
  ba_options_.constant_camera_position = false;
  ba_options_.verbose = true;
  LOG(INFO) << "Optimizing all estimated poses.";
  theia::BundleAdjustViews(
      ba_options_, pose_dataset_.ViewIds(), &pose_dataset_);
  LOG(INFO) << "Finished optimizing camera poses.";
}

}  // namespace core
}  // namespace OpenICC
