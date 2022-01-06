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

#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include <unordered_map>

namespace OpenICC {
namespace core {

struct Pose {
public:
  Eigen::Quaterniond rotation;
  Eigen::Vector3d position;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PoseEstimator {
public:
  PoseEstimator();

  bool EstimatePosePinhole(const theia::ViewId &view_id,
      const std::vector<theia::FeatureCorrespondence2D3D>
          &correspondences_undist,
      const std::vector<int> &board_pts3_ids);

  bool EstimatePosesFromJson(const nlohmann::json &scene_json,
                             const theia::Camera camera,
                             const double max_reproj_error = 3.0);

  void GetPoseDataset(theia::Reconstruction &pose_dataset) {
    pose_dataset = pose_dataset_;
  }

  void OptimizeBoardPoints();

  void OptimizeAllPoses();

private:
  //! Pose datasets
  theia::Reconstruction pose_dataset_;

  //! Bundle adjustment options for pose optimization
  theia::BundleAdjustmentOptions ba_options_;

  //! Ransac parameters for initial pose estimation
  theia::RansacParameters ransac_params_;

  //! Minimum number of inliers for pose estimation success
  size_t min_num_points_ = 8;

  //! Save how often a board point has been observed (for point optimization)
  std::unordered_map<theia::TrackId, size_t> tracks_to_nr_obs_;

  //! Minimum number of observations of a scene point to be optimized
  size_t min_num_obs_for_optim_ = 30;

  //! PnP type
  theia::PnPType pnp_type_ = theia::PnPType::DLS;
};

} // namespace core
} // namespace OpenICC
