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

#include <algorithm>
#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/utils/types.h"

#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

namespace OpenICC {
namespace utils {

bool initialize_pinhole_camera(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const theia::RansacParameters& ransac_params,
    theia::RansacSummary& ransac_summary,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t,
    double& focal_length,
    const bool verbose = false);

bool initialize_radial_undistortion_camera(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const theia::RansacParameters& ransac_params,
    theia::RansacSummary& ransac_summary,
    const cv::Size& img_size,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t,
    double& focal_length,
    double& radial_distortion,
    const bool verbose = false);

// took that initialization from basalt,
// however at the moment working with initialize_radial_undistortion_camera
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
    const bool verbose = false);

}  // namespace utils
}  // namespace OpenICC
