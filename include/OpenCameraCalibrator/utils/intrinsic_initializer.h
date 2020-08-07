// created by Steffen Urban Juli 2020
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <algorithm>

#include <Eigen/Core>

#include "OpenCameraCalibrator/utils/types.h"

#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"

#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

namespace OpenCamCalib {

bool initialize_pinhole_camera(
        const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
        const theia::RansacParameters& ransac_params,
        theia::RansacSummary& ransac_summary,
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t,
        double& focal_length);


bool initialize_radial_undistortion_camera(
        const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
        const theia::RansacParameters& ransac_params,
        theia::RansacSummary& ransac_summary,
        const int img_cols,
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t,
        double& focal_length,
        double& radial_distortion);

// took that initialization from basalt
// https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/src/calibration/calibraiton_helper.cpp
bool initialize_doublesphere_model(
        const std::vector<theia::FeatureCorrespondence2D3D> &correspondences,
        std::vector<int> charuco_ids,
        cv::Ptr<cv::aruco::CharucoBoard> &charucoboard,
        const theia::RansacParameters &ransac_params, const int img_cols,
        const int img_rows, theia::RansacSummary &ransac_summary,
        Eigen::Matrix3d &rotation, Eigen::Vector3d &position,
        double &focal_length);



}
