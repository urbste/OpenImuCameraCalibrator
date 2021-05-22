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

#include "OpenCameraCalibrator/utils/utils.h"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <algorithm>
#include <fstream>
#include <vector>

using namespace cv;

namespace OpenICC {
namespace utils {

bool DoesFileExist(const std::string &path) {
  std::ifstream file(path);
  if (file.is_open()) {
    file.close();
    return true;
  }
  return false;
}

bool ReadDetectorParameters(std::string filename,
                            Ptr<aruco::DetectorParameters> &params) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >>
      params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  //  fs["useAruco3Detection"] >> params->useAruco3Detection;
  //  if (params->useAruco3Detection) {
  //      params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
  //  }
  //  fs["cameraMotionSpeed"] >> params->cameraMotionSpeed;
  //  fs["useGlobalThreshold"] >> params->useGlobalThreshold;
  return true;
}

double MedianOfDoubleVec(std::vector<double> &double_vec) {
  assert(!double_vec.empty());
  if (double_vec.size() % 2 == 0) {
    const auto median_it1 = double_vec.begin() + double_vec.size() / 2 - 1;
    const auto median_it2 = double_vec.begin() + double_vec.size() / 2;

    std::nth_element(double_vec.begin(), median_it1, double_vec.end());
    const auto e1 = *median_it1;

    std::nth_element(double_vec.begin(), median_it2, double_vec.end());
    const auto e2 = *median_it2;

    return (e1 + e2) / 2;

  } else {
    const auto median_it = double_vec.begin() + double_vec.size() / 2;
    std::nth_element(double_vec.begin(), median_it, double_vec.end());
    return *median_it;
  }
}

void PrintResult(const std::string cam_type,
                 const theia::Reconstruction &recon_calib_dataset) {
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    theia::ViewId id = recon_calib_dataset.ViewIds()[i];
    std::cout << "Viewid: " << id << "\n";
    std::cout << "Optimized camera focal length: "
              << recon_calib_dataset.View(id)->Camera().FocalLength()
              << std::endl;
    std::cout << "Optimized principal point: "
              << recon_calib_dataset.View(id)->Camera().PrincipalPointX() << " "
              << recon_calib_dataset.View(id)->Camera().PrincipalPointY()
              << std::endl;
    std::cout << "Optimized aspect ratio: "
              << recon_calib_dataset.View(id)
                     ->Camera()
                     .intrinsics()[theia::PinholeCameraModel::
                                       InternalParametersIndex::ASPECT_RATIO]
              << std::endl;
    if (cam_type == "DIVISION_UNDISTORTION") {
      std::cout
          << "Optimized radial distortion: "
          << recon_calib_dataset.View(id)
                 ->Camera()
                 .intrinsics()[theia::DivisionUndistortionCameraModel::
                                   InternalParametersIndex::RADIAL_DISTORTION_1]
          << std::endl;
    } else if (cam_type == "DOUBLE_SPHERE") {
      std::cout
          << "Optimized XI: "
          << recon_calib_dataset.View(id)->Camera().intrinsics()
                 [theia::DoubleSphereCameraModel::InternalParametersIndex::XI]
          << std::endl;
      std::cout << "Optimized ALPHA: "
                << recon_calib_dataset.View(id)
                       ->Camera()
                       .intrinsics()[theia::DoubleSphereCameraModel::
                                         InternalParametersIndex::ALPHA]
                << std::endl;
    }
  }
}

std::string CameraIDToString(const int theia_enum) {
  if (theia_enum == (int)theia::CameraIntrinsicsModelType::DOUBLE_SPHERE) {
    return "DOUBLE_SPHERE";
  } else if ((int)theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION) {
    return "DIVISION_UNDISTORTION";
  } else if ((int)theia::CameraIntrinsicsModelType::PINHOLE) {
    return "PINHOLE";
  }
}

double GetReprojErrorOfView(const theia::Reconstruction &recon_dataset,
                            const theia::ViewId v_id) {
  const theia::View *v = recon_dataset.View(v_id);
  std::vector<theia::TrackId> track_ids = v->TrackIds();
  double view_reproj_error = 0.0;
  for (int t = 0; t < track_ids.size(); ++t) {
    const theia::Feature *feat = v->GetFeature(track_ids[t]);
    const theia::Track *track = recon_dataset.Track(track_ids[t]);
    Eigen::Vector2d pt;
    v->Camera().ProjectPoint(track->Point(), &pt);
    view_reproj_error += (pt - (*feat).point_).norm();
  }
  view_reproj_error /= track_ids.size();
  return view_reproj_error;
}

std::vector<std::string> load_images(const std::string &img_dir_path) {
  DIR *dir;
  if ((dir = opendir(img_dir_path.c_str())) == nullptr) {
  }
  std::vector<std::string> img_paths;
  dirent *dp;
  for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
    img_paths.push_back(img_dir_path + "/" + std::string(dp->d_name));
  }
  closedir(dir);

  std::sort(img_paths.begin(), img_paths.end());
  return img_paths;
}

int FindClosestTimestamp(const double t_imu,
                         const std::vector<double> &vis_timestamps,
                         double &distance_to_nearest_timestamp) {
  double dist = std::numeric_limits<double>::max();
  int idx = 0;
  for (int i = 0; i < vis_timestamps.size(); ++i) {
    double new_dist = std::abs(t_imu - vis_timestamps[i]);
    if (new_dist < dist) {
      distance_to_nearest_timestamp = new_dist;
      idx = i;
      dist = new_dist;
      if (distance_to_nearest_timestamp == 0.0) {
          break;
      }
    }
  }

  return idx;
}

Eigen::Vector3d lerp3d(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
                       double fraction) {
  return (1.0 - fraction) * v0 + fraction * v1;
}

void InterpolateQuaternions(std::vector<double> t_old,
                            std::vector<double> t_new,
                            const quat_vector &input_q,
                            quat_vector &interpolated_q) {
  for (size_t i = 0; i < t_new.size(); ++i) {
    double dist_to_nearest_vis_t;
    int nearest_old_idx =
        FindClosestTimestamp(t_new[i], t_old, dist_to_nearest_vis_t);
    if (nearest_old_idx < t_new.size()) {
      const double fraction =
          dist_to_nearest_vis_t /
          (t_old[nearest_old_idx + 1] - t_old[nearest_old_idx]);
      interpolated_q.push_back(input_q[nearest_old_idx].slerp(
          fraction, input_q[nearest_old_idx + 1]));
    } else {
      interpolated_q.push_back(input_q[nearest_old_idx]);
    }
  }
}

void InterpolateVector3d(std::vector<double> t_old, std::vector<double> t_new,
                         const vec3_vector &input_vec,
                         vec3_vector &interpolated_vec) {

  for (size_t i = 0; i < t_new.size(); ++i) {
    double dist_to_nearest_vis_t;
    int nearest_old_idx =
        FindClosestTimestamp(t_new[i], t_old, dist_to_nearest_vis_t);
    const double fraction =
        dist_to_nearest_vis_t /
        (t_old[nearest_old_idx + 1] - t_old[nearest_old_idx]);
    if (nearest_old_idx < t_new.size()) {
      interpolated_vec.push_back(lerp3d(input_vec[nearest_old_idx],
                                        input_vec[nearest_old_idx + 1],
                                        fraction));
    } else {
      interpolated_vec.push_back(input_vec[nearest_old_idx]);
    }
  }
}



} // namespace utils
} // namespace OpenICC
