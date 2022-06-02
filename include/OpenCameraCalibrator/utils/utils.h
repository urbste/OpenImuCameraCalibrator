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
#include <opencv2/opencv.hpp>

#include <theia/sfm/reconstruction.h>
#include <theia/sfm/view.h>

#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <vector>

#include "OpenCameraCalibrator/utils/types.h"

namespace OpenICC {
namespace utils {

bool DoesFileExist(const std::string& path);

bool ReadDetectorParameters(std::string filename,
                            cv::Ptr<cv::aruco::DetectorParameters>& params);

double MedianOfDoubleVec(std::vector<double>& double_vec);

void PrintResult(const std::string cam_type,
                 const theia::Reconstruction& recon_calib_dataset);

std::string CameraIDToString(const int theia_enum);

int GravDirStringToInt(const std::string& string);

double GetReprojErrorOfView(const theia::Reconstruction& recon_dataset,
                            const theia::ViewId v_id);

std::vector<std::string> load_images(const std::string& img_dir_path);

size_t FindClosestTimestamp(const double t_imu,
                         const std::vector<double>& vis_timestamps,
                         double& distance_to_nearest_timestamp);

Eigen::Vector3d lerp3d(const Eigen::Vector3d& v0,
                       const Eigen::Vector3d& v1,
                       double fraction);

void InterpolateQuaternions(std::vector<double> t_old,
                            std::vector<double> t_new,
                            const quat_vector& input_q,
                            quat_vector& interpolated_q);

void InterpolateVector3d(std::vector<double> t_old,
                         std::vector<double> t_new,
                         const vec3_vector& input_vec,
                         vec3_vector& interpolated_vec);

// average calculation
template <class T>
T average(const std::vector<T> datas) {
  T sum_data = T(0);
  for (auto data : datas) {
    sum_data += data;
  }
  return sum_data / datas.size();
}

bool IsPathAFile(const std::string& path);


// method for calculating the pseudo-Inverse as recommended by Eigen developers
// https://gist.github.com/gokhansolak/d2abaefcf3e3b767f5bc7d81cfe0b36a
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

}  // namespace utils
}  // namespace OpenICC
