// created by Steffen Urban November 2019
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/reconstruction.h>
#include <theia/sfm/view.h>

#include <algorithm>
#include <dirent.h>
#include <vector>

#include "OpenCameraCalibrator/utils/types.h"

namespace OpenCamCalib {
namespace utils {

bool DoesFileExist(const std::string &path);

bool ReadDetectorParameters(std::string filename,
                            cv::Ptr<cv::aruco::DetectorParameters> &params);

double MedianOfDoubleVec(std::vector<double> &double_vec);

void PrintResult(const std::string cam_type,
                 const theia::Reconstruction &recon_calib_dataset);

std::string CameraIDToString(const int theia_enum);

double GetReprojErrorOfView(const theia::Reconstruction &recon_dataset,
                            const theia::ViewId v_id);

std::vector<std::string> load_images(const std::string &img_dir_path);

int FindClosestTimestamp(const double t_imu,
                         const std::vector<double> &vis_timestamps,
                         double &distance_to_nearest_timestamp);

Eigen::Vector3d lerp3d(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
                       double fraction);

void InterpolateQuaternions(std::vector<double> t_old,
                            std::vector<double> t_new,
                            const QuatVector &input_q,
                            QuatVector &interpolated_q);

void InterpolateVector3d(std::vector<double> t_old,
                         std::vector<double> t_new,
                         const Vec3Vector &input_vec,
                         Vec3Vector &interpolated_vec);

} // namespace utils
} // namespace OpenCamCalib
