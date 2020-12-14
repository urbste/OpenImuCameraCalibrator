// created by Steffen Urban November 2019
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/view.h>
#include <theia/sfm/reconstruction.h>

#include <vector>
#include <algorithm>
#include <dirent.h>

namespace OpenCamCalib {
namespace utils {

bool ReadDetectorParameters(std::string filename,
                            cv::Ptr<cv::aruco::DetectorParameters>& params);

double MedianOfDoubleVec(std::vector<double>& double_vec);

void PrintResult(const std::string cam_type,
                 const theia::Reconstruction &recon_calib_dataset);

std::string CameraIDToString(const int theia_enum);

double GetReprojErrorOfView(const theia::Reconstruction& recon_dataset,
                            const theia::ViewId v_id);

std::vector<std::string> load_images(const std::string &img_dir_path);

}
}
