// created by Steffen Urban November 2019
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <algorithm>

namespace OpenCamCalib {

bool ReadDetectorParameters(std::string filename,
                            cv::Ptr<cv::aruco::DetectorParameters>& params);

double MedianOfDoubleVec(std::vector<double>& double_vec);


}
