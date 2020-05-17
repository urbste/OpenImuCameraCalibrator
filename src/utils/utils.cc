
#include "OpenCameraCalibrator/utils/utils.h"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

using namespace cv;

namespace OpenCamCalib {

bool ReadDetectorParameters(std::string filename,
                            Ptr<aruco::DetectorParameters>& params) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened()) return false;
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
  return true;
}

double MedianOfDoubleVec(std::vector<double>& double_vec) {
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

}  // namespace OpenCamCalib
