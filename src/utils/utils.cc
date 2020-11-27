
#include "OpenCameraCalibrator/utils/utils.h"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>

#include <algorithm>
#include <vector>

using namespace cv;

namespace OpenCamCalib {
namespace utils {

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
  fs["useAruco3Detection"] >> params->useAruco3Detection;
  if (params->useAruco3Detection) {
      params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
  }
  fs["cameraMotionSpeed"] >> params->cameraMotionSpeed;
  fs["useGlobalThreshold"] >> params->useGlobalThreshold;
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

double GetReprojErrorOfView(const theia::Reconstruction& recon_dataset,
                            const theia::ViewId v_id) {
  const theia::View *v = recon_dataset.View(v_id);
  std::vector<theia::TrackId> track_ids = v->TrackIds();
  double view_reproj_error = 0.0;
  for (int t = 0; t < track_ids.size(); ++t) {
    const theia::Feature *feat = v->GetFeature(track_ids[t]);
    const theia::Track *track = recon_dataset.Track(track_ids[t]);
    Eigen::Vector2d pt;
    v->Camera().ProjectPoint(track->Point(), &pt);
    view_reproj_error += (pt - (*feat)).norm();
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

}  // namespace utils
}  // namespace OpenCamCalib
