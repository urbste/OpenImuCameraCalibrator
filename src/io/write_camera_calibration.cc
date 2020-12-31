#include <fstream>
#include <ios>
#include <iostream>

#include "OpenCameraCalibrator/io/write_camera_calibration.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"

namespace OpenCamCalib {
namespace io {

bool write_camera_calibration(const std::string &output_file,
                              const theia::Camera &camera, const double fps,
                              const int nr_calib_images,
                              const double total_reproj_error) {
  std::ofstream json_file(output_file);
  if (!json_file.is_open()) {
    std::cerr << "Could not open: " << output_file << "\n";
    return false;
  }
  nlohmann::json json_obj;

  const std::shared_ptr<theia::CameraIntrinsicsModel> intrinsics =
      camera.CameraIntrinsics();

  json_obj["stabelized"] = false;
  json_obj["fps"] = fps;
  json_obj["nr_calib_images"] = nr_calib_images;
  json_obj["final_reproj_error"] = total_reproj_error;
  json_obj["image_width"] = camera.ImageWidth();
  json_obj["image_height"] = camera.ImageHeight();
  json_obj["intrinsics"]["skew"] = 0.0;

  json_obj["intrinsics"]["focal_length"] = camera.FocalLength();
  json_obj["intrinsics"]["principal_pt_x"] = camera.PrincipalPointX();
  json_obj["intrinsics"]["principal_pt_y"] = camera.PrincipalPointY();
  json_obj["intrinsic_type"] = theia::CameraIntrinsicsModelTypeToString(intrinsics->Type());

  if (camera.GetCameraIntrinsicsModelType() ==
      theia::CameraIntrinsicsModelType::PINHOLE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::PinholeCameraModel::InternalParametersIndex::ASPECT_RATIO);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION) {
    json_obj["intrinsics"]["aspect_ratio"] =
        intrinsics->GetParameter(theia::DivisionUndistortionCameraModel::
                                     InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsic_type"] = utils::CameraIDToString(
        (int)theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
    json_obj["intrinsics"]["div_undist_distortion"] = intrinsics->GetParameter(
        theia::DivisionUndistortionCameraModel::InternalParametersIndex::
            RADIAL_DISTORTION_1);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::DOUBLE_SPHERE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsics"]["xi"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::InternalParametersIndex::XI);
    json_obj["intrinsics"]["alpha"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED) {
    json_obj["intrinsics"]["aspect_ratio"] =
        intrinsics->GetParameter(theia::ExtendedUnifiedCameraModel::
                                     InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsics"]["alpha"] = intrinsics->GetParameter(
        theia::ExtendedUnifiedCameraModel::InternalParametersIndex::ALPHA);
    json_obj["intrinsics"]["beta"] = intrinsics->GetParameter(
        theia::ExtendedUnifiedCameraModel::InternalParametersIndex::BETA);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::FISHEYE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsics"]["radial_distortion_1"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::InternalParametersIndex::
            RADIAL_DISTORTION_1);
    json_obj["intrinsics"]["radial_distortion_2"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::InternalParametersIndex::
            RADIAL_DISTORTION_2);
    json_obj["intrinsics"]["radial_distortion_3"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::InternalParametersIndex::
            RADIAL_DISTORTION_3);
    json_obj["intrinsics"]["radial_distortion_4"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::InternalParametersIndex::
            RADIAL_DISTORTION_4);
  } else {
    std::cerr << "Camera model type not supported\n";
  }
  json_file << std::setw(2) << json_obj << std::endl;
  json_file.close();
  return true;
}

} // namespace io
} // namespace OpenCamCalib
