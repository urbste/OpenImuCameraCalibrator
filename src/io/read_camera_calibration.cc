#include <fstream>
#include <ios>
#include <iostream>

#include "OpenCameraCalibrator/io/read_camera_calibration.h"
#include "OpenCameraCalibrator/utils/json.h"

#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"

using nlohmann::json;

namespace OpenCamCalib {
namespace io {

bool read_camera_calibration(const std::string &input_json,
                             theia::Camera &camera, double &fps) {
  std::ifstream input(input_json);
  if (!input.is_open()) {
    std::cerr << "Could not open: " << input_json << "\n";
    return false;
  }
  json json_content;
  input >> json_content;

  std::string camera_model_type = json_content["intrinsic_type"];

  camera.SetCameraIntrinsicsModelType(
      theia::StringToCameraIntrinsicsModelType(camera_model_type));

  camera.SetImageSize(json_content["image_width"],
                      json_content["image_height"]);
  camera.SetPrincipalPoint(json_content["intrinsics"]["principal_pt_x"],
                           json_content["intrinsics"]["principal_pt_y"]);
  camera.SetFocalLength(json_content["intrinsics"]["focal_length"]);

  fps = json_content["fps"];

  double *intr = camera.mutable_intrinsics();
  if (json_content["intrinsic_type"] == "DIVISION_UNDISTORTION") {
    intr[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_1] =
        json_content["intrinsics"]["div_undist_distortion"];
    intr[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
             ASPECT_RATIO] = json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "DOUBLE_SPHERE") {
    intr[theia::DoubleSphereCameraModel::InternalParametersIndex::XI] =
        json_content["intrinsics"]["xi"];
    intr[theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] =
        json_content["intrinsics"]["alpha"];
    intr[theia::DoubleSphereCameraModel::InternalParametersIndex::
             ASPECT_RATIO] = json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "EXTENDED_UNIFIED") {
    intr[theia::ExtendedUnifiedCameraModel::InternalParametersIndex::ALPHA] =
        json_content["intrinsics"]["alpha"];
    intr[theia::ExtendedUnifiedCameraModel::InternalParametersIndex::BETA] =
        json_content["intrinsics"]["beta"];
    intr[theia::ExtendedUnifiedCameraModel::InternalParametersIndex::
             ASPECT_RATIO] = json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "FISHEYE") {
    intr[theia::FisheyeCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_1] =
        json_content["intrinsics"]["radial_distortion_1"];
    intr[theia::FisheyeCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_2] =
        json_content["intrinsics"]["radial_distortion_2"];
    intr[theia::FisheyeCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_3] =
        json_content["intrinsics"]["radial_distortion_3"];
    intr[theia::FisheyeCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_4] =
        json_content["intrinsics"]["radial_distortion_4"];
    intr[theia::FisheyeCameraModel::InternalParametersIndex::ASPECT_RATIO] =
        json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "LINEAR_PINHOLE") {
    intr[theia::PinholeCameraModel::InternalParametersIndex::ASPECT_RATIO] =
        json_content["intrinsics"]["aspect_ratio"];
  }
  intr = NULL;
  delete intr;
  input.close();
  return true;
}

} // namespace io
} // namespace OpenCamCalib
