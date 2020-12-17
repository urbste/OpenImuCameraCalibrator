#include <fstream>
#include <ios>
#include <iostream>

#include "OpenCameraCalibrator/io/read_camera_calibration.h"
#include "OpenCameraCalibrator/utils/json.h"

#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
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

  if (json_content["intrinsic_type"] ==
      "DIVISION_UNDISTORTION") {
    camera.mutable_intrinsics()
        [theia::DivisionUndistortionCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_1] =
        json_content["intrinsics"]["div_undist_distortion"];
    camera.mutable_intrinsics()
        [theia::DivisionUndistortionCameraModel::InternalParametersIndex::ASPECT_RATIO] =
        json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "DOUBLE_SPHERE") {
    camera.mutable_intrinsics()
        [theia::DoubleSphereCameraModel::InternalParametersIndex::XI] =
        json_content["intrinsics"]["xi"];
    camera.mutable_intrinsics()
        [theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] =
        json_content["intrinsics"]["alpha"];
    camera.mutable_intrinsics()
        [theia::DoubleSphereCameraModel::InternalParametersIndex::ASPECT_RATIO] =
        json_content["intrinsics"]["aspect_ratio"];
  } else if (json_content["intrinsic_type"] == "LINEAR_PINHOLE") {
      camera.mutable_intrinsics()
          [theia::PinholeCameraModel::InternalParametersIndex::ASPECT_RATIO] =
          json_content["intrinsics"]["aspect_ratio"];
  }


  input.close();
  return true;
}

} // namespace io
} // namespace OpenCamCalib
