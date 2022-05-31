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
#include "theia/sfm/camera/orthographic_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"

namespace OpenICC {
namespace io {

bool write_camera_calibration(const std::string& output_file,
                              const theia::Camera& camera,
                              const double fps,
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
  json_obj["intrinsics"]["principal_pt_x"] = camera.PrincipalPointX();
  json_obj["intrinsics"]["principal_pt_y"] = camera.PrincipalPointY();
  json_obj["intrinsic_type"] =
      theia::CameraIntrinsicsModelTypeToString(intrinsics->Type());

  if (camera.GetCameraIntrinsicsModelType() ==
      theia::CameraIntrinsicsModelType::PINHOLE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::PinholeCameraModel::ASPECT_RATIO);
    json_obj["intrinsics"]["radial_distortion_1"] = intrinsics->GetParameter(
        theia::PinholeCameraModel::
            RADIAL_DISTORTION_1);
    json_obj["intrinsics"]["radial_distortion_2"] = intrinsics->GetParameter(
        theia::PinholeCameraModel::
            RADIAL_DISTORTION_2);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION) {
    json_obj["intrinsics"]["aspect_ratio"] =
        intrinsics->GetParameter(theia::DivisionUndistortionCameraModel::
                                     InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsic_type"] = utils::CameraIDToString(
        (int)theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
    json_obj["intrinsics"]["div_undist_distortion"] = intrinsics->GetParameter(
        theia::DivisionUndistortionCameraModel::
            RADIAL_DISTORTION_1);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::DOUBLE_SPHERE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::ASPECT_RATIO);
    json_obj["intrinsics"]["xi"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::XI);
    json_obj["intrinsics"]["alpha"] = intrinsics->GetParameter(
        theia::DoubleSphereCameraModel::ALPHA);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED) {
    json_obj["intrinsics"]["aspect_ratio"] =
        intrinsics->GetParameter(theia::ExtendedUnifiedCameraModel::
                                     InternalParametersIndex::ASPECT_RATIO);
    json_obj["intrinsics"]["alpha"] = intrinsics->GetParameter(
        theia::ExtendedUnifiedCameraModel::ALPHA);
    json_obj["intrinsics"]["beta"] = intrinsics->GetParameter(
        theia::ExtendedUnifiedCameraModel::BETA);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::FISHEYE) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::ASPECT_RATIO);
    json_obj["intrinsics"]["radial_distortion_1"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::
            RADIAL_DISTORTION_1);
    json_obj["intrinsics"]["radial_distortion_2"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::
            RADIAL_DISTORTION_2);
    json_obj["intrinsics"]["radial_distortion_3"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::
            RADIAL_DISTORTION_3);
    json_obj["intrinsics"]["radial_distortion_4"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::
            RADIAL_DISTORTION_4);
  } else if (camera.GetCameraIntrinsicsModelType() ==
             theia::CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL) {
    json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
        theia::FisheyeCameraModel::ASPECT_RATIO);
    json_obj["intrinsics"]["radial_distortion_1"] = intrinsics->GetParameter(
        theia::PinholeRadialTangentialCameraModel::
            RADIAL_DISTORTION_1);
    json_obj["intrinsics"]["radial_distortion_2"] = intrinsics->GetParameter(
        theia::PinholeRadialTangentialCameraModel::
            RADIAL_DISTORTION_2);
    json_obj["intrinsics"]["radial_distortion_3"] = intrinsics->GetParameter(
        theia::PinholeRadialTangentialCameraModel::
            RADIAL_DISTORTION_3);
    json_obj["intrinsics"]["tangential_distortion_1"] =
        intrinsics->GetParameter(
            theia::PinholeRadialTangentialCameraModel::
                TANGENTIAL_DISTORTION_1);
    json_obj["intrinsics"]["tangential_distortion_2"] =
        intrinsics->GetParameter(
            theia::PinholeRadialTangentialCameraModel::
                TANGENTIAL_DISTORTION_2);
  } else if (camera.GetCameraIntrinsicsModelType() ==
               theia::CameraIntrinsicsModelType::ORTHOGRAPHIC) {
      json_obj["intrinsics"]["aspect_ratio"] = intrinsics->GetParameter(
          theia::OrthographicCameraModel::ASPECT_RATIO);
      json_obj["intrinsics"]["radial_distortion_1"] = intrinsics->GetParameter(
          theia::OrthographicCameraModel::
              RADIAL_DISTORTION_1);
      json_obj["intrinsics"]["radial_distortion_2"] = intrinsics->GetParameter(
          theia::OrthographicCameraModel::
              RADIAL_DISTORTION_2);
   } else {
    std::cerr << "Camera model type not supported\n";
  }

  json_obj["intrinsics"]["focal_length"] = camera.FocalLength();

  json_file << std::setw(2) << json_obj << std::endl;
  json_file.close();
  return true;
}

}  // namespace io
}  // namespace OpenICC
