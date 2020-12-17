#pragma once

#include <string>

#include "theia/sfm/reconstruction.h"

#include <OpenCameraCalibrator/utils/json.h>

namespace OpenCamCalib {
namespace io {

bool read_scene_bson(const std::string &input_bson,
                     nlohmann::json &scene_json);


void scene_points_to_calib_dataset(const nlohmann::json &json, theia::Reconstruction &reconstruction);

}
}
