#pragma once

#include <fstream>
#include <ios>
#include <iostream>

#include "theia/sfm/camera/camera.h"

namespace OpenCamCalib {
namespace io {

bool read_camera_calibration(const std::string &input_json,
                             theia::Camera &camera, double &fps);

}
}
