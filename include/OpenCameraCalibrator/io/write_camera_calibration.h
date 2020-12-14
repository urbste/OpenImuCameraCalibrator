#pragma once

#include <fstream>
#include <ios>
#include <iostream>

#include "theia/sfm/camera/camera.h"


namespace OpenCamCalib {
namespace io {

bool write_camera_calibration(const std::string &output_file,
                              const theia::Camera &camera, double fps,
                              const int nr_calib_images,
                              const double final_ba_cost,
                              const double total_reproj_error);
}
} // namespace OpenCamCalib
