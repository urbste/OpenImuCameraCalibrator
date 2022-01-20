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

#pragma once

#include <fstream>
#include <ios>
#include <iostream>

#include "theia/sfm/camera/camera.h"

namespace OpenICC {
namespace io {

bool write_camera_calibration(const std::string& output_file,
                              const theia::Camera& camera,
                              const double fps,
                              const int nr_calib_images,
                              const double total_reproj_error);
}  // namespace io
}  // namespace OpenICC
