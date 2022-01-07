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

bool read_camera_calibration(const std::string& input_json,
                             theia::Camera& camera,
                             double& fps);

}  // namespace io
}  // namespace OpenICC
