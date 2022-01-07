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

#include <string>

#include "theia/sfm/reconstruction.h"

#include <OpenCameraCalibrator/utils/json.h>

namespace OpenICC {
namespace io {

bool read_scene_bson(const std::string& input_bson, nlohmann::json& scene_json);

void scene_points_to_calib_dataset(const nlohmann::json& json,
                                   theia::Reconstruction& reconstruction);

}  // namespace io
}  // namespace OpenICC
