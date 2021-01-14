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

#include <Eigen/Geometry>
#include <string>

#include "OpenCameraCalibrator/utils/types.h"

namespace OpenICC {
namespace io {

bool ReadSplineErrorWeighting(
    const std::string &path_to_spline_error_weighting_json,
    SplineWeightingData &spline_weighting);

bool ReadIMUBias(const std::string &path_to_imu_bias,
                 Eigen::Vector3d &gyro_bias, Eigen::Vector3d &accl_bias);

bool ReadIMU2CamInit(const std::string &path_to_file,
                     Eigen::Quaterniond &imu_to_cam_rotation,
                     double &time_offset_imu_to_cam);
} // namespace io
} // namespace OpenICC
