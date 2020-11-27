// created by Steffen Urban November 2019
#pragma once

#include <string>
#include "OpenCameraCalibrator/utils/types.h"

namespace OpenCamCalib {

bool ReadGoProTelemetry(const std::string& path_to_telemetry_file,
                        CameraTelemetryData& telemetry);

bool ReadSplineErrorWeighting(const std::string& path_to_spline_error_weighting_json,
                             SplineWeightingData& spline_weighting);

bool ReadIMUBias(const std::string& path_to_imu_bias,
                 Eigen::Vector3d& gyro_bias,
                 Eigen::Vector3d& accl_bias);
}
