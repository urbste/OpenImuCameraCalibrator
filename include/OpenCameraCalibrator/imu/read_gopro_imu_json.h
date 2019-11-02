// created by Steffen Urban November 2019
#pragma once

#include <string>
#include "OpenCameraCalibrator/utils/types.h"

namespace OpenCamCalib {

bool ReadGoProTelemetry(const std::string& path_to_telemetry_file,
                         CameraTelemetryData& telemetry);


}
