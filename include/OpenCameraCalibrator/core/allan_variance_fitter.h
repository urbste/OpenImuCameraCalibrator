#pragma once

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include "OpenCameraCalibrator/allanvariance/allan_gyr.h"
#include "OpenCameraCalibrator/allanvariance/allan_acc.h"

#include "OpenCameraCalibrator/allanvariance/fitallan_acc.h"
#include "OpenCameraCalibrator/allanvariance/allan_gyr.h"

namespace OpenICC {
namespace core {

class AllanVarianceFitter {
public:
  AllanVarianceFitter(const CameraTelemetryData &telemetry_data, const int nr_clusters);

  bool RunFit();

private:
  CameraTelemetryData telemetry_data_;

  allanvar::AllanAcc* data_acc_x_;
  allanvar::AllanAcc* data_acc_y_;
  allanvar::AllanAcc* data_acc_z_;

  allanvar::AllanGyr* data_gyr_x_;
  allanvar::AllanGyr* data_gyr_y_;
  allanvar::AllanGyr* data_gyr_z_;


};

} // namespace core
} // namespace OpenICC
