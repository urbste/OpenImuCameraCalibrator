// created by Steffen Urban 2019, January

#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gtest/gtest.h"

#include "OpenCameraCalibrator/filter/kalmanRTS.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/reconstruction.h"
#include "OpenCameraCalibrator/imu/imu_utils.h"
#include "OpenCameraCalibrator/utils/splinterp.h"
#include "theia/util/string.h"
#include "theia/util/util.h"

namespace filter {

TEST(EstimateCameraImuAlignment, SimpleTest) {
  // create timestamps
  std::vector<double> timestamps(1000);
  timestamps[0] = 0.0;
  for (int i = 1; i < 1000; ++i) {
    timestamps[i] = 0.03 + timestamps[i - 1];
  }
  // create poses
  std::map<double, Eigen::Vector3d> poses;
  for (int i = 0; i < 1000; ++i) {
    double v = timestamps[i];
    poses[v] =
        Eigen::Vector3d(std::sin(v), std::cos(v), std::sin(v) * std::cos(v));
  }
}
}
