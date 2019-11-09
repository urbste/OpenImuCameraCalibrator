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
#include "theia/util/string.h"
#include "theia/util/util.h"

namespace filter {

//TEST(RTSSmoother, SimpleTest) {
//  // create timestamps
//  std::vector<double> timestamps(1000);
//  timestamps[0] = 0.0;
//  for (int i = 1; i < 1000; ++i) {
//    timestamps[i] = 0.03 + timestamps[i - 1];
//  }
//  // create poses
//  std::map<double, Eigen::Vector3d> poses;
//  for (int i = 0; i < 1000; ++i) {
//    double v = timestamps[i];
//    poses[v] =
//        Eigen::Vector3d(std::sin(v), std::cos(v), std::sin(v) * std::cos(v));
//  }

//  std::vector<Eigen::Vector3d> accelerations;
//  filter::FilterVisualPositions(poses, 0.1 * 0.1, &accelerations);
//}

//TEST(RTSSmoother, TestRealData) {
//  const std::string scan_path =
//      "/media/steffen/Data/image_data_sets/scale_estimation/"
//      "2018_11_29-07_41_57/";
//  theia::CameraImuData imu_data;
//  theia::Reconstruction reconstruction;

//  theia::ReadIMUData(scan_path, "accelerations.csv", "rotations.csv", imu_data);
//  CHECK(theia::ReadReconstruction(
//      scan_path + "/extracted_kfs/" + "sparse_recon_localized",
//      &reconstruction))
//      << "Could not read Reconstruction file.";

//  // get poses and rotations from reconstruction
//  const std::vector<theia::ViewId> view_ids = reconstruction.ViewIds();
//  std::map<double, Eigen::Quaterniond> time_to_R_wc;
//  std::map<double, Eigen::Vector3d> time_to_X_cw;
//  for (int i = 0; i < view_ids.size(); ++i) {
//    theia::ViewId view_id = view_ids[i];
//    // search for pose
//    const theia::View& view = *reconstruction.View(view_id);
//    double timestamp =
//        theia::extract_timestamp_from_string(view.Name()) / theia::usec_to_sec;

//    time_to_R_wc[timestamp] =
//        Eigen::Quaterniond(view.Camera().GetOrientationAsRotationMatrix());
//    time_to_X_cw[timestamp] = view.Camera().GetPosition();
//  }

//  // filter positions
//  std::vector<Eigen::Vector3d> accelerations;
//  filter::FilterVisualPositions(time_to_X_cw, 0.1 * 0.1, &accelerations);
//}
}
