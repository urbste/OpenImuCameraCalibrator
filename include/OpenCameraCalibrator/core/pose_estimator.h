#pragma once

#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

namespace OpenCamCalib {
namespace core {

struct Pose {
public:
  Eigen::Quaterniond rotation;
  Eigen::Vector3d position;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PoseEstimator {
public:
  PoseEstimator();

  bool EstimatePosePinhole(
      const theia::ViewId& view_id,
      const std::vector<theia::FeatureCorrespondence2D3D> &correspondences,
      const std::vector<int> &board_pts3_ids);

  bool EstimatePosesFromJson(const nlohmann::json &scene_json,
                             const theia::Camera &camera,
                             const double max_reproj_error = 3.0);

  void GetPosesAndTimestampsSorted(std::map<uint64_t, Pose> poses);

  void GetPoseDataset(theia::Reconstruction &pose_dataset) {
    pose_dataset = pose_dataset_;
  }

private:
  //! Pose datasets
  theia::Reconstruction pose_dataset_;

  //! Bundle adjustment options for pose optimization
  theia::BundleAdjustmentOptions ba_options_;

  //! Ransac parameters for initial pose estimation
  theia::RansacParameters ransac_params_;
};

} // namespace core
} // namespace OpenCamCalib
