#pragma once

#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/utils/json.h"

namespace OpenCamCalib {
namespace core {

class CameraCalibrator {
public:
  CameraCalibrator(const std::string &camera_model);

  bool RunCalibration();

  bool CalibrateCameraFromJson(const nlohmann::json &scene_json,
                               const std::string &output_path);

  bool WriteCalibration(const std::string &output_path);

  void RemoveViewsReprojError(const double max_reproj_error = 2.0);

  bool AddObservation(const theia::ViewId &view_id,
                      const theia::TrackId &object_point_id,
                      const Eigen::Vector2d &corner);

  theia::ViewId AddView(const Eigen::Matrix3d &initial_rotation,
                        const Eigen::Vector3d &initial_translation,
                        const double &initial_focal_length,
                        const double &initial_distortion,
                        const int &image_width, const int &image_height,
                        const double &timestamp_s,
                        theia::CameraIntrinsicsGroupId group_id = 0);

  void SetRansacErrorThresh(const double error_thresh = 0.1) {
    ransac_params_.error_thresh = error_thresh;
  }
  void SetVerbose() { verbose_ = true; }

  //! camera poses will only be used for calibration if there is no other camera
  //! pose in a voxel
  void SetGridSize(const double grid_size = 0.04) { grid_size_ = grid_size; }

private:
  //! holds all calibration information like views and features
  theia::Reconstruction recon_calib_dataset_;

  //! Ransac parameters for initial pose estimation
  theia::RansacParameters ransac_params_;

  //! camera model
  std::string camera_model_;

  //! if we want verbose output
  bool verbose_ = false;

  //! voxel grid size
  double grid_size_;
};

} // namespace core
} // namespace OpenCamCalib
