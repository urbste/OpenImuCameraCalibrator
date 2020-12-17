#include "OpenCameraCalibrator/core/camera_calibrator.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <theia/io/reconstruction_writer.h>
#include <theia/sfm/bundle_adjustment/bundle_adjuster.h>
#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/io/write_camera_calibration.h"
#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

namespace OpenCamCalib {
namespace core {

CameraCalibrator::CameraCalibrator(const std::string &camera_model)
    : camera_model_(camera_model) {
  ransac_params_.failure_probability = 0.001;
  ransac_params_.use_mle = true;
  ransac_params_.max_iterations = 1000;
  ransac_params_.min_iterations = 10;
  ransac_params_.error_thresh = 0.5;
}

void CameraCalibrator::RemoveViewsReprojError(const double max_reproj_error) {
  // reproj error per view, remove some views which have a high error
  std::map<theia::ViewId, double> ids_to_remove;
  for (int i = 0; i < recon_calib_dataset_.NumViews(); ++i) {
    const theia::ViewId v_id = recon_calib_dataset_.ViewIds()[i];
    const double view_reproj_error =
        utils::GetReprojErrorOfView(recon_calib_dataset_, v_id);
    if (view_reproj_error > max_reproj_error) {
      ids_to_remove[v_id] = view_reproj_error;
    }
  }
  for (auto v_id : ids_to_remove) {
    recon_calib_dataset_.RemoveView(v_id.first);
    LOG(INFO) << "Removed view: " << v_id.first
              << " with RMSE reproj error: " << v_id.second << "\n";
  }
}

bool CameraCalibrator::AddObservation(const theia::ViewId &view_id,
                                      const theia::TrackId &object_point_id,
                                      const Eigen::Vector2d &corner) {
  return recon_calib_dataset_.AddObservation(view_id, object_point_id, corner);
}

theia::ViewId
CameraCalibrator::AddView(const Eigen::Matrix3d &initial_rotation,
                          const Eigen::Vector3d &initial_translation,
                          const double &initial_focal_length,
                          const double &initial_distortion,
                          const int &image_width, const int &image_height,
                          const double &timestamp_s,
                          const theia::CameraIntrinsicsGroupId group_id) {
  // fill charucoCorners to theia reconstruction
  std::string view_name = std::to_string((uint64_t)(timestamp_s * 1e6));
  theia::ViewId view_id = recon_calib_dataset_.AddView(view_name, group_id, timestamp_s);
  theia::View *theia_view = recon_calib_dataset_.MutableView(view_id);
  theia_view->SetEstimated(true);

  // initialize intrinsics
  theia::Camera *cam = theia_view->MutableCamera();
  cam->SetImageSize(image_width, image_height);
  cam->SetPrincipalPoint(image_width / 2.0, image_height / 2.0);
  const Eigen::Vector3d position =
      -initial_rotation.transpose() * initial_translation;
  cam->SetPosition(position);
  cam->SetOrientationFromRotationMatrix(initial_rotation);
  cam->SetFocalLength(initial_focal_length);

  if (camera_model_ == "LINEAR_PINHOLE") {
    cam->SetCameraIntrinsicsModelType(
        theia::CameraIntrinsicsModelType::PINHOLE);
  } else if (camera_model_ == "DIVISION_UNDISTORTION") {
    cam->SetCameraIntrinsicsModelType(
        theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
    cam->mutable_intrinsics()
        [theia::DivisionUndistortionCameraModel::InternalParametersIndex::
             RADIAL_DISTORTION_1] = initial_distortion;
  } else if (camera_model_ == "DOUBLE_SPHERE") {
    cam->SetCameraIntrinsicsModelType(
        theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
    cam->mutable_intrinsics()
        [theia::DoubleSphereCameraModel::InternalParametersIndex::XI] = 1.0;
    cam->mutable_intrinsics()
        [theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] = 0.0;
  }

  return view_id;
}

bool CameraCalibrator::RunCalibration() {
  if (recon_calib_dataset_.NumViews() < 10) {
    LOG(ERROR) << "Not enough views for proper calibration!" << std::endl;
    return 0;
  }

  LOG(INFO) << "Using " << recon_calib_dataset_.NumViews()
            << " in bundle adjustment\n";
  // bundle adjust everything
  theia::BundleAdjustmentOptions ba_options;
  ba_options.verbose = false;
  ba_options.fix_tracks = true;
  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options.robust_loss_width = 1.345;

  /////////////////////////////////////////////////
  /// 1. Optimize focal length and radial distortion, keep principal point fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = false;
  ba_options.constant_camera_position = false;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
  if (camera_model_ == "DIVISION_UNDISTORTION" ||
      camera_model_ == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
  LOG(INFO) << "Bundle adjusting focal length and radial distortion. Keeping "
               "cam position fixed.\n";

  theia::BundleAdjustmentSummary summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset_);

  RemoveViewsReprojError(2.0);

  /////////////////////////////////////////////////
  /// 2. Optimize principal point keeping everything else fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = true;
  ba_options.constant_camera_position = true;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS;

  summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset_);

  if (recon_calib_dataset_.NumViews() < 8) {
    std::cout << "Not enough views left for proper calibration!" << std::endl;
    return 0;
  }

  /////////////////////////////////////////////////
  /// 3. Full reconstruction
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = false;
  ba_options.constant_camera_position = false;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS |
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
  if (camera_model_ == "DIVISION_UNDISTORTION" ||
      camera_model_ == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;

  summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset_);

  RemoveViewsReprojError(1.0);

  summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset_);
}

bool CameraCalibrator::CalibrateCameraFromJson(const nlohmann::json &scene_json,
                                               const std::string &output_path) {

  io::scene_points_to_calib_dataset(scene_json, recon_calib_dataset_);

  const int image_width = scene_json["image_width"];
  const int image_height = scene_json["image_height"];
  // initial principal point
  const double px = static_cast<double>(image_width) / 2.0;
  const double py = static_cast<double>(image_height) / 2.0;

  aligned_vector<Eigen::Vector3d> saved_poses;
  // iterate views and estimate poses
  const auto views = scene_json["views"];
  for (const auto &view : views.items()) {
    const double timestamp_us = std::stod(view.key()); // to seconds
    const double timestamp_s = timestamp_us * 1e-6;    // to seconds
    const auto image_points = view.value()["image_points"];
    std::vector<int> board_pt3_ids;
    aligned_vector<Eigen::Vector2d> corners;
    for (const auto &img_pts : image_points.items()) {
      board_pt3_ids.push_back(std::stoi(img_pts.key()));
      corners.push_back(
          Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
    }

    LOG(INFO) << "Initializing view at timestamp: " << timestamp_s << "\n";
    // initialize cam pose
    std::vector<theia::FeatureCorrespondence2D3D> correspondences(
        board_pt3_ids.size());
    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      theia::FeatureCorrespondence2D3D correspondence;
      correspondence.feature[0] = corners[i][0] - px;
      correspondence.feature[1] = corners[i][1] - py;
      const Eigen::Vector4d track =
          recon_calib_dataset_.Track(board_pt3_ids[i])->Point();
      correspondence.world_point = track.hnormalized();
      correspondences[i] = correspondence;
    }

    theia::RansacSummary ransac_summary;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    bool success_init = false;
    double focal_length = 0.0, radial_distortion = 0.0;
    if (camera_model_ == "LINEAR_PINHOLE") {
      success_init = initialize_pinhole_camera(
          correspondences, ransac_params_, ransac_summary, rotation, position,
          focal_length, verbose_);
    } else if (camera_model_ == "DIVISION_UNDISTORTION" ||
               camera_model_ == "DOUBLE_SPHERE") {
      success_init = initialize_radial_undistortion_camera(
          correspondences, ransac_params_, ransac_summary, image_width,
          rotation, position, focal_length, radial_distortion, verbose_);
      if (camera_model_ == "DOUBLE_SPHERE") {
        focal_length *= 0.5;
      }
    } else {
      LOG(ERROR) << "THIS CAMERA MODEL (" << camera_model_
                 << ") DOES NOT EXIST!\n";
      LOG(ERROR) << "CHOOSE BETWEEN: LINEAR_PINHOLE, DIVISION_UNDISTORTION or "
                    "DOUBLE_SPHERE\n";
      return false;
    }

    // check if a very close by pose is already present
    bool take_image = true;
    for (int i = 0; i < saved_poses.size(); ++i) {
      if ((position - saved_poses[i]).norm() < grid_size_) {
        take_image = false;
        break;
      }
    }

    if (!take_image || !success_init) {
      continue;
    }

    saved_poses.push_back(position);

    theia::ViewId view_id =
        AddView(rotation, -rotation * position, focal_length, radial_distortion,
                image_width, image_height, timestamp_s);

    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      AddObservation(view_id, board_pt3_ids[i], corners[i]);
    }
  }
  // remove views that failed to initialize
  RemoveViewsReprojError(10.0);

  if (!RunCalibration()) {
    LOG(ERROR) << "Calibration failed.\n";
    return false;
  }

  // final reprojection error
  double reproj_error = 0;
  for (int i = 0; i < recon_calib_dataset_.NumViews(); ++i) {
    const double view_reproj_error = utils::GetReprojErrorOfView(
        recon_calib_dataset_, recon_calib_dataset_.ViewIds()[i]);
    reproj_error += view_reproj_error;
    if (verbose_) {
      LOG(INFO) << "View: " << recon_calib_dataset_.ViewIds()[i]
                << " RMSE reprojection error: " << view_reproj_error << "\n";
    }
  }

  const double total_repro_error =
      reproj_error / recon_calib_dataset_.NumViews();
  std::cout << "Final camera calibration reprojection error: "
            << total_repro_error << " from " << recon_calib_dataset_.NumViews()
            << " view." << std::endl;
  const theia::Camera cam =
      recon_calib_dataset_.View(recon_calib_dataset_.ViewIds()[0])->Camera();

  if (output_path != "") {
    theia::WriteReconstruction(recon_calib_dataset_,
                               output_path + ".calibdata");
    CHECK(io::write_camera_calibration(
        output_path + ".json", cam, scene_json["camera_fps"],
        recon_calib_dataset_.NumViews(), total_repro_error))
        << "Could not write calibration file.\n";
  }
}

} // namespace core
} // namespace OpenCamCalib
