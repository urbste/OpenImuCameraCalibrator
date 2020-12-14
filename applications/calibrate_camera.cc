#include <algorithm>
#include <chrono> // NOLINT
#include <fstream>
#include <gflags/gflags.h>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <theia/io/reconstruction_writer.h>
#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/io/write_camera_calibration.h"
#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;
using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
using namespace OpenCamCalib::io;

DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_model_to_calibrate, "DOUBLE_SPHERE",
              "What camera model do you want to calibrate. Options:"
              "LINEAR_PINHOLE,DIVISION_UNDISTORTION,DOUBLE_SPHERE");
DEFINE_string(save_path_calib_dataset, "",
              "Where to save the recon dataset to.");
DEFINE_double(grid_size, 0.04,
              "Only take images that are at least grid_size apart");
DEFINE_bool(optimize_scene_points, false,
              "If in the end also the scene points should be adjusted. (if the board is not planar)");
DEFINE_bool(verbose, false, "If more stuff should be printed");
//--input_corners=/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/GoPro6/1080_30/dataset1/cam/GH016354_corners.uson
//--save_path_calib_dataset=/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/
//--camera_model_to_calibrate=DOUBLE_SPHERE --verbose=1

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  nlohmann::json scene_json;
  CHECK(read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  theia::Reconstruction recon_calib_dataset;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
  scene_points_to_calib_dataset(scene_json, recon_calib_dataset,
                                charuco_id_to_theia_track_id);

  theia::RansacParameters ransac_params;
  ransac_params.error_thresh = 0.5;
  ransac_params.failure_probability = 0.001;
  ransac_params.min_iterations = 30;
  ransac_params.use_mle = true;

  const int image_width = scene_json["image_width"];
  const int image_height = scene_json["image_height"];
  // initial principal point
  const double px = static_cast<double>(image_width) / 2.0;
  const double py = static_cast<double>(image_height) / 2.0;

  aligned_vector<Eigen::Vector3d> saved_poses;
  std::map<theia::ViewId, double> ids_to_remove_after_init;

  // iterate views and estimate poses
  const auto views = scene_json["views"];
  for (const auto &view : views.items()) {
    const double timstamp_us = std::stod(view.key()); // to seconds
    const double timstamp_s = timstamp_us * 1e-6;     // to seconds
    const auto image_points = view.value()["image_points"];
    std::vector<int> board_pt3_ids;
    aligned_vector<Eigen::Vector2d> corners;
    for (const auto &img_pts : image_points.items()) {
      board_pt3_ids.push_back(std::stoi(img_pts.key()));
      corners.push_back(
          Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
    }

    std::cout << "timestamp: " << timstamp_s << "\n";
    // initialize cam pose
    std::vector<theia::FeatureCorrespondence2D3D> correspondences(
        board_pt3_ids.size());
    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      theia::FeatureCorrespondence2D3D correspondence;
      correspondence.feature[0] = corners[i][0] - px;
      correspondence.feature[1] = corners[i][1] - py;
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(board_pt3_ids[i])->second;
      const Eigen::Vector4d track =
          recon_calib_dataset.Track(track_id)->Point();
      correspondence.world_point = track.hnormalized();
      correspondences[i] = correspondence;
    }

    theia::RansacSummary ransac_summary;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    bool success_init = false;
    double focal_length, radial_distortion;
    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
      success_init = initialize_pinhole_camera(
          correspondences, ransac_params, ransac_summary, rotation, position,
          focal_length, FLAGS_verbose);
    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
               FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
      success_init = initialize_radial_undistortion_camera(
          correspondences, ransac_params, ransac_summary, image_width, rotation,
          position, focal_length, radial_distortion, FLAGS_verbose);
      if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
        focal_length *= 0.5;
      }
    } else {
      std::cout << "THIS CAMERA MODEL (" << FLAGS_camera_model_to_calibrate
                << ") DOES NOT EXIST!\n";
      std::cout << "CHOOSE BETWEEN: LINEAR_PINHOLE, DIVISION_UNDISTORTION or "
                   "DOUBLE_SPHERE\n";
      return 0;
    }

    // check if a very close by pose is already present
    bool take_image = true;
    for (int i = 0; i < saved_poses.size(); ++i) {
      if ((position - saved_poses[i]).norm() < FLAGS_grid_size) {
        take_image = false;
        break;
      }
    }

    if (!take_image || !success_init) {
      continue;
    }

    saved_poses.push_back(position);

    // fill charucoCorners to theia reconstruction
    theia::ViewId view_id = recon_calib_dataset.AddView(
        std::to_string((uint64_t)timstamp_us), 0, timstamp_s);
    theia::View *theia_view = recon_calib_dataset.MutableView(view_id);
    theia_view->SetEstimated(true);

    theia::Camera *cam = theia_view->MutableCamera();
    cam->SetImageSize(image_width, image_height);
    cam->SetPrincipalPoint(image_width / 2.0, image_height / 2.0);
    cam->SetPosition(position);
    cam->SetOrientationFromRotationMatrix(rotation);
    cam->SetFocalLength(focal_length);
    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::PINHOLE);
    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
      cam->mutable_intrinsics()
          [theia::DivisionUndistortionCameraModel::InternalParametersIndex::
               RADIAL_DISTORTION_1] = radial_distortion;
    } else if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
      cam->mutable_intrinsics()
          [theia::DoubleSphereCameraModel::InternalParametersIndex::XI] = 1.0;
      cam->mutable_intrinsics()
          [theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] =
          0.0;
    }

    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(board_pt3_ids[i])->second;
      recon_calib_dataset.AddObservation(view_id, track_id, corners[i]);
    }

    const double init_reproj_error =
        GetReprojErrorOfView(recon_calib_dataset, view_id);
    if (FLAGS_verbose) {
      std::cout << "View init reprojection error: " << init_reproj_error
                << std::endl;
    }
    if (init_reproj_error > 25.0) {
      ids_to_remove_after_init[view_id] = init_reproj_error;
    }
  }

  if (recon_calib_dataset.NumViews() < 10) {
    std::cout << "Not enough views left for proper calibration!" << std::endl;
    return 0;
  }

  std::cout << "Using " << recon_calib_dataset.NumViews()
            << " in bundle adjustment\n";
  // bundle adjust everything
  theia::BundleAdjustmentOptions ba_options;
  ba_options.fix_tracks = true;
  ba_options.verbose = true;
  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options.robust_loss_width = 1.345;

  /////////////////////////////////////////////////
  /// 1. Optimize focal length and radial distortion, keep principal point fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = false;
  ba_options.constant_camera_position = false;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
  std::cout << "Bundle adjusting focal length and radial distortion. Keeping "
               "cam position fixed.\n";
  theia::BundleAdjustmentSummary summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  // reproj error per view, remove some views which have a high error
  std::map<theia::ViewId, double> ids_to_remove;
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    const theia::ViewId v_id = recon_calib_dataset.ViewIds()[i];
    const double view_reproj_error =
        GetReprojErrorOfView(recon_calib_dataset, v_id);
    if (view_reproj_error > 2.0) {
      ids_to_remove[v_id] = view_reproj_error;
    }
  }
  for (auto v_id : ids_to_remove) {
    recon_calib_dataset.RemoveView(v_id.first);
    std::cout << "Removed view: " << v_id.first
              << " with RMSE reproj error: " << v_id.second << "\n";
  }

  /////////////////////////////////////////////////
  /// 2. Optimize principal point keeping everything else fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = true;
  ba_options.constant_camera_position = true;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS;

  summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  if (recon_calib_dataset.NumViews() < 8) {
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
  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
  summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);
  ids_to_remove.clear();
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    const theia::ViewId v_id = recon_calib_dataset.ViewIds()[i];
    const double view_reproj_error =
        GetReprojErrorOfView(recon_calib_dataset, v_id);
    if (view_reproj_error > 1.0) {
      ids_to_remove[v_id] = view_reproj_error;
    }
  }
  for (auto v_id : ids_to_remove) {
    recon_calib_dataset.RemoveView(v_id.first);
    std::cout << "Removed view in second pass: " << v_id.first
              << " with RMSE reproj error: " << v_id.second << "\n";
  }
  summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);
  PrintResult(FLAGS_camera_model_to_calibrate, recon_calib_dataset);

  if (FLAGS_optimize_scene_points) {
      ba_options.fix_tracks = false;
      summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);
  }

  theia::WriteReconstruction(recon_calib_dataset,
                             FLAGS_save_path_calib_dataset + ".calibdata");

  // final reprojection error
  double reproj_error = 0;
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    const double view_reproj_error = GetReprojErrorOfView(
        recon_calib_dataset, recon_calib_dataset.ViewIds()[i]);
    reproj_error += view_reproj_error;
    if (FLAGS_verbose) {
      std::cout << "View: " << recon_calib_dataset.ViewIds()[i]
                << " RMSE reprojection error: " << view_reproj_error << "\n";
    }
  }

  const double total_repro_error =
      reproj_error / recon_calib_dataset.NumViews();
  std::cout << "Final camera calibration reprojection error: "
            << total_repro_error << " from " << recon_calib_dataset.NumViews()
            << " view." << std::endl;
  const theia::Camera cam =
      recon_calib_dataset.View(recon_calib_dataset.ViewIds()[0])->Camera();

  CHECK(write_camera_calibration(
      FLAGS_save_path_calib_dataset + ".json", cam, scene_json["camera_fps"],
      recon_calib_dataset.NumViews(), summary.final_cost, reproj_error))
      << "Could not write calibration file.\n";

  return 0;
}
