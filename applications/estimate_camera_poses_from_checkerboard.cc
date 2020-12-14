#include <dirent.h>
#include <gflags/gflags.h>
#include <time.h>
#include <algorithm>
#include <chrono>  // NOLINT
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/imu/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"
#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/io/read_camera_calibration.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

using namespace cv;
using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
using namespace OpenCamCalib::io;

// Input/output files.
DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_calibration_json, "",
              "Path to camera calibration json.");
DEFINE_string(output_pose_dataset, "",
              "Path to write the pose calibration dataset to.");

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  nlohmann::json scene_json;
  CHECK(read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  theia::Reconstruction pose_dataset;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
  scene_points_to_calib_dataset(scene_json, pose_dataset, charuco_id_to_theia_track_id);

  // read camera calibration
  theia::Camera camera;
  double fps;
  CHECK(!read_camera_calibration(FLAGS_camera_calibration_json, camera, fps)) <<"Could not read camera calibratio: "<<FLAGS_camera_calibration_json;

  const int min_number_detected_corners = 2;
  theia::RansacParameters ransac_params;
  ransac_params.error_thresh = 0.5;
  ransac_params.failure_probability = 0.001;
  ransac_params.min_iterations = 30;
  ransac_params.use_mle = true;

  // bundle adjustment options
  theia::BundleAdjustmentOptions ba_options;
  ba_options.fix_tracks = true;
  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options.robust_loss_width = 1.345;
  ba_options.intrinsics_to_optimize = theia::OptimizeIntrinsicsType::NONE;

  double total_repro_error = 0.0;
  int processed_frames = 0;
  // iterate views and estimate poses
  const auto views = scene_json["views"];
  for (const auto &view : views.items()) {
    const double timstamp_us = std::stod(view.key()); // to seconds
    const double timstamp_s = timstamp_us * 1e-6;     // to seconds
    const auto image_points = view.value()["image_points"];
    std::vector<int> board_pt3_ids;
    aligned_vector<Eigen::Vector2d> corners;
    std::vector<theia::FeatureCorrespondence2D3D> correspondences;

    for (const auto &img_pts : image_points.items()) {
      const int board_pt3_id = std::stoi(img_pts.key());
      board_pt3_ids.push_back(std::stoi(img_pts.key()));
      const Eigen::Vector2d corner(Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
      corners.push_back(corner);
      Eigen::Vector3d undist_pt = camera.PixelToNormalizedCoordinates(corner);
      undist_pt /= undist_pt[2];

      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(board_pt3_id)->second;
      const Eigen::Vector4d track =
          pose_dataset.Track(track_id)->Point();

      theia::FeatureCorrespondence2D3D correspondence;
      correspondence.world_point = track.hnormalized();
      correspondence.feature[0] = undist_pt[0];
      correspondence.feature[1] = undist_pt[1];
      correspondences.push_back(correspondence);
    }

    theia::CalibratedAbsolutePose pose;
    theia::RansacSummary ransac_summary;
    theia::EstimateCalibratedAbsolutePose(ransac_params, theia::RansacType::RANSAC, correspondences, &pose, &ransac_summary);

    theia::ViewId view_id = pose_dataset.AddView(
        std::to_string((uint64_t)timstamp_us), 0, timstamp_s);
    theia::View *theia_view = pose_dataset.MutableView(view_id);
    theia_view->SetEstimated(true);

    theia::Camera *cam = theia_view->MutableCamera();
    cam->SetImageSize(camera.ImageWidth(), camera.ImageHeight());
    cam->SetPrincipalPoint(camera.ImageWidth() / 2.0, camera.ImageHeight() / 2.0);
    cam->SetPosition(pose.position);
    cam->SetOrientationFromRotationMatrix(pose.rotation);
    cam->SetFocalLength(1.0);

    for (int i=0; i < ransac_summary.inliers.size(); ++i) {
        int inlier = ransac_summary.inliers[i];

        theia::TrackId track_id =
            charuco_id_to_theia_track_id.find(board_pt3_ids[inlier])->second;
        pose_dataset.AddObservation(view_id, track_id, corners[inlier]);
    }

    // optimize pose
    theia::BundleAdjustmentSummary summary =
        theia::BundleAdjustView(ba_options, view_id, &pose_dataset);

    // test back projection
    double reproj_error = 0;
    for (size_t i = 0; i < pose_dataset.View(view_id)->TrackIds().size(); ++i) {
      theia::TrackId track_id = pose_dataset.View(view_id)->TrackIds()[i];
      const theia::Track* track = pose_dataset.Track(track_id);
      Eigen::Vector2d reproj_point;
      pose_dataset.View(view_id)->Camera().ProjectPoint(track->Point(), &reproj_point);
      reproj_error += (reproj_point - (*pose_dataset.View(view_id)->GetFeature(track_id))).norm();
    }
    const double repro_error_n = reproj_error / pose_dataset.View(view_id)->TrackIds().size();
    if (repro_error_n > 3.0) {
        std::cout<<"Removing view as reprojection error is to big: "<<repro_error_n<<"\n";
        pose_dataset.RemoveView(view_id);
    }
    total_repro_error += repro_error_n;
    ++processed_frames;
  }

  total_repro_error /= (double)processed_frames;
  std::cout<<"Mean reprojection error all images: "<<total_repro_error<<"\n";
  theia::WriteReconstruction(pose_dataset, FLAGS_output_pose_dataset);

  return 0;
}
