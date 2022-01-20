/* Copyright (C) 2021 Steffen Urban
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <gflags/gflags.h>
#include <iostream>
#include <string>

#include "OpenCameraCalibrator/core/imu_camera_calibrator.h"
#include "OpenCameraCalibrator/io/read_camera_calibration.h"
#include "OpenCameraCalibrator/io/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/io/read_misc.h"
#include "OpenCameraCalibrator/io/read_telemetry.h"

#include "OpenCameraCalibrator/io/read_scene.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_ply_file.h"
#include "theia/sfm/reconstruction.h"

// Input/output files.
DEFINE_string(

    telemetry_json,
    "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(input_pose_dataset, "", "Path to pose dataset.");
DEFINE_string(input_corners,
              "",
              "Corners of the original imu to cam calibration video file.");
DEFINE_string(camera_calibration_json, "", "Camera calibration.");
DEFINE_string(gyro_to_cam_initial_calibration,
              "",
              "Initial gyro to camera calibration json.");
DEFINE_string(imu_intrinsics,
              "",
              "IMU intrinsics, scale and misalignment matrices. E.g. estimated "
              "with static_imu_calibration or from a datasheet.");
DEFINE_string(imu_bias_file, "/media/Data/work_projects/ImageStabelization/GoPro10Calibration/BatchCalib/dataset2/imu_bias/imu_bias_GX010018.json", "IMU bias json");
DEFINE_bool(global_shutter, false, "If camera has a global shutter.");

DEFINE_string(spline_error_weighting_json,
              "",
              "Path to spline error weighting data");
DEFINE_string(output_path, "", "");
DEFINE_bool(calibrate_cam_line_delay,
            false,
            "If camera rolling shutter line delay should be calibrated.");
DEFINE_string(result_output_json, "/media/Data/work_projects/ImageStabelization/GoPro10Calibration/BatchCalib/dataset2/cam_imu/cam_imu_calib_result_GX010020.json", "Path to result json file");
DEFINE_double(max_t, 1000., "Maximum nr of seconds to take");
DEFINE_bool(reestimate_biases,
            false,
            "If accelerometer and gyroscope biases should be estimated during "
            "spline optim");
DEFINE_double(gravity_const, 9.81, "gravity constant");
DEFINE_string(known_grav_dir_axis,
              "Z",
              "Possible values (X,Y,Z,UNKNOWN) if the gravity direction of "
              "your calibration board is exactly known (e.g. supplying Z we "
              "will fix gravity to [0,0,gravity_const]. UNKNOWN means it is "
              "not known and will be estimated.");
DEFINE_string(debug_video_path,
              "",
              "Load the video to display the reprojection error.");

using json = nlohmann::json;

using namespace cv;
using namespace theia;
using namespace OpenICC;
using namespace OpenICC::core;
using namespace OpenICC::utils;
using namespace OpenICC::io;

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Get pose dataset
  theia::Reconstruction pose_dataset;
  CHECK(theia::ReadReconstruction(FLAGS_input_pose_dataset, &pose_dataset))
      << "Could not read Reconstruction file.";
  nlohmann::json scene_json;
  CHECK(io::read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  theia::Camera camera;
  double fps;
  CHECK(io::read_camera_calibration(FLAGS_camera_calibration_json, camera, fps))
      << "Could not read camera calibration: " << FLAGS_camera_calibration_json;

  // fill tracks. we use the ones from pose estimation because they might have
  // been optimized (to account for non planarity of the target)
  theia::Reconstruction recon_calib_dataset;
  // io::scene_points_to_calib_dataset(scene_json, recon_calib_dataset);
  for (const auto& old_track_id : pose_dataset.TrackIds()) {
    recon_calib_dataset.AddTrack(old_track_id);
    theia::Track* new_track = recon_calib_dataset.MutableTrack(old_track_id);
    const theia::Track* old_track = pose_dataset.Track(old_track_id);
    Eigen::Vector4d* new_point = new_track->MutablePoint();
    for (int j = 0; j < 4; ++j) {
      (*new_point)[j] = old_track->Point()[j];
    }
  }
  for (const auto& view : scene_json["views"].items()) {
    const double timestamp_us = std::stod(view.key());
    const double timestamp_s = timestamp_us * US_TO_S;  // to seconds
    std::string view_name = std::to_string((uint64_t)timestamp_us);
    theia::ViewId view_id =
        recon_calib_dataset.AddView(view_name, 0, timestamp_s);

    theia::ViewId old_view_id = pose_dataset.ViewIdFromName(view_name);
    if (old_view_id == theia::kInvalidViewId) {
      recon_calib_dataset.RemoveView(view_id);
      continue;
    }
    theia::View* view_new = recon_calib_dataset.MutableView(view_id);
    theia::Camera* mutable_cam = view_new->MutableCamera();
    const theia::Camera cam_old = pose_dataset.View(old_view_id)->Camera();
    mutable_cam->SetOrientationFromAngleAxis(
        cam_old.GetOrientationAsAngleAxis());
    mutable_cam->SetPosition(cam_old.GetPosition());
    mutable_cam->SetFromCameraIntrinsicsPriors(
        camera.CameraIntrinsicsPriorFromIntrinsics());

    const auto image_points = view.value()["image_points"];
    for (const auto& img_pts : image_points.items()) {
      const int board_pt3_id = std::stoi(img_pts.key());
      const Eigen::Vector2d corner(
          Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
      Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
      theia::Feature feat(corner, cov);
      recon_calib_dataset.AddObservation(view_id, board_pt3_id, feat);
    }
  }

  // read gopro telemetry
  CameraTelemetryData telemetry_data;
  CHECK(ReadTelemetryJSON(FLAGS_telemetry_json, telemetry_data))
      << "Could not read: " << FLAGS_telemetry_json;

  // read a gyro to cam calibration json to initialize rotation between imu and
  // camera
  Eigen::Quaterniond imu2cam;
  double time_offset_imu_to_cam;
  CHECK(ReadIMU2CamInit(
      FLAGS_gyro_to_cam_initial_calibration, imu2cam, time_offset_imu_to_cam))
      << "Could not read: " << FLAGS_gyro_to_cam_initial_calibration;
  Sophus::SE3<double> T_i_c_init(imu2cam.conjugate(), Eigen::Vector3d(0, 0, 0));

  // Read a imu intrinsics
  ThreeAxisSensorCalibParams<double> acc_intr, gyr_intr;
  CHECK(ReadIMUIntrinsics(
      FLAGS_imu_intrinsics, FLAGS_imu_bias_file, acc_intr, gyr_intr))
      << "Could not open " << FLAGS_imu_intrinsics;
  std::cout << "Loaded IMU intrinsics.\n";
  CHECK(FLAGS_spline_error_weighting_json != "")
      << "You need to provide spline error weighting factors. Create with "
         "get_sew_for_dataset.py.";
  SplineWeightingData weight_data;
  CHECK(
      ReadSplineErrorWeighting(FLAGS_spline_error_weighting_json, weight_data))
      << "Could not open " << FLAGS_spline_error_weighting_json;

  double init_line_delay_us = 1. / fps / camera.ImageHeight();
  if (FLAGS_global_shutter) {
    init_line_delay_us = 0.0;
  }

  ImuCameraCalibrator imu_cam_calibrator;
  imu_cam_calibrator.BatchInitSpline(recon_calib_dataset,
                                     T_i_c_init,
                                     weight_data,
                                     time_offset_imu_to_cam,
                                     telemetry_data,
                                     init_line_delay_us,
                                     acc_intr,
                                     gyr_intr);
  const int grav_dir_axis = GravDirStringToInt(FLAGS_known_grav_dir_axis);
  int flags = SplineOptimFlags::SPLINE | SplineOptimFlags::T_I_C;
  if (FLAGS_reestimate_biases) {
    flags |= SplineOptimFlags::IMU_BIASES;
  }
  if (grav_dir_axis != -1) {
    Eigen::Vector3d grav_dir(0, 0, 0);
    grav_dir[grav_dir_axis] = FLAGS_gravity_const;
    imu_cam_calibrator.SetKnownGravityDir(grav_dir);
    std::cout << "Setting a-priori gravity direction supplied by the user to: "
              << grav_dir.transpose() << "\n";
  } else {
    flags |= SplineOptimFlags::GRAVITY_DIR;
  }

  double reproj_error = imu_cam_calibrator.Optimize(50, flags);

  double reproj_error_after_ld = reproj_error;
  if (FLAGS_calibrate_cam_line_delay && !FLAGS_global_shutter) {
    flags = SplineOptimFlags::CAM_LINE_DELAY;
    reproj_error_after_ld = imu_cam_calibrator.Optimize(10, flags);
  }
  LOG(INFO) << "Mean reprojection error " << reproj_error << "px\n";
  LOG(INFO) << "Mean reprojection error after line delay optim "
            << reproj_error_after_ld << "px\n";

  std::cout << "g: " << imu_cam_calibrator.trajectory_.GetGravity().transpose()
            << std::endl;
  std::cout << "accel_bias at time 0: "
            << imu_cam_calibrator.trajectory_.GetAcclBias(0).transpose()
            << std::endl;
  std::cout << "gyro_bias at time 0: "
            << imu_cam_calibrator.trajectory_.GetGyroBias(0).transpose()
            << std::endl;
  const Eigen::Quaterniond q_i_c =
      imu_cam_calibrator.trajectory_.GetT_i_c().so3().unit_quaternion();
  const Eigen::Vector3d t_i_c =
      imu_cam_calibrator.trajectory_.GetT_i_c().translation();
  const double calib_line_delay_us =
      imu_cam_calibrator.GetCalibratedRSLineDelay() * S_TO_US;
  std::cout << "T_i_c qw,qx,qy,qz: " << q_i_c.w() << " " << q_i_c.x() << " "
            << q_i_c.y() << " " << q_i_c.z() << std::endl;
  std::cout << "T_i_c t: " << t_i_c.transpose() << std::endl;
  std::cout << "T_i_c R: " << q_i_c.matrix() << std::endl;
  std::cout << "Initialized line delay [us]: " << init_line_delay_us * S_TO_US
            << "\n";
  std::cout << "Calibrated line delay [us]: " << calib_line_delay_us << "\n";
  nlohmann::json json_calibspline_results_out;

  json_calibspline_results_out["q_i_c"]["w"] = q_i_c.w();
  json_calibspline_results_out["q_i_c"]["x"] = q_i_c.x();
  json_calibspline_results_out["q_i_c"]["y"] = q_i_c.y();
  json_calibspline_results_out["q_i_c"]["z"] = q_i_c.z();
  json_calibspline_results_out["t_i_c"]["x"] = t_i_c[0];
  json_calibspline_results_out["t_i_c"]["y"] = t_i_c[1];
  json_calibspline_results_out["t_i_c"]["z"] = t_i_c[2];
  json_calibspline_results_out["final_reproj_error"] = reproj_error;
  json_calibspline_results_out["r3_dt"] = weight_data.dt_r3;
  json_calibspline_results_out["so3_dt"] = weight_data.dt_so3;
  json_calibspline_results_out["init_line_delay_us"] =
      init_line_delay_us * S_TO_US;
  json_calibspline_results_out["calib_line_delay_us"] = calib_line_delay_us;
  json_calibspline_results_out["time_offset_imu_to_cam_s"] =
      time_offset_imu_to_cam;

  std::vector<double> cam_timestamps_s = imu_cam_calibrator.GetCamTimestamps();
  std::sort(cam_timestamps_s.begin(), cam_timestamps_s.end(), std::less<>());

  aligned_map<double, Eigen::Vector3d> gyro_meas =
      imu_cam_calibrator.GetGyroMeasurements();
  aligned_map<double, Eigen::Vector3d> accl_meas =
      imu_cam_calibrator.GetAcclMeasurements();

  // Evaluate spline for all accelerometer and gyro and output them
  for (auto& g : gyro_meas) {
    const int64_t t_ns = g.first * S_TO_NS;
    const std::string t_ns_s = std::to_string(t_ns);
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_imu"]["x"] =
        g.second[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_imu"]["y"] =
        g.second[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_imu"]["z"] =
        g.second[2];
    // write out spline estimates
    Eigen::Vector3d gyro_spline;
    imu_cam_calibrator.trajectory_.GetAngularVelocity(t_ns, gyro_spline);
    const auto bias = imu_cam_calibrator.trajectory_.GetGyroBias(t_ns);
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_spline"]["x"] =
        gyro_spline[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_spline"]["y"] =
        gyro_spline[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_spline"]["z"] =
        gyro_spline[2];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_bias"]["x"] =
        bias[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_bias"]["y"] =
        bias[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["gyro_bias"]["z"] =
        bias[2];
  }
  for (auto& a : accl_meas) {
    const int64_t t_ns = a.first * S_TO_NS;
    const std::string t_ns_s = std::to_string(t_ns);
    // accelerometer
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_imu"]["x"] =
        a.second[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_imu"]["y"] =
        a.second[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_imu"]["z"] =
        a.second[2];
    // write out spline estimates
    Eigen::Vector3d accl_spline;
    imu_cam_calibrator.trajectory_.GetAcceleration(t_ns, accl_spline);
    const auto bias = imu_cam_calibrator.trajectory_.GetAcclBias(t_ns);
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_spline"]["x"] =
        accl_spline[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_spline"]["y"] =
        accl_spline[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_spline"]["z"] =
        accl_spline[2];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_bias"]["x"] =
        bias[0];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_bias"]["y"] =
        bias[1];
    json_calibspline_results_out["trajectory"][t_ns_s]["accl_bias"]["z"] =
        bias[2];
  }

  std::ofstream calibspline_output_json_file(FLAGS_result_output_json);
  calibspline_output_json_file << std::setw(4) << json_calibspline_results_out
                               << std::endl;
  calibspline_output_json_file.close();

  // read camera calibration
  theia::Reconstruction output_spline_recon;
  for (size_t i = 0; i < cam_timestamps_s.size(); ++i) {
    const int64_t t_ns = cam_timestamps_s[i] * S_TO_NS;
    Sophus::SE3d T_w_i;
    imu_cam_calibrator.trajectory_.GetPose(t_ns, T_w_i);
    Sophus::SE3d T_w_c = T_w_i * imu_cam_calibrator.trajectory_.GetT_i_c();
    theia::ViewId v_id_theia =
        output_spline_recon.AddView(std::to_string(t_ns), 0, t_ns);
    theia::View* view = output_spline_recon.MutableView(v_id_theia);
    view->SetEstimated(true);
    theia::Camera* camera_ptr = view->MutableCamera();
    camera_ptr->SetFromCameraIntrinsicsPriors(
        camera.CameraIntrinsicsPriorFromIntrinsics());
    camera_ptr->SetOrientationFromRotationMatrix(
        T_w_c.rotationMatrix().transpose());
    camera_ptr->SetPosition(T_w_c.translation());
  }

  // save spline recon as nvm to perform dense recon
  const Eigen::Vector3i cam_spline_color(0, 255, 0);
  const Eigen::Vector3i cam_recon_calib_color(255, 0, 0);
  CHECK(theia::WritePlyFile(FLAGS_output_path + "/" + "sparse_recon_spline.ply",
                            output_spline_recon,
                            cam_spline_color,
                            2));
  CHECK(theia::WritePlyFile(
      FLAGS_output_path + "/" + "sparse_recon_calib_dataset.ply",
      recon_calib_dataset,
      cam_recon_calib_color,
      2));

  if (FLAGS_debug_video_path != "") {
    nlohmann::json scene_json;
    CHECK(io::read_scene_bson(FLAGS_input_corners, scene_json))
        << "Failed to load " << FLAGS_input_corners;

    theia::Reconstruction recon_calib_dataset;

    io::scene_points_to_calib_dataset(scene_json, recon_calib_dataset);
    for (const auto& view : scene_json["views"].items()) {
      const double timestamp_us = std::stod(view.key());
      const double timestamp_s = timestamp_us * 1e-6;  // to seconds
      const auto image_points = view.value()["image_points"];
      std::string view_name = std::to_string((uint64_t)(timestamp_s * S_TO_NS));
      theia::ViewId view_id =
          recon_calib_dataset.AddView(view_name, 0, timestamp_s);

      for (const auto& img_pts : image_points.items()) {
        const int board_pt3_id = std::stoi(img_pts.key());
        const Eigen::Vector2d corner(
            Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
        Eigen::Matrix2d cov;
        cov << 0.5, 0, 0, 0.5;
        const theia::Feature feat(corner, cov);
        recon_calib_dataset.AddObservation(view_id, board_pt3_id, feat);
      }
    }
    VideoCapture input_video;
    input_video.open(FLAGS_debug_video_path);
    int cnt_wrong = 0;
    int nr_frames = 0;
    while (true) {
      Mat image;
      if (!input_video.read(image)) {
        cnt_wrong++;
        if (cnt_wrong > 500) break;
        continue;
      }

      const double timstamp_s =
          input_video.get(cv::CAP_PROP_POS_MSEC) * MS_TO_S;

      const int64_t t_ns = timstamp_s * S_TO_NS;

      const theia::ViewId view_id_spline =
          output_spline_recon.ViewIdFromName(std::to_string(t_ns));
      const theia::ViewId view_id_calib =
          recon_calib_dataset.ViewIdFromName(std::to_string(t_ns));

      if (view_id_spline == theia::kInvalidViewId &&
          view_id_calib == theia::kInvalidViewId)
        continue;

      theia::View* view_spline =
          output_spline_recon.MutableView(view_id_spline);
      const theia::View* view_calib = recon_calib_dataset.View(view_id_calib);
      if (!view_spline || !view_calib) continue;
      cv::resize(
          image, image, cv::Size(camera.ImageWidth(), camera.ImageHeight()));

      double reproj_error = 0.0;
      for (size_t i = 0; i < view_calib->TrackIds().size(); ++i) {
        theia::TrackId id = view_calib->TrackIds()[i];
        Eigen::Vector2d pixel;
        view_spline->Camera().ProjectPoint(
            recon_calib_dataset.Track(id)->Point(), &pixel);
        const theia::Feature measurement = (*view_calib->GetFeature(id));
        cv::drawMarker(image,
                       cv::Point(cvRound(pixel[0]), cvRound(pixel[1])),
                       cv::Scalar(0, 0, 255),
                       cv::MARKER_CROSS,
                       10,
                       1);

        reproj_error += (measurement.point_ - pixel).norm();
      }
      reproj_error /= (double)view_calib->TrackIds().size();
      cv::putText(
          image,
          "Reprojection error: " + std::to_string(reproj_error) + " pixel",
          cv::Point(20, 20),
          cv::FONT_HERSHEY_COMPLEX_SMALL,
          1.0,
          cv::Scalar(255, 0, 0));
      cv::imshow("spline reprojection", image);
      cv::waitKey(10);
      ++nr_frames;
      if (nr_frames > 100) break;
    }
  }
  return 0;
}
