#include <algorithm>
#include <chrono> // NOLINT
#include <dirent.h>
#include <fstream>
#include <gflags/gflags.h>
#include <iostream>
#include <ostream>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/imu/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include "OpenCameraCalibrator/spline/measurements/accelerometer_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/gyroscope_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/orientation_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/position_measurement.h"
//#include "OpenCameraCalibrator/spline/measurements/static_rscamera_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/static_rscamera_measurement_xyz.h"
#include "OpenCameraCalibrator/spline/sensors/atan_camera.h"
#include "OpenCameraCalibrator/spline/sensors/basic_imu.h"
#include "OpenCameraCalibrator/spline/sensors/camera.h"
#include "OpenCameraCalibrator/spline/sensors/constant_bias_imu.h"
#include "OpenCameraCalibrator/spline/sensors/division_undistortion_camera.h"
#include "OpenCameraCalibrator/spline/sensors/imu.h"
#include "OpenCameraCalibrator/spline/sensors/pinhole_camera.h"
#include "OpenCameraCalibrator/spline/sensors/sensors.h"
#include "OpenCameraCalibrator/spline/sfm/sfm.h"
#include "OpenCameraCalibrator/spline/trajectories/spline_base.h"
#include "OpenCameraCalibrator/spline/trajectories/split_trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/uniform_r3_spline_trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/uniform_se3_spline_trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/uniform_so3_spline_trajectory.h"
#include "OpenCameraCalibrator/spline/trajectory_estimator.h"

#include "OpenCameraCalibrator/utils/json.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_ply_file.h"
#include "theia/sfm/reconstruction.h"
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

// Input/output files.
// Input/output files.
DEFINE_string(
    gopro_telemetry_json, "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(input_calibration_dataset, "",
              "Path to input calibration dataset.");
DEFINE_string(gyro_to_cam_initial_calibration, "",
              "Initial gyro to camera calibration json.");
DEFINE_string(imu_bias_file, "", "IMU bias json");
DEFINE_string(spline_error_weighting_json, "",
              "Path to spline error weighting data");
DEFINE_string(output_path, "", "Output path for results");
DEFINE_string(result_output_json, "", "Path to result json file");
DEFINE_double(max_t, 1000., "Maximum nr of seconds to take");

namespace TT = kontiki::trajectories;
namespace M = kontiki::measurements;
namespace S = kontiki::sensors;
namespace SFM = kontiki::sfm;

using TrajClass = TT::UniformSE3SplineTrajectory;
using SO3TrajClass = TT::UniformSO3SplineTrajectory;
using R3TrajClass = TT::UniformR3SplineTrajectory;
using SplitTrajClass = TT::SplitTrajectory;
using AtanCameraClass = S::AtanCamera;
using DivisionUndistortionCameraClass = S::DivisionUndistortionCamera;
using PinholeCameraClass = S::PinholeCamera;

using IMUClass = S::ConstantBiasImu;
using Landmark = SFM::LandmarkXYZ;
using ViewKontiki = SFM::ViewXYZ;
using Observation = SFM::ObservationXYZ;
using CamMeasurementPinhole =
    M::StaticRsCameraMeasurementXYZ<PinholeCameraClass>;
//using CamMeasurementAtan = M::StaticRsCameraMeasurement<AtanCameraClass>;
using CamMeasurementDivUndist =
    M::StaticRsCameraMeasurementXYZ<DivisionUndistortionCameraClass>;
using GyroMeasurement = M::GyroscopeMeasurement<IMUClass>;
using AccMeasurement = M::AccelerometerMeasurement<IMUClass>;
using PositionMeasurement = M::PositionMeasurement;

using json = nlohmann::json;

using namespace cv;
using namespace theia;

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // IMU Bias
  Eigen::Vector3d accl_bias, gyro_bias;
  CHECK(OpenCamCalib::ReadIMUBias(FLAGS_imu_bias_file, gyro_bias, accl_bias))
      << "Could not open " << FLAGS_imu_bias_file;

  // Load camera calibration reconstuction.
  theia::Reconstruction calib_dataset;
  CHECK(theia::ReadReconstruction(FLAGS_input_calibration_dataset,
                                  &calib_dataset))
      << "Could not read Reconstruction file.";
  // read gopro telemetry
  OpenCamCalib::CameraTelemetryData telemetry_data;
  CHECK(OpenCamCalib::ReadGoProTelemetry(FLAGS_gopro_telemetry_json,
                                         telemetry_data))
      << "Could not read: " << FLAGS_gopro_telemetry_json;

  // read a gyro to cam calibration json to initialize rotation between imu and
  // camera
  Eigen::Quaterniond imu2cam;
  double time_offset_imu_to_cam;
  CHECK(OpenCamCalib::ReadIMU2CamInit(FLAGS_gyro_to_cam_initial_calibration,
                                      imu2cam, time_offset_imu_to_cam))
      << "Could not read: " << FLAGS_gyro_to_cam_initial_calibration;

  CHECK(FLAGS_spline_error_weighting_json != "")
      << "You need to provide spline error weighting factors. Create with "
         "get_sew_for_dataset.py.";
  OpenCamCalib::SplineWeightingData weight_data;
  CHECK(OpenCamCalib::ReadSplineErrorWeighting(
      FLAGS_spline_error_weighting_json, weight_data))
      << "Could not open " << FLAGS_spline_error_weighting_json;

  // get one view for calibration info
  const std::vector<theia::ViewId> vids = calib_dataset.ViewIds();
  const theia::Camera camera = calib_dataset.View(vids[0])->Camera();

  const double dt_r3 = weight_data.dt_r3;
  const double dt_so3 = weight_data.dt_so3;

  // Number of cameras.
  const auto &view_ids = calib_dataset.ViewIds();
  std::unordered_map<ViewId, int> view_id_to_index;
  std::unordered_map<ViewId, std::shared_ptr<ViewKontiki>> kontiki_views;
  std::vector<double> timestamps;
  // get all timestamps and find smallest one
  // Output each camera.
  for (const ViewId view_id : view_ids) {
    const View &view = *calib_dataset.View(view_id);
    double timestamp = std::stod(view.Name());
    timestamps.push_back(timestamp);
  }

  // find smallest timestamp
  auto result = std::minmax_element(timestamps.begin(), timestamps.end());
  double t0 = timestamps[result.first - timestamps.begin()];// - 1 * dt_so3;
  double tend = timestamps[result.second - timestamps.begin()];// + 1 * dt_r3;
  const double max_t_offset = std::max(dt_so3, dt_r3);

  // split trajectory
  std::shared_ptr<SO3TrajClass> so3_traj_spline =
      std::make_shared<SO3TrajClass>(dt_so3, t0);
  std::shared_ptr<R3TrajClass> r3_traj_spline =
      std::make_shared<R3TrajClass>(dt_r3, t0);

  std::shared_ptr<SplitTrajClass> split_traj_spline =
      std::make_shared<SplitTrajClass>(r3_traj_spline, so3_traj_spline);

  kontiki::TrajectoryEstimator<SplitTrajClass> traj_spline_estimator(
      split_traj_spline);

  for (const ViewId view_id : view_ids) {
    const int current_index = view_id_to_index.size();
    view_id_to_index[view_id] = current_index;

    const View &view = *calib_dataset.View(view_id);
    double timestamp = std::stod(view.Name());
    if (timestamp >= tend || timestamp < t0)
      continue;
    kontiki_views[view_id] = std::make_shared<ViewKontiki>(view_id, timestamp);
  }

  // Number of points.
  const auto &track_ids = calib_dataset.TrackIds();
  // Output each point.
  std::vector<std::shared_ptr<Landmark>> kontiki_landmarks;
  Eigen::Matrix3d K;
  int lauf = 0;
  std::vector<double> feat_depths;
  for (const TrackId track_id : track_ids) {
    const Track *track = calib_dataset.Track(track_id);

    // Output the observations of this 3D point.
    const auto &views_observing_track = track->ViewIds();
    int nr_views = 0;
    for (const ViewId &view_id : views_observing_track) {
      if (kontiki_views.find(view_id) == kontiki_views.end())
        continue;
      nr_views++;
    }
    // if the landmark has more than 2 views add it to estimation
    if (nr_views >= 2) {
      // we know our landmarks for calibration and lock them!
      kontiki_landmarks.push_back(std::make_shared<Landmark>());
      kontiki_landmarks[kontiki_landmarks.size() - 1]->set_point(
          track->Point());
      //kontiki_landmarks[kontiki_landmarks.size() - 1]->Lock(true);

      nr_views = 0;
      for (const ViewId &view_id : views_observing_track) {
        const View *view = calib_dataset.View(view_id);
        const Feature feature = (*view->GetFeature(track_id));
        if (nr_views == 0) {
          // only do it for view 0
          Eigen::Vector2d tmp;
          feat_depths.push_back(
              view->Camera().ProjectPoint(track->Point(), &tmp));
        }
        auto cur_view_it = kontiki_views.find(view_id);
        if (cur_view_it == kontiki_views.end()) {
          continue;
        }
        cur_view_it->second->CreateObservation(
            kontiki_landmarks[kontiki_landmarks.size() - 1], feature);

        ++nr_views;
      }
    }
    ++lauf;
  }

  // set landmark references to first observation
  // set initial inverse depth here?!
  for (size_t l = 0; l < kontiki_landmarks.size(); ++l) {
    kontiki_landmarks[l]->set_reference(
        kontiki_landmarks[l]->observations()[0]);
    kontiki_landmarks[l]->Lock(false);
  }
  camera.GetCalibrationMatrix(&K);
  // K.setIdentity();
  std::shared_ptr<DivisionUndistortionCameraClass> cam_kontiki =
      std::make_shared<DivisionUndistortionCameraClass>(
          camera.ImageWidth(), camera.ImageHeight(), 1./30., K);
  cam_kontiki->set_distortion(
      camera.intrinsics()[theia::DivisionUndistortionCameraModel::
                              InternalParametersIndex::RADIAL_DISTORTION_1]);
  std::cout << "Initial rotation matrix: " << imu2cam.toRotationMatrix()
            << std::endl;
  cam_kontiki->LockRelativeOrientation(true);
  cam_kontiki->LockRelativePosition(true);
  cam_kontiki->LockTimeOffset(true);
  cam_kontiki->set_time_offset(0.0);

  Eigen::Vector3d zero;
  zero.setZero();
  std::shared_ptr<IMUClass> imu_kontiki =
      std::make_shared<IMUClass>(zero, zero);
  imu_kontiki->LockAccelerometerBias(true);
  imu_kontiki->LockTimeOffset(true);
  imu_kontiki->LockGyroscopeBias(true);
  imu_kontiki->set_relative_orientation(imu2cam);

  int sub_sample_imu = 1;
  // add imu
  std::vector<std::shared_ptr<AccMeasurement>> acc_measurements;
  std::vector<std::shared_ptr<GyroMeasurement>> gyr_measurements;

  std::cout << "Trajectory start time: " << t0 << " tend: " << tend
            << std::endl;
  std::cout << "Knot spacing SO3 / R3: " << dt_so3 << "/" << dt_r3 << "\n";
  std::cout << "Error weighting SO3 / R3: " << 1. / weight_data.var_so3 << "/"
            << 1. / weight_data.var_r3 << "\n";

  Sophus::SE3<double> T_i_c_init(imu2cam.conjugate(), Eigen::Vector3d(0, 0, 0));
  Sophus::SE3<double> T_a_c_i = Sophus::SE3<double>(
      calib_dataset.View(view_ids[0])
          ->Camera()
          .GetOrientationAsRotationMatrix()
          .transpose(),
      calib_dataset.View(view_ids[0])->Camera().GetPosition());
  Sophus::SE3d T_w_i_init = T_a_c_i * T_i_c_init.inverse();

  //r3_traj_spline->ExtendTo(tend, T_w_i_init.translation());
  //so3_traj_spline->ExtendTo(tend, T_w_i_init.unit_quaternion());
  r3_traj_spline->ExtendTo(tend, Eigen::Vector3d(0.0,0.0,0.0));
  so3_traj_spline->ExtendTo(tend, Eigen::Quaterniond(1.0,0.0,0.0,0.0));

  OpenCamCalib::aligned_vector<Eigen::Vector3d> accl_data;
  for (int i = 0; i < telemetry_data.accelerometer.acc_measurement.size();
       ++i) {
    if (i % sub_sample_imu == 0) {
      double t = telemetry_data.accelerometer.timestamp_ms[i] / 1000.0 +
                 time_offset_imu_to_cam;
      if (t > tend || t < t0)
        continue;
      std::shared_ptr<AccMeasurement> acc_measurement =
          std::make_shared<AccMeasurement>(
              imu_kontiki, t,
              telemetry_data.accelerometer.acc_measurement[i] + accl_bias,
              1.);
      accl_data.push_back(telemetry_data.accelerometer.acc_measurement[i] +
                          accl_bias);
      acc_measurements.push_back(acc_measurement);
      traj_spline_estimator.AddMeasurement(
          acc_measurements[acc_measurements.size() - 1]);
    }
  }
  OpenCamCalib::aligned_vector<Eigen::Vector3d> gyro_data;
  for (int i = 0; i < telemetry_data.gyroscope.gyro_measurement.size(); ++i) {
    if (i % sub_sample_imu == 0) {
      double t = telemetry_data.gyroscope.timestamp_ms[i] / 1000.0 +
                 time_offset_imu_to_cam;
      if (t > tend || t < t0)
        continue;
      std::shared_ptr<GyroMeasurement> gyr_measurement =
          std::make_shared<GyroMeasurement>(
              imu_kontiki, t,
              telemetry_data.gyroscope.gyro_measurement[i] + gyro_bias,
              1. );
      gyro_data.push_back(telemetry_data.gyroscope.gyro_measurement[i] +
                          gyro_bias);
      gyr_measurements.push_back(gyr_measurement);
      traj_spline_estimator.AddMeasurement(
          gyr_measurements[gyr_measurements.size() - 1]);
    }
  }
  // iterate all observations and create measurements
  std::vector<std::shared_ptr<CamMeasurementDivUndist>> measurements;
  for (auto it : kontiki_views) {
    auto kon_view = it.second;
    std::vector<std::shared_ptr<Observation>> observations =
        kon_view->observations();
    for (int i = 0; i < observations.size(); ++i) {
      measurements.push_back(std::make_shared<CamMeasurementDivUndist>(
          cam_kontiki, observations[i], 1.345));
      traj_spline_estimator.AddMeasurement(
          measurements[measurements.size() - 1]);
    }
  }
  traj_spline_estimator.Solve(100, true, -1);

  std::cout << "relative translation: " << cam_kontiki->relative_position()
            << "\n";
  std::cout << "relative_orientation: "
            << cam_kontiki->relative_orientation().toRotationMatrix() << "\n";

  std::cout << "Getting poses at camera timestamps on spline.";
  double mean_reproj_error = 0.0;
  uint32_t num_obs = 0;
  theia::Reconstruction reconstruction_out;
  for (const theia::ViewId view_id : view_ids) {
    theia::View *view_old_recon = calib_dataset.MutableView(view_id);
    const double timestamp_old_recon = std::stod(view_old_recon->Name());
    if (timestamp_old_recon > tend || timestamp_old_recon < t0)
      continue;
    theia::Camera *camera_old_recon = view_old_recon->MutableCamera();
    Eigen::Vector3d pos =
        r3_traj_spline
            ->Evaluate(timestamp_old_recon,
                       kontiki::trajectories::EvaluationFlags::EvalPosition)
            ->position;
    Eigen::Quaterniond q =
        so3_traj_spline
            ->Evaluate(timestamp_old_recon,
                       kontiki::trajectories::EvaluationFlags::EvalOrientation)
            ->orientation;

    auto k_view = kontiki_views.find(view_id);
    if (k_view == kontiki_views.end())
        continue;

    std::vector<std::shared_ptr<Observation>> obs_kont = k_view->second->observations();
    const int flags = kontiki::trajectories::EvaluationFlags::EvalPosition | kontiki::trajectories::EvaluationFlags::EvalOrientation;
    for (int i=0; i < obs_kont.size(); ++i) {

        double time_offset = cam_kontiki->time_offset();
        double row_delta = cam_kontiki->readout() / cam_kontiki->rows();
        double t_obs = obs_kont[i]->view()->t0() + time_offset + obs_kont[i]->v() * row_delta;

        auto eval_obs = split_traj_spline->Evaluate(t_obs, flags);

        Eigen::Vector3d landmark_world = obs_kont[i]->landmark()->get_point().hnormalized();
        Eigen::Vector3d X_in_spline = eval_obs->orientation.conjugate() * (landmark_world - eval_obs->position);
        Eigen::Vector3d X_camera = cam_kontiki->FromTrajectory(X_in_spline);

        Eigen::Vector2d error = cam_kontiki->Project(X_camera) - obs_kont[i]->uv();
        mean_reproj_error += error.norm();
        ++num_obs;
    }
    // add view to new reconstruction
    theia::ViewId v_id_theia = reconstruction_out.AddView(
        view_old_recon->Name(), 0, timestamp_old_recon);
    theia::View *view = reconstruction_out.MutableView(v_id_theia);
    view->SetEstimated(true);
    theia::Camera *camera = view->MutableCamera();
    camera->MutableCameraIntrinsics() =
        camera_old_recon->MutableCameraIntrinsics();
    camera->SetOrientationFromRotationMatrix(q.toRotationMatrix());
    camera->SetPosition(pos);
  }

  mean_reproj_error /= (double)num_obs;
  std::cout<<"Mean reprojection error: "<<mean_reproj_error<<std::endl;

  const Eigen::Vector3d p_ct = cam_kontiki->relative_position();
  const Eigen::Quaterniond q_ct = cam_kontiki->relative_orientation();
  std::cout << "Calculating landmarks\n";
  for (int i = 0; i < kontiki_landmarks.size(); ++i) {
    std::shared_ptr<Landmark> l = kontiki_landmarks[i];
    std::shared_ptr<Observation> o = l->reference();
    std::shared_ptr<ViewKontiki> v = o->view();

    Eigen::Vector3d pos =
        r3_traj_spline
            ->Evaluate(v->t0(),
                       kontiki::trajectories::EvaluationFlags::EvalPosition)
            ->position;
    Eigen::Quaterniond q =
        so3_traj_spline
            ->Evaluate(v->t0(),
                       kontiki::trajectories::EvaluationFlags::EvalOrientation)
            ->orientation;
    //    double inverse_depth = l->inverse_depth();
    //    Eigen::Vector2d y = o->uv();
    //    Eigen::Vector3d yh = cam_kontiki->Unproject(y);
    //    Eigen::Vector3d X_ref = q_ct.conjugate() * (yh - inverse_depth *
    //    p_ct); Eigen::Vector3d X = q.toRotationMatrix() * X_ref + pos *
    //    inverse_depth;
    theia::TrackId track_id = reconstruction_out.AddTrack();
    theia::Track *track = reconstruction_out.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d *point = track->MutablePoint();
    //    (*point)(0) = X(0);
    //    (*point)(1) = X(1);
    //    (*point)(2) = X(2);
    //    (*point)(3) = 1.0;
    (*point)(0) = l->get_point()[0];
    (*point)(1) = l->get_point()[1];
    (*point)(2) = l->get_point()[2];
    (*point)(3) = l->get_point()[3];
    for (int j = 0; j < kontiki_views.size(); ++j)
      track->AddView(j);
    Eigen::Matrix<uint8_t, 3, 1> *color = track->MutableColor();
    (*color)(0) = 0;
    (*color)(1) = 0;
    (*color)(2) = 0;
  }

  // Evaluate spline for all accelerometer and gyro and output them
  nlohmann::json json_calib_results_out;
  for (size_t t = 0; t < telemetry_data.gyroscope.gyro_measurement.size();
       ++t) {
    Eigen::Vector3d gyro_spline, accl_spline;
    gyro_spline.setZero();
    accl_spline.setZero();

    const double t_ms = telemetry_data.gyroscope.timestamp_ms[t] / 1000.0;
    if (t_ms < tend && t_ms > t0) {
      gyro_spline =
          split_traj_spline
              ->Evaluate(
                  t_ms,
                  kontiki::trajectories::EvaluationFlags::EvalAngularVelocity)
              ->angular_velocity;

      accl_spline =
          split_traj_spline
              ->Evaluate(
                  t_ms,
                  kontiki::trajectories::EvaluationFlags::EvalAcceleration)
              ->acceleration;
    }
    const std::string t_ms_s = std::to_string(t_ms);
    json_calib_results_out[t_ms_s]["gyro_imu"]["x"] = gyro_data[t][0];
    json_calib_results_out[t_ms_s]["gyro_imu"]["y"] = gyro_data[t][1];
    json_calib_results_out[t_ms_s]["gyro_imu"]["z"] = gyro_data[t][2];
    // write out spline estimates
    json_calib_results_out[t_ms_s]["gyro_spline"]["x"] = gyro_spline[0];
    json_calib_results_out[t_ms_s]["gyro_spline"]["y"] = gyro_spline[1];
    json_calib_results_out[t_ms_s]["gyro_spline"]["z"] = gyro_spline[2];
    // accelerometer
    json_calib_results_out[t_ms_s]["accl_imu"]["x"] = accl_data[t][0];
    json_calib_results_out[t_ms_s]["accl_imu"]["y"] = accl_data[t][1];
    json_calib_results_out[t_ms_s]["accl_imu"]["z"] = accl_data[t][2];
    // write out spline estimates
    json_calib_results_out[t_ms_s]["accl_spline"]["x"] = accl_spline[0];
    json_calib_results_out[t_ms_s]["accl_spline"]["y"] = accl_spline[1];
    json_calib_results_out[t_ms_s]["accl_spline"]["z"] = accl_spline[2];
  }

  std::ofstream calib_output_json(FLAGS_result_output_json);
  calib_output_json << std::setw(4) << json_calib_results_out << std::endl;
  calib_output_json.close();

  // save spline recon
  CHECK(theia::WriteReconstruction(
      reconstruction_out, FLAGS_output_path + "/" + "sparse_spline_recon"))
      << "Could not write reconstruction to file.";
  // save spline recon as nvm to perform dense recon
  const Eigen::Vector3i cam_spline_color(0, 255, 0);
  const Eigen::Vector3i cam_recon_calib_color(255, 0, 0);
  CHECK(theia::WritePlyFile(FLAGS_output_path + "/" +
                                "sparse_recon_spline_kont.ply",
                            reconstruction_out, cam_spline_color, 2));
  CHECK(theia::WritePlyFile(FLAGS_output_path + "/" +
                                "sparse_recon_calib_dataset_kont.ply",
                            calib_dataset, cam_recon_calib_color, 2));
  // CHECK(theia::WriteNVMFile(FLAGS_recon_path+"/"+"sparse_recon_spline.nvm",
  // reconstruction_out));

  return 0;
}
