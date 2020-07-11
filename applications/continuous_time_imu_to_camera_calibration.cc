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

#include "OpenCameraCalibrator/spline/measurements/accelerometer_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/gyroscope_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/orientation_measurement.h"
#include "OpenCameraCalibrator/spline/measurements/position_measurement.h"
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

#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/reconstruction.h"
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include "OpenCameraCalibrator/utils/json.h"

// Input/output files.
DEFINE_string(
    gopro_telemetry_json, "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(input_video, "", "Path to corresponding video file.");
DEFINE_string(detector_params, "", "Path detector yaml.");
DEFINE_string(input_calibration_dataset, "",
              "Path to input calibration dataset.");
DEFINE_double(checker_size_m, 0.039,
              "Size of one square on the checkerbaord in [m].");
DEFINE_string(output_path_txt_files_for_testing, "",
              "Output files for scale estimator.");
DEFINE_string(gyro_to_cam_initial_calibration, "",
              "Initial gyro to camera calibration json.");
DEFINE_string(camera_model_to_calibrate, "DIVISION_UNDISTORTION",
              "What camera model do you want to calibrate. Options:"
              "LINEAR_PINHOLE,DIVISION_UNDISTORTION");
DEFINE_double(cam_fps, 60.0,
              "Cameras fps");
DEFINE_double(max_t, 1000.,
              "Maximum nr of seconds to take");
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
using CamMeasurementAtan = M::StaticRsCameraMeasurementXYZ<AtanCameraClass>;
using CamMeasurementDivUndist =
    M::StaticRsCameraMeasurementXYZ<DivisionUndistortionCameraClass>;
using GyroMeasurement = M::GyroscopeMeasurement<IMUClass>;
using AccMeasurement = M::AccelerometerMeasurement<IMUClass>;
using PositionMeasurement = M::PositionMeasurement;

using json = nlohmann::json;

using namespace cv;
using namespace theia;

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load camera calibration reconstuction.
  theia::Reconstruction cam_calib_recon;
  CHECK(theia::ReadReconstruction(FLAGS_input_calibration_dataset,
                                  &cam_calib_recon))
      << "Could not read Reconstruction file.";
  // get one view for calibration info
  const theia::Camera camera = cam_calib_recon.View(0)->Camera();

  cv::Mat K_cv = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
  cv::Mat dist_coeffs = cv::Mat::zeros(cv::Size(4,1), CV_32FC1);
  K_cv.at<float>(0, 0) = camera.FocalLength();
  K_cv.at<float>(1, 1) = camera.FocalLength();
  K_cv.at<float>(0, 2) = camera.PrincipalPointX();
  K_cv.at<float>(1, 2) = camera.PrincipalPointY();

  int squaresX = 10;
  int squaresY = 8;
  float squareLength = FLAGS_checker_size_m;
  float markerLength = FLAGS_checker_size_m / 2.0;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
  const int min_number_detected_corners = 2;

  // read gopro telemetry
  OpenCamCalib::CameraTelemetryData telemetry_data;
  if (!OpenCamCalib::ReadGoProTelemetry(FLAGS_gopro_telemetry_json,
                                        telemetry_data)) {
    std::cout << "Could not read: " << FLAGS_gopro_telemetry_json << std::endl;
  }

  // set charuco detector parameters
  Ptr<aruco::DetectorParameters> detectorParams =
      aruco::DetectorParameters::create();

  if (!OpenCamCalib::ReadDetectorParameters(FLAGS_detector_params,
                                            detectorParams)) {
    std::cerr << "Invalid detector parameters file\n";
    return 0;
  }

  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // create charuco board object
  Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(
      squaresX, squaresY, squareLength, markerLength, dictionary);
  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

  theia::Reconstruction recon_calib_dataset;

  std::vector<cv::Point3f> chessoard3d = charucoboard->chessboardCorners;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
  for (int i = 0; i < chessoard3d.size(); ++i) {
    theia::TrackId track_id = recon_calib_dataset.AddTrack();
    theia::Track* track = recon_calib_dataset.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d* point = track->MutablePoint();
    (*point)[0] = static_cast<double>(chessoard3d[i].x);
    (*point)[1] = static_cast<double>(chessoard3d[i].y);
    (*point)[2] = static_cast<double>(chessoard3d[i].z);
    (*point)[3] = 1.0;
    charuco_id_to_theia_track_id[i] = track_id;
  }

  // run video and extract charuco board
  VideoCapture inputVideo;
  inputVideo.open(FLAGS_input_video);
  bool showRejected = false;
  int cnt_wrong = 0;
  int frame_cnt = 0;
  std::ofstream cam_pose_file, accl_file, gyro_file;
  if (FLAGS_output_path_txt_files_for_testing != "") {
    cam_pose_file.open(FLAGS_output_path_txt_files_for_testing + "/poses.txt");
    gyro_file.open(FLAGS_output_path_txt_files_for_testing +
                       "/gyroscope.txt");
    accl_file.open(FLAGS_output_path_txt_files_for_testing +
                       "/accelerometer.txt");
  }

  while (true) {
    Mat image, imageCopy;
    if (!inputVideo.read(image)) {
      cnt_wrong++;
      if (cnt_wrong > 200) break;
      continue;
    }
    const double timestamp_ = inputVideo.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
    if (timestamp_ > FLAGS_max_t)
        break;
    std::string timestamp_s = std::to_string(timestamp_);
    ++frame_cnt;

    cv::resize(image, image,
               cv::Size(camera.ImageWidth(), camera.ImageHeight()));

    std::vector<int> markerIds, charucoIds;
    std::vector<std::vector<Point2f>> markerCorners, rejectedMarkers;
    std::vector<Point2f> charucoCorners;

    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds,
                         detectorParams, rejectedMarkers);

    // refind strategy to detect more markers
    aruco::refineDetectedMarkers(image, board, markerCorners, markerIds,
                                 rejectedMarkers);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0)
      interpolatedCorners = aruco::interpolateCornersCharuco(
          markerCorners, markerIds, image, charucoboard, charucoCorners,
          charucoIds);

    if (charucoIds.size() < min_number_detected_corners) continue;
    // draw results
    image.copyTo(imageCopy);
    if (markerIds.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if (showRejected && rejectedMarkers.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(),
                                 Scalar(100, 0, 255));
    if (interpolatedCorners < 10)
        continue;

    if (interpolatedCorners > 10) {
      Scalar color;
      color = Scalar(255, 0, 0);
      aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds,
                                        color);
      if (FLAGS_output_path_txt_files_for_testing != "") {
        cv::Mat rvec, tvec;
        if(!cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                            K_cv, dist_coeffs, rvec, tvec))
            continue;
        Eigen::Vector3d axis;
        axis << rvec.at<double>(0),rvec.at<double>(1),rvec.at<double>(2);
        axis.normalize();
        Eigen::Vector3d pos;
        pos <<tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2);
        Eigen::AngleAxisd angle_axis(cv::norm(rvec), axis);
        Eigen::Quaterniond EigenQuat(angle_axis.toRotationMatrix());
        pos = -angle_axis.toRotationMatrix().transpose()*pos;
        if (cam_pose_file.is_open()) {
          cam_pose_file << std::to_string(std::stod(timestamp_s) * 1e9) << " ";
          cam_pose_file << pos(0) << " " << pos(1) << " " << pos(2);
          cam_pose_file << EigenQuat.w() << " " << EigenQuat.x() << " "
                        << EigenQuat.y() << " " << EigenQuat.z() << "\n";
        }
      }
    }


    // fill charucoCorners to theia reconstruction
    theia::ViewId view_id = recon_calib_dataset.AddView(timestamp_s, 0);
    theia::View* view = recon_calib_dataset.MutableView(view_id);
    view->SetEstimated(true);

    std::cout << "Found: " << charucoIds.size()
              << " marker. Extraced corners from: " << frame_cnt << " frames."
              << " at time: " << timestamp_s << std::endl;
    for (int i = 0; i < charucoIds.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(charucoIds[i])->second;
      theia::Feature feature;
      feature << static_cast<double>(charucoCorners[i].x),
          static_cast<double>(charucoCorners[i].y);

      Eigen::Vector3d normalized_coord = camera.PixelToNormalizedCoordinates(feature);

      recon_calib_dataset.AddObservation(view_id, track_id, normalized_coord.hnormalized());
    }
    if (frame_cnt > 20 * 60) break;
  }
  cam_pose_file.close();


  // read a gyro to cam calibration json
  std::ifstream in_file(FLAGS_gyro_to_cam_initial_calibration);
  json gyro_to_cam_calib;
  in_file >> gyro_to_cam_calib;
  Eigen::Quaterniond imu2cam(gyro_to_cam_calib["gyro_to_camera_rotation"]["w"],
          gyro_to_cam_calib["gyro_to_camera_rotation"]["x"],
          gyro_to_cam_calib["gyro_to_camera_rotation"]["y"],
          gyro_to_cam_calib["gyro_to_camera_rotation"]["z"]);
  Eigen::Vector3d gyro_bias;
  gyro_bias << gyro_to_cam_calib["gyro_bias"][0],
          gyro_to_cam_calib["gyro_bias"][1],
          gyro_to_cam_calib["gyro_bias"][2];
  in_file.close();

  const double dt_r3 = 0.1;
  const double dt_so3 = 0.1;
  const double time_offset_imu_to_cam = gyro_to_cam_calib["time_offset_gyro_to_cam"];
  const double time_offset_cam_to_imu = -time_offset_imu_to_cam;

  // Number of cameras.
  const auto& view_ids = recon_calib_dataset.ViewIds();
  std::unordered_map<ViewId, int> view_id_to_index;
  std::unordered_map<ViewId, std::shared_ptr<ViewKontiki>> kontiki_views;
  std::vector<double> timestamps;
  // get all timestamps and find smallest one
  // Output each camera.
  for (const ViewId view_id : view_ids) {
    const View& view = *recon_calib_dataset.View(view_id);
    double timestamp = std::stod(view.Name());
    timestamps.push_back(timestamp);
  }

  // find smallest timestamp
  auto result = std::minmax_element(timestamps.begin(), timestamps.end());
  double t0 = timestamps[result.first - timestamps.begin()]-time_offset_cam_to_imu - 10*dt_so3;
  double tend = timestamps[result.second - timestamps.begin()]+time_offset_cam_to_imu + 10*dt_r3;

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

    const View& view = *recon_calib_dataset.View(view_id);
    double timestamp = std::stod(view.Name());
    if (timestamp >= tend - std::max(dt_so3, dt_r3) ||
        timestamp < t0 + std::max(dt_so3, dt_r3))
      continue;
    kontiki_views[view_id] = std::make_shared<ViewKontiki>(view_id, timestamp);
  }

  // Number of points.
  const auto& track_ids = recon_calib_dataset.TrackIds();
  // Output each point.
  std::vector<std::shared_ptr<Landmark>> kontiki_landmarks;
  Eigen::Matrix3d K;
  int lauf = 0;
  for (const TrackId track_id : track_ids) {
    const Track* track = recon_calib_dataset.Track(track_id);

    // Output the observations of this 3D point.
    const auto& views_observing_track = track->ViewIds();
    int nr_views = 0;
    for (const ViewId& view_id : views_observing_track) {
      if (kontiki_views.find(view_id) == kontiki_views.end()) continue;
      nr_views++;
    }
    if (nr_views >= 2) {
      // we know our landmarks for calibration and lock them!
      kontiki_landmarks.push_back(std::make_shared<Landmark>());
      kontiki_landmarks[kontiki_landmarks.size() - 1]->set_point(
          track->Point());
      kontiki_landmarks[kontiki_landmarks.size() - 1]->Lock(true);

      nr_views = 0;
      for (const ViewId& view_id : views_observing_track) {
        const View* view = recon_calib_dataset.View(view_id);

        const Feature feature = (*view->GetFeature(track_id));
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
  for (auto l : kontiki_landmarks) {
      l->set_reference(l->observations()[0]);
  }
  //camera.GetCalibrationMatrix(&K);
  K.setIdentity();
  std::shared_ptr<PinholeCameraClass> cam_kontiki =
      std::make_shared<PinholeCameraClass>(camera.ImageWidth(),
                                           camera.ImageHeight(), 1. / FLAGS_cam_fps, K);
  //cam_kontiki->set_distortion(camera.intrinsics()[theia::DivisionUndistortionCameraModel::
  //        InternalParametersIndex::RADIAL_DISTORTION_1]);
  imu2cam.normalize();
  std::cout << imu2cam.toRotationMatrix() << std::endl;
  cam_kontiki->set_relative_orientation(imu2cam);
  cam_kontiki->LockRelativeOrientation(true);
  cam_kontiki->LockRelativePosition(true);
  cam_kontiki->LockTimeOffset(true);

  Eigen::Vector3d acc_bias(0.0,0.0,0.0);
  std::shared_ptr<IMUClass> imu_kontiki =
      std::make_shared<IMUClass>(acc_bias, gyro_bias);
  imu_kontiki->LockAccelerometerBias(true);
  imu_kontiki->LockTimeOffset(true);
  imu_kontiki->LockGyroscopeBias(true);

  int sub_sample_imu = 1;
  // add imu
  std::vector<std::shared_ptr<AccMeasurement>> acc_measurements;
  std::vector<std::shared_ptr<GyroMeasurement>> gyr_measurements;

  double weight_r3 = 1.0;
  double weight_so3 = 1.0;

  std::cout << "Trajectory start time: " << t0 << " tend: " << tend
            << std::endl;

  r3_traj_spline->ExtendTo(tend, Eigen::Vector3d(0.0, 0.0, 0.0));
  so3_traj_spline->ExtendTo(tend, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  for (int i = 0; i < telemetry_data.accelerometer.acc_masurement.size(); ++i) {
    if (i % sub_sample_imu == 0) {
      double t = telemetry_data.accelerometer.timestamp_ms[i] / 1000.0 + time_offset_imu_to_cam;
      accl_file << t * 1e9 << " "
                << telemetry_data.accelerometer.acc_masurement[i](0) << " "
                << telemetry_data.accelerometer.acc_masurement[i](1) << " "
                << telemetry_data.accelerometer.acc_masurement[i](2) << "\n";
      if (t >= tend - std::max(dt_so3, dt_r3)) break;
      if (t < t0 - std::max(dt_so3, dt_r3)) continue;
      std::shared_ptr<AccMeasurement> acc_measurement =
          std::make_shared<AccMeasurement>(
              imu_kontiki, t, telemetry_data.accelerometer.acc_masurement[i],
              weight_r3);

      acc_measurements.push_back(acc_measurement);
      traj_spline_estimator.AddMeasurement(
          acc_measurements[acc_measurements.size() - 1]);
    }
  }
  accl_file.close();
  for (int i = 0; i < telemetry_data.gyroscope.gyro_measurement.size(); ++i) {
    if (i % sub_sample_imu == 0) {
      double t = telemetry_data.gyroscope.timestamp_ms[i] / 1000.0 + time_offset_imu_to_cam;
      gyro_file << t * 1e9 << " "
                << telemetry_data.gyroscope.gyro_measurement[i](0) << " "
                << telemetry_data.gyroscope.gyro_measurement[i](1) << " "
                << telemetry_data.gyroscope.gyro_measurement[i](2) << "\n";
      if (t >= tend - std::max(dt_so3, dt_r3)) break;
      if (t < t0 - std::max(dt_so3, dt_r3)) continue;
      std::shared_ptr<GyroMeasurement> gyr_measurement =
          std::make_shared<GyroMeasurement>(
              imu_kontiki, t, telemetry_data.gyroscope.gyro_measurement[i],
              weight_so3);
      gyr_measurements.push_back(gyr_measurement);
      traj_spline_estimator.AddMeasurement(
          gyr_measurements[gyr_measurements.size() - 1]);
    }
  }
  gyro_file.close();
  // iterate all observations and create measurements
  std::vector<std::shared_ptr<CamMeasurementPinhole>> measurements;
  for (auto it : kontiki_views) {
    auto kon_view = it.second;
    std::vector<std::shared_ptr<Observation>> observations =
        kon_view->observations();
    for (int i = 0; i < observations.size(); ++i) {
      measurements.push_back(std::make_shared<CamMeasurementPinhole>(
          cam_kontiki, observations[i]));
      traj_spline_estimator.AddMeasurement(
          measurements[measurements.size() - 1]);
    }
  }

  traj_spline_estimator.Solve(100, true, 1);

  std::cout<<"relative_orientation: "<<cam_kontiki->relative_orientation().toRotationMatrix()<<"\n";

  return 0;
}
