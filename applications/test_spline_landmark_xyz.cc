#include <algorithm>
#include <chrono>  // NOLINT
#include <dirent.h>
#include <gflags/gflags.h>
#include <iostream>
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

#include "theia/sfm/reconstruction.h"

// Input/output files.
DEFINE_string(
    gopro_telemetry_json,
    "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(input_video, "", "Path to corresponding video file.");
DEFINE_string(detector_params, "", "Path detector yaml.");
DEFINE_string(input_calibration_dataset,
              "",
              "Path to input calibration dataset.");
DEFINE_double(downsample_factor, 2.5, "Downsample factor for images.");
DEFINE_string(save_path_spline_calib_dataset,
              "",
              "Where to save the recon dataset to.");

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

#include <theia/io/reconstruction_reader.h>

using namespace cv;
using namespace theia;

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load the reconstuction.
  theia::Reconstruction reconstruction;
  CHECK(theia::ReadReconstruction(FLAGS_input_calibration_dataset,
                                  &reconstruction))
      << "Could not read Reconstruction file.";

  // Number of cameras.
  const auto& view_ids = reconstruction.ViewIds();
  std::unordered_map<ViewId, int> view_id_to_index;
  std::unordered_map<ViewId, std::shared_ptr<ViewKontiki>> kontiki_views;
  std::vector<double> timestamps;
  // get all timestamps and find smallest one
  // Output each camera.
  for (const ViewId view_id : view_ids) {
    const View& view = *reconstruction.View(view_id);
    double timestamp = std::stod(view.Name());
    timestamps.push_back(timestamp);
  }

  const double dt_r3 = 1.0 / 30.0;
  const double dt_so3 = 1.0 / 30.0;

  // find smallest timestamp
  auto result = std::minmax_element(timestamps.begin(), timestamps.end());
  double t0 = timestamps[result.first - timestamps.begin()];
  double tend = timestamps[result.second - timestamps.begin()];

  for (const ViewId view_id : view_ids) {
    const int current_index = view_id_to_index.size();
    view_id_to_index[view_id] = current_index;

    const View& view = *reconstruction.View(view_id);
    double timestamp = std::stod(view.Name());
    if (timestamp >= tend - std::max(dt_so3, dt_r3) ||
        timestamp < t0 + std::max(dt_so3, dt_r3))
      continue;
    kontiki_views[view_id] = std::make_shared<ViewKontiki>(view_id, timestamp);
  }

  // Number of points.
  const auto& track_ids = reconstruction.TrackIds();
  // Output each point.
  std::vector<std::shared_ptr<Landmark>> kontiki_landmarks;
  Eigen::Matrix3d K;
  int img_width, img_height;
  int lauf = 0;
  for (const TrackId track_id : track_ids) {
    const Track* track = reconstruction.Track(track_id);

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
        const View* view = reconstruction.View(view_id);

        // Get the feature location normalized by the principal point.
        const Camera& camera = view->Camera();

        const Feature feature = (*view->GetFeature(track_id));

        auto cur_view_it = kontiki_views.find(view_id);
        if (cur_view_it == kontiki_views.end()) {
          continue;
        }
        cur_view_it->second->CreateObservation(
            kontiki_landmarks[kontiki_landmarks.size() - 1], feature);

        img_width = camera.ImageWidth();
        img_height = camera.ImageHeight();

        ++nr_views;
      }
    }
    ++lauf;
  }

  // set landmark references to first observation
  for (auto l : kontiki_landmarks) l->set_reference(l->observations()[0]);

  const theia::View* view_1 = reconstruction.View(0);
  view_1->Camera().GetCalibrationMatrix(&K);
  std::shared_ptr<PinholeCameraClass> cam_kontiki =
      std::make_shared<PinholeCameraClass>(img_width, img_height, 0.00, K);

  Eigen::Quaterniond imu2cam;
  std::cout << imu2cam.w() << " " << imu2cam.x() << " " << imu2cam.y() << " "
            << imu2cam.z() << std::endl;

  // Eigen::Quaterniond imu2cam(-0.0602071, -0.715386, 0.695802, 0.0214116);
  // cam_kontiki->set_relative_orientation(imu2cam);
  cam_kontiki->LockRelativeOrientation(true);
  cam_kontiki->LockRelativePosition(true);

  std::cout << "Trajectory start time: " << t0 << " tend: " << tend
            << std::endl;
  std::shared_ptr<TrajClass> traj_spline =
      std::make_shared<TrajClass>(dt_r3, 0.0);
  // split trajectory
  std::shared_ptr<SO3TrajClass> so3_traj_spline =
      std::make_shared<SO3TrajClass>(dt_so3, 0.0);
  std::shared_ptr<R3TrajClass> r3_traj_spline =
      std::make_shared<R3TrajClass>(dt_r3, 0.0);

  r3_traj_spline->ExtendTo(tend, Eigen::Vector3d(0.0, 0.0, 0.0));
  so3_traj_spline->ExtendTo(tend, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  std::shared_ptr<SplitTrajClass> split_traj_spline =
      std::make_shared<SplitTrajClass>(r3_traj_spline, so3_traj_spline);

  kontiki::TrajectoryEstimator<SplitTrajClass> traj_spline_estimator(
      split_traj_spline);

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

  r3_traj_spline->ExtendTo(std::max(t0, tend), Eigen::Vector3d(0.0, 0.0, 0.0));
  so3_traj_spline->ExtendTo(std::max(t0, tend),
                            Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  Eigen::Quaterniond q_before = cam_kontiki->relative_orientation();
  std::cout << cam_kontiki->relative_position() << std::endl;
  std::cout << q_before.w() << " " << q_before.x() << " " << q_before.y() << " "
            << q_before.z() << std::endl;

  traj_spline_estimator.Solve(100);

  return 0;
}
