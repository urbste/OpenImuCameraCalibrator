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

#include "OpenCameraCalibrator/core/imu_camera_calibrator.h"

namespace OpenICC {
namespace core {

void ImuCameraCalibrator::BatchInitSpline(
    const theia::Reconstruction& vision_dataset,
    const Sophus::SE3<double>& T_i_c_init,
    const SplineWeightingData& spline_weight_data,
    const double time_offset_imu_to_cam,
    const OpenICC::CameraTelemetryData& telemetry_data,
    const double initial_line_delay,
    const ThreeAxisSensorCalibParams<double> accl_intrinsics,
    const ThreeAxisSensorCalibParams<double> gyro_intrinsics) {
  image_data_ = vision_dataset;
  spline_weight_data_ = spline_weight_data;
  T_i_c_init_ = T_i_c_init;

  trajectory_.SetT_i_c(T_i_c_init);
  trajectory_.SetImuToCameraTimeOffset(
      -telemetry_data.accelerometer[0].timestamp_s());
  trajectory_.SetIMUIntrinsics(accl_intrinsics, gyro_intrinsics);

  // set camera timestamps and sort them
  const auto& view_ids = vision_dataset.ViewIds();
  for (const theia::ViewId view_id : view_ids) {
    cam_timestamps_.push_back(vision_dataset.View(view_id)->GetTimestamp());
  }
  std::sort(cam_timestamps_.begin(), cam_timestamps_.end());

  // initialize readout with 1/fps * 1/image_rows
  inital_cam_line_delay_s_ = initial_line_delay;
  trajectory_.SetCameraLineDelay(inital_cam_line_delay_s_);

  std::cout << "Initialized Line Delay to: "
            << inital_cam_line_delay_s_ * S_TO_US << "ns\n";

  // find smallest vision timestamp and set spline times
  auto result =
      std::minmax_element(cam_timestamps_.begin(), cam_timestamps_.end());
  t0_s_ = cam_timestamps_[result.first - cam_timestamps_.begin()];
  tend_s_ = cam_timestamps_[result.second - cam_timestamps_.begin()];
  const int64_t start_t_ns = t0_s_ * S_TO_NS;
  const int64_t end_t_ns =
      tend_s_ * S_TO_NS + S_TO_NS*(540*inital_cam_line_delay_s_);
  const int64_t dt_so3_ns = spline_weight_data_.dt_so3 * S_TO_NS;
  const int64_t dt_r3_ns = spline_weight_data_.dt_r3 * S_TO_NS;

  trajectory_.SetTimes(dt_so3_ns, dt_r3_ns, start_t_ns, end_t_ns);

  LOG(INFO) << "Spline initialized with. Start/End: " << t0_s_ << "/" << tend_s_
            << " knots spacing r3/so3: " << spline_weight_data_.dt_r3 << "/"
            << spline_weight_data_.dt_so3;

  nr_knots_so3_ = (end_t_ns - start_t_ns) / dt_so3_ns + SPLINE_N;
  nr_knots_r3_ = (end_t_ns - start_t_ns) / dt_r3_ns + SPLINE_N;

  std::cout << "Initializing " << nr_knots_so3_ << " SO3 knots.\n";
  std::cout << "Initializing " << nr_knots_r3_ << " R3 knots.\n";

  // after initing times, let's now initialize the knots using the known
  // camera poses (T_w_c)
  trajectory_.SetImageData(image_data_);
  trajectory_.BatchInitSO3R3VisPoses();
  // TODO make the bias spline dt configurable
  trajectory_.InitBiasSplines(accl_intrinsics.GetBiasVector(),
                              gyro_intrinsics.GetBiasVector(),
                              5 * 1e9,
                              5 * 1e9,
                              1.0,
                              1e-2);

  LOG(INFO) << "Adding Vision measurements to spline";

  // rolling shutter camera
  if (inital_cam_line_delay_s_ != 0.0) {
    for (const auto& vid : vision_dataset.ViewIds()) {
      trajectory_.AddRSCameraMeasurement(vision_dataset.View(vid), 5.0);
    }
  } else {
    for (const auto& vid : vision_dataset.ViewIds()) {
      trajectory_.AddGSCameraMeasurement(vision_dataset.View(vid), 5.0);
    }
  }
  LOG(INFO) << "Added all Vision measurements to the spline estimator";

  LOG(INFO) << "Adding IMU measurements to spline";
  for (size_t i = 0; i < telemetry_data.accelerometer.size(); ++i) {
    const double t =
        telemetry_data.accelerometer[i].timestamp_s() - telemetry_data.accelerometer[0].timestamp_s();
    if (t < t0_s_ || t >= tend_s_) continue;
    gyro_measurements_[t] = telemetry_data.gyroscope[i].data();
    accl_measurements_[t] = telemetry_data.accelerometer[i].data();
    if (!trajectory_.AddAccelerometerMeasurement(
            telemetry_data.accelerometer[i].data(),
            t * S_TO_NS,
            1. / spline_weight_data.std_r3)) {
      std::cerr << "Failed to add accelerometer measurement at time: " << t
                << "\n";
    }
    if (!trajectory_.AddGyroscopeMeasurement(telemetry_data.gyroscope[i].data(),
                                             t * S_TO_NS,
                                             1. / spline_weight_data.std_so3)) {
      std::cerr << "Failed to add gyroscope measurement at time: " << t << "\n";
    }
  }
  LOG(INFO) << "Added all IMU measurements to the spline estimator";

  InitializeGravity(telemetry_data);
}

void ImuCameraCalibrator::SetKnownGravityDir(const Eigen::Vector3d& gravity) {
  trajectory_.SetGravity(gravity);
}

void ImuCameraCalibrator::InitializeGravity(
    const OpenICC::CameraTelemetryData& telemetry_data) {
  for (size_t j = 0; j < cam_timestamps_.size(); ++j) {
    const theia::View* v =
        image_data_.View(image_data_.ViewIdFromTimestamp(cam_timestamps_[j]));
    if (!v) {
      continue;
    }

    const auto q_w_c = Eigen::Quaterniond(
        v->Camera().GetOrientationAsRotationMatrix().transpose());
    const auto p_w_c = v->Camera().GetPosition();
    Sophus::SE3d T_a_i = Sophus::SE3d(q_w_c, p_w_c) * T_i_c_init_.inverse();

    if (!gravity_initialized_) {
      for (size_t i = 0; i < telemetry_data.accelerometer.size(); i++) {
        const Eigen::Vector3d ad = telemetry_data.accelerometer[i].data();
        const int64_t accl_t = telemetry_data.accelerometer[i].timestamp_s();
        if (std::abs(accl_t - cam_timestamps_[j]) < 1. / 30.) {
          gravity_init_ = T_a_i.so3() * ad;
          gravity_initialized_ = true;
          std::cout << "g_a initialized with " << gravity_init_.transpose()
                    << " at timestamp: " << accl_t << std::endl;
        }
        if (gravity_initialized_) {
          break;
        }
      }
    }
  }
  trajectory_.SetGravity(gravity_init_);
}

double ImuCameraCalibrator::Optimize(const int iterations,
                                     const int optim_flags) {
  ceres::Solver::Summary summary =
      trajectory_.Optimize(iterations, optim_flags);
  if (inital_cam_line_delay_s_ == 0.0) {
    return trajectory_.GetMeanGSReprojectionError();
  }
  return trajectory_.GetMeanRSReprojectionError();
}

void ImuCameraCalibrator::ToTheiaReconDataset(
    theia::Reconstruction& output_recon) {
  // convert spline to theia output
  for (size_t i = 0; i < cam_timestamps_.size(); ++i) {
    const int64_t t_ns = cam_timestamps_[i] * S_TO_NS;
    Sophus::SE3d spline_pose;
    trajectory_.GetPose(t_ns, spline_pose);
    theia::ViewId v_id_theia =
        output_recon.AddView(std::to_string(t_ns), 0, t_ns);
    theia::View* view = output_recon.MutableView(v_id_theia);
    view->SetEstimated(true);
    theia::Camera* camera = view->MutableCamera();
    camera->SetOrientationFromRotationMatrix(
        spline_pose.rotationMatrix().transpose());
    camera->SetPosition(spline_pose.translation());
  }
}

void ImuCameraCalibrator::ClearSpline() {
  cam_timestamps_.clear();
  gyro_measurements_.clear();
  accl_measurements_.clear();
}

void ImuCameraCalibrator::GetIMUIntrinsics(
    ThreeAxisSensorCalibParams<double>& acc_intrinsics,
    ThreeAxisSensorCalibParams<double>& gyr_intrinsics,
    const int64_t time_ns) {
  acc_intrinsics = trajectory_.GetAcclIntrinsics(time_ns);
  gyr_intrinsics = trajectory_.GetGyroIntrinsics(time_ns);
}

}  // namespace core
}  // namespace OpenICC
