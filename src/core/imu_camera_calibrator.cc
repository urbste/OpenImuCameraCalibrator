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

using namespace theia;

namespace OpenICC {
namespace core {

void ImuCameraCalibrator::InitSpline(
    const theia::Reconstruction &calib_dataset,
    const Sophus::SE3<double> &T_i_c_init,
    const SplineWeightingData &spline_weight_data,
    const double time_offset_imu_to_cam, const Eigen::Vector3d &gyro_bias,
    const Eigen::Vector3d &accl_bias,
    const OpenICC::CameraTelemetryData &telemetry_data,
    const double initial_line_delay) {

  image_data_ = calib_dataset;
  spline_weight_data_ = spline_weight_data;
  T_i_c_init_ = T_i_c_init;

  // set camera timestamps and sort them
  const auto &view_ids = calib_dataset.ViewIds();
  for (const ViewId view_id : view_ids) {
    cam_timestamps_.push_back(calib_dataset.View(view_id)->GetTimestamp());
  }
  std::sort(cam_timestamps_.begin(), cam_timestamps_.end());

  // initialize readout with 1/fps * 1/image_rows
  inital_cam_line_delay_s_ = initial_line_delay;
  trajectory_.SetInitialRSLineDelay(inital_cam_line_delay_s_);

  std::cout << "Initialized Line Delay to: "
            << inital_cam_line_delay_s_ * S_TO_US << "ns\n";

  // find smallest timestamp
  auto result =
      std::minmax_element(cam_timestamps_.begin(), cam_timestamps_.end());
  t0_s_ = cam_timestamps_[result.first - cam_timestamps_.begin()];
  tend_s_ = cam_timestamps_[result.second - cam_timestamps_.begin()];
  const int64_t start_t_ns = t0_s_ * S_TO_NS;
  const int64_t end_t_ns = tend_s_ * S_TO_NS;
  const int64_t dt_so3_ns = spline_weight_data_.dt_so3 * S_TO_NS;
  const int64_t dt_r3_ns = spline_weight_data_.dt_r3 * S_TO_NS;
  LOG(INFO) << "Spline initialized with. Start/End: " << t0_s_ << "/" << tend_s_
            << " knots spacing r3/so3: " << spline_weight_data_.dt_r3 << "/"
            << spline_weight_data_.dt_so3;

  nr_knots_so3_ = (end_t_ns - start_t_ns) / dt_so3_ns + SPLINE_N;
  nr_knots_r3_ = (end_t_ns - start_t_ns) / dt_r3_ns + SPLINE_N;

  std::cout << "Initializing " << nr_knots_so3_ << " SO3 knots.\n";
  std::cout << "Initializing " << nr_knots_r3_ << " R3 knots.\n";

  trajectory_.init_times(dt_so3_ns, dt_r3_ns, start_t_ns);
  trajectory_.setCalib(calib_dataset);
  trajectory_.setT_i_c(T_i_c_init);

  trajectory_.initAll(image_data_, T_i_c_init, nr_knots_so3_, nr_knots_r3_);

  // add visual measurements
  for (const auto &vid : view_ids) {
    const auto *view = image_data_.View(vid);
    const double timestamp_s = view->GetTimestamp();
    trajectory_.addRSCornersMeasurement(&image_data_, view,
                                        &calib_dataset.View(0)->Camera(),
                                        timestamp_s * S_TO_NS);
  }

  // Add Accelerometer
  for (size_t i = 0; i < telemetry_data.accelerometer.measurement.size(); ++i) {
    const double t = telemetry_data.accelerometer.timestamp_ms[i] * MS_TO_S +
                     time_offset_imu_to_cam;
    if (t < t0_s_ || t >= tend_s_)
      continue;

    const Eigen::Vector3d accl_unbiased =
        telemetry_data.accelerometer.measurement[i] + accl_bias;
    trajectory_.addAccelMeasurement(accl_unbiased, t * S_TO_NS,
                                    1. / spline_weight_data_.var_r3,
                                    reestimate_biases_);
    accl_measurements_[t] = accl_unbiased;
  }

  // Add Gyroscope
  for (size_t i = 0; i < telemetry_data.gyroscope.measurement.size(); ++i) {
    const double t = telemetry_data.gyroscope.timestamp_ms[i] * MS_TO_S +
                     time_offset_imu_to_cam;
    if (t < t0_s_ || t >= tend_s_)
      continue;

    const Eigen::Vector3d gyro_unbiased =
        telemetry_data.gyroscope.measurement[i] + gyro_bias;
    trajectory_.addGyroMeasurement(gyro_unbiased, t * S_TO_NS,
                                   1. / spline_weight_data_.var_so3,
                                   reestimate_biases_);
    gyro_measurements_[t] = gyro_unbiased;
  }
}

void ImuCameraCalibrator::InitializeGravity(
    const OpenICC::CameraTelemetryData &telemetry_data,
    const Eigen::Vector3d &accl_bias) {
  for (size_t j = 0; j < cam_timestamps_.size(); ++j) {
    const theia::View *v =
        image_data_.View(image_data_.ViewIdFromTimestamp(cam_timestamps_[j]));
    if (!v) {
      continue;
    }

    const auto q_w_c =
        Eigen::Quaterniond(v->Camera().GetOrientationAsRotationMatrix());
    const auto p_w_c = v->Camera().GetPosition();
    Sophus::SE3d T_a_i = Sophus::SE3d(q_w_c, p_w_c) * T_i_c_init_.inverse();

    if (!gravity_initialized_) {
      for (size_t i = 0; i < telemetry_data.accelerometer.measurement.size();
           i++) {
        const Eigen::Vector3d ad =
            telemetry_data.accelerometer.measurement[i] + accl_bias;
        const int64_t accl_t =
            telemetry_data.accelerometer.timestamp_ms[i] * MS_TO_S;
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
  trajectory_.setG(gravity_init_);
}

double ImuCameraCalibrator::Optimize(const int iterations,
                                     const bool fix_so3_spline,
                                     const bool fix_r3_spline,
                                     const bool fix_T_i_c,
                                     const bool fix_line_delay) {
  ceres::Solver::Summary summary = trajectory_.optimize(
      iterations, fix_so3_spline, fix_r3_spline, fix_T_i_c, fix_line_delay);
  return trajectory_.meanRSReprojection(calib_corners_);
}

void ImuCameraCalibrator::ToTheiaReconDataset(Reconstruction &output_recon) {
  // convert spline to theia output
  for (size_t i = 0; i < cam_timestamps_.size(); ++i) {
    const int64_t t_ns = cam_timestamps_[i] * S_TO_NS;
    Sophus::SE3d spline_pose = trajectory_.getPose(t_ns);
    theia::ViewId v_id_theia =
        output_recon.AddView(std::to_string(t_ns), 0, t_ns);
    theia::View *view = output_recon.MutableView(v_id_theia);
    view->SetEstimated(true);
    theia::Camera *camera = view->MutableCamera();
    camera->SetOrientationFromRotationMatrix(
        spline_pose.rotationMatrix().transpose());
    camera->SetPosition(spline_pose.translation());
  }
}

void ImuCameraCalibrator::ClearSpline() {
  cam_timestamps_.clear();
  gyro_measurements_.clear();
  accl_measurements_.clear();
  calib_corners_.clear();
  calib_init_poses_.clear();
  spline_init_poses_.clear();
  trajectory_.Clear();
}

} // namespace core
} // namespace OpenICC
