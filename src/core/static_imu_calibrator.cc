#include "OpenCameraCalibrator/core/static_imu_calibrator.h"
#include "OpenCameraCalibrator/utils/gyro_integration.h"
#include "OpenCameraCalibrator/utils/imu_data_interval.h"

#include "ceres/ceres.h"
#include <glog/logging.h>
#include <iostream>
#include <limits>

using namespace Eigen;
using namespace OpenICC::utils;

namespace OpenICC {
namespace core {

/*
 * imu_tk - Inertial Measurement Unit Toolkit
 *
 *  Copyright (c) 2014, Alberto Pretto <pretto@diag.uniroma1.it>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

StaticImuCalibrator::StaticImuCalibrator()
    : g_mag_(9.81),
      min_num_intervals_(12),
      init_interval_duration_(30.0),
      interval_n_samples_(100),
      acc_use_means_(false),
      gyro_dt_(-1.0),
      optimize_gyro_bias_(false),
      verbose_output_(true) {}

bool StaticImuCalibrator::CalibrateAcc(const ImuReadings& acc_samples) {
  std::cout << "Accelerometers calibration: calibrating...";

  min_cost_static_intervals_.clear();
  calib_acc_samples_.clear();
  calib_gyro_samples_.clear();

  int n_samps = acc_samples.size();

  utils::DataInterval init_static_interval =
      DataInterval::InitialInterval(acc_samples, init_interval_duration_);
  Vector3d acc_mean = DataMean(acc_samples, init_static_interval);
  Eigen::Vector3d::Index max_index;
  acc_mean.maxCoeff(&max_index);
  acc_mean[max_index] -= g_mag_;
  init_acc_calib_.SetBias(acc_mean);
  std::cout << "Setting initial accelerometer bias: "
            << init_acc_calib_.GetBiasVector().transpose() << "\n";
  Vector3d acc_variance = DataVariance(acc_samples, init_static_interval);
  double norm_th = acc_variance.norm();

  double min_cost = std::numeric_limits<double>::max();
  int min_cost_th = -1;
  std::vector<double> min_cost_calib_params;

  for (int th_mult = 1; th_mult <= 10; th_mult++) {
    std::vector<DataInterval> static_intervals;
    ImuReadings static_samples;
    std::vector<double> acc_calib_params(9);

    acc_calib_params[0] = init_acc_calib_.misYZ();
    acc_calib_params[1] = init_acc_calib_.misZY();
    acc_calib_params[2] = init_acc_calib_.misZX();

    acc_calib_params[3] = init_acc_calib_.scaleX();
    acc_calib_params[4] = init_acc_calib_.scaleY();
    acc_calib_params[5] = init_acc_calib_.scaleZ();

    acc_calib_params[6] = init_acc_calib_.biasX();
    acc_calib_params[7] = init_acc_calib_.biasY();
    acc_calib_params[8] = init_acc_calib_.biasZ();

    std::vector<DataInterval> extracted_intervals;
    StaticIntervalsDetector(acc_samples, th_mult * norm_th, static_intervals);
    ExtractIntervalsSamples(acc_samples,
                            static_intervals,
                            static_samples,
                            extracted_intervals,
                            interval_n_samples_,
                            acc_use_means_);

    if (verbose_output_) {
      std::cout << "Accelerometers calibration: extracted "
                << extracted_intervals.size()
                << " intervals using threshold multiplier " << th_mult
                << " -> ";
    }
    // TODO Perform here a quality test
    if (extracted_intervals.size() < min_num_intervals_) {
      if (verbose_output_)
        std::cout << "Not enough intervals, calibration is not possible";
      continue;
    }

    ceres::Problem problem;
    for (int i = 0; i < static_samples.size(); i++) {
      ceres::CostFunction* cost_function =
          MultiPosAccResidual::Create(g_mag_, static_samples[i].data());

      problem.AddResidualBlock(
          cost_function, NULL /* squared loss */, acc_calib_params.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose_output_;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (summary.final_cost < min_cost) {
      min_cost = summary.final_cost;
      min_cost_th = th_mult;
      min_cost_static_intervals_ = static_intervals;
      min_cost_calib_params = acc_calib_params;
    }
    std::cout << "Accelerometer residual " << summary.final_cost << "\n";
  }

  if (min_cost_th < 0) {
    if (verbose_output_)
      std::cout
          << "Accelerometers calibration: Can't obtain any calibratin with "
             "the current dataset";
    return false;
  }

  acc_calib_ = ThreeAxisSensorCalibParams<double>(min_cost_calib_params[0],
                                                  min_cost_calib_params[1],
                                                  min_cost_calib_params[2],
                                                  0,
                                                  0,
                                                  0,
                                                  min_cost_calib_params[3],
                                                  min_cost_calib_params[4],
                                                  min_cost_calib_params[5],
                                                  min_cost_calib_params[6],
                                                  min_cost_calib_params[7],
                                                  min_cost_calib_params[8]);

  calib_acc_samples_.reserve(n_samps);

  // Calibrate the input accelerometer data with the obtained calibration
  for (int i = 0; i < n_samps; i++) {
    calib_acc_samples_.push_back(
        ImuReading(acc_samples[i].timestamp_s(),
                   acc_calib_.UnbiasNormalize(acc_samples[i].data())));
  }

  // final accelerometer calibration

  std::cout << "Accelerometer misalignment matrix: \n"
            << acc_calib_.GetMisalignmentMatrix() << std::endl
            << "Accelerometer scale matrix: \n"
            << acc_calib_.GetScaleMatrix() << std::endl
            << "Accelerometer bias: \n"
            << acc_calib_.GetBiasVector().transpose() << std::endl
            << "Accelerometer inverse scale factors: "
            << 1.0 / acc_calib_.scaleX() << " " << 1.0 / acc_calib_.scaleY()
            << " " << 1.0 / acc_calib_.scaleZ() << std::endl
            << std::endl;

  return true;
}

bool StaticImuCalibrator::CalibrateAccGyro(const ImuReadings& acc_samples,
                                           const ImuReadings& gyro_samples) {
  if (!CalibrateAcc(acc_samples)) {
    std::cerr << "Failed to calibra accelerometer\n";
    return false;
  }

  std::cout << "Gyroscopes calibration: calibrating...";

  ImuReadings static_acc_means;
  std::vector<DataInterval> extracted_intervals;
  ExtractIntervalsSamples(calib_acc_samples_,
                          min_cost_static_intervals_,
                          static_acc_means,
                          extracted_intervals,
                          interval_n_samples_,
                          true);

  int n_static_pos = static_acc_means.size(), n_samps = gyro_samples.size();

  // Compute the gyroscopes biases in the (static) initialization interval
  DataInterval init_static_interval =
      DataInterval::InitialInterval(gyro_samples, init_interval_duration_);
  Vector3d gyro_bias = DataMean(gyro_samples, init_static_interval);

  gyro_calib_ = ThreeAxisSensorCalibParams<double>(0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   1.0,
                                                   1.0,
                                                   1.0,
                                                   gyro_bias(0),
                                                   gyro_bias(1),
                                                   gyro_bias(2));

  // calib_gyro_samples_ already cleared in calibrateAcc()
  calib_gyro_samples_.reserve(n_samps);
  // Remove the bias
  for (int i = 0; i < n_samps; i++)
    calib_gyro_samples_.push_back(
        ImuReading(gyro_samples[i].timestamp_s(),
                   gyro_calib_.Unbias(gyro_samples[i].data())));

  std::vector<double> gyro_calib_params(12);

  gyro_calib_params[0] = init_gyro_calib_.misYZ();
  gyro_calib_params[1] = init_gyro_calib_.misZY();
  gyro_calib_params[2] = init_gyro_calib_.misZX();
  gyro_calib_params[3] = init_gyro_calib_.misXZ();
  gyro_calib_params[4] = init_gyro_calib_.misXY();
  gyro_calib_params[5] = init_gyro_calib_.misYX();

  gyro_calib_params[6] = init_gyro_calib_.scaleX();
  gyro_calib_params[7] = init_gyro_calib_.scaleY();
  gyro_calib_params[8] = init_gyro_calib_.scaleZ();

  // Bias has been estimated and removed in the initialization period
  gyro_calib_params[9] = 0.0;
  gyro_calib_params[10] = 0.0;
  gyro_calib_params[11] = 0.0;

  ceres::Problem problem;

  for (int i = 0, t_idx = 0; i < n_static_pos - 1; i++) {
    Vector3d g_versor_pos0 = static_acc_means[i].data(),
             g_versor_pos1 = static_acc_means[i + 1].data();

    g_versor_pos0 /= g_versor_pos0.norm();
    g_versor_pos1 /= g_versor_pos1.norm();

    int gyro_idx0 = -1, gyro_idx1 = -1;
    double ts0 =
               calib_acc_samples_[extracted_intervals[i].end_idx].timestamp_s(),
           ts1 = calib_acc_samples_[extracted_intervals[i + 1].start_idx]
                     .timestamp_s();

    // Assume monotone signal time
    for (; t_idx < n_samps; t_idx++) {
      if (gyro_idx0 < 0) {
        if (calib_gyro_samples_[t_idx].timestamp_s() >= ts0) gyro_idx0 = t_idx;
      } else {
        if (calib_gyro_samples_[t_idx].timestamp_s() >= ts1) {
          gyro_idx1 = t_idx - 1;
          break;
        }
      }
    }

    DataInterval gyro_interval(gyro_idx0, gyro_idx1);

    ceres::CostFunction* cost_function =
        MultiPosGyroResidual::Create(g_versor_pos0,
                                     g_versor_pos1,
                                     calib_gyro_samples_,
                                     gyro_interval,
                                     gyro_dt_,
                                     optimize_gyro_bias_);

    problem.AddResidualBlock(
        cost_function, NULL /* squared loss */, gyro_calib_params.data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = verbose_output_;

  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);
  gyro_calib_ =
      ThreeAxisSensorCalibParams<double>(gyro_calib_params[0],
                                         gyro_calib_params[1],
                                         gyro_calib_params[2],
                                         gyro_calib_params[3],
                                         gyro_calib_params[4],
                                         gyro_calib_params[5],
                                         gyro_calib_params[6],
                                         gyro_calib_params[7],
                                         gyro_calib_params[8],
                                         gyro_bias(0) + gyro_calib_params[9],
                                         gyro_bias(1) + gyro_calib_params[10],
                                         gyro_bias(2) + gyro_calib_params[11]);

  // Calibrate the input gyroscopes data with the obtained calibration
  for (int i = 0; i < n_samps; i++) {
    calib_gyro_samples_.push_back(
        ImuReading(gyro_samples[i].timestamp_s(),
                   gyro_calib_.UnbiasNormalize(gyro_samples[i].data())));
  }

  if (verbose_output_) {
    std::cout << summary.FullReport();
  }
  std::cout << "Gyroscopes calibration: residual " << summary.final_cost
            << std::endl
            << "Gyroscope misalignment matrix: \n"
            << gyro_calib_.GetMisalignmentMatrix() << std::endl
            << "Gyroscope scale matrix: \n"
            << gyro_calib_.GetScaleMatrix() << std::endl
            << "Gyroscope bias: \n"
            << gyro_calib_.GetBiasVector().transpose() << std::endl
            << "Gyroscope inverse scale factors: " << 1.0 / gyro_calib_.scaleX()
            << " " << 1.0 / gyro_calib_.scaleY() << " "
            << 1.0 / gyro_calib_.scaleZ() << std::endl;

  return true;
}

}  // namespace core
}  // namespace OpenICC
