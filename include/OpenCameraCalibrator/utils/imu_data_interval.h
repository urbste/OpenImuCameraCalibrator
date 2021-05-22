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
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/reconstruction.h>
#include <theia/sfm/view.h>

#include <algorithm>
#include <dirent.h>
#include <vector>

#include "OpenCameraCalibrator/utils/types.h"

// CODE TAKEN FROM
// https://github.com/Kyle-ak/imu_tk/blob/master/include/imu_tk/base.h slight
// adjustments made to data types if you use the static imu calibration method
// consider citing the original authors paper:
// @inproceedings{tpm_icra2014,
//   title={A Robust and Easy to Implement Method for IMU Calibration
//             without External Equipments},
//   author={Tedaldi, A. and Pretto, A. and Menegatti, E.},
//   booktitle={Proc. of: IEEE International Conference on Robotics and
//                    Automation (ICRA)},
//   year={2014},
//   pages={3042--3049}
// }

namespace OpenICC {
namespace utils {

struct DataInterval {
  /** @brief Default constructor (without parameters, it constructs an invalid
   * interval, i.e. both -1 indices ) */
  DataInterval(int start_idx = -1, int end_idx = -1)
      : start_idx(start_idx), end_idx(end_idx) {}

  /** @brief Provides a DataInterval object from a time interval. The indices
   * are extracted from the data samples vector using a nearest neighbor
   * approach.
   *
   * @param samples Input signal (data samples vector)
   * @param start_ts Initial timestamp
   * @param end_ts Final timestamp
   */
  static DataInterval FromTimestamps(const ImuReadings &samples,
                                     double start_ts, double end_ts) {
    if (start_ts < 0 || end_ts <= start_ts)
      throw std::invalid_argument("Invalid timestamps");
    if (samples.size() < 3)
      throw std::invalid_argument("Invalid data samples vector");

    int start_idx, end_idx;
    if (start_ts <= samples[0].timestamp_s())
      start_idx = 0;
    else
      start_idx = TimeToIndex(samples, start_ts);

    if (end_ts >= samples[samples.size() - 1].timestamp_s())
      end_idx = samples.size() - 1;
    else
      end_idx = TimeToIndex(samples, end_ts);

    return DataInterval(start_idx, end_idx);
  }

  /** @brief Extracts from the data samples vector a DataInterval object that
   * represents the initial interval with a given duration.
   *
   * @param samples Input signal (data samples vector)
   * @param duration Interval duration
   */
  static DataInterval InitialInterval(const ImuReadings &samples,
                                      double duration_s) {
    if (duration_s <= 0)
      throw std::invalid_argument("Invalid interval duration");
    if (samples.size() < 3)
      throw std::invalid_argument("Invalid data samples vector");

    double end_ts = samples[0].timestamp_s() + duration_s;
    int end_idx;
    if (end_ts >= samples[samples.size() - 1].timestamp_s())
      end_idx = samples.size() - 1;
    else
      end_idx = TimeToIndex(samples, end_ts);

    return DataInterval(0, end_idx);
  }

  /** @brief Extracts from the data samples vector a DataInterval object that
   * represents the final interval with a given duration.
   *
   * @param samples Input signal (data samples vector)
   * @param duration Interval duration
   */
  static DataInterval FinalInterval(const ImuReadings &samples,
                                    double duration) {
    if (duration <= 0)
      throw std::invalid_argument("Invalid interval duration");
    if (samples.size() < 3)
      throw std::invalid_argument("Invalid data samples vector");

    double start_ts = samples[samples.size() - 1].timestamp_s() - duration;
    int start_idx;
    if (start_ts <= 0)
      start_idx = 0;
    else
      start_idx = TimeToIndex(samples, start_ts);

    return DataInterval(start_idx, samples.size() - 1);
  }

  int start_idx, end_idx;

private:
  static int TimeToIndex(const ImuReadings &samples, double ts) {
    int idx0 = 0, idx1 = samples.size() - 1, idxm;
    while (idx1 - idx0 > 1) {
      idxm = (idx1 + idx0) / 2;
      if (ts > samples[idxm].timestamp_s())
        idx0 = idxm;
      else
        idx1 = idxm;
    }

    if (ts - samples[idx0].timestamp_s() < samples[idx1].timestamp_s() - ts)
      return idx0;
    else
      return idx1;
  };
};

/** @brief Perform a simple consistency check on a target input interval,
 *         given an input data sample vector, and return the "corrected"
 *         interval
 *
 * @param samples Input signal (data samples vector)
 * @param interval Interval to be checked
 *
 * @returns The "corrected" interval
 */
template <typename T>
DataInterval CheckInterval(const std::vector<ImuReading<T>> &samples,
                           const DataInterval &interval) {
  int start_idx = interval.start_idx, end_idx = interval.end_idx;
  if (start_idx < 0)
    start_idx = 0;
  if (end_idx < start_idx || end_idx > samples.size() - 1)
    end_idx = samples.size() - 1;

  return DataInterval(start_idx, end_idx);
}

/** @brief Compute the arithmetic mean of a sequence of TriadData_ objects. If a
 * valid data interval is provided, the mean is computed only inside this
 * interval
 *
 * @param samples Input signal (data samples vector)
 * @param interval Data interval where to compute the mean. If this interval is
 * not valid, i.e., one of the two indices is -1, the mean is computed for the
 * whole data sequence.
 *
 * @returns A three dimensional vector representing the mean.
 */
Eigen::Vector3d DataMean(const ImuReadings &samples,
                         const DataInterval &interval = DataInterval());

/** @brief Compute the variance of a sequence of TriadData_ objects. If a valid
 * data interval is provided, the variance is computed only inside this interval
 *
 * @param samples Input signal (data samples vector)
 * @param interval Data interval where to compute the variance. If this interval
 * is not valid, i.e., one of the two indices is -1, the variance is computed
 * for the whole data sequence.
 *
 * @returns A three dimensional vector representing the variance.
 */
Eigen::Vector3d DataVariance(const ImuReadings &samples,
                             const DataInterval &interval = DataInterval());

/** @brief If the flag only_means is set to false, for each interval
 *        (input vector intervals) extract from the input signal
 *        (samples) the first interval_n_samps samples, and store them
 *        in the output vector extracted_samples. If the flag only_means
 *        is set to true, extract for each interval only the local mean,
 * computed in interval with size at least interval_n_samps samples. Only
 * intervals with at least interval_n_samps samples are considered.
 *
 * @param samples Input signal (data samples vector)
 * @param intervals Input intervals vector
 * @param[out] extracted_samples Output signal that contains the extracted
 * samples
 * @param[out] extracted_intervals Output intervals vector with all the used
 * intervals, i.e. intervals with size at least interval_n_samps samples
 * @param interval_n_samps Number of samples to be extracted from each interval
 * (or interval size to be used to compute the local mean if only_means is set
 * to true)
 * @param only_means If true, extract for each interval only the local mean,
 * computed in intervals with size at least interval_n_samps samples. The
 * timestamp is the one of the center of the interval.
 *
 */

void ExtractIntervalsSamples(const ImuReadings &samples,
                             const std::vector<DataInterval> &intervals,
                             ImuReadings &extracted_samples,
                             std::vector<DataInterval> &extracted_intervals,
                             int interval_n_samps = 100,
                             bool only_means = false);

/** @brief Decompose a rotation matrix into the roll, pitch, and yaw angular
 * components
 *
 *  @param rot_mat Input rotation matrix
 *  @param[out] rpy_rot_vec Output roll, pitch, and yaw angular components
 */
template <typename _T>
void DecomposeRotation(const Eigen::Matrix<_T, 3, 3> &rot_mat,
                       Eigen::Matrix<_T, 3, 1> &rpy_rot_vec) {
  rpy_rot_vec(0) = atan2(rot_mat(2, 1), rot_mat(2, 2));
  rpy_rot_vec(1) = atan2(-rot_mat(2, 0), sqrt(rot_mat(2, 1) * rot_mat(2, 1) +
                                              rot_mat(2, 2) * rot_mat(2, 2)));
  rpy_rot_vec(2) = atan2(rot_mat(1, 0), rot_mat(0, 0));
}

/**
 * @brief Classify between static and motion intervals checking if for each
 * sample of the input signal \f$s\f$ (samples) the local variance magnitude is
 * lower or greater then a threshold.
 *
 * @param samples Input 3D signal (e.g, the acceleremeter readings)
 * @param threshold Threshold used in the classification
 * @param[out] intervals  Ouput detected static intervals
 * @param win_size Size of the sliding window (i.e., number of samples)
 *                 used to compute the local variance magnitude. It should be
 * equal or greater than 11
 *
 *
 * The variance magnitude is a scalar computed in a temporal sliding window of
 * size \f$w_s\f$ (i.e., win_size) as: \f[ \varsigma(t) =
 *     \sqrt{[var_{w_s}(s^t_x)]^2 + [var_{w_s}(s^t_y)]^2 + [var_{w_s}(s^t_z)]^2}
 * \f]
 *
 * Where \f$var_{w_s}(s^t)\f$ is an operator that compute the variance of
 * a general 1D signal in a interval of length \f$w_s\f$ samples
 * centered in \f$t\f$.
 */
void StaticIntervalsDetector(const ImuReadings &samples, double threshold,
                             std::vector<DataInterval> &intervals,
                             int win_size = 101);

} // namespace utils
} // namespace OpenICC
