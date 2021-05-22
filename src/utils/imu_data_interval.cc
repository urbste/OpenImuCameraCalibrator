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

#include "OpenCameraCalibrator/utils/imu_data_interval.h"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <fstream>
#include <vector>

// CODE TAKEN FROM
// https://github.com/Kyle-ak/imu_tk/blob/master/include/imu_tk/base.h

using namespace cv;

namespace OpenICC {
namespace utils {

using Vector3d = Eigen::Vector3d;


Vector3d DataMean(const ImuReadings &samples, const DataInterval &interval) {
  DataInterval rev_interval = CheckInterval(samples, interval);
  int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
  Vector3d mean(0, 0, 0);

  for (int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++)
    mean += samples[i].data();

  mean /= double(n_samp);

  return mean;
}

Vector3d DataVariance(const ImuReadings &samples,
                      const DataInterval &interval) {
  DataInterval rev_interval = CheckInterval(samples, interval);
  int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
  Vector3d mean = DataMean(samples, rev_interval);

  Vector3d variance(0, 0, 0);
  for (int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++) {
    Vector3d diff = samples[i].data() - mean;
    variance += (diff.array() * diff.array()).matrix();
  }
  variance /= double(n_samp - 1);

  return variance;
}

void ExtractIntervalsSamples(const ImuReadings &samples,
                             const std::vector<DataInterval> &intervals,
                             ImuReadings &extracted_samples,
                             std::vector<DataInterval> &extracted_intervals,
                             int interval_n_samps, bool only_means) {
  // Check for valid intervals  (i.e., intervals with at least interval_n_samps
  // samples)
  int n_valid_intervals = 0, n_static_samples;
  for (int i = 0; i < intervals.size(); i++) {
    if ((intervals[i].end_idx - intervals[i].start_idx + 1) >= interval_n_samps)
      n_valid_intervals++;
  }

  if (only_means)
    n_static_samples = n_valid_intervals;
  else
    n_static_samples = n_valid_intervals * interval_n_samps;

  extracted_samples.clear();
  extracted_intervals.clear();
  extracted_samples.reserve(n_static_samples);
  extracted_intervals.reserve(n_valid_intervals);

  // For each valid interval, extract the first interval_n_samps samples
  for (int i = 0; i < intervals.size(); i++) {
    int interval_size = intervals[i].end_idx - intervals[i].start_idx + 1;
    if (interval_size >= interval_n_samps) {
      extracted_intervals.push_back(intervals[i]);
      if (only_means) {
        DataInterval mean_inerval(intervals[i].start_idx, intervals[i].end_idx);
        // Take the timestamp centered in the interval where the mean is
        // computed
        double timestamp =
            samples[intervals[i].start_idx + interval_size / 2].timestamp_s();
        Vector3d mean_val = DataMean(samples, mean_inerval);
        extracted_samples.push_back(ImuReading(timestamp, mean_val));
      } else {
        for (int j = intervals[i].start_idx;
             j < intervals[i].start_idx + interval_n_samps; j++)
          extracted_samples.push_back(samples[j]);
      }
    }
  }
}

void StaticIntervalsDetector(const ImuReadings &samples, double threshold,
                             std::vector<DataInterval> &intervals,
                             int win_size) {
  if (win_size < 11)
    win_size = 11;
  if (!(win_size % 2))
    win_size++;

  int h = win_size / 2;

  if (win_size >= samples.size())
    return;

  intervals.clear();

  bool look_for_start = true;
  DataInterval current_interval;

  for (int i = h; i < samples.size() - h; i++) {
    Vector3d variance = DataVariance(samples, DataInterval(i - h, i + h));
    double norm = variance.norm();

    if (look_for_start) {
      if (norm < threshold) {
        current_interval.start_idx = i;
        look_for_start = false;
      }
    } else {
      if (norm >= threshold) {
        current_interval.end_idx = i - 1;
        look_for_start = true;
        intervals.push_back(current_interval);
      }
    }
  }

  // If the last interval has not been included in the intervals vector
  if (!look_for_start) {
    current_interval.end_idx = samples.size() - h - 1;
    // current_interval.end_ts = samples[current_interval.end_idx].timestamp();
    intervals.push_back(current_interval);
  }
}

} // namespace utils
} // namespace OpenICC
