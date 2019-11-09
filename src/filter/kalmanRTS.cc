// created by Steffen Urban 2019, January

#include "OpenCameraCalibrator/filter/kalmanRTS.h"

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unsupported/Eigen/MatrixFunctions>

// C++ port from https://github.com/jannemus/InertialScale

namespace filter {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Matrix31d = Eigen::Matrix<double, 3, 1>;
using Matrix13d = Eigen::Matrix<double, 1, 3>;
using Eigen::MatrixXd;
using Matrix63d = Eigen::Matrix<double, 6, 3>;
// Input, F is NxN Feedback matrix
//        L is NxL Noise effect matrix
//        Qc is Diagonal Spectral Density
//        dt is timestep
// Output, A - Transition Matrix
//         Q - Discrete Process Variance
void lti_disc(const Matrix3d F, const Matrix31d L, const double Qc,
              const double dt, Matrix3d* Q, Matrix3d* A) {
  // close form integration of transition matrix
  // matlabs expm()
  (*A) = (F * dt).exp();

  // close form integration of covariance by matrix fraction decomposition
  Matrix<double, 6, 6> Phi;
  Phi.fill(0.0);
  Phi.block<3, 3>(0, 0) = F;
  Phi.block<3, 3>(3, 3) = -F.transpose();
  Phi.block<3, 3>(0, 3) = L * Qc * L.transpose();
  Matrix63d Temp;
  Temp.fill(0.0);
  Temp.bottomRows<3>() = MatrixXd::Identity(3, 3);
  Phi *= dt;
  Matrix63d AB = Phi.exp() * Temp;
  (*Q) = AB.topRows<3>() * AB.bottomRows<3>().inverse();
}

void FilterVisualPositions(
    const std::map<double, Eigen::Vector3d>& visual_positions,
    const double process_noise, std::vector<Eigen::Vector3d>* accelerations) {
  std::vector<double> visual_positions_rate(visual_positions.size() - 1);
  int i = 0;
  double last_t;
  // find mean camera rate
  for (auto const& time_to_pos : visual_positions) {
    if (i == 0) {
      last_t = time_to_pos.first;
    } else {
      visual_positions_rate[i - 1] = time_to_pos.first - last_t;
      last_t = time_to_pos.first;
    }
    ++i;
  }
  const double mean_dt = std::accumulate(visual_positions_rate.begin(),
                                         visual_positions_rate.end(), 0.0) /
                         (double)visual_positions_rate.size();

  Matrix3d F;
  F << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  const double R = process_noise;
  Matrix31d L(0.0, 0.0, 1.0);
  Matrix13d H(1.0, 0.0, 0.0);
  Matrix31d m0(0.0, 0.0, 0.0);
  Matrix3d P0;
  P0 << 1e4, 0.0, 0.0, 0.0, 1e4, 0.0, 0.0, 0.0, 1e4;

  const std::vector<int> qc_list = {45, 55, 65, 75, 85, 95, 105, 115, 125};
  std::vector<double> lh_list(qc_list.size());

  // identify process noise
  for (int i = 0; i < qc_list.size(); ++i) {
    const int qc = qc_list[i];
    Matrix3d Q, A;
    lti_disc(F, L, qc, mean_dt, &Q, &A);
    double lh = 0.0;
    // kalman filter
    for (int c = 0; c < 3; ++c) {
      std::vector<Matrix3d> kf_P(visual_positions.size());
      std::vector<Vector3d> kf_m(visual_positions.size());
      Vector3d m = m0;
      Matrix3d P = P0;
      int p = 0;
      for (auto const& time_to_poses : visual_positions) {
        Vector3d position = time_to_poses.second;
        m = A * m;
        P = A * P * A.transpose() + Q;
        double nu = position(c) - H * m;
        double S = H * P * H.transpose() + R;

        Vector3d K = P * H.transpose() / S;
        m = m + K * nu;
        P = P - K * S * K.transpose();

        lh = lh + 0.5 * std::log(2.0 * M_PI) + 0.5 * std::log(S) +
             0.5 * nu / S * nu;

        kf_m[p] = m;
        kf_P[p] = P;
        ++p;
      }
    }
    lh_list[i] = lh;
  }

  std::vector<double>::iterator min_lh =
      std::min_element(std::begin(lh_list), std::end(lh_list));
  const int lh_min_idx = std::distance(std::begin(lh_list), min_lh);
  const int qc = qc_list[lh_min_idx];

  std::cout << "Process noise is set to: " << qc;

  std::vector<Vector3d> pos_kfs(visual_positions.size());
  std::vector<Vector3d> vel_kfs(visual_positions.size());
  std::vector<Vector3d> acc_kfs(visual_positions.size());

  Matrix3d Q, A;
  lti_disc(F, L, qc, mean_dt, &Q, &A);

  // kalman filter
  for (int c = 0; c < 3; ++c) {
    std::vector<Matrix3d> kf_P(visual_positions.size());
    std::vector<Vector3d> kf_m(visual_positions.size());
    Vector3d m = m0;
    Matrix3d P = P0;
    int p = 0;
    for (auto const& time_to_poses : visual_positions) {
      Vector3d position = time_to_poses.second;
      m = A * m;
      P = A * P * A.transpose() + Q;
      double nu = position(c) - H * m;
      double S = H * P * H.transpose() + R;

      Vector3d K = P * H.transpose() / S;
      m = m + K * nu;
      P = P - K * S * K.transpose();

      kf_m[p] = m;
      kf_P[p] = P;
      ++p;
    }

    // rts smoothing
    Vector3d ms = m;
    Matrix3d Ps = P;
    std::vector<Vector3d> rts_m(visual_positions.size());
    std::vector<Matrix3d> rts_P(visual_positions.size());
    rts_m[visual_positions.size() - 1] = ms;
    rts_P[visual_positions.size() - 1] = Ps;
    for (int k = visual_positions.size() - 1; k > -1; --k) {
      Vector3d mp = A * kf_m[k];
      Matrix3d Pp = A * kf_P[k] * A.transpose() + Q;
      Matrix3d Ck = kf_P[k] * A.transpose() * Pp.inverse();
      ms = kf_m[k] + Ck * (ms - mp);
      Matrix3d Ps = kf_P[k] + Ck * (Ps - Pp) * Ck.transpose();
      rts_m[k] = ms;
      rts_P[k] = Ps;
    }

    for (int j = 0; j < visual_positions.size(); ++j) {
      pos_kfs[j](c) = rts_m[j](0);
      vel_kfs[j](c) = rts_m[j](1);
      acc_kfs[j](c) = rts_m[j](2);
    }
  }
}
}
