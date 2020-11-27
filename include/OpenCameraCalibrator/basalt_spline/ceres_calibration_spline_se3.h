#pragma once

#include <type_traits>

#include "assert.h"
#include "eigen_utils.h"
#include "third_party/Sophus/sophus/so3.hpp"

#include <ceres/ceres.h>
#include "ceres_calib_se3_residuals.h"
#include "ceres_local_param.h"

template <int _N, bool OLD_TIME_DERIV = false>
class CeresCalibrationSplineSe3 {
 public:
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  static constexpr double ns_to_s = 1e-9;  ///< Nanosecond to second conversion
  static constexpr double s_to_ns = 1e9;   ///< Second to nanosecond conversion

  CeresCalibrationSplineSe3(int64_t time_interval_ns, int64_t start_time_ns = 0)
      : dt_ns(time_interval_ns), start_t_ns(start_time_ns) {
    pow_inv_dt[0] = 1.0;
    pow_inv_dt[1] = s_to_ns / dt_ns;
    pow_inv_dt[2] = pow_inv_dt[1] * pow_inv_dt[1];
    pow_inv_dt[3] = pow_inv_dt[2] * pow_inv_dt[1];

    accel_bias.setZero();
    gyro_bias.setZero();
  };

  Sophus::SE3d getPose(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    Sophus::SE3d res;

    Sophus::SO3d rot;
    Eigen::Vector3d trans;

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    SplineFunctorBase<N>::template evaluate_lie<double, Sophus::SE3>(
        &vec[0], u, pow_inv_dt[1], &res);

    return res;
  }

  Eigen::Vector3d getGyro(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    Sophus::Vector6d gyro;

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    SplineFunctorBase<N>::template evaluate_lie<double, Sophus::SE3>(
        &vec[0], u, pow_inv_dt[1], nullptr, &gyro);

    return gyro.tail<3>();
  }

  Eigen::Vector3d getAccel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    Sophus::SE3d pose;
    Sophus::Vector6d se3_vel, se3_accel;

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(knots[s + i].data());
      }

      SplineFunctorBase<N>::template evaluate_lie<double, Sophus::SE3>(
          &vec[0], u, pow_inv_dt[1], &pose, &se3_vel, &se3_accel);
    }

    Eigen::Matrix4d vel_hat = Sophus::SE3d::hat(se3_vel);
    Eigen::Matrix4d accel_hat = Sophus::SE3d::hat(se3_accel);

    Eigen::Matrix4d ddpose = pose.matrix() * (vel_hat * vel_hat + accel_hat);

    Eigen::Vector3d accel_w = ddpose.col(3).template head<3>();

    Eigen::Vector3d accel_meas = pose.so3().inverse() * (accel_w + g);

    return accel_meas;
  }

  void init(const Sophus::SE3d& init, int num_knots) {
    knots = Eigen::aligned_vector<Sophus::SE3d>(num_knots, init);

    for (int i = 0; i < num_knots; i++) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SE3d>();

      problem.AddParameterBlock(knots[i].data(), Sophus::SE3d::num_parameters,
                                local_parameterization);
    }

    // Local parametrization of T_i_c
    for (int i = 0; i < calib.T_i_c.size(); i++) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SE3d>();

      problem.AddParameterBlock(calib.T_i_c[i].data(),
                                Sophus::SE3d::num_parameters,
                                local_parameterization);
    }
  }

  void addGyroMeasurement(const Eigen::Vector3d& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = CalibGyroCostFunctorSE3<_N, OLD_TIME_DERIV>;

    FunctorT* functor = new FunctorT(
        meas, u, pow_inv_dt[1], 1.0 / calib.dicrete_time_gyro_noise_std()[0]);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(7);
    }
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }
    vec.emplace_back(gyro_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addAccelMeasurement(const Eigen::Vector3d& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = CalibAccelerationCostFunctorSE3<N, OLD_TIME_DERIV>;

    FunctorT* functor = new FunctorT(
        meas, u, pow_inv_dt[1], 1.0 / calib.dicrete_time_accel_noise_std()[0]);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(7);
    }
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }
    vec.emplace_back(g.data());
    vec.emplace_back(accel_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addCornersMeasurement(const basalt::CalibCornerData* corners, int cam_id,
                             int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = CalibReprojectionCostFunctorSE3<N>;

    FunctorT* functor = new FunctorT(
        corners, aprilgrid.get(), calib.intrinsics[cam_id], u, pow_inv_dt[1]);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(7);
    }
    // T_i_c
    cost_function->AddParameterBlock(7);

    cost_function->SetNumResiduals(corners->corner_ids.size() * 2);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }
    vec.emplace_back(calib.T_i_c[cam_id].data());

    problem.AddResidualBlock(cost_function, NULL, vec);

    //    {
    //      Eigen::VectorXd residual;
    //      residual.setZero(corners->corner_ids.size() * 2);

    //      cost_function->Evaluate(&vec[0], residual.data(), NULL);
    //      std::cerr << "residual " << residual.transpose() << std::endl;
    //    }
  }

  int64_t maxTimeNs() const {
    return start_t_ns + (knots.size() - N + 1) * dt_ns - 1;
  }

  int64_t minTimeNs() const { return start_t_ns; }

  double meanReprojection(
      const std::unordered_map<basalt::TimeCamId, basalt::CalibCornerData>&
          calib_corners) const {
    double sum_error = 0;
    int num_points = 0;

    for (const auto& kv : calib_corners) {
      int64_t time_ns = kv.first.frame_id;

      if (time_ns < minTimeNs() || time_ns >= maxTimeNs()) continue;

      int64_t st_ns = (time_ns - start_t_ns);

      BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns "
                                                << time_ns << " start_t_ns "
                                                << start_t_ns);

      int64_t s = st_ns / dt_ns;
      double u = double(st_ns % dt_ns) / double(dt_ns);

      BASALT_ASSERT_STREAM(s >= 0, "s " << s);
      BASALT_ASSERT_STREAM(
          size_t(s + N) <= knots.size(),
          "s " << s << " N " << N << " knots.size() " << knots.size());

      using FunctorT = CalibReprojectionCostFunctorSE3<N>;

      FunctorT* functor =
          new FunctorT(&kv.second, aprilgrid.get(),
                       calib.intrinsics[kv.first.cam_id], u, pow_inv_dt[1]);

      ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
          new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

      for (int i = 0; i < N; i++) {
        cost_function->AddParameterBlock(7);
      }
      // T_i_c
      cost_function->AddParameterBlock(7);

      cost_function->SetNumResiduals(kv.second.corner_ids.size() * 2);

      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(knots[s + i].data());
      }
      vec.emplace_back(calib.T_i_c[kv.first.cam_id].data());

      {
        Eigen::VectorXd residual;
        residual.setZero(kv.second.corner_ids.size() * 2);

        cost_function->Evaluate(&vec[0], residual.data(), NULL);

        for (int i = 0; i < kv.second.corner_ids.size(); i++) {
          Eigen::Vector2d res_point = residual.segment<2>(2 * i);

          if (res_point[0] != 0.0 && res_point[1] != 0.0) {
            sum_error += res_point.norm();
            num_points += 1;
          }
        }
      }
    }

    std::cout << "mean error " << sum_error / num_points << " num_points "
              << num_points << std::endl;

    return sum_error / num_points;
  }

  ceres::Solver::Summary optimize() {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 50;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return summary;
  }

  Sophus::SE3d getKnot(int i) const { return knots[i]; }

  size_t numKnots() { return knots.size(); }

  void setAprilgrid(AprilGrid::Ptr& a) { aprilgrid = a; }

  void setG(Eigen::Vector3d& a) { g = a; }
  const Eigen::Vector3d& getG() { return g; }

  void setCalib(const basalt::Calibration<double>& c) { calib = c; }

  const basalt::Calibration<double>& getCalib() { return calib; }

  Eigen::Vector3d getGyroBias() { return gyro_bias; }
  Eigen::Vector3d getAccelBias() { return accel_bias; }

 private:
  int64_t dt_ns, start_t_ns;
  std::array<double, 4> pow_inv_dt;

  Eigen::aligned_vector<Sophus::SE3d> knots;
  Eigen::Vector3d g, accel_bias, gyro_bias;
  basalt::Calibration<double> calib;

  AprilGrid::Ptr aprilgrid;

  ceres::Problem problem;
};
