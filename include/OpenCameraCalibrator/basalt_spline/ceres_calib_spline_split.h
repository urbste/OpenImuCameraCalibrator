#pragma once

#include "assert.h"
#include "common_types.h"
#include "ceres_local_param.h"
#include "eigen_utils.h"
#include "calib_helpers.h"

#include <ceres/ceres.h>
#include "ceres_calib_split_residuals.h"

#include <theia/sfm/camera/division_undistortion_camera_model.h>

template <int _N, bool OLD_TIME_DERIV = false>
class CeresCalibrationSplineSplit {
 public:
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  static constexpr double ns_to_s = 1e-9;  ///< Nanosecond to second conversion
  static constexpr double s_to_ns = 1e9;   ///< Second to nanosecond conversion

  CeresCalibrationSplineSplit(int64_t time_interval_ns,
                              int64_t start_time_ns = 0)
      : dt_ns(time_interval_ns), start_t_ns(start_time_ns) {
    inv_dt = s_to_ns / dt_ns;

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
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    Sophus::SE3d res;

    Sophus::SO3d rot;
    Eigen::Vector3d trans;

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SO3>(
          &vec[0], u, inv_dt, &rot);
    }

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate<double, 3, 0>(&vec[0], u, inv_dt,
                                                            &trans);
    }

    res = Sophus::SE3d(rot, trans);

    return res;
  }

  Eigen::Vector3d getGyro(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    Eigen::Vector3d gyro;

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(so3_knots[s + i].data());
    }

    CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SO3>(
        &vec[0], u, inv_dt, nullptr, &gyro);

    return gyro;
  }

  Eigen::Vector3d getAccel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    Eigen::Vector3d accel;

    Sophus::SO3d rot;
    Eigen::Vector3d trans_accel_world;

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SO3>(
          &vec[0], u, inv_dt, &rot);
    }

    {
      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate<double, 3, 2>(&vec[0], u, inv_dt,
                                                            &trans_accel_world);
    }

    accel = rot.inverse() * (trans_accel_world + g);

    return accel;
  }

  void init(const Sophus::SE3d& init, int num_knots) {
    so3_knots = Eigen::aligned_vector<Sophus::SO3d>(num_knots, init.so3());
    trans_knots =
        Eigen::aligned_vector<Eigen::Vector3d>(num_knots, init.translation());

    // Add local parametrization for SO(3) rotation

    for (int i = 0; i < num_knots; i++) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SO3d>();

      problem.AddParameterBlock(so3_knots[i].data(),
                                Sophus::SO3d::num_parameters,
                                local_parameterization);
    }

    // Local parametrization of T_i_c
    //for (size_t i = 0; i < T_i_c.size(); i++) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SE3d>();

      problem.AddParameterBlock(T_i_c.data(),
                                Sophus::SE3d::num_parameters,
                                local_parameterization);
    //}
  }

  void addGyroMeasurement(const Eigen::Vector3d& meas, const int64_t time_ns, const double weight_so3) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    using FunctorT = CalibGyroCostFunctorSplit<N, Sophus::SO3, OLD_TIME_DERIV>;

    FunctorT* functor = new FunctorT(
        meas, u, inv_dt, weight_so3);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(so3_knots[s + i].data());
    }
    vec.emplace_back(gyro_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addAccelMeasurement(const Eigen::Vector3d& meas, const int64_t time_ns, const double weight_se3) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    using FunctorT = CalibAccelerationCostFunctorSplit<N>;

    FunctorT* functor = new FunctorT(
        meas, u, inv_dt, weight_se3);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(3);
    }
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(so3_knots[s + i].data());
    }
    for (int i = 0; i < N; i++) {
      vec.emplace_back(trans_knots[s + i].data());
    }
    vec.emplace_back(g.data());
    vec.emplace_back(accel_bias.data());

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addCornersMeasurement(const CalibCornerData* corners,
                             const theia::Reconstruction* calib,
                             const theia::Camera* cam,
                             int cam_id,
                             int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(
        size_t(s + N) <= so3_knots.size(),
        "s " << s << " N " << N << " knots.size() " << so3_knots.size());

    using FunctorT = CalibReprojectionCostFunctorSplit<N, theia::DivisionUndistortionCameraModel>;
    FunctorT* functor = new FunctorT(corners,
                                     calib,
                                     cam,
                                     u, inv_dt);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(3);
    }
    // T_i_c
    cost_function->AddParameterBlock(7);

    cost_function->SetNumResiduals(corners->track_ids.size() * 2);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(so3_knots[s + i].data());
    }
    for (int i = 0; i < N; i++) {
      vec.emplace_back(trans_knots[s + i].data());
    }
    vec.emplace_back(T_i_c.data());

    problem.AddResidualBlock(cost_function, NULL, vec);

    //    {
    //      Eigen::VectorXd residual;
    //      residual.setZero(corners->corner_ids.size() * 2);

    //      cost_function->Evaluate(&vec[0], residual.data(), NULL);
    //      std::cerr << "residual " << residual.transpose() << std::endl;
    //    }
  }

  int64_t maxTimeNs() const {
    return start_t_ns + (so3_knots.size() - N + 1) * dt_ns - 1;
  }

  int64_t minTimeNs() const { return start_t_ns; }

  double meanReprojection(
      const std::unordered_map<TimeCamId, CalibCornerData>&
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
          size_t(s + N) <= so3_knots.size(),
          "s " << s << " N " << N << " knots.size() " << so3_knots.size());

      using FunctorT = CalibReprojectionCostFunctorSplit<N, theia::DivisionUndistortionCameraModel>;

      FunctorT* functor =
          new FunctorT(&kv.second,
                       &calib,
                       &calib.View(0)->Camera(),
                       u, inv_dt);

      ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
          new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

      for (int i = 0; i < N; i++) {
        cost_function->AddParameterBlock(4);
      }
      for (int i = 0; i < N; i++) {
        cost_function->AddParameterBlock(3);
      }
      // T_i_c
      cost_function->AddParameterBlock(7);

      cost_function->SetNumResiduals(kv.second.track_ids.size() * 2);

      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots[s + i].data());
      }
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots[s + i].data());
      }
      //vec.emplace_back(calib.T_i_c[kv.first.cam_id].data());
      vec.emplace_back(T_i_c.data());
      {
        Eigen::VectorXd residual;
        residual.setZero(kv.second.track_ids.size() * 2);

        cost_function->Evaluate(&vec[0], residual.data(), NULL);

        for (size_t i = 0; i < kv.second.track_ids.size(); i++) {
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
    options.num_threads = 1;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return summary;
  }

  Sophus::SE3d getKnot(int i) const {
    return Sophus::SE3d(so3_knots[i], trans_knots[i]);
  }

  size_t numKnots() { return so3_knots.size(); }

  void setTracks(const std::vector<theia::TrackId>& a) { track_ids = a; }
  void setCalib(const theia::Reconstruction& c) { calib = c; }
  void setT_i_c(const Sophus::SE3<double>& T) { T_i_c = T; }
  void setNoiseLevels(const double noise_std_gyro,
                      const double noise_std_accl) {
      dicrete_time_gyros_noise_std = noise_std_gyro;
      dicrete_time_accel_noise_std = noise_std_accl;
  }

  void setG(Eigen::Vector3d& a) { g = a; }
  const Eigen::Vector3d& getG() { return g; }

  Eigen::Vector3d getGyroBias() { return gyro_bias; }
  Eigen::Vector3d getAccelBias() { return accel_bias; }

  Sophus::SE3<double> getT_i_c() { return T_i_c; }
 private:
  int64_t dt_ns, start_t_ns;
  double inv_dt;

  Eigen::aligned_vector<Sophus::SO3d> so3_knots;
  Eigen::aligned_vector<Eigen::Vector3d> trans_knots;
  Eigen::Vector3d g, accel_bias, gyro_bias;
  theia::Reconstruction calib;
  std::vector<theia::TrackId> track_ids;

  Sophus::SE3<double> T_i_c;

  double dicrete_time_accel_noise_std = 0.0;
  double dicrete_time_gyros_noise_std = 0.0;


  ceres::Problem problem;
};
