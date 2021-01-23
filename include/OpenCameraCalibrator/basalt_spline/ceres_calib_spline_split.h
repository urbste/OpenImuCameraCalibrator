#pragma once

#include "assert.h"
#include "calib_helpers.h"
#include "ceres_local_param.h"
#include "common_types.h"
#include <thread>

#include "ceres_calib_split_residuals.h"
#include <ceres/ceres.h>

#include <theia/sfm/camera/division_undistortion_camera_model.h>

#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace OpenICC;

template <int _N, bool OLD_TIME_DERIV = false>
class CeresCalibrationSplineSplit {
public:
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  CeresCalibrationSplineSplit()
      : dt_so3_ns_(0.1 * S_TO_NS), dt_r3_ns_(0.1 * S_TO_NS), start_t_ns(0.0) {
    inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
    inv_r3_dt_ = S_TO_NS / dt_r3_ns_;
    accel_bias_.setZero();
    gyro_bias_.setZero();
  }

  CeresCalibrationSplineSplit(int64_t time_interval_so3_ns,
                              int64_t time_interval_r3_ns,
                              int64_t start_time_ns = 0)
      : dt_so3_ns_(time_interval_so3_ns), dt_r3_ns_(time_interval_r3_ns),
        start_t_ns(start_time_ns) {
    inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
    inv_r3_dt_ = S_TO_NS / dt_r3_ns_;
    accel_bias_.setZero();
    gyro_bias_.setZero();
  }

  void init_times(int64_t time_interval_so3_ns, int64_t time_interval_r3_ns,
                  int64_t start_time_ns = 0) {
    dt_so3_ns_ = time_interval_so3_ns;
    dt_r3_ns_ = time_interval_r3_ns;
    start_t_ns = start_time_ns;
    inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
    inv_r3_dt_ = S_TO_NS / dt_r3_ns_;
    accel_bias_.setZero();
    gyro_bias_.setZero();
  }

  Sophus::SE3d getPose(int64_t time_ns) const {
    const int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    const int64_t s_so3 = st_ns / dt_so3_ns_;
    double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);
    const int64_t s_r3 = st_ns / dt_r3_ns_;
    double u_r3 = double(st_ns % dt_r3_ns_) / double(dt_r3_ns_);

    BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
    BASALT_ASSERT_STREAM(s_r3 >= 0, "s " << s_r3);

    BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                         "s " << s_so3 << " N " << N << " knots.size() "
                              << so3_knots_.size());
    BASALT_ASSERT_STREAM(size_t(s_r3 + N) <= trans_knots_.size(),
                         "s " << s_r3 << " N " << N << " knots.size() "
                              << trans_knots_.size());

    Sophus::SE3d res;

    Sophus::SO3d rot;
    Eigen::Vector3d trans;

    {
      std::vector<const double *> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots_[s_so3 + i].data());
      }

      CeresSplineHelper<double, N>::template evaluate_lie<Sophus::SO3>(
          &vec[0], u_so3, inv_so3_dt_, &rot);
    }

    {
      std::vector<const double *> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots_[s_r3 + i].data());
      }

      CeresSplineHelper<double, N>::template evaluate<3, 0>(&vec[0], u_r3,
                                                            inv_r3_dt_, &trans);
    }

    res = Sophus::SE3d(rot, trans);

    return res;
  }

  Eigen::Vector3d getGyro(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s_so3 = st_ns / dt_so3_ns_;
    double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);

    BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
    BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                         "s " << s_so3 << " N " << N << " knots.size() "
                              << so3_knots_.size());

    Eigen::Vector3d gyro;

    std::vector<const double *> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(so3_knots_[s_so3 + i].data());
    }

    CeresSplineHelper<double, N>::template evaluate_lie<Sophus::SO3>(
        &vec[0], u_so3, inv_so3_dt_, nullptr, &gyro);

    return gyro;
  }

  Eigen::Vector3d getAccel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    const int64_t s_so3 = st_ns / dt_so3_ns_;
    double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);
    const int64_t s_r3 = st_ns / dt_r3_ns_;
    double u_r3 = double(st_ns % dt_r3_ns_) / double(dt_r3_ns_);

    BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
    BASALT_ASSERT_STREAM(s_r3 >= 0, "s " << s_r3);

    BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                         "s " << s_so3 << " N " << N << " knots.size() "
                              << so3_knots_.size());
    BASALT_ASSERT_STREAM(size_t(s_r3 + N) <= trans_knots_.size(),
                         "s " << s_r3 << " N " << N << " knots.size() "
                              << trans_knots_.size());

    Eigen::Vector3d accel;

    Sophus::SO3d rot;
    Eigen::Vector3d trans_accel_world;

    {
      std::vector<const double *> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots_[s_so3 + i].data());
      }

      CeresSplineHelper<double, N>::template evaluate_lie<Sophus::SO3>(
          &vec[0], u_so3, inv_so3_dt_, &rot);
    }

    {
      std::vector<const double *> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots_[s_r3 + i].data());
      }

      CeresSplineHelper<double, N>::template evaluate<3, 2>(
          &vec[0], u_r3, inv_r3_dt_, &trans_accel_world);
    }

    accel = rot.inverse() * (trans_accel_world + gravity_);

    return accel;
  }

  void initFromSpline(const OpenICC::so3_vector &so3_knots_old,
                      const OpenICC::vec3_vector &trans_knots_old,
                      const Eigen::Vector3d &g_old,
                      const Sophus::SE3<double> &T_i_c_calib_) {

    so3_knots_ = so3_knots_old;
    trans_knots_ = trans_knots_old;
    gravity_ = g_old;
    T_i_c_ = T_i_c_calib_;
  }

  void initAll(
      const std::unordered_map<TimeCamId, CalibInitPoseData> &init_spline_poses,
      const int num_knots_so3, const int num_knots_r3) {

    so3_knots_ = OpenICC::so3_vector(num_knots_so3);
    trans_knots_ = OpenICC::vec3_vector(num_knots_r3);
    so3_knot_in_problem_ = std::vector(num_knots_so3, false);
    r3_knot_in_problem_ = std::vector(num_knots_r3, false);
    // first interpolate spline poses for imu update rate
    // create zero-based maps
    OpenICC::quat_map quat_vis_map;
    OpenICC::vec3_map translations_map;

    // get sorted poses
    for (auto const &data : init_spline_poses) {
      const double t_s = data.first.frame_id * NS_TO_S;
      quat_vis_map[t_s] = data.second.T_a_c.so3().unit_quaternion();
      translations_map[t_s] = data.second.T_a_c.translation();
    }

    OpenICC::quat_vector quat_vis;
    OpenICC::vec3_vector translations;
    std::vector<double> t_vis;
    for (auto const &q : quat_vis_map) {
      quat_vis.push_back(q.second);
      t_vis.push_back(q.first);
    }

    for (auto const &t : translations_map) {
      translations.push_back(t.second);
    }

    // get time at which we want to interpolate
    std::vector<double> t_so3_spline, t_r3_spline;
    for (int i = 0; i < num_knots_so3; ++i) {
      const double t = i * dt_so3_ns_ * NS_TO_S;
      t_so3_spline.push_back(t);
    }

    for (int i = 0; i < num_knots_r3; ++i) {
      const double t = i * dt_r3_ns_ * NS_TO_S;
      t_r3_spline.push_back(t);
    }

    OpenICC::quat_vector interp_spline_quats;
    OpenICC::vec3_vector interpo_spline_trans;
    OpenICC::utils::InterpolateQuaternions(t_vis, t_so3_spline, quat_vis,
                                           interp_spline_quats);
    OpenICC::utils::InterpolateVector3d(t_vis, t_r3_spline, translations,
                                        interpo_spline_trans);

    for (int i = 0; i < num_knots_so3; ++i) {
      so3_knots_[i] = Sophus::SO3d(interp_spline_quats[i]);
    }
    for (int i = 0; i < num_knots_r3; ++i) {
      trans_knots_[i] = interpo_spline_trans[i];
    }

    // Add local parametrization for SO(3) rotation
    for (int i = 0; i < num_knots_so3; i++) {
      ceres::LocalParameterization *local_parameterization =
          new LieLocalParameterization<Sophus::SO3d>();

      problem_.AddParameterBlock(so3_knots_[i].data(),
                                 Sophus::SO3d::num_parameters,
                                 local_parameterization);
    }
    ceres::LocalParameterization *local_parameterization =
        new LieLocalParameterization<Sophus::SE3d>();

    problem_.AddParameterBlock(T_i_c_.data(), Sophus::SE3d::num_parameters,
                               local_parameterization);
  }

  void init(const Sophus::SE3d &init, const int num_knots_so3,
            const int num_knots_r3) {

    so3_knots_ = OpenICC::so3_vector(num_knots_so3, init.so3());
    trans_knots_ = OpenICC::vec3_vector(num_knots_r3, init.translation());

    // Add local parametrization for SO(3) rotation
    for (int i = 0; i < num_knots_so3; i++) {
      ceres::LocalParameterization *local_parameterization =
          new LieLocalParameterization<Sophus::SO3d>();

      problem_.AddParameterBlock(so3_knots_[i].data(),
                                 Sophus::SO3d::num_parameters,
                                 local_parameterization);
    }

    ceres::LocalParameterization *local_parameterization =
        new LieLocalParameterization<Sophus::SE3d>();

    problem_.AddParameterBlock(T_i_c_.data(), Sophus::SE3d::num_parameters,
                               local_parameterization);
  }

  void addGyroMeasurement(const Eigen::Vector3d &meas, const int64_t time_ns,
                          const double weight_so3, const bool calib_bias) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_so3_ns_;
    double u = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= so3_knots_.size(),
                         "s " << s << " N " << N << " knots.size() "
                              << so3_knots_.size());

    using FunctorT = CalibGyroCostFunctorSplit<N, Sophus::SO3, OLD_TIME_DERIV>;

    FunctorT *functor =
        new FunctorT(meas, u, inv_so3_dt_, weight_so3, calib_bias);

    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    if (calib_bias) {
      cost_function->AddParameterBlock(3);
    }
    cost_function->SetNumResiduals(3);

    std::vector<double *> vec;
    for (int i = 0; i < N; i++) {
        const int t = s + i;
      vec.emplace_back(so3_knots_[t].data());
      so3_knot_in_problem_[t] = true;
    }
    if (calib_bias) {
      vec.emplace_back(gyro_bias_.data());
    }
    problem_.AddResidualBlock(cost_function, NULL, vec);
  }

  void addAccelMeasurement(const Eigen::Vector3d &meas, const int64_t time_ns,
                           const double weight_se3, const bool calib_bias) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s_so3 = st_ns / dt_so3_ns_;
    double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);
    int64_t s_r3 = st_ns / dt_r3_ns_;
    double u_r3 = double(st_ns % dt_r3_ns_) / double(dt_r3_ns_);

    BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
    BASALT_ASSERT_STREAM(s_r3 >= 0, "s " << s_r3);

    BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                         "s " << s_so3 << " N " << N << " knots.size() "
                              << so3_knots_.size());
    BASALT_ASSERT_STREAM(size_t(s_r3 + N) <= trans_knots_.size(),
                         "s " << s_r3 << " N " << N << " knots.size() "
                              << trans_knots_.size());

    using FunctorT = CalibAccelerationCostFunctorSplit<N>;

    FunctorT *functor = new FunctorT(meas, u_r3, inv_r3_dt_, u_so3, inv_so3_dt_,
                                     weight_se3, calib_bias);

    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(3);
    }
    // gravity
    cost_function->AddParameterBlock(3);
    // bias
    if (calib_bias) {
      cost_function->AddParameterBlock(3);
    }

    cost_function->SetNumResiduals(3);

    std::vector<double *> vec;
    for (int i = 0; i < N; i++) {
      const int t = s_so3 + i;
      vec.emplace_back(so3_knots_[t].data());
      so3_knot_in_problem_[t] = true;
    }
    for (int i = 0; i < N; i++) {
      const int t = s_r3 + i;
      vec.emplace_back(trans_knots_[t].data());
      r3_knot_in_problem_[t] = true;
    }
    vec.emplace_back(gravity_.data());
    if (calib_bias) {
      vec.emplace_back(accel_bias_.data());
    }

    problem_.AddResidualBlock(cost_function, NULL, vec);
  }

  void addRSCornersMeasurement(const CalibCornerData *corners,
                               const theia::Reconstruction *calib_,
                               const theia::Camera *cam, int64_t time_ns,
                               const double robust_loss_width = 3.0) {
    const int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    const int64_t s_so3 = st_ns / dt_so3_ns_;
    double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);
    const int64_t s_r3 = st_ns / dt_r3_ns_;
    double u_r3 = double(st_ns % dt_r3_ns_) / double(dt_r3_ns_);

    BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
    BASALT_ASSERT_STREAM(s_r3 >= 0, "s " << s_r3);

    BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                         "s " << s_so3 << " N " << N << " knots.size() "
                              << so3_knots_.size());
    BASALT_ASSERT_STREAM(size_t(s_r3 + N) <= trans_knots_.size(),
                         "s " << s_r3 << " N " << N << " knots.size() "
                              << trans_knots_.size());

    using FunctorT = CalibRSReprojectionCostFunctorSplit<N>;
    FunctorT *functor = new FunctorT(corners, calib_, cam, u_so3, u_r3,
                                     inv_so3_dt_, inv_r3_dt_, 1.0);

    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(4);
    }
    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(3);
    }
    // T_i_c_ -> quaternion
    cost_function->AddParameterBlock(7);

    // cam line delay time
    cost_function->AddParameterBlock(1);

    cost_function->SetNumResiduals(corners->track_ids.size() * 2);

    std::vector<double *> vec;
    for (int i = 0; i < N; i++) {
      const int t = s_so3 + i;
      vec.emplace_back(so3_knots_[t].data());
      so3_knot_in_problem_[t] = true;
    }
    for (int i = 0; i < N; i++) {
      const int t = s_r3 + i;
      vec.emplace_back(trans_knots_[t].data());
      r3_knot_in_problem_[t] = true;
    }
    // camera to imu transformation
    vec.emplace_back(T_i_c_.data());

    // line delay for rolling shutter cameras
    vec.emplace_back(&cam_line_delay_s_);

    ceres::LossFunction *loss_function =
        new ceres::HuberLoss(robust_loss_width);
    problem_.AddResidualBlock(cost_function, loss_function, vec);
  }

  int64_t maxTimeNs() const {
    return start_t_ns + (so3_knots_.size() - N + 1) * dt_so3_ns_ - 1;
  }

  int64_t minTimeNs() const { return start_t_ns; }

  double meanRSReprojection(const std::unordered_map<TimeCamId, CalibCornerData>
                                &calib__corners) const {
    double sum_error = 0;
    int num_points = 0;

    for (const auto &kv : calib__corners) {
      const int64_t time_ns = kv.first.frame_id;

      if (time_ns < minTimeNs() || time_ns >= maxTimeNs())
        continue;

      const int64_t st_ns = (time_ns - start_t_ns);

      BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns "
                                                << time_ns << " start_t_ns "
                                                << start_t_ns);

      const int64_t s_so3 = st_ns / dt_so3_ns_;
      const double u_so3 = double(st_ns % dt_so3_ns_) / double(dt_so3_ns_);
      int64_t s_r3 = st_ns / dt_r3_ns_;
      const double u_r3 = double(st_ns % dt_r3_ns_) / double(dt_r3_ns_);

      BASALT_ASSERT_STREAM(s_so3 >= 0, "s " << s_so3);
      BASALT_ASSERT_STREAM(s_r3 >= 0, "s " << s_r3);

      BASALT_ASSERT_STREAM(size_t(s_so3 + N) <= so3_knots_.size(),
                           "s " << s_so3 << " N " << N << " knots.size() "
                                << so3_knots_.size());
      BASALT_ASSERT_STREAM(size_t(s_r3 + N) <= trans_knots_.size(),
                           "s " << s_r3 << " N " << N << " knots.size() "
                                << trans_knots_.size());

      using FunctorT = CalibRSReprojectionCostFunctorSplit<N>;

      FunctorT *functor =
          new FunctorT(&kv.second, &calib_, &calib_.View(0)->Camera(), u_so3,
                       u_r3, inv_so3_dt_, inv_r3_dt_);

      ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
          new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

      for (int i = 0; i < N; ++i) {
        cost_function->AddParameterBlock(4);
      }
      for (int i = 0; i < N; ++i) {
        cost_function->AddParameterBlock(3);
      }
      // T_i_c_
      cost_function->AddParameterBlock(7);
      cost_function->AddParameterBlock(1);

      cost_function->SetNumResiduals(kv.second.track_ids.size() * 2);

      std::vector<const double *> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(so3_knots_[s_so3 + i].data());
      }
      for (int i = 0; i < N; i++) {
        vec.emplace_back(trans_knots_[s_r3 + i].data());
      }
      vec.emplace_back(T_i_c_.data());
      vec.emplace_back(&cam_line_delay_s_);

      {
        Eigen::VectorXd residual;
        residual.setZero(kv.second.track_ids.size() * 2);

        cost_function->Evaluate(&vec[0], residual.data(), NULL);

        for (size_t i = 0; i < kv.second.track_ids.size(); i++) {
          Eigen::Vector2d res_point = residual.segment<2>(2 * i);
          // res_point /= 1000.0;
          if (res_point[0] != 0.0 && res_point[1] != 0.0) {
            sum_error += res_point.norm();
            num_points += 1;
          }
        }
      }
    }

    std::cout << "mean rolling shutter reproj error " << sum_error / num_points
              << " num_points " << num_points << std::endl;

    return sum_error / num_points;
  }

  ceres::Solver::Summary optimize(const int iterations,
                                  const bool fix_so3_spline,
                                  const bool fix_r3_spline,
                                  const bool fix_T_i_c,
                                  const bool fix_line_delay) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = iterations;
    options.num_threads = std::thread::hardware_concurrency();
    // options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = true;

    if (fix_so3_spline) {
        for (int i = 0; i < so3_knots_.size(); ++i) {
            if (so3_knot_in_problem_[i]) {
                problem_.SetParameterBlockConstant(so3_knots_[i].data());
            }
        }
    }
    if (fix_r3_spline) {
        for (int i = 0; i < trans_knots_.size(); ++i) {
            if (r3_knot_in_problem_[i]) {
                problem_.SetParameterBlockConstant(trans_knots_[i].data());
            }
        }
    }
    if (fix_line_delay) {
      problem_.SetParameterBlockConstant(&cam_line_delay_s_);
    } else {
      problem_.SetParameterBlockVariable(&cam_line_delay_s_);
      problem_.SetParameterBlockConstant(gravity_.data());
      problem_.SetParameterLowerBound(&cam_line_delay_s_, 0, 1e-6);
      problem_.SetParameterUpperBound(&cam_line_delay_s_, 0, 1e-4);
    }
    if (fix_T_i_c) {
      problem_.SetParameterBlockConstant(T_i_c_.data());
    }
    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    std::cout << summary.FullReport() << std::endl;

    return summary;
  }

  Sophus::SE3d getKnot(int i) const {
    return Sophus::SE3d(so3_knots_[i], trans_knots_[i]);
  }

  size_t numSO3Knots() { return so3_knots_.size(); }
  size_t numSE3Knots() { return trans_knots_.size(); }

  void setTracks(const std::vector<theia::TrackId> &a) { track_ids_ = a; }
  void setCalib(const theia::Reconstruction &c) { calib_ = c; }
  void setT_i_c(const Sophus::SE3<double> &T) { T_i_c_ = T; }

  void setG(Eigen::Vector3d &a) { gravity_ = a; }
  const Eigen::Vector3d &getG() { return gravity_; }

  Eigen::Vector3d getGyroBias() { return gyro_bias_; }
  Eigen::Vector3d getAccelBias() { return accel_bias_; }

  Sophus::SE3<double> getT_i_c() { return T_i_c_; }

  void Clear() {
    so3_knots_.clear();
    trans_knots_.clear();
    track_ids_.clear();
    gravity_.setZero();
    accel_bias_.setZero();
    gyro_bias_.setZero();
  }

  void SetInitialRSLineDelay(const double cam_line_delay) {
    cam_line_delay_s_ = cam_line_delay;
  }

  double GetOptimizedRSLineDelay() { return cam_line_delay_s_; }

  void FixT_i_c_() { fix_T_i_c_ = true; }

  void GetSO3Knots(OpenICC::so3_vector &so3_knots__out) {
    so3_knots__out = so3_knots_;
  }
  void GetR3Knots(OpenICC::vec3_vector &r3_knots_out) {
    r3_knots_out = trans_knots_;
  }

private:
  int64_t dt_so3_ns_, dt_r3_ns_, start_t_ns;
  double inv_so3_dt_, inv_r3_dt_;
  double cam_line_delay_s_ = 0.0;
  bool fix_T_i_c_ = false;

  OpenICC::so3_vector so3_knots_;
  OpenICC::vec3_vector trans_knots_;
  std::vector<bool> so3_knot_in_problem_;
  std::vector<bool> r3_knot_in_problem_;
  Eigen::Vector3d gravity_, accel_bias_, gyro_bias_;
  theia::Reconstruction calib_;
  std::vector<theia::TrackId> track_ids_;

  Sophus::SE3<double> T_i_c_;

  ceres::Problem problem_;
};
