#include "OpenCameraCalibrator/core/spline_trajectory_estimator.h"

#include <theia/theia.h>

namespace OpenICC {
namespace core {

template <int _T>
SplineTrajectoryEstimator<_T>::SplineTrajectoryEstimator()
    : dt_so3_ns_(0.1 * S_TO_NS),
      dt_r3_ns_(0.1 * S_TO_NS),
      start_t_ns_(0.0),
      gravity_(Eigen::Vector3d(0, 0, GRAVITY_MAGN)) {
  inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
  inv_r3_dt_ = S_TO_NS / dt_r3_ns_;

  accl_intrinsics_ << 0, 0, 0, 1, 1, 1;
  gyro_intrinsics_ << 0, 0, 0, 0, 0, 0, 1, 1, 1;
  accl_bias_.setZero();
  gyro_bias_.setZero();
}

template <int _T>
SplineTrajectoryEstimator<_T>::SplineTrajectoryEstimator(
    int64_t time_interval_so3_ns,
    int64_t time_interval_r3_ns,
    int64_t start_time_ns)
    : dt_so3_ns_(time_interval_so3_ns),
      dt_r3_ns_(time_interval_r3_ns),
      start_t_ns_(start_time_ns),
      gravity_(Eigen::Vector3d(0, 0, GRAVITY_MAGN)) {
  inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
  inv_r3_dt_ = S_TO_NS / dt_r3_ns_;

  accl_intrinsics_ << 0, 0, 0, 1, 1, 1;
  gyro_intrinsics_ << 0, 0, 0, 0, 0, 0, 1, 1, 1;
  accl_bias_.setZero();
  gyro_bias_.setZero();
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetTimes(int64_t time_interval_so3_ns,
                                             int64_t time_interval_r3_ns,
                                             int64_t start_time_ns,
                                             int64_t end_time_ns) {
  dt_so3_ns_ = time_interval_so3_ns;
  dt_r3_ns_ = time_interval_r3_ns;
  start_t_ns_ = start_time_ns;
  end_t_ns_ = end_time_ns;
  const int64_t duration = end_t_ns_ - start_t_ns_;
  nr_knots_so3_ = duration / dt_so3_ns_ + _T;
  nr_knots_r3_ = duration / dt_r3_ns_ + _T;
  inv_so3_dt_ = S_TO_NS / dt_so3_ns_;
  inv_r3_dt_ = S_TO_NS / dt_r3_ns_;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetFixedParams(const int flags) {
  // if IMU to Cam trafo should be optimized
  if (problem_.HasParameterBlock(T_i_c_.data())) {
    if (!(flags & SplineOptimFlags::T_I_C)) {
      problem_.SetParameterBlockConstant(T_i_c_.data());
      LOG(INFO) << "Keeping T_I_C constant.";
    } else {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SE3d>();
      problem_.SetParameterization(T_i_c_.data(), local_parameterization);
      problem_.SetParameterBlockVariable(T_i_c_.data());
      LOG(INFO) << "Optimizing T_I_C.";
    }
  }

  // if IMU to Cam trafo should be optimized
  if (problem_.HasParameterBlock(&cam_line_delay_s_) &&
      cam_line_delay_s_ != 0.0) {
    if (!(flags & SplineOptimFlags::CAM_LINE_DELAY)) {
      problem_.SetParameterBlockConstant(&cam_line_delay_s_);
      LOG(INFO) << "Keeping camera line delay constant at: "
                << cam_line_delay_s_;
    } else {
      problem_.SetParameterBlockVariable(&cam_line_delay_s_);
      LOG(INFO) << "Optimizing camera line delay.";
    }
  }

  // if IMU to Cam trafo should be optimized
  if (problem_.HasParameterBlock(gravity_.data())) {
    if (!(flags & SplineOptimFlags::GRAVITY_DIR)) {
      LOG(INFO) << "Keeping gravity direction constant at: "
                << gravity_.transpose();

      problem_.SetParameterBlockConstant(gravity_.data());
    } else {
      if (problem_.HasParameterBlock(gravity_.data()))
        problem_.SetParameterBlockVariable(gravity_.data());
      LOG(INFO) << "Optimizing gravity direction.";
    }
  }

  // if world points should be optimized
  if (!(flags & SplineOptimFlags::POINTS)) {
    LOG(INFO) << "Keeping object points constant.";
    for (const auto& tid : tracks_in_problem_) {
      const auto track = image_data_.MutableTrack(tid)->MutablePoint()->data();
      if (problem_.HasParameterBlock(track))
        problem_.SetParameterBlockConstant(track);
    }
  } else {
    for (const auto& tid : tracks_in_problem_) {
      const auto track = image_data_.MutableTrack(tid)->MutablePoint()->data();
      if (problem_.HasParameterBlock(track)) {
        problem_.SetParameterBlockVariable(track);
        ceres::LocalParameterization* local_parameterization =
            new ceres::HomogeneousVectorParameterization(4);
        problem_.SetParameterization(track, local_parameterization);
      }
    }
    LOG(INFO) << "Optimizing object points.";
  }

  // if imu intrinics should be optimized
  if (problem_.HasParameterBlock(accl_intrinsics_.data()) &&
      problem_.HasParameterBlock(gyro_intrinsics_.data())) {
    if (!(flags & SplineOptimFlags::IMU_INTRINSICS)) {
      LOG(INFO) << "Keeping IMU intrinsics constant.";
      problem_.SetParameterBlockConstant(accl_intrinsics_.data());
      problem_.SetParameterBlockConstant(gyro_intrinsics_.data());
    } else {
      problem_.SetParameterBlockVariable(accl_intrinsics_.data());
      problem_.SetParameterBlockVariable(gyro_intrinsics_.data());
      LOG(INFO) << "Optimizing IMU intrinsics.";
    }
  }

  // if imu intrinics should be optimized
  if (problem_.HasParameterBlock(accl_bias_.data()) &&
      problem_.HasParameterBlock(gyro_bias_.data())) {
    if (!(flags & SplineOptimFlags::IMU_BIASES)) {
      LOG(INFO) << "Keeping IMU biases constant.";
      problem_.SetParameterBlockConstant(accl_bias_.data());
      problem_.SetParameterBlockConstant(gyro_bias_.data());
    } else {
      problem_.SetParameterBlockVariable(accl_bias_.data());
      problem_.SetParameterBlockVariable(gyro_bias_.data());
      LOG(INFO) << "Optimizing IMU biases and intrinsics.";
    }
  }

  // add local parametrization for SO(3)
  for (size_t i = 0; i < so3_knots_.size(); ++i) {
    if (problem_.HasParameterBlock(so3_knots_[i].data())) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Sophus::SO3d>();

      problem_.SetParameterization(so3_knots_[i].data(),
                                   local_parameterization);
    }
  }
  if (!(flags & SplineOptimFlags::SPLINE)) {
    // set knots constant if asked
    for (size_t i = 0; i < r3_knots_.size(); ++i) {
      if (problem_.HasParameterBlock(r3_knots_[i].data())) {
        problem_.SetParameterBlockConstant(r3_knots_[i].data());
      }
    }
    for (size_t i = 0; i < so3_knots_.size(); ++i) {
      if (problem_.HasParameterBlock(so3_knots_[i].data())) {
        problem_.SetParameterBlockConstant(so3_knots_[i].data());
      }
    }
  } else {
    // set knots constant if asked
    for (size_t i = 0; i < r3_knots_.size(); ++i) {
      if (problem_.HasParameterBlock(r3_knots_[i].data())) {
        problem_.SetParameterBlockVariable(r3_knots_[i].data());
      }
    }
    for (size_t i = 0; i < so3_knots_.size(); ++i) {
      if (problem_.HasParameterBlock(so3_knots_[i].data())) {
        problem_.SetParameterBlockVariable(so3_knots_[i].data());
      }
    }
  }
}

template <int _T>
ceres::Solver::Summary SplineTrajectoryEstimator<_T>::Optimize(
    const int max_iters, const int flags) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = max_iters;
  options.num_threads = std::thread::hardware_concurrency();
  options.minimizer_progress_to_stdout = true;

  SetFixedParams(flags);

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  std::cout << summary.FullReport() << std::endl;

  return summary;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::BatchInitSO3R3VisPoses() {
  so3_knots_ = OpenICC::so3_vector(nr_knots_so3_);
  r3_knots_ = vec3_vector(nr_knots_r3_);
  so3_knot_in_problem_ = std::vector(nr_knots_so3_, false);
  r3_knot_in_problem_ = std::vector(nr_knots_r3_, false);
  // first interpolate spline poses for imu update rate
  // create zero-based maps
  OpenICC::quat_map quat_vis_map;
  OpenICC::vec3_map translations_map;

  // get sorted poses
  const auto view_ids = image_data_.ViewIds();
  for (const auto& vid : view_ids) {
    const auto* v = image_data_.View(vid);
    const double t_s = v->GetTimestamp();
    const auto q_w_c = Eigen::Quaterniond(
        v->Camera().GetOrientationAsRotationMatrix().transpose());
    const Sophus::SE3d T_w_c(q_w_c, v->Camera().GetPosition());
    const Sophus::SE3d T_w_i = T_w_c * T_i_c_.inverse();
    quat_vis_map[t_s] = T_w_i.so3().unit_quaternion();
    translations_map[t_s] = T_w_i.translation();
  }

  OpenICC::quat_vector quat_vis;
  OpenICC::vec3_vector translations;
  std::vector<double> t_vis;
  for (auto const& q : quat_vis_map) {
    quat_vis.push_back(q.second);
    t_vis.push_back(q.first);
  }

  for (auto const& t : translations_map) {
    translations.push_back(t.second);
  }

  // get time at which we want to interpolate
  std::vector<double> t_so3_spline, t_r3_spline;
  for (int i = 0; i < nr_knots_so3_; ++i) {
    const double t = i * dt_so3_ns_ * NS_TO_S;
    t_so3_spline.push_back(t);
  }

  for (int i = 0; i < nr_knots_r3_; ++i) {
    const double t = i * dt_r3_ns_ * NS_TO_S;
    t_r3_spline.push_back(t);
  }

  OpenICC::quat_vector interp_spline_quats;
  OpenICC::vec3_vector interpo_spline_trans;
  OpenICC::utils::InterpolateQuaternions(
      t_vis, t_so3_spline, quat_vis, interp_spline_quats);
  OpenICC::utils::InterpolateVector3d(
      t_vis, t_r3_spline, translations, interpo_spline_trans);

  for (int i = 0; i < nr_knots_so3_; ++i) {
    so3_knots_[i] = Sophus::SO3d(interp_spline_quats[i]);
  }
  for (int i = 0; i < nr_knots_r3_; ++i) {
    r3_knots_[i] = interpo_spline_trans[i];
  }
}

// template <int _T> void SplineTrajectoryEstimator<_T>::InitR3WithGPS() {

//  r3_knots_ = vec3_vector(nr_knots_r3_);
//  r3_knot_in_problem_ = std::vector(nr_knots_r3_, false);

//  // first interpolate spline poses for imu update rate
//  // create zero-based maps
//  vec3_map gps_ecef_map;

//  // get sorted gps poses and times
//  for (size_t i = 0; i < telemetry_data_.gps.timestamp_ms.size(); ++i) {
//    const double t_s = telemetry_data_.gps.timestamp_ms[i] * MS_TO_S;
//    gps_ecef_map[t_s] = telemetry_data_.gps.ecef[i];
//  }

//  std::vector<double> t_gps;
//  vec3_vector gps_ecef_poses;
//  for (auto const &gps : gps_ecef_map) {
//    t_gps.push_back(gps.first);
//    gps_ecef_poses.push_back(gps.second);
//  }
//  // get spline times
//  std::vector<double> t_r3_spline;
//  for (size_t i = 0; i < nr_knots_r3_; ++i) {
//    const double t = (i * dt_r3_ns_ + start_t_ns_) * NS_TO_S;
//    t_r3_spline.push_back(t);
//  }

//  vec3_vector interpo_spline_pos;
//  utils::InterpolateVector3d(t_gps, t_r3_spline, gps_ecef_poses,
//                             interpo_spline_pos);

//  for (size_t i = 0; i < nr_knots_r3_; ++i) {
//    r3_knots_[i] = interpo_spline_pos[i];
//  }
//  spline_initialized_with_gps_ = true;
//  LOG(INFO) << "Initialized the spline with GPS coordinates.";
//}

// template <int _T> void SplineTrajectoryEstimator<_T>::InitR3Knots() {
//  r3_knots_ = vec3_vector(nr_knots_r3_);
//  r3_knot_in_problem_ = std::vector(nr_knots_r3_, false);
//  for (auto i = 0; i < nr_knots_r3_; ++i) {
//    r3_knots_[i].setZero();
//  }
//}

// template <int _T> void SplineTrajectoryEstimator<_T>::InitSO3Knots() {
//  so3_knots_ = so3_vector(nr_knots_so3_);
//  so3_knot_in_problem_ = std::vector(nr_knots_so3_, false);

//  // Add local parametrization for SO(3) rotation
//  for (size_t i = 0; i < nr_knots_so3_; ++i) {
//    ceres::LocalParameterization *local_parameterization =
//        new LieLocalParameterization<Sophus::SO3d>();

//    problem_.AddParameterBlock(so3_knots_[i].data(),
//                               Sophus::SO3d::num_parameters,
//                               local_parameterization);
//  }
//}

// template <int _T>
// bool SplineTrajectoryEstimator<_T>::AddGPSMeasurement(
//    const Eigen::Vector3d &meas, const int64_t time_ns,
//    const double weight_gps) {
//  double u_r3;
//  int64_t s_r3;
//  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
//    LOG(INFO) << "Wrong time adding GPS measurements.";
//    return false;
//  }

//  using FunctorT = GPSCostFunctor<N_>;
//  FunctorT *functor = new FunctorT(meas, u_r3, inv_r3_dt_, weight_gps);

//  ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
//      new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

//  for (int i = 0; i < N_; i++) {
//    cost_function->AddParameterBlock(3);
//  }
//  cost_function->SetNumResiduals(3);

//  std::vector<double *> vec;
//  for (int i = 0; i < N_; i++) {
//    const int t = s_r3 + i;
//    vec.emplace_back(r3_knots_[t].data());
//    r3_knot_in_problem_[t] = true;
//  }
//  problem_.AddResidualBlock(cost_function, NULL, vec);
//  return true;
//}

template <int _T>
bool SplineTrajectoryEstimator<_T>::AddAccelerometerMeasurement(
    const Eigen::Vector3d& meas,
    const int64_t time_ns,
    const double weight_se3) {
  double u_r3, u_so3;
  int64_t s_r3, s_so3;
  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
    LOG(INFO) << "Wrong time adding r3 accelerometer measurements. time_ns: "
              << time_ns << " u_r3: " << u_r3 << " s_r3:" << s_r3;
    return false;
  }
  if (!CalcSO3Times(time_ns, u_so3, s_so3)) {
    LOG(INFO) << "Wrong time adding so3 accelerometer measurements. time_ns: "
              << time_ns << " u_r3: " << u_r3 << " s_r3:" << s_r3;
    return false;
  }

  using FunctorT = AccelerationCostFunctorSplit<N_>;
  FunctorT* functor =
      new FunctorT(meas, u_r3, inv_r3_dt_, u_so3, inv_so3_dt_, weight_se3);

  ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
      new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

  std::vector<double*> vec;
  // so3
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(4);
    const int t = s_so3 + i;
    vec.emplace_back(so3_knots_[t].data());
    so3_knot_in_problem_[t] = true;
  }

  // r3
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(3);
    const int t = s_r3 + i;
    vec.emplace_back(r3_knots_[t].data());
    r3_knot_in_problem_[t] = true;
  }
  // gravity
  cost_function->AddParameterBlock(3);
  vec.emplace_back(gravity_.data());

  // imu intrinsics and bias
  cost_function->AddParameterBlock(6);
  vec.emplace_back(accl_intrinsics_.data());
  cost_function->AddParameterBlock(3);
  vec.emplace_back(accl_bias_.data());

  // number of residuals
  cost_function->SetNumResiduals(3);

  problem_.AddResidualBlock(cost_function, NULL, vec);

  for (int i = 0; i < 3; ++i) {
    problem_.SetParameterLowerBound(accl_bias_.data(), i, -0.5);
    problem_.SetParameterUpperBound(accl_bias_.data(), i, 0.5);
  }
  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::AddGyroscopeMeasurement(
    const Eigen::Vector3d& meas,
    const int64_t time_ns,
    const double weight_so3) {
  double u_so3;
  int64_t s_so3;
  if (!CalcSO3Times(time_ns, u_so3, s_so3)) {
    LOG(INFO) << "Wrong time adding so3 gyroscope measurements. time_ns: "
              << time_ns << " u_r3: " << u_so3 << " s_r3:" << s_so3;
    return false;
  }

  using FunctorT = GyroCostFunctorSplit<N_, Sophus::SO3, false>;
  FunctorT* functor = new FunctorT(meas, u_so3, inv_so3_dt_, weight_so3);

  ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
      new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

  std::vector<double*> vec;
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(4);
    const int t = s_so3 + i;
    vec.emplace_back(so3_knots_[t].data());
    so3_knot_in_problem_[t] = true;
  }

  // bias and intrinsics
  cost_function->AddParameterBlock(9);
  vec.emplace_back(gyro_intrinsics_.data());
  cost_function->AddParameterBlock(3);
  vec.emplace_back(gyro_bias_.data());

  cost_function->SetNumResiduals(3);

  problem_.AddResidualBlock(cost_function, NULL, vec);

  // gyro biases are usually very small numbers < 1e-2
  for (int i = 0; i < 3; ++i) {
    problem_.SetParameterLowerBound(gyro_bias_.data(), i, -1e-2);
    problem_.SetParameterUpperBound(gyro_bias_.data(), i, 1e-2);
  }

  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::AddGSCameraMeasurement(
    const theia::View* view, const double robust_loss_width) {
  const int64_t image_obs_time_ns = view->GetTimestamp() * S_TO_NS;
  const auto track_ids = view->TrackIds();

  double u_r3 = 0.0, u_so3 = 0.0;
  int64_t s_r3 = 0, s_so3 = 0;
  if (!CalcR3Times(image_obs_time_ns, u_r3, s_r3)) {
    LOG(INFO) << "Wrong time observation r3 vision measurements. time_ns: "
              << image_obs_time_ns << " u_r3: " << u_r3 << " s_r3:" << s_r3;
    return false;
  }
  if (!CalcSO3Times(image_obs_time_ns, u_so3, s_so3)) {
    LOG(INFO) << "Wrong time reference so3 vision measurements. time_ns: "
              << image_obs_time_ns << " u_r3: " << u_so3 << " s_r3:" << s_so3;
    return false;
  }

  using FunctorT = GSReprojectionCostFunctorSplit<N_>;
  FunctorT* functor = new FunctorT(
      view, &image_data_, u_so3, u_r3, inv_so3_dt_, inv_r3_dt_, track_ids);

  ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
      new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

  std::vector<double*> vec;
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(4);
    const int t = s_so3 + i;
    vec.emplace_back(so3_knots_[t].data());
    so3_knot_in_problem_[t] = true;
  }
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(3);
    const int t = s_r3 + i;
    vec.emplace_back(r3_knots_[t].data());
    r3_knot_in_problem_[t] = true;
  }

  // camera to imu transformation
  cost_function->AddParameterBlock(7);
  vec.emplace_back(T_i_c_.data());

  // object point
  for (size_t i = 0; i < track_ids.size(); ++i) {
    cost_function->AddParameterBlock(4);
    vec.emplace_back(
        image_data_.MutableTrack(track_ids[i])->MutablePoint()->data());
    tracks_in_problem_.insert(track_ids[i]);
  }

  cost_function->SetNumResiduals(track_ids.size() * 2);

  ceres::LossFunction* loss_function = new ceres::HuberLoss(robust_loss_width);
  problem_.AddResidualBlock(cost_function, loss_function, vec);

  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::AddRSCameraMeasurement(
    const theia::View* view, const double robust_loss_width) {
  const int64_t image_obs_time_ns = view->GetTimestamp() * S_TO_NS;
  const auto track_ids = view->TrackIds();

  double u_r3 = 0.0, u_so3 = 0.0;
  int64_t s_r3 = 0, s_so3 = 0;
  if (!CalcR3Times(image_obs_time_ns, u_r3, s_r3)) {
    LOG(INFO) << "Wrong time observation r3 vision measurements. time_ns: "
              << image_obs_time_ns << " u_r3: " << u_r3 << " s_r3:" << s_r3;
    return false;
  }
  if (!CalcSO3Times(image_obs_time_ns, u_so3, s_so3)) {
    LOG(INFO) << "Wrong time reference so3 vision measurements. time_ns: "
              << image_obs_time_ns << " u_r3: " << u_so3 << " s_r3:" << s_so3;
    return false;
  }

  using FunctorT = RSReprojectionCostFunctorSplit<N_>;
  FunctorT* functor = new FunctorT(
      view, &image_data_, u_so3, u_r3, inv_so3_dt_, inv_r3_dt_, track_ids);

  ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
      new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

  std::vector<double*> vec;
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(4);
    const int t = s_so3 + i;
    vec.emplace_back(so3_knots_[t].data());
    so3_knot_in_problem_[t] = true;
  }
  for (int i = 0; i < N_; i++) {
    cost_function->AddParameterBlock(3);
    const int t = s_r3 + i;
    vec.emplace_back(r3_knots_[t].data());
    r3_knot_in_problem_[t] = true;
  }

  // camera to imu transformation
  cost_function->AddParameterBlock(7);
  vec.emplace_back(T_i_c_.data());

  // line delay for rolling shutter cameras
  cost_function->AddParameterBlock(1);
  vec.emplace_back(&cam_line_delay_s_);

  // object point
  for (size_t i = 0; i < track_ids.size(); ++i) {
    cost_function->AddParameterBlock(4);
    vec.emplace_back(
        image_data_.MutableTrack(track_ids[i])->MutablePoint()->data());
    tracks_in_problem_.insert(track_ids[i]);
  }

  cost_function->SetNumResiduals(track_ids.size() * 2);

  if (robust_loss_width == 0.0) {
    problem_.AddResidualBlock(cost_function, NULL, vec);
  } else {
    ceres::LossFunction* loss_function =
        new ceres::HuberLoss(robust_loss_width);
    problem_.AddResidualBlock(cost_function, loss_function, vec);
  }

  // bound translation
  //  problem_.SetParameterLowerBound(T_i_c_.data(), 4, -1e-2);
  //  problem_.SetParameterUpperBound(T_i_c_.data(), 4, 1e-2);
  //  problem_.SetParameterLowerBound(T_i_c_.data(), 5, -10e-2);
  //  problem_.SetParameterUpperBound(T_i_c_.data(), 5, 10e-2);
  //  problem_.SetParameterLowerBound(T_i_c_.data(), 6, -1e-2);
  //  problem_.SetParameterUpperBound(T_i_c_.data(), 6, 1e-2);

  return true;
}

// template <int _T>
// bool SplineTrajectoryEstimator<_T>::AddRSInvCameraMeasurement(
//    const theia::View *view, const double robust_loss_width) {
//  std::vector<theia::TrackId> tracks = view->TrackIds();
//  const size_t nr_obs = tracks.size();
//  if (nr_obs <= 0) {
//    return false;
//  }

//  for (size_t i = 0; i < nr_obs; ++i) {

//    const int64_t image_obs_time_ns = view->GetTimestamp() * S_TO_NS;
//    const auto ref_view_id = image_data_.Track(tracks[i])->ReferenceViewId();
//    const int64_t image_ref_time_ns =
//        image_data_.View(ref_view_id)->GetTimestamp() * S_TO_NS;

//    double u_r3_obs, u_so3_obs;
//    int64_t s_r3_obs, s_so3_obs;
//    if (!CalcR3Times(image_obs_time_ns, u_r3_obs, s_r3_obs)) {
//      LOG(INFO) << "Wrong time observation r3 vision measurements. time_ns: "
//                << image_obs_time_ns << " u_r3: " << u_r3_obs
//                << " s_r3:" << s_r3_obs;
//      return false;
//    }
//    if (!CalcSO3Times(image_obs_time_ns, u_so3_obs, s_so3_obs)) {
//      LOG(INFO) << "Wrong time reference so3 vision measurements. time_ns: "
//                << image_obs_time_ns << " u_r3: " << u_so3_obs
//                << " s_r3:" << s_so3_obs;
//      return false;
//    }

//    double u_r3_ref, u_so3_ref;
//    int64_t s_r3_ref, s_so3_ref;
//    if (!CalcR3Times(image_ref_time_ns, u_r3_ref, s_r3_ref)) {
//      LOG(INFO) << "Wrong time reference r3 vision measurements. time_ns: "
//                << image_ref_time_ns << " u_r3: " << u_r3_ref
//                << " s_r3:" << s_r3_ref;
//      return false;
//    }
//    if (!CalcSO3Times(image_ref_time_ns, u_so3_ref, s_so3_ref)) {
//      LOG(INFO) << "Wrong time reference so3 vision measurements time_ns: "
//                << image_ref_time_ns << " u_r3: " << u_so3_ref
//                << " s_r3:" << s_so3_ref;
//      return false;
//    }

//    // maps will be nicely sorted after time
//    std::map<int, double *> time_to_so3_knots_in_prob;
//    std::map<int, double *> time_to_r3_knots_in_prob;

//    std::vector<double *> vec;
//    // spline data for observation
//    for (int i = 0; i < N_; i++) {
//      const int t = s_so3_obs + i;
//      time_to_so3_knots_in_prob.insert(std::make_pair(t,
//      so3_knots_[t].data()));
//    }

//    // spline data for reference
//    for (int i = 0; i < N_; i++) {
//      const int t = s_so3_ref + i;
//      if (time_to_so3_knots_in_prob.find(t) !=
//      time_to_so3_knots_in_prob.end())
//        continue;
//      time_to_so3_knots_in_prob.insert(std::make_pair(t,
//      so3_knots_[t].data()));
//    }

//    // now fill the vector
//    for (const auto &k : time_to_so3_knots_in_prob) {
//      vec.emplace_back(k.second);
//      so3_knot_in_problem_[k.first] = true;
//    }

//    // get pointer offsets
//    const int obs_so3_offset = GetPtrOffset(so3_knots_[s_so3_obs].data(),
//    vec); const int ref_so3_offset =
//    GetPtrOffset(so3_knots_[s_so3_ref].data(), vec);

//    // spline data for observation
//    for (int i = 0; i < N_; i++) {
//      const int t = s_r3_obs + i;
//      time_to_r3_knots_in_prob.insert(std::make_pair(t, r3_knots_[t].data()));
//    }

//    // spline data for reference
//    for (int i = 0; i < N_; i++) {
//      const int t = s_r3_ref + i;
//      if (time_to_r3_knots_in_prob.find(t) != time_to_r3_knots_in_prob.end())
//        continue;
//      time_to_r3_knots_in_prob.insert(std::make_pair(t, r3_knots_[t].data()));
//    }

//    // now fill the vector
//    for (const auto &k : time_to_r3_knots_in_prob) {
//      vec.emplace_back(k.second);
//      r3_knot_in_problem_[k.first] = true;
//    }

//    // get pointer offsets
//    const int obs_r3_offset = GetPtrOffset(r3_knots_[s_r3_obs].data(), vec);
//    const int ref_r3_offset = GetPtrOffset(r3_knots_[s_r3_ref].data(), vec);

//    std::vector<int> ptr_offsets;
//    ptr_offsets.push_back(obs_so3_offset);
//    ptr_offsets.push_back(ref_so3_offset);
//    ptr_offsets.push_back(obs_r3_offset);
//    ptr_offsets.push_back(ref_r3_offset);

//    // object point
//    ptr_offsets.push_back(vec.size());
//    vec.emplace_back(
//        image_data_.MutableTrack(tracks[i])->MutableInverseDepth());

//    tracks_in_problem_.insert(tracks[i]);

//    // add offset for inverse depth which comes last
//    using FunctorT = RSInvDepthReprojCostFunctorSplit<N_>;
//    FunctorT *functor = new FunctorT(view, &image_data_, T_i_c_, tracks[i],
//                                     u_so3_obs, u_r3_obs, u_so3_ref, u_r3_ref,
//                                     inv_so3_dt_, inv_r3_dt_,
//                                     ptr_offsets, 1.0);

//    ceres::DynamicAutoDiffCostFunction<FunctorT> *cost_function =
//        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

//    for (auto i = 0; i < time_to_so3_knots_in_prob.size(); ++i) {
//      cost_function->AddParameterBlock(4);
//    }
//    for (auto i = 0; i < time_to_r3_knots_in_prob.size(); ++i) {
//      cost_function->AddParameterBlock(3);
//    }

//    // add inverse depth
//    cost_function->AddParameterBlock(1);

//    cost_function->SetNumResiduals(2);

//    ceres::LossFunction *loss_function =
//        new ceres::HuberLoss(robust_loss_width);
//    problem_.AddResidualBlock(cost_function, loss_function, vec);
//    problem_.SetParameterLowerBound(
//        image_data_.MutableTrack(tracks[i])->MutableInverseDepth(), 0,
//        1e-10); // always positive depth
//  }
//  return true;
//}

template <int _T>
bool SplineTrajectoryEstimator<_T>::CalcTimes(const int64_t sensor_time,
                                              double& u,
                                              int64_t& s,
                                              int64_t dt_ns,
                                              size_t nr_knots) {
  const int64_t st_ns = (sensor_time - start_t_ns_);

  if (st_ns < 0.0) {
    u = 0.0;
    return false;
  }

  s = st_ns / dt_ns;
  if (s < 0) {
    return false;
  }

  if (size_t(s + N_) > nr_knots) {
    return false;
  }

  u = double(st_ns % dt_ns) / double(dt_ns);
  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::CalcSO3Times(const int64_t sensor_time,
                                                 double& u_so3,
                                                 int64_t& s_so3) {
  return CalcTimes(sensor_time, u_so3, s_so3, dt_so3_ns_, so3_knots_.size());
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::CalcR3Times(const int64_t sensor_time,
                                                double& u_r3,
                                                int64_t& s_r3) {
  return CalcTimes(sensor_time, u_r3, s_r3, dt_r3_ns_, r3_knots_.size());
}

template <int _T>
Sophus::SE3d SplineTrajectoryEstimator<_T>::GetKnot(int i) const {
  return Sophus::SE3d(so3_knots_[i], r3_knots_[i]);
}

template <int _T>
size_t SplineTrajectoryEstimator<_T>::GetNumSO3Knots() const {
  return so3_knots_.size();
}

template <int _T>
size_t SplineTrajectoryEstimator<_T>::GetNumR3Knots() const {
  return r3_knots_.size();
}

template <int _T>
int64_t SplineTrajectoryEstimator<_T>::GetMaxTimeNs() const {
  return start_t_ns_ + (so3_knots_.size() - N_ + 1) * dt_so3_ns_ - 1;
}

template <int _T>
int64_t SplineTrajectoryEstimator<_T>::GetMinTimeNs() const {
  return start_t_ns_;
}

template <int _T>
Eigen::Vector3d SplineTrajectoryEstimator<_T>::GetGyroBias() const {
  return gyro_bias_;
}

template <int _T>
Eigen::Vector3d SplineTrajectoryEstimator<_T>::GetAccelBias() const {
  return accl_bias_;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetImageData(
    const theia::Reconstruction& c) {
  image_data_ = c;
  // calculate all reference bearings
  //  const auto track_ids = image_data_.TrackIds();
  //  for (auto t = 0; t < track_ids.size(); ++t) {
  //    theia::Track *mut_track = image_data_.MutableTrack(track_ids[t]);
  //    theia::ViewId ref_view_id = mut_track->ReferenceViewId();
  //    const theia::View *v = image_data_.View(ref_view_id);
  //    const Eigen::Vector2d feat = (*v->GetFeature(track_ids[t])).point_;
  //    Eigen::Vector3d bearing =
  //    v->Camera().PixelToNormalizedCoordinates(feat); Eigen::Vector3d
  //    adjusted_point =
  //        mut_track->Point().head<3>() -
  //        mut_track->Point()[3] * v->Camera().GetPosition();
  //    Eigen::Vector3d rotated_point =
  //        v->Camera().GetOrientationAsRotationMatrix() * adjusted_point;
  ////    if (std::abs(rotated_point[2]) < 1e-10)
  ////        *mut_track->MutableInverseDepth() = 1 / 1e-10;
  ////    else
  ////        *mut_track->MutableInverseDepth() = 1 / rotated_point[2];
  //   *mut_track->MutableInverseDepth() = 1 / 0.5;
  //    mut_track->SetReferenceBearingVector(bearing);
  //  }
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetG(const Eigen::Vector3d& g) {
  gravity_ = g;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetT_i_c(const Sophus::SE3<double>& T) {
  T_i_c_ = T;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetImuToCameraTimeOffset(
    const double imu_to_camera_time_offset_s) {
  imu_to_camera_time_offset_s_ = imu_to_camera_time_offset_s;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetCameraLineDelay(
    const double cam_line_delay_s) {
  cam_line_delay_s_ = cam_line_delay_s;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetPosition(const int64_t& time_ns,
                                                Eigen::Vector3d& position) {
  double u_r3;
  int64_t s_r3;
  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
    return false;
  }

  std::vector<const double*> vec;
  for (int i = 0; i < N_; ++i) {
    vec.emplace_back(r3_knots_[s_r3 + i].data());
  }

  CeresSplineHelper<double, N_>::template evaluate<3, 0>(
      &vec[0], u_r3, inv_r3_dt_, &position);

  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetPose(const int64_t& time_ns,
                                            Sophus::SE3d& pose) {
  double u_r3, u_so3;
  int64_t s_r3, s_so3;
  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
    return false;
  }
  if (!CalcSO3Times(time_ns, u_so3, s_so3)) {
    return false;
  }

  Sophus::SO3d rot;
  Eigen::Vector3d trans;
  {
    std::vector<const double*> vec;
    for (int i = 0; i < N_; ++i) {
      vec.emplace_back(so3_knots_[s_so3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
        &vec[0], u_so3, inv_so3_dt_, &rot);
  }
  {
    std::vector<const double*> vec;
    for (int i = 0; i < N_; ++i) {
      vec.emplace_back(r3_knots_[s_r3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate<3, 0>(
        &vec[0], u_r3, inv_r3_dt_, &trans);
  }
  pose = Sophus::SE3d(rot, trans);

  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetAngularVelocity(
    const int64_t& time_ns, Eigen::Vector3d& velocity) {
  double u_r3, u_so3;
  int64_t s_r3, s_so3;
  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
    return false;
  }
  if (!CalcSO3Times(time_ns, u_so3, s_so3)) {
    return false;
  }

  std::vector<const double*> vec;
  for (int i = 0; i < N_; ++i) {
    vec.emplace_back(so3_knots_[s_so3 + i].data());
  }

  CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
      &vec[0], u_so3, inv_so3_dt_, nullptr, &velocity);

  return true;
}

template <int _T>
bool SplineTrajectoryEstimator<_T>::GetAcceleration(
    const int64_t& time_ns, Eigen::Vector3d& acceleration) {
  double u_r3, u_so3;
  int64_t s_r3, s_so3;
  if (!CalcR3Times(time_ns, u_r3, s_r3)) {
    return false;
  }
  if (!CalcSO3Times(time_ns, u_so3, s_so3)) {
    return false;
  }

  Sophus::SO3d rot;
  Eigen::Vector3d trans_accel_world;
  {
    std::vector<const double*> vec;
    for (int i = 0; i < N_; ++i) {
      vec.emplace_back(so3_knots_[s_so3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate_lie<Sophus::SO3>(
        &vec[0], u_so3, inv_so3_dt_, &rot);
  }
  {
    std::vector<const double*> vec;
    for (int i = 0; i < N_; ++i) {
      vec.emplace_back(r3_knots_[s_r3 + i].data());
    }

    CeresSplineHelper<double, N_>::template evaluate<3, 2>(
        &vec[0], u_r3, inv_r3_dt_, &trans_accel_world);
  }
  acceleration = rot.inverse() * (trans_accel_world + gravity_);

  return true;
}

template <int _T>
double SplineTrajectoryEstimator<_T>::GetMeanReprojectionError() {
  // ConvertInvDepthPointsToHom();
  double sum_error = 0.0;
  int num_points = 0;
  for (const auto vid : image_data_.ViewIds()) {
    const auto* view = image_data_.View(vid);
    std::vector<theia::TrackId> tracks = view->TrackIds();
    const size_t nr_obs = tracks.size();
    if (nr_obs <= 0) {
      return false;
    }

    const int64_t image_time_ns = view->GetTimestamp() * S_TO_NS;

    double u_r3, u_so3;
    int64_t s_r3, s_so3;
    if (!CalcR3Times(image_time_ns, u_r3, s_r3)) {
      return 0.0;
    }
    if (!CalcSO3Times(image_time_ns, u_so3, s_so3)) {
      return 0.0;
    }

    using FunctorT = RSReprojectionCostFunctorSplit<N_>;
    FunctorT* functor = new FunctorT(
        view, &image_data_, u_so3, u_r3, inv_so3_dt_, inv_r3_dt_, tracks);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    std::vector<double*> vec;
    for (int i = 0; i < N_; i++) {
      cost_function->AddParameterBlock(4);
      const int t = s_so3 + i;
      vec.emplace_back(so3_knots_[t].data());
    }
    for (int i = 0; i < N_; i++) {
      cost_function->AddParameterBlock(3);
      const int t = s_r3 + i;
      vec.emplace_back(r3_knots_[t].data());
    }

    // camera to imu transformation
    cost_function->AddParameterBlock(7);
    vec.emplace_back(T_i_c_.data());

    // line delay for rolling shutter cameras
    cost_function->AddParameterBlock(1);
    vec.emplace_back(&cam_line_delay_s_);

    // all object points
    for (size_t i = 0; i < nr_obs; ++i) {
      cost_function->AddParameterBlock(4);
      vec.emplace_back(
          image_data_.MutableTrack(tracks[i])->MutablePoint()->data());
    }

    cost_function->SetNumResiduals(2 * nr_obs);
    {
      Eigen::VectorXd residual;
      residual.setZero(nr_obs * 2);

      cost_function->Evaluate(&vec[0], residual.data(), NULL);

      for (size_t i = 0; i < nr_obs; i++) {
        Eigen::Vector2d res_point = residual.segment<2>(2 * i);
        if (res_point[0] != 0.0 && res_point[1] != 0.0) {
          sum_error += res_point.norm();
          num_points += 1;
        }
      }
    }
  }

  std::cout << "Mean reprojection error " << sum_error / num_points
            << " number residuals: " << num_points << std::endl;

  return sum_error / num_points;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::ConvertInvDepthPointsToHom() {
  const auto track_ids = image_data_.TrackIds();
  for (size_t p = 0; p < track_ids.size(); ++p) {
    theia::Track* mutable_track = image_data_.MutableTrack(track_ids[p]);
    const theia::View* v = image_data_.View(mutable_track->ReferenceViewId());
    Eigen::Vector3d bearing =
        v->Camera().PixelToUnitDepthRay((*v->GetFeature(track_ids[p])).point_);

    const int64_t ts = v->GetTimestamp() * S_TO_NS;
    Sophus::SE3d T_w_i;
    GetPose(ts, T_w_i);
    Eigen::Vector3d X_ref =
        T_i_c_.so3() *
        (bearing - mutable_track->InverseDepth() * T_i_c_.translation());
    // 2. Transform point from IMU to world frame
    Eigen::Vector3d X = T_w_i.so3() * X_ref +
                        T_w_i.translation() * mutable_track->InverseDepth();
    *mutable_track->MutablePoint() = X.homogeneous();
    mutable_track->SetEstimated(true);
  }
}

template <int _T>
void SplineTrajectoryEstimator<_T>::ConvertToTheiaRecon(
    theia::Reconstruction* recon_out) {
  // read camera calibration
  std::vector<theia::ViewId> view_ids = image_data_.ViewIds();
  for (size_t i = 0; i < view_ids.size(); ++i) {
    const int64_t t_ns =
        image_data_.View(view_ids[i])->GetTimestamp() * S_TO_NS;
    Sophus::SE3d T_w_i;
    GetPose(t_ns, T_w_i);
    Sophus::SE3d T_w_c = T_w_i * T_i_c_;
    theia::ViewId v_id_theia =
        recon_out->AddView(std::to_string(t_ns), 0, t_ns);
    theia::View* view = recon_out->MutableView(v_id_theia);
    view->SetEstimated(true);
    theia::Camera* camera_ptr = view->MutableCamera();
    camera_ptr->SetOrientationFromRotationMatrix(
        T_w_c.rotationMatrix().transpose());
    camera_ptr->SetPosition(T_w_c.translation());
  }
  // ConvertInvDepthPointsToHom();
  const auto track_ids = image_data_.TrackIds();
  for (size_t p = 0; p < track_ids.size(); ++p) {
    TrackId tid = recon_out->AddTrack();
    *recon_out->MutableTrack(tid)->MutablePoint() =
        image_data_.Track(track_ids[p])->Point();
    recon_out->MutableTrack(tid)->SetEstimated(true);
  }
}

template <int _T>
Eigen::Vector3d SplineTrajectoryEstimator<_T>::GetGravity() const {
  return gravity_;
}

template <int _T>
Sophus::SE3d SplineTrajectoryEstimator<_T>::GetT_i_c() const {
  return T_i_c_;
}

template <int _T>
double SplineTrajectoryEstimator<_T>::GetRSLineDelay() const {
  return cam_line_delay_s_;
}

template <int _T>
ThreeAxisSensorCalibParams<double>
SplineTrajectoryEstimator<_T>::GetAcclIntrinsics() const {
  ThreeAxisSensorCalibParams<double> accel_calib_triad(accl_intrinsics_[0],
                                                       accl_intrinsics_[1],
                                                       accl_intrinsics_[2],
                                                       0,
                                                       0,
                                                       0,
                                                       accl_intrinsics_[3],
                                                       accl_intrinsics_[4],
                                                       accl_intrinsics_[5],
                                                       accl_bias_[0],
                                                       accl_bias_[1],
                                                       accl_bias_[2]);
  return accel_calib_triad;
}

template <int _T>
ThreeAxisSensorCalibParams<double>
SplineTrajectoryEstimator<_T>::GetGyroIntrinsics() const {
  ThreeAxisSensorCalibParams<double> gyro_calib_triad(gyro_intrinsics_[0],
                                                      gyro_intrinsics_[1],
                                                      gyro_intrinsics_[2],
                                                      gyro_intrinsics_[3],
                                                      gyro_intrinsics_[4],
                                                      gyro_intrinsics_[5],
                                                      gyro_intrinsics_[6],
                                                      gyro_intrinsics_[7],
                                                      gyro_intrinsics_[8],
                                                      gyro_bias_[0],
                                                      gyro_bias_[1],
                                                      gyro_bias_[2]);
  return gyro_calib_triad;
}

template <int _T>
void SplineTrajectoryEstimator<_T>::SetIMUIntrinsics(
    const ThreeAxisSensorCalibParams<double>& accl_intrinsics,
    const ThreeAxisSensorCalibParams<double>& gyro_intrinsics) {
  accl_intrinsics_ << accl_intrinsics.misYZ(), accl_intrinsics.misZY(),
      accl_intrinsics.misZX(), accl_intrinsics.scaleX(),
      accl_intrinsics.scaleY(), accl_intrinsics.scaleZ();

  gyro_intrinsics_ << gyro_intrinsics.misYZ(), gyro_intrinsics.misZY(),
      gyro_intrinsics.misZX(), gyro_intrinsics.misXZ(), gyro_intrinsics.misXY(),
      gyro_intrinsics.misYX(), gyro_intrinsics.scaleX(),
      gyro_intrinsics.scaleY(), gyro_intrinsics.scaleZ();

  accl_bias_ << accl_intrinsics.biasX(), accl_intrinsics.biasY(),
      accl_intrinsics.biasZ();
  gyro_bias_ << gyro_intrinsics.biasX(), gyro_intrinsics.biasY(),
      gyro_intrinsics.biasZ();
}
}  // namespace core
}  // namespace OpenICC
