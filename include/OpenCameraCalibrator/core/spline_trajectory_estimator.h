#pragma once

#include "ceres/ceres.h"
#include "theia/sfm/reconstruction.h"

#include "OpenCameraCalibrator/basalt_spline/ceres_calib_split_residuals.h"
#include "OpenCameraCalibrator/basalt_spline/ceres_local_param.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include <iostream>
#include <thread>

namespace OpenICC {
namespace core {

enum SplineOptimFlags {
  POINTS = 1 << 0,
  T_I_C = 1 << 1,
  IMU_BIASES = 1 << 2,
  IMU_INTRINSICS = 1 << 3,
  GRAVITY_DIR = 1 << 4,
  CAM_LINE_DELAY = 1 << 5,
  SPLINE = 1 << 6,
  ACC_BIAS = 1 << 7,
  GYR_BIAS = 1 << 8
};

const double GRAVITY_MAGN = 9.81;

template <int _N>
class SplineTrajectoryEstimator {
 public:
  static constexpr int N_ = _N;        // Order of the spline.
  static constexpr int DEG_ = _N - 1;  // Degree of the spline.

  SplineTrajectoryEstimator();

  SplineTrajectoryEstimator(int64_t time_interval_so3_ns,
                            int64_t time_interval_r3_ns,
                            int64_t start_time_ns);

  void SetTimes(int64_t time_interval_so3_ns,
                int64_t time_interval_r3_ns,
                int64_t start_time_ns,
                int64_t end_time_ns);

  void InitSpline(const int flags, const double end_time_s = 0.0);

  void InitBiasSplines(const Eigen::Vector3d& accl_init_bias,
                       const Eigen::Vector3d& gyr_init_bias,
                       int64_t dt_accl_bias_ns = 500000000,
                       int64_t dt_gyro_bias_ns = 500000000,
                       const double max_accl_range = 1.0,
                       const double max_gyro_range = 1e-2);

  void BatchInitSO3R3VisPoses();

  void InitScenePoints();

  void SetFixedParams(const int flags);

  ceres::Solver::Summary Optimize(const int max_iters, const int flags);

  // keep the rest constant and only optimize a window
  ceres::Solver::Summary Optimize(const int max_iters,
                                  const int flags,
                                  const int64_t start_time,
                                  const int64_t end_time);

  bool AddGPSMeasurement(const Eigen::Vector3d& meas,
                         const int64_t time_ns,
                         const double weight_gps);

  bool AddAccelerometerMeasurement(const Eigen::Vector3d& meas,
                                   const int64_t time_ns,
                                   const double weight_se3);

  bool AddGyroscopeMeasurement(const Eigen::Vector3d& meas,
                               const int64_t time_ns,
                               const double weight_se3);

  bool AddGSCameraMeasurement(const theia::View* view,
                              const double robust_loss_width);
  bool AddRSCameraMeasurement(const theia::View* view,
                              const double robust_loss_width = 0.0);
  bool AddGSInvCameraMeasurement(const theia::View* view,
                                 const double robust_loss_width);
  bool AddRSInvCameraMeasurement(const theia::View* view,
                                 const double robust_loss_width);

  // setter
  void SetImageData(const theia::Reconstruction& c);

  void SetGravity(const Eigen::Vector3d& g);

  void SetT_i_c(const Sophus::SE3<double>& T);

  void SetTelemetryData(const CameraTelemetryData& telemetry_data);

  void SetImuToCameraTimeOffset(const double imu_to_camera_time_offset_s);

  void SetCameraLineDelay(const double cam_line_delay_s);

  void SetIMUIntrinsics(
      const ThreeAxisSensorCalibParams<double>& accl_intrinsics,
      const ThreeAxisSensorCalibParams<double>& gyro_intrinsics);

  // getter
  Sophus::SE3d GetKnot(int i) const;

  bool GetPose(const int64_t& time_ns, Sophus::SE3d& pose);

  bool GetPosition(const int64_t& time_ns, Eigen::Vector3d& position);

  bool GetAngularVelocity(const int64_t& time_ns, Eigen::Vector3d& velocity);

  bool GetVelocity(const int64_t& time_ns, Eigen::Vector3d& velocity);

  bool GetAcceleration(const int64_t& time_ns, Eigen::Vector3d& acceleration);

  size_t GetNumSO3Knots() const;

  size_t GetNumR3Knots() const;

  int64_t GetMaxTimeNs() const;

  int64_t GetMinTimeNs() const;

  Eigen::Vector3d GetGyroBias(const int64_t& time_ns);

  Eigen::Vector3d GetAcclBias(const int64_t& time_ns);

  double GetMeanRSReprojectionError();

  double GetMeanGSReprojectionError();

  Eigen::Vector3d GetGravity() const;

  Sophus::SE3d GetT_i_c() const;

  double GetRSLineDelay() const;

  ThreeAxisSensorCalibParams<double> GetAcclIntrinsics(const int64_t& time_ns);

  ThreeAxisSensorCalibParams<double> GetGyroIntrinsics(const int64_t& time_ns);

  void ConvertToTheiaRecon(theia::Reconstruction* recon_out);

  void ConvertInvDepthPointsToHom();

 private:
  bool CalcSO3Times(const int64_t sensor_time, double& u_so3, int64_t& s_so3);
  bool CalcR3Times(const int64_t sensor_time, double& u_r3, int64_t& s_r3);
  bool CalcTimes(const int64_t sensor_time,
                 double& u,
                 int64_t& s,
                 int64_t dt_ns,
                 size_t nr_knots,
                 const int N = N_);

  int64_t start_t_ns_;
  int64_t end_t_ns_;

  //! SO3 and R3 spline meta data
  int64_t dt_so3_ns_;
  int64_t dt_r3_ns_;

  double inv_r3_dt_;
  double inv_so3_dt_;

  size_t nr_knots_so3_;
  size_t nr_knots_r3_;

  so3_vector so3_knots_;
  vec3_vector r3_knots_;

  std::vector<bool> so3_knot_in_problem_;
  std::vector<bool> r3_knot_in_problem_;

  //! bias spline meta data
  size_t nr_knots_accl_bias_;
  size_t nr_knots_gyro_bias_;

  int64_t dt_accl_bias_ns_;
  int64_t dt_gyro_bias_ns_;

  double inv_accl_bias_dt_;
  double inv_gyro_bias_dt_;

  vec3_vector gyro_bias_spline_;
  vec3_vector accl_bias_spline_;

  double max_accl_bias_range_ = 1.0;
  double max_gyro_bias_range_ = 1e-2;

  //! parameters
  int optim_flags_;

  bool fix_imu_intrinsics_ = false;

  double cam_line_delay_s_ = 0.0;
  double cam_line_delay_offset_s_ = 0.0;
  double imu_to_camera_time_offset_s_ = 0.0;

  std::set<theia::TrackId> tracks_in_problem_;

  Eigen::Vector3d gravity_;

  Eigen::Matrix<double, 6, 1> accl_intrinsics_;
  Eigen::Matrix<double, 9, 1> gyro_intrinsics_;

  theia::Reconstruction image_data_;

  Sophus::SE3<double> T_i_c_;

  ceres::Problem problem_;

  bool spline_initialized_with_gps_ = false;
};

inline bool KnotInBlock(const std::vector<double*> vec, double* knot_ptr) {
  for (const auto& v : vec) {
    if (knot_ptr == v) {
      return true;
    }
  }
  return false;
}

inline int GetPtrOffset(const double* knot_ptr,
                        const std::vector<double*>& container) {
  for (size_t i = 0; i < container.size(); ++i) {
    if (knot_ptr == container[i]) {
      return i;
    }
  }
  // this should not happen!
  return 0;
}
}  // namespace core
}  // namespace OpenICC

#include "OpenCameraCalibrator/core/spline_trajectory_estimator.impl.h"
