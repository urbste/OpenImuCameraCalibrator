#ifndef KONTIKIV2_GYROSCOPE_MEASUREMENT_H
#define KONTIKIV2_GYROSCOPE_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/sensors/imu.h"
#include "OpenCameraCalibrator/spline/trajectory_estimator.h"

namespace kontiki {
namespace measurements {

template<typename ImuModel>
class GyroscopeMeasurement {
  using Vector3 = Eigen::Vector3d;
 public:
  GyroscopeMeasurement(std::shared_ptr<ImuModel> imu, double t, const Vector3& w, double weight) :
    imu_(imu), t(t), w(w), weight(weight) { };

  GyroscopeMeasurement(std::shared_ptr<ImuModel> imu, double t, const Vector3& w) :
      GyroscopeMeasurement(imu, t, w, 1.0) { };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const type::Imu<ImuModel, T> &imu, const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    return imu.template Gyroscope<TrajectoryModel>(trajectory, T(t));
  };

  template<typename TrajectoryModel>
  Vector3 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Measure<TrajectoryModel, double>(*imu_, trajectory);
  };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const type::Imu<ImuModel, T> &imu, const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    return T(weight) * (w.cast<T>() - Measure<TrajectoryModel, T>(imu, trajectory));
  }

  template<typename TrajectoryModel>
  Vector3 Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return Error<TrajectoryModel, double>(*imu_, trajectory);
  }

  // Data
  std::shared_ptr<ImuModel> imu_;

  // Measurement data
  double t;  // Time
  Vector3 w; // Gyroscope angular velocity measurement (rad/s)
  double weight;

 protected:
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const GyroscopeMeasurement &m) : measurement(m) {};

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      const auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
      offset += trajectory_meta.NumParameters();
      const auto imu = entity::Map<ImuModel, T>(&params[offset], imu_meta);

      Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(imu, trajectory);
      return true;
    }

    const GyroscopeMeasurement& measurement;
    typename ImuModel::Meta imu_meta;
    typename TrajectoryModel::Meta trajectory_meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;

    // Add trajectory to problem
    double tmin, tmax;
    if (this->imu_->TimeOffsetIsLocked()) {
      tmin = t;
      tmax = t;
    }
    else {
      tmin = t - this->imu_->max_time_offset();
      tmax = t + this->imu_->max_time_offset();
    }
    estimator.AddTrajectoryForTimes({{tmin, tmax}}, residual->trajectory_meta, parameter_info);

    // Add IMU to problem
    imu_->AddToProblem(estimator.problem(), {{tmin, tmax}}, residual->imu_meta, parameter_info);

    // Let cost function know about the number and sizes of parameters dynamically added
    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(3);
    estimator.problem().AddResidualBlock(cost_function, nullptr, entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

} // namespace measurement
} // namespace kontiki

#endif //KONTIKIV2_GYROSCOPE_MEASUREMENT_H
