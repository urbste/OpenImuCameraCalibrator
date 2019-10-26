//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_ORIENTATION_MEASUREMENT_H
#define KONTIKIV2_ORIENTATION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>

namespace kontiki {
namespace measurements {

class OrientationMeasurement {
  using Quaternion = Eigen::Quaterniond;
 public:
  OrientationMeasurement(double t, const Quaternion &q) : t(t), q(q) {};
  OrientationMeasurement(double t, const Eigen::Vector4d &qvec) : t(t),
                                                                  q(Eigen::Quaterniond(qvec(0), qvec(1), qvec(2), qvec(3))) {};

  template<typename TrajectoryModel, typename T>
  Eigen::Quaternion<T> Measure(const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    return trajectory.Orientation(T(t));
  };

  template<typename TrajectoryModel, typename T>
  T Error(const type::Trajectory<TrajectoryModel, T> &trajectory) const {
    Eigen::Quaternion<T> qhat = Measure<TrajectoryModel, T>(trajectory);
    return q.cast<T>().angularDistance(qhat);
  }

  // Measurement data
  double t;
  Quaternion q;

 protected:

  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const OrientationMeasurement &m) : measurement(m) {};

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      auto trajectory = entity::Map<TrajectoryModel, T>(params, meta);
      residual[0] = measurement.Error<TrajectoryModel, T>(trajectory);
      return true;
    }

    const OrientationMeasurement& measurement;
    typename TrajectoryModel::Meta meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;

    // Add trajectory to problem
    //estimator.trajectory()->AddToProblem(estimator.problem(), residual->meta, parameter_blocks, parameter_sizes);
    estimator.AddTrajectoryForTimes({{t,t}}, residual->meta, parameter_info);
    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(1);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(cost_function,
                                         nullptr,
                                         entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};
} // namespace measurements
} // namespace kontiki


#endif //KONTIKIV2_ORIENTATION_MEASUREMENT_H
