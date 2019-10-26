//
// Created by hannes on 2018-02-07.
//

#ifndef KONTIKIV2_SENSORS_H
#define KONTIKIV2_SENSORS_H

#include <Eigen/Dense>

#include <entity/entity.h>
#include <kontiki/types.h>

namespace kontiki {
namespace sensors {

namespace internal {

struct SensorMeta : public entity::MetaData {
  size_t NumParameters() const override {
    return 3; // Relative pose (2) + time offset (1)
  }

  double max_time_offset;
};

template<typename T, typename MetaType>
class SensorView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
 public:
  // Import constructor
  using entity::EntityView<T, MetaType>::EntityView;

  QuaternionMap relative_orientation() const {
    return QuaternionMap(this->pstore_->ParameterData(0));
  }

  void set_relative_orientation(const Quaternion &q) {
    QuaternionMap qmap(this->pstore_->ParameterData(0));
    qmap = q;
  }

  Vector3Map relative_position() const {
    return Vector3Map(this->pstore_->ParameterData(1));
  }

  void set_relative_position(const Vector3 &p) {
    Vector3Map pmap(this->pstore_->ParameterData(1));
    pmap = p;
  }

  T time_offset() const {
    auto ptr = this->pstore_->ParameterData(2);
    return *ptr;
  }

  void set_time_offset(T d) {
    if (std::abs(d) <= this->max_time_offset()) {
      auto ptr = this->pstore_->ParameterData(2);
      *ptr = d;
    }
    else {
      std::stringstream ss;
      ss << "Time offset |" << d << "| > " << this->max_time_offset();
      throw std::range_error(ss.str());
    }
  }

  double max_time_offset() const {
    return this->meta_.max_time_offset;
  }

  void set_max_time_offset(double max) {
    this->meta_.max_time_offset = max;
  }

  Vector3 FromTrajectory(const Vector3 &X_trajectory) const {
    return relative_orientation() * X_trajectory + relative_position();
  };

  Vector3 ToTrajectory(const Vector3 &X_camera) const {
    return relative_orientation().conjugate() * (X_camera - relative_position());
  };
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class SensorEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
 public:
  SensorEntity() :
      orientation_parameterization_(new ceres::EigenQuaternionParameterization()),
      relative_position_locked_(true),
      relative_orientation_locked_(true),
      time_offset_locked_(true) {
      // 0: Relative orientation
      this->pstore_->AddParameter(4, orientation_parameterization_.get());

      // 1: Relative position
      this->pstore_->AddParameter(3);

      // 2: Time offset
      this->pstore_->AddParameter(1);

    this->set_relative_position(Eigen::Vector3d::Zero());
    this->set_relative_orientation(Eigen::Quaterniond::Identity());
    this->set_max_time_offset(0.1);
    this->set_time_offset(0);
  };

  bool RelativeOrientationIsLocked() const {
    return relative_orientation_locked_;
  }

  void LockRelativeOrientation(bool flag) {
    relative_orientation_locked_ = flag;
  }

  bool RelativePositionIsLocked() const {
    return relative_position_locked_;
  }

  void LockRelativePosition(bool flag) {
    relative_position_locked_ = flag;
  }

  bool TimeOffsetIsLocked() const {
    return time_offset_locked_;
  }

  void LockTimeOffset(bool flag) {
    time_offset_locked_ = flag;
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    auto pi_qct = this->pstore_->Parameter(0);
    auto pi_pct = this->pstore_->Parameter(1);
    auto pi_offset = this->pstore_->Parameter(2);

    // Relative orientation q_ct
    problem.AddParameterBlock(pi_qct.data, pi_qct.size, pi_qct.parameterization);
    parameters.push_back(pi_qct);

    if (relative_orientation_locked_)
      problem.SetParameterBlockConstant(pi_qct.data);

    // Relative translation p_ct
    problem.AddParameterBlock(pi_pct.data, pi_pct.size, pi_pct.parameterization);
    parameters.push_back(pi_pct);

    if (relative_position_locked_)
      problem.SetParameterBlockConstant(pi_pct.data);

    // Time offset is constrained to (-d, d)
    problem.AddParameterBlock(pi_offset.data, pi_offset.size, pi_offset.parameterization);
    problem.SetParameterLowerBound(pi_offset.data, 0, -this->max_time_offset());
    problem.SetParameterUpperBound(pi_offset.data, 0, this->max_time_offset());
    parameters.push_back(pi_offset);

    if (time_offset_locked_)
      problem.SetParameterBlockConstant(pi_offset.data);
  }

 protected:
  bool relative_position_locked_;
  bool relative_orientation_locked_;
  bool time_offset_locked_;
  std::unique_ptr<ceres::EigenQuaternionParameterization> orientation_parameterization_;
};


} // namespace internal
} // namespace sensors

// Type specifier to get an Imu instance
namespace type {
template<typename E, typename T>
using Sensor = typename entity::type::base::ForView<E, sensors::internal::SensorView, T>;
}

} // namespace kontiki


#endif //KONTIKIV2_SENSORS_H
