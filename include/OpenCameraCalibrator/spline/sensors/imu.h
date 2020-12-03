#ifndef KONTIKIV2_IMU_H
#define KONTIKIV2_IMU_H

#include <Eigen/Dense>

#include "OpenCameraCalibrator/spline/sensors/sensors.h"
#include "OpenCameraCalibrator/spline/entity.h"
#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/types.h"
#include "OpenCameraCalibrator/spline/constants.h"
#include "OpenCameraCalibrator/spline/paramstore/empty_pstore.h"
#include "OpenCameraCalibrator/spline/paramstore/dynamic_pstore.h"

namespace kontiki {
namespace sensors {

namespace internal {

static const double STANDARD_GRAVITY = 9.80665;

// Base Imu view using CRTP to access the correct Gyroscope()/Accelerometer() methods
// All IMU views must inherit from this one directly, and not through subclasses.
template<typename T, typename MetaType, typename Derived>
class ImuView : public SensorView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Flags = trajectories::EvaluationFlags;
 public:
  // Import constructor
  using SensorView<T, MetaType>::SensorView;

//  static const Vector3 GRAVITY;

  // Accelerometer measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Accelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Accelerometer<TrajectoryModel>(trajectory, t);
  }

  // Gyroscope measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Gyroscope<TrajectoryModel>(trajectory, t);
  }

 protected:
  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardGyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t + this->time_offset(), Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }

  // Standard accelerometer function
  template<typename TrajectoryModel>
  Vector3 StandardAccelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t + this->time_offset(), Flags::EvalOrientation | Flags::EvalAcceleration);
    return result->orientation.conjugate() * (result->acceleration + Constants<T>::Gravity);
  }

};

//template<typename T, typename MetaType, typename Derived>
//const Eigen::Matrix<T, 3, 1> ImuView<T, MetaType, Derived>::GRAVITY = Eigen::Matrix<T, 3, 1>(T(0), T(0), T(-STANDARD_GRAVITY));


template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ImuEntity : public SensorEntity<ViewTemplate, MetaType, StoreType> {
  using Vector3 = Eigen::Vector3d;
 public:
  // Inherit constructors
  using SensorEntity<ViewTemplate, MetaType, StoreType>::SensorEntity;
};

} // namespace detail
} // namespace sensors

// Type specifier to get an Imu instance
namespace type {
template<typename E, typename T>
using Imu = typename sensors::internal::ImuView<T,
                                                typename E::Meta,
                                                typename E::template View<T, typename E::Meta>>;
}

} // namespace kontiki

#endif //KONTIKIV2_IMU_H
