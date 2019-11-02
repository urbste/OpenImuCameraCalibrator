#ifndef KONTIKIV2_TRAJECTORY_H
#define KONTIKIV2_TRAJECTORY_H

#include <memory>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "OpenCameraCalibrator/spline/entity.h"

#include "OpenCameraCalibrator/spline/types.h"

namespace kontiki {
namespace trajectories {

enum EvaluationFlags {
  EvalPosition = 1,
  EvalVelocity = 2,
  EvalAcceleration = 4,
  EvalOrientation = 8,
  EvalAngularVelocity = 16
};

template<typename T>
struct TrajectoryEvaluation {
  TrajectoryEvaluation(int flags) :
      needs(flags) { };

  using Vector3 = Eigen::Matrix<T, 3, 1>;
  Vector3 position; // Position in world coordinates
  Vector3 velocity; // Velocity relative to world coordinates
  Vector3 acceleration; // Acceleration relative to world coordinates
  Eigen::Quaternion<T> orientation; // Orientation in world coordinates. x_world = orientation * x_body
  Vector3 angular_velocity; // Angular velocity in world coordinate frame

  // Struct to simplify lookup of what needs to be computed
  struct Needs {
    Needs(int flags) : flags(flags) { };

    bool Position() const {
      return flags & EvalPosition;
    }

    bool Velocity() const {
      return flags & EvalVelocity;
    }

    bool Acceleration() const {
      return flags & EvalAcceleration;
    }

    bool Orientation() const {
      return flags & EvalOrientation;
    }

    bool AngularVelocity() const {
      return flags & EvalAngularVelocity;
    }

    bool AnyLinear() const {
      return FlagsLinear();
    }

    bool AnyRotation() const {
      return FlagsRotation();
    }

    int FlagsLinear() const {
      return flags & (EvalPosition | EvalVelocity | EvalAcceleration);
    }

    int FlagsRotation() const {
      return flags & (EvalOrientation | EvalAngularVelocity);
    }

   protected:
    int flags;
  } needs;
};

// Base class for trajectory views
// This is used to collect utility functions common to all views (Position, ...)
// Views are intended to be immutable and uses readonly DataHolderBase parameter stores.
template<typename T, class MetaType>
class TrajectoryView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Quaternion = Eigen::Quaternion<T>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  // Import constructors
  using entity::EntityView<T, MetaType>::EntityView;

  virtual Result Evaluate(T t, int flags) const = 0;
  virtual double MinTime() const = 0;
  virtual double MaxTime() const = 0;

  Vector3 Position(T t) const {
    return this->Evaluate(t, EvalPosition)->position;
  }

  Vector3 Velocity(T t) const {
    return this->Evaluate(t, EvalVelocity)->velocity;
  }

  Vector3 Acceleration(T t) const {
    return this->Evaluate(t, EvalAcceleration)->acceleration;
  }

  Quaternion Orientation(T t) const {
    return this->Evaluate(t, EvalOrientation)->orientation;
  }

  Vector3 AngularVelocity(T t) const {
    return this->Evaluate(t, EvalAngularVelocity)->angular_velocity;
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, T t) {
    Result result = this->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation.conjugate() * (Xw - result->position);
  }

  // Move point from trajectory to world coordinate frame
  Vector3 ToWorld(Vector3 &Xt, T t) {
    Result result = this->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation * Xt + result->position;
  }

  std::pair<double, double> ValidTime() const {
    return std::make_pair(this->MinTime(), this->MaxTime());
  };
};


template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class TrajectoryEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
  using Base = type::Entity<ViewTemplate, MetaType, StoreType>;
 public:
  // Import constructor
//  using type::Entity<ViewTemplate, MetaType, StoreType>::Entity;

  TrajectoryEntity() :
    locked_(false) {
    // Nothing else
  }

  virtual bool IsLocked() const {
    return locked_;
  }

  virtual void Lock(bool lock) {
    locked_ = lock;
  }

 protected:
  bool locked_; // Locked: Trajectory not updated by optimizer
};

} // namespace trajectories

namespace type {
template<typename _Entity, typename T>
using Trajectory = typename entity::type::base::ForView<_Entity, trajectories::TrajectoryView, T>;
} // namespace type

} // namespace kontiki

#endif //KONTIKIV2_TRAJECTORY_H
