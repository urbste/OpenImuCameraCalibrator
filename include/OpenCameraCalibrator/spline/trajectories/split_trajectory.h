#ifndef KONTIKIV2_SPLIT_TRAJECTORY_H
#define KONTIKIV2_SPLIT_TRAJECTORY_H

#include <memory>
#include "OpenCameraCalibrator/spline/types.h"
#include "OpenCameraCalibrator/spline/trajectories/uniform_r3_spline_trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/uniform_so3_spline_trajectory.h"

namespace kontiki {
namespace trajectories {
namespace internal {

struct SplitMeta : public entity::MetaData {
  using R3Meta = UniformR3SplineTrajectory::Meta;
  using SO3Meta = UniformSO3SplineTrajectory::Meta;

  R3Meta r3_meta;
  SO3Meta so3_meta;

  size_t NumParameters() const override {
    return r3_meta.NumParameters() + so3_meta.NumParameters();
  }
};


template<typename T, typename MetaType>
class SplitView : public TrajectoryView<T, MetaType> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Base = TrajectoryView<T, MetaType>;
  using R3View = type::View<UniformR3SplineTrajectory, T>;
  using SO3View = type::View<UniformSO3SplineTrajectory, T>;
 public:

  SplitView(const MetaType &meta, entity::ParameterStore<T> *holder) :
      Base(meta, holder),
      r3_view_(std::make_shared<R3View>(meta.r3_meta, holder->Slice(0, meta.r3_meta.NumParameters()))),
      so3_view_(std::make_shared<SO3View>(meta.so3_meta, holder->Slice(meta.r3_meta.NumParameters(), meta.so3_meta.NumParameters()))) {
    // Nothing more required
  }

  Result Evaluate(T t, int flags) const {
    Result result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    if (result->needs.AnyLinear()) {
      Result r3_result = r3_view_->Evaluate(t, result->needs.FlagsLinear());
      result->position = r3_result->position;
      result->velocity = r3_result->velocity;
      result->acceleration = r3_result->acceleration;
    }

    if (result->needs.AnyRotation()) {
      Result so3_result = so3_view_->Evaluate(t, result->needs.FlagsRotation());
      result->orientation = so3_result->orientation;
      result->angular_velocity = so3_result->angular_velocity;
    }

    return result;
  }

  double MinTime() const override {
    return std::max<double>(r3_view_->MinTime(), so3_view_->MinTime());
  }

  double MaxTime() const override {
    return std::min<double>(r3_view_->MaxTime(), so3_view_->MaxTime());
  }

  std::shared_ptr<R3View> r3_view_;
  std::shared_ptr<SO3View> so3_view_;
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class SplitEntity : public TrajectoryEntity<ViewTemplate, MetaType, StoreType> {
  using Base = TrajectoryEntity<ViewTemplate, MetaType, StoreType>;
 public:

  SplitEntity(std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory,
              std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory) :
    r3_trajectory_(r3_trajectory),
    so3_trajectory_(so3_trajectory) {

    // Bind view to concrete entities
    this->r3_view_ = r3_trajectory_;
    this->so3_view_ = so3_trajectory_;
  };

  SplitEntity(double r3_dt, double so3_dt, double r3_t0, double so3_t0) :
    SplitEntity(std::make_shared<UniformR3SplineTrajectory>(r3_dt, r3_t0),
                std::make_shared<UniformSO3SplineTrajectory>(so3_dt, so3_t0)) { }

  SplitEntity(double r3_dt, double so3_dt) :
    SplitEntity(r3_dt, so3_dt, 0.0, 0.0) { };

  SplitEntity() :
    SplitEntity(1.0, 1.0) { };

  SplitEntity(const SplitEntity &rhs) :
    SplitEntity(std::make_shared<UniformR3SplineTrajectory>(*rhs.r3_trajectory_),
                std::make_shared<UniformSO3SplineTrajectory>(*rhs.so3_trajectory_)) { };

  bool IsLocked() const override {
    bool r3_locked = this->r3_trajectory_->IsLocked();
    bool so3_locked = this->so3_trajectory_->IsLocked();

    if (r3_locked != so3_locked) {
      throw std::runtime_error("R3 and SO3 trajectories have different lock status!");
    }

    return r3_locked;
  }

  void Lock(bool lock) override {
    this->r3_trajectory_->Lock(lock);
    this->so3_trajectory_->Lock(lock);
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    r3_trajectory_->AddToProblem(problem, times, meta.r3_meta, parameters);
    so3_trajectory_->AddToProblem(problem, times, meta.so3_meta, parameters);
  }

  std::shared_ptr<UniformR3SplineTrajectory> R3Spline() const {
    return r3_trajectory_;
  }

  std::shared_ptr<UniformSO3SplineTrajectory> SO3Spline() const {
    return so3_trajectory_;
  }

 protected:
  std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory_;
  std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory_;
};

} // namespace internal

class SplitTrajectory : public internal::SplitEntity<internal::SplitView,
                                                     internal::SplitMeta,
                                                     entity::EmptyParameterStore<double>> {
 public:
  using internal::SplitEntity<internal::SplitView,
                              internal::SplitMeta,
                              entity::EmptyParameterStore<double>>::SplitEntity;
  static constexpr const char* CLASS_ID = "SplitTrajectory";
};

} // namespace trajectories
} // namespace kontiki

#endif //KONTIKIV2_SPLIT_TRAJECTORY_H
