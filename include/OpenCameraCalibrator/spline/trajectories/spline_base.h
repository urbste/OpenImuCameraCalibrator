//
// Created by hannes on 2018-01-15.
//

#ifndef KONTIKIV2_SPLINE_BASE_H
#define KONTIKIV2_SPLINE_BASE_H

#include <Eigen/Dense>

#include "OpenCameraCalibrator/spline/entity.h"
#include "OpenCameraCalibrator/spline/paramstore/empty_pstore.h"
#include "OpenCameraCalibrator/spline/paramstore/dynamic_pstore.h"
#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"

namespace kontiki {
namespace trajectories {
namespace internal {
static const Eigen::Matrix4d M = (Eigen::Matrix4d() <<
                                                    1. / 6., 4. / 6.,  1. / 6., 0,
    -3. / 6.,       0,  3. / 6., 0,
    3. / 6., -6. / 6,  3. / 6., 0,
    -1. / 6.,   3./6., -3. / 6., 1./6.).finished();

static const Eigen::Matrix4d M_cumul = (Eigen::Matrix4d() <<
                                                          6. / 6.,  5. / 6.,  1. / 6., 0,
    0. / 6.,  3. / 6.,  3. / 6., 0,
    0. / 6., -3. / 6.,  3. / 6., 0,
    0. / 6.,  1. / 6., -2. / 6., 1. / 6.).finished();

struct SplineSegmentMeta : public entity::MetaData {
  double t0; // First valid time
  double dt; // Knot spacing
  size_t n; // Number of knots

  SplineSegmentMeta(double dt, double t0) :
      dt(dt),
      t0(t0),
      n(0) { };

  SplineSegmentMeta() :
      SplineSegmentMeta(1.0, 0.0) { };

  size_t NumParameters() const override {
    return n;
  }

  double MinTime() const {
      Validate();
      return t0;
  }

  double MaxTime() const {
    Validate();
    return t0 + (n-3) * dt;
  }

  void Validate() const {
    if (n < 4) {
      throw std::range_error("Spline had too few control points");
    }
  }
};

struct SplineMeta : public entity::MetaData {

  std::vector<SplineSegmentMeta> segments;

  size_t NumParameters() const override {
    /*return std::accumulate(segments.begin(), segments.end(),
                           0, [](int n, SplineSegmentMeta& meta) {
          return n + meta.NumParameters();
        });*/
    int n = 0;
    for (auto &segment_meta : segments) {
      n += segment_meta.NumParameters();
    }
    return n;
  }

  double MinTime() const {
    if (segments.size() == 1)
      return segments[0].MinTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }

  double MaxTime() const {
    if (segments.size() == 1)
      return segments[0].MaxTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }
};

template<typename Type, int Size>
struct ControlPointInfo {
  using type = Type;
  const int size = Size;

  virtual void Validate(const Type&) const {
    // Do nothing
  }

  virtual ceres::LocalParameterization* parameterization() const {
    return nullptr; // No parameterization
  }
};

template<
    typename T,
    typename _ControlPointInfo>
class SplineSegmentView : public TrajectoryView<T, SplineSegmentMeta> {
 public:
  using ControlPointType = typename _ControlPointInfo::type;
  using ControlPointMap = Eigen::Map<ControlPointType>;

  // Inherit constructor
  using TrajectoryView<T, SplineSegmentMeta>::TrajectoryView;

  const ControlPointMap ControlPoint(int i) const {
    return ControlPointMap(this->pstore_->ParameterData(i));
  }

  ControlPointMap MutableControlPoint(int i) {
    return ControlPointMap(this->pstore_->ParameterData(i));
  }

  T t0() const {
    return T(this->meta_.t0);
  }

  T dt() const {
    return T(this->meta_.dt);
  }

  size_t NumKnots() const {
    return this->meta_.n;
  }

  double MinTime() const override {
    return this->meta_.MinTime();
  }

  double MaxTime() const override {
    return this->meta_.MaxTime();
  }

  void CalculateIndexAndInterpolationAmount(T t, int& i0, T& u) const {
    T s = (t - t0()) / dt();
    i0 = PotentiallyUnsafeFloor(s);
    u = s - T(i0);
  }

 protected:
  int PotentiallyUnsafeFloor(double x) const {
    return static_cast<int>(std::floor(x));
  }

  // This way of treating Jets are potentially unsafe, hence the function name
  template<typename Scalar, int N>
  int PotentiallyUnsafeFloor(const ceres::Jet<Scalar, N>& x) const {
    return static_cast<int>(std::floor(x.a));
  };

  _ControlPointInfo control_point_info_;
};


template<typename T, typename MetaType, template<typename...> typename SegmentTemplate>
class SplineView : public TrajectoryView<T, MetaType> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using SegmentView = SegmentTemplate<T>;
  using Base = TrajectoryView<T, MetaType>;
 public:
  using ControlPointType = typename SegmentView::ControlPointType;
  using ControlPointMap = typename SegmentView::ControlPointMap;

  SplineView(const MetaType &meta, entity::ParameterStore<T> *holder) :
      Base(meta, holder) {
    size_t offset = 0;
    for (auto &segment_meta : meta.segments) {
      size_t length = segment_meta.NumParameters();
      segments.push_back(std::make_shared<SegmentView>(segment_meta, holder->Slice(offset, length)));
      offset += length;
    }
  }

  Result Evaluate(T t, int flags) const override {
    int parameter_offset = 0;
    for (auto &seg : segments) {
      if ((t >= seg->MinTime()) && (t < seg->MaxTime())) {
        return seg->Evaluate(t, flags);
      }
    }

    std::stringstream ss;
    ss << "No segment found for time t=" << t;
    ss << "Segments: ";
    for (auto &seg : segments)
      ss << "[" << seg->MinTime() << ", " << seg->MaxTime() << ") ";
    throw std::range_error(ss.str());
  }

  const ControlPointMap ControlPoint(int i) const {
    return this->ConcreteSegmentViewOrError()->ControlPoint(i);
  }

  ControlPointMap MutableControlPoint(int i) {
    return this->MutableConcreteSegmentViewOrError()->MutableControlPoint(i);
  }

  double MinTime() const override{
    return ConcreteSegmentViewOrError()->MinTime();
  }

  double MaxTime() const override {
    return ConcreteSegmentViewOrError()->MaxTime();
  }

  double t0() const {
    return ConcreteSegmentViewOrError()->t0();
  }

  double dt() const {
    return ConcreteSegmentViewOrError()->dt();
  }

  int NumKnots() const {
    return ConcreteSegmentViewOrError()->NumKnots();
  }

  void CalculateIndexAndInterpolationAmount(T t, int& i0, T& u) const {
    return this->ConcreteSegmentViewOrError()->CalculateIndexAndInterpolationAmount(t, i0, u);
  }

 protected:
  std::vector<std::shared_ptr<SegmentView>> segments;

  const std::shared_ptr<SegmentView> ConcreteSegmentViewOrError() const {
    if (segments.size() == 1) {
      return segments[0];
    }
    else {
      throw std::runtime_error("Spline had more than one segment!");
    }
  }

  std::shared_ptr<SegmentView> MutableConcreteSegmentViewOrError() {
    if (segments.size() == 1) {
      return segments[0];
    }
    else {
      throw std::runtime_error("Spline had more than one segment!");
    }
  }

  virtual ceres::LocalParameterization* ControlPointParameterization() {
    return nullptr;
  }
};

using _SplineParamStore = entity::EmptyParameterStore<double>;


template<template<typename...> typename SegmentViewTemplate>
struct SplineFactory {

  template<typename T, typename MetaType>
  struct View : public SplineView<T, MetaType, SegmentViewTemplate> {
    using SplineView<T, MetaType, SegmentViewTemplate>::SplineView;
  };

  template<typename T, typename MetaType>
  struct SegmentView : public SegmentViewTemplate<T> {
    using SegmentViewTemplate<T>::SegmentViewTemplate;
  };

  struct SegmentEntity : public TrajectoryEntity<SegmentView,
                                                 SplineSegmentMeta,
                                                 entity::DynamicParameterStore<double>> {
    using Base = TrajectoryEntity<SegmentView, SplineSegmentMeta, entity::DynamicParameterStore<double>>;
    using ControlPointType = typename Base::ControlPointType;

    SegmentEntity(double dt, double t0) {
      this->meta_.dt = dt;
      this->meta_.t0 = t0;
      this->meta_.n = 0;
    }

    void AddToProblem(ceres::Problem &problem,
                      time_init_t times,
                      SplineSegmentMeta &meta,
                      std::vector<entity::ParameterInfo<double>> &parameters) const override {
      throw std::runtime_error("Segments do not know how to add to problem. This is handled by the Spline entity");
    }

    void AppendKnot(const ControlPointType& cp) {
      // 1) Validate the control point
      this->control_point_info_.Validate(cp);

      // 2) Create parameter data and set its value
      auto i = this->pstore_->AddParameter(this->control_point_info_.size,
                                           this->control_point_info_.parameterization());
      this->MutableControlPoint(i) = cp;

      // 3) Update segment meta
      this->meta_.n += 1;
    }

    // Allow the owning Spline Entity to access parameters
    entity::ParameterInfo<double> Parameter(size_t i) {
      return this->pstore_->Parameter(i);
    }
  };
};

template<template<typename> typename SegmentViewTemplate>
class SplineEntity : public TrajectoryEntity<SplineFactory<SegmentViewTemplate>::template View,
                                             SplineMeta,
                                             _SplineParamStore> {
  using Base = TrajectoryEntity<SplineFactory<SegmentViewTemplate>::template View, SplineMeta, _SplineParamStore>;
  using ControlPointType = typename Base::ControlPointType;
  using SegmentType = typename SplineFactory<SegmentViewTemplate>::SegmentEntity;

  // Hidden constructor, not intended for user code
  SplineEntity(double dt, double t0, ceres::LocalParameterization* control_point_parameterization) :
      segment_entity_(std::make_shared<SegmentType>(dt, t0)),
      control_point_parameterization_(control_point_parameterization) {
    this->segments.push_back(segment_entity_);
  };

 public:

  SplineEntity(double dt, double t0) :
      SplineEntity(dt, t0, this->ControlPointParameterization()) { };

  SplineEntity(double dt) :
      SplineEntity(dt, 0.0) { };

  SplineEntity() :
      SplineEntity(1.0) { };

  SplineEntity(const SplineEntity &rhs) :
    Base(rhs),
    segment_entity_(std::make_shared<SegmentType>(*rhs.segment_entity_)),
    control_point_parameterization_(this->ControlPointParameterization())
  {
    this->segments.push_back(segment_entity_);
  }

  void AppendKnot(const ControlPointType& cp) {
    segment_entity_->AppendKnot(cp);
  }

  void ExtendTo(double t, const ControlPointType& fill_value) {
    while ((this->NumKnots() < 4) || (this->MaxTime() < t)) {
      this->AppendKnot(fill_value);
    }
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    SplineMeta &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    double master_dt = segment_entity_->dt();
    double master_t0 = segment_entity_->t0();
    int current_segment_start = 0;
    int current_segment_end = -1; // Negative signals no segment created yet

    // Times are guaranteed to be sorted correctly and t2 >= t1
    for (auto tt : times) {

      int i1, i2;
      double u_notused;
      segment_entity_->CalculateIndexAndInterpolationAmount(tt.first, i1, u_notused);
      segment_entity_->CalculateIndexAndInterpolationAmount(tt.second, i2, u_notused);

      // Create new segment, or extend the current one
      if (i1 > current_segment_end) {
        double segment_t0 = master_t0 + master_dt * i1;
        meta.segments.push_back(SplineSegmentMeta(master_dt, segment_t0));
        current_segment_start = i1;
      }
      else {
        i1 = current_segment_end + 1;
      }

      auto& current_segment_meta = meta.segments.back();

      // Add parameters and update currently active segment meta
      for (int i=i1; i < (i2 + 4); ++i) {
        auto pi = this->segment_entity_->Parameter(i);
        parameters.push_back(pi);
        problem.AddParameterBlock(pi.data, pi.size, pi.parameterization);

        if (this->IsLocked())
          problem.SetParameterBlockConstant(pi.data);

        current_segment_meta.n += 1;
      }

      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    } // for times
  }

 protected:
  std::shared_ptr<SegmentType> segment_entity_;
  std::unique_ptr<ceres::LocalParameterization> control_point_parameterization_;
};

} // namespace internal
} // namespace trajectories
} // namespace kontiki

#endif //KONTIKIV2_SPLINE_BASE_H
