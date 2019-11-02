//
// Created by hannes on 2017-04-12.
//

#ifndef KONTIKI_ATAN_H
#define KONTIKI_ATAN_H

#include "pinhole_camera.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "OpenCameraCalibrator/spline/paramstore/dynamic_pstore.h"

namespace kontiki {
namespace sensors {

namespace internal {

struct AtanMeta : public PinholeMeta {
  double gamma;
  Eigen::Vector2d wc;

  size_t NumParameters() const override {
    return PinholeMeta::NumParameters();
  }
};

template<typename T, typename MetaType>
class AtanView : public PinholeView<T, MetaType> {
  using Base = PinholeView<T, MetaType>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<CameraEvaluation<T>>;
 public:
  // Inherit constructors
  using Base::PinholeView;

  Vector2 wc() const {
    return this->meta_.wc.template cast<T>();
  }

  void set_wc(const Vector2& wc) {
    this->meta_.wc = wc.template cast<double>();
  }

  T gamma() const {
    return T(this->meta_.gamma);
  }

  void set_gamma(T gamma) {
    this->meta_.gamma = (double) gamma;
  }

  Result EvaluateProjection(const Vector3 &X, const Vector3 &dX, bool derive) const override {
    const T eps = T(1e-32);

    auto result = std::make_unique<CameraEvaluation<T>>(derive);

    // Common parts
    Vector2 A = X.head(2)/(X(2) + eps);
    Vector2 L = A - wc();
    T r = ceres::sqrt(L.squaredNorm() + eps);
    T f = ceres::atan(r*gamma())/gamma();

    Vector3 Y(T(0), T(0), T(1));
    Vector2 g = L / r;
    Y.head(2) = wc() + f*g;

    // Apply camera matrix
    // Normalization not needed since Y(2) == 1
    result->y = (this->camera_matrix() * Y).head(2);

    if (derive) {
      T dx = (dX(0)*X(2) - X(0)*dX(2)) / (X(2)*X(2) + eps);
      T dy = (dX(1)*X(2) - X(1)*dX(2)) / (X(2)*X(2) + eps);

      T common = (g(0) * dx + g(1) * dy);
      T df = common / (T(1) + ceres::pow(gamma(), 2)*r*r);

      T dgu = (dx * r - L(0) * common) / (r*r);
      T du = f * dgu + df * g(0);

      T dgv = (dy*r - L(1) * common) / (r*r);
      T dv = f * dgv + df * g(1);

      result->dy = (this->camera_matrix() * Eigen::Matrix<T, 3, 1>(du, dv, T(0.0))).head(2);
    }

    return result;
  }
  Vector3 Unproject(const Vector2 &y) const override {
    const T eps = T(1e-32);
    Vector3 ph(y(0), y(1), T(1.0));
    Vector3 phn = this->camera_matrix().inverse() * ph;
    Vector2 L = phn.head(2) - wc();

    T r = ceres::sqrt(L.squaredNorm() + eps);
    T f = ceres::tan(r*gamma())/gamma();

    Vector3 Y(T(0), T(0), T(1));
    Y.head(2) = wc() + f*L/r;
    return Y;
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class AtanEntity : public PinholeEntity<ViewTemplate, MetaType, StoreType> {
  using Base = PinholeEntity<ViewTemplate, MetaType, StoreType>;

 public:
  // Import constructors
  using Base::PinholeEntity;

  AtanEntity(size_t rows, size_t cols, double readout, const Eigen::Matrix3d &camera_matrix, const Eigen::Vector2d& wc, double gamma) :
      Base(rows, cols, readout, camera_matrix) {
    this->set_gamma(gamma);
    this->set_wc(wc);
  }
};
} // namespace internal

class AtanCamera : public internal::AtanEntity<internal::AtanView,
                                               internal::AtanMeta,
                                               entity::DynamicParameterStore<double>> {
 public:
  using internal::AtanEntity<internal::AtanView,
                             internal::AtanMeta,
                             entity::DynamicParameterStore<double>>::AtanEntity;

  static constexpr const char *CLASS_ID = "AtanCamera";
};

} // namespace sensors
} // namespace kontiki

#endif //KONTIKI_ATAN_H
