//
// Created by hannes on 2017-03-20.
//

#ifndef KONTIKI_PINHOLE_CAMERA_H
#define KONTIKI_PINHOLE_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include "OpenCameraCalibrator/spline/sensors/camera.h"
#include "OpenCameraCalibrator/spline/paramstore/dynamic_pstore.h"

namespace kontiki {
namespace sensors {

namespace internal {

struct PinholeMeta : public CameraMeta {
  size_t NumParameters() const override {
    return CameraMeta::NumParameters();
  }

  Eigen::Matrix3d camera_matrix; // FIXME: This should be a set of parameters
};

template<typename T, typename MetaType>
class PinholeView : public CameraView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Result = std::unique_ptr<CameraEvaluation<T>>;
 public:
  using CameraMatrix = Eigen::Matrix<T, 3, 3>;

  // Inherit constructors
  using CameraView<T, MetaType>::CameraView;

  CameraMatrix camera_matrix() const {
    return this->meta_.camera_matrix.template cast<T>();
  }

  void set_camera_matrix(const CameraMatrix& K) {
    this->meta_.camera_matrix = K.template cast<double>();
  }

  Result EvaluateProjection(const Vector3 &X, const Vector3 &dX, bool derive) const override {
    auto result = std::make_unique<CameraEvaluation<T>>(derive);
    Vector3 p = camera_matrix() * X;
    result->y = p.head(2)/p(2);

    if (derive) {
      const T z_eps = T(1e-32);
      Vector3 dp = camera_matrix() * dX;
      T denominator = (p(2)*p(2)) + z_eps;
      result->dy(0) = ((dp(0) * p(2)) - (p(0)*dp(2))) / denominator;
      result->dy(1) = ((dp(1) * p(2)) - (p(1)*dp(2))) / denominator;
    }

    return result;
  }

  Vector3 Unproject(const Vector2 &y) const override {
    Vector3 xh(y(0), y(1), T(1));
    CameraMatrix K = camera_matrix();
    return camera_matrix().inverse() * xh;
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class PinholeEntity : public CameraEntity<ViewTemplate, MetaType, StoreType> {
  using Base = CameraEntity<ViewTemplate, MetaType, StoreType>;
 public:
  using CameraMatrix = Eigen::Matrix3d;

  PinholeEntity(size_t rows, size_t cols, double readout, const CameraMatrix &camera_matrix) :
    Base(rows, cols, readout) {
    this->set_camera_matrix(camera_matrix);
  }

  PinholeEntity(size_t rows, size_t cols, double readout) :
      PinholeEntity(rows, cols, readout, Eigen::Matrix3d::Identity()) {
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    meta = this->meta_;
    Base::AddToProblem(problem, times, meta, parameters);
  }
};

} // namespace internal



class PinholeCamera : public internal::PinholeEntity<internal::PinholeView,
                                                     internal::PinholeMeta,
                                                     entity::DynamicParameterStore<double>> {
 public:
  using internal::PinholeEntity< internal::PinholeView,
                                 internal::PinholeMeta,
                                 entity::DynamicParameterStore<double>>::PinholeEntity;

  static constexpr const char *CLASS_ID = "PinholeCamera";
};

} // namespace sensors
} // namespace kontiki
#endif //KONTIKI_PINHOLE_CAMERA_H
