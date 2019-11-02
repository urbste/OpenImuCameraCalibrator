// Copyright Steffen Urban


#pragma once

#include "pinhole_camera.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "OpenCameraCalibrator/spline/paramstore/dynamic_pstore.h"

namespace kontiki {
namespace sensors {

namespace internal {

struct DivisionUndistortionMeta : public PinholeMeta {
  double distortion;

  size_t NumParameters() const override {
    return PinholeMeta::NumParameters();
  }
};

template<typename T, typename MetaType>
class DivisionUndistortionView : public PinholeView<T, MetaType> {
  using Base = PinholeView<T, MetaType>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<CameraEvaluation<T>>;
 public:
  // Inherit constructors
  using Base::PinholeView;

  T distortion() const {
    return T(this->meta_.distortion);
  }

  void set_distortion(T distortion) {
    this->meta_.distortion = (double) distortion;
  }

  Result EvaluateProjection(const Vector3 &X, const Vector3 &dX, bool derive) const override {
    const T eps = T(1e-32);

    auto result = std::make_unique<CameraEvaluation<T>>(derive);

    Vector2 A = X.head(2)/(X(2) + eps);
    A(0) *= this->camera_matrix()(0,0);
    A(1) *= this->camera_matrix()(1,1);

    const T r_u_sq = A(0) * A(0) + A(1) * A(1);
    const T r_u = ceres::sqrt(r_u_sq);

    const T denom = 2.0 * distortion() * r_u;

    // If the denominator is nearly zero, use L'Hopital's rule to compute r_d by
    // taking the derivatives of the numerator and denominator.
    T r_d_sq;
    if (denom < eps && denom > -eps)
    {
        // We can directly compute r_d_sq to avoid a sqrt call.
        r_d_sq = 1.0 / (1.0 - 4.0 * distortion() * r_u_sq);
    }
    else
    {
        const T r_d = (1.0 - ceres::sqrt(1.0 - 4.0 * distortion() * r_u_sq)) / denom;
        r_d_sq = r_d * r_d;
    }

    // Plug in r_d into Eq (2) to obtain the distorted point.
    T rad = 1.0 + distortion() * r_d_sq;
    Vector3 Y(A(0) * rad + this->camera_matrix()(0,2),
              A(1) * rad + this->camera_matrix()(1,2),
              T(1.0));

    result->y = Y.head(2);

    if (derive) {

    }

    return result;
  }
  Vector3 Unproject(const Vector2 &y) const override {
      Vector3 ph(y(0)-this->camera_matrix()(0,2), y(1)-this->camera_matrix()(1,2), T(1.0));
      const T r_d_sq = ph(0) * ph(0) + ph(1) * ph(1);

      T undistortion = 1.0 / (1.0 + distortion() * r_d_sq);

      Vector3 Y(
              ph(0) * undistortion / this->camera_matrix()(0,0),
              ph(1) * undistortion / this->camera_matrix()(1,1),
              T(1));


      return Y;
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class DivisionUndistortionEntity : public PinholeEntity<ViewTemplate, MetaType, StoreType> {
  using Base = PinholeEntity<ViewTemplate, MetaType, StoreType>;

 public:
  // Import constructors
  using Base::PinholeEntity;

  DivisionUndistortionEntity(size_t rows, size_t cols,const double readout,
                             const Eigen::Matrix3d &camera_matrix, const double distortion) :
      Base(rows, cols, readout, camera_matrix) {
    this->set_distortion(distortion);
  }
};
} // namespace internal

class DivisionUndistortionCamera : public internal::DivisionUndistortionEntity<internal::DivisionUndistortionView,
                                               internal::DivisionUndistortionMeta,
                                               entity::DynamicParameterStore<double>> {
 public:
  using internal::DivisionUndistortionEntity<internal::DivisionUndistortionView,
                             internal::DivisionUndistortionMeta,
                             entity::DynamicParameterStore<double>>::DivisionUndistortionEntity;

  static constexpr const char *CLASS_ID = "DivisionUndistortionCamera";
};

} // namespace sensors
} // namespace kontiki

