//
// Created by hannes on 2018-02-05.
//

#ifndef KONTIKIV2_UNIFORM_SE3_SPLINE_H
#define KONTIKIV2_UNIFORM_SE3_SPLINE_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <sophus/se3.hpp>

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/spline_base.h"

namespace Sophus {
class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d const> const delta(delta_raw);
    Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE3d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Matrix<double, 6, 7> > jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const { return SE3d::num_parameters; }

  virtual int LocalSize() const { return SE3d::DoF; }
};
}

namespace kontiki {
namespace trajectories {
namespace internal {

template<typename T>
struct SE3SplineControlPointInfo : public ControlPointInfo<Sophus::SE3<T>, Sophus::SE3d::num_parameters> {

  SE3SplineControlPointInfo() :
    parameterization_(new Sophus::LocalParameterizationSE3) { };

  ceres::LocalParameterization *parameterization() const override {
    return parameterization_.get();
  }

  std::unique_ptr<Sophus::LocalParameterizationSE3> parameterization_;
};

template <typename T>
class UniformSE3SplineSegmentView : public SplineSegmentView<T, SE3SplineControlPointInfo<T>> {
  using Base = SplineSegmentView<T, SE3SplineControlPointInfo<T>>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using Matrix4 = Eigen::Matrix<T, 4, 4>;
  using SE3Type = Sophus::SE3<T>;
  using SE3Map = Eigen::Map<SE3Type>;
 public:
  // Inherit constructors
  using Base::SplineSegmentView;

  Result Evaluate(T t, int flags) const override {
    SE3Type P;
    Matrix4 P_prim, P_bis;
    EvaluateSpline(t, flags, P, P_prim, P_bis);

    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);
    result->position = P.translation();
    result->velocity = P_prim.col(3).head(3);
    result->acceleration = P_bis.col(3).head(3);

    result->orientation = P.unit_quaternion();

    Eigen::Matrix<T, 3, 3> omega_hat = P_prim.topLeftCorner(3, 3) * P.rotationMatrix().transpose();
    result->angular_velocity(0) = 0.5 * (omega_hat(2, 1) - omega_hat(1, 2));
    result->angular_velocity(1) = 0.5 * (omega_hat(0, 2) - omega_hat(2, 0));
    result->angular_velocity(2) = 0.5 * (omega_hat(1, 0) - omega_hat(0, 1));

    return result;
  }

  void EvaluateSpline(T t, int flags, SE3Type &P, Matrix4 &P_prim, Matrix4 &P_bis) const {
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    int num_derivatives;

    if (result->needs.Acceleration()) {
      num_derivatives = 2;
    }
    else if (result->needs.Velocity() || result->needs.AngularVelocity()) {
      num_derivatives = 1;
    }
    else {
      num_derivatives = 0;
    }



    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    Vector4 U, dU, d2U;
    Vector4 B, dB, d2B;
    T u2 = ceres::pow(u, 2);
    T u3 = ceres::pow(u, 3);
    T dt_inv = T(1) / this->dt();

    U = Vector4(T(1), u, u2, u3);
    B = U.transpose() * M_cumul.cast<T>();

    if (result->needs.AngularVelocity() || result->needs.Velocity()) {
      dU = dt_inv * Vector4(T(0), T(1), T(2) * u, T(3) * u2);
      dB = dU.transpose() * M_cumul.cast<T>();
    }

    if (result->needs.Acceleration()) {
      d2U = ceres::pow(dt_inv, 2)*Vector4(T(0), T(0), T(2), T(6)*u);
      d2B = d2U.transpose() * M_cumul.cast<T>();
    }

    // Components needed to calculate derivatives
    Matrix4 A[3];
    Matrix4 A_prim[3];
    Matrix4 A_bis[3];
    Matrix4 Aj_prim;

    P = this->ControlPoint(i0); // Pose

    const int K = (i0 + 4);
    for (int i=(i0 + 1); i < K; ++i) {
      int j = i - i0;
      SE3Map Pa = this->ControlPoint(i - 1);
      SE3Map Pb = this->ControlPoint(i);
      typename SE3Type::Tangent omega = (Pa.inverse() * Pb).log();
      Matrix4 omega_hat = SE3Type::hat(omega);
      SE3Type Aj = SE3Type::exp(B(j) * omega);
      P *= Aj;

      if (num_derivatives >= 1) {
        A[j-1] = Aj.matrix();
        Aj_prim = Aj.matrix() * omega_hat * dB(j);
        A_prim[j - 1] = Aj_prim;
      }

      if (num_derivatives >= 2) {
        A_bis[j - 1] = Aj_prim * omega_hat * dB(j) + Aj.matrix() * omega_hat * d2B(j);
      }

    } // for i (control points)

    SE3Map P0 = this->ControlPoint(i0);
    if (num_derivatives >= 1) {
      Matrix4 M1 = A_prim[0] * A[1] * A[2] +
          A[0] * A_prim[1] * A[2] +
          A[0] * A[1] * A_prim[2];
      P_prim = P0.matrix() * M1;
    }

    if (num_derivatives >= 2) {
      Matrix4 M2 = A_bis[0] * A[1] * A[2] + A[0] * A_bis[1] * A[2] +
          A[0] * A[1] * A_bis[2] + T(2.0) * A_prim[0] * A_prim[1] * A[2] +
          T(2.0) * A_prim[0] * A[1] * A_prim[2] + T(2.0) * A[0] * A_prim[1] * A_prim[2];
      P_bis = P0.matrix() * M2;
    }


  }
};

} // namespace internal

class UniformSE3SplineTrajectory : public internal::SplineEntity<internal::UniformSE3SplineSegmentView> {
 public:
  static constexpr const char* CLASS_ID = "UniformSE3SplineTrajectory";
  using internal::SplineEntity<internal::UniformSE3SplineSegmentView>::SplineEntity;

  void EvaluateSpline(double t, int flags, Sophus::SE3d &P, Eigen::Matrix4d &P_prim, Eigen::Matrix4d &P_bis) const {
    this->segment_entity_->EvaluateSpline(t, flags, P, P_prim, P_bis);
  }
};

} // namespace trajectories
} // namespace kontiki

#endif //KONTIKIV2_UNIFORM_SE3_SPLINE_H
