//
// Created by hannes on 2018-01-15.
//

#ifndef KONTIKIV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
#define KONTIKIV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H

#include <Eigen/Dense>

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/trajectories/spline_base.h"
#include "OpenCameraCalibrator/spline/math/quaternion_math.h"

namespace kontiki {
namespace trajectories {
namespace internal {

template<typename T>
struct SO3SplineControlPointInfo : public ControlPointInfo<Eigen::Quaternion<T>, 4> {
  SO3SplineControlPointInfo() :
    parameterization_(new ceres::EigenQuaternionParameterization) { };

  void Validate(const Eigen::Quaternion<T> &cp) const override {
    if (!math::IsUnitQuaternion(cp)) {
      throw std::domain_error("Control point must be unit quaternion!");
    }
  }
  ceres::LocalParameterization *parameterization() const override {
    return parameterization_.get();
  }

  std::unique_ptr<ceres::EigenQuaternionParameterization> parameterization_;
};

template <typename T>
class UniformSO3SplineSegmentView : public SplineSegmentView<T, SO3SplineControlPointInfo<T>> {
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using Base = SplineSegmentView<T, SO3SplineControlPointInfo<T>>;
 public:
  // Inherit constructors
  using Base::SplineSegmentView;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    if (result->needs.Position())
      result->position.setZero();
    if (result->needs.Velocity())
      result->velocity.setZero();
    if (result->needs.Acceleration())
      result->acceleration.setZero();

    // Early return if we shouldn't calculate rotation components
    if (!result->needs.AnyRotation()) {
      return result;
    }

    // Since angular velocity computations implicitly requires orientation computations
    // we can henceforth assume do_orientation=true

    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    Vector4 U, dU;
    Vector4 B, dB;
    T u2 = ceres::pow(u, 2);
    T u3 = ceres::pow(u, 3);
    T dt_inv = T(1) / this->dt();

    U = Vector4(T(1), u, u2, u3);
    B = U.transpose() * M_cumul.cast<T>();

    if (result->needs.AngularVelocity()) {
      dU = dt_inv * Vector4(T(0), T(1), T(2) * u, T(3) * u2);
      dB = dU.transpose() * M_cumul.cast<T>();
    }

    Quaternion &q = result->orientation;

    // These parts are updated when computing the derivative
    Quaternion dq_parts[3] = {{T(1), T(0), T(0), T(0)},
                              {T(1), T(0), T(0), T(0)},
                              {T(1), T(0), T(0), T(0)}};

    q = this->ControlPoint(i0);
    const int K = (i0 + 4);
    for (int i=(i0 + 1); i < K; ++i) {
      // Orientation
      QuaternionMap qa = this->ControlPoint(i - 1);
      QuaternionMap qb = this->ControlPoint(i);
      Quaternion omega = math::logq(qa.conjugate() * qb);
      Quaternion eomegab = math::expq(Quaternion(omega.coeffs() * B(i-i0)));
      q *= eomegab;

      // Angular velocity
      // This iterative scheme follows from the product rule of derivatives
      if (result->needs.AngularVelocity()) {
        for (int j = (i0 + 1); j < K; ++j) {
          const int m = j - i0 - 1;
          if (i==j) {
            dq_parts[m] *= Quaternion(omega.coeffs()*dB(i-i0));
          }
          dq_parts[m] *= eomegab;
        }
      }
    }

    if (result->needs.AngularVelocity()) {
      Quaternion dq = this->ControlPoint(i0) * Quaternion(dq_parts[0].coeffs() + dq_parts[1].coeffs() + dq_parts[2].coeffs());
      result->angular_velocity = math::angular_velocity(q, dq);
    }

    return result;
  }

};

} // namespace internal

class UniformSO3SplineTrajectory : public internal::SplineEntity<internal::UniformSO3SplineSegmentView> {
 public:
  static constexpr const char* CLASS_ID = "UniformSO3SplineTrajectory";
  using internal::SplineEntity<internal::UniformSO3SplineSegmentView>::SplineEntity;
};

} // namespace trajectories
} // namespace kontiki

#endif //KONTIKIV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
