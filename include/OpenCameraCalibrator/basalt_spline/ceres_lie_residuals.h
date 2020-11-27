#pragma once

#include "ceres_spline_helper_old.h"

template <int _N, template <class> class GroupT>
struct LieGroupSplineValueCostFunctor : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using Groupd = GroupT<double>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LieGroupSplineValueCostFunctor(const Groupd& measurement, double u)
      : measurement(measurement), u(u) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Group = GroupT<T>;
    using Tangent = typename GroupT<T>::Tangent;

    Group res;
    CeresSplineHelper<N>::template evaluate_lie<T, GroupT>(sKnots, u, 1, &res,
                                                           nullptr, nullptr);

    Eigen::Map<Tangent> residuals(sResiduals);
    residuals = (res * measurement.inverse()).log();

    return true;
  }

  Groupd measurement;
  double u;
};

template <int _N, template <class> class GroupT, bool OLD_TIME_DERIV>
struct LieGroupSplineVelocityCostFunctor : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using Tangentd = typename GroupT<double>::Tangent;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LieGroupSplineVelocityCostFunctor(const Tangentd& measurement, double u,
                                    double inv_dt, double inv_std = 1)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Tangent = typename GroupT<T>::Tangent;

    Eigen::Map<Tangent> residuals(sResiduals);

    Tangent rot_vel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<N>::template evaluate_lie_vel_old<T, GroupT>(
          sKnots, u, inv_dt, nullptr, &rot_vel);
    } else {
      CeresSplineHelper<N>::template evaluate_lie<T, GroupT>(
          sKnots, u, inv_dt, nullptr, &rot_vel, nullptr);
    }

    residuals = inv_std * (rot_vel - measurement);

    return true;
  }

  Tangentd measurement;
  double u, inv_dt, inv_std;
};

template <int _N, template <class> class GroupT, bool OLD_TIME_DERIV>
struct LieGroupSplineAccelerationCostFunctor : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using Tangentd = typename GroupT<double>::Tangent;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LieGroupSplineAccelerationCostFunctor(const Tangentd& measurement, double u,
                                        double inv_dt)
      : measurement(measurement), u(u), inv_dt(inv_dt) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Tangent = typename GroupT<T>::Tangent;

    Eigen::Map<Tangent> residuals(sResiduals);

    Tangent rot_accel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<N>::template evaluate_lie_accel_old<T, GroupT>(
          sKnots, u, inv_dt, nullptr, nullptr, &rot_accel);
    } else {
      CeresSplineHelper<N>::template evaluate_lie<T, GroupT>(
          sKnots, u, inv_dt, nullptr, nullptr, &rot_accel);
    }

    residuals = rot_accel - measurement;

    return true;
  }

  Tangentd measurement;
  double u, inv_dt;
};
