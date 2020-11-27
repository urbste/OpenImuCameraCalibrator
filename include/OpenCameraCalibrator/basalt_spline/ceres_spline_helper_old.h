#pragma once

#include "ceres_spline_helper.h"


template <int _N>
struct CeresSplineHelperOld : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  template <class T, template <class> class GroupT>
  static inline void evaluate_lie_vel_old(
      T const* const* sKnots, const double u, const double inv_dt,
      GroupT<T>* transform_out = nullptr,
      typename GroupT<T>::Tangent* vel_out = nullptr) {
    if (!vel_out && !transform_out) return;

    using Group = GroupT<T>;
    using Tangent = typename GroupT<T>::Tangent;
    using Transformation = typename GroupT<T>::Transformation;

    VecN p, coeff, dcoeff;

    CeresSplineHelper<N>::template baseCoeffsWithTime<0>(p, u);
    coeff = CeresSplineHelper<N>::cumulative_blending_matrix_ * p;

    CeresSplineHelper<N>::template baseCoeffsWithTime<1>(p, u);
    dcoeff = inv_dt * CeresSplineHelper<N>::cumulative_blending_matrix_ * p;

    Group accum;
    Transformation rot_vel_mat[DEG];

    for (int i = 0; i < DEG; i++) {
      Eigen::Map<Group const> const p0(sKnots[i]);
      Eigen::Map<Group const> const p1(sKnots[i + 1]);

      Group r01 = p0.inverse() * p1;
      Tangent delta = r01.log();
      Group exp_delta = Group::exp(delta * coeff[i + 1]);
      Transformation A_i = exp_delta.matrix();

      for (int j = 0; j < DEG; j++) {
        if (i == 0) {
          rot_vel_mat[j] = A_i;
        } else {
          rot_vel_mat[j] *= A_i;
        }

        if (i == j) {
          rot_vel_mat[j] *= Group::hat(delta * dcoeff[i + 1]);
        }
      }

      if (i == 0) {
        accum = exp_delta;
      } else {
        accum *= exp_delta;
      }
    }

    for (int i = 1; i < DEG; i++) {
      rot_vel_mat[0] += rot_vel_mat[i];
    }

    rot_vel_mat[0] = accum.inverse().matrix() * rot_vel_mat[0];

    if (transform_out) {
      Eigen::Map<Group const> const p00(sKnots[0]);
      *transform_out = p00 * accum;
    }

    if (vel_out) *vel_out = Group::vee(rot_vel_mat[0]);
  }

  template <class T, template <class> class GroupT>
  static inline void evaluate_lie_accel_old(
      T const* const* sKnots, const double u, const double inv_dt,
      GroupT<T>* transform_out = nullptr,
      typename GroupT<T>::Tangent* vel_out = nullptr,
      typename GroupT<T>::Tangent* accel_out = nullptr) {
    using Group = GroupT<T>;
    using Tangent = typename GroupT<T>::Tangent;
    using Transformation = typename GroupT<T>::Transformation;

    VecN p, coeff, dcoeff, ddcoeff;

    CeresSplineHelper<N>::template baseCoeffsWithTime<0>(p, u);
    coeff = CeresSplineHelper<N>::cumulative_blending_matrix_ * p;

    CeresSplineHelper<N>::template baseCoeffsWithTime<1>(p, u);
    dcoeff = inv_dt * CeresSplineHelper<N>::cumulative_blending_matrix_ * p;

    CeresSplineHelper<N>::template baseCoeffsWithTime<2>(p, u);
    ddcoeff =
        inv_dt * inv_dt * CeresSplineHelper<N>::cumulative_blending_matrix_ * p;

    Transformation A[DEG];
    Transformation Adot[DEG];
    Transformation Adotdot[DEG];

    Group val;

    for (int i = 0; i < DEG; i++) {
      Eigen::Map<Group const> const p0(sKnots[i]);
      Eigen::Map<Group const> const p1(sKnots[i + 1]);

      Group r01 = p0.inverse() * p1;
      Tangent delta = r01.log();

      Transformation omega_hat = Group::hat(delta);
      Group v = Group::exp(delta * coeff[i + 1]);
      A[i] = v.matrix();
      Adot[i] = A[i] * omega_hat * dcoeff[i + 1];
      Adotdot[i] = Adot[i] * omega_hat * dcoeff[i + 1] +
                   A[i] * omega_hat * ddcoeff[i + 1];

      if (i == 0) {
        val = v;
      } else {
        val *= v;
      }
    }

    Transformation rot_vel_mat[DEG];
    Transformation rot_accel_mat1[DEG];

    // 1 part
    for (int i = 0; i < DEG; i++) {
      for (int j = 0; j < DEG; j++) {
        const Transformation& r = (i == j) ? Adot[i] : A[i];
        const Transformation& rd = (i == j) ? Adotdot[i] : A[i];

        if (i == 0) {
          rot_vel_mat[j] = r;
          rot_accel_mat1[j] = rd;
        } else {
          rot_vel_mat[j] *= r;
          rot_accel_mat1[j] *= rd;
        }
      }
    }

    // Permutation part
    constexpr int size = DEG * (DEG - 1) / 2;
    int permutations[size][2];
    int idx = 0;
    for (int j = 0; j < DEG - 1; j++) {
      for (int k = j + 1; k < DEG; k++) {
        permutations[idx][0] = j;
        permutations[idx][1] = k;
        idx++;
      }
    }

    Transformation rot_accel_mat2[size];

    for (int i = 0; i < DEG; i++) {
      for (int j = 0; j < size; j++) {
        Transformation r = (i == permutations[j][0] || i == permutations[j][1])
                               ? Adot[i]
                               : A[i];
        if (i == 0) {
          rot_accel_mat2[j] = r;
        } else {
          rot_accel_mat2[j] *= r;
        }
      }
    }

    for (int i = 1; i < DEG; i++) {
      rot_vel_mat[0] += rot_vel_mat[i];
      rot_accel_mat1[0] += rot_accel_mat1[i];
    }

    for (int i = 0; i < size; i++) {
      rot_accel_mat1[0] += 2. * rot_accel_mat2[i];
    }

    rot_vel_mat[0] = val.inverse().matrix() * rot_vel_mat[0];
    rot_accel_mat1[0] = val.inverse().matrix() * rot_accel_mat1[0];
    rot_accel_mat1[0] -= rot_vel_mat[0] * rot_vel_mat[0];

    if (transform_out) {
      Eigen::Map<Group const> const p00(sKnots[0]);
      *transform_out = p00 * val;
    }

    if (vel_out) *vel_out = Group::vee(rot_vel_mat[0]);
    if (accel_out) *accel_out = Group::vee(rot_accel_mat1[0]);
  }
};
