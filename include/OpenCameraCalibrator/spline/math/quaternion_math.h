#ifndef KONTIKIV2_QUATERNION_MATH_H
#define KONTIKIV2_QUATERNION_MATH_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace kontiki {
namespace math {

static const double eps = 1e-16;
static const double eps_unit_check = 1e-5;

// FIXME: Change to Eigen::Map as inputs and outputs?

template<typename T>
Eigen::Quaternion <T> logq(const Eigen::Quaternion <T> &q) {
  auto qn = q.norm();

  if(ceres::abs(qn - T(1.0)) > eps_unit_check) {
    std::stringstream s;
    s << "logq: Only implemented for unit quaternions. Magnitude was " << qn;
    throw std::runtime_error(s.str());
  }

  // If we write the quaternion as q = (a, v) where a is the scalar part, then for unit quaternions we have
  //
  //     log(q) = (0, (k * v), where k = acos(a) / ||v||)
  //
  // For unit quaternions we also have q = (a, v) = (cos(phi), n*sin(phi)) where n is a unit vector.
  // This means that acos(a) = phi and ||v|| = sin(phi)
  // Then we have
  //
  //     k = phi/sin(phi)
  //
  // Using tan(phi) = sin(phi)/cos(phi)we then get
  //
  //    phi = atan2(sin(phi), cos(phi)) = atan2(||v||, a) and finally
  //    k = atan2(||v||, a) / ||v||
  //
  // Unfortunatey this behaves badly around ||v|| = 0 because of the sqrt in the norm and the division.
  // We can instead do a taylor expansion of k = phi/sin(phi) around ||v|| = sin(phi) = 0 which is in this case
  // equivalent to exapnding around phi=0 since phi encodes the half-angle of the actual rotation.
  // The expansion is phi/sin(phi) = 1 + phi^2/6 + ...
  T k;
  auto v_squared = q.vec().squaredNorm();
  if (v_squared > eps) {
    T vn = ceres::sqrt(v_squared);
    k = ceres::atan2(vn, q.w()) / vn;
  }
  else {
    k = T(1.0); // First term of the Taylor expansion
  }

  Eigen::Quaternion<T> out;
  out.w() = T(0);
  out.vec() = q.vec() * k;

  return out;
}

template<typename T>
Eigen::Quaternion<T> expq(const Eigen::Quaternion<T>& q)
{
  Eigen::Quaternion<T> out;

  // If we write the quaternion as q = (a, v) where a is the scalar part, then for unit quaternions we have
  //
  //    exp(q) = exp(a) * (cos(||v||), v * sin(||v||)/||v||))
  //
  // This is problematic for ||v|| close to zero because of the sqrt in the norm, and the division.
  // When ||v|| is close to zero we instead use the Taylor expansion sin(x)/x = 1 - x^2/6 + ...
  auto v_squared = q.vec().squaredNorm();
  auto ea = ceres::exp(q.w());
  T ka, kv;
  if (v_squared > eps) {
    T v_norm = ceres::sqrt(v_squared);
    ka = ea * ceres::cos(v_norm);
    kv = ea * ceres::sin(v_norm) / v_norm;
  }
  else {
    ka = ea; // cos(||v||) ~= 1
    kv = ea; // Use first term (1) in the Taylor expansion of sin(x)/x
  }

  out.w() = ka;
  out.vec() = kv * q.vec();

  return out;
}

template<typename T>
Eigen::Matrix<T, 3, 1> angular_velocity(const Eigen::Quaternion<T> &q, const Eigen::Quaternion<T> &dq) {
  Eigen::Quaternion<T> w(T(2) * (dq * q.conjugate()).coeffs());
  return w.vec();
};

template<typename T>
Eigen::Quaternion<T> embed_vector(const Eigen::Matrix<T, 3, 1> &v) {
  return Eigen::Quaternion<T>(T(0), v(0), v(1), v(2));
}

template<typename T>
Eigen::Quaternion<T> dq_from_angular_velocity(const Eigen::Matrix<T, 3, 1> &w, const Eigen::Quaternion<T> &q) {
  Eigen::Quaternion<T> qw = embed_vector(w);
  return Eigen::Quaternion<T>(T(0.5) * (qw * q).coeffs());
}

// Calculate the quaternion product qa * qx * qb, where qx is x embedded in a quaternion
template<typename T>
Eigen::Matrix<T, 3, 1> vector_sandwich(const Eigen::Quaternion<T> &qa,
                                       const Eigen::Matrix<T, 3, 1> &x,
                                       const Eigen::Quaternion<T> &qb) {
  Eigen::Quaternion<T> qx = embed_vector(x);
  return (qa * qx * qb).vec();
};

template<typename T>
bool IsUnitQuaternion(const Eigen::Quaternion<T>& q) {
  auto err = ceres::abs(q.norm() - T(1));
  return err < T(math::eps_unit_check);
}

} // namespace math
} // namespace kontiki
#endif //KONTIKIV2_QUATERNION_MATH_H
