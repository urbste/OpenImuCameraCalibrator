#pragma once

#include "ceres_spline_helper.h"
#include "common_types.h"
//#include "ceres_spline_helper_old.h"
#include "calib_helpers.h"

#include <Eigen/Core>
#include <theia/sfm/camera/camera.h>
#include <theia/sfm/camera/camera_intrinsics_model.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/double_sphere_camera_model.h>
#include <theia/sfm/camera/extended_unified_camera_model.h>
#include <theia/sfm/camera/fisheye_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

#include <theia/sfm/reconstruction.h>

#include <sophus/so3.hpp>

template <int _N>
struct CalibAccelerationCostFunctorSplit
    : public CeresSplineHelper<double, _N> {
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibAccelerationCostFunctorSplit(const Eigen::Vector3d &measurement,
                                    double u_r3, double inv_r3_dt, double u_so3,
                                    double inv_so3_dt, double inv_std,
                                    const bool calib_bias)
      : measurement(measurement), u_r3(u_r3), inv_r3_dt(inv_r3_dt),
        u_so3(u_so3), inv_so3_dt(inv_so3_dt), inv_std(inv_std),
        calib_bias(calib_bias) {}

  template <class T>
  bool operator()(T const *const *sKnots, T *sResiduals) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vector3> residuals(sResiduals);

    Sophus::SO3<T> R_w_i;
    CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
        sKnots, T(u_so3), T(inv_so3_dt), &R_w_i);

    Vector3 accel_w;
    CeresSplineHelper<T, N>::template evaluate<3, 2>(sKnots + N, T(u_r3),
                                                     T(inv_r3_dt), &accel_w);

    // Gravity
    Eigen::Map<Vector3 const> const g(sKnots[2 * N]);
    if (calib_bias) {
      Eigen::Map<Vector3 const> const bias(sKnots[2 * N + 1]);
      residuals =
          inv_std * (R_w_i.inverse() * (accel_w + g) - measurement + bias);
    } else {
      residuals = inv_std * (R_w_i.inverse() * (accel_w + g) - measurement);
    }

    return true;
  }

  Eigen::Vector3d measurement;
  double u_r3;
  double u_so3;
  double inv_r3_dt;
  double inv_so3_dt;
  double inv_std;
  bool calib_bias;
};

template <int _N, template <class> class GroupT, bool OLD_TIME_DERIV>
struct CalibGyroCostFunctorSplit : public CeresSplineHelper<double, _N> {
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using Tangentd = typename GroupT<double>::Tangent;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibGyroCostFunctorSplit(const Tangentd &measurement, double u_so3,
                            double inv_so3_dt, double inv_std = 1,
                            bool calib_bias = true)
      : measurement(measurement), u_so3(u_so3), inv_so3_dt(inv_so3_dt),
        inv_std(inv_std), calib_bias(calib_bias) {}

  template <class T>
  bool operator()(T const *const *sKnots, T *sResiduals) const {
    using Tangent = typename GroupT<T>::Tangent;

    Eigen::Map<Tangent> residuals(sResiduals);

    Tangent rot_vel;

    CeresSplineHelper<T, N>::template evaluate_lie<GroupT>(
        sKnots, T(u_so3), T(inv_so3_dt), nullptr, &rot_vel);
    if (calib_bias) {
      Eigen::Map<Tangent const> const bias(sKnots[N]);

      residuals = inv_std * (rot_vel - measurement + bias);
    } else {
      residuals = inv_std * (rot_vel - measurement);
    }
    return true;
  }

  Tangentd measurement;
  double u_so3, inv_std;
  double inv_so3_dt;
  bool calib_bias;
};

template <int _N>
struct CalibReprojectionCostFunctorSplit
    : public CeresSplineHelper<double, _N> {
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibReprojectionCostFunctorSplit(const CalibCornerData *corner_data,
                                    const theia::Reconstruction *calib,
                                    const theia::Camera *cam, double u_so3,
                                    double u_r3, double inv_so3_dt,
                                    double inv_r3_dt)
      : corners(corner_data), calib(calib), cam(cam), u_so3(u_so3), u_r3(u_r3),
        inv_so3_dt(inv_so3_dt), inv_r3_dt(inv_r3_dt) {}

  template <class T>
  bool operator()(T const *const *sKnots, T *sResiduals) const {
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector4 = Eigen::Matrix<T, 4, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Sophus::SO3<T> R_w_i;
    CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
        sKnots, T(u_so3), T(inv_so3_dt), &R_w_i);

    Vector3 t_w_i;
    CeresSplineHelper<T, N>::template evaluate<3, 0>(sKnots + N, T(u_so3),
                                                     T(inv_r3_dt), &t_w_i);

    Eigen::Map<Sophus::SE3<T> const> const T_i_c(sKnots[2 * N]);

    Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
    Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

    T intr[10];
    for (int i = 0; i < cam->CameraIntrinsics()->NumParameters(); ++i) {
      intr[i] = T(cam->intrinsics()[i]);
    }
    for (size_t i = 0; i < corners->track_ids.size(); ++i) {
      Vector3 p3d =
          (T_c_w_matrix * calib->Track(corners->track_ids[i])->Point())
              .hnormalized();
      T reprojection[2];
      bool success = false;
      if (theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION ==
          cam->GetCameraIntrinsicsModelType()) {
        success =
            theia::DivisionUndistortionCameraModel::CameraToPixelCoordinates(
                intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::DOUBLE_SPHERE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::DoubleSphereCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::PINHOLE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::PinholeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::FISHEYE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::FisheyeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::ExtendedUnifiedCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      }
      if (!success) {
        sResiduals[2 * i + 0] = T(1e10);
        sResiduals[2 * i + 1] = T(1e10);
      } else {
        sResiduals[2 * i + 0] = reprojection[0] - T(corners->corners[i][0]);
        sResiduals[2 * i + 1] = reprojection[1] - T(corners->corners[i][1]);
      }
    }

    return true;
  }

  const CalibCornerData *corners;
  const theia::Camera *cam;
  const theia::Reconstruction *calib;

  double u_so3;
  double u_r3;
  double inv_so3_dt;
  double inv_r3_dt;
};

template <int _N>
struct CalibRSReprojectionCostFunctorSplit
    : public CeresSplineHelper<double, _N> {
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibRSReprojectionCostFunctorSplit(const CalibCornerData *corner_data,
                                      const theia::Reconstruction *calib,
                                      const theia::Camera *cam, double u_so3,
                                      double u_r3, double inv_so3_dt,
                                      double inv_r3_dt, double weight = 1.0)
      : corners(corner_data), calib(calib), cam(cam), u_so3(u_so3), u_r3(u_r3),
        inv_so3_dt(inv_so3_dt), inv_r3_dt(inv_r3_dt), weight(weight) {}

  template <class T>
  bool operator()(T const *const *sKnots, T *sResiduals) const {
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector4 = Eigen::Matrix<T, 4, 1>;
    using Vector1 = Eigen::Matrix<T, 1, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Eigen::Map<Vector1 const> const line_delay(sKnots[2 * N + 1]);

    T intr[10];
    for (int i = 0; i < cam->CameraIntrinsics()->NumParameters(); ++i) {
      intr[i] = T(cam->intrinsics()[i]);
    }

    for (size_t i = 0; i < corners->track_ids.size(); ++i) {
      // std::cout<<"line delay: "<<line_delay[0]<<"\n";
      const T y_coord = T(corners->corners[i][1]) * line_delay[0];
      const T t_so3_row = T(u_so3) + y_coord;
      const T t_r3_row = T(u_r3) + y_coord;
      Sophus::SO3<T> R_w_i;
      CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
          sKnots, t_so3_row, T(inv_so3_dt), &R_w_i);

      Vector3 t_w_i;
      CeresSplineHelper<T, N>::template evaluate<3, 0>(sKnots + N, t_r3_row,
                                                       T(inv_r3_dt), &t_w_i);

      Eigen::Map<Sophus::SE3<T> const> const T_i_c(sKnots[2 * N]);

      Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
      Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

      Vector3 p3d =
          (T_c_w_matrix * calib->Track(corners->track_ids[i])->Point())
              .hnormalized();
      T reprojection[2];
      bool success = false;
      if (theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION ==
          cam->GetCameraIntrinsicsModelType()) {
        success =
            theia::DivisionUndistortionCameraModel::CameraToPixelCoordinates(
                intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::DOUBLE_SPHERE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::DoubleSphereCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::PINHOLE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::PinholeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::FISHEYE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::FisheyeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::ExtendedUnifiedCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      }
      if (!success) {
        sResiduals[2 * i + 0] = T(1e10);
        sResiduals[2 * i + 1] = T(1e10);
      } else {
        sResiduals[2 * i + 0] =
            T(weight) * (reprojection[0] - T(corners->corners[i][0]));
        sResiduals[2 * i + 1] =
            T(weight) * (reprojection[1] - T(corners->corners[i][1]));
      }
    }
    return true;
  }

  const CalibCornerData *corners;
  const theia::Camera *cam;
  const theia::Reconstruction *calib;
  double u_so3;
  double u_r3;
  double inv_so3_dt;
  double inv_r3_dt;
  double weight;
};

template <int _N>
struct RSReprojectionCostFunctorSplit : public CeresSplineHelper<double, _N> {
  static constexpr int N = _N;       // Order of the spline.
  static constexpr int DEG = _N - 1; // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RSReprojectionCostFunctorSplit(const theia::View *view,
                                 const theia::Reconstruction *image_data,
                                 const theia::Camera *cam, const double u_so3,
                                 const double u_r3, const double inv_so3_dt,
                                 const double inv_r3_dt)
      : view(view), image_data(image_data), cam(cam), u_so3(u_so3), u_r3(u_r3),
        inv_so3_dt(inv_so3_dt), inv_r3_dt(inv_r3_dt) {}

  template <class T>
  bool operator()(T const *const *sKnots, T *sResiduals) const {
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector4 = Eigen::Matrix<T, 4, 1>;
    using Vector1 = Eigen::Matrix<T, 1, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Eigen::Map<Vector1 const> const line_delay(sKnots[2 * N + 1]);

    T intr[10];
    for (int i = 0; i < cam->CameraIntrinsics()->NumParameters(); ++i) {
      intr[i] = T(cam->intrinsics()[i]);
    }
    const auto track_ids = view->TrackIds();
    for (size_t i = 0; i < track_ids.size(); ++i) {
      // std::cout<<"line delay: "<<line_delay[0]<<"\n";
      const auto feature = *view->GetFeature(track_ids[i]);
      const T y_coord = T(feature.point_[1]) * line_delay[0];
      const T t_so3_row = T(u_so3) + y_coord;
      const T t_r3_row = T(u_r3) + y_coord;
      Sophus::SO3<T> R_w_i;
      CeresSplineHelper<T, N>::template evaluate_lie<Sophus::SO3>(
          sKnots, t_so3_row, T(inv_so3_dt), &R_w_i);

      Vector3 t_w_i;
      CeresSplineHelper<T, N>::template evaluate<3, 0>(sKnots + N, t_r3_row,
                                                       T(inv_r3_dt), &t_w_i);

      Eigen::Map<Sophus::SE3<T> const> const T_i_c(sKnots[2 * N]);

      Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
      Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

      Vector3 p3d = (T_c_w_matrix * image_data->Track(track_ids[i])->Point())
                        .hnormalized();
      T reprojection[2];
      bool success = false;
      if (theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION ==
          cam->GetCameraIntrinsicsModelType()) {
        success =
            theia::DivisionUndistortionCameraModel::CameraToPixelCoordinates(
                intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::DOUBLE_SPHERE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::DoubleSphereCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::PINHOLE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::PinholeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::FISHEYE ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::FisheyeCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      } else if (theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED ==
                 cam->GetCameraIntrinsicsModelType()) {
        success = theia::ExtendedUnifiedCameraModel::CameraToPixelCoordinates(
            intr, p3d.data(), reprojection);
      }
      if (!success) {
        sResiduals[2 * i + 0] = T(1e10);
        sResiduals[2 * i + 1] = T(1e10);
      } else {
        const T inv_info_x = T(1. / ceres::sqrt(feature.covariance_(0, 0)));
        const T inv_info_y = T(1. / ceres::sqrt(feature.covariance_(1, 1)));
        sResiduals[2 * i + 0] = inv_info_x * (reprojection[0] - T(feature.x()));
        sResiduals[2 * i + 1] = inv_info_y * (reprojection[1] - T(feature.y()));
      }
    }
    return true;
  }
  const theia::View *view;
  const theia::Reconstruction *image_data;
  const theia::Camera *cam;
  double u_so3;
  double inv_so3_dt;
  double u_r3;
  double inv_r3_dt;
};
