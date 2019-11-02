//
// Created by hannes on 2017-12-04.
//

#ifndef KONTIKIV2_NEWTON_RSCAMERA_MEASUREMENT_H
#define KONTIKIV2_NEWTON_RSCAMERA_MEASUREMENT_H

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/sfm/sfm.h"
#include "OpenCameraCalibrator/spline/trajectory_estimator.h"
#include "OpenCameraCalibrator/spline/sensors/camera.h"
#include "OpenCameraCalibrator/spline/types.h"

#include "OpenCameraCalibrator/spline/math/quaternion_math.h"

namespace kontiki {
namespace measurements {

template<
    typename TrajectoryModel,
    typename CameraModel,
    typename T>
Eigen::Matrix<T, 2, 1> reproject_newton(const sfm::Observation& ref,
                                        const sfm::Observation& obs,
                                        T inverse_depth,
                                        const type::Trajectory<TrajectoryModel, T>& trajectory,
                                        const type::Camera<CameraModel, T>& camera,
                                        const int max_iterations=5) {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using Flags = trajectories::EvaluationFlags;

  T time_offset = camera.time_offset();
  T row_delta = camera.readout() / T(camera.rows());
  T t0_obs = T(obs.view()->t0()) + time_offset;
  T t_ref = T(ref.view()->t0()) + time_offset + T(ref.v()) * row_delta; // constant value
  T t_obs = t0_obs + T(obs.v()) * row_delta; // initial value

  // We need only first derivatives
  int flags = Flags::EvalPosition | Flags::EvalVelocity | Flags::EvalOrientation | Flags::EvalAngularVelocity;

  const Vector3 p_ct = camera.relative_position();
  const Eigen::Quaternion<T> q_ct = camera.relative_orientation();

  // Reference is only evaluated once
  auto eval_ref = trajectory.Evaluate(t_ref, flags);
  Vector2 y = ref.uv().cast<T>();
  Vector3 yh = camera.Unproject(y);
  Vector3 X_ref = q_ct.conjugate() * (yh - inverse_depth * p_ct);
  Vector3 X = eval_ref->orientation * X_ref + eval_ref->position * inverse_depth;

  // Threshold for termination is a half row
  const T max_time_delta = T(0.5) * camera.readout() / T(camera.rows());
  const T max_time_delta_squared = ceres::pow(max_time_delta, 2);

  T min_bound = t0_obs;
  T max_bound = t0_obs + T(camera.readout());

  Vector2 y_out;

  // Run the Newton algorithm to get the correct projection time t_obs
  // This means that we need to differentiate the projection point with respect to t_obs.
  // Since the world landmark position X above only depends on t_ref, it is constant in the following computations.
  for (int iter=0; iter < max_iterations; ++iter) {
    auto eval_obs = trajectory.Evaluate(t_obs, flags);
    Vector3 p = eval_obs->position;
    Vector3 dp = eval_obs->velocity;
    Quaternion q = eval_obs->orientation;
    Quaternion dq = math::dq_from_angular_velocity(eval_obs->angular_velocity, q);
    Quaternion dq_inv = dq.conjugate();
    Quaternion q_inv = q.conjugate();

    Vector3 s = X - inverse_depth * p;
    Vector3 ds = -inverse_depth * dp;

    // World to trajectory  at t_obs
    Vector3 X_obs = q.conjugate() * s;

    // Trajectory to camera
    Vector3 X_obs_cam = q_ct * X_obs + inverse_depth * p_ct;

    // Derivatives
    Vector3 dX_obs = (
        math::vector_sandwich(dq_inv, s, q) +
        math::vector_sandwich(q_inv, ds, q) +
        math::vector_sandwich(q_inv, s, dq)
    );

    Vector3 dX_obs_cam = q_ct * dX_obs + inverse_depth * p_ct;

    // Project with derivatives
    auto camera_result = camera.EvaluateProjection(X_obs_cam, dX_obs_cam, true);
    y_out = camera_result->y;

    // Newton
    T v = y_out(1);
    T dv = camera_result->dy(1);
    T f = v - (T(camera.rows()) * (t_obs - t0_obs) / camera.readout());
    T df = dv - (T(camera.rows()) / camera.readout());
    T dt = f / df;

    t_obs -= dt;

    if ((dt*dt) < max_time_delta_squared) {
      break;
    }

    if (t_obs < min_bound) {
      t_obs = min_bound;
    }
    else if (t_obs > max_bound) {
      t_obs = max_bound;
    }

  } // end for

  return y_out;
}


  template<typename CameraModel>
  class NewtonRsCameraMeasurement {
    using Vector2 = Eigen::Vector2d;
   public:
    NewtonRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs, double huber_loss, double weight)
        : camera(camera),
          observation(obs),
          loss_function_(huber_loss),
          weight(weight),
          max_iterations_(5) { };

    NewtonRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs, double huber_loss)
        : NewtonRsCameraMeasurement(camera, obs, huber_loss, 1.) { };

    NewtonRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs)
        : NewtonRsCameraMeasurement(camera, obs, 5.) { };

    std::shared_ptr<CameraModel> camera;
    double weight;
    // Measurement data
    std::shared_ptr<sfm::Observation> observation;

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Project(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                   const type::Camera<CameraModel, T> &camera,
                                   const T inverse_depth) const {
      return reproject_newton<TrajectoryModel, CameraModel>(*observation->landmark()->reference(),
                                                            *observation,
                                                            inverse_depth,
                                                            trajectory,
                                                            camera,
                                                            max_iterations_);
    };

    template<typename TrajectoryModel>
    Vector2 Project(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::Camera<CameraModel, T> &camera,
                                 const T inverse_depth) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project<TrajectoryModel, T>(trajectory, camera, inverse_depth);
      return T(weight) * (observation->uv().cast<T>() - y_hat);
    }

    template<typename TrajectoryModel>
    Vector2 Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Error<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel>
    Vector2 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

   protected:

    template<typename TrajectoryModel>
    struct Residual {
      Residual(const NewtonRsCameraMeasurement<CameraModel> &m) : measurement(m) {};

      template <typename T>
      bool operator()(T const* const* params, T* residual) const {
        size_t offset = 0;
        auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
        offset += trajectory_meta.NumParameters();
        auto camera = entity::Map<CameraModel, T>(&params[offset], camera_meta);
        offset += camera_meta.NumParameters();
        T inverse_depth = params[offset][0];
        Eigen::Map<Eigen::Matrix<T,2,1>> r(residual);
        r = measurement.Error<TrajectoryModel, T>(trajectory, camera, inverse_depth);
        return true;
      }

      const NewtonRsCameraMeasurement<CameraModel> &measurement;
      typename TrajectoryModel::Meta trajectory_meta;
      typename CameraModel::Meta camera_meta;
    }; // Residual;

    template<typename TrajectoryModel>
    void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
      using ResidualImpl = Residual<TrajectoryModel>;
      auto residual = new ResidualImpl(*this);
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
      std::vector<entity::ParameterInfo<double>> parameters;

      // Find timespans for the two observations
      const auto landmark = observation->landmark();
      auto t0_ref = landmark->reference()->view()->t0();
      auto t0_obs = observation->view()->t0();

      // The list of spans must be ordered
      double t1, t2;
      if (t0_ref <= t0_obs) {
        t1 = t0_ref;
        t2 = t0_obs;
      }
      else {
        t1 = t0_obs;
        t2 = t0_ref;
      }

      // Add time offset margins, if neccessary
      if (!camera->TimeOffsetIsLocked()) {
        t1 -= camera->max_time_offset();
        t2 += camera->max_time_offset();
      }

      const double margin = 1e-3;

      estimator.AddTrajectoryForTimes({
                                          {t1 - margin, t1 + camera->readout() + margin},
                                          {t2 - margin, t2 + camera->readout() + margin}
                                      },
                                      residual->trajectory_meta,
                                      parameters);

      // Add camera to proble
      camera->AddToProblem(estimator.problem(), {
                               {t1 - margin, t1 + camera->readout() + margin},
                               {t2 - margin, t2 + camera->readout() + margin}
                           },
                           residual->camera_meta,
                           parameters);

      // Add Landmark inverse depth
      // FIXME: Landmarks could be an entity
      double *p_rho = landmark->inverse_depth_ptr();
      estimator.problem().AddParameterBlock(p_rho, 1);
      estimator.problem().SetParameterLowerBound(p_rho, 0, 0.); // Only positive inverse depths please
      parameters.push_back(entity::ParameterInfo<double>(p_rho, 1));
      if (landmark->IsLocked()) {
        estimator.problem().SetParameterBlockConstant(p_rho);
      }

      // Add parameters to cost function
      for (auto& pi : parameters) {
        cost_function->AddParameterBlock(pi.size);
      }

      // Add measurement info
      cost_function->SetNumResiduals(2);

      // Give residual block to Problem
      estimator.problem().AddResidualBlock(cost_function,
                                           &loss_function_,
                                           entity::ParameterInfo<double>::ToParameterBlocks(parameters));
    }

    // The loss function is not a pointer since the Problem does not take ownership.
    ceres::HuberLoss loss_function_;
    int max_iterations_;

    template<template<typename> typename TrajectoryModel>
    friend class kontiki::TrajectoryEstimator;
  };

} // namespace measurements
} // namespace kontiki

#endif //KONTIKIV2_NEWTON_RSCAMERA_MEASUREMENT_H
