//
// Created by hannes on 2017-12-04.
//

#ifndef KONTIKIV2_LIFTING_RSCAMERA_MEASUREMENT_H
#define KONTIKIV2_LIFTING_RSCAMERA_MEASUREMENT_H

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"
#include "OpenCameraCalibrator/spline/sfm/sfm.h"
#include "OpenCameraCalibrator/spline/trajectory_estimator.h"
#include "OpenCameraCalibrator/spline/sensors/camera.h"
#include "OpenCameraCalibrator/spline/types.h"

namespace kontiki {
namespace measurements {

template<
    typename TrajectoryModel,
    typename CameraModel,
    typename T>
Eigen::Matrix<T, 2, 1> reproject_lifting(const sfm::Observation& ref,
                                        const sfm::Observation& obs,
                                        T vt,  // [0-1]: frame normalized time
                                        T inverse_depth,
                                        const type::Trajectory<TrajectoryModel, T>& trajectory,
                                        const type::Camera<CameraModel, T>& camera) {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Flags = trajectories::EvaluationFlags;

  T time_offset = camera.time_offset();
  T row_delta = camera.readout() / T(camera.rows());
  T t_ref = ref.view()->t0() + time_offset + ref.v() * row_delta;
  T t_obs = obs.view()->t0() + time_offset + vt * camera.readout();

  int flags = Flags::EvalPosition | Flags::EvalOrientation;
  auto eval_ref = trajectory.Evaluate(t_ref, flags);
  auto eval_obs = trajectory.Evaluate(t_obs, flags);

  // FIXME: We have a ToTrajectory/FromTrajectory function
  const Vector3 p_ct = camera.relative_position();
  const Eigen::Quaternion<T> q_ct = camera.relative_orientation();

  Vector2 y = ref.uv().cast<T>();
  Vector3 yh = camera.Unproject(y);

//  std::cout << "y=" << y.transpose() << "\nyh=" << yh.transpose() << std::endl;

  // fixme: It might be faster to make this a single expression (Eigen optimizations)
  Vector3 X_ref = q_ct.conjugate() * (yh - inverse_depth * p_ct);
  Vector3 X = eval_ref->orientation * X_ref + eval_ref->position * inverse_depth;
  Vector3 X_obs = eval_obs->orientation.conjugate() * (X - inverse_depth * eval_obs->position);
  Vector3 X_camera = q_ct * X_obs + p_ct * inverse_depth;

  return camera.Project(X_camera);
}


  template<typename CameraModel>
  class LiftingRsCameraMeasurement {
    using Vector2 = Eigen::Vector2d;
    using Vector3 = Eigen::Vector3d;
   public:
    LiftingRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs, double huber_loss, double weight)
        : camera(camera),
          observation(obs),
          loss_function_(huber_loss),
          weight(weight),
          vt_orig_(observation->v() / camera->rows()){
      vt_ = vt_orig_;
    };

    LiftingRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs, double huber_loss)
        : LiftingRsCameraMeasurement(camera, obs, huber_loss, 1.) { };

    LiftingRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::Observation> obs)
        : LiftingRsCameraMeasurement(camera, obs, 5.) { };

    std::shared_ptr<CameraModel> camera;
    // Measurement data
    double vt_;
    std::shared_ptr<sfm::Observation> observation;
    double weight;

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Project(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                   const type::Camera<CameraModel, T> &camera,
                                   const T vt,
                                   const T inverse_depth) const {
      return reproject_lifting<TrajectoryModel, CameraModel>(*observation->landmark()->reference(), *observation, vt, inverse_depth, trajectory, camera);
    };

    template<typename TrajectoryModel>
    Vector2 Project(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, vt_, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 3, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::Camera<CameraModel, T> &camera,
                                 const T vt,
                                 const T inverse_depth) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project<TrajectoryModel, T>(trajectory, camera, vt, inverse_depth);
      Eigen::Matrix<T, 3, 1> e;

      // Projection error (in pixels)
      e.head(2) = observation->uv().cast<T>() - y_hat;

      // Timing error (in pixels)
      e(2) = T(camera.rows()) * (vt - T(vt_orig_));

      return T(weight) * e;
    }

    template<typename TrajectoryModel>
    Vector3 Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Error<TrajectoryModel, double>(trajectory, *camera, vt_, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel>
    Vector2 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, vt_, observation->landmark()->inverse_depth());
    };

   protected:

    template<typename TrajectoryModel>
    struct Residual {
      Residual(const LiftingRsCameraMeasurement<CameraModel> &m) : measurement(m) {};

      template <typename T>
      bool operator()(T const* const* params, T* residual) const {
        size_t offset = 0;
        auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
        offset += trajectory_meta.NumParameters();
        auto camera = entity::Map<CameraModel, T>(&params[offset], camera_meta);
        offset += camera_meta.NumParameters();
        T vt = params[offset][0];
        offset += 1;
        T inverse_depth = params[offset][0];
        Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
        r = measurement.Error<TrajectoryModel, T>(trajectory, camera, vt, inverse_depth);
        return true;
      }

      const LiftingRsCameraMeasurement<CameraModel> &measurement;
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

      // Add observation normalized time
      double *p_vt = &vt_;
      estimator.problem().AddParameterBlock(p_vt, 1);
      // Set vt to be in [0, 1]
      estimator.problem().SetParameterLowerBound(p_vt, 0, 0.);
      estimator.problem().SetParameterUpperBound(p_vt, 0, 1.);
      parameters.push_back(entity::ParameterInfo<double>(p_vt, 1));

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
      cost_function->SetNumResiduals(3);

      // Give residual block to Problem
      estimator.problem().AddResidualBlock(cost_function,
                                           &loss_function_,
                                           entity::ParameterInfo<double>::ToParameterBlocks(parameters));
    }

    // The loss function is not a pointer since the Problem does not take ownership.
    ceres::HuberLoss loss_function_;
    const double vt_orig_;

    template<template<typename> typename TrajectoryModel>
    friend class kontiki::TrajectoryEstimator;
  };

} // namespace measurements
} // namespace kontiki

#endif //KONTIKIV2_STATIC_RSCAMERA_MEASUREMENT_H
