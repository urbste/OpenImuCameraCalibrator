#pragma once
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
Eigen::Matrix<T, 2, 1> reproject_static(const sfm::ObservationXYZ& obs,
                                        const Eigen::Matrix<T, 4, 1> landmark,
                                        const type::Trajectory<TrajectoryModel, T>& trajectory,
                                        const type::Camera<CameraModel, T>& camera) {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Flags = trajectories::EvaluationFlags;

  T time_offset = camera.time_offset();
  T row_delta = camera.readout() / T(camera.rows());
  T t_obs = obs.view()->t0() + time_offset + obs.v() * row_delta;

  int flags = Flags::EvalPosition | Flags::EvalOrientation;
  auto eval_obs = trajectory.Evaluate(t_obs, flags);

  // FIXME: We have a ToTrajectory/FromTrajectory function
  const Vector3 p_ct = camera.relative_position();
  const Eigen::Quaternion<T> q_ct = camera.relative_orientation();

  Vector3 landmark_world = landmark.hnormalized();
  Vector3 X_in_spline = eval_obs->orientation.conjugate() * (landmark_world -  eval_obs->position);
  Vector3 X_camera = q_ct * X_in_spline + p_ct;

  return camera.Project(X_camera);
}


  template<typename CameraModel>
  class StaticRsCameraMeasurementXYZ {
    using Vector2 = Eigen::Vector2d;
   public:
    StaticRsCameraMeasurementXYZ(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::ObservationXYZ> obs, double huber_loss, double weight)
        : camera(camera), observation(obs), loss_function_(huber_loss), weight(weight) {};

    StaticRsCameraMeasurementXYZ(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::ObservationXYZ> obs, double huber_loss)
        : StaticRsCameraMeasurementXYZ(camera, obs, huber_loss, 1.0) { };

    StaticRsCameraMeasurementXYZ(std::shared_ptr<CameraModel> camera, std::shared_ptr<sfm::ObservationXYZ> obs)
        : StaticRsCameraMeasurementXYZ(camera, obs, 5.) { };

    std::shared_ptr<CameraModel> camera;
    // Measurement data
    std::shared_ptr<sfm::ObservationXYZ> observation;
    double weight;

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Project(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                   const type::Camera<CameraModel, T> &camera,
                                   const Eigen::Matrix<T, 4, 1> landmark_xyz) const {
      return reproject_static<TrajectoryModel, CameraModel>(*observation, landmark_xyz, trajectory, camera);
    };

    template<typename TrajectoryModel>
    Vector2 Project(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->get_point());
    };

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::Camera<CameraModel, T> &camera,
                                 const Eigen::Matrix<T, 4, 1>& landmark_xyz) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project<TrajectoryModel, T>(trajectory, camera, landmark_xyz);
      return T(weight) * (observation->uv().cast<T>() - y_hat);
    }

    template<typename TrajectoryModel>
    Vector2 Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Error<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->get_point());
    };

    template<typename TrajectoryModel>
    Vector2 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->get_point());
    };

   protected:

    template<typename TrajectoryModel>
    struct Residual {
      Residual(const StaticRsCameraMeasurementXYZ<CameraModel> &m) : measurement(m) {};

      template <typename T>
      bool operator()(T const* const* params, T* residual) const {
        size_t offset = 0;
        auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
        offset += trajectory_meta.NumParameters();
        auto camera = entity::Map<CameraModel, T>(&params[offset], camera_meta);
        offset += camera_meta.NumParameters();
        //T inverse_depth = params[offset][0];
        Eigen::Matrix<T, 4, 1> landmark_xyz;
        landmark_xyz(0,0) = params[offset][0];
        landmark_xyz(1,0) = params[offset][1];
        landmark_xyz(2,0) = params[offset][2];
        landmark_xyz(3,0) = T(1.0);
        Eigen::Map<Eigen::Matrix<T,2,1>> r(residual);
        r = measurement.Error<TrajectoryModel, T>(trajectory, camera, landmark_xyz);
        return true;
      }

      const StaticRsCameraMeasurementXYZ<CameraModel> &measurement;
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
      double *p_rho = landmark->xyz_ptr();
      estimator.problem().AddParameterBlock(p_rho, 4);
      //estimator.problem().SetParameterLowerBound(p_rho, 0, 0.); // Only positive inverse depths please
      parameters.push_back(entity::ParameterInfo<double>(p_rho, 4));
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

    template<template<typename> typename TrajectoryModel>
    friend class kontiki::TrajectoryEstimator;
  };

} // namespace measurements
} // namespace kontiki
