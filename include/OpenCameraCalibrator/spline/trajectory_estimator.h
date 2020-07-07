//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_TRAJECTORY_ESTIMATOR_H
#define KONTIKIV2_TRAJECTORY_ESTIMATOR_H

#include <iostream>
#include <vector>
#include <memory>
#include <thread>

#include <ceres/ceres.h>

#include "OpenCameraCalibrator/spline/trajectories/trajectory.h"

namespace kontiki {

template <typename TrajectoryModel>
class TrajectoryEstimator {
  using Meta = typename TrajectoryModel::Meta;
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }
 public:
  TrajectoryEstimator(std::shared_ptr<TrajectoryModel> trajectory) :
      trajectory_(trajectory),
      problem_(DefaultProblemOptions()),
      callback_needs_state_(false) { };

  auto trajectory() const {
    return trajectory_;
  }

  ceres::Solver::Summary Solve(int max_iterations=50, bool progress=true, int num_threads=-1) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = progress;

    if (num_threads < 1) {
      num_threads = std::thread::hardware_concurrency();
    }

    options.num_linear_solver_threads = num_threads;
    options.num_threads = num_threads;

    options.max_num_iterations = max_iterations;

    if (callbacks_.size() > 0) {
      for (auto &cb : callbacks_) {
        options.callbacks.push_back(cb.get());
      }

      if (callback_needs_state_)
        options.update_state_every_iteration = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    return summary;
  }

  template<typename MeasurementType>
  void AddMeasurement(std::shared_ptr<MeasurementType> m) {
    m->AddToEstimator(*this);
  }

  ceres::Problem& problem() {
    return problem_;
  }

  bool AddTrajectoryForTimes(const time_init_t &times,
                             Meta &meta,
                             std::vector<entity::ParameterInfo<double>> &parameter_info) {
    CheckTimeSpans(times);
    trajectory_->AddToProblem(problem_, times, meta, parameter_info);
    return true;
  }

  void AddCallback(std::unique_ptr<ceres::IterationCallback> callback, bool needs_state=false) {
    callbacks_.push_back(std::move(callback));

    // If any callback requires state, the flag must be set
    if (!callback_needs_state_)
      callback_needs_state_ = needs_state;
  }

 protected:

  // Check the following
  // 1) All time spans are valid for the current trajectory
  // 2) Time spans are ordered (ascending)
  // Throw std::range_error on failure
  void CheckTimeSpans(const time_init_t &times) {
    int i=0;
    double t1_prev;

    for (auto &tspan : times) {
      double t1, t2;
      std::tie(t1, t2) = tspan;

      // Valid for trajectory?
      if ((t1 < trajectory_->MinTime()) ||
          (t2 >= trajectory_->MaxTime())) {
          std::cout<<"traj min time: "<<trajectory_->MinTime()<< " traj max time: "<<trajectory_->MaxTime()<<std::endl;
          std::cout<<"t1: "<<t1<<" t2: "<<t2<<std::endl;
        throw std::range_error("Time span out of range for trajectory");
      }

      // Ordered?
      if (t1 > t2) {
        throw std::range_error("At least one time span begins before it ends");
      }
      else if((i > 0) && (t1 < t1_prev)) {
        throw std::range_error("Time spans are not ordered");
      }

      t1_prev = t1;
      i += 1;
    }
  }


  // Data members
  std::shared_ptr<TrajectoryModel> trajectory_;
  ceres::Problem problem_;
  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;
  bool callback_needs_state_;
};

} // namespace kontiki
#endif //KONTIKIV2_TRAJECTORY_ESTIMATOR_H
