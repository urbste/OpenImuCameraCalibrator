#pragma once

#include "assert.h"
#include "ceres_local_param.h"
#include "eigen_utils.h"

#include <ceres/ceres.h>"
#include "ceres_lie_residuals.h"

template <int _N, template <class> class GroupT, bool OLD_TIME_DERIV = false>
class CeresLieGroupSpline {
 public:
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  static constexpr double ns_to_s = 1e-9;  ///< Nanosecond to second conversion
  static constexpr double s_to_ns = 1e9;   ///< Second to nanosecond conversion

  using Groupd = GroupT<double>;
  using Tangentd = typename GroupT<double>::Tangent;
  using Transformationd = typename GroupT<double>::Transformation;

  CeresLieGroupSpline(int64_t time_interval_ns, int64_t start_time_ns = 0)
      : dt_ns(time_interval_ns), start_t_ns(start_time_ns) {
    inv_dt = s_to_ns / dt_ns;
  };

  void init(const Groupd& init, int num_knots) {
    knots = Eigen::aligned_vector<Groupd>(num_knots, init);

    for (int i = 0; i < num_knots; i++) {
      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Groupd>();

      problem.AddParameterBlock(knots[i].data(), Groupd::num_parameters,
                                local_parameterization);
    }
  }

  void initRandom(int num_knots) {
    knots = Eigen::aligned_vector<Groupd>(num_knots);

    for (int i = 0; i < num_knots; i++) {
      knots[i] = Groupd::exp(Tangentd::Random());

      ceres::LocalParameterization* local_parameterization =
          new LieLocalParameterization<Groupd>();

      problem.AddParameterBlock(knots[i].data(), Groupd::num_parameters,
                                local_parameterization);
    }
  }

  void addMeasurement(const Groupd& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT = LieGroupSplineValueCostFunctor<N, GroupT>;
    FunctorT* functor = new FunctorT(meas, u);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(Groupd::num_parameters);
    }
    cost_function->SetNumResiduals(Groupd::DoF);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    problem.AddResidualBlock(cost_function, NULL, vec);
  }

  void addVelMeasurement(const Tangentd& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT =
        LieGroupSplineVelocityCostFunctor<N, GroupT, OLD_TIME_DERIV>;

    FunctorT* functor = new FunctorT(meas, u, inv_dt);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(Groupd::num_parameters);
    }
    cost_function->SetNumResiduals(Groupd::DoF);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    problem.AddResidualBlock(cost_function, NULL, vec);

    //    {
    //      Tangentd residual;
    //      cost_function->Evaluate(&vec[0], residual.data(), NULL);
    //      std::cerr << "residual " << residual.transpose() << std::endl;
    //    }
  }

  void addAccelMeasurement(const Tangentd& meas, int64_t time_ns) {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    using FunctorT =
        LieGroupSplineAccelerationCostFunctor<N, GroupT, OLD_TIME_DERIV>;

    FunctorT* functor = new FunctorT(meas, u, inv_dt);

    ceres::DynamicAutoDiffCostFunction<FunctorT>* cost_function =
        new ceres::DynamicAutoDiffCostFunction<FunctorT>(functor);

    for (int i = 0; i < N; i++) {
      cost_function->AddParameterBlock(Groupd::num_parameters);
    }
    cost_function->SetNumResiduals(Groupd::DoF);

    std::vector<double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    problem.AddResidualBlock(cost_function, NULL, vec);

    //    {
    //      Tangentd residual;
    //      cost_function->Evaluate(&vec[0], residual.data(), NULL);
    //      std::cerr << "residual " << residual.transpose() << std::endl;
    //    }
  }

  Groupd getValue(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    Groupd res;
    CeresSplineHelper<N>::template evaluate_lie<double, GroupT>(&vec[0], u,
                                                                inv_dt, &res);

    return res;
  }

  Tangentd getVel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    Tangentd res;
    CeresSplineHelper<N>::template evaluate_lie<double, GroupT>(
        &vec[0], u, inv_dt, nullptr, &res);

    return res;
  }

  Tangentd getAccel(int64_t time_ns) const {
    int64_t st_ns = (time_ns - start_t_ns);

    BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << time_ns
                                              << " start_t_ns " << start_t_ns);

    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(), "s " << s << " N " << N
                                                             << " knots.size() "
                                                             << knots.size());

    std::vector<const double*> vec;
    for (int i = 0; i < N; i++) {
      vec.emplace_back(knots[s + i].data());
    }

    Tangentd res, tmp;
    CeresSplineHelper<N>::template evaluate_lie<double, GroupT>(
        &vec[0], u, inv_dt, nullptr, &tmp, &res);

    return res;
  }

  int64_t maxTimeNs() const {
    return start_t_ns + (knots.size() - N + 1) * dt_ns - 1;
  }

  int64_t minTimeNs() const { return start_t_ns; }

  ceres::Solver::Summary optimize() {
    ceres::Solver::Options options;
    options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 200;
    options.num_threads = 1;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return summary;
  }

  const Groupd& getKnot(int i) const { return knots[i]; }
  Groupd& getKnot(int i) { return knots[i]; }

 private:
  int64_t dt_ns, start_t_ns;
  double inv_dt;

  Eigen::aligned_vector<Groupd> knots;

  ceres::Problem problem;
};
