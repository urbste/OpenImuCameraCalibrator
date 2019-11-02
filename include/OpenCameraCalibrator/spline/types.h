//
// Created by hannes on 2018-01-30.
//

#ifndef KONTIKIV2_TYPES_H
#define KONTIKIV2_TYPES_H

#include "OpenCameraCalibrator/spline/entity.h"
#include <ceres/ceres.h>

namespace kontiki {

// Define time types
using time_span_t = std::pair<double, double>;
using time_init_t = std::initializer_list<time_span_t>;

namespace type {

using entity::type::View;

// All optimizable objects should inherit the Entity type
template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class Entity : public entity::Entity<ViewTemplate, MetaType, StoreType> {
 public:
  // Add this type to an optimization problem
  virtual void AddToProblem(ceres::Problem &problem,
                            time_init_t times,
                            MetaType& meta,
                            std::vector<entity::ParameterInfo<double>> &parameters) const = 0;

};

} // namespace type
} // namespace kontiki

#endif //KONTIKIV2_TYPES_H
