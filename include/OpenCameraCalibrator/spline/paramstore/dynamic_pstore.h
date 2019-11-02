//
// Created by hannes on 2018-01-30.
//

#ifndef ENTITY_DYNAMIC_PSTORE_H
#define ENTITY_DYNAMIC_PSTORE_H

#include <vector>

#include "OpenCameraCalibrator/spline/paramstore/paramstore.h"

namespace entity {

template <typename T>
class DynamicParameterStore : public ParameterStore<T> {
 public:
  T* ParameterData(int i) {
    return parameters_.at(i).data;
  }

  const T* ParameterData(int i) const {
    return parameters_.at(i).data;
  }

  size_t AddParameter(size_t ndims, ceres::LocalParameterization *parameterization=nullptr) override {
    size_t new_idx = parameters_.size();
    auto data = new T[ndims];
    parameters_.push_back(ParameterInfo<T>(data, ndims, parameterization));
    return new_idx;
  }

  entity::ParameterInfo<T> Parameter(size_t i) const override {
    return parameters_.at(i);
  }


  size_t Size() const override {
    return parameters_.size();
  }

  ParameterStore<T> *Slice(size_t offset, size_t length) const override {
    throw std::runtime_error("DynamicParameterStore does not support slicing!");
  }

 protected:
  std::vector<ParameterInfo<T>> parameters_;
};

} // namespace entity

#endif //ENTITY_DYNAMIC_PSTORE_H
