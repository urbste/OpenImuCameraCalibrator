//
// Created by hannes on 2018-01-30.
//

#ifndef ENTITY_EMPTY_PSTORE_H
#define ENTITY_EMPTY_PSTORE_H

#include "OpenCameraCalibrator/spline/paramstore/paramstore.h"

namespace entity {

template<typename T>
class EmptyParameterStore : public ParameterStore<T> {
 public:
  T *ParameterData(int i) override {
    throw std::runtime_error("Not supported by EmptyParameterStore");
  }
  const T *ParameterData(int i) const override {
    throw std::runtime_error("Not supported by EmptyParameterStore");
  }

  size_t AddParameter(size_t ndims, ceres::LocalParameterization *parameterization) override {
    throw std::runtime_error("Not supported by EmptyParameterStore");
  }
  ParameterInfo<T> Parameter(size_t i) const override {
    throw std::runtime_error("Not supported by EmptyParameterStore");
  }

  size_t Size() const override {
    return 0;
  }

  ParameterStore<T>* Slice(size_t offset, size_t length) const override {
    return new EmptyParameterStore<T>();
  }
};

} // namespace entity

#endif //ENTITY_EMPTY_PSTORE_H
