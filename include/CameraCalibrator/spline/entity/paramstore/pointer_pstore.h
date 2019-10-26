//
// Created by hannes on 2018-01-30.
//

#ifndef ENTITY_POINTER_PSTORE_H
#define ENTITY_POINTER_PSTORE_H

#include "paramstore.h"

namespace entity {

template<typename T>
struct PointerParameterStore : public ParameterStore<T> {
  PointerParameterStore(T const* const* params) :
      params(params) { }

  T *ParameterData(int i) override {
    T* ptr = (T*) params[i];
    return ptr;
  }

  const T *ParameterData(int i) const override {
    const T* ptr = params[i];
    return ptr;
  }

  size_t AddParameter(size_t ndims, ceres::LocalParameterization *parameterization) override {
    throw std::runtime_error("PointerParameterStore can not add parameters");
  }

  ParameterInfo<T> Parameter(size_t i) const override {
    throw std::runtime_error("PointerParameterStore can not return ParameterInfo data");
  }


  ParameterStore<T>* Slice(size_t offset, size_t length) const {
    return new PointerParameterStore<T>(&params[offset]);
  }

  size_t Size() const override {
    throw std::runtime_error("PointerParameterStore does not know its size");
  }

 private:
  T const* const* params;
};

} // namespace entity

#endif //ENTITY_POINTER_PSTORE_H
