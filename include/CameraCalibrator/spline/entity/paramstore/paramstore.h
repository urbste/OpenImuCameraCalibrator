//
// Created by hannes on 2018-01-30.
//

#ifndef ENTITY_PARAMSTORE_H
#define ENTITY_PARAMSTORE_H

#include <ceres/ceres.h>

namespace entity {

template<typename T>
struct ParameterInfo {
  ParameterInfo(T* data, size_t size, ceres::LocalParameterization* parameterization) :
    data(data), size(size), parameterization(parameterization) { };

  ParameterInfo(T *data, size_t size) :
    ParameterInfo(data, size, nullptr) { };

  static std::vector<T*> ToParameterBlocks(const std::vector<ParameterInfo<T>> &infolist) {
    std::vector<T*> param_blocks;

    for (auto& pi : infolist)
      param_blocks.push_back(pi.data);

    return param_blocks;
  }


  T* data;     // Pointer to data
  size_t size;      // Size of data
  ceres::LocalParameterization* parameterization;
};

template<typename T>
struct ParameterStore {
  virtual T* ParameterData(int i) = 0;
  virtual const T* ParameterData(int i) const = 0;
  virtual size_t AddParameter(size_t ndims, ceres::LocalParameterization* parameterization=nullptr) = 0;
  virtual size_t Size() const = 0;
  virtual ParameterStore<T>* Slice(size_t offset, size_t length) const = 0;
  virtual ParameterInfo<T> Parameter(size_t i) const = 0;
};

} // namespace entity

#endif //ENTITY_PARAMSTORE_H
