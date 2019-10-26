#ifndef ENTITY_ENTITY_H
#define ENTITY_ENTITY_H

#include "paramstore/paramstore.h"
#include "paramstore/pointer_pstore.h" // Used by Map()

namespace entity {

// Each entity type should define its own metadata struct which
// paramstore any data that is not part of the optimization problem.
struct MetaData {
  virtual size_t NumParameters() const = 0;
};

// Views define methods and member accessors.
// The data can come from either the metadata object, or a parameter from the dataholder.
// The View owns its metadata data, but not necessarily the data in the data holder.
// Views can therefore never create new parameters! This must be done by the Entity.
template<typename T, typename MetaType>
class EntityView {
 public:
  EntityView(const MetaType& meta, ParameterStore<T> *holder) :
      meta_(meta), pstore_(holder) { };

 protected:
  std::unique_ptr<ParameterStore<T>> pstore_;
  MetaType meta_;
};


// The Entity is a concrete implementation of a View.
// It know how to construct the object, and how to add it to an optimization problem.
template<template<typename...> typename ViewTemplate,
    typename MetaType,
    typename StoreType>
class Entity : public ViewTemplate<double, MetaType> {
  using Base = ViewTemplate<double, MetaType>;
 public:
  // Default constructor
  Entity() : Base(MetaType(), new StoreType()) { };

  // Copy constructor
  Entity(const Entity& rhs) :
      Base(rhs.meta_, new StoreType()) {
    // Copy parameters
    for (int i=0; i < rhs.pstore_->Size(); ++i) {
      auto pi = rhs.pstore_->Parameter(i);
      this->pstore_->AddParameter(pi.size, pi.parameterization);
      memcpy(this->pstore_->ParameterData(i), pi.data, pi.size*sizeof(double));
    }
  }

  // Access view
  template<typename T, typename MetaT=MetaType>
  using View = ViewTemplate<T, MetaT>;

  using Meta = MetaType;
  using Holder = StoreType;
};


namespace type {
// The view type of the entity class
template<typename _Entity, typename T>
using View = typename _Entity::template View<T>;

namespace base {
// Type that accepts an entity/view if it is a subclass the base view
template<typename _Entity, template<typename...> typename ViewTemplate, typename T>
using ForView = ViewTemplate<T, typename _Entity::Meta>;

// Type that accepts an entity/view if it is a subclass the base entity
template<typename _Entity, typename Base, typename T>
using ForEntity = typename Base::template View<T, typename _Entity::Meta>;
} // namespace base
} // namespace type


// Map raw data and metadata to an entity view
template<typename _Entity, typename T>
static type::View<_Entity, T> Map(T const* const* params, const typename _Entity::Meta& meta) {
  return type::View<_Entity, T>(meta, new PointerParameterStore<T>(params));
};

} // namespace entity

#endif //ENTITY_ENTITY_H
