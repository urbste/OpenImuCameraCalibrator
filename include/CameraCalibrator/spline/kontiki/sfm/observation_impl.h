//
// Created by hannes on 2018-02-16.
//

#ifndef KONTIKIV2_OBSERVATION_IMPL_H
#define KONTIKIV2_OBSERVATION_IMPL_H

#include "observation.h"

namespace kontiki {
namespace sfm {

Observation::Observation(const Eigen::Vector2d &uv, std::shared_ptr<Landmark> landmark, std::shared_ptr<View> view) :
    uv_(uv),
    landmark_(landmark),
    view_(view) {};

std::shared_ptr<Landmark> Observation::landmark() const {
  return landmark_;
};

std::shared_ptr<View> Observation::view() const {
  auto sp = view_.lock();
  if (sp)
    return sp;
  else
    throw std::runtime_error("View does not exist anymore");
}

bool Observation::IsReference() const {
  return landmark_->reference().get()==this;
}

Eigen::Vector2d Observation::uv() const {
  return uv_;
}

void Observation::set_uv(const Eigen::Vector2d &uv) {
  uv_ = uv;
}

double Observation::u() const {
  return uv_(0);
}

double Observation::v() const {
  return uv_(1);
}

} // namespace sfm
} // namespace kontiki

#endif //KONTIKIV2_OBSERVATION_IMPL_H
