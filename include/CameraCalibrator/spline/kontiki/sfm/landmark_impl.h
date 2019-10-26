//
// Created by hannes on 2018-02-16.
//

#ifndef KONTIKIV2_LANDMARK_IMPL_H
#define KONTIKIV2_LANDMARK_IMPL_H

#include "landmark.h"

namespace kontiki {
namespace sfm {

Landmark::Landmark() :
    inverse_depth_(0),
    locked_(false),
    id_(new_id()) {}

size_t Landmark::id() const {
  return id_;
}

void Landmark::set_reference(std::shared_ptr<Observation> new_ref) {
  if (new_ref->landmark().get() != this)
    throw std::runtime_error("Observation not in observations list");
  else
    reference_observation_ = new_ref;
}

std::shared_ptr<Observation> Landmark::reference() const {
  auto sp = reference_observation_.lock();
  if (sp)
    return sp;
  else {
    std::stringstream ss;
    ss << "Landmark id=" << id() << ": Failed to get reference observation!";
    throw std::runtime_error(ss.str());
  }
}

std::vector<std::shared_ptr<Observation>> Landmark::observations() const {
  std::vector<std::shared_ptr<Observation>> obslist;
  for (auto wp : observations_) {
    auto sp = wp.lock();
    if (sp)
      obslist.push_back(sp);
    else
      throw std::runtime_error("Observation removed without Landmark knowing about it!");
  }
  return obslist;
}

double Landmark::inverse_depth() const {
  return inverse_depth_;
}

void Landmark::set_inverse_depth(double x) {
  inverse_depth_ = x;
}

double* Landmark::inverse_depth_ptr() {
  return &inverse_depth_;
}

bool Landmark::IsLocked() const {
  return locked_;
}

void Landmark::Lock(bool flag) {
  locked_ = flag;
}


void Landmark::AddObservation(std::shared_ptr<Observation> obs) {
  observations_.push_back(obs);
}

void Landmark::RemoveObservation(std::shared_ptr<Observation> obs) {
  auto found_it = std::find_if(observations_.begin(), observations_.end(), [&obs](auto &wp) {
    auto sp = wp.lock();
    return sp && sp == obs;
  });

  if (found_it != observations_.end()) {
    observations_.erase(found_it);
  }
  else {
    throw std::runtime_error("Landmark: Observation can not be removed as it is not ours");
  }
}

} // namespace sfm
} // namespace kontiki



#endif //KONTIKIV2_LANDMARK_IMPL_H
