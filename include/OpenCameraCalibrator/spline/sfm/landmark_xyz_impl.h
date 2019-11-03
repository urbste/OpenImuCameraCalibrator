// created by Steffen Urban (urbste@googlemail.com)

#pragma once

#include "OpenCameraCalibrator/spline/sfm/landmark_xyz.h"

namespace kontiki {
namespace sfm {

LandmarkXYZ::LandmarkXYZ() :
    locked_(false),
    id_(new_id()) 
{    xyz_.setZero(); }

size_t LandmarkXYZ::id() const {
  return id_;
}

void LandmarkXYZ::set_reference(std::shared_ptr<ObservationXYZ> new_ref) {
  if (new_ref->landmark().get() != this)
    throw std::runtime_error("Observation not in observations list");
  else
    reference_observation_ = new_ref;
}

std::shared_ptr<ObservationXYZ> LandmarkXYZ::reference() const {
  auto sp = reference_observation_.lock();
  if (sp)
    return sp;
  else {
    std::stringstream ss;
    ss << "Landmark id=" << id() << ": Failed to get reference observation!";
    throw std::runtime_error(ss.str());
  }
}

std::vector<std::shared_ptr<ObservationXYZ>> LandmarkXYZ::observations() const {
  std::vector<std::shared_ptr<ObservationXYZ>> obslist;
  for (auto wp : observations_) {
    auto sp = wp.lock();
    if (sp)
      obslist.push_back(sp);
    else
      throw std::runtime_error("Observation removed without Landmark knowing about it!");
  }
  return obslist;
}

Eigen::Vector4d LandmarkXYZ::get_point() const {
  return xyz_;
}

void LandmarkXYZ::set_point(const Eigen::Vector4d& pt) {
  xyz_ = pt;
}

double* LandmarkXYZ::xyz_ptr() {
    return nullptr;
  //return xyz_.data;
}

bool LandmarkXYZ::IsLocked() const {
  return locked_;
}

void LandmarkXYZ::Lock(bool flag) {
  locked_ = flag;
}


void LandmarkXYZ::AddObservation(std::shared_ptr<ObservationXYZ> obs) {
  observations_.push_back(obs);
}

void LandmarkXYZ::RemoveObservation(std::shared_ptr<ObservationXYZ> obs) {
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




