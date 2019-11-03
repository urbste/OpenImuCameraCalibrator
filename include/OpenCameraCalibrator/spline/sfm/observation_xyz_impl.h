#pragma once

#include "OpenCameraCalibrator/spline/sfm/observation_xyz.h"

namespace kontiki {
namespace sfm {

ObservationXYZ::ObservationXYZ(const Eigen::Vector2d &uv,
                               std::shared_ptr<LandmarkXYZ> landmark,
                               std::shared_ptr<ViewXYZ> view)
    : uv_(uv), landmark_(landmark), view_(view){};

std::shared_ptr<LandmarkXYZ> ObservationXYZ::landmark() const {
  return landmark_;
};

std::shared_ptr<ViewXYZ> ObservationXYZ::view() const {
  auto sp = view_.lock();
  if (sp)
    return sp;
  else
    throw std::runtime_error("View does not exist anymore");
}

bool ObservationXYZ::IsReference() const {
  return landmark_->reference().get() == this;
}

Eigen::Vector2d ObservationXYZ::uv() const { return uv_; }

void ObservationXYZ::set_uv(const Eigen::Vector2d &uv) { uv_ = uv; }

double ObservationXYZ::u() const { return uv_(0); }

double ObservationXYZ::v() const { return uv_(1); }

}  // namespace sfm
}  // namespace kontiki

