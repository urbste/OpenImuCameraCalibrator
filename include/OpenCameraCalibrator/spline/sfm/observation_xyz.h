#pragma once

namespace kontiki {
namespace sfm {
class LandmarkXYZ; // Forward declaration
class View; // Forward declaration

class ObservationXYZ {
 public:
  ObservationXYZ(const Eigen::Vector2d &uv, std::shared_ptr<LandmarkXYZ> landmark, std::shared_ptr<ViewXYZ> view);

  std::shared_ptr<LandmarkXYZ> landmark() const;

  std::shared_ptr<ViewXYZ> view() const;

  bool IsReference() const;

  Eigen::Vector2d uv() const;

  void set_uv(const Eigen::Vector2d& uv);

  double u() const;

  double v() const;

 protected:
  Eigen::Vector2d uv_;
  std::shared_ptr<LandmarkXYZ> landmark_;
  std::weak_ptr<ViewXYZ> view_;
};

} // namespace sfm
} // namespace kontiki

#endif //KONTIKI_OBSERVATION_H
