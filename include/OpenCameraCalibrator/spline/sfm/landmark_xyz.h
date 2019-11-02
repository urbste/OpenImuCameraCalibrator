// created by Steffen Urban (urbste@googlemail.com)

#pragma once
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace kontiki {
namespace sfm {

class ObservationXYZ;
class ViewXYZ;

class LandmarkXYZ {
  friend ViewXYZ;

  static size_t new_id() {
    static size_t next_id = -1;
    ++next_id;
    return next_id;
  }

 public:
  LandmarkXYZ();

  size_t id() const;

  void set_reference(std::shared_ptr<ObservationXYZ> new_ref);
  std::shared_ptr<ObservationXYZ> reference() const;

  std::vector<std::shared_ptr<ObservationXYZ>> observations() const;

  Eigen::Vector4d get_point() const;
  void set_point(const Eigen::Vector3d& pt);
  double* xyz_ptr();

  bool IsLocked() const;
  void Lock(bool flag);

 protected:
  void AddObservation(std::shared_ptr<ObservationXYZ> obs);
  void RemoveObservation(std::shared_ptr<ObservationXYZ> obs);

  size_t id_;
  Eigen::Vector4d xyz_;
  std::weak_ptr<ObservationXYZ> reference_observation_;
  std::vector<std::weak_ptr<ObservationXYZ>> observations_;
  bool locked_;
};

} // namespace sfm
} // namespace kontiki

