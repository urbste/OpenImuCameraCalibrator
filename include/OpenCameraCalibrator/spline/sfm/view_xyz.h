#pragma once
#include <vector>
#include <memory>

namespace kontiki {
namespace sfm {

class ObservationXYZ;
class LandmarkXYZ;

class View : public std::enable_shared_from_this<View> {
 public:
  View(size_t frame, double t0);
  ~View();

  size_t frame_nr() const;
  void set_frame_nr(size_t fnr);
  double t0() const;
  void set_t0(double t0);

  std::vector<std::shared_ptr<ObservationXYZ>> observations() const;
  std::shared_ptr<Observation> CreateObservation(std::shared_ptr<LandmarkXYZ> landmark, const Eigen::Vector2d &uv);
  void RemoveObservation(std::shared_ptr<ObservationXYZ> obs);

 protected:
  size_t frame_nr_;
  double t0_;
  std::vector<std::shared_ptr<ObservationXYZ>> observations_;
};

} // namespace sfm
} // namespace kontiki
#endif //KONTIKI_VIEW_H
