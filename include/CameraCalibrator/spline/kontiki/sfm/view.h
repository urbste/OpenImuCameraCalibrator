//
// Created by hannes on 2017-10-04.
//

#ifndef KONTIKI_VIEW_H
#define KONTIKI_VIEW_H

#include <vector>
#include <memory>

namespace kontiki {
namespace sfm {

class Observation;
class Landmark;

class View : public std::enable_shared_from_this<View> {
 public:
  View(size_t frame, double t0);
  ~View();

  size_t frame_nr() const;
  void set_frame_nr(size_t fnr);
  double t0() const;
  void set_t0(double t0);

  std::vector<std::shared_ptr<Observation>> observations() const;
  std::shared_ptr<Observation> CreateObservation(std::shared_ptr<Landmark> landmark, const Eigen::Vector2d &uv);
  void RemoveObservation(std::shared_ptr<Observation> obs);

 protected:
  size_t frame_nr_;
  double t0_;
  std::vector<std::shared_ptr<Observation>> observations_;
};

} // namespace sfm
} // namespace kontiki
#endif //KONTIKI_VIEW_H
