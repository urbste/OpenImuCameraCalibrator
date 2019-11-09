// created by Steffen Urban 2019, January

#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>

namespace filter {

void FilterVisualPositions(
    const std::map<double, Eigen::Vector3d>& visual_positions,
    const double process_noise, std::vector<Eigen::Vector3d>* accelerations);
}
