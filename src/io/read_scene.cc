/* Copyright (C) 2021 Steffen Urban
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <ios>
#include <iostream>

#include "OpenCameraCalibrator/io/read_scene.h"

namespace OpenICC {
namespace io {

bool read_scene_bson(const std::string& input_bson,
                     nlohmann::json& scene_json) {
  std::ifstream input_corner_json(input_bson, std::ios::binary);
  if (!input_corner_json.is_open()) {
    std::cerr << "Can not open " << input_bson << "\n";
    return false;
  }
  // read the contents of the file into the vector
  std::uint8_t cont;
  std::vector<std::uint8_t> uson_file_content;
  while (input_corner_json.read(reinterpret_cast<char*>(&cont), sizeof(cont))) {
    uson_file_content.push_back(cont);
  }
  scene_json = nlohmann::json::from_ubjson(uson_file_content);
  input_corner_json.close();
  return true;
}

void scene_points_to_calib_dataset(const nlohmann::json& json,
                                   theia::Reconstruction& reconstruction) {
  // fill reconstruction with board points
  const auto scene_pts_it = json["scene_pts"];
  for (auto& it : scene_pts_it.items()) {
    const theia::TrackId track_id = (theia::TrackId)std::stoi(it.key());
    reconstruction.AddTrack(track_id);
    theia::Track* track = reconstruction.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d* point = track->MutablePoint();
    (*point)[0] = static_cast<double>(it.value()[0]);
    (*point)[1] = static_cast<double>(it.value()[1]);
    (*point)[2] = static_cast<double>(it.value()[2]);
    (*point)[3] = 1.0;
  }
}

}  // namespace io
}  // namespace OpenICC
