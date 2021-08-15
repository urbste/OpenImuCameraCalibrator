/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once


#include "common_types.h"

#include <theia/sfm/track.h>
#include "OpenCameraCalibrator/utils/types.h"

struct CalibCornerData {
  OpenICC::vec2_vector corners;
  std::vector<theia::TrackId> track_ids;
  std::vector<double> radii;  //!< threshold used for maximum displacement
                              //! during sub-pix refinement; Search region is
  //! slightly larger.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ProjectedCornerData {
  OpenICC::vec2_vector corners_proj;
  std::vector<bool> corners_proj_success;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CalibInitPoseData {
  Sophus::SE3d T_a_c;
  size_t num_inliers;

  OpenICC::vec2_vector reprojected_corners;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

