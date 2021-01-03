#pragma once

#include <opencv2/viz/vizcore.hpp>

namespace OpenCamCalib {
namespace viz {

class SplineVisualizer {
    SplineVisualizer() {}


    void UpdateCameraPoses();

    void UpdateImuPoses();

    void UpdateObjectPoints(const std::vector<cv::Point3d>& points);

    void DrawScene();

private:
    //! camera poses
    std::vector<cv::Affine3d> camera_poses_;

    //! imu poses
    std::vector<cv::Affine3d> imu_poses_;

    //! global frame
    cv::Affine3d global_frame_;

    //! points
    std::vector<cv::Point3d> object_points_;


};



}
}
