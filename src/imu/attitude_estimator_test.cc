#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "theia/sfm/feature.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/imu/imu_utils.h"
#include "OpenCameraCalibrator/imu/attitude_estimator.h"
#include "OpenCameraCalibrator/imu/comp_filter.h"

#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace theia {

//TEST(AttitudeEstimator, Name)

//{

//    std::string imu_data_path = "/media/steffen/Data/image_data_sets/Roadshow/PilotGuru/Chair";
//    theia::CameraImuData imu_data;
//    theia::ReadIMUData(imu_data_path, "linear_accelerations.csv", "rotations.csv", imu_data);

//    imu_tools::ComplementaryFilter rotation_estimator;

//    cv::viz::Viz3d window;
//    window.showWidget("CoordinateWidget", cv::viz::WCoordinateSystem());


//    for (int i = 0; i < imu_data.accelerometer.measurement.size(); ++i)
//    {
//        rotation_estimator.update(imu_data.accelerometer.measurement[i](0),
//                imu_data.accelerometer.measurement[i](1),
//                imu_data.accelerometer.measurement[i](2),
//                imu_data.gyroscope.measurement[i](0),
//                imu_data.gyroscope.measurement[i](1),
//                imu_data.gyroscope.measurement[i](2),
//                1./500.);

//        double q0,q1,q2,q3;
//        rotation_estimator.getOrientation(q0,q1,q2,q3);
//        std::cout<<q0<<" "<<q1<<" "<<q2<<" "<<q3<<std::endl;

//        Eigen::Quaternionf q(q0,q1,q2,q3);
//        Eigen::Matrix3f R = q.toRotationMatrix();
//        cv::Mat Rcv;
//        cv::eigen2cv(R,Rcv);
//        cv::Affine3f pose(Rcv, cv::Vec3f(0,0,0));
//        window.setWidgetPose("CoordinateWidget",pose);
//        window.spinOnce(1,true);
//    }


//}


}
