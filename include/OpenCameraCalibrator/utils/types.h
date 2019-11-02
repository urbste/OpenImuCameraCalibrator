// created by Steffen Urban November 2019
#pragma once

#include <Eigen/Core>
#include <vector>

namespace OpenCamCalib {

struct GPXData
{
    std::vector<Eigen::Vector3d> lle;
    std::vector<double> timestamp_utc_unixtime;
    std::vector<double> timestamp_ms;
    std::vector<double> precision;
    std::vector<double> geoid_height;
    std::vector<Eigen::Vector2d> vel2d_vel3d;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraGyroData
{
    std::vector<Eigen::Vector3d> gyro_measurement;
    std::vector<double> timestamp_ms;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraAccData
{
    std::vector<Eigen::Vector3d> acc_masurement;
    std::vector<double> timestamp_ms;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CameraTelemetryData
{
  // IMU
  CameraAccData accererometer;
  CameraGyroData gyroscope;
  // GPS
  GPXData gps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IMUCalibData
{
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;
    double rate;
    Eigen::Matrix3d R_imu_to_camera;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
