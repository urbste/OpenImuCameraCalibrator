#pragma once

#include <unordered_map>

#include "OpenCameraCalibrator/utils/types.h"

#include "OpenCameraCalibrator/basalt_spline/calib_helpers.h"
#include "OpenCameraCalibrator/basalt_spline/ceres_calib_spline_split.h"

namespace OpenCamCalib {
namespace core {

const int SPLINE_N = 4;

class ImuCameraCalibrator {
public:
    ImuCameraCalibrator() {}
    void InitSpline(const theia::Reconstruction &calib_dataset,
                    const Sophus::SE3<double>& T_i_c_init,
                    const OpenCamCalib::SplineWeightingData& spline_weight_data,
                    const double time_offset_imu_to_cam,
                    const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accl_bias,
                    const OpenCamCalib::CameraTelemetryData& telemetry_data);

    void InitializeGravity(const OpenCamCalib::CameraTelemetryData& telemetry_data,
                           const Eigen::Vector3d& accl_bias);

    double Optimize(const int iterations);

    void ToTheiaReconDataset(theia::Reconstruction& output_recon);

    CeresCalibrationSplineSplit<SPLINE_N> trajectory_;

    //! camera timestamps in seconds
    std::vector<double> GetCamTimestamps() { return cam_timestamps_; }

    //! get gyroscope measurements
    aligned_map<double, Eigen::Vector3d> GetGyroMeasurements() { return gyro_measurements; }

    //! get accelerometer measurements
    aligned_map<double, Eigen::Vector3d> GetAcclMeasurements() { return accl_measurements; }

private:
    //! camera timestamps
    std::vector<double> cam_timestamps_;

    //! accl measurements
    aligned_map<double, Eigen::Vector3d> gyro_measurements;

    //! gyro measurements
    aligned_map<double, Eigen::Vector3d> accl_measurements;

    //! imu update rate in hz
    double imu_update_rate_hz_;

    //! spline know spacing in R3 and SO3 in seconds
    SplineWeightingData spline_weight_data_;

    //! spline start and end time in seconds
    double t0_s_;
    double tend_s_;

    //! number of knots in SO3 and R3
    uint64_t nr_knots_so3_;
    uint64_t nr_knots_r3_;

    //! camera readout time
    double cam_readout_s_;

    //! is gravity direction in sensor frame is initialized
    bool gravity_initialized_ = false;

    //! gravity in sensor frame
    Eigen::Vector3d gravity_init_;

    Sophus::SE3<double> T_i_c_init_;

    //! -
    std::unordered_map<TimeCamId, CalibCornerData> calib_corners_;
    std::unordered_map<TimeCamId, CalibInitPoseData> calib_init_poses_;
    std::unordered_map<TimeCamId, CalibInitPoseData> spline_init_poses_;

};

}
}
