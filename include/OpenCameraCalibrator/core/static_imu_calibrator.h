#pragma once

#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"

#include "OpenCameraCalibrator/utils/gyro_integration.h"
#include "OpenCameraCalibrator/utils/imu_data_interval.h"

#include <ceres/ceres.h>

using namespace OpenICC::utils;

namespace OpenICC {
namespace core {

using Vector3d = Eigen::Vector3d;

struct MultiPosAccResidual {
  MultiPosAccResidual(const double &g_mag, const Eigen::Vector3d &sample)
      : g_mag_(g_mag), sample_(sample) {}

  template <typename T>
  bool operator()(const T *const params, T *residuals) const {
    Eigen::Matrix<T, 3, 1> raw_samp(T(sample_(0)), T(sample_(1)),
                                    T(sample_(2)));
    /* Assume body frame same as accelerometer frame,
     * so bottom left params in the misalignment matris are set to zero */
    ThreeAxisSensorCalibParams<T> calib_triad(
        params[0], params[1], params[2], T(0), T(0), T(0), params[3], params[4],
        params[5], params[6], params[7], params[8]);

    Eigen::Matrix<T, 3, 1> calib_samp = calib_triad.UnbiasNormalize(raw_samp);
    residuals[0] = T(g_mag_) - calib_samp.norm();
    return true;
  }

  static ceres::CostFunction *Create(const double &g_mag,
                                     const  Vector3d &sample) {
    return (new ceres::AutoDiffCostFunction<MultiPosAccResidual, 1, 9>(
        new MultiPosAccResidual(g_mag, sample)));
  }

  const double g_mag_;
  const Vector3d sample_;
};

struct MultiPosGyroResidual {
  MultiPosGyroResidual(const Vector3d &g_versor_pos0,
                       const Vector3d &g_versor_pos1,
                       const ImuReadings &gyro_samples,
                       const DataInterval &gyro_interval_pos01, double dt,
                       bool optimize_bias)
      :

        g_versor_pos0_(g_versor_pos0), g_versor_pos1_(g_versor_pos1),
        gyro_samples_(gyro_samples), interval_pos01_(gyro_interval_pos01),
        dt_(dt), optimize_bias_(optimize_bias) {}

  template <typename T>
  bool operator()(const T *const params, T *residuals) const {
    ThreeAxisSensorCalibParams<T> calib_triad(
        params[0], params[1], params[2], params[3], params[4], params[5],
        params[6], params[7], params[8], optimize_bias_ ? params[9] : T(0),
        optimize_bias_ ? params[10] : T(0), optimize_bias_ ? params[11] : T(0));

    std::vector<ImuReading<T>> calib_gyro_samples;
    calib_gyro_samples.reserve(interval_pos01_.end_idx -
                               interval_pos01_.start_idx + 1);

    for (int i = interval_pos01_.start_idx; i <= interval_pos01_.end_idx; i++) {
      Eigen::Matrix<T, 3, 1> gyro(T(gyro_samples_[i].x()), T(gyro_samples_[i].y()), T(gyro_samples_[i].z()));
      calib_gyro_samples.push_back(
          ImuReading<T>(T(gyro_samples_[i].timestamp_s()), calib_triad.UnbiasNormalize(gyro)));
    }
    Eigen::Matrix<T, 3, 3> rot_mat;
    IntegrateGyroInterval(calib_gyro_samples, rot_mat, T(dt_));

    Eigen::Matrix<T, 3, 1> diff =
        rot_mat.transpose() * g_versor_pos0_.template cast<T>() -
        g_versor_pos1_.template cast<T>();

    residuals[0] = diff(0);
    residuals[1] = diff(1);
    residuals[2] = diff(2);

    return true;
  }

  static ceres::CostFunction *
  Create(const Vector3d &g_versor_pos0,
         const Vector3d &g_versor_pos1,
         const ImuReadings &gyro_samples,
         const DataInterval &gyro_interval_pos01, double dt,
         bool optimize_bias) {
    if (optimize_bias)
      return (new ceres::AutoDiffCostFunction<MultiPosGyroResidual, 3, 12>(
          new MultiPosGyroResidual(g_versor_pos0, g_versor_pos1, gyro_samples,
                                   gyro_interval_pos01, dt, optimize_bias)));
    else
      return (new ceres::AutoDiffCostFunction<MultiPosGyroResidual, 3, 9>(
          new MultiPosGyroResidual(g_versor_pos0, g_versor_pos1, gyro_samples,
                                   gyro_interval_pos01, dt, optimize_bias)));
  }

  const Vector3d g_versor_pos0_, g_versor_pos1_;
  const ImuReadings gyro_samples_;
  const DataInterval interval_pos01_;
  const double dt_;
  const bool optimize_bias_;
};

/** @brief This object enables to calibrate an accelerometers triad and
 * eventually a related gyroscopes triad (i.e., to estimate theirs misalignment
 * matrix, scale factors and biases) using the multi-position calibration
 * method.
 *
 * For more details, please see:
 *
 * D. Tedaldi, A. Pretto and E. Menegatti
 * "A Robust and Easy to Implement Method for IMU Calibration without External
 * Equipments" In: Proceedings of the IEEE International Conference on Robotics
 * and Automation (ICRA 2014), May 31 - June 7, 2014 Hong Kong, China, Page(s):
 * 3042 - 3049
 */
class StaticImuCalibrator {
public:
  /** @brief Default constructor: initilizes all the internal members with
   * default values */
  StaticImuCalibrator();
  ~StaticImuCalibrator() {}

  /** @brief Provides the magnitude of the gravitational filed
   *         used in the calibration (i.e., the gravity measured in
   *         the place where the calibration dataset has been acquired) */
  double GravityMagnitede() const { return g_mag_; }

  /** @brief Provides the duration in seconds of the initial static interval */
  double InitStaticIntervalDuration() const { return init_interval_duration_; }

  /** @brief Provides the number of data samples to be extracted from each
   * detected static intervals */
  int IntarvalsNumSamples() const { return interval_n_samples_; }

  /** @brief Provides the accelerometers initial guess calibration parameters */
  const ThreeAxisSensorCalibParams<double> &initAccCalibration() {
    return init_acc_calib_;
  }

  /** @brief Provides the gyroscopes initial guess calibration parameters */
  const ThreeAxisSensorCalibParams<double> &initGyroCalibration() {
    return init_gyro_calib_;
  }

  /** @brief True if the accelerometers calibration is obtained using the mean
   *         accelerations of each static interval instead of all samples */
  bool AccUseMeans() const { return acc_use_means_; }

  /** @brief Provides the (fixed) data period used in the gyroscopes
   * integration. If this period is less than 0, the gyroscopes timestamps are
   * used in place of this period. */
  double GyroDataPeriod() const { return gyro_dt_; }

  /** @brief True if the gyroscopes biases are estimated along with the
   * calibration parameters. If false, the gyroscopes biases (computed in the
   * initial static period) are assumed known. */
  bool OptimizeGyroBias() const { return optimize_gyro_bias_; }

  /** @brief True if the verbose output is enabled */
  bool VerboseOutput() const { return verbose_output_; }

  /** @brief Set the magnitude of the gravitational filed
   *         used in the calibration (i.e., the gravity measured in
   *         the place where the calibration dataset has been acquired)
   *
   *         To find your magnitude of the gravitational filed,
   *         take a look for example to https://www.wolframalpha.com
   */
  void SetGravityMagnitude(double g) { g_mag_ = g; }

  /** @brief Set the duration in seconds of the initial static interval. Default
   * 30 seconds. */
  void SetInitStaticIntervalDuration(double duration_s) {
    init_interval_duration_ = duration_s;
  }

  /** @brief Set the number of data samples to be extracted from each detected
   * static intervals. Default is 100.  */
  void SetIntarvalsNumSamples(int num) { interval_n_samples_ = num; }

  /** @brief Set the accelerometers initial guess calibration parameters */
  void SetInitAccCalibration(ThreeAxisSensorCalibParams<double> &init_calib) {
    init_acc_calib_ = init_calib;
  }

  /** @brief Set the gyroscopes initial guess calibration parameters */
  void SetInitGyroCalibration(ThreeAxisSensorCalibParams<double> &init_calib) {
    init_gyro_calib_ = init_calib;
  }

  /** @brief If the parameter enabled is true, the accelerometers calibration is
   * obtained using the mean accelerations of each static interval instead of
   * all samples. Default is false.
   */
  void EnableAccUseMeans(bool enabled) { acc_use_means_ = enabled; }

  /** @brief Set the (fixed) data period used in the gyroscopes integration.
   *         If this period is less than 0, the gyroscopes timestamps are used
   *         in place of this period. Default is -1.
   */
  void SetGyroDataPeriod(double dt) { gyro_dt_ = dt; }

  /** @brief If the parameter enabled is true, the gyroscopes biases are
   * estimated along with the calibration parameters. If false, the gyroscopes
   * biases (computed in the initial static period) are assumed known. */
  void EnableGyroBiasOptimization(bool enabled) {
    optimize_gyro_bias_ = enabled;
  }

  /** @brief If the parameter enabled is true, verbose output is activeted  */
  void EnableVerboseOutput(bool enabled) { verbose_output_ = enabled; }

  /** @brief Estimate the calibration parameters for the acceleremoters triad
   *         (see CalibratedTriad_) using the multi-position calibration method
   *
   * @param acc_samples Acceleremoters data vector, ordered by increasing
   * timestamps, collected at the sensor data rate.
   */
  bool CalibrateAcc(const CameraAccData &acc_samples);

  /** @brief Estimate the calibration parameters for both the acceleremoters
   *         and the gyroscopes triads (see CalibratedTriad_) using the
   *         multi-position calibration method
   *
   * @param acc_samples Acceleremoters data vector, ordered by increasing
   * timestamps, collected at the sensor data rate.
   * @param gyro_samples Gyroscopes data vector, ordered by increasing
   * timestamps, collected in parallel with the acceleations at the sensor data
   * rate.
   */
  bool CalibrateAccGyro(const CameraAccData &acc_samples,
                        const CameraGyroData &gyro_samples);

  /** @brief Provide the calibration parameters for the acceleremoters triad (it
   * should be called after calibrateAcc() or calibrateAccGyro() ) */
  const ThreeAxisSensorCalibParams<double> &getAccCalib() const { return acc_calib_; }
  /** @brief Provide the calibration parameters for the gyroscopes triad (it
   * should be called after calibrateAccGyro() ). */
  const ThreeAxisSensorCalibParams<double> &getGyroCalib() const { return gyro_calib_; }

  /** @brief Provide the calibrated acceleremoters data vector (it should be
   * called after calibrateAcc() or calibrateAccGyro() ) */
  const CameraAccData &getCalibAccSamples() const { return calib_acc_samples_; }

  /** @brief Provide the calibrated gyroscopes data vector (it should be called
   * after calibrateAccGyro() ) */
  const CameraGyroData &getCalibGyroSamples() const {
    return calib_gyro_samples_;
  }

private:
  double g_mag_;
  const int min_num_intervals_;
  double init_interval_duration_;
  int interval_n_samples_;
  bool acc_use_means_;
  double gyro_dt_;
  bool optimize_gyro_bias_;
  std::vector<utils::DataInterval> min_cost_static_intervals_;
  ThreeAxisSensorCalibParams<double> init_acc_calib_, init_gyro_calib_;
  ThreeAxisSensorCalibParams<double> acc_calib_, gyro_calib_;
  CameraAccData calib_acc_samples_;
  CameraGyroData calib_gyro_samples_;

  bool verbose_output_;
};

} // namespace core
} // namespace OpenICC
