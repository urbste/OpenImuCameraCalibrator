#include "OpenCameraCalibrator/core/allan_variance_fitter.h"

#include "OpenCameraCalibrator/allanvariance/allan_acc.h"
#include "OpenCameraCalibrator/allanvariance/allan_gyr.h"

#include "OpenCameraCalibrator/allanvariance/fitallan_acc.h"
#include "OpenCameraCalibrator/allanvariance/fitallan_gyr.h"

namespace OpenICC {
namespace core {

AllanVarianceFitter::AllanVarianceFitter(
    const CameraTelemetryData &telemetry_data, const int nr_clusters)
    : telemetry_data_(telemetry_data) {

  std::cout << "Loading datastructes\n";

  data_acc_x_ = new allanvar::AllanAcc("acc_x", nr_clusters);
  data_acc_y_ = new allanvar::AllanAcc("acc_y", nr_clusters);
  data_acc_z_ = new allanvar::AllanAcc("acc_z", nr_clusters);

  data_gyr_x_ = new allanvar::AllanGyr("gyr_x", nr_clusters);
  data_gyr_y_ = new allanvar::AllanGyr("gyr_y", nr_clusters);
  data_gyr_z_ = new allanvar::AllanGyr("gyr_z", nr_clusters);

  for (size_t i = 0; i < telemetry_data_.accelerometer.timestamp_ms.size();
       ++i) {
    const double t_s = telemetry_data_.accelerometer.timestamp_ms[i] * MS_TO_S;
    data_acc_x_->pushRadPerSec(telemetry_data_.accelerometer.measurement[i][0],
                               t_s);
    data_acc_y_->pushRadPerSec(telemetry_data_.accelerometer.measurement[i][1],
                               t_s);
    data_acc_z_->pushRadPerSec(telemetry_data_.accelerometer.measurement[i][2],
                               t_s);

    data_gyr_x_->pushRadPerSec(telemetry_data_.gyroscope.measurement[i][0],
                               t_s);
    data_gyr_y_->pushRadPerSec(telemetry_data_.gyroscope.measurement[i][1],
                               t_s);
    data_gyr_z_->pushRadPerSec(telemetry_data_.gyroscope.measurement[i][2],
                               t_s);
  }
}

bool AllanVarianceFitter::RunFit() {
  data_gyr_x_->calc();
  std::vector<double> gyro_v_x = data_gyr_x_->getVariance();
  std::vector<double> gyro_d_x = data_gyr_x_->getDeviation();
  std::vector<double> gyro_ts_x = data_gyr_x_->getTimes();

  data_gyr_y_->calc();
  std::vector<double> gyro_v_y = data_gyr_y_->getVariance();
  std::vector<double> gyro_d_y = data_gyr_y_->getDeviation();
  std::vector<double> gyro_ts_y = data_gyr_y_->getTimes();

  data_gyr_z_->calc();
  std::vector<double> gyro_v_z = data_gyr_z_->getVariance();
  std::vector<double> gyro_d_z = data_gyr_z_->getDeviation();
  std::vector<double> gyro_ts_z = data_gyr_z_->getTimes();

  std::cout << "Gyro X " << std::endl;
  allanvar::FitAllanGyr fit_gyr_x( gyro_v_x, gyro_ts_x, data_gyr_x_->getFreq( ) );
  std::cout << "  bias " << data_gyr_x_->getAvgValue( ) / 3600 << " degree/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::cout << "Gyro y " << std::endl;
  allanvar::FitAllanGyr fit_gyr_y( gyro_v_y, gyro_ts_y, data_gyr_y_->getFreq( ) );
  std::cout << "  bias " << data_gyr_y_->getAvgValue( ) / 3600 << " degree/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::cout << "Gyro z " << std::endl;
  allanvar::FitAllanGyr fit_gyr_z( gyro_v_z, gyro_ts_z, data_gyr_z_->getFreq( ) );
  std::cout << "  bias " << data_gyr_z_->getAvgValue( ) / 3600 << " degree/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::vector< double > gyro_sim_d_x = fit_gyr_x.calcSimDeviation( gyro_ts_x );
  std::vector< double > gyro_sim_d_y = fit_gyr_y.calcSimDeviation( gyro_ts_y );
  std::vector< double > gyro_sim_d_z = fit_gyr_z.calcSimDeviation( gyro_ts_z );


  return true;
}

} // namespace core
} // namespace OpenICC