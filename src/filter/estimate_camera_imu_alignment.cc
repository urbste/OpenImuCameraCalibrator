// created by Steffen Urban 2019, January

#include "OpenCameraCalibrator/filter/estimate_camera_imu_alignment.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include "OpenCameraCalibrator/utils/utils.h"
// C++ port from https://github.com/jannemus/InertialScale

namespace OpenCamCalib {
namespace filter {

int FindMinNearestTimestamp(
        const double t_imu,
        const double dt,
        const std::vector<double> &vis_timestamps,
        double& distance_to_nearest_timestamp)
{
    double dist = std::numeric_limits<double>::max();
    int idx = 0;
    for ( int i = 0; i < vis_timestamps.size(); ++i ) {
        double new_dist = std::abs(t_imu - vis_timestamps[i]);
        if (new_dist < dist)  {
            distance_to_nearest_timestamp = new_dist;
            idx = i;
            dist = new_dist;
            // if we are between two timestamps we can break here,
            // because there will be no closer timestamp
            if (std::abs(dist) < dt)
                break;
        }
    }

    return idx;
}

Eigen::Vector3d lerp3d(const Eigen::Vector3d& v0,
                     const Eigen::Vector3d& v1,
                     double fraction) {
    return (1.0-fraction) * v0 + fraction * v1;
}

void InterpolateQuaternions(
        std::vector<double> t_vis_s,
        std::vector<double> t_imu_s,
        const QuatVector& input_qtVis,
        const double vis_dt_s,
        QuatVector& interpolated_vis_quat) {
    for (size_t i=0; i < t_imu_s.size(); ++i) {
        double dist_to_nearest_vis_t;
        int nearest_vis_idx = FindMinNearestTimestamp(t_imu_s[i], vis_dt_s, t_vis_s, dist_to_nearest_vis_t);
        double fraction = dist_to_nearest_vis_t / vis_dt_s;
        interpolated_vis_quat.push_back(input_qtVis[nearest_vis_idx].slerp(fraction, input_qtVis[nearest_vis_idx+1]));
    }
}

void InterpolateVector3d(
        std::vector<double> t_old,
        std::vector<double> t_new,
        const Vec3Vector& input_vec,
        const double dt,
        Vec3Vector& interpolated_vec) {


    for (size_t i=0; i < t_new.size(); ++i) {
        double dist_to_nearest_vis_t;
        int nearest_vis_idx = FindMinNearestTimestamp(t_new[i], dt, t_old, dist_to_nearest_vis_t);
        double fraction = dist_to_nearest_vis_t / dt;
        if (fraction > 1.0)
            interpolated_vec.push_back(input_vec[nearest_vis_idx]);
        else
            interpolated_vec.push_back(
                lerp3d(input_vec[nearest_vis_idx],input_vec[nearest_vis_idx+1],fraction));
    }
}


double SolveClosedForm(
    const Vec3Vector& angVis,
    const Vec3Vector& angImu,
    const std::vector<double> timestamps_s,
    const double td,
    const double dt_imu,
    Matrix3d& Rs,
    Vector3d& bias) {

    // offset the angular velocities with td
    std::vector<double> time_with_offset(timestamps_s.size());
    for (size_t i=0; i < timestamps_s.size(); ++i) {
        time_with_offset[i] = timestamps_s[i] - td;
    }
    Vec3Vector interpolated_angVis;
    InterpolateVector3d(time_with_offset, timestamps_s, angVis, dt_imu, interpolated_angVis);

    // compute mean vectors
    Vector3d mean_vis;
    Vector3d mean_imu;
    for (size_t i = 0; i < interpolated_angVis.size(); ++i) {
        mean_imu += angImu[i];
        mean_vis += interpolated_angVis[i];
    }
    mean_imu /= static_cast<double>(interpolated_angVis.size());
    mean_vis /= static_cast<double>(interpolated_angVis.size());

    // centralized
    MatrixXd P, Q;
    P.resize(interpolated_angVis.size(),3);
    Q.resize(interpolated_angVis.size(),3);

    for (size_t i = 0; i < interpolated_angVis.size(); ++i) {
        P.row(i) = angImu[i] - mean_imu;
        Q.row(i) = interpolated_angVis[i] - mean_vis;
    }
    Eigen::JacobiSVD<MatrixXd> svd(P.transpose()*Q, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Matrix3d C;
    C.setIdentity();
    if ( (svd.matrixV()*svd.matrixU().transpose()).determinant() < 0.0)
        C(2,2) = -1.0;

    Rs = svd.matrixV() * C * svd.matrixU().transpose();

    bias = mean_vis - Rs * mean_imu;

    double error = 0.0;
    for (size_t i = 0; i < angVis.size(); ++i) {
        Vector3d D = interpolated_angVis[i] - (Rs * angImu[i]  + bias);
        error += D.squaredNorm();
    }

    return error;
}

void EstimateCameraImuAlignment(
    const QuatMap& visual_rotations,
    const Vec3Map& imu_angular_vel,
    const double dt_vis,
    const double dt_imu,
    Matrix3d& R_imu_to_camera,
    double& time_offset_imu_to_camera,
    Vector3d& gyro_bias) {
  // find start and end points of camera and imu
  const double start_time_cam = visual_rotations.begin()->first;
  const double end_time_cam = visual_rotations.rbegin()->first;
  const double start_time_imu = imu_angular_vel.begin()->first;
  const double end_time_imu = imu_angular_vel.rbegin()->first;

  double t0 =
      (start_time_cam >= start_time_imu) ? start_time_cam : start_time_imu;
  double tend = (end_time_cam >= end_time_imu) ? end_time_cam : end_time_imu;

  // create zero-based maps
  QuatMap visual_rotations_clamped;
  Vec3Map imu_angular_vel_clamped;


  for (auto const& imu_rot : imu_angular_vel) {
    if (imu_rot.first >= t0 && imu_rot.first <= tend) {
      imu_angular_vel_clamped[imu_rot.first - t0] = imu_rot.second;
    }
  }
  Vec3Vector angImu;
  for (const auto& imu : imu_angular_vel_clamped) {
      angImu.push_back(imu.second);
  }

  // interpolate visual rotations
  for (auto const& v_rot : visual_rotations) {
    if (v_rot.first >= t0 && v_rot.first <= tend) {
      visual_rotations_clamped[v_rot.first - t0] = v_rot.second;
    }
  }

  // get vectors of clamped imu angular velocities
  std::vector<double> tIMU;
  for (auto const& imu : imu_angular_vel_clamped) {
      tIMU.push_back(imu.first);
  }

  std::vector<double> tVis;
  QuatVector qtVis;

  for (auto const& vis : visual_rotations_clamped) {
      tVis.push_back(vis.first);
      qtVis.push_back(vis.second);
  }

  QuatVector qtVis_interp;
  InterpolateQuaternions(tVis, tIMU, qtVis, dt_vis, qtVis_interp);

  // compute angular velocities
  QuatVector qtDiffs;
  Vec3Vector angVis;
  for (size_t i = 1; i < qtVis_interp.size();++i) {
      Quaterniond q;
      q.w() = qtVis_interp[i].w() - qtVis_interp[i-1].w();
      q.x() = qtVis_interp[i].x() - qtVis_interp[i-1].x();
      q.y() = qtVis_interp[i].y() - qtVis_interp[i-1].y();
      q.z() = qtVis_interp[i].z() - qtVis_interp[i-1].z();
      qtDiffs.push_back(q);
  }
  qtDiffs.push_back(qtDiffs[qtDiffs.size()-1]);

  for (size_t i = 0; i < qtDiffs.size();++i) {
      Quaterniond angVisQ = qtDiffs[i] * qtVis_interp[i].inverse();
      const double diff_dt = -2.0 * dt_imu;
      Vector3d angVisVec(
                  diff_dt * angVisQ.x(),
                  diff_dt * angVisQ.y(),
                  diff_dt * angVisQ.z());
      angVis.push_back(angVisVec);
  }

  const double gRatio = (1.0 + std::sqrt(5.0)) / 2.0;
  const double tolerance = 1e-4;

  const double maxOffset = 0.5;
  double a = -maxOffset;
  double b = maxOffset;

  double c = b - (b - a) / gRatio;
  double d = a + (b - a) / gRatio;

  unsigned int iter = 0;
  double error = 0.0;
  while (std::abs(c-d) > tolerance) {

      Eigen::Matrix3d Rsc, Rsd;
      Eigen::Vector3d biasc, biasd;
      const double fc = SolveClosedForm(angVis, angImu, tIMU, c, dt_imu, Rsc, biasc);
      const double fd = SolveClosedForm(angVis, angImu, tIMU, d, dt_imu, Rsd, biasd);

      if (fc < fd) {
          b = d;
          R_imu_to_camera = Rsc;
          gyro_bias = biasc;
          error = fc;
      } else {
          a = c;
          R_imu_to_camera = Rsd;
          gyro_bias = biasd;
          error = fd;
      }

      c = b - (b - a) / gRatio;
      d = a + (b - a) / gRatio;

      iter = iter + 1;
  }
  time_offset_imu_to_camera = (b + a) / 2;

  std::cout<<"Finished golden-section search in "<<iter<<" iterations.\n";
  std::cout<<"Final gyro to camera rotation is: "<<R_imu_to_camera<<"\n";
  Eigen::Quaterniond qat(R_imu_to_camera);
  std::cout<<"Final gyro to camera quaternion is: "<<qat.w()<<" "<<qat.x()<<" "<< qat.y()<<" "<<qat.z()<<"\n";
  std::cout<<"Gyro bias is estimated to be: "<<gyro_bias<<"\n";
  std::cout<<"Estimated time offset: "<<time_offset_imu_to_camera<<"\n";
  std::cout<<"Final alignment error: "<<error<<"\n";


}
}
}
