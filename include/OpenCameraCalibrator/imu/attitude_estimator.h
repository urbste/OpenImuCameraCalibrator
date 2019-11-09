// Copyright (c) 2014, Autonomous Intelligent Systems Group, Rheinische
// Friedrich-Wilhelms-Universität Bonn
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the Rheinische Friedrich-Wilhelms-Universität Bonn
//      nor the names of its contributors may be used to endorse or promote
//      products derived from this software without specific prior written
//      permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// Attitude estimator based on the fusion of 3-axis accelerometer, gyroscope and
// magnetometer data
// File: attitude_estimator.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

// State estimation namespace
namespace stateestimation {
/**
* @class AttitudeEstimator
*
* @brief Fuses 3-axis accelerometer, gyroscope and magnetometer data into an
*attitude estimate using a nonlinear Passive Complementary Filter.
*
* Attitude Estimator is a generic platform-independent C++ library that
*implements an IMU sensor fusion algorithm. Up to 3-axis gyroscope,
* accelerometer and magnetometer data can be processed into a full 3D quaternion
*orientation estimate, with the use of a nonlinear Passive
* Complementary Filter. The library is targeted at robotic applications, but is
*by no means limited to this. Features of the estimator
* include gyro bias estimation, transient quick learning phases, tuneable
*estimator parameters, and near-global stability backed by
* theoretical analysis.
*
* Great emphasis has been placed on having a very efficient, yet totally
*numerically and algorithmically robust implementation
* of the filter. The code size has also been kept to a minimum, and has been
*extremely well-commented. The programmatic interface
* has also been made as easy to use as possible. Please refer to the extensive
*documentation of the library for more information on its
* capabilities and usage caveats.
*
* Attitude Estimator was developed as part of the <a
*href="http://www.nimbro.net/OP/">NimbRo-OP</a> project at the University of
*Bonn.
*
* The inputs to the estimator should respect the following guidelines:
*
* - The @b gyroscope data must be in `rad/s`, and should express the measured
*angular velocity in body-fixed coordinates. If only 2-axis
*   gyro data is available, then the missing component should simply be set to
*zero. The attitude estimator in this case will do the best
*   it can to still estimate the full 3D orientation, but you can expect
*effective loss of rotational information around one axis.
*   This can manifest itself in various ways in the output, and it must be
*decided on a per-situation basis how to best manage the
*   resulting information loss, so as to minimise resulting performance
*reduction.
*
* - The @b accelerometer data can be in any self-consistent units, preferably
*<tt>ms<sup>-2</sup></tt>, and should express the measured
*   <i><b>inertial acceleration</b></i> in body-fixed coordinates. This is a
*<i><b>very important point</b></i> to note, or you may
*   end up giving data of the wrong sign convention to the filter! The
*convention of supplying an inertial acceleration means specifically
*   that if gravity is measured to be in one direction, then the supplied
*accelerometer value should be in exactly the opposite direction.
*   So for example, if gravity is measured to point down the negative z-axis in
*body-fixed coordinates, then the accelerometer data provided
*   to the `update()` function should be the exact negative thereof, pointing in
*the direction of the positive z-axis (i.e. up). Note that only
*   the direction of the acceleration vector is considered by the estimator -
*the magnitude is ignored, unless it is zero of course. If only
*   2-axis accelerometer data is available, then you may calculate a third
*component that makes the total magnitude of the acceleration
*   vector equal to what you expect from gravity. In this case the units and
*quantitative correctness of the accelerometer values are important.
*
* - The @b magnetometer data can be in any self-consistent units, preferably
*`gauss` (`Ga`), and should express the measured magnetic field
*   strength in body-fixed coordinates. If the body is rotated in-place in its
*environment, then ideally the magnetometer data should stay
*   within a sphere of some radius that corresponds to the overall magnetic
*field strength. This data quality may be achieved using hard
*   and/or soft iron calibrations, as required. If no magnetometer is present in
*the system, then just pass zero for each component. The
*   consideration of magnetometer information is automatically disabled in this
*case. You can also achieve the same effect by setting a zero
*   magnetometer calibration value (see `setMagCalib()`). Providing a
*magnetometer calibration value simply makes the difference between having
*   relative or absolute yaw information encoded in the output quaternion. Note
*that the magnetometer readings do not influence the pitch and
*   roll components of the output quaternion state estimate. This was a design
*decision to limit the invalidation of the state estimation output
*   in the case of an invalid or inaccurate magnetometer calibration. If only
*1-axis compass data is available, then you can mathematically
*   transform this into a synthesised 3D magnetometer value, based on the
*current orientation estimate. This is a slightly more involved
*   calculation however, and is beyond the scope of this discussion.
*
* - The <b>time increment</b> (@c dt) passed to the `update()` function in each
*step must be expressed in seconds. It is recommended that you
*   measure the time between the successive calls to the `update()` function
*using a high resolution timer, and calculate the @c dt value from
*   this, making sure that you coerce the value of @c dt to a suitable range.
*The coercion is important to avoid the error
*   inducing effects of having abnormally large and/or small @c dt values
*passing through the estimator. For example, if the nominal call rate of
*   the `update()` function is once per 10ms, then a reasonable coercion range
*would be something like 8ms to 25ms.
*
* All of the most common issues encountered when attempting to use the
*AttitudeEstimator class come from passing incorrect, incorrectly signed,
* and/or badly conditioned data to the `update()` function. *The estimator can't
*do any better than the data you give it!*
*
* It is highly recommended to ensure via experimentation that the x, y and
*z-axes of the gyroscope, accelerometer and magnetometer readings
* are consistent with each other, as they must be for the estimator to work
*correctly. For example, when performing a pure positive x-rotation
* according to the gyroscope, from an upright position, you should observe that
*the y-component of the accelerometer reading increases, while
* the x-component stays constant and the z-component decreases. You should check
*all (up to) nine sensor readings for such internal consistency.
*
* It is also highly recommended that you experimentally confirm that the units
*of the sensor data values are exactly as you expect, and that
* reasonable values are being returned from your hardware. For example, when the
*hardware is stationary you should have that the magnitude
* of the accelerometer measurement is close to 9.81<tt>ms<sup>-2</sup></tt>, in
*whatever units you choose to express the accelerations in,
* and that the gyroscope measurement is in the vague vicinity of zero. You
*should also observe that the magnetometer data values you are
* providing lie approximately on a sphere centered at the origin (i.e. constant
*vector magnitude). This is rarely the case for raw magnetometer
* measurements, and so you will likely need to do a hard-iron calibration (or
*equivalent).
*
* If there is excessive noise on the data inputs, then there is only so much
*that the estimator can be expected to filter out while still
* maintaining its tracking performance (although, incidentally, this balance can
*be adjusted via the PI gains). In situations like this you
* would be right to invest time into signal conditioning and/or tracing the true
*source of the sensor noise.
*
* All angles are expressed in radians (e.g. the return values of the
*`eulerYaw()`, `eulerPitch()` and `eulerRoll()` functions, etc...).
*
* The AttitudeEstimator class can be used as follows:
* @code
* // Declare an instance of the class (e.g. as a member of another class)
* AttitudeEstimator Est;
*
* // Initialise the estimator (e.g. in the class constructor, none of these are
*actually strictly required for the estimator to work, and can be set at any
*time)
* Est.setMagCalib(0.68, -1.32, 0.0);         // Recommended: Use if you want
*absolute yaw information as opposed to just relative yaw (Default: (1.0, 0.0,
*0.0))
* Est.setPIGains(2.2, 2.65, 10, 1.25);       // Recommended: Use if the default
*gains (shown) do not provide optimal estimator performance (Note: Ki = Kp/Ti)
* Est.setQLTime(2.5);                        // Optional: Use if the default
*quick learning time is too fast or too slow for your application (Default: 3.0)
* Est.setAttitude(0.5, 0.5, 0.5, 0.5);       // Optional: Use if you have prior
*knowledge about the orientation of the robot (Default: Identity orientation)
* Est.setAttitudeEuler(M_PI, 0.0, 0.0);      // Optional: Use if you have prior
*knowledge about the orientation of the robot (Default: Identity orientation)
* Est.setAttitudeFused(M_PI, 0.0, 0.0, 1.0); // Optional: Use if you have prior
*knowledge about the orientation of the robot (Default: Identity orientation)
* Est.setGyroBias(0.152, 0.041, -0.079);     // Optional: Use if you have prior
*knowledge about the gyroscope bias (Default: (0.0, 0.0, 0.0))
* Est.setAccMethod(Est.ME_FUSED_YAW);        // Optional: Use if you wish to
*experiment with varying acc-only resolution methods
*
* // Main loop
* while(true)
* {
* 	...
* 	double g[3], a[3], m[3];
* 	get_sensor_data(g, a, m);
* 	Est.update(0.020, g[0], g[1], g[2], a[0], a[1], a[2], m[0], m[1], m[2]);
* 	...
* 	double q[4];
* 	Est.getAttitude(q);
* 	cout << "My attitude is (quaternion): (" << q[0] << "," << q[1] << ","
*<< q[2] << "," << q[3] << ")" << endl;
* 	cout << "My attitude is (ZYX Euler): (" << Est.eulerYaw() << "," <<
*Est.eulerPitch() << "," << Est.eulerRoll() << ")" << endl;
* 	cout << "My attitude is (fused): (" << Est.fusedYaw() << "," <<
*Est.fusedPitch() << "," << Est.fusedRoll() << "," << (Est.fusedHemi() ? 1 : -1)
*<< ")" << endl;
* 	...
* 	if(robot_moved_manually_on_field())
* 	{
* 		Est.reset(true, false);                 // Reset into quick
*learning mode, but preserve the current gyro bias estimate
* 		Est.setAttitudeEuler(M_PI_2, 0.0, 0.0); // Optional: Use if you
*have prior knowledge about the new orientation of the robot
* 	}
* 	...
* }
* @endcode
**/
class AttitudeEstimator {
 public:
  // Constants (see attitude_estimator.cpp for the numeric values)
  static const double ACC_TOL_SQ;  //!< @brief If an acc measurement has
                                   //!norm-squared less than this, then it is
                                   //!considered to be faulty and the
                                   //!measurement is discarded.
  static const double QY_NORM_TOL_SQ;  //!< @brief If an acc-only generated
                                       //!quaternion `Qy` has norm-squared less
                                       //!than this, then the quaternion is
                                       //!considered to be zero and a fallback
                                       //!solution is used.
  static const double QHAT_NORM_TOL_SQ;  //!< @brief If a supposedly near-unit
                                         //!quaternion has norm-squared less
                                         //!than this during normalisation, then
                                         //!we declare a general state of
                                         //!emergency and completely reset the
                                         //!estimator.
  static const double
      XGYG_NORM_TOL_SQ;  //!< @brief If the norm-squared of the calculated `xG`
                         //!and/or `yG` basis vectors (`=
                         //!norm(m_magCalib)*norm(mag)*sin(angle(acc,mag))`) is
                         //!less than this prior to basis normalisation, then
                         //!the calculation is assumed to be faulty due to
                         //!acc/mag collinearity, and the mag measurement is
                         //!discarded. Note however that this constant is also
                         //!used for various checks in the `updateQy()` special
                         //!cases.
  static const double WEZE_NORM_TOL_SQ;  //!< @brief If the norm-squared of the
                                         //!`Qhat` vector with its `x` and `y`
                                         //!components zeroed out is less than
                                         //!this, then normalisation is avoided
                                         //!and a fallback method is used
                                         //!instead to generate a unit pure yaw
                                         //!quaternion in the fused yaw acc-only
                                         //!case.
  static const double ZGHAT_ABS_TOL;     //!< @brief If the absolute value of
                                         //!components of the `zGhat` vector are
                                         //!less than this, then they are
                                      //!considered to be zero (used deep inside
                                      //!the near-impossible special cases
                                      //!only).

  //! @brief Acc-only resolution method enumeration.
  enum AccMethodEnum {
    ME_DEFAULT =
        0,  //!< @brief Default acc-only resolution method (`ME_FUSED_YAW`).
    ME_FUSED_YAW = ME_DEFAULT,  //!< @brief Resolve acc-only cases into full 3D
                                //!orientations using the relative fused yaw
                                //!method (default).
    ME_ABS_FUSED_YAW,           //!< @brief Resolve acc-only cases into full 3D
                       //!orientations using the absolute fused yaw method.
    ME_ZYX_YAW,  //!< @brief Resolve acc-only cases into full 3D orientations
                 //!using the relative ZYX yaw method.
    ME_COUNT     //!< @brief Total number of acc-only resolution methods.
  };

  // Constructor
  explicit AttitudeEstimator(
      bool quickLearn = true);  //!< @brief Default constructor.

  // Reset functions
  void reset(bool quickLearn = true,
             bool resetGyroBias = true);  //!< @brief Resets the entire class,
                                          //!except for the magnetometer
                                          //!calibration, the acc-only
                                          //!resolution method, and the
                                          //!configuration variables (i.e. the
                                          //!PI gains and the quick learn time).
  void resetAll(bool quickLearn = true);  //!< @brief Resets the entire class,
                                          //!including all variables not reset
                                          //!by the `reset()` function.
 private:
  void resetState(bool resetGyroBias);  //!< @brief Equivalent to reset(), but
                                        //!also leaves the lambda value
                                        //!untouched.

 public:
  // Get/set functions for the acc-only resolution method
  AccMethodEnum getAccMethod() const {
    return m_accMethod;
  }  //!< @brief Returns the currently selected acc-only measurement resolution
     //!method.
  void setAccMethod(AccMethodEnum method) {
    m_accMethod =
        (method < ME_DEFAULT || method >= ME_COUNT ? ME_DEFAULT : method);
  }  //!< @brief Sets the acc-only measurement resolution method to use.

  // Get/set functions for the current attitude estimate
  void getAttitude(double q[]) const {
    for (int i = 0; i < 4; i++) q[i] = m_Qhat[i];
  }  //!< @brief Returns the current attitude estimate in the quaternion form
     //!(w,x,y,z) (Note: `q[]` must have room for 4 elements).
  void setAttitude(const double q[]) {
    setAttitude(q[0], q[1], q[2], q[3]);
  }  //!< @brief Resets the current attitude estimate to a particular quaternion
     //!orientation (Note: `q[]` must have 4 elements, `q[]` is normalised, if
     //!`q[]` has zero norm then the attitude estimate is reset to the identity
     //!orientation).
  void setAttitude(double w, double x, double y,
                   double z);  //!< @brief Resets the current attitude estimate
                               //!to a particular quaternion orientation (Note:
                               //!`q[]` is normalised, if `q[]` has zero norm
                               //!then the attitude estimate is reset to the
                               //!identity orientation).
  void setAttitudeEuler(double yaw, double pitch,
                        double roll);  //!< @brief Resets the current attitude
                                       //!estimate to a particular set of ZYX
                                       //!Euler angles.
  void setAttitudeFused(double yaw, double pitch, double roll,
                        bool hemi);  //!< @brief Resets the current attitude
                                     //!estimate to a particular set of fused
                                     //!angles.

  // Get functions for the current attitude estimate in alternative
  // representations
  double eulerYaw() {
    if (!m_eulerValid) {
      updateEuler();
    }
    return m_Ehat[0];
  }  //!< @brief Returns the ZYX Euler yaw of the current attitude estimate (1st
     //!of the three ZYX Euler angles).
  double eulerPitch() {
    if (!m_eulerValid) {
      updateEuler();
    }
    return m_Ehat[1];
  }  //!< @brief Returns the ZYX Euler pitch of the current attitude estimate
     //!(2nd of the three ZYX Euler angles).
  double eulerRoll() {
    if (!m_eulerValid) {
      updateEuler();
    }
    return m_Ehat[2];
  }  //!< @brief Returns the ZYX Euler roll of the current attitude estimate
     //!(3rd of the three ZYX Euler angles).
  double fusedYaw() {
    if (!m_fusedValid) {
      updateFused();
    }
    return m_Fhat[0];
  }  //!< @brief Returns the fused yaw of the current attitude estimate (1st of
     //!the fused angles).
  double fusedPitch() {
    if (!m_fusedValid) {
      updateFused();
    }
    return m_Fhat[1];
  }  //!< @brief Returns the fused pitch of the current attitude estimate (2nd
     //!of the fused angles).
  double fusedRoll() {
    if (!m_fusedValid) {
      updateFused();
    }
    return m_Fhat[2];
  }  //!< @brief Returns the fused roll of the current attitude estimate (3rd of
     //!the fused angles).
  bool fusedHemi() {
    if (!m_fusedValid) {
      updateFused();
    }
    return m_FhatHemi;
  }  //!< @brief Returns the hemisphere of the current attitude estimate
     //!(boolean 4th parameter of the fused angles representation, where `true`
     //!implies `1` and `false` implies `-1`).

  // Get/set functions for the gyroscope bias
  void getGyroBias(double b[]) const {
    for (int i = 0; i < 3; i++) b[i] = m_bhat[i];
  }  //!< @brief Returns the current estimated gyro bias (Note: `b[]` must have
     //!room for 3 elements).
  void setGyroBias(const double b[]) {
    for (int i = 0; i < 3; i++) m_bhat[i] = b[i];
  }  //!< @brief Resets the current estimated gyro bias to a particular vector
     //!value (Note: `b[]` must have 3 elements).
  void setGyroBias(double bx, double by, double bz) {
    m_bhat[0] = bx;
    m_bhat[1] = by;
    m_bhat[2] = bz;
  }  //!< @brief Resets the current estimated gyro bias to a particular vector
     //!value.

  // Get/set functions for the magnetometer calibration
  void getMagCalib(double mt[]) const {
    for (int i = 0; i < 3; i++) mt[i] = m_magCalib[i];
  }  //!< @brief Returns the current magnetometer calibration vector. This
     //!should be the value of `(magX,magY,magZ)` that corresponds to a true
     //!orientation of identity (Note: `mt[]` must have room for 3 elements).
  void setMagCalib(const double mt[]) {
    for (int i = 0; i < 3; i++) m_magCalib[i] = mt[i];
  }  //!< @brief Sets the magnetometer calibration vector to use in the update
     //!functions. This should be the value of `(magX,magY,magZ)` that
     //!corresponds to a true orientation of identity (Note: `mt[]` must have 3
     //!elements).
  void setMagCalib(double mtx, double mty, double mtz = 0.0) {
    m_magCalib[0] = mtx;
    m_magCalib[1] = mty;
    m_magCalib[2] = mtz;
  }  //!< @brief Sets the magnetometer calibration vector to use in the update
     //!functions. This should be the value of `(magX,magY,magZ)` that
     //!corresponds to a true orientation of identity.

  // Get/set functions for the quick learning lambda parameter
  double getLambda() const {
    return m_lambda;
  }  //!< @brief Returns the current value of the quick learning parameter,
     //!&lambda;. This will always be on the unit interval `[0,1]`, and
     //!specifies the factor for linear interpolation between the standard
     //!(`Kp`, `Ti`) and quick learning (`KpQuick`, `TiQuick`) PI gains.
     //!&lambda; is auto-incremented in each call to `update()` in inverse
     //!proportion to `QLTime`.
  void resetLambda() {
    setLambda(0.0);
  }  //!< @brief Restarts (activates) quick learning by setting &lambda; to
     //!zero.
  void setLambda(double value = 1.0) {
    m_lambda = (value >= 1.0 ? 1.0 : (value <= 0.0 ? 0.0 : value));
  }  //!< @brief Sets &lambda; to the desired value, by default this is `1.0`,
     //!which turns quick learning off. Values outside of `[0,1]` are coerced.

  // Get/set functions for the configuration variables
  void getPIGains(double &Kp, double &Ti, double &KpQuick,
                  double &TiQuick);  //!< @brief Returns the currently set
                                     //!attitude estimator PI gains. Refer to
                                     //!`getLambda()` for more information on
                                     //!the PI gains (Note: The values are
                                     //!passed out of this function via the four
                                     //!reference parameters).
  void setPIGains(double Kp, double Ti, double KpQuick,
                  double TiQuick);  //!< @brief Sets the PI gains to use in both
                                    //!normal situations and for quick learning
                                    //!(Note: Each `Kp/Ti` pair must be a pair
                                    //!of positive values or the respective pair
                                    //!is not updated).
  double getQLTime() const {
    return m_QLTime;
  }  //!< @brief Returns the currently set quick learning time. A rate at which
     //!to auto-increment &lambda; is calculated so that quick learning fades
     //!over into standard operation in exactly `QLTime` seconds.
  void setQLTime(double QLTime) {
    if (QLTime > 0.0) m_QLTime = QLTime;
  }  //!< @brief Sets the quick learning time to use. See `getQLTime()` for more
     //!details (Note: Non-positive values of `QLTime` are ignored by this
     //!function).

  // Public estimation update functions
  void update(double dt, double gyroX, double gyroY, double gyroZ, double accX,
              double accY, double accZ, double magX, double magY,
              double magZ);  //!< @brief Updates the current attitude estimate
                             //!based on new sensor measurements and the amount
                             //!of elapsed time since the last update.

#ifndef ATT_EST_PRIVATE_ARE_AVAILABLE
 private:  // <-- This private accessor can be pre-processored out in order to
           // be able to run the special tests in test_attitude_estimator.cpp!
#endif
  // Private estimation update functions
  void updateQy(double accX, double accY, double accZ, double magX, double magY,
                double magZ);  // Calculates a 'measured' orientation based on
                               // given sensor measurements
  void updateEuler();  // Takes the current value of m_Qhat, converts it into
                       // Euler angles, and stores the result in m_Ehat
  void updateFused();  // Takes the current value of m_Qhat, converts it into
                       // fused angles, and stores the result in m_Fhat and
                       // m_FhatHemi

  // Acc-only measurement resolution method
  AccMethodEnum m_accMethod;  // The method to use if we cannot use a mag
                              // measurement to resolve the corresponding acc
                              // measurement into a full 3D orientation

  // Configuration variables
  double m_Kp;       // Kp for standard operation (at lambda = 1)
  double m_Ti;       // Ti for standard operation (at lambda = 1)
  double m_KpQuick;  // Kp for quick learning (at lambda = 0)
  double m_TiQuick;  // Ti for quick learning (at lambda = 0)
  double m_QLTime;   // The nominal duration of quick learning when lambda is
                     // reset to zero

  // Magnetometer calibration
  double m_magCalib[3];  // The mag value corresponding to/calibrated at the
                         // identity orientation

  // Internal variables
  double m_lambda;  // The quick learning parameter (0 => Quick learning, 1 =>
                    // Normal operation, In-between => Faded PI gains with
                    // m_lambda as the interpolation constant)
  double m_Qy[4], m_Qtilde[4],
      m_Qhat[4];  // Quaternions: Format is (q0,qvec) = (w,x,y,z), m_Qhat must
                  // *always* be a unit quaternion (or within a few eps of it)!
  double m_dQ[4],
      m_dQold[4];  // Quaternion derivatives: Format is (dw,dx,dy,dz)
  double m_w[3], m_wold[3], m_bhat[3], m_omega[3],
      m_base[3];     // 3D vectors: Format is (x,y,z)
  double m_Ry[9];    // Rotation matrix: The numbering of the 3x3 matrix entries
                     // is left to right, top to bottom
  double m_Ehat[3];  // Euler angles: Format is (psi,theta,phi) =
                     // (yaw,pitch,roll) and follows the ZYX convention
  double m_Fhat[3];  // Fused angles: Format is (psi,theta,phi) =
                     // (yaw,pitch,roll) and follows the standard definition of
                     // fused angles
  bool m_FhatHemi;   // Fused angles: Extra fourth hemisphere parameter, true is
                     // taken to mean 1 (positive z hemisphere), and false is
                     // taken to mean -1 (negative z hemisphere)
  bool m_eulerValid, m_fusedValid;  // Flags specifying whether the currently
                                    // stored Euler and fused angle
                                    // representations are up to date with the
                                    // quaternion stored in m_Qhat
};
}

#endif /* ATTITUDE_ESTIMATOR_H */
// EOF
