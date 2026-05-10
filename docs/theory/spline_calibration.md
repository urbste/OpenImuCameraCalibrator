# Spline-based Visual-Inertial Calibration

OpenICC performs continuous-time batch optimization using **SE(3) and R³ B-splines** to estimate the camera-to-IMU transformation. This approach was inspired by the work of Sommer et al. (CVPR 2020) on Lie-group cumulative B-splines and the Basalt spline implementation.

## Why splines?

Traditional discrete-time visual-inertial calibration treats each camera pose as an independent variable. For high-rate IMUs (e.g. 200 Hz) and cameras (e.g. 60 fps) this leads to:

- A very large number of pose variables.
- Difficulty in modelling the time offset and rolling-shutter readout time.

**Continuous-time splines** solve this by representing the trajectory as a smooth function of time. At any given timestamp we can query the pose, velocity and acceleration by evaluating the spline.

## What is optimized?

The calibration jointly optimizes:

- **Camera intrinsics** (fixed after camera calibration, but can be refined).
- **Spline control points** (SO3 and R3 knots) representing the sensor trajectory.
- **IMU-to-camera transformation** \(T_{ic}\) (rotation and translation).
- **Time offset** between IMU and camera clocks.
- **[Experimental] Rolling-shutter line delay**.
- **IMU biases** modelled as additional R³ splines over time.

## Spline error weighting

The optimizer balances reprojection errors, IMU accelerometer errors and gyroscope errors. The relative weighting is derived from the Allan variance of the IMU (see [IMU Noise Parameters](tutorials/imu_noise_parameters.md)).

## References

- [8] Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR 2020.
- [10] Ovrén & Forssén, "Spline Error Weighting for Robust Visual-Inertial Fusion", CVPR 2018.
