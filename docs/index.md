# OpenICC: An Open IMU and Camera Calibrator

OpenICC is an open-source toolbox for the accurate calibration of action cameras (e.g. GoPro), smartphones and other visual-inertial sensors. Modern devices are equipped with various sensors like IMUs (accelerometer, gyroscope and magnetometer) and GPS. However the calibration data (e.g. camera projection and IMU to camera transformations) is often not available from the manufacturer.

This is where **OpenImuCameraCalibrator** comes in.

## What you can do

- **Camera intrinsic calibration** using several wide-angle and fisheye models:
  - Fisheye (Kannala-Brandt)
  - Division Undistortion
  - Field-of-View (Devernay-Faugeras)
  - Double Sphere
  - Extended Unified
  - Pinhole
  - Pinhole with radial-tangential distortion
- **Telemetry extraction** from MP4 video files (GoPro GPMF, Pilotguru, ZED, generic CSV/JSON).
- **Camera-to-IMU rotation matrix** and **time offset** estimation.
- **Continuous-time batch optimization** to find the full transformation matrix between IMU and camera using B-splines.
- **IMU intrinsic calibration** (scale, misalignment, axis alignment) using multi-pose static calibration.
- **[Experimental] Rolling-shutter line delay** calibration.

## Quick links

- [Installation](installation.md)
- [Docker Quickstart](docker.md)
- [GoPro Calibration Tutorial](tutorials/gopro_calibration.md)
- [Smartphone Calibration Tutorial](tutorials/smartphone_calibration.md)
- [Calibration from Images](tutorials/calibration_from_images.md)

## Citation

If this tool helped you and you are using it in your work, please consider citing it:

```bibtex
@misc{OpenICC,
  author = {Steffen Urban},
  title = {OpenICC: An Open IMU and Camera Calibrator},
  howpublished = "\url{https://github.com/urbste/OpenImuCameraCalibrator}",
}
```
