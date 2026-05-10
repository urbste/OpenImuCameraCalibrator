# OpenICC: An Open IMU and Camera Calibrator

[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue)](https://urbste.github.io/OpenImuCameraCalibrator)

OpenICC is an open-source toolbox for the accurate calibration of action cameras (e.g. GoPro), smartphones and other visual-inertial sensors for geometric vision tasks like Structure-from-Motion, Photogrammetry and SLAM.

Modern action cameras are equipped with various sensors like IMUs (accelerometer, gyroscope and magnetometer) and GPS. However the calibration data (e.g. camera projection and IMU to camera transformations) is often not available from the manufacturer.

## What you can do

- **Camera intrinsic calibration** using several wide-angle and fisheye models:
  - Fisheye [6]
  - Division Undistortion [5]
  - Field-of-View [3]
  - Double Sphere [2]
  - Extended Unified [4]
  - Pinhole
  - Pinhole with radial-tangential distortion
- **Extract telemetry data** integrated in MP4 video files (GoPro GPMF, Pilotguru, ZED, generic CSV/JSON).
- **Calibrate the Camera-to-IMU rotation matrix** and find the dataset-dependent **time offset**.
- **Perform continuous-time batch optimization** to find the full transformation matrix between IMU and camera using B-splines.
- **IMU intrinsic calibration** (scale, misalignment, axis alignment) using the method described in [11].
- **[Experimental] Calibrate the rolling-shutter line delay**.

## Documentation

Full documentation is available at **[urbste.github.io/OpenImuCameraCalibrator](https://urbste.github.io/OpenImuCameraCalibrator)**.

Quick links:

- [Installation](https://urbste.github.io/OpenImuCameraCalibrator/installation/)
- [Docker Quickstart](https://urbste.github.io/OpenImuCameraCalibrator/docker/)
- [GoPro Calibration Tutorial](https://urbste.github.io/OpenImuCameraCalibrator/tutorials/gopro_calibration/)
- [Smartphone Calibration Tutorial](https://urbste.github.io/OpenImuCameraCalibrator/tutorials/samsung_s20_calibration/)
- [Calibration from Images](https://urbste.github.io/OpenImuCameraCalibrator/tutorials/calibration_from_images/)
- [Supported Camera Models](https://urbste.github.io/OpenImuCameraCalibrator/camera_models/)

## Results

This section provides some results for my two GoPro cameras (6 and 9). You can use this to verify your own results or use them as initial values for your application. So far I have been setting them to FullHD with wide FoV and 30/60 fps. This is probably the most common setting that people use.

### Camera Calibration

| Dataset | Camera | Setting | Camera model | Intrinsics (f, cx, cy) | Reproj error |
|---|---|---|---|---|---|
| 1 | GoPro 9 | 960x540 / 60fps / Wide | Division Undistortion | (437.13, 489.07, 270.87) Dist: -1.4386e-06 | 0.31 |
| 2 | GoPro 9 | 960x540 / 60fps / Wide | Extended Unified | (437.97, 489.47, 272.02) Alpha: 0.5115 Beta: 1.062 | 0.209 |
| 3 | GoPro 9 | 960x540 / 60fps / Wide | Fisheye | (435.45, 479.12, 274.46) d1:0.05 d2:0.07 d3:-0.11 d4:0.05 | 0.24 |
| 4 | GoPro 6 | 960x540 / 60fps / Wide | Division Undistortion | (438.59, 480.80, 274.80) Dist: -1.47079e-06 | 0.09 |
| 5 | GoPro 6 | 960x540 / 60fps / Wide | Double Sphere | (342.43, 472.60, 273.88) XI: -0.215 Alpha 0.5129 | 0.16 |
| 6 | GoPro 6 | 960x540 / 60fps / Wide | Fisheye | (439.13, 479.66, 273.19) d1: 0.046, d2: 0.064, d3:-0.10, d4: 0.052 | 0.17 |
| 7 | GoPro 6 | 960x540 / 30fps / Wide | Division Undistortion | (436.06, 481.87, 272.58) dist: -1.468e-6 | 0.16 |

### IMU to Camera Calibration

| Dataset | Time offset IMU to camera | dt_r3 / dt_so3 | T_camera_to_imu (qw,qx,qy,qz) (tx,ty,tz)_m | RS Line delay init / calib | Final mean reproj error |
|---|---|---|---|---|---|
| 1 | -0.0813s | 0.128/0.056 | (0.0048,-0.006,-0.7076,0.7065),(0.0069,-0.0217, 0.001) | 30.895 / 31.62 | 0.84 |
| 2 | -0.0813s | 0.072/0.048 | (0.005,-0.0068,-0.7083,0.7057),(0.0021, -0.018,-0.004) | 30.895 / 36.56 | 0.82 |
| 3 | -0.0815s | 0.089/0.050 | (0.0001,-0.0002,0.7100,-0.7040),(0.009,-0.0182,-0.001) | 30.895 / 33.38 | 0.83 |
| 4 | -0.0129s | 0.15/0.062 | (-0.005,0.003,-0.706,0.7080),(0.009, -0.019, 0.012) | 30.895 / 29.58 | 0.79 |
| 5 | -0.0127s | 0.060/0.051 | (0.0007,-0.007,0.705,-0.7085),(0.005,-0.017, 0.008) | 30.895 / 26.03 | 0.59 |
| 6 | -0.0127s | 0.15/0.054 | (0.006,-0.006,0.706,-0.7072),(0.007,-0.030, 0.010) | 30.895 / 28.33 | 0.66 |
| 7 | -0.0129s | 0.056/0.035 | (-0.002,-0.0026,0.7049,-0.7092),(0.0216,-0.0165, 0.0108) | 61.79 / 61.76 | 0.9 |

### Some SLAM Examples using ORB-SLAM3

- [GoPro9_25fps_1080](https://youtu.be/0wIqkUEjhiw)
- [GoPro9_50fps_1080](https://youtu.be/IOpty7u7_04)
- [GoPro9_25fps_1440_maxlens_fisheye](https://youtu.be/Phw_OVP6sxI)
- [ORB-SLAM3 fork](https://github.com/urbste/ORB_SLAM3/)

## Quick Start

Tested on Ubuntu 18.04, 20.04 and 22.04.

```bash
# 1. Dependencies: OpenCV >= 4.5 (with contrib), Ceres >= 2.1, pyTheiaSfM fork
# See full instructions in the documentation.

# 2. Build OpenICC
git clone https://github.com/urbste/OpenImuCameraCalibrator
cd OpenImuCameraCalibrator && mkdir -p build && cd build
cmake .. && make -j

# 3. Python dependencies
pip install -r requirements.txt
```

### Docker

```bash
docker build -t openicc .
docker run -it --rm -v $(pwd):/home -v /path/to/dataset:/dataset openicc
```

Run a calibration inside the container:

```bash
cd /home
python3 python/run_gopro_calibration.py \
  --path_calib_dataset /dataset/dataset3/ \
  --path_to_build ../OpenImuCameraCalibrator/build/applications/
```

## Breaking Changes

- **2024-07**: Removed JavaScript requirement. Please install `py_gpmf_parser` from now on. Updated Dockerfile and dependencies to Ceres 2.1.0+ (now compatible with Ceres 2.2 Manifold API).
- **2023-06**: Initial release.

## Acknowledgements

This library would not have been possible without these great open-source projects:

- [TheiaSfM](http://theia-sfm.org) – Camera models and optimization
- [Basalt-Headers](https://github.com/borglab/basalt) – Spline implementation and optimization
- [Lie Group Cumulative B-Splines](https://gitlab.com/tum-vision/lie-spline-experiments) – Lie Splines
- [InertialScale](https://github.com/jannemus/InertialScale) – IMU to camera time offset and rotation matrix initialization
- [OpenCV](https://opencv.org/) – Computer vision framework
- [Sophus](https://github.com/strasdat/Sophus) – C++ Lie groups
- [Kontiki](https://github.com/hovren/kontiki) – Spline error weighting, VISfM
- [IMU-TK](https://github.com/Kyle-ak/imu_tk) – Static multi-pose IMU calibration
- [Allan variance](https://github.com/gaowenliang/imu_utils) – Gyro noise characterization

## Literature

### Libraries
- [1] Theia Multiview Geometry Library: Tutorial & Reference

### Camera Models
- [2] Usenko et al., "The Double Sphere Camera Model", 3DV 2018
- [3] Devernay & Faugeras, "Straight lines have to be straight", MVA 2001
- [4] Khomutenko et al., "An Enhanced Unified Camera Model", IEEE RA-L 2016
- [5] Fitzgibbon, "Simultaneous Linear Estimation of Multiple View Distortion", CVPR 2001
- [6] Kannala & Brandt, "A Generic Camera Model and Calibration Method", TPAMI 2006

### IMU Calibration
- [11] Tedaldi et al., "A Robust and Easy to Implement Method for IMU Calibration without External Equipments", ICRA 2014

### Misc
- [7] Mustaniemi et al., "Inertial-Based Scale Estimation for Structure from Motion on Mobile Devices", IROS 2017
- [8] Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR 2020
- [9] Larsson et al., "Making Minimal Solvers for Absolute Pose Estimation Compact and Robust", ICCV 2017
- [10] Ovrén & Forssén, "Spline Error Weighting for Robust Visual-Inertial Fusion", CVPR 2018

## Citation

If this tool helped you and you are using it in your work, please consider citing it:

```bibtex
@misc{OpenICC,
  author = {Steffen Urban},
  title = {OpenICC: An Open IMU and Camera Calibrator},
  howpublished = "\url{https://github.com/urbste/OpenImuCameraCalibrator}",
}
```
