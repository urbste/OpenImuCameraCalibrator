# API Overview

OpenICC is organised as a C++ static library with a set of command-line applications orchestrated by Python scripts.

## C++ Library (`OpenImuCameraCalibrator`)

### Core modules

| Header | Purpose |
|--------|---------|
| `core/board_extractor.h` | Charuco / Radon / AprilTag corner extraction |
| `core/camera_calibrator.h` | Bundle-adjustment based intrinsic calibration |
| `core/imu_camera_calibrator.h` | Continuous-time spline VI calibration |
| `core/imu_to_camera_rotation_estimator.h` | Initial rotation + time-offset estimation |
| `core/pose_estimator.h` | PnP pose estimation from extracted corners |
| `core/static_imu_calibrator.h` | Static multi-pose IMU intrinsic calibration |
| `core/spline_trajectory_estimator.h` | SE3/R3 spline trajectory representation |

### I/O modules

| Header | Purpose |
|--------|---------|
| `io/read_camera_calibration.h` | Load JSON camera calibration |
| `io/write_camera_calibration.h` | Save JSON camera calibration |
| `io/read_telemetry.h` | Load generic IMU telemetry JSON |
| `io/read_scene.h` | Load BSON corner datasets |

### Utility modules

| Header | Purpose |
|--------|---------|
| `utils/types.h` | Common type aliases (Eigen, Sophus) |
| `utils/utils.h` | File I/O, interpolation, median helpers |
| `utils/intrinsic_initializer.h` | Camera model initializers for RANSAC |
| `utils/gyro_integration.h` | Simple gyro integration helpers |

## Command-line applications

All binaries are built in `build/applications/`.

| Application | Typical caller | Purpose |
|-------------|----------------|---------|
| `extract_board_to_json` | Python scripts | Extract corners from video / image folder to BSON/JSON |
| `calibrate_camera` | Python scripts | Intrinsic calibration from corner dataset |
| `estimate_camera_poses_from_checkerboard` | Python scripts | Estimate camera poses from corners + calibration |
| `estimate_imu_to_camera_rotation` | Python scripts | Initialize IMU-to-camera rotation and time offset |
| `continuous_time_imu_to_camera_calibration` | Python scripts | Full spline-based VI calibration |
| `static_imu_calibration` | Python scripts | Static multi-pose IMU calibration |
| `fit_allan_variance` | Manual / script | Fit Allan variance curves to gyro/accel data |
| `create_charuco_board` | Manual | Generate a printable Charuco board |

## Python scripts

| Script | Purpose |
|--------|---------|
| `run_gopro_calibration.py` | End-to-end GoPro calibration pipeline |
| `run_smartphone_calibration.py` | End-to-end smartphone calibration (Pilotguru) |
| `run_camera_calibration_from_images.py` | Camera-only calibration from a folder of images |
| `run_mynteye_calibration.py` | MyntEye calibration |
| `run_zed_calibration.py` | ZED 2i calibration |
| `get_imu_biases.py` | Estimate gyro / accel biases from a static sequence |
| `get_sew_for_dataset.py` | Compute spline error weighting from IMU data |
| `static_multipose_imu_calibration.py` | Multi-pose static IMU intrinsic calibration |
| `telemetry_converter.py` | Convert between telemetry formats (GoPro, Pilotguru, ZED, CSV) |
