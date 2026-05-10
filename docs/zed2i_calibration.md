# Example: Visual-Inertial Calibration of a ZED 2i

This tutorial walks through the calibration of a **ZED 2i** stereo camera using OpenICC.

## Prerequisites

- ZED 2i camera with IMU
- ZED Recorder or custom capture tool that outputs `.avi` video and `.jsonl` telemetry
- Printed Charuco or Radon board

## Data acquisition

1. Print the board from `resource/board.png` and attach it to a rigid surface.
2. Record three sequences with the ZED Recorder:
   - **cam**: slow motion around the board (20-30 s)
   - **imu_bias**: camera static on a table (10-20 s)
   - **cam_imu**: dynamic motion exciting all axes while keeping the board visible
3. Place the `.avi` videos and `.jsonl` telemetry files into the following structure:

```
MyZEDDataset
|-- cam
|     |-- video.avi
|     |-- telemetry.jsonl
|-- imu_bias
|     |-- video.avi
|     |-- telemetry.jsonl
|-- cam_imu
|     |-- video.avi
|     |-- telemetry.jsonl
```

## Run the calibration

```bash
python python/run_zed_calibration.py \
  --path_calib_dataset /path/to/MyZEDDataset \
  --checker_size_m 0.021 \
  --camera_model PINHOLE_RADIAL_TANGENTIAL \
  --known_gravity_axis Z
```

## Output

The script produces camera intrinsics, IMU biases, and the full IMU-to-camera transformation in the `cam_imu/` folder.
