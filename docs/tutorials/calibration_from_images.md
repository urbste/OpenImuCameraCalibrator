# Calibration from Images

This tutorial explains how to calibrate a camera using a folder of still images (`.png`) instead of a video file. This is useful when you have a frame sequence from a machine-vision camera or when you want to avoid video compression artifacts.

## Requirements

- A folder containing `.png` images of a Charuco or Radon board.
- Images should have good lighting, minimal motion blur and cover many viewpoints.

## Workflow

The script `python/run_camera_calibration_from_images.py` handles the full pipeline:

1. **Copies** your original images into a temporary `timestamped_frames/` sub-folder.
2. **Renames** them with synthetic nanosecond timestamps so the C++ extractor can parse them.
3. **Extracts corners** using `extract_board_to_json`.
4. **Runs camera calibration** using `calibrate_camera`.

## Usage

```bash
python python/run_camera_calibration_from_images.py \
  --path_calib_dataset /path/to/your/image_folder \
  --checker_size_m 0.021 \
  --camera_model DIVISION_UNDISTORTION \
  --image_downsample_factor 2 \
  --num_squares_x 10 \
  --num_squares_y 8
```

### Important parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `--path_calib_dataset` | Folder containing the `.png` images | *(required)* |
| `--checker_size_m` | Length of one checkerboard square in meters | `0.0015` |
| `--marker_length_m` | Length of one ArUco marker in meters (Charuco only) | half of checker size |
| `--camera_model` | Camera model to calibrate | `DOUBLE_SPHERE` |
| `--image_downsample_factor` | Downsample factor for speed | `2` |
| `--num_squares_x` | Number of squares horizontally | `35` |
| `--num_squares_y` | Number of squares vertically | `57` |
| `--board_type` | `charuco` or `radon` | `charuco` |

!!! note
    Your original images are **never overwritten**. The script works on copies inside `timestamped_frames/`.

## Output

- `cam_calib_<model>_<downsample>.json` – camera intrinsics
- `cam_calib_<model>_<downsample>.calibdata` – Theia reconstruction dataset
- `corners.json` – extracted corner observations
