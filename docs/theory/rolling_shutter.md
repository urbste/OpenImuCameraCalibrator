# Rolling Shutter Calibration

Rolling-shutter (RS) cameras expose each row of pixels at a slightly different time. For fast motions this causes geometric distortions (skew, wobble) that must be compensated for accurate visual-inertial fusion.

## Model

OpenICC models the RS effect with a per-row time offset:

\[
t_{row} = t_{frame} + \tau \cdot (row - row_{center})
\]

where \(\tau\) is the **line delay** in seconds per row.

## Current status

Rolling-shutter line-delay calibration is marked as **experimental**. Early experiments showed that the parameter is identifiable, but convergence can be sensitive to:

- Very fast shutter speeds (short exposure).
- High excitation of all rotation and translation axes.
- Accurate corner extraction (no motion blur).

If you enable it (`--calib_cam_line_delay=1`) make sure your calibration sequence satisfies the conditions above and inspect the final reprojection error. In many cases fixing the line delay to the manufacturer readout time (or leaving it at zero for global-shutter sensors) yields more stable results.

## Usage

```bash
python python/run_gopro_calibration.py \
  --path_calib_dataset /dataset \
  --calib_cam_line_delay 1
```
