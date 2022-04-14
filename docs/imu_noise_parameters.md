# Estimating IMU Noise Parameters
1. Record a > 2h video keeping your camera completely still. Use the lowest resolution and FPS setting, as we do not need the video feed for this. In addition, you can cover the lens. A video of a black screen will remain quite small.
2. Still this will create multiple video files, so we need to extract the telemetry from each and merge it to a large one.
3. Put all video files in a single folder
4. Use [merge_gopro_telemetry_from_folder.py](../python/merge_gopro_telemetry_from_folder.py) to concatenate all telemetry files into a single on
5. Finally run [fit_allan_variance](../applications/fit_allan_variance) binary on the concatenated telemetry file
6. This will give you noise density and random walk values for each axis x-y-z of gyroscope and accelerometer
7. Average these values and use them in your favorite VIO or SLAM, e.g. [ORB-SLAM3](https://github.com/urbste/ORB_SLAM3)
