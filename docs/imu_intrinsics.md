# Example: Estimating IMU Intrinsics

For this example I am using a dataset recorded with the GoPro9. Please read the paper [11] on how to record the dataset and details on how the method works.
Remember to let the camera stand still for >= 10-15 seconds in the beginning. The IMU bias will be estimated in this phase.
This is also a parameter you need to supply to the calibration script.

You can get a sample dataset from: [link](https://drive.google.com/file/d/1XjtUX-4ZI0Ydkd2O3BWnaUzfmzm96He4/view?usp=share_link). 
The folder **static_imu_calib** contains an example file.
You can run the static imu calibration with. Remember to supply your build and source path of OpenICC to the script:
``` python
python python/static_multipose_imu_calibration.py --path_static_calib_dataset=/path/to/example_dataset/dataset3/static_multi_pose --initial_static_duration_s=10 --path_to_build=/your_path_to_openicc_build/applications --path_to_src=/your_path_to_openicc_src 
```

If successfull this script will output a json file **static_calib_result.json** containing IMU intrinsics for your camera. 
**You can supply this json file later to VI calibration using the --path_to_imu_intrinsics flag.**
