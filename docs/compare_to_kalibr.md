## How to compare to Kalibr
Work-in-progress!

1. Download CDE package from [here](https://github.com/ethz-asl/kalibr/wiki/downloads)

2. Open AprilTag target on screen or print it from [here](https://github.com/ethz-asl/kalibr/wiki/downloads)

3. Create a AprilTag yaml file and enter size of tag

4. Find out your GoPro IMU noise params (see above)

5. Finally create a imu.yaml

Example from my GoPro 9 
``` yaml
#Accelerometers
accelerometer_noise_density: 1.5e-02   # White noise
accelerometer_random_walk:   4.5e-04   # Bias instability

#Gyroscopes
gyroscope_noise_density:     1.1e-03   # White noise
gyroscope_random_walk:       3.0e-05   # Bias instability

rostopic:                    /imu0      #the IMU ROS topic
update_rate:                 200.0      #Hz (for discretization of the values above)
```

6. Use python/extract_for_kalibr_bagcreator.py to extract telemetry and single images

7. Run kalibr_bagcreator

8. Run kalibr_calibrate_cameras

Use omni-radtan model.

9. Run kalibr_calibrate_imu_camera

with --time-calibration

