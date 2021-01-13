# OpenGoProCalibrator
I developed this repository to experiment with the accurate calibration of action cameras (especially GoPro cameras) to use them for geometric vision tasks like Structure-from-Motion, Photogrammetry and SLAM. Modern action cameras are equipped with various sensors like IMUs (accelerometer, gyroscope and magnetometer) and GPS. However the calibration data (e.g. camera projection and IMU to camera transformations) are not available.

This is where the OpenGoProCalibrator comes in. With this toolbox you can:
* Calibrate the intrinsics of your a GoPro camera. Supported **camera models** are:
  * Fisheye [6]
  * Division Undistortion [5]
  * Field-of-View [3]
  * Double Sphere [2]
  * Extended Unified [4]
  * Pinhole
  * RadTan pinhole
* Extract the meta data integrated in the MP4 video file (called **telemetry data**)
* Calibrate the **Camera to IMU rotation matrix** and find the dataset dependent **time offset**
* Perform full **continuous time batch optimization** to find the full transformation matrix between IMU and camera

## Example: Calibration of a GoPro
For this example I am using a GoPro 9. To calibrate the camera and the IMU to camera transformation we will use the following script: **python/run_gopro_calibration.
The py**
```Python
python python/run_gopro_calibration.py --path_calib_dataset=/your/path --checker_size_m=0.022 --image_downsample_factor=1 --camera_model=DIVISION_UNDISTORTION
```

## Results
This section provides some results on my cameras. You can use this to verify your own results or use them as initial values for your application.
Dataset | Camera | Setting | Camera model | Intrinsics (f, cx, cy) | Time offset IMU to camera | dt_r3 / dt_so3 | T_camera_to_imu (qw,qx,qy,qz) (tx,ty,tz)_m | Final mean reproj error |
|--|--|--|--|--|--|--|--|--|
| Dataset 2 | GoPro 9 | 1920x1080 / 60fps / Wide | Division Undistortion | (869.96, 975.68, 543.73) Dist: -3.586e-07 | -0.0813s | 0.01/0.0573 | (0.005904,-0.006493,-0.7094,0.70472), (-0.0016,-0.0185,-0.0054) | 1.86 pixel |
| Dataset 2 | GoPro 9 | 1920x1080 / 60fps / Wide | Extended Unified | (872.00, 975.18, 543.479) Alpha: 0.4589 Beta: 1.2047  | -0.0813s | 0.01/0.0573 | (0.00573,-0.00617,-0.7097,0.7044),(-0.0015, -0.0185,-0.0053) | 1.86 pixel |
| Dataset 3 | GoPro 9 | 1920x1080 / 60fps / Wide | Extended Unified | (871.94, 960.91, 551.49) Alpha: 0.60101 Beta: 0.8661  | -0.0815s | 0.01/0.0599 | (0.0016,-0.0004,-0.7119,0.7022),(-0.0005,-0.0188,-0.0061) | 1.94 pixel |
| Dataset 1 | GoPro 9 | 960x540 / 120fps / Wide | Extended Unified | (438.51, 482.44, 276.41) Alpha: 0.5042 Beta: 1.1104  | -0.0564s | 0.01/0.0667 | (0.0027,-0.0005,-0.7107,0.703428),(-0.0006, -0.0212,  -0.0046) | 0.7411 pixel |
| Dataset 1 | GoPro 6 | 960x540 / 60fps / Wide | Division Undistortion | (438.18, 274.54, 276.41) Dist: -1.4747e-06  | -0.0129s | 0.025/0.089| (0.011,-0.0070,0.7065,-0.7075),(0.0012, -0.0167, 0.009) | 1.22 pixel |
| Dataset 2 | GoPro 6 | 960x540 / 60fps / Wide | Division Undistortion | (437.35, 474.16, 274.48) Dist: -1.4558e-06  | -0.0129s | 0.01/0.062| (0.009,-0.009,0.705,-0.7085),(0.006,-0.0156, 0.006) | 0.80 pixel |


## Installation instructions
ToDo



## Acknowlegements
This library would not have been possible without these great OpenSource projects:
* [TheiaSfM](http://theia-sfm.org) - Camera models and optimization
* [Basalt-Headers]() - Spline implementation and optimization
* [Lie Group Cumulative B-Splines](https://gitlab.com/tum-vision/lie-spline-experiments) - Lie Splines
* [InertialScale](https://github.com/jannemus/InertialScale) - IMU to camera time offset and rotation matrix initialization
* [OpenCV](https://opencv.org/) - Well OpenCV ;)
* [Sophus](https://github.com/strasdat/Sophus) - C++ Lie groups
* [GoPro-Telemetry](https://github.com/JuanIrache/gopro-telemetry) - Great JavaScript GoPro telemetry extractor
* [Kontiki](https://github.com/hovren/kontiki) - Spline error weighting, VISFM
## Literature and Code
### Libraries
   * [1] Theia Multiview Geometry Library: Tutorial & Reference
### Camera models:
   * [2] The **Double Sphere** Camera Model, V. Usenko and N. Demmel and D. Cremers, In 2018 International Conference on 3D Vision (3DV)
   * [3] **FoV model**: F. Devernay and O. Faugeras. Straight lines have to be
straight. Machine vision and applications, 13(1):14–24, 2001.
   * [4] **Extended unified**: B. Khomutenko, G. Garcia, and P. Martinet. An enhanced
unified camera model. IEEE Robotics and Automation Let-
ters, 1(1):137–144, Jan 2016.
   * [5] **Division undistortion**: "Simultaneous linear estimation of multiple view distortion" by Andrew Fitzgibbon, CVPR 2001.
   * [6] **Fisheye**: Kannala, Juho, and Sami S. Brandt. "A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses." IEEE transactions on pattern analysis and machine intelligence 28.8 (2006): 1335-1340.
### Misc:
   * [7] Mustaniemi J., Kannala J., Särkkä S., Matas J., Heikkilä J. "Inertial-Based Scale Estimation for Structure from Motion on Mobile Devices", International Conference on Intelligent Robots and Systems (IROS), 2017
   * [8] Efficient Derivative Computation for Cumulative B-Splines on Lie Groups, C. Sommer, V. Usenko, D. Schubert, N. Demmel, D. Cremers, In 2020 Conference on Computer Vision and Pattern Recognition (CVPR) 
   * [9] Larsson, Viktor, Zuzana Kukelova, and Yinqiang Zheng. "Making minimal solvers for absolute pose estimation compact and robust." Proceedings of the IEEE International Conference on Computer Vision. 2017.
   * [10] Hannes Ovrén and Per-Erik Forssén Spline Error Weighting for Robust Visual-Inertial Fusion In Proceedings of the IEEE on Computer Vision and Pattern Recognition (CVPR) June 2018




## Working on / ToDo / Contributions Welcome
* [ ] Code cleanup, add license header
* [x] use different initialization for linear mode
* [ ] Beautify logs
* [ ] Add more camera models -> Scaramuzza omni model, ...
* [ ] Use more generic JSON meta data interface, so that others cameras can be calibrated that come with different telemetry formats
* [ ] Rolling shutter calibration -> first resolve problems with templated spline functions
* [ ] Model bias over time with a R3 spline
* [ ] Pose estimation with UPNP or MLPnP to support arbitraty camera types. Right now: undistortion and then using vanilla PnP --> will lead to problems for Ultra Wide Angle fisheye lenses (e.g. GoPro Max or potentially Max lens mod)
* [ ] use only bearings in spline reproj error -> local tangent reprojection error
* [ ] More accurate checkerboard (OpenCV has findCheckerboardSB. I integrated a first version but results were weird. Re-check...)
* [ ] Model IMU - camera time difference continously (or at least optimize for different time windows)
* [ ] Extend to multi-camera systems
* [ ] Docker?
* [ ] Put together a little paper on how this all works
