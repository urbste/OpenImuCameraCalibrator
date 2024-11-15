# Breaking Changes

* **1/7/2024**: 
  * Removed javascript requirement. Please install py_gpmf_parser from now on.
  * Fixed Dockerfile and updated Readme with new installation instructions
  * Updated dependencies to Ceres 2.1.0
  * Updated pyTheiaSfM version --> Install new version (either master or 69c3d37).

# OpenICC: An Open IMU and Camera Calibrator

I developed this repository to experiment with the accurate calibration of action cameras (e.g. GoPro cameras) to use them for geometric vision tasks like Structure-from-Motion, Photogrammetry and SLAM. Modern action cameras are equipped with various sensors like IMUs (accelerometer, gyroscope and magnetometer) and GPS. However the calibration data (e.g. camera projection and IMU to camera transformations) is not available.

This is where the OpenImuCameraCalibrator comes in. With this toolbox you can:
* Calibrate the intrinsics of your a GoPro camera. Supported **camera models** are:
  * Fisheye [6]
  * Division Undistortion [5]
  * Field-of-View [3]
  * Double Sphere [2]
  * Extended Unified [4]
  * Pinhole
  * Pinhole with radial-tangential distortion
* Extract the meta data integrated in the MP4 video file (called **telemetry data**)
* Calibrate the **Camera to IMU rotation matrix** and find the dataset dependent **time offset**
* Perform full **continuous time batch optimization** to find the full transformation matrix between IMU and camera
* Do an intrinsic calibration of your IMU using the method described in [11]
* [Experimental] Calibrate the **rolling shutter line delay** (not really working yet)

## Results
This section provides some results for my two GoPro cameras (6 and 9). You can use this to verify your own results or use them as initial values for your application. So far I have been setting them to FullHD with wide FoV and 30/60 fps. This is probably the most common setting that people use.
### Camera Calibration
Dataset | Camera | Setting | Camera model | Intrinsics (f, cx, cy) | Reproj error | 
|--|--|--|--|--|--|
| 1 | GoPro 9 | 960x540 / 60fps / Wide | Division Undistortion | (437.13, 489.07, 270.87) Dist: -1.4386e-06 | 0.31 |
| 2 | GoPro 9 | 960x540 / 60fps / Wide | Extended Unified | (437.97, 489.47, 272.02) Alpha: 0.5115 Beta: 1.062  | 0.209 |
| 3 | GoPro 9 | 960x540 / 60fps / Wide | Fisheye | (435.45, 479.12, 274.46) d1:0.05 d2:0.07 d3:-0.11 d4:0.05  | 0.24 |
| 4 | GoPro 6 | 960x540 / 60fps / Wide | Division Undistortion | (438.59, 480.80, 274.80) Dist: -1.47079-06  | 0.09 |
| 5 | GoPro 6 | 960x540 / 60fps / Wide | Double Sphere | (342.43, 472.60, 273.88) XI: -0.215 Alpha 0.5129 | 0.16 | 
| 6 | GoPro 6 | 960x540 / 60fps / Wide | Fisheye | (439.13, 479.66, 273.19) d1: 0.046, d2: 0.064, d3:-0.10, d4: 0.052 | 0.17 | 
| 7 | GoPro 6 | 960x540 / 30fps / Wide | Division Undistortion | (436.06, 481.87, 272.58) dist: -1.468e-6 | 0.16 | 

### IMU to Camera Calibration

Dataset | Time offset IMU to camera | dt_r3 / dt_so3 | T_camera_to_imu (qw,qx,qy,qz) (tx,ty,tz)_m | RS Line delay init / calib | Final mean reproj error |
|--|--|--|--|--|--|
1 | -0.0813s | 0.128/0.056 | (0.0048,-0.006,-0.7076,0.7065),(0.0069,-0.0217, 0.001) | 30.895 / 31.62 | 0.84 |
2 | -0.0813s | 0.072/0.048 | (0.005,-0.0068,-0.7083,0.7057),(0.0021, -0.018,-0.004)| 30.895 / 36.56 | 0.82 |
3 | -0.0815s | 0.089/0.050 | (0.0001,-0.0002,0.7100,-0.7040),(0.009,-0.0182,-0.001)| 30.895 / 33.38 | 0.83  |
4 | -0.0129s | 0.15/0.062| (-0.005,0.003,-0.706,0.7080),(0.009, -0.019, 0.012) | 30.895 /  29.58 | 0.79  |
5 | -0.0127s |  0.060/0.051| (0.0007,-0.007,0.705,-0.7085),(0.005,-0.017, 0.008) | 30.895 / 26.03 | 0.59  | 
6 | -0.0127s | 0.15/0.054| (0.006,-0.006,0.706,-0.7072),(0.007,-0.030, 0.010) | 30.895 /  28.33 |0.66  | 
7 | -0.0129s | 0.056/0.035| (-0.002,-0.0026,0.7049,-0.7092),(0.0216,-0.0165, 0.0108) | 61.79 / 61.76 | 0.9  | 

### Some SLAM Examples using ORB-SLAM3
[GoPro9_25fps_1080](https://youtu.be/0wIqkUEjhiw)

[GoPro9_50fps_1080](https://youtu.be/IOpty7u7_04)

[GoPro9_25fps_1440_maxlens_fisheye](https://youtu.be/Phw_OVP6sxI)

[ORB-SLAM3 fork](https://github.com/urbste/ORB_SLAM3/)

## Installation instructions

Tested on Ubuntu 18.04 and 20.04. and 22.04

1. Clone and build [OpenCV](https://github.com/opencv/opencv) >= 4.5.0 **with** [contrib](https://github.com/opencv/opencv) modules. Latter are needed for Aruco marker detection.
On Ubuntu 22.04 you can also just install it from apt:
``` bash
sudo apt-get install libopencv-dev libopencv-contrib-dev
``` 

2. Install [ceres 2.1](http://ceres-solver.org/installation.html)
``` bash
git clone https://github.com/ceres-solver/ceres-solver
git checkout 2.1.0
mkdir -p build && cd build && cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
sudo make -j install
```

3. Clone and build the [TheiaSfM fork](https://github.com/urbste/pyTheiaSfM).
``` bash
git clone https://github.com/urbste/pyTheiaSfM
cd pyTheiaSfM && git checkout 69c3d37 && mkdir -p build && cd build
cmake .. && make -j
sudo make install
```
4. Build this project
``` bash
git clone https://github.com/urbste/OpenImuCameraCalibrator
mkdir -p build && cd build && cmake ..
make -j
``` 

5. Create a python >3.5 environment (or use your local python - not recommended)
``` bash
pip install -r requirements.txt
```

### Dockerfile
Build the docker container
``` bash
docker build -t openicc .
```
Now you can mount the OpenICC folder to your docker container, as well as the folder that contains your calibration data (e.g. download [GoPro9 dataset](https://drive.google.com/file/d/1t2_hb4ko6llmVy_rZZHCi7S6D9-lYUa1/view?usp=drive_link) to /home/Downloads/GoPro9 )
``` bash
docker run -it --rm -v `pwd`:/home -v /home/Downloads/GoPro9:/dataset openicc
``` 
Finally you can run the calibration like this:
``` bash
cd /home
python3 python/run_gopro_calibration.py --path_calib_dataset /dataset/dataset3/ --path_to_build ../OpenImuCameraCalibrator/build/applications/
```

## Usage examples

- [Calibrate a GoPro9](docs/gopro_calibration.md)
- [Calibrate a SamsungS20FE](docs/samsung_s20_calibration.md)
- [Calibrate a GoPro IMU intrinsics](docs/imu_intrinsics.md)
- [Estimate a GoPro IMU noise parameters](docs/imu_noise_parameters.md)


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
* [IMU-TK](https://github.com/Kyle-ak/imu_tk) - Static multi pose imu calibration
* [Allan variance](https://github.com/gaowenliang/imu_utils) - Gyro noise characterization
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

### IMU calibration:
   * [11] D. Tedaldi, A. Pretto and E. Menegatti, "A Robust and Easy to Implement Method for IMU Calibration without  External Equipments". In: Proceedings of the IEEE International Conference on Robotics and Automation (ICRA  2014), May 31 - June 7, 2014 Hong Kong, China, Page(s): 3042 - 3049

### Misc:
   * [7] Mustaniemi J., Kannala J., Särkkä S., Matas J., Heikkilä J. "Inertial-Based Scale Estimation for Structure from Motion on Mobile Devices", International Conference on Intelligent Robots and Systems (IROS), 2017
   * [8] Efficient Derivative Computation for Cumulative B-Splines on Lie Groups, C. Sommer, V. Usenko, D. Schubert, N. Demmel, D. Cremers, In 2020 Conference on Computer Vision and Pattern Recognition (CVPR) 
   * [9] Larsson, Viktor, Zuzana Kukelova, and Yinqiang Zheng. "Making minimal solvers for absolute pose estimation compact and robust." Proceedings of the IEEE International Conference on Computer Vision. 2017.
   * [10] Hannes Ovrén and Per-Erik Forssén Spline Error Weighting for Robust Visual-Inertial Fusion In Proceedings of the IEEE on Computer Vision and Pattern Recognition (CVPR) June 2018

## Working on / ToDo / Contributions Welcome
v0.1
* [x] Code cleanup, add license header
* [x] use different initialization for linear mode
* [x] Use more generic JSON meta data interface, so that others cameras can be calibrated that come with different telemetry formats
* [x] Write example for the calibration of a Smartphone
* [x] Rolling shutter calibration -> first resolve problems with templated spline functions
* [x] Add readout time as a optimizable parameter -> not working well, probably residual weighting needs to be improved
* [x] Allan variance -> imu_utils
* [x] Support AprilTag board from Kalibr to record same datasets
* [x] Calibrate axis misalignment and so on with MutliPoseOptimization from imu_tk
* [x] Include imu axis and scale in spline error functions
* [x] Beautify logs
* [x] Cleanup

v0.2
* [x] Model bias over time with a R3 spline
* [ ] Pose estimation with UPNP or MLPnP to support arbitraty camera types. Right now: undistortion and then using vanilla PnP --> will lead to problems for Ultra Wide Angle fisheye lenses (e.g. GoPro Max or potentially Max lens mod)
* [ ] Pose estimation with RSPnP
* [ ] use only bearings in spline reproj error -> local tangent reprojection error
* [ ] Accurate checkerboard detector (OpenCV has findCheckerboardSB. I integrated a first version but results were weird. Re-check...)
* [ ] Extend to multi-camera systems
* [x] Docker?
* [ ] Add more camera models -> Scaramuzza omni model, ...
* [x] Integrate updated version of spline optimizer

misc
* [ ] Put together a little paper on how this all works


## Citation
If this tool helped you and you are using it in your work consider citing it as follows for now:
```
@misc{OpenICC,
  author = {Steffen Urban},
  title = {OpenICC: An Open IMU and Camera Calibrator},
  howpublished = "\url{https://github.com/urbste/OpenImuCameraCalibrator}",
}
```