# OpenICC: OpenImuCameraCalibrator
WORK IN PROGRESS

I developed this repository to experiment with the accurate calibration of action cameras (e.g. GoPro cameras) to use them for geometric vision tasks like Structure-from-Motion, Photogrammetry and SLAM. Modern action cameras are equipped with various sensors like IMUs (accelerometer, gyroscope and magnetometer) and GPS. However the calibration data (e.g. camera projection and IMU to camera transformations) is not available.

This is where the OpenImuCameraCalibrator comes in. With this toolbox you can:
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
* [Experimental] Calibrate the **rolling shutter line delay**

## Results
This section provides some results for my two GoPro cameras (6 and 9). You can use this to verify your own results or use them as initial values for your application. So far I have been setting them to FullHD with wide FoV and 30/60 fps. This is probably the most common setting that people use.
### Camera Calibration
Dataset | Camera | Setting | Camera model | Intrinsics (f, cx, cy) | Reproj error | 
|--|--|--|--|--|--|
| 1 | GoPro 9 | 960x540 / 60fps / Wide | Division Undistortion | (871.81, 980.123, 544.36) Dist: -3.588e-07 | 0.62 |
| 2 | GoPro 9 | 960x540 / 60fps / Wide | Double Sphere | (790.152, 978.640, 545.584) XI: -0.09694 Alpha: 0.52873  | 0.41 |
| 3 | GoPro 9 | 960x540 / 60fps / Wide | Extended Unified | (871.94, 960.91, 551.49) Alpha: 0.6010 Beta: 0.8661  | 0.41 |
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



## Installation instructions
ToDo

1. Clone and build OpenCV >= 4.5.0

2. Install ceres 2.0 http://ceres-solver.org/installation.html

3. Clone and build the TheiaSfM fork.
``` bash
git clone https://github.com/urbste/TheiaSfM
cd TheiaSfM && mkdir -p build && cd build
cmake .. && make -j4
sudo make install
```

4. Install nodejs (needed to extract GoPro telemetry)
``` bash
sudo apt install nodejs npm
``` 

5. Build this project
``` bash
git clone https://github.com/urbste/OpenImuCameraCalibrator
mkdir -p build && cd build && cmake ..
make -j
``` 

## Example: Visual-Inertial Calibration of a GoPro Camera
For this example I am using a GoPro 9. To calibrate the camera and the IMU to camera transformation we will use the following script: **python/run_gopro_calibration.py** 

0. Get the test data from here: [ipfs-link](https://ipfs.io/ipfs/QmVNrzBEVgfN63MneoAvvzsVGpihbuhz3BGTgKGMhuad1w). Then you can skip step 1. & 2. 

1. Data acquisition

   1.1 Print out the target: resource/board.png to a paper and attach it to something rigid (e.g. a wall). Measure the size of a black square in meter (e.g. 0.021m).
   
   1.2 Now record **3** videos.

   1.2.1 The first is to calibrate the camera.
        Move **SLOWLY** around the board. We do not want motion blur or the rolling shutter to influence the result. Record for about 20-30s. --> e.g. GH0000.MP4

   1.2.2 Place the GoPro on the floor or on a table and press record. Leave it there for 10-20s without touching it or walking around it. This video will be used to estimate the current IMU bias. We assume it to be fixed during the calibration. --> e.g. GH0001.MP4

   1.2.3 Finally record the last video. Again record the board and make sure that you have good lighting conditions. If possible set the shutter time of your GoPro the minimum (e.g. 1/480). If the board is barely visible your lighting condition is not good enough. Having a very fast shutter assures crisp corners and less motion blur. Also it should improve RS line delay calibration. --> e.g. GH0002.MP4
   - Excite all 3 axis -> 3 translation and 3 rotation.
   - Move fast, but not too fast (motion blur). 
   - Make sure that most of the board is visible

2. Create the following folder structure:

```
MyDataset
|-- cam
|     |-- GH0000.MP4
|   imu_bias
|     |-- GH0001.MP4
|   cam_imu
|     |-- GH0002.MP4
```

3. Run the calibration
``` python
python python/run_gopro_calibration.py --path_calib_dataset=/your/path/MyDataset --checker_size_m=0.021 --image_downsample_factor=2 --camera_model=DIVISION_UNDISTORTION
```
Also check out all the other parameters you can set!

4. The spline calibration in the end should converge smoothly after 8-15 iterations. If not, your recordings are probably not good enough to perform a decent calibration. Also have a look at the final spline fit to the IMU readings:
![SplineFit](resource/ExampleSplineFit.png)


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
v0.1
* [x] Code cleanup, add license header
* [x] use different initialization for linear mode
* [x] Use more generic JSON meta data interface, so that others cameras can be calibrated that come with different telemetry formats
* [ ] Beautify logs
* [x] Write example for the calibration of a Smartphone
v0.2
* [ ] Model time delay for optimization
* [x] Rolling shutter calibration -> first resolve problems with templated spline functions
* [x] Add readout time as a optimizable parameter -> not working well, probably residual weighting needs to be improved
* [ ] Model bias over time with a R3 spline
* [ ] Pose estimation with UPNP or MLPnP to support arbitraty camera types. Right now: undistortion and then using vanilla PnP --> will lead to problems for Ultra Wide Angle fisheye lenses (e.g. GoPro Max or potentially Max lens mod)
* [ ] Pose estimation with RSPnP
* [ ] use only bearings in spline reproj error -> local tangent reprojection error
* [ ] Accurate checkerboard detector (OpenCV has findCheckerboardSB. I integrated a first version but results were weird. Re-check...)
* [ ] Model IMU - camera time difference continously (or at least optimize for different time windows)
* [ ] Extend to multi-camera systems
* [ ] Docker?
* [ ] Put together a little paper on how this all works
* [ ] Add more camera models -> Scaramuzza omni model, ...
