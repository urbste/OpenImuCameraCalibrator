# Acknowledgements & Citation

This library would not have been possible without these great open-source projects:

- [TheiaSfM](http://theia-sfm.org) – Camera models and optimization
- [Basalt-Headers](https://github.com/borglab/basalt) – Spline implementation and optimization
- [Lie Group Cumulative B-Splines](https://gitlab.com/tum-vision/lie-spline-experiments) – Lie splines
- [InertialScale](https://github.com/jannemus/InertialScale) – IMU to camera time offset and rotation matrix initialization
- [OpenCV](https://opencv.org/) – Computer vision framework
- [Sophus](https://github.com/strasdat/Sophus) – C++ Lie groups
- [Kontiki](https://github.com/hovren/kontiki) – Spline error weighting, VISfM
- [IMU-TK](https://github.com/Kyle-ak/imu_tk) – Static multi-pose IMU calibration
- [Allan variance](https://github.com/gaowenliang/imu_utils) – Gyro noise characterization

## Literature

### Libraries
1. Theia Multiview Geometry Library: Tutorial & Reference

### Camera models
2. Usenko et al., "The Double Sphere Camera Model", 3DV 2018
3. Devernay & Faugeras, "Straight lines have to be straight", MVA 2001
4. Khomutenko et al., "An Enhanced Unified Camera Model", IEEE RA-L 2016
5. Fitzgibbon, "Simultaneous Linear Estimation of Multiple View Distortion", CVPR 2001
6. Kannala & Brandt, "A Generic Camera Model and Calibration Method", TPAMI 2006

### IMU calibration
11. Tedaldi et al., "A Robust and Easy to Implement Method for IMU Calibration without External Equipments", ICRA 2014

### Misc
7. Mustaniemi et al., "Inertial-Based Scale Estimation for Structure from Motion on Mobile Devices", IROS 2017
8. Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR 2020
9. Larsson et al., "Making Minimal Solvers for Absolute Pose Estimation Compact and Robust", ICCV 2017
10. Ovrén & Forssén, "Spline Error Weighting for Robust Visual-Inertial Fusion", CVPR 2018

## Citation

If you use OpenICC in your work, please consider citing it:

```bibtex
@misc{OpenICC,
  author = {Steffen Urban},
  title = {OpenICC: An Open IMU and Camera Calibrator},
  howpublished = "\url{https://github.com/urbste/OpenImuCameraCalibrator}",
}
```
