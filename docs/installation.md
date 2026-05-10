# Installation

Tested on Ubuntu 18.04, 20.04 and 22.04.

## 1. OpenCV with contrib modules

Build [OpenCV](https://github.com/opencv/opencv) >= 4.5.0 **with** [contrib](https://github.com/opencv/opencv_contrib) modules. The contrib modules are needed for ArUco marker detection.

On Ubuntu 22.04 you can also install it from apt:

```bash
sudo apt-get install libopencv-dev libopencv-contrib-dev
```

## 2. Ceres Solver 2.1

```bash
git clone https://github.com/ceres-solver/ceres-solver
cd ceres-solver && git checkout 2.1.0
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
sudo make -j install
```

## 3. pyTheiaSfM fork

```bash
git clone https://github.com/urbste/pyTheiaSfM
cd pyTheiaSfM && git checkout 69c3d37
mkdir -p build && cd build
cmake .. && make -j
sudo make install
```

## 4. Build OpenICC

```bash
git clone https://github.com/urbste/OpenImuCameraCalibrator
mkdir -p build && cd build
cmake ..
make -j
```

## 5. Python dependencies

Create a Python >3.5 environment and install the requirements:

```bash
pip install -r requirements.txt
```

---

**Next step:** [Docker Quickstart](docker.md) or [GoPro Calibration Tutorial](tutorials/gopro_calibration.md).
