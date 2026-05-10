# Docker Quickstart

A Dockerfile is provided that installs all dependencies and builds the project automatically.

## Build the image

```bash
docker build -t openicc .
```

## Run the container

Mount the OpenICC folder as well as the folder that contains your calibration data (e.g. a GoPro9 dataset):

```bash
docker run -it --rm \
  -v $(pwd):/home \
  -v /home/Downloads/GoPro9:/dataset \
  openicc
```

## Run a calibration inside the container

```bash
cd /home
python3 python/run_gopro_calibration.py \
  --path_calib_dataset /dataset/dataset3/ \
  --path_to_build ../OpenImuCameraCalibrator/build/applications/
```

---

For a detailed tutorial see [GoPro Calibration](tutorials/gopro_calibration.md).
