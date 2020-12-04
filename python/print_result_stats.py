import os
import json
from matplotlib import pyplot as plt
from argparse import ArgumentParser
import numpy as np
import natsort 

def read_calib_json(file):
    with open(file, 'r') as f:
        results = json.load(f)

    return results
    


def main():
    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    parser.add_argument('--path_results', 
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/GoPro6/1080_30/dataset1/cam_imu/cam_imu_calib_result_kont_GH016390.json', 
                        help="Path to calibration dataset")
    args = parser.parse_args()


    data = read_calib_json(args.path_results)
    data = dict(data) 
    data = natsort.natsorted(data.items())

    accl_spline = []
    accl_imu = []
    gyro_spline = []
    gyro_imu = []
    t = []
    for d in data:
        t.append(d[0])
        accl_spline.append([d[1]["accl_spline"]["x"], d[1]["accl_spline"]["y"],d[1]["accl_spline"]["z"]])
        accl_imu.append([d[1]["accl_imu"]["x"], d[1]["accl_imu"]["y"],d[1]["accl_imu"]["z"]])
        gyro_spline.append([d[1]["gyro_spline"]["x"], d[1]["gyro_spline"]["y"],d[1]["gyro_spline"]["z"]])
        gyro_imu.append([d[1]["gyro_imu"]["x"], d[1]["gyro_imu"]["y"],d[1]["gyro_imu"]["z"]])

    accl_spline_np = np.asarray(accl_spline)
    accl_imu_np = np.asarray(accl_imu)
    gyro_spline_np = np.asarray(gyro_spline)
    gyro_imu_np = np.asarray(gyro_imu)
    t_np = np.asarray(t)
    skip = 1
    plt.plot(accl_spline_np[0:-1:skip,0], 'r')
    plt.plot(accl_imu_np[0:-1:skip,0], 'r--')
    plt.plot(accl_spline_np[0:-1:skip,1], 'g')
    plt.plot(accl_imu_np[0:-1:skip,1], 'g--')
    plt.plot(accl_spline_np[0:-1:skip,2], 'b')
    plt.plot(accl_imu_np[0:-1:skip,2], 'b--')
    plt.show()

    plt.plot(gyro_spline_np[0:-1:skip,0], 'r')
    plt.plot(gyro_imu_np[0:-1:skip,0], 'r--')
    plt.plot(gyro_spline_np[0:-1:skip,1], 'g')
    plt.plot(gyro_imu_np[0:-1:skip,1], 'g--')
    plt.plot(gyro_spline_np[0:-1:skip,2], 'b')
    plt.plot(gyro_imu_np[0:-1:skip,2], 'b--')
    plt.show()

if __name__ == "__main__":
    main()