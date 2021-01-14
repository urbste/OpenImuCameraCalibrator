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
                        default='', 
                        help="Path to calibration dataset")
    args = parser.parse_args()


    data = read_calib_json(args.path_results)
    data = dict(data["trajectory"]) 
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

    labels = ['spline x', 'imu y', 
            'spline y', 'imu y', 
            'spline z', 'imu z'] 

    fig, ax = plt.subplots(1,2)
    ax[0].set_title("Accelerometer - Spline value vs Measurements")
    ax[0].plot(accl_spline_np[0:-1:skip,0], 'r')
    ax[0].plot(accl_imu_np[0:-1:skip,0], 'r--')
    ax[0].plot(accl_spline_np[0:-1:skip,1], 'g')
    ax[0].plot(accl_imu_np[0:-1:skip,1], 'g--')
    ax[0].plot(accl_spline_np[0:-1:skip,2], 'b')
    ax[0].plot(accl_imu_np[0:-1:skip,2], 'b--')
    ax[0].set_xlabel("measurement")
    ax[0].set_ylabel("m/s2")

    ax[1].set_title("Gyroscope - Spline value vs Measurements")
    ax[1].plot(gyro_spline_np[0:-1:skip,0], 'r', label="spline rot vel x")
    ax[1].plot(gyro_imu_np[0:-1:skip,0], 'r--', label="gyro rot vel x")
    ax[1].plot(gyro_spline_np[0:-1:skip,1], 'g', label="spline rot vel y")
    ax[1].plot(gyro_imu_np[0:-1:skip,1], 'g--', label="gyro rot vel y")
    ax[1].plot(gyro_spline_np[0:-1:skip,2], 'b', label="spline rot vel z")
    ax[1].plot(gyro_imu_np[0:-1:skip,2], 'b--', label="gyro rot vel z")
    ax[1].set_xlabel("measurement")
    ax[1].set_ylabel("rad/s")
    fig.legend(ax, labels=labels, loc="upper right", borderaxespad=0.1) 
    plt.show()

if __name__ == "__main__":
    main()