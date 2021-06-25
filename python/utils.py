import os
import json
import numpy as np

ms_to_sec = 1./1000.

def get_abbr_from_cam_model(model_name):
    if model_name == "DIVISION_UNDISTORTION":
        return "di"
    elif model_name == "DOUBLE_SPHERE":
        return "ds"
    elif model_name == "PINHOLE":
        return "ph"
    elif model_name == "EXTENDED_UNIFIED":
        return "ex"
    elif model_name == "FISHEYE":
        return "fi"
    elif model_name == "PINHOLE_RADIAL_TANGENTIAL":
        return "pr"
    else:
        print("Model name ", model_name, " does not exist.")
        exit(-1)


def read_imu_data(path_to_json, skip_seconds=0):
    accl = []
    gyro  = []
    timestamps = []
    with open(path_to_json, 'r') as json_file:
        json_data = json.load(json_file)
        for a in json_data['1']['streams']['ACCL']['samples']:
            timestamps.append(a['cts']*ms_to_sec)
            accl.append([a['value'][1], a['value'][2], a['value'][0]])
        for g in json_data['1']['streams']['GYRO']['samples']:
            gyro.append([g['value'][1], g['value'][2], g['value'][0]])


    camera_fps = json_data['frames/second']

    if skip_seconds != 0.0:
        ms = timestamps[1] - timestamps[0]
        nr_remove = round(skip_seconds / ms)

        accl = accl[nr_remove:len(timestamps) - nr_remove]
        gyro = gyro[nr_remove:len(timestamps) - nr_remove]
        timestamps = timestamps[nr_remove:len(timestamps) - nr_remove]


    accl_np = np.asarray(accl)
    gyro_np = np.asarray(gyro)
    timestamps_np = np.asarray(timestamps)
    accl_np = accl_np[0:len(timestamps_np)]
    gyro_np = gyro_np[0:len(timestamps_np)]


    return timestamps_np, accl_np, gyro_np, camera_fps
