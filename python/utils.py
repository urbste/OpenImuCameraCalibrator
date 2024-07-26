import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import cv2

ms_to_sec = 1./1000.
s_to_nsec = 1e9

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
    elif model_name == "ORTHOGRAPHIC":
        return "or"
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


def load_camera_imu_calibration(user_calib):

    with open(user_calib, 'r') as f:
        imu_camera = json.load(f)
    R_imu_cam = R.from_quat([
        imu_camera["q_i_c"]["x"],
        imu_camera["q_i_c"]["y"],
        imu_camera["q_i_c"]["z"], 
        imu_camera["q_i_c"]["w"]])
    t_imu_cam = np.array([
        imu_camera["t_i_c"]["x"],
        imu_camera["t_i_c"]["y"],
        imu_camera["t_i_c"]["z"]]).T
    T_imu_cam = np.eye(4,dtype=np.float32)
    T_imu_cam[:3,:3] = R_imu_cam.as_matrix()
    T_imu_cam[:3,3] = t_imu_cam

    return R_imu_cam.as_matrix(), t_imu_cam, T_imu_cam

def load_camera_calibration(user_calib):
    
    with open(user_calib, 'r') as f:
        camera_intrinsics = json.load(f)

    cam_matrix = np.identity(3, dtype=np.float64)
    cam_matrix[0,0] = camera_intrinsics["intrinsics"]["focal_length_x"]
    cam_matrix[1,1] = camera_intrinsics["intrinsics"]["focal_length_y"]
    cam_matrix[0,2] = camera_intrinsics["intrinsics"]["principal_pt_x"]
    cam_matrix[1,2] = camera_intrinsics["intrinsics"]["principal_pt_y"]

    
    image_width = int(camera_intrinsics["image_width"])
    image_height = int(camera_intrinsics["image_height"])


    return cam_matrix, (image_width, image_height)

# bagcreater expects image filenames with 
# filename = "{0}{1:09d}.png".format(timestamp.secs, timestamp.nsecs)
# https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_bagextractor#L66
def time_to_s_nsec(timestamp_s):
    dec, sec = math.modf(timestamp_s) 
    nanosecs = dec * s_to_nsec
    return int(sec), int(nanosecs)




def extract_images_from_video_to_folder(video_file, img_savepath, resize_factor):

    cap = cv2.VideoCapture(video_file)

    if not os.path.exists(img_savepath):
        os.makedirs(img_savepath)

    invalid_images = 0
    had_zero = False
    print("Extracting frames")
    while True:
        ret, I = cap.read()
        ts_ns = int(1e6*cap.get(cv2.CAP_PROP_POS_MSEC))
        if not ret:
            invalid_images += 1
            if invalid_images > 100:
                break
            continue
        if had_zero and ts_ns == 0:
            continue

        if not had_zero and ts_ns == 0:
            had_zero = True
        savepath = os.path.join(img_savepath, str(ts_ns)+".png")
        I = cv2.resize(I, (0,0), fx=1./resize_factor, fy=1./resize_factor)
        cv2.imwrite(savepath, I)
        print("Extracted and saved to {}".format(savepath))

