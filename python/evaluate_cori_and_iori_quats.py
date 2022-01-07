# Steffen Urban, November 2021

# script to evaluate and have a look at the accuracy 
# of the CORI and IORI parameters from the GoPro telemetry

import os
import cv2
import glob
import json
import natsort
import numpy as np
from numpy.core.records import get_remaining_size
from utils import load_camera_imu_calibration, load_camera_calibration
from scipy.spatial.transform import Rotation as R
from telemetry_converter import TelemetryImporter

def estimate_camera_poses(image_names, cam_matrix):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()
    square_length = 0.021
    board = cv2.aruco.CharucoBoard_create(10, 8, square_length, square_length/2.0, arucoDict)

    pose_results = {}
    for i_name in image_names:
        I = cv2.imread(i_name,0)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(I, arucoDict,
            parameters=arucoParams)

        # refine
        allCorners = []
        allIds = []
        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(I, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,I,board)

            if res2[1] is not None and res2[2] is not None and len(res2[1])>5:
                allCorners.append(res2[1])
                allIds.append(res2[2])
            if len(res2[1]) < 10:
                continue
            objPts = board.chessboardCorners[res2[2],:]
            imgPts = res2[1]
            pose = cv2.solvePnP(objPts, imgPts, cam_matrix, None, flags=cv2.SOLVEPNP_SQPNP)
            
            if pose[0]:
                rvec, tvec = cv2.solvePnPRefineLM(objPts[:,:], imgPts, cam_matrix, None, pose[1], pose[2])
                image_name = os.path.basename(i_name)
                timestamp = float(image_name[:image_name.rfind(".png")])
                R_c_w = cv2.Rodrigues(rvec)[0]
                p_w_c = -R_c_w.T @ tvec
                q_w_c = R.from_matrix(R_c_w.T)
                inv_cam_matrix = np.linalg.inv(cam_matrix)
                
                img_pts3 = np.ones((imgPts.shape[0],3))
                img_pts3[:,:2] = imgPts.squeeze()
                img_pts_normalized = inv_cam_matrix @ img_pts3.T
                img_pts_normalized /= img_pts_normalized[2,:]
                pose_results[timestamp] = {"R_c_w" : rvec.tolist(), 
                    "q_w_c": q_w_c.as_quat().tolist(), "t_c_w": tvec.tolist(), 
                    "p_w_c": p_w_c.tolist(), "img_pts": img_pts_normalized[:2,:].tolist()}
                
                # I_rgb = cv2.cvtColor(I, cv2.COLOR_GRAY2BGR)
                # I_rgb = cv2.aruco.drawAxis(I_rgb, cam_matrix, None, rvec, tvec, 2*square_length)
                # I_rgb = cv2.aruco.drawDetectedCornersCharuco(I_rgb, res2[1], res2[2], (255,255,0))
                # cv2.imshow("I_rgb",I_rgb)
                # cv2.waitKey(1)

    return pose_results

def cori_to_iori(gopro_cori_quat, R_cam_imu):
    return R.from_matrix(R_cam_imu) * R.from_quat(gopro_cori_quat)

def get_R12(R1, R2):
    return R1 * R2.inv()

# load camera and camera to imu calibrations
cam_calib = "/media/Data/Sparsenet/GoProEvaluation/EvaluateCoriAndIori/gopro_9_linear_200/cam/cam_calib_GX017366_ph_1.json"
cam_imu_calib = "/media/Data/Sparsenet/GoProEvaluation/EvaluateCoriAndIori/gopro_9_linear_200/cam_imu/cam_imu_calib_result_GX017368.json"
cam_matrix, img_size = load_camera_calibration(cam_calib)
R_imu_cam, t_imu_cam, T_imu_cam = load_camera_imu_calibration(cam_imu_calib)

# load gopro telemtry 
telemetry = TelemetryImporter()
telemetry.read_gopro_telemetry("/media/Data/Sparsenet/GoProEvaluation/EvaluateCoriAndIori/GX017365.json")
cori_quats = telemetry.get_camera_quaternions_at_frametimes()


image_names = natsort.natsorted(glob.glob("/media/Data/Sparsenet/GoProEvaluation/EvaluateCoriAndIori/images/*.png"))

camera_poses_path = "/media/Data/Sparsenet/GoProEvaluation/EvaluateCoriAndIori/camera_poses.json"
if not os.path.exists(camera_poses_path):
    absolute_poses = estimate_camera_poses(image_names, cam_matrix)
    with open(camera_poses_path, 'w') as f:
        json.dump(absolute_poses, f)
else:
    with open(camera_poses_path, 'r') as f:
        absolute_poses = json.load(f)
    
    t0 = str(0.05*1e9)
    R_imu_0 = cori_to_iori(cori_quats[float(t0)], R_imu_cam.T)
    R_gt_0 = R.from_quat(absolute_poses[t0]["q_w_c" ]).inv()

    R_12_gts = []
    R_12_imus = []
    timestamps = natsort.natsorted(absolute_poses.keys())
    error = []
    # estimate relative orientation from absolute and from cori
    for ts in timestamps:
        if t0 == ts or float(ts) < float(ts) or float(ts) not in cori_quats:
            continue

        R12_gt = get_R12(R_gt_0,  
            R.from_quat(absolute_poses[ts]["q_w_c" ]).inv()).inv()

        iori_ts = cori_to_iori(cori_quats[float(ts)], R_imu_cam.T)
        R12_iori = get_R12(R_imu_0, iori_ts)

        R_12_gts.append(R12_gt.as_rotvec() * 180./np.pi)
        R_12_imus.append(R12_iori.as_rotvec() * 180./np.pi)
        error.append((R12_gt.as_rotvec() - R12_iori.as_rotvec())* 180./np.pi)

R_12_gts = np.array(R_12_gts)
R_12_imus = np.array(R_12_imus)
error = np.array(error)
import matplotlib.pyplot as plt
fig, (ax1, ax2, ax3) = plt.subplots(1, 3)

ax1.plot( R_12_gts[:,0], 'r')
ax1.plot( R_12_gts[:,1], 'g')
ax1.plot( R_12_gts[:,2], 'b')
ax1.plot( R_12_imus[:,0], 'r--')
ax1.plot( R_12_imus[:,1], 'g--')
ax1.plot( R_12_imus[:,2], 'b--')
ax3.plot( error[:,0], 'r--')
ax3.plot( error[:,1], 'g--')
ax3.plot( error[:,2], 'b--')
plt.show()
