import os
import json
import shutil
from argparse import ArgumentParser
import numpy as np
import cv2
from subprocess import Popen, PIPE
from os.path import join as pjoin
import glob
import time
import natsort
from utils import get_abbr_from_cam_model
from telemetry_converter import TelemetryConverter

def main():

    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_calib_dataset', 
                        default='/home/steffen/Downloads/session2/calib_images/frames/', 
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_build', 
                        help="Path to OpenCameraCalibrator build folder.",
                        default='/home/steffen/projects/OpenImuCameraCalibrator/build/applications')
    parser.add_argument("--image_downsample_factor", 
                        help="The amount to downsample the image size.", 
                        default=2, type=float)
    parser.add_argument("--camera_model", 
                        help="Camera model to use.", 
                        choices=['PINHOLE', 'PINHOLE_RADIAL_TANGENTIAL', 'DIVISION_UNDISTORTION', 'DOUBLE_SPHERE', 'EXTENDED_UNIFIED', 'FISHEYE'],
                        default="DOUBLE_SPHERE", type=str)
    parser.add_argument("--checker_size_m",
                        help="Length checkerboard square in m.",
                        default=0.0015, 
                        type=float)
    parser.add_argument("--marker_length_m",
                        help="Length of marker in m. If -1 we will use half of the checker size.",
                        default=0.001, 
                        type=float)
    parser.add_argument("--num_squares_x",
                        help="number of squares in x direction.",
                        default=35)
    parser.add_argument("--num_squares_y",
                        help="number of squares in y direction.",
                        default=57)
    parser.add_argument("--voxel_grid_size",
                        help="Voxel grid size for camera calibration. Will only take images that if there does not exist another pose in the voxel.",
                        default=0.0005)
    parser.add_argument("--calib_cam_line_delay",
                        help="If camera line delay should be calibrated", default=1)
    parser.add_argument("--board_type", help="Board type (radon or charuco)", default="charuco", type=str)
    parser.add_argument("--recompute_corners", help="If the corners should be extracted again when running a dataset multiple times.", default=0, type=int)
    parser.add_argument("--optimize_board_points", help="if board points should be optimized during camera calibration and after pose estimation.", default=0, type=int)
    parser.add_argument("--verbose", help="If calibration steps should output more information.", default=1, type=int)

    args = parser.parse_args()

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file,"../")

    # 
    # 0. Check inputs 
    #
    bin_path = pjoin(args.path_to_build)
    cam_calib_path = pjoin(args.path_calib_dataset)

    #
    cam_calib = "cam_calib_" + \
                         get_abbr_from_cam_model(args.camera_model) + "_" + \
                         str(args.image_downsample_factor)
    cam_calib_file_path = pjoin(cam_calib_path, cam_calib)

    aruco_detector_params = pjoin(path_to_src, 'resource', 'charuco_detector_params.yml')
    checker_size_m = str(args.checker_size_m)
    marker_length_m = str(args.marker_length_m)
    cam_corners_json = pjoin(cam_calib_path, 'corners.json')

    # Copy images to a temporary sub-folder and rename them with nanosecond
    # timestamps so the C++ extractor can parse them, while preserving originals.
    timestamped_dir = pjoin(cam_calib_path, 'timestamped_frames')
    os.makedirs(timestamped_dir, exist_ok=True)
    files_in_folders = natsort.natsorted(glob.glob(pjoin(cam_calib_path, '*.png')))
    if len(files_in_folders) == 0:
        print("Error! No PNG images found in " + cam_calib_path)
        exit(-1)
    for i, f in enumerate(files_in_folders):
        new_name = str(int(i*1e9))+'.png'
        shutil.copy2(f, pjoin(timestamped_dir, new_name))

    #
    # 0. Extract corners for camera calibration and camera imu calibration
    #
    print("==================================================================")
    print("Running corner extraction.")
    print("==================================================================")   
    start = time.time()
    print("Extracing corners for camera calibration.")
    cam_calib = Popen([pjoin(bin_path,'extract_board_to_json'),
                    "--input_path=" + timestamped_dir,
                    "--aruco_detector_params=" + aruco_detector_params,
                    "--board_type=" + args.board_type,
                    "--save_corners_json_path=" + cam_corners_json,
                    "--downsample_factor=" + str(args.image_downsample_factor),
                    "--checker_square_length_m=" + checker_size_m,
                    "--marker_length_m=" + marker_length_m,
                    "--verbose=" + str(args.verbose),
                    "--recompute_corners=" + str(args.recompute_corners),
                    "--num_squares_x="+str(args.num_squares_x),
                    "--num_squares_y="+str(args.num_squares_y),
                    "--aruco_dict=" + str(cv2.aruco.DICT_4X4_1000),
                    "--logtostderr=1"])
    error_cam_calib = cam_calib.wait()
    print("Finished corner extraction.")
    print("==================================================================")
    print("Corner extraction took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 1. Calibrate camera
    #
    print("==================================================================")
    print("Running camera calibration.")
    print("==================================================================")
    start = time.time()
    print("Calibrating camera.")
    cam_calib = Popen([pjoin(bin_path,'calibrate_camera'),
                    "--input_corners=" + cam_corners_json,
                    "--save_path_calib_dataset=" + cam_calib_file_path,
                    "--camera_model_to_calibrate=" + args.camera_model,
                    "--grid_size=" + str(args.voxel_grid_size),
                    "--optimize_board_points="+str(args.optimize_board_points),
                    "--verbose=" + str(args.verbose),
                    "--logtostderr=0"])
    error_cam_calib = cam_calib.wait()
    print("Finished camera calibration.")
    print("==================================================================")
    print("Camera calibration took {:.2f}s.".format(time.time()-start))
    print("==================================================================")
  
if __name__ == "__main__":
    main()