import os
import json
from argparse import ArgumentParser
from subprocess import Popen
from os.path import join as pjoin
import glob
import time
from utils import get_abbr_from_cam_model

def main():

    parser = ArgumentParser("OpenCameraCalibrator - Orthographic Camera Calibrator")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_calib_dataset', 
                        default='', 
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_build', 
                        help="Path to OpenCameraCalibrator build folder.",
                        default='/media/Data/builds/openicc_release/applications') 
    parser.add_argument("--image_downsample_factor", 
                        help="The amount to downsample the image size.", 
                        default=2, type=float)
    parser.add_argument("--checker_size_m",
                        help="Length checkerboard square in m.",
                        default=0.0047, type=float)
    parser.add_argument("--num_squares_x",
                        help="number of squares in x direction.",
                        default=34, type=int) # 10 for charuco board, 14 radon board
    parser.add_argument("--num_squares_y",
                        help="number of squares in x direction.",
                        default=15, type=int) # 8 for charuco board, 9 radon board
    parser.add_argument("--voxel_grid_size",
                        help="Voxel grid size for camera calibration. Will only take images that if there does not exist another pose in the voxel.",
                        default=0.03, type=float)
    parser.add_argument("--board_type", 
                        help="Board type (radon or charuco)", 
                        default="charuco", type=str)
    parser.add_argument("--recompute_corners", 
                        help="If the corners should be extracted again when running a dataset multiple times.", 
                        default=1, type=int)
    parser.add_argument("--verbose", 
                        help="If calibration steps should output more information.", default=0, type=int)

    args = parser.parse_args()

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file,"../")

    # # 
    # # 0. Check inputs 
    # #
    bin_path = pjoin(args.path_to_build)
    cam_calib_path = pjoin(args.path_calib_dataset,'cam')
    cam_calib_video = glob.glob(pjoin(cam_calib_path,"*.MP4"))
    if len(cam_calib_video) == 0:
        print("Error! Could not find cam calibration video file with MP4 ending in path "+cam_calib_path)
        exit(-1)
    print(cam_calib_video)

    # globals
    cam_video_fn = os.path.basename(cam_calib_video[0])[:-4]

    cam_calib = "cam_calib_"+cam_video_fn+"_" + \
                         get_abbr_from_cam_model("ORTHOGRAPHIC") + "_" + \
                         str(args.image_downsample_factor)
    calib_dataset_name = cam_calib+".calibdata"
    cam_calib_file_path = pjoin(cam_calib_path, cam_calib)
    calib_dataset_json = cam_calib_file_path+".json"

    aruco_detector_params = pjoin(path_to_src, 'resource', 'charuco_detector_params.yml')
    checker_size_m = str(args.checker_size_m)

    cam_corners_json = pjoin(cam_calib_path, "cam_corners_"+cam_video_fn+".uson")

    #
    # 0. Extract corners for camera calibration and camera imu calibration
    #
    print("==================================================================")
    print("Running corner extraction.")
    print("==================================================================")   
    start = time.time()
    print("Extracing corners for camera calibration.")
    cam_calib = Popen([pjoin(bin_path,'extract_board_to_json'),
                    "--input_path=" + cam_calib_video[0],
                    "--aruco_detector_params=" + aruco_detector_params,
                    "--board_type=" + args.board_type,
                    "--save_corners_json_path=" + cam_corners_json,
                    "--downsample_factor=" + str(args.image_downsample_factor),
                    "--checker_square_length_m=" + checker_size_m,
                    "--verbose=" + str(args.verbose),
                    "--recompute_corners=" + str(args.recompute_corners),
                    "--num_squares_x="+str(args.num_squares_x),
                    "--num_squares_y="+str(args.num_squares_y),
                    "--logtostderr=1"])
    error_cam_calib = cam_calib.wait()

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
                    "--camera_model_to_calibrate=ORTHOGRAPHIC",
                    "--grid_size=" + str(args.voxel_grid_size),
                    "--optimize_board_points=0",
                    "--verbose=" + str(args.verbose),
                    "--logtostderr=1"])
    error_cam_calib = cam_calib.wait()
    print("Finished camera calibration.")
    print("==================================================================")
    print("Camera calibration took {:.2f}s.".format(time.time()-start))
    print("==================================================================")


if __name__ == "__main__":
    main()