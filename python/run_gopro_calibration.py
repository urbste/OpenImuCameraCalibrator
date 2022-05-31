import os
import json
from argparse import ArgumentParser
from subprocess import Popen
from os.path import join as pjoin
import glob
import time
from utils import get_abbr_from_cam_model
from telemetry_converter import TelemetryConverter, TelemetryImporter

def main():

    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_calib_dataset', 
                        default='/media/Data/Sparsenet/CameraCalibrationStudy/GoPro9/1080_50/dataset2', 
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_build', 
                        help="Path to OpenCameraCalibrator build folder.",
                        default='/media/Data/builds/openicc_release/applications') 
    parser.add_argument('--path_to_imu_intrinsics', 
                        help="If available you can also supply imu intrinsics. Can be generated with static_multipose_imu_calibration.py",
                        default='/media/Data/Sparsenet/CameraCalibrationStudy/StaticMultiPose/GoPro9/dataset2/static_calib_result.json')  
    parser.add_argument("--image_downsample_factor", 
                        help="The amount to downsample the image size.", 
                        default=2, type=float)
    parser.add_argument("--camera_model", 
                        help="Camera model to use.", 
                        choices=['PINHOLE', 'PINHOLE_RADIAL_TANGENTIAL', 'DIVISION_UNDISTORTION', 'DOUBLE_SPHERE', 'EXTENDED_UNIFIED', 'FISHEYE'],
                        default="DIVISION_UNDISTORTION", type=str)
    parser.add_argument("--checker_size_m",
                        help="Length checkerboard square in m.",
                        default=0.021, type=float)
    parser.add_argument("--num_squares_x",
                        help="number of squares in x direction.",
                        default=10, type=int) # 10 for charuco board, 14 radon board
    parser.add_argument("--num_squares_y",
                        help="number of squares in x direction.",
                        default=8, type=int) # 8 for charuco board, 9 radon board
    parser.add_argument("--voxel_grid_size",
                        help="Voxel grid size for camera calibration. Will only take images that if there does not exist another pose in the voxel.",
                        default=0.03, type=float)
    parser.add_argument("--calib_cam_line_delay",
                        help="If camera line delay should be calibrated (EXPERIMENTAL)", 
                        default=0)
    parser.add_argument("--board_type", 
                        help="Board type (radon or charuco)", 
                        default="charuco", type=str)
    parser.add_argument("--gravity_const", 
                        help="gravity constant", 
                        default=9.811104, type=float)
    parser.add_argument("--recompute_corners", 
                        help="If the corners should be extracted again when running a dataset multiple times.", 
                        default=1, type=int)
    parser.add_argument("--bias_calib_remove_s", 
                        help="How many seconds to remove from start and end (due to press of button)", 
                        default=1.0, type=float)
    parser.add_argument("--reestimate_bias_spline_opt", 
                        help="If biases should be also estimated during spline optimization", 
                        default=0, type=int)
    parser.add_argument("--optimize_board_points", 
                        help="if board points should be optimized during camera calibration and after pose estimation.", 
                        default=1, type=int)
    parser.add_argument("--known_gravity_axis", 
                        help="If the gravity direction in the calibration board is exactly known.", 
                        choices=["X","Y","Z","UNKOWN"], default="UNKOWN", type=str)
    parser.add_argument("--global_shutter", 
                        help="If the camera is a global shutter cam.", default=0, type=int)
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
    imu_bias_path = pjoin(args.path_calib_dataset,'imu_bias')
    imu_bias_video = glob.glob(pjoin(imu_bias_path,"*.MP4"))
    if len(imu_bias_video) == 0:
        print("Error! Could not find imu bias calibration video file with MP4 ending in path "+imu_bias_path)
        exit(-1)
    cam_imu_path = pjoin(args.path_calib_dataset,'cam_imu')
    cam_imu_video = glob.glob(pjoin(cam_imu_path,"*.MP4"))
    
    if len(cam_imu_video) == 0:
        print("Error! Could not find imu camera calibration video file with MP4 ending in path "+cam_imu_path)
        exit(-1)


    # globals
    cam_imu_video_fn = os.path.basename(cam_imu_video[0])[:-4] # strip .MP4
    bias_video_fn = os.path.basename(imu_bias_video[0])[:-4]
    cam_video_fn = os.path.basename(cam_calib_video[0])[:-4]

    pose_calib_dataset = pjoin(cam_imu_path, "pose_calib_"+cam_imu_video_fn+".calibdata")
    cam_calib = "cam_calib_"+cam_video_fn+"_" + \
                         get_abbr_from_cam_model(args.camera_model) + "_" + \
                         str(args.image_downsample_factor)
    calib_dataset_name = cam_calib+".calibdata"
    cam_calib_file_path = pjoin(cam_calib_path, cam_calib)
    calib_dataset_json = cam_calib_file_path+".json"

    aruco_detector_params = pjoin(path_to_src, 'resource', 'charuco_detector_params.yml')
    checker_size_m = str(args.checker_size_m)
    imu_cam_calibration_json = pjoin(cam_imu_path, "imu_to_cam_calibration_"+cam_imu_video_fn+".json")

    imu_bias_json =  pjoin(imu_bias_path, "imu_bias_"+bias_video_fn+".json")
    spline_weighting_json = pjoin(cam_imu_path, "spline_info_"+cam_imu_video_fn+".json")
    cam_imu_result_json = pjoin(cam_imu_path, "cam_imu_calib_result_"+cam_imu_video_fn+".json")
    cam_imu_corners_json = pjoin(cam_imu_path, "cam_imu_corners_"+cam_imu_video_fn+".uson")
    cam_corners_json = pjoin(cam_calib_path, "cam_corners_"+cam_video_fn+".uson")

    gopro_telemetry = glob.glob(pjoin(cam_imu_path,"G*.MP4"))[0][:-4]+".json"
    imu_bias_telemetry_json_in = glob.glob(pjoin(imu_bias_path,"G*.MP4"))[0][:-4]+".json"
    gopro_telemetry_gen = gopro_telemetry[:-5] + "_gen.json"
    imu_bias_telemetry_json_in_gen = imu_bias_telemetry_json_in[:-5] + "_gen.json"

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
    # print("Extracing corners for imu camera calibration.")
    # cam_imu_calib_corners = Popen([pjoin(bin_path,'extract_board_to_json'),
    #                 "--input_path=" + cam_imu_video[0],
    #                 "--aruco_detector_params=" + aruco_detector_params,
    #                 "--board_type=" + args.board_type,
    #                 "--save_corners_json_path=" + cam_imu_corners_json,
    #                 "--downsample_factor=" + str(args.image_downsample_factor),
    #                 "--checker_square_length_m=" + checker_size_m,
    #                 "--verbose=" + str(args.verbose),
    #                 "--recompute_corners=" + str(args.recompute_corners),
    #                 "--num_squares_x="+str(args.num_squares_x),
    #                 "--num_squares_y="+str(args.num_squares_y),
    #                 "--logtostderr=1"])
    # error_cam_calib = cam_imu_calib_corners.wait()
    # print("Finished corner extraction.")
    # print("==================================================================")
    # print("Corner extraction took {:.2f}s.".format(time.time()-start))
    # print("==================================================================")

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
                    "--logtostderr=1"])
    error_cam_calib = cam_calib.wait()
    print("Finished camera calibration.")
    print("==================================================================")
    print("Camera calibration took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 2. Extracting GoPro telemetry
    #   
    js_extract_file = pjoin(path_to_src,"javascript","extract_metadata.js")
    print("==================================================================")
    print("Extracting GoPro telemetry for imu bias and camera imu calibration.")
    print("==================================================================")
    start = time.time()
    telemetry_extract = Popen(["node",js_extract_file,
                       imu_bias_path,
                       bias_video_fn+".MP4",
                       imu_bias_path])
    # error_telemetry_extract = telemetry_extract.wait()
    telemetry_extract = Popen(["node",js_extract_file,
                       cam_imu_path,
                       cam_imu_video_fn+".MP4",
                       cam_imu_path])
    error_telemetry_extract = telemetry_extract.wait()
    print("==================================================================")
    print("Telemetry extraction took {:.2f}s.".format(time.time()-start))
    print("==================================================================")
    

    #
    # 3. Convert gopro json telemetry to common format
    #
    telemetry_conv = TelemetryConverter()
    telemetry_conv.convert_gopro_telemetry_file(gopro_telemetry, gopro_telemetry_gen)
    telemetry_conv.convert_gopro_telemetry_file(imu_bias_telemetry_json_in, imu_bias_telemetry_json_in_gen)

    #
    # 4. Estimating IMU biases
    #  
    py_imu_file = pjoin(path_to_src,"python","get_imu_biases.py")
    print("==================================================================")
    print("Estimating IMU biases.")
    print("==================================================================")
    start = time.time()
    bias_estimation = Popen(["python", py_imu_file,
                       "--input_json_path=" + imu_bias_telemetry_json_in_gen,
                       "--output_path=" + imu_bias_json,
                       "--gravity_const=" + str(args.gravity_const),
                       "--remove_sec=" + str(args.bias_calib_remove_s)])
    error_bias_estimation = bias_estimation.wait()
    print("==================================================================")
    print("IMU bias estimation took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 5. Creating pose dataset for IMU - CAM calibration
    #   
    print("==================================================================")
    print("Estimating camera poses for IMU - CAM calibration.")
    print("==================================================================")
    start = time.time()
    pose_estimation = Popen([pjoin(bin_path,"estimate_camera_poses_from_checkerboard"),
                       "--input_corners=" + cam_imu_corners_json,
                       "--camera_calibration_json=" + calib_dataset_json,
                       "--output_pose_dataset=" + pose_calib_dataset,
                       "--optimize_board_points="+str(args.optimize_board_points),
                       "--logtostderr=1"])
    error_pose_estimation = pose_estimation.wait()  
    print("==================================================================")
    print("Pose estimation estimation took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    # 
    # 6. Estimate spline error weighting parameters
    #  
    py_spline_file = pjoin(path_to_src,"python","get_sew_for_dataset.py")
    print("==================================================================")
    print("Estimating Spline error weighting and knot spacing.")
    print("==================================================================")
    start = time.time()
    spline_init = Popen(["python", py_spline_file,
                       "--input_json_path=" + gopro_telemetry_gen,
                       "--output_path=" + spline_weighting_json,
                       "--q_so3=" + str(0.99),
                       "--q_r3=" + str(0.99)])
    error_spline_init = spline_init.wait()  
    print("==================================================================")
    print("Spline weighting and knot spacing estimation took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 7. Estimate IMU to cam rotation
    #   
    print("==================================================================")
    print("Initializing IMU to camera rotation.")
    print("==================================================================")
    start = time.time()
    # in the case of GoPro's we can actually take the first IMU timestamp as an initial guess
    # for the time offset IMU->CAM
    importer = TelemetryImporter()
    importer.read_generic_json(gopro_telemetry_gen)
    t_imu_2_cam = -importer.telemetry["timestamps_ns"][0]*1e-9
    spline_init = Popen([pjoin(bin_path,"estimate_imu_to_camera_rotation"),
                       "--telemetry_json=" + gopro_telemetry_gen,
                       "--input_pose_calibration_dataset=" + pose_calib_dataset,
                       "--imu_bias_estimate=" + imu_bias_json,
                       "--imu_rotation_init_output=" + imu_cam_calibration_json,
                       "--delta_t_imu_to_cam=" + str(t_imu_2_cam),
                       "--logtostderr=1"])
    error_spline_init = spline_init.wait()  
    print("==================================================================")
    print("Spline weighting and knot spacing estimation took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 8. Run IMU to Camera calibration using Spline Fusion
    #  
    print("==================================================================")
    print("Optimizing IMU to Camera calibration using Spline Fusion.")
    print("==================================================================")
    start = time.time()
    spline_init = Popen([pjoin(bin_path,"continuous_time_imu_to_camera_calibration"),
                       "--gyro_to_cam_initial_calibration=" + imu_cam_calibration_json,
                       "--telemetry_json=" + gopro_telemetry_gen,
                       "--imu_intrinsics="+args.path_to_imu_intrinsics,
                       "--input_pose_dataset=" + pose_calib_dataset,
                       "--input_corners=" + cam_imu_corners_json,
                       "--camera_calibration_json=" + calib_dataset_json,
                       "--imu_bias_file=" + imu_bias_json,
                       "--output_path=" + cam_imu_path,
                       "--spline_error_weighting_json=" + spline_weighting_json,
                       "--result_output_json=" + cam_imu_result_json,
                       "--reestimate_biases="+str(args.reestimate_bias_spline_opt),
                       "--logtostderr=1",
                       "--global_shutter="+str(args.global_shutter),
                       "--gravity_const="+str(args.gravity_const),
                       "--known_grav_dir_axis="+args.known_gravity_axis,
                       "--calibrate_cam_line_delay="+str(args.calib_cam_line_delay),
                       "--debug_video_path="+cam_imu_video[0]])
    error_spline_init = spline_init.wait()  
    print("==================================================================")
    print("Spline weighting and knot spacing estimation took {:.2f}s.".format(time.time()-start))
    print("==================================================================")

    #
    # 9. Print results
    #   
    py_spline_file = pjoin(path_to_src,"python","print_result_stats.py")
    print("==================================================================")
    print("Print results.")
    print("==================================================================")
    start = time.time()
    print_res = Popen(["python", py_spline_file,
                       "--path_results=" + cam_imu_result_json])
    error_print_res = print_res.wait()  

if __name__ == "__main__":
    main()