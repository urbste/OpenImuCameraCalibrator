import os
import json
from argparse import ArgumentParser
from subprocess import Popen
from os.path import join as pjoin
import glob
import time
from utils import get_abbr_from_cam_model
from telemetry_converter import TelemetryConverter



def main():

    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_static_calib_dataset', 
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/StaticMultiPose/GoPro9/dataset2', 
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_build', 
                        help="Path to OpenCameraCalibrator build folder.",
                        default='') 
    parser.add_argument('--path_to_src', 
                        help="Path to OpenCameraCalibrator src folder.",
                        default='/home/steffen/Projects/OpenCameraCalibrator')   
    parser.add_argument("--gravity_const", help="gravity constant", default=9.811104, type=float)
    parser.add_argument("--initial_static_duration_s", 
    help="duration of the initial static phase for bias estimation", default=30, type=float)

    parser.add_argument("--verbose", help="If calibration steps should output more information.", default=0, type=int)

    args = parser.parse_args()


    # # 
    # # 0. Check inputs 
    # #
    bin_path = pjoin(args.path_to_build)
    cam_calib_path = args.path_static_calib_dataset
    cam_calib_video = glob.glob(pjoin(cam_calib_path,"*.MP4"))
    if len(cam_calib_video) == 0:
        print("Error! Could not find cam calibration video file with MP4 ending in path "+cam_calib_path)
        exit(-1)
    print(cam_calib_video)

    # globals
    cam_video_fn = os.path.basename(cam_calib_video[0])[:-4]
    gopro_telemetry = glob.glob(pjoin(cam_calib_path,"G*.MP4"))[0][:-4]+".json"
    gopro_telemetry_gen = glob.glob(pjoin(cam_calib_path,"G*.MP4"))[0][:-4]+"_gen.json"

    #
    # 1. Extracting GoPro telemetry
    #   
    js_extract_file = pjoin(args.path_to_src,"javascript","extract_metadata.js")
    print("==================================================================")
    print("Extracting GoPro telemetry.")
    print("==================================================================")
    start = time.time()
    telemetry_extract = Popen(["node",js_extract_file,
                       cam_calib_path,
                       cam_video_fn+".MP4",
                       cam_calib_path])
    error = telemetry_extract.wait()
    print("==================================================================")
    print("Telemetry extraction took {:.2f}s.".format(time.time()-start))
    print("==================================================================")
    
    #
    # 2. Convert gopro json telemetry to common format
    #
    telemetry_conv = TelemetryConverter()
    telemetry_conv.convert_gopro_telemetry_file(gopro_telemetry, gopro_telemetry_gen)

    #
    # 3. Perform static multi pose calibration
    #   
    print("==================================================================")
    print("Performing static multi pose IMU calibration.")
    print("==================================================================")
    start = time.time()
    spline_init = Popen([pjoin(bin_path,"static_imu_calibration"),
                       "--gravity_magnitude="+str(args.gravity_const),
                       "--initial_static_interval_s="+str(args.initial_static_duration_s),
                       "--verbose="+str(args.verbose), 
                       "--output_calibration_path="+pjoin(args.path_static_calib_dataset,"static_calib_result.json"),
                       "--logtostderr=1"])
    error_spline_init = spline_init.wait()  
    print("==================================================================")
    print("Static multi pose IMU calibration took {:.2f}s.".format(time.time()-start))
    print("==================================================================")


if __name__ == "__main__":
    main()