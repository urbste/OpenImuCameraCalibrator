import os
from argparse import ArgumentParser
from subprocess import Popen
from os.path import join as pjoin
import glob
import time
from telemetry_converter import TelemetryConverter
from py_gpmf_parser.gopro_telemetry_extractor import GoProTelemetryExtractor



def main():

    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    # Cast the input to string, int or float type
    parser.add_argument('--path_static_calib_dataset',
                        default='/media/Data/work_projects/ImageStabelization/GoPro10Calibration/ImuIntrinsics/dataset1',
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_build',
                        help="Path to OpenCameraCalibrator build folder.",
                        default='/media/Data/builds/openicc_release/applications')
    parser.add_argument("--gravity_const", help="gravity constant",
                        default=9.811104, type=float)
    parser.add_argument("--initial_static_duration_s",
                        help="duration of the initial static phase for bias estimation",
                        default=15, type=float)
    parser.add_argument("--verbose", help="If calibration steps should output more information.",
                        default=0, type=int)
    args = parser.parse_args()

    # #
    # # 0. Check inputs
    # #
    bin_path = pjoin(args.path_to_build)
    cam_calib_path = args.path_static_calib_dataset
    cam_calib_video = glob.glob(pjoin(cam_calib_path,"*.MP4"))[0]
    if len(cam_calib_video) == 0:
        print("Error! Could not find cam calibration video file with MP4 ending in path "+cam_calib_path)
        exit(-1)
    print(cam_calib_video)

    # json telemetry globals
    gopro_py_gmpf_telemetry = glob.glob(pjoin(cam_calib_video))[0][:-4]+"_pygpmf.json"
    gopro_conv_telemetry = glob.glob(pjoin(cam_calib_video))[0][:-4]+".json"

    #
    # 1. Extracting GoPro telemetry
    #
    print("==================================================================")
    print("Extracting GoPro telemetry for imu bias and camera imu calibration.")
    print("==================================================================")
    start = time.time()
    extractor = GoProTelemetryExtractor(os.path.join(cam_calib_video))
    extractor.open_source()
    extractor.extract_data_to_json(os.path.join(cam_calib_path, gopro_py_gmpf_telemetry),
            ["ACCL", "GYRO", "GPS5", "GPSP", "GPSU", "GPSF", "GRAV", "MAGN", "CORI", "IORI"])
    extractor.close_source()

    print("==================================================================")
    print("Telemetry extraction took {:.2f}s.".format(time.time()-start))
    print("==================================================================")


    #
    # 2. Convert gopro json telemetry to common format
    #
    telemetry_conv = TelemetryConverter()
    telemetry_conv.convert_pygpmf_telemetry(gopro_py_gmpf_telemetry, gopro_conv_telemetry)

    #
    # 3. Perform static multi pose calibration
    #
    print("==================================================================")
    print("Performing static multi pose IMU calibration.")
    print("==================================================================")
    start = time.time()
    spline_init = Popen([pjoin(bin_path,"static_imu_calibration"),
                       "--telemetry_json="+gopro_conv_telemetry,
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