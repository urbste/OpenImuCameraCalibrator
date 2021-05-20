import os
import json
from argparse import ArgumentParser
import numpy as np
from subprocess import Popen, PIPE
from os.path import join as pjoin
import glob
import time
from telemetry_converter import TelemetryConverter
from natsort import natsorted

def main():

    parser = ArgumentParser("OpenCameraCalibrator - GoPro Calibrator")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_calib_dataset', 
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/AllanVariance', 
                        help="Path to calibration dataset")
    parser.add_argument('--path_to_src', 
                        help="Path to OpenCameraCalibrator src folder.",
                        default='/home/steffen/Projects/OpenCameraCalibrator')   
    args = parser.parse_args()


    final_merged_json = pjoin(args.path_calib_dataset, "merged_telemetry.json")

    # extract all json files
    js_extract_file = pjoin(args.path_to_src,"javascript","extract_metadata.js")

    all_files = glob.glob(pjoin(args.path_calib_dataset, '*.MP4'))

    for f in all_files:
        filename = f[f.rfind("/")+1:]
        telemetry_extract = Popen(["node",js_extract_file,
                        args.path_calib_dataset,
                        filename,
                        args.path_calib_dataset])
        telemetry_extract.wait()
    # merge files to a single large file
    all_json_files = natsorted(glob.glob(pjoin(args.path_calib_dataset, '*.json')), key=lambda y: y.lower())

    # estimate variance
    telemetry_conv = TelemetryConverter()
    telemetry_conv.convert_gopro_telemetry_file(all_json_files, final_merged_json)

if __name__ == "__main__":
    main()