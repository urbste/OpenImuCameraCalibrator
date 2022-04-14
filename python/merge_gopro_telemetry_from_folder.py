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

    parser = ArgumentParser("OpenICC - Merges GoPro telemetry from multiple video files for allan variance estimation")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_calib_dataset', 
                        default='', 
                        help="Path to calibration dataset") 
    args = parser.parse_args()

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file,"../")

    final_merged_json = pjoin(args.path_calib_dataset, "merged_telemetry.json")

    # extract all json files
    js_extract_file = pjoin(path_to_src,"javascript","extract_metadata.js")

    # find all video files
    all_files = glob.glob(pjoin(args.path_calib_dataset, '*.MP4'))

    # extract telemetry from each video
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