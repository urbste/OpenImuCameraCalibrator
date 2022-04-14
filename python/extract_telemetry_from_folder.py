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

    parser = ArgumentParser("OpenCameraCalibrator - Extract GoPro telemetry from a folder of files")
    # Cast the input to string, int or float type 
    parser.add_argument('--path', 
                        default='', 
                        help="Path to calibration dataset")

    args = parser.parse_args()
    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file,"../")

    # extract all json files
    js_extract_file = pjoin(path_to_src,"javascript","extract_metadata.js")

    all_files = glob.glob(pjoin(args.path, '*.MP4'))

    for f in all_files:
        filename = f[f.rfind("/")+1:]
        telemetry_extract = Popen(["node",js_extract_file,
                        args.path,
                        filename,
                        args.path])
        telemetry_extract.wait()

if __name__ == "__main__":
    main()