
import os
import json
from argparse import ArgumentParser
import numpy as np
from subprocess import Popen, PIPE
from os.path import join as pjoin
import glob
import time
from telemetry_converter import TelemetryConverter
from py_gpmf_parser.gopro_telemetry_extractor import GoProTelemetryExtractor

def main():

    parser = ArgumentParser("OpenCameraCalibrator - Extract GoPro telemetry from a folder of files using pygpmf")
    # Cast the input to string, int or float type 
    parser.add_argument('--path', 
                        default='/home/steffen/Data/GPStrava/GoproCalib/GoPro9Neu/1080_60_wide/cam', 
                        help="Path to calibration dataset")

    args = parser.parse_args()
    
    all_files = glob.glob(pjoin(args.path, '*.MP4'))

    for f in all_files:
        filename = os.path.splitext(os.path.basename(f))[0]
        os.path.dirname(f)
        print("Extracting telemetry from file: " + f)

        extractor = GoProTelemetryExtractor(f)
        extractor.open_source()
        extractor.extract_data_to_json(os.path.join(os.path.dirname(f), filename + "_pygpmf.json"), 
            ["ACCL", "GYRO", "GPS5", "GPSP", "GPSU", "GPSF", "GRAV", "MAGN", "CORI", "IORI"])
        extractor.close_source()

        converter = TelemetryConverter()
        converter.convert_pygpmf_telemetry(os.path.join(os.path.dirname(f), filename + "_pygpmf.json"),
                                           os.path.join(os.path.dirname(f), filename + ".json"))
                                           
        print("Finished extracting telemetry from file: " + f)

if __name__ == "__main__":
    main()