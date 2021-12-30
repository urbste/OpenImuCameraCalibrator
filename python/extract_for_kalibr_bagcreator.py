import cv2
import os
import glob
from subprocess import Popen, PIPE
from os.path import join as pjoin
from argparse import ArgumentParser
from telemetry_converter import TelemetryConverter
import math
from utils import time_to_s_nsec

def extract_frames(gopro_video, output_image_path, downsample_fac=2.0, skip_frames=3):
    if not os.path.exists(output_image_path):
        os.makedirs(output_image_path)
    cap = cv2.VideoCapture(gopro_video)
    count = 0
    empty_frame = 0
    while (cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        timestamp_ns = cap.get(cv2.CAP_PROP_POS_MSEC) * 1e-3
        timestamp_s, timestamp_ns = time_to_s_nsec(timestamp_ns)
        num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if count % skip_frames == 0:
            if ret:
                if count % 100 == 0:
                    print('Wrote frame {}/{}: '.format(count,num_frames))
                frame = cv2.resize(frame, (0,0), fx=1/downsample_fac, fy=1/downsample_fac)
                cv2.imwrite(os.path.join(output_image_path, "{0}{1:09d}.png".format(timestamp_s, timestamp_ns)), frame) 
            else:
                if empty_frame > 200:
                    break
                empty_frame += 1
                print("empty")
        count += 1

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = ArgumentParser()
    parser.add_argument('--input_path', 
                        help="path to metadata json",
                        default='/media/Data/Sparsenet/CameraCalibrationStudy/Kalibr/Gopro9/1080_50/imu_calib')
    parser.add_argument('--output_path', 
                        help="output path",
                        default='/media/Data/Sparsenet/CameraCalibrationStudy/Kalibr/Gopro9/1080_50/imu_calib/bag_input')
    parser.add_argument('--path_to_src', 
                        help="Path to OpenCameraCalibrator src folder.",
                        default='/home/Data/Projects/OpenCameraCalibrator')  
    parser.add_argument('--downsample_fac', default=2, type=float)
    parser.add_argument('--skip_frames', default=0, type=int)   
    args = parser.parse_args()

    if not os.path.exists(args.output_path):
        os.makedirs(args.output_path)

    calib_video = glob.glob(pjoin(args.input_path,"*.MP4"))[0]
    extract_frames(calib_video, os.path.join(args.output_path,'cam0'), skip_frames=args.skip_frames)

    cam_video_fn = os.path.basename(calib_video)
    # extract imu data
    js_extract_file = pjoin(args.path_to_src,"javascript","extract_metadata.js")
    telemetry_extract = Popen(["node",js_extract_file,
                       args.input_path,
                       cam_video_fn,
                       args.input_path])
    telemetry_extract.wait()

    gopro_telemetry = glob.glob(pjoin(args.input_path,"G*.MP4"))[0][:-4]+".json"
    gopro_telemetry_gen = pjoin(args.output_path,"imu0.csv")

    telemetry_conv = TelemetryConverter()
    telemetry_conv.convert_gopro_telemetry_file_to_kalibr(gopro_telemetry, gopro_telemetry_gen)


if __name__=="__main__":
    main()