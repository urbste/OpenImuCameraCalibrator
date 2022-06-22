import cv2
import os

from utils import extract_images_from_video_to_folder

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("OpenCameraCalibrator - Image Extractor")
    # Cast the input to string, int or float type 
    parser.add_argument('--path_to_videofile', 
                        default='')
    parser.add_argument('--path_to_image_output', 
                        default='')
    parser.add_argument('--resize_factor', default=1.0, type=float)
    args = parser.parse_args()

    extract_images_from_video_to_folder(args.path_to_videofile, args.path_to_image_output, args.resize_factor)