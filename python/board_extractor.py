import os
import cv2
import json
from argparse import ArgumentParser


class BoardExtractor():
    def __init__(self, board_type, pattern_size, img_resize_fac, aruco_settings=None):

        self.board_type = board_type
        self.img_resize_fac = img_resize_fac
        self.aruco_settings = aruco_settings
        self.pattern_size = pattern_size

    def extract_corners(self, img):
        
        img = cv2.resize(img, (0,0), fx=self.img_resize_fac, fy=self.img_resize_fac)

        if self.board_type == "radon":
            retval, corners, meta = cv2.findChessboardCornersSB(img, self.pattern_size, 
                cv2.CALIB_CB_NORMALIZE_IMAGE| cv2.CALIB_CB_EXHAUSTIVE)
        elif self.board_type == "aruco":
            print("detectaruco")
        else:
            print("Not supported board type. Choices: radon, aruco")
            pass
        return pts3d, corners

def main():

    parser = ArgumentParser("OpenCameraCalibrator - Board Extractor")
    parser.add_argument('--path_to_video_file', 
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/GH016448.MP4', 
                        help="Path to video file")
    args = parser.parse_args()

    board_extractor = BoardExtractor("radon", (9,14), 0.5)

    cap = cv2.VideoCapture(args.path_to_video_file)
    while (cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        corners = board_extractor.extract_corners(frame)  

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()

