
DOWNSAMPLE_FAC=2
CAM_MODEL=FISHEYE
CHECKERSIZE=0.04
NUM_SQ_X=6
NUM_SQ_Y=6
CALIB_LINE_DELAY=0
RECOMP_CORNERS=0
GRID_SIZE=0.04

BASEPATH=/media/Data/Sparsenet/CameraCalibrationStudy/Kalibr/Gopro9
SETTING=1080_w_100_1600


DATASET=dataset1
python3 ../python/run_gopro_calibration.py --path_calib_dataset=${BASEPATH}/${SETTING}/${DATASET} \
    --image_downsample_factor=${DOWNSAMPLE_FAC} \
    --camera_model=${CAM_MODEL} \
    --checker_size_m=${CHECKERSIZE} \
    --num_squares_x=${NUM_SQ_X} --num_squares_y=${NUM_SQ_Y} \
    --calib_cam_line_delay=${CALIB_LINE_DELAY} \
    --board_type=apriltag \
    --recompute_corners=${RECOMP_CORNERS} \
    --optimize_board_points=0 \
    --reestimate_bias_spline_opt=1 \
    --voxel_grid_size=${GRID_SIZE}
