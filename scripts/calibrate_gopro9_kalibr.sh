# steps how to run kalibr
# 1. docker pull stereolabs/kalibr
# 2. Use python/extract_for_kalibr_bagcreator.py to extract telemetry and single images
# 3. Kalibr calibrate camera
# 4. Kalibr calibrate cam imu
DOWNSAMPLE_FAC=2
CAM_MODEL='eucm-none'

BASEPATH=/media/Data/Sparsenet/CameraCalibrationStudy/Kalibr/Gopro9
SETTING=1080_w_100_1600
DATASET=dataset1


# INPUT=${BASEPATH}/${SETTING}/${DATASET}/cam/
# OUTPUT=${BASEPATH}/${SETTING}/${DATASET}/cam/bag_input

# python3 ../python/extract_for_kalibr_bagcreator.py --input_path=${INPUT} --output_path=${OUTPUT} \
#     --path_to_src=../ --skip_frames=4 --downsample_fac=${DOWNSAMPLE_FAC}

INPUT=${BASEPATH}/${SETTING}/${DATASET}/cam_imu/
OUTPUT=${BASEPATH}/${SETTING}/${DATASET}/cam_imu/bag_input
python3 ../python/extract_for_kalibr_bagcreator.py --input_path=${INPUT} --output_path=${OUTPUT} \
    --path_to_src=../ --skip_frames=1 --downsample_fac=${DOWNSAMPLE_FAC}

docker run -v ${BASEPATH}/${SETTING}:/data -it stereolabs/kalibr:kinetic

#kalibr_bagcreater --folder=/data/${DATASET}/cam/bag_input --output-bag=/data/${DATASET}/cam/for_kalibr.bag
#kalibr_bagcreater --folder=/data/${DATASET}/cam_imu/bag_input --output-bag=/data/${DATASET}/cam_imu/for_kalibr.bag

#kalibr_calibrate_cameras --bag=/data/${DATASET}/cam/for_kalibr.bag --models ${CAM_MODEL} --target /data/april_6x6_50x50cm.yaml --topics /cam0/image_raw --dont-show-report
# cp camchain-data${DATASET}camfor_kalibr.yaml /data/${DATASET}/cam/
kalibr_calibrate_imu_camera --target /data/april_6x6_50x50cm.yaml --imu /data/imu_noise.yaml --imu-models 'scale-misalignment' --cam /data/${DATASET}/cam/camchain-data${DATASET}camfor_kalibr.yaml --bag /data/${DATASET}/cam_imu/for_kalibr.bag --topics /cam0/image_raw --dont-show-report
cp camchain-imucam-data${DATASET}cam_imufor_kalibr.yaml /data/${DATASET}/cam_imu
cp imu-data${DATASET}cam_imufor_kalibr.yaml /data/${DATASET}/cam_imu
cp results-imucam-data${DATASET}cam_imufor_kalibr.txt /data/${DATASET}/cam_imu