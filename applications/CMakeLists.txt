
add_executable(calibrate_camera calibrate_camera.cc)
target_link_libraries(calibrate_camera OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(create_charuco_board create_charuco_board.cc)
target_link_libraries(create_charuco_board OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

#add_executable(continuous_time_imu_to_camera_calibration continuous_time_imu_to_camera_calibration.cc)
#target_link_libraries(continuous_time_imu_to_camera_calibration OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(test_spline_landmark_xyz test_spline_landmark_xyz.cc)
target_link_libraries(test_spline_landmark_xyz OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(estimate_camera_poses_from_checkerboard estimate_camera_poses_from_checkerboard.cc)
target_link_libraries(estimate_camera_poses_from_checkerboard OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(estimate_imu_to_camera_rotation estimate_imu_to_camera_rotation.cc)
target_link_libraries(estimate_imu_to_camera_rotation OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(continuous_time_imu_to_camera_calibration_new continuous_time_imu_to_camera_calibration_new.cc)
target_link_libraries(continuous_time_imu_to_camera_calibration_new OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(extract_board_to_json extract_board_to_json.cc)
target_link_libraries(extract_board_to_json OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})
