add_executable(calibrate_camera calibrate_camera.cc)
target_link_libraries(calibrate_camera OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(create_charuco_board create_charuco_board.cc)
target_link_libraries(create_charuco_board OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(estimate_camera_poses_from_checkerboard estimate_camera_poses_from_checkerboard.cc)
target_link_libraries(estimate_camera_poses_from_checkerboard OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(estimate_imu_to_camera_rotation estimate_imu_to_camera_rotation.cc)
target_link_libraries(estimate_imu_to_camera_rotation OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(extract_board_to_json extract_board_to_json.cc)
target_link_libraries(extract_board_to_json OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(continuous_time_imu_to_camera_calibration continuous_time_imu_to_camera_calibration.cc)
target_link_libraries(continuous_time_imu_to_camera_calibration OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(fit_allan_variance fit_allan_variance.cc)
target_link_libraries(fit_allan_variance OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(static_imu_calibration static_imu_calibration.cc)
target_link_libraries(static_imu_calibration OpenImuCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})
