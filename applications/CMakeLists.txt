
add_executable(calibrate_camera calibrate_camera.cc)
target_link_libraries(calibrate_camera OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(create_charuco_board create_charuco_board.cc)
target_link_libraries(create_charuco_board OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(create_imu_camera_charuco create_imu_camera_charuco.cc)
target_link_libraries(create_imu_camera_charuco OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(test_spline_landmark_xyz test_spline_landmark_xyz.cc)
target_link_libraries(test_spline_landmark_xyz OpenCameraCalibrator ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})
