
add_executable(calibrate_camera calibrate_camera.cc)
target_link_libraries(calibrate_camera ${CameraCalibrator} ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(create_charuco_board create_charuco_board.cc)
target_link_libraries(create_charuco_board ${CameraCalibrator} ${GLOG_LIBRARIES} ${THEIA_LIBRARIES} ${OpenCV_LIBRARIES} ${GFLAGS_LIBRARIES})

