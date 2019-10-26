cmake_minimum_required(VERSION 2.8.3)
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif (COMMAND cmake_policy)

project(OpenCameraCalibrator C CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 11)

# OpenCV
message("-- Check for OpenCV")
find_package(OpenCV 4.1.2)
if (OpenCV_FOUND)
  message("-- Found OpenCV: ${OpenCV_INCLUDE_DIRS}")
else (OpenCV_FOUND)
  message(FATAL_ERROR "Can't find OpenCV. Please set OpenCV_INCLUDE_DIR & "
    "OpeneCV_LIBRARY")
endif (OpenCV_FOUND)

# TheiaSfM
find_package(Theia REQUIRED)
if (THEIA_FOUND)
  message("-- Found TheiaSfM: ${THEIA_INCLUDE_DIRS}")
else (THEIA_FOUND)
  message("Cannot find TheiaSfM!")
endif (THEIA_FOUND)

add_subdirectory(applications)

include_directories(include
    ${THEIA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE CAMCALIB_SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/*.cc)

add_library(CameraCalibrator STATIC ${CAMCALIB_SOURCE_FILES})
