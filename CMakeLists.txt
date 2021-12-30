cmake_minimum_required(VERSION 2.8.3)
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif (COMMAND cmake_policy)

project(OpenImuCameraCalibrator C CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# c++17 is important, otherwise template deduction in so3 spline will fail
set(CMAKE_CXX_STANDARD 17)
# getting some weird errors with alignment in theia view->features_
# add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)
set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization (-march=native)")
if(BUILD_WITH_MARCH_NATIVE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    message(STATUS "Architecture-aware optimization (-march=native): ENABLED")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    message(STATUS "Architecture-aware optimization (-march=native): DISABLED")
endif()

# OpenCV
message("-- Check for OpenCV")
find_package(OpenCV 4.5.0)
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


file(GLOB_RECURSE CAMCALIB_SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/*.cc)
file(GLOB_RECURSE CAMCALIB_HEADER_FILES ${CMAKE_SOURCE_DIR}/include/*.h)

add_subdirectory(third_party/apriltag)
# commit d5ccee8 before fmt was added
include_directories(third_party/Sophus)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include
                    ${THEIA_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_library(OpenImuCameraCalibrator STATIC ${CAMCALIB_SOURCE_FILES})
target_link_libraries(OpenImuCameraCalibrator apriltag)
add_subdirectory(applications)
