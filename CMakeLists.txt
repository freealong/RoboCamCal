cmake_minimum_required(VERSION 3.5)
project(robocamcal)

# check if I am a top-level project
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    # using c++11
    set(CMAKE_CXX_STANDARD 11)
    # Define DEBUG in Debug mode
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
    endif()
    # enable openmp
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
    find_package(OpenCV 3.0.0 REQUIRED)
    include_directories(OpenCV_INCLUDE_DIRS)
endif()

find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        sensor_msgs
        std_msgs
        cv_bridge
        roslib
        )
include_directories(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   SnapShotMsg.msg
# )

## Generate services in the 'srv' folder
add_service_files(
        FILES
        CameraData.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES xyz_vision
        #  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs cv_bridge image_transport
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
        #  DEPENDS system_lib
)

find_package(VISP REQUIRED)
include_directories(VISP_INCLUDE_DIRS)

set(src_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)
file(GLOB_RECURSE robocamcal_HEADERS ${src_ROOT}/*.hpp ${src_ROOT}/*.h)
file(GLOB_RECURSE robocamcal_SOURCES ${src_ROOT}/*.cpp ${src_ROOT}/*.cc)

add_library(robocamcal ${robocamcal_HEADERS} ${robocamcal_SOURCES})
target_link_libraries(robocamcal ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

include_directories(${src_ROOT})

add_executable(hand_eye hand_eye.cpp)
target_link_libraries(hand_eye gflags robocamcal ${VISP_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(create_marker create_marker.cpp)
target_link_libraries(create_marker ${OpenCV_LIBRARIES})

add_executable(detect_markers detect_markers.cpp)
target_link_libraries(detect_markers ${OpenCV_LIBRARIES})

add_executable(temp_test temp_test.cpp)
target_link_libraries(temp_test robocamcal ${VISP_LIBRARIES})

add_executable(camera_calibration camera_calibration.cpp)
target_link_libraries(camera_calibration robocamcal ${OpenCV_LIBRARIES})

add_executable(stereo_calibration stereo_calibration.cpp)
target_link_libraries(stereo_calibration robocamcal ${OpenCV_LIBRARIES})

add_executable(camera_bridge_test test/camera_bridge_test.cc)
target_link_libraries(camera_bridge_test robocamcal)
