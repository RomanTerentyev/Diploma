cmake_minimum_required(VERSION 2.8.3)
project(diploma)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    rospy
    geometry_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories( ${catkin_INCLUDE_DIRS} )


catkin_package()

add_executable(
    colormask.py
	keypoints.py
)

target_link_libraries(
    colormask.py
	keypoints.py
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

