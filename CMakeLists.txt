cmake_minimum_required(VERSION 2.8.3)
project(aruco_ekf_slam)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
		roscpp
  visualization_msgs
  cv_bridge
  geometry_msgs
 sensor_msgs
 image_transport
)
catkin_package()

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3  REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
src/aruco_ekf_slam.cc
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)

add_executable(slam_node
test/slam_node.cc)
target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)