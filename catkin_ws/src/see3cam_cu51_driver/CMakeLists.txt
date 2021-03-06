cmake_minimum_required(VERSION 2.8.3)
project(see3cam_cu51_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(libuvc REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  $ENV{OPENCV_ECON_PATH}/include
  /usr/local/include
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

set(CMAKE_PREFIX_PATH "$ENV{OPENCV_ECON_PATH}")
link_directories($ENV{OPENCV_ECON_PATH}/lib)

catkin_package()

add_library(uvccamera src/uvccamera.cpp)

add_executable(see3cam_cu51_node src/see3cam_cu51_driver.cpp)
#add_executable(see3cam_cu51_v4l2_node src/see3cam_cu51_v4l2_driver.cpp)

target_link_libraries(see3cam_cu51_node uvccamera
                      ${libuvc_LIBRARIES}
                      ${catkin_LIBRARIES})

#target_link_libraries(see3cam_cu51_v4l2_node
#                      ${libuvc_LIBRARIES}
#                      ${catkin_LIBRARIES})


install(TARGETS see3cam_cu51_node uvccamera #see3cam_cu51_v4l2_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
