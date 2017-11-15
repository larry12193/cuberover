cmake_minimum_required(VERSION 2.8.3)
project(tmcm_1633_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
    FILES
    canMsg.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs
               sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(canopen src/canopen.cpp)
add_library(tmcm_1633 src/tmcm_1633.cpp)

add_executable(${PROJECT_NAME}_node src/tmcm_1633_driver.cpp)

target_link_libraries(${PROJECT_NAME}_node
                      canopen
                      tmcm_1633
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
