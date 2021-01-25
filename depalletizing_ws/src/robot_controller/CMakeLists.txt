cmake_minimum_required(VERSION 3.0.2)
project(robot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  tf2
  tf2_ros
  hardware_interface
  controller_manager
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(robot_node
        src/RobotNode.cpp
        src/RobotDriver.cpp
        src/RobotFakeDriver.cpp
        src/ROBOTHW.cpp
        src/RobotController.cpp)

#add robot control lib here
target_link_libraries(robot_node
       ${catkin_LIBRARIES}
      )

install(TARGETS robot_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
