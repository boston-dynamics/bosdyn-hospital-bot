cmake_minimum_required(VERSION 2.8.3)
project(drspot)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs rospy roslaunch)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
    DEPENDS
)
catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/ir_face_tracker
  nodes/respiratory_rate
  nodes/skin_temperature
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslaunch_add_file_check(launch)
install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
