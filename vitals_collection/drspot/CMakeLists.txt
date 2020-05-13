cmake_minimum_required(VERSION 2.8.3)
project(drspot)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs std_srvs sensor_msgs geometry_msgs rospy roslaunch)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
    DEPENDS
)
catkin_python_setup()

install(FILES rqt_vitals_readout_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/ir_face_tracker
  nodes/respiratory_rate
  nodes/skin_temperature
  nodes/multichrome_face_tracker_standalone
  nodes/heart_rate_spo2_standalone
  nodes/multi_tracker_with_hr_spo2
  nodes/mode_manager
  scripts/rqt_vitals_readout
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslaunch_add_file_check(launch)
install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  rqt_vitals_readout_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
