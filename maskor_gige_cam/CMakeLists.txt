cmake_minimum_required(VERSION 2.8.3)
project(maskor_gige_cam)


set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  dynamic_reconfigure
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


#Generate dynamic reconfigure header files
generate_dynamic_reconfigure_options(cfg/Flir.cfg)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES maskor_gige_cam
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

find_package( OpenCV REQUIRED )

set(EBUS_SDK_PATH /opt/pleora/ebus_sdk/Ubuntu-x86_64)
set(EBUS_INCLUDE_DIRS ${EBUS_SDK_PATH}/include)

set(EBUS_LIBS
		${EBUS_SDK_PATH}/lib/libEbTransportLayerLib.so
		${EBUS_SDK_PATH}/lib/libEbUtilsLib.so
		${EBUS_SDK_PATH}/lib/libPtConvertersLib.so
		${EBUS_SDK_PATH}/lib/libPtUtilsLib.so
		${EBUS_SDK_PATH}/lib/libPvAppUtils.so
		${EBUS_SDK_PATH}/lib/libPvBase.so
		${EBUS_SDK_PATH}/lib/libPvBuffer.so
		${EBUS_SDK_PATH}/lib/libPvDevice.so
		${EBUS_SDK_PATH}/lib/libPvGenICam.so
		${EBUS_SDK_PATH}/lib/libPvGUI.so
		${EBUS_SDK_PATH}/lib/libPvPersistence.so
		${EBUS_SDK_PATH}/lib/libPvSerial.so
		${EBUS_SDK_PATH}/lib/libPvStream.so
		${EBUS_SDK_PATH}/lib/libPvTransmitter.so
		${EBUS_SDK_PATH}/lib/libPvVirtualDevice.so
		${EBUS_SDK_PATH}/lib/libSimpleImagingLib.so
)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EBUS_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(maskor_gige_cam      src/maskor_gige_cam.cpp
                                 include/maskor_gige_cam/maskor_gige_cam.h
)

## Declare a cpp executable
add_executable(maskor_flir_cam_node      src/maskor_flir_cam_node.cpp
					 include/maskor_gige_cam//maskor_flir_cam_node.h
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(maskor_gige_cam_node maskor_gige_cam_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(maskor_gige_cam
  ${catkin_LIBRARIES}
  ${EBUS_LIBS}
  ${OpenCV_LIBS}
)

target_link_libraries(maskor_flir_cam_node
  ${catkin_LIBRARIES}
  maskor_gige_cam
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS maskor_gige_cam maskor_gige_cam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_maskor_gige_cam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
