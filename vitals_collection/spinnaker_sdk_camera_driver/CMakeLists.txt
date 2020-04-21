cmake_minimum_required(VERSION 2.8.3)
project(spinnaker_sdk_camera_driver)

add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

###
# camera Acquisition specific 
###
#SET("OpenCV_DIR" "~/apps/opencv-2.4.13/")
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" CACHE PATH "Project Include Directory")
set(SPINNAKER_INCLUDE_DIR "/usr/include/spinnaker" CACHE PATH "Spinnaker Include Directory")
set(SPINNAKER_LIB_DIR "/usr/lib" CACHE PATH "Spinnaker Libs Directory")
# set(yaml-cpp_DIR "~/apps/yaml-cpp" CACHE PATH "yaml-cpp Directory")
set(CUDA_USE_STATIC_CUDA_RUNTIME  OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
  dynamic_reconfigure
  nodelet
)

#find_package(PCL REQUIRED)

###
# Acquisition specific
###
# Find Packages
find_package(OpenCV REQUIRED)
# use LibUnwind only for x86_64 or x86_32 architecture
# do not use LibUnwind for arm architecture
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  message("uses LibUnwind for x86_64 or x86_32 architecture")
  find_package(LibUnwind REQUIRED)
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)
  message("Detected ARM architecture")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)

find_package(Boost REQUIRED)
if(Boost_FOUND)
  find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()

add_message_files(
  FILES
  SpinnakerImageNames.msg
)

generate_dynamic_reconfigure_options(
  cfg/spinnaker_cam.cfg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)
## setting catkin_package, include_directories and libs based on architecture
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
    catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs message_runtime nodelet
    DEPENDS OpenCV LibUnwind
    )

    include_directories(
      ${PROJECT_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS}
      ${SPINNAKER_INCLUDE_DIR}
      ${OpenCV_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIR}
      ${LibUnwind_INCLUDE_DIRS}
    )

    link_directories( ${SPINNAKER_LIB_DIR} )

    set (LIBS ${LibUnwind_LIBRARIES} Spinnaker ${OpenCV_LIBS} ${Boost_GENERAL})
    
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)
    catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs message_runtime nodelet
    DEPENDS OpenCV
    )

    include_directories(
      ${PROJECT_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS}
      ${SPINNAKER_INCLUDE_DIR}
      ${OpenCV_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIR}
    )

    link_directories( ${SPINNAKER_LIB_DIR} )

    set (LIBS Spinnaker ${OpenCV_LIBS} ${Boost_GENERAL})
    
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)

add_library (acquilib SHARED
  src/capture.cpp
  src/camera.cpp
)

add_dependencies(acquilib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(acquilib ${LIBS} ${catkin_LIBRARIES})

add_library(capture_nodelet src/capture_nodelet.cpp src/subscriber_nodelet.cpp)
target_link_libraries(capture_nodelet acquilib ${catkin_LIBRARIES})

add_executable (acquisition_node src/acquisition_node.cpp)
add_dependencies(acquisition_node acquilib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries (acquisition_node acquilib ${LIBS} ${catkin_LIBRARIES})


add_executable (acquisition_nodelet src/acquisition_nodelet.cpp)
add_dependencies(acquisition_nodelet acquilib capture_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries (acquisition_nodelet acquilib capture_nodelet ${LIBS} ${catkin_LIBRARIES})

add_executable (nodelet_test src/nodelet_test.cpp)
add_dependencies(nodelet_test ${catkin_EXPORTED_TARGETS})
target_link_libraries (nodelet_test capture_nodelet ${LIBS} ${catkin_LIBRARIES})

install(TARGETS acquilib acquisition_node capture_nodelet acquisition_nodelet nodelet_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
