cmake_minimum_required(VERSION 3.0.2)
project(solar_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_generation 
  dji_osdk_ros
)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-fPIC ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

include_directories(
        ../../include/dji_osdk_ros_obsoleted
  ../../include
#  ${DJIOSDK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HSV.msg
  THERMO.msg
  KEY_PRESSED.msg
)

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
#   std_msgs
# )

generate_messages(
   DEPENDENCIES
   std_msgs
 )
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################








###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
   INCLUDE_DIRS include
  LIBRARIES solar_project
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs nmea_msgs geometry_msgs  dji_osdk_ros
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${DJIOSDK_INCLUDE_DIRS}
  /home/mx/dji_ws/src/Onboard-SDK-ROS-master/include/dji_osdk_ros_obsoleted
  /home/mx/dji_ws/src/Onboard-SDK-ROS-master/include
)

###########
## Build ##
###########
#add_executable(dji_sdk_node_pkg
#  src/main.cpp
#  src/modules/dji_sdk_node_control.cpp
#  src/modules/dji_sdk_node_services.cpp
#  src/modules/dji_sdk_node.cpp
#  src/modules/dji_sdk_node_mission_services.cpp
#  src/modules/dji_sdk_node_subscriber.cpp
#  src/modules/dji_sdk_node_publisher.cpp
#  src/modules/dji_sdk_node_mobile_comm.cpp
#  src/modules/dji_sdk_node_payload_comm.cpp
#  src/modules/dji_sdk_node_time_sync.cpp
#        ../dji_osdk_ros/modules/osdkhal_linux.c
#        ../dji_osdk_ros/modules/osdkosal_linux.c
#  src/dji_linux_environment.cpp
#  src/dji_linux_helpers.cpp)

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )







#target_link_libraries(
#  ${catkin_LIBRARIES}
    #djiosdk-core.a
    #dji-linker.a
    #advanced-sensing.a
    #${LIBUSB_1_LIBRARIES}
    #${FFMPEG_LIBRARIES}
#        waypointv2-interface.a
#        waypointv2-core.a
#        DJIProtobuf.a
#        djisdk-common.a
#  ${DJIOSDK_LIBRARIES}
#)


## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
 # ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/solar_project.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/solar_project_node.cpp)
add_executable(solar_flight_control
               src/solar_project/solar_flight_control.cpp)

add_library(pid src/solar_project/pid.cpp)
add_library(KF src/solar_project/KF.cpp)

target_link_libraries(solar_flight_control
       pid 
       KF
       
       ${catkin_LIBRARIES}
    djiosdk-core.a
    dji-linker.a
    advanced-sensing.a
    ${LIBUSB_1_LIBRARIES}
    ${FFMPEG_LIBRARIES}
    opencv_core 
    opencv_highgui 
    opencv_imgproc
    ${OpenCV_LIBRARIES}
    )
        
add_dependencies(solar_flight_control dji_osdk_ros_generate_messages_cpp)




add_executable(start_camera_stream
               src/solar_project/start_camera_stream.cpp)

target_link_libraries(start_camera_stream
       ${catkin_LIBRARIES}
    djiosdk-core.a
    dji-linker.a
    advanced-sensing.a
    ${LIBUSB_1_LIBRARIES}
    ${FFMPEG_LIBRARIES}
    opencv_core 
    opencv_highgui 
    opencv_imgproc
    ${OpenCV_LIBRARIES}
    )
        
add_dependencies(start_camera_stream dji_osdk_ros_generate_messages_cpp)



add_executable(take_overlapped_photos
               src/solar_project/take_overlapped_photos.cpp)

target_link_libraries(take_overlapped_photos
       ${catkin_LIBRARIES}
    djiosdk-core.a
    dji-linker.a
    advanced-sensing.a
    ${LIBUSB_1_LIBRARIES}
    ${FFMPEG_LIBRARIES}
    opencv_core 
    opencv_highgui 
    opencv_imgproc
    ${OpenCV_LIBRARIES}
    )
        
add_dependencies(take_overlapped_photos dji_osdk_ros_generate_messages_cpp)



## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

#install(TARGETS dji_sdk_node_pkg
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_solar_project.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
