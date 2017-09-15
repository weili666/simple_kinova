#TODO 遵从ros的代码规范重构代码 参考http://wiki.ros.org/CppStyleGuide
cmake_minimum_required(VERSION 2.8.3)
project(turtle)
SET(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#specify OpenCV version
set(OpenCV_DIR /home/turtle/opencv-3.1.0-build)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_sensor_msgs
  move_base_agv_msgs
  cv_bridge
  tf
  tf2
  pcl_conversions
  pcl_ros
  rosconsole
  #cmake_modules
  #Eigen3
)
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)

message("OpenCV version "${OpenCV_VERSION})

#find_package(Eigen 3 REQUIRED)
find_package(Eigen) #wangwen modified on 2016-3-14

set(CMAKE_BUILD_TYPE "Release")

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
        set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

message("package turtle Build Type:"${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS})

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
 add_message_files(
   FILES
   Sync_sensor.msg
   Can_frame.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
## add_action_files(
##   FILES
##   MoveBaseAGV.action
## )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
#  LIBRARIES mikuSensor
  CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs nav_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
 include/linuxcan/include
 include/xcommunication
 include/xsens
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
 ${OpenCV_INCLUDE_DIRS}
  /usr/local/xsens/include
  /usr/include
)


link_directories(
  /usr/local/xsens/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare a C++ library
# add_library(mikuSensor
#   src/${PROJECT_NAME}/mikuSensor.cpp
# )

#set cml files
SET(CML
    CML/Amp.cpp
    CML/AmpFile.cpp
    #CML/AmpFW.cpp
    CML/AmpParam.cpp
    CML/AmpPDO.cpp
    CML/AmpPVT.cpp
    CML/AmpStruct.cpp
    CML/AmpUnits.cpp
    CML/AmpVersion.cpp
    CML/Can.cpp
    CML/can_copley.cpp
    CML/can_kvaser.cpp
    #CML/can_ixxat.cpp
    #CML/can_ixxat_v3.cpp
    CML/CanOpen.cpp
    CML/CML.cpp
    CML/CopleyIO.cpp
    CML/CopleyIOFile.cpp
    CML/CopleyNode.cpp
    CML/ecatdc.cpp
    CML/Error.cpp
    CML/EtherCAT.cpp


    CML/EventMap.cpp
    CML/File.cpp
    CML/Filter.cpp
    #CML/Firmware.cpp
    CML/Geometry.cpp
    CML/InputShaper.cpp
    CML/IOmodule.cpp

    CML/Linkage.cpp
    CML/LSS.cpp
    CML/Network.cpp
    CML/Node.cpp

    CML/Path.cpp
    CML/PDO.cpp
    CML/Reference.cpp
    CML/SDO.cpp
    CML/Threads.cpp
    CML/Threads_posix.cpp
    CML/TrjScurve.cpp
    CML/Utils.cpp
)

SET(XSENS_PUBLIC
    Xsens/xcommunication/enumerateusbdevices.cpp
    Xsens/xcommunication/int_xsdatapacket.cpp
    Xsens/xcommunication/iointerface.cpp
    Xsens/xcommunication/iointerfacefile.cpp
    Xsens/xcommunication/legacydatapacket.cpp
    Xsens/xcommunication/mtwsdidata.cpp
    Xsens/xcommunication/legacydatapacket.cpp
    Xsens/xcommunication/packetfixeddata.cpp
    Xsens/xcommunication/packetstamper.cpp
    Xsens/xcommunication/protocolhandler.cpp
    Xsens/xcommunication/serialinterface.cpp
    Xsens/xcommunication/synclinegmt.c
    Xsens/xcommunication/synclinemk4.c
    Xsens/xcommunication/usbinterface.cpp
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(mikuSensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

add_executable(motor_control_node src/motor_control_node/motor_control_node.cpp src/motor_control_node/MotorController.cpp ${CML})

add_executable(move src/move/move.cpp ${CML})

add_executable(move_test src/move/move_test.cpp ${CML})


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  turtle_generate_message)

add_dependencies(move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  turtle_generate_message)

## Specify libraries to link a library or executable target against

target_link_libraries(motor_control_node
    ${catkin_LIBRARIES}
    libcanlib.so
    libdl.so
)

target_link_libraries(move
    ${catkin_LIBRARIES}
    libcanlib.so
    libdl.so
)

target_link_libraries(move_test
    ${catkin_LIBRARIES}
    libcanlib.so
    libdl.so
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
# install(TARGETS mikuSensor mikuSensor_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mikuSensor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
