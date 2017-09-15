cmake_minimum_required(VERSION 2.8.3)
project(wpi_jaco_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs geometry_msgs sensor_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AngularCommand.msg
  AngularTorqueCommand.msg
  CartesianCommand.msg
  CartesianForceCommand.msg
  JacoFingerVel.msg
  FingerPosition.msg
  JointPosiCartForce.msg
  AngularTorqueCommand.msg
  CartesianForceCommand.msg
  jointStateConvertedForce.msg

)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EStop.srv
  EulerToQuaternion.srv
  GetAngularPosition.srv
  GetAngularEffort.srv
  GetCartesianPosition.srv
  JacoFK.srv
  QuaternionToEuler.srv
  StartForceControl.srv
  StopForceControl.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  HomeArm.action
  SetFingersPosition.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs
)
