#!/bin/bash

#killall -9 rosmaster &
#roscore &
#gnome-terminal -x bash -c "./test.sh" &
roslaunch $(find move_group)/launch/graspit_part.launch &
roslaunch $(find move_group)/launch/move_part.launch &
exit 0
