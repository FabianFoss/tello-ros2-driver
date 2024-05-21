#!/bin/bash

cd ../workspace
rm -rf build install log

rosdep install -i --from-path src
colcon build --symlink-install --packages-select takeoff_and_land_interface tello tello_control tello_msg nav_through_waypoints
