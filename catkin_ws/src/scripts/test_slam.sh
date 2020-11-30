#!/bin/sh

export
TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboProj5/catkin_ws/src/map/largehouseworld.world

xterm -e "source /home/workspace/RoboProj5/catkin_ws/devel/setup.bash &
roslaunch turtlebot_gazebo turtlebot_world.launch
world_file:=/home/workspace/RoboProj5/catkin_ws/src/map/largehouseworld.world" &

sleep 3

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 3

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 3

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
