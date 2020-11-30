#!/bin/sh

export
TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboProj5/catkin_ws/src/map/largehouseworld.world
TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/RoboProj5/catkin_ws/src/map/map.yaml


xterm -e "source /home/workspace/RoboProj5/catkin_ws/devel/setup.bash &
roslaunch turtlebot_gazebo turtlebot_world.launch
world_file:=/home/workspace/RoboProj5/catkin_ws/src/map/largehouseworld.world" &

sleep 3

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch
map_file:=/home/workspace/RoboProj5/catkin_ws/src/map/map.yaml" &

sleep 3

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 15

xterm -e "roslaunch pick_objects pick_objects.launch" &

sleep 5

xterm -e "roslaunch add_markers add_markers.launch" 



