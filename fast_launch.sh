#!/bin/bash

gnome-terminal -- bash -c "ros2 launch robot_urdf_test gazebo2.launch.py; exec bash"
sleep 1

gnome-terminal -- bash -c "ros2 launch slam_toolbox online_sync_launch.py; exec bash"
sleep 0.5

gnome-terminal -- bash -c "ros2 launch nav2_bringup navigation_launch.py; exec bash"
sleep 0.5

gnome-terminal -- bash -c "ros2 launch exp_assignment2 plansys2_simple_example_launch.py; exec bash"
sleep 3

gnome-terminal -- bash -c "ros2 run exp_assignment2 patrolling_controller_node; exec bash"
