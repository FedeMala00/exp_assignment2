# EXPERIMENTAL ROBOTICS LABORATORY ASSIGNMENT 2: Mobile Robot Waypoint Navigation

... a project made by Federico Malatesta and Samuele Viola

## Overview

This ROS package enables a mobile robot equipped with a camera and a laser scanner to autonomously navigate to a waypoint corresponding to the lowest marker ID. The robot uses camera to identify ids of all aruco markers that can be found in the four waypoints and determine which of them has the lowest ID, then moves to it.
The assignment is implemented in a simulation environment using Gazebo and rviz.

## Features

- **Autonomous Navigation**: the robot follow a path that is created basing on a map generated by a SLAM algorithm. In our case it is done by the ROS2 package "navigation2"
- **SLAM**: robot localize and maps using SLAM algorithm, which utilize the laser scanner included in the robot
- **Autonomous Planning**: robot plans its action in order to find the best action sequence and replan in case of errors
- **Aruco Marker Detection**: robot can detect aruco markers by using the camera provided

## Necessary package
List of all the package that are needed to run this project: 
- `navigation2`
- `ros2_planning_system`
- `slam_toolbox`
- `ros2_aruco`

## Instructions on how to download and build 
The first step is to clone this repository as well as [this one](https://github.com/FedeMala00/robot_urdf_test) that are:  
One containing the launch file for all the implemented actions and everything regarding `plansys2` and  
The other one for loading the map and launching robot, `gazebo` and `rviz`

Then it is necessary to build the workspace using the command `colcon build` into the ros2 workspace folder.

## How to run the package
In order to run it is necessary to open five different terminals and use the commands written in the `fast_launch.sh` script or directly make it executable and then using the command `./fast_launch.sh` 
This will launch all the scripts in five terminals separated.


## Package Structure

```bash
exp_assignment2/
│
├── launch/
│   ├── plansys2_simple_example_launch.py          
│   ├── rviz_launch.py          
│   └── navigation_launch.py       
│
├── params/
│   ├── nav2_params.yaml     
│   └── slam_config.yaml          
│
├── pddl/
│   ├── assignment_domain.pddl  # ROS processing IDs
│
├── src/
│   ├── move_action_node.cpp         
│   ├── patrol_action_node.cpp    
   
│   ├── patrolling_controller_node.cpp     
│   
│   └── sretrieve_pos.cpp           
│
└── CMakeLists.txt                      
└── package.xml                     
└── README.md                     

```

## Waypoint Details

The four waypoints are as follows:

- **WP 0**: x = 6.0, y = 2.0
- **WP 1**: x = 7.0, y = -5.0
- **WP 2**: x = -3.0, y = -8.0
- **WP 3**: x = -7.0, y = 1.5

The robot will find the waypoint with the lowest marker ID and move towards it.

## How It Works

1. **Marker Detection**: The robot uses the camera and the aruco node to detect markers id in when patrolling. The markers are labeled with IDs.
2. **Autonomous Planning**: The robot follows a sequence of states, in which it, for each state, generate a plan using the domain provided to reach the goal. When an action is called, the related node will operate.
3. **Navigation**: The robot create a path using SLAM_toolbox, and follows it autonomously using the nav2_bringup node.
4. **Visualization**: The robot's position, map and the path can be visualized in RViz, providing real-time feedback. Be careful to select "map", in the left where you can select the fixed frame.

### Action Definition
- **MOVE**: Move from waypoint A to waypoint B
- **PATROL**: Rotate around z-axis while checking for aruco markers IDs


### ROS Nodes

- **move_action_node**: This node is aimed to execute the action MOVE when requested
- **patrol_action_node**: This node is aimed to execute the action PATROL when requested
- **patrolling_control_node**: This node defines the problem and generate and execute the plan
- **patrol_action_node**: This node can be used for showing in real time the robot coordinates

## Video
[<video src="https://github.com/FedeMala00/exp_assignment2/raw/master/Experimental_video.mp4" controls width="600">
</video>](https://github.com/user-attachments/assets/5a2c6082-dd5c-42c5-91da-054444503d1d)
