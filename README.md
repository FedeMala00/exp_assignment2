# Experimental Robotics Laboratory second assignment #

### Necessary package ###
List of all the package that are needed to run this project: 
- `navigation2`
- `ros2_planning_system`
- `slam_toolbox`

### Instructions on how to download and build ###
The first step is to clone this repository as well as [this one](https://github.com/FedeMala00/robot_urdf_test) that are:  
One containing the launch file for all the implemented actions and everything regarding `plansys2` and  
The other one for loading the map and launching robot, `gazebo` and `rviz`

Then it is necessary to build the workspace using the command `colcon build` into the ros2 workspace folder.

### How to run the package ###
In order to run it is necessary to open five different terminals and use the commands written in the `fast_launch.sh` script or directly make it executable and then using the command `./fast_launch.sh` in a new terminal.  
This will launch all the scripts in five terminals separated.
