<h1 align="center"> ECE470 Final Project </h1>

## Summary ##
This is the final project for ECE470 - Introduction to Robotics. This project utilizes camera sensor and forward/inverse kinematics to control the robot. The aim of the project is to allow the robot to perform a pick and place task given certain inputs.

## Setting up Code ##
1. Make catkin workspace:
   * `mkdir -p catkin_ws/src`
   * `cd catkin_ws/src`
   * `catkin_init_workspace`
   * `cd ~/catkin_ws/src/`
2. Clone this repository under catkin_ws/src using `git clone`
3. Do `catkin_make` under catkin_ws/
4. source for every new command prompt - `source devel/setup.bash`
5. Optional (taken care in roslaunch): In one command prompt, launch roscore - `roscore`
6. In a new command prompt, `roslaunch ur3_driver ur3_gazebo.launch`
7. In catkin_ws/src/projectandDriver/projectpkg_py/scripts - `chmod +x project.py`
8. `rosrun lab2pkg_py project.py --simulator True`
