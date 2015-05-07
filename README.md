# srl_global_planner
The SRL_GLOBAL_PLANNER ROS package provides an implementation of the sampling based motion planners (RRT, RGG, RRT*, Theta*-RRT) as global planner plugin for move_base, a ROS framework. Please refer to http://wiki.ros.org/move_base , for a detailed description of the move_base framework.

## Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler

## Installation

Clone the package into you catkin workspace
- cd [workspace]/src
- git clone https://github.com/srl-freiburg/srl_global_planner.git
- cd ..
- catkin_make (or catkin build)
- 
