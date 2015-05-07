# srl_global_planner
The SRL_GLOBAL_PLANNER ROS package provides an implementation of the sampling based motion planners (RRT, RGG, RRT*, Theta**-RRT) as global planner plugin for move_base, a ROS framework. Please refer to http://wiki.ros.org/move_base , for a detailed description of the move_base framework.

## Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler

## Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/srl-freiburg/srl_global_planner.git`
- `cd ../`
- `catkin_make` or `catkin build`



## Usage
- roslaunch srl_global_planner move_base_global_srl.launch will launch the global planner node. You can launch the planner with different configurations, by varying some parameters:
  - `TYPE_PLANNER`, set to:
    - 0, use RRT
    - 1, use RRT* only partial rewiring
    - 2, use RRT*
  - `NUMBER_UPDATE_TRAJ`, set to:
    - Choose after how many cost improvements the planner could stop, currently set at 2. Minimum value is 1. Higher the value, higher the computaion time required to generate a trajectory
  - `BOX` :
    - if it is set to 1, the nearest vertex is selected from a weighted box according to the Ball-Box Theorem.
  - `RADIUS` :
    - the size of the radius where the near neighbor set is generated, in case you use RRT* select -1 so to have the RRT* shrinking ball.
  - `RHO` :
    - end condition for the POSQ steer function, should be set to a value of few cm.
  - `DT` :
    - integration time step of the POSQ steer function, maximum value 0.5s
  - `TYPE_SAMPLING` :
    - if `TYPE_SAMPLING` == 0 support set as uniform over a strips following a discrete path generate by a Theta* algorithm
    - If `TYPE_SAMPLING` == 1 support set as Gaussian Mixture over the Theta* path
    - if `TYPE_SAMPLING` == 2 support set as gaussians over a spline fitting the Theta* waypoints
    - if `TYPE_SAMPLING` == 3 support for Theta*-RRT, if set need to specify the range where to set orientations `OR_RANGE` and the width of the strip along the Theta* path `WIDTH_STRIP`
    - if `TYPE_SAMPLING` == 4 support set as the entire state space, the dimension of the state space are read from the grid generate by the move_base framework
    - if `TYPE_SAMPLING` == 5 Path Biasing along the current available trajectory. If used need to set also the biasing probability BIAS_PROB and the DISPERSION
  - `GOAL_BIASING`
    - if set to 1 activate goal biasing.
  - `GOAL_BIASING_THS`
    - set the probability to select a state not in the state space
  - `ADD_COST_FROM_COSTMAP`, set to true if you want to add cost from global cost map
  - `ADD_COST_PATHLENGTH`, set to true if you want to add the cost associated to path length and changes of heading
  - `ADD_COST_THETASTAR`, set to true if you want to add cost resembling closeness to thetastar path
  - Params related to the distance metric, only one of them shoul be set to 1. `LEARNED` and 'NOTLEARNED' select the best vertex from the spherical neighborhood with radius equal to the parameter `RADIUS`:
    - `LEARNED`, set to 1, if you want to find the nearest vertex according to the learned cost
    - `FINDNEAREST`, set to 1 if you want to find the nearest vertex according to the Kd Tree Euclidean Distance
    - `NOTLEARNED`, set to 1 if you want to find the nearest vertex according to the cost computed over extensions of POSQ path
  - `TIMECOUNTER`, set to 1 if you want to specify the maximum amount of seconds your planner should work.
  - `MAXTIME`, max number of seconds allowed to find a path
  - `max_iterations`, if `TIMECOUNTER` is 0, this is the maximum number of iterations the planner will execute to find a path,
