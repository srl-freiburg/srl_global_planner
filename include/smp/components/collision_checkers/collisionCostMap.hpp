#ifndef _SMP_COLLISION_CHECKER_CIRCLE_HPP_
#define _SMP_COLLISION_CHECKER_CIRCLE_HPP_

#include <smp/components/collision_checkers/collisionCostMap.h>

#include <smp/components/collision_checkers/base.hpp>
#include <smp/common/region.hpp>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>


template< class typeparams, int NUM_DIMENSIONS >
smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::collision_checker_costmap () {

  num_discretization_steps = 20;
  discretization_length = 0.1;
  discretization_method = 0;
  //introducing the size of the robot
  size_robot=0.30;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::~collision_checker_costmap () {

  for (typename list<region_t*>::iterator iter = list_obstacles.begin();
       iter != list_obstacles.end(); iter++) {

    region_t *region_curr = *iter;

    delete region_curr;
  }

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::initialize(base_local_planner::CostmapModel* world_model,std::vector<geometry_msgs::Point> footprint_spec, double inscribed_radius, double circumscribed_radius, costmap_2d::Costmap2DROS* costmap_ros, std::string planner_frame, tf::TransformListener *listener){

  this->world_model_ = world_model;
  this->footprint_spec_ = footprint_spec;
  ROS_DEBUG("CollisionChecker Size of the footprint_spec %d", (int )footprint_spec_.size());

  this->circumscribed_radius_= circumscribed_radius;
  this->inscribed_radius_ = inscribed_radius;
  this->costmap_ros_ = costmap_ros; ///< @brief The ROS wrapper for the costmap the controller will use

  CollCheckerlistener = listener;

  global_frame_ = costmap_ros_->getGlobalFrameID();

  rob_foot_print_ = costmap_ros_->getRobotFootprint();

  try{

      ROS_DEBUG("Getting Transform from %s to %s", planner_frame.c_str(), global_frame_.c_str() );

      CollCheckerlistener->waitForTransform( global_frame_, planner_frame, ros::Time(0), ros::Duration(0.20));
      CollCheckerlistener->lookupTransform(  global_frame_, planner_frame, ros::Time(0), transform_);
    }
    catch(tf::TransformException){

        ROS_ERROR("Failed to receive transform from CostMap");
        return 0;
  }


  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::clean_obs (int a) {


  list_obstacles.clear();
  a=1;
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::cc_update_insert_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::cc_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::cc_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::cc_update_delete_edge (edge_t *edge_in) {

  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::check_collision_state (state_t *state_in) {


    double x_i, y_i, theta_i;

    x_i = (*state_in)[0];
    y_i = (*state_in)[1];
    theta_i = (*state_in)[2];


    tf::Pose source;

    tf::Quaternion q= tf::createQuaternionFromRPY(0,0,theta_i);

    tf::Matrix3x3 base(q);

    source.setOrigin(tf::Vector3(x_i,y_i,0));

    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result=transform_*source;

    double x_bot = result.getOrigin().x();
    double y_bot = result.getOrigin().y();
    double orient_bot = tf::getYaw( result.getRotation());

    // ROS_DEBUG("From (%f,%f,%f) to (%f,%f,%f) ", x_i, y_i, theta_i, x_bot, y_bot, orient_bot);

    std::vector<geometry_msgs::Point> footprint_spec;

    footprint_spec.clear();

    // Given a pose, build the oriented footprint of the robot.

  //  std::vector<geometry_msgs::Point> rob_foot_print = costmap_ros_->getRobotFootprint();

    // build the oriented footprint at the robot's current location
    double cos_th = cos(orient_bot);
    double sin_th = sin(orient_bot);
    for (unsigned int i = 0; i < rob_foot_print_.size(); ++i){
        geometry_msgs::Point new_pt;
        new_pt.x = x_bot + (rob_foot_print_[i].x * cos_th - rob_foot_print_[i].y * sin_th);
        new_pt.y = y_bot + (rob_foot_print_[i].x * sin_th + rob_foot_print_[i].y * cos_th);
        footprint_spec.push_back(new_pt);

    }
    // costmap_ros_->getOrientedFootprint(x_bot , y_bot , orient_bot, footprint_spec);

    // ROS_DEBUG("CollisionCheckerState Size of the footprint_spec %d", (int )footprint_spec.size());
    geometry_msgs::Point robot_point;

    robot_point.x = x_bot;
    robot_point.y = y_bot;
    robot_point.z = 0;


    double cost = world_model_->footprintCost(robot_point, footprint_spec, inscribed_radius_, circumscribed_radius_);



    if( cost < 0 || cost >= LEVEL_OBSTACLE_ ){

                // ROS_ERROR("Collisions");
                return 0;
      }else{

          // ROS_DEBUG("Cost of collisions %f, point cost %f", cost, world_model_->pointCost(x_i, y_i));
      }



  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::check_collision_trajectory (trajectory_t *trajectory_in) {


  if (trajectory_in->list_states.size() == 0)
    return 1;


  typename list<state_t*>::iterator iter = trajectory_in->list_states.begin();


  state_t *state_prev = *iter;


  if (this->check_collision_state (state_prev) == 0)
    return 0;

  iter++;


  for (; iter != trajectory_in->list_states.end(); iter++) {

    state_t *state_curr = *iter;

    if (discretization_method != 0) {
      // Compute the increments
      double dist_total = 0.0;
      double increments[NUM_DIMENSIONS];
      for (int i = 0; i < NUM_DIMENSIONS; i++) {
        double increment_curr = (*state_curr)[i] - (*state_prev)[i];
        dist_total += increment_curr * increment_curr;
        increments[i] = increment_curr;
    }
      dist_total = sqrt(dist_total);


      // Compute the number of increments
      int num_increments;
      if (discretization_method == 1) {
            num_increments = num_discretization_steps;
      }
      else if (discretization_method == 2){
          num_increments = (int) floor(dist_total/discretization_length);
      }


      if (num_increments > 0) { // Execute the remaining only if the discretization is required.

        for (int i = 0; i < NUM_DIMENSIONS; i++)  // Normalize the increments.
          increments[i] = increments[i]/((double)(num_increments+1));

          for (int idx_state = 1; idx_state <= num_increments; idx_state++){

                bool collision = true;
                state_t *state_to_check = new state_t;
                (*state_to_check)[0] = (*state_prev)[0] + increments[0]*idx_state;
                (*state_to_check)[1] = (*state_prev)[1] + increments[1]*idx_state;
                (*state_to_check)[2] = (*state_prev)[2] + increments[2]*idx_state;

                if(check_collision_state (state_to_check))
                    collision = false;

                if (collision == true) {
                      return 0;
                }
          }

      }
    }

    if (check_collision_state (state_curr) == 0){
      return 0;
    }

    state_prev = state_curr;
  }

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::set_discretization_steps (int num_discretization_steps_in) {

  if (num_discretization_steps <= 0) {
    num_discretization_steps = 0;
    discretization_length = 0;
    discretization_method = 0;
  }
  else {
    num_discretization_steps = num_discretization_steps_in;
    discretization_method = 1;
  }

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::set_discretization_length (double discretization_length_in) {

  if (discretization_length <= 0.0) {
    num_discretization_steps = 0;
    discretization_length = 0.05;
    discretization_method = 0;
  }
  else {
    discretization_length = discretization_length_in;
    discretization_method = 2;
  }

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_costmap<typeparams,NUM_DIMENSIONS>
::add_obstacle (region_t &obstacle_in) {

  list_obstacles.push_back (new region_t(obstacle_in));
  return 1;
}


#endif
