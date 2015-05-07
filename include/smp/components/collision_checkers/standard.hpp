#ifndef _SMP_COLLISION_CHECKER_STANDARD_HPP_
#define _SMP_COLLISION_CHECKER_STANDARD_HPP_

#include <smp/components/collision_checkers/standard.h>

#include <smp/components/collision_checkers/base.hpp>
#include <smp/common/region.hpp>


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
::collision_checker_standard () {
  
  num_discretization_steps = 20;
  discretization_length = 0.1;  
  discretization_method = 2;
  //introducing the size of the robot
  size_robot=0.30;
}


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
::~collision_checker_standard () {

  for (typename list<region_t*>::iterator iter = list_obstacles.begin();
       iter != list_obstacles.end(); iter++) {
    
    region_t *region_curr = *iter;
    
    delete region_curr;
  }
}

/**
 * added to clean obstacles
 */

template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
:: clean_obs (int a) {
  list_obstacles.clear();
  
  //for (typename list<region_t*>::iterator iter = list_obstacles.begin();
    //   iter != list_obstacles.end(); iter++) {
    
    //region_t *region_curr = *iter;
    
    //delete region_curr;
  
  //}
  a=1;
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS>
::cc_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS>
::cc_update_insert_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS>
::cc_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS>
::cc_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
::check_collision_state (state_t *state_in) {

  if (list_obstacles.size() == 0)
    return 1;
  
  for (typename list<region_t *>::iterator iter = list_obstacles.begin(); iter != list_obstacles.end(); iter++) {
    region_t *region_curr = *iter;
    
    bool collision = true; 

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      
      if ( fabs((*state_in)[i] - region_curr->center[i]) >= (region_curr->size[i]/2.0+this->size_robot/3)) 
	collision = false;
    }
    
    if (collision)  {
      return 0;
    }
  }
  
  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
::check_collision_trajectory (trajectory_t *trajectory_in) {
  
  
  if (list_obstacles.size() == 0)
    return 1;
  
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

	for (typename list<region_t *>::iterator iter = list_obstacles.begin(); 
	     iter != list_obstacles.end(); iter++) {

	  region_t *region_curr = *iter;

	  for (int idx_state = 1; idx_state <= num_increments; idx_state++){
	    bool collision = true;

	    for (int i = 0; i < NUM_DIMENSIONS; i++) {
	      if (fabs((*state_prev)[i] + increments[i]*idx_state - region_curr->center[i]) 
		  >= region_curr->size[i]/2.0) {
		collision = false;
	      }
	    }
	    if (collision == true) {
	      return 0;
	    }
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
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
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
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
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
int smp::collision_checker_standard<typeparams,NUM_DIMENSIONS> 
::add_obstacle (region_t &obstacle_in) {
  
  list_obstacles.push_back (new region_t(obstacle_in));
  
  return 1;
}


#endif
