#ifndef _SMP_COLLISION_CHECKER_MU_CALCULUS_HPP_
#define _SMP_COLLISION_CHECKER_MU_CALCULUS_HPP_


#include <smp/components/collision_checkers/mu_calculus.h>

#include <smp/components/collision_checkers/base.hpp>
#include <smp/common/region.hpp>


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
::collision_checker_mu_calculus () {
  
  num_discretization_steps = 20;
  discretization_length = 0.1;  
  discretization_method = 2;
}


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
::~collision_checker_mu_calculus () {

  for (typename list<region_t*>::iterator iter = list_regions.begin();
       iter != list_regions.end(); iter++) {
    
    region_t *region_curr = *iter;
    
    delete region_curr;
  }
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS>
::cc_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS>
::cc_update_insert_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS>
::cc_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS>
::cc_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
::check_collision_state (state_t *state_in) {

  return 1; // Accept all states

  if (list_regions.size() == 0)
    return 1;
  
  for (typename list<region_t *>::iterator iter = list_regions.begin(); iter != list_regions.end(); iter++) {
    region_t *region_curr = *iter;
    
    bool collision = true; 

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      
      if ( fabs((*state_in)[i] - region_curr->center[i]) >= region_curr->size[i]/2.0) 
	collision = false;
    }
    
    if (collision)  {
      return 0;
    }
  }
  
  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS>
::get_region_index (double state_vars[NUM_DIMENSIONS]) {

  int idx_curr = 0;
  
  for (typename list<region_t*>::iterator it_region = list_regions.begin();
       it_region != list_regions.end(); it_region++) {
    
    region_t *region_curr = *it_region;
    idx_curr++;
    
    bool state_in_this_region = true;
    
    // Check whether the state is in this region
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      
      if ( fabs (state_vars[i] - region_curr->center[i]) >= region_curr->size[i]/2.0 ) {
	state_in_this_region = false;
	break;
      }
    }

    // If the state is in this region then immediately return the current region index
    if (state_in_this_region == true) {
      return idx_curr;
    }
  }
  

  // If the execution got here, then state_in is not in any particular region,
  //    in which case this function returns zero.
  return 0;


  return 1;
}



// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
::check_collision_trajectory (trajectory_t *trajectory_in) {

  
  if (list_regions.size() == 0)
    return 1;
  
  if (trajectory_in->list_states.size() == 0)
    return 1;

  
  int num_traversals = 0;  // Number of transitions from one region to another.
  
  // Start the collision checking procedure with the first state in the trajectory
  typename list<state_t*>::iterator iter = trajectory_in->list_states.begin();
  state_t *state_prev = *iter;

  // Determine the region that the first state in the trajectory lies in.
  double state_vars[NUM_DIMENSIONS];
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    state_vars[i] = (*state_prev)[i];
  }
  
  int idx_region_prev = get_region_index (state_vars);
  int idx_region_curr = -1;

  // Continue with the remaining states.
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
	  
	  for (int i = 0; i < NUM_DIMENSIONS; i++) 
	    state_vars[i] = (*state_prev)[i] + increments[i]*idx_state;
	    
	  // Check the region of the current interpolated state
	  idx_region_curr = get_region_index (state_vars);
	  
	  if (idx_region_curr != idx_region_prev) {
	    num_traversals++;
	    if (num_traversals >= 2) // If the number of traversals exceeds one then
	      return 0;              //   return collision.
	  }
	  idx_region_prev = idx_region_curr;
	  
	}
      }
    }
    
    for (int i = 0; i < NUM_DIMENSIONS; i++) 
      state_vars[i] = (*state_curr)[i];
    idx_region_curr = get_region_index (state_vars);
    if (idx_region_curr != idx_region_prev){
      num_traversals++;
      if (num_traversals >= 2)  // If the number of traversals exceeds one then 
	return 0;               //   return collision.
    }
    idx_region_prev = idx_region_curr;

    state_prev = state_curr;

    
  }     
  
  
  // If the execution reaches this point, then 
  return 1; // return no collision.
}


template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
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
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
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
int smp::collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> 
::add_region (region_t &obstacle_in) {
  
  list_regions.push_back (new region_t(obstacle_in));
  
  return 1;
}


#endif
