#ifndef _SMP_SAMPLER_TRAJECTORY_BIAS_HPP_
#define _SMP_SAMPLER_TRAJECTORY_BIAS_HPP_

#include <iostream>
#include <cstdlib>
#include <cmath>

#include <smp/components/samplers/trajectory_bias.h>

#include <smp/components/samplers/base.hpp>
#include <smp/common/region.hpp>
#include <smp/planner_utils/trajectory.hpp>


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sampler_trajectory_bias () {

  bias_probability = 0.1;
  
  dispersion = 1.0;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }

  sample_trajectory.clear();
  
  length_sample_trajectory = -1.0;
  
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::~sampler_trajectory_bias () {
    

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {
  
  if (NUM_DIMENSIONS <= 0)
    return 0;
  
  
  if (length_sample_trajectory > 0.0) {
    
    
    // 1. Test the threshold
    double threshold_test = rand()/(RAND_MAX + 1.0);
    if (threshold_test <= bias_probability)  {

    
      // 2. Generate a sample from box of side length sampler_trajectory_bias::dispersion
      state_t *state_new = new state_t;
      for (int i = 0; i < NUM_DIMENSIONS; i++)  {
	(*state_new)[i] = (rand()/(RAND_MAX + 1.0) - 0.5)*dispersion;
      }
     
      
      // 3 Sample a point along the trajectory
      double sample_length = rand()/(RAND_MAX + 1.0) * length_sample_trajectory;
      
      
      // 4. Find the point along the trajecotry that corresponds to the sample 
      double length_curr = 0.0;      
      typename list<state_t*>::iterator it_state = sample_trajectory.list_states.begin();
      state_t *state_prev = *it_state;

      it_state++;
      for (; it_state != sample_trajectory.list_states.end(); it_state++) {

	state_t *state_curr = *it_state;

    
	double length_curr_segment = 0.0;  // Calculate the length of this segment 
	for (int i = 0; i < NUM_DIMENSIONS; i++) {
	  double length_sqrt = (*state_curr)[i] - (*state_prev)[i];
	  length_curr_segment += length_sqrt * length_sqrt;
	}
	length_curr_segment = sqrt (length_curr_segment);
    
	length_curr += length_curr_segment; // Add the length of this segment to the total length
    
	if (length_curr > sample_length) { // If the length has exceeded the sample then stop.
	  double increment = (length_curr - sample_length)/length_curr_segment;

	  for (int i = 0; i < NUM_DIMENSIONS; i++) // Move the sample along the trajectory
	    (*state_new)[i] += increment*((*state_curr)[i]) + (1.0 - increment)*((*state_prev)[i]);
	  
	  break;
	}
	
	state_prev = state_curr; // Update the previous state
      }
      

      // 5. Check whether the new state is within the support
      bool sample_in_bounds = true;
      
      for (int i = 0; i < NUM_DIMENSIONS; i++) {
	
	if ( fabs((*state_new)[i] - support.center[i]) >= (support.size[i])/2.0 ) {
	  sample_in_bounds = false;
	  break;
	}
      }


      // 6. Setup the output variables and return
      if (sample_in_bounds) {

	*state_sample_out = state_new; 	
	return 1;
      }
      else {  // Clean up the memory if trajectory sampling failed. 
  	delete state_new;
      }
    }
  }
  
  
  // If no trajectory biasing, then sample a state uniformly from the support.
  state_t *state_uniform = new state_t;
      
  for (int i = 0; i < NUM_DIMENSIONS; i++)      // Generate an independent random variable for each axis.
    (*state_uniform)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];
  
  *state_sample_out = state_uniform;
  
  return 1;

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS> 
::update_trajectory (trajectory_t *trajectory_in) {
  

  { // Update the sample trajectory
    sample_trajectory.clear_delete ();
    
    for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin();
	 it_state != trajectory_in->list_states.end(); it_state++) {
      
      state_t *state_curr = *it_state;
      sample_trajectory.list_states.push_back (new state_t(*state_curr));
    }
    
    for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin();
	 it_input != trajectory_in->list_inputs.end(); it_input++) {
      
      input_t *input_curr = *it_input;
      sample_trajectory.list_inputs.push_back (new input_t(*input_curr));
    }
  }
  
  
  { // Calculate the length of the trajectory 
    length_sample_trajectory = 0.0; 
    
    typename list<state_t*>::iterator it_state = sample_trajectory.list_states.begin();
    
    state_t *state_prev = *it_state;
    it_state++;
    for (; it_state != sample_trajectory.list_states.end(); it_state++) {
      state_t *state_curr = *it_state; 
    
      double length_curr_segment = 0.0;  // Calculate the length of this segment 
      for (int i = 0; i < NUM_DIMENSIONS; i++) {

	double length_sqrt = (*state_curr)[i] - (*state_prev)[i];
	length_curr_segment += length_sqrt * length_sqrt;
      }
      
      length_curr_segment = sqrt (length_curr_segment);
    
      length_sample_trajectory += length_curr_segment; // Add the length of this segment to the total length
      
      state_prev = state_curr; // Update the previous state
    }
  }
  
  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS> 
::set_sample_dispersion (double dispersion_in) {

  if (dispersion_in > 0.0)  {
    dispersion = dispersion_in;
    return 1;
  }
  
  return 0;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_trajectory_bias<typeparams,NUM_DIMENSIONS> 
::set_bias_probability (double bias_probability_in) {

  if (bias_probability_in > 0.0)  {
    bias_probability = bias_probability_in;
    return 1;
  }
  
  return 0;
}



#endif
