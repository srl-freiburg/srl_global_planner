#ifndef _SMP_PLANNER_RRG_HPP_
#define _SMP_PLANNER_RRG_HPP_

#include <smp/planners/rrg.h>

#include <smp/planners/base_incremental.hpp>
#include <smp/planners/planner_parameters.hpp>



template< class typeparams > 
smp::rrg<typeparams>
::rrg () {

}


template< class typeparams > 
smp::rrg<typeparams>
::~rrg () {

}



template< class typeparams >
smp::rrg<typeparams>
::rrg (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
       collision_checker_t &collision_checker_in, model_checker_t &model_checker_in) :
  planner_incremental<typeparams>(sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in) {
  
  
}


template< class typeparams > 
int smp::rrg<typeparams>
::iteration () {
  
  // TODO: Check whether the rrt is initialized properly (including its base classes)
  
  // 1. Sample a new state from the obstacle-free space
  state_t *state_sample;
  this->sampler.sample (&state_sample);

  if (this->collision_checker.check_collision_state (state_sample) == 0) {
    delete state_sample;
    // cout << "State in collision" << endl;
    return 0; 
  }


  // 2. Find the nearest vertex
  vertex_t *vertex_nearest;
  this->distance_evaluator.find_nearest_vertex (state_sample, (void **)&vertex_nearest);
  
  
  // 3. Extend the nearest vertex towards the sample    
  double radius;
  if (parameters.get_fixed_radius() < 0.0) {
    double num_vertices = (double)(this->get_num_vertices());
    radius = parameters.get_gamma() * pow (log(num_vertices)/num_vertices,  1.0 /( (double)(parameters.get_dimension()) )  );
    if (radius > parameters.get_max_radius())
      radius = parameters.get_max_radius();
  }
  else 
    radius = parameters.get_fixed_radius();
  
  int exact_connection = -1;
  trajectory_t *trajectory = new trajectory_t; 
  list<state_t*> *intermediate_vertices = new list<state_t*>;
  if (this->extender.extend (vertex_nearest->state, state_sample,
			     &exact_connection, trajectory, intermediate_vertices) == 1) { // If the extension is successful
 
    // 4. Check the new trajectory for collision
    if (check_extended_trajectory_for_collision (vertex_nearest->state, trajectory) == 1) { // If the trajectory is collision free
      

      // 5. Add the collision-free trajectory to the graph and create the new vertex
      this->insert_trajectory (vertex_nearest, trajectory, intermediate_vertices);


      // 6. Make connections with vertices that lie within a ball of certain radius

      // Define the new variables that are used in both phase 1 and 2.
      list<void*> list_vertices_in_ball;
      vertex_t *vertex_extended = NULL;

      // 6.a. Find all vertices that lie within a ball of certain radius 
      if (parameters.get_phase() >= 1) {

	vertex_extended = (vertex_t *) (this->list_vertices.back()); // Get the end of the extension, which 
	//   is the last vertex added to the tree.
	this->distance_evaluator.find_near_vertices_r (vertex_extended->state, radius, &list_vertices_in_ball);
      
	// 6.b Extend from all nodes inside the ball to the extended vertex, and keep edges if exact connection can be made
	for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	  vertex_t *vertex_curr = (vertex_t*)(*iter);
	
	  // Skip if current vertex is the same as the extended vertex
	  if (vertex_curr == vertex_extended)  
	    continue;
	
	  if (vertex_curr == vertex_nearest)
	    continue;
	
	  // Attempt an extension from vertex_curr to the extended state
	  trajectory_t *trajectory_tmp = new trajectory_t;
	  list<state_t*> *intermediate_vertices_tmp = new list<state_t*>;
	  bool free_tmp_memory = true;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_curr->state, vertex_extended->state, 
				      &exact_connection, trajectory_tmp, intermediate_vertices_tmp) == 1) { 

	    // Check whether the connection is both exact and collision free
	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision (vertex_curr->state, trajectory_tmp) == 1) ) { 
	    
	      // Add the new collision-free trajectory to the graph
	      this->insert_trajectory (vertex_curr, trajectory_tmp, intermediate_vertices_tmp, vertex_extended);
	      free_tmp_memory = false;
	  
	    } //-- if ( (exact_connection == 1) && (this->collision_checker->check_collision_trajectory (trajectory_tmp) == 1) )

	  } //-- if (this->extender->extend (vertex_curr->state, vertex_extended->state,

	  // Free temporary memory
	  if (free_tmp_memory == true) {
	    delete trajectory_tmp;
	    delete intermediate_vertices_tmp;
	  }
	} //-- for (typename list<void*>::iterator iter = list_vertices_in_ball.begin();
      }
      

      // 6.c Extend from the extended vertex to all nodes inside the ball, and keep edges if exact connections can be made
      if (parameters.get_phase() >= 2) {

	for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	  vertex_t *vertex_curr = (vertex_t*)(*iter);
	
	  // Skip if the current vertex is the same as the extended vertex
	  if (vertex_curr == vertex_extended)
	    continue;
	
	  // Attempt an extension from the extended state to vertex_curr
	  trajectory_t *trajectory_tmp = new trajectory_t;
	  list<state_t*> *intermediate_vertices_tmp = new list<state_t*>;
	  bool free_tmp_memory = true;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_extended->state, vertex_curr->state,
				      &exact_connection, trajectory_tmp, intermediate_vertices_tmp) == 1) {
	  
	    // Check whether the connection is both exact and collision free
	    if ( (exact_connection == 1) 
		 && (check_extended_trajectory_for_collision (vertex_extended->state, trajectory_tmp) == 1) ) {

	      // Add the new collision-free trajectory to the graph
	      this->insert_trajectory (vertex_extended, trajectory_tmp, intermediate_vertices_tmp, vertex_curr);
	      free_tmp_memory = false;
	    } //-- if ( (exact_connection == 1) && (this->collision_checker->check_collision_trajectory (trajectory_tmp) == 1) )
	  } 
		
	  if (free_tmp_memory == true) {
	    delete trajectory_tmp;
	    delete intermediate_vertices_tmp;
	  }

	}//-- for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices.end(); iter++)
      }


      // Completed all the phases, return with success
      delete state_sample;
      return 1;
      
    }//-- if (this->collision_checker->check_collision_trajectory (trajectory) == 1)
  
  } //-- if (this->extender->extend (vertex_nearest->state, state_sample,


  // 7. Handle the error case
  // If the first extension was not successful, or the trajectory was not collision free,
  //     then free the memory and return failure
  delete state_sample;
  delete trajectory;
  delete intermediate_vertices;

  return 0;
}


#endif

