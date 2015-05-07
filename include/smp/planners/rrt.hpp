#ifndef _SMP_PLANNER_RRT_HPP_
#define _SMP_PLANNER_RRT_HPP_


#include <smp/planners/rrt.h>

#include <smp/planners/base_incremental.hpp>


template< class typeparams >
smp::rrt<typeparams>
::rrt () {
  

}



template< class typeparams >
smp::rrt<typeparams>
::~rrt () {

  
}


template< class typeparams >
smp::rrt<typeparams>
::rrt (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
       collision_checker_t &collision_checker_in, model_checker_t &model_checker_in) :
  planner_incremental<typeparams>(sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in) {
  
  
}


template< class typeparams >
int smp::rrt<typeparams>
::iteration () {
  
  // TODO: Check whether the rrt is initialized properly (including its base classes)
  
  // 1. Sample a new state from the obstacle-free space
  state_t *state_sample;
  this->sampler.sample (&state_sample);
  if (this->collision_checker.check_collision_state (state_sample) == 0) {
    delete state_sample;
    return 0; 
  }
  

  x=state_sample->state_vars[0];
  y=state_sample->state_vars[1];
  z=state_sample->state_vars[2];

  // 2. Find the nearest vertex
  vertex_t *vertex_nearest;
  this->distance_evaluator.find_nearest_vertex (state_sample, (void **)&vertex_nearest);


  // 3. Extend the nearest vertex towards the sample
  int exact_connection = -1;
  trajectory_t *trajectory = new trajectory_t; 
  list<state_t*> *intermediate_vertices = new list<state_t*>;
  if (this->extender.extend (vertex_nearest->state, state_sample,
			     &exact_connection, trajectory, intermediate_vertices) == 1) { // If the extension is successful
    // 4. Check the new trajectory for collision
    if (this->check_extended_trajectory_for_collision (vertex_nearest->state, trajectory) == 1) { // If collision free
      

      vertex_t *vertex_parent = vertex_nearest;
      trajectory_t *trajectory_parent = trajectory;
      list<state_t*> *intermediate_vertices_parent = intermediate_vertices;
      
      double cost_trajectory_from_parent = this->min_time_reachability.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
      double cost_parent = vertex_parent->data.total_cost + cost_trajectory_from_parent;

      // 5. Add the new collision-free trajectory to the tree
      this->insert_trajectory (vertex_nearest, trajectory, intermediate_vertices);

// Update the cost of the edge and the vertex      
      vertex_t *vertex_last = this->list_vertices.back();
      vertex_last->data.total_cost = cost_parent;
      this->min_time_reachability.ce_update_vertex_cost (vertex_last);
      
      edge_t *edge_last = vertex_parent->outgoing_edges.back();
      edge_last->data.edge_cost = cost_trajectory_from_parent;
      
      // Exit with success
      delete state_sample;
      return 1; 
    }
  }


  // 6. Handle the error case
  // If the extension was not successful, or the trajectory was not collision free,
  //     then free the memory and return failure.
  delete state_sample;
  delete trajectory;
  delete intermediate_vertices;
  
  // Exit with error
  return 0;

}

#endif
