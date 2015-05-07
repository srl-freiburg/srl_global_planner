#ifndef _SMP_PLANNER_INCREMENTAL_HPP_
#define _SMP_PLANNER_INCREMENTAL_HPP_

#include <smp/planners/base_incremental.h>

#include <smp/planners/base.hpp>


template< class typeparams >
smp::planner_incremental<typeparams>
::planner_incremental () {

  root_vertex = 0;
}


template< class typeparams >
smp::planner_incremental<typeparams>
::~planner_incremental () {

  // Note that the root vertex is deleted by the smp_planner class
}

template< class typeparams >
smp::planner_incremental<typeparams>
::planner_incremental (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
		       collision_checker_t &collision_checker_in, model_checker_t &model_checker_in) 
  : planner<typeparams> (sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in) {
  
}


template< class typeparams >
int smp::planner_incremental<typeparams>
::initialize (state_t *initial_state_in) {
  
  planner_t::initialize(); // This function deletes all existing vertices 
                           // in the graph, including the root vertex.

  if (initial_state_in == 0) {
    root_vertex = 0;
    return 1;
  }
  
  root_vertex = new vertex_t;
  root_vertex->state = initial_state_in;

  this->insert_vertex (root_vertex);
  
  return 1;
}


#endif
