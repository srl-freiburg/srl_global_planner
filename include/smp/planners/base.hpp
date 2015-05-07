#ifndef _SMP_PLANNER_BASE_HPP_
#define _SMP_PLANNER_BASE_HPP_

#include <iostream>
#include <list>


#include <smp/planners/base.h>

#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/planner_utils/trajectory.hpp>

#include <smp/components/extenders/base.hpp>
#include <smp/components/samplers/base.hpp>
#include <smp/components/collision_checkers/base.hpp>
#include <smp/components/distance_evaluators/base.hpp>
#include <smp/components/model_checkers/base.hpp>

using namespace std; 



template< class typeparams >
smp::planner<typeparams> 
::planner () {
  
  list_vertices.clear ();
  num_vertices = 0;

}


template< class typeparams >
smp::planner<typeparams> 
::planner (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
	   collision_checker_t &collision_checker_in, model_checker_t &model_checker_in) 
  : sampler(sampler_in), distance_evaluator(distance_evaluator_in), extender(extender_in), 
    collision_checker(collision_checker_in), model_checker(model_checker_in) {
  
  list_vertices.clear ();
  num_vertices = 0;

}


template< class typeparams >
smp::planner<typeparams> 
::~planner () {

  initialize ();  // run the initialization function that clears up
                  //     the memory occupied by the graph.


}


template< class typeparams >
int smp::planner<typeparams> 
::initialize () {

  // Delete all edges and vertices
  for (typename list<vertex_t*>::iterator iter_vertex = list_vertices.begin(); iter_vertex != list_vertices.end(); iter_vertex++) {
    vertex_t *vertex_curr = *iter_vertex;
    for (typename list<edge_t*>::iterator iter_edge = vertex_curr->outgoing_edges.begin(); 
  	 iter_edge != vertex_curr->outgoing_edges.end(); iter_edge++) {
      edge_t *edge_curr = *iter_edge;
      delete edge_curr;
    }
  }

  for (typename list< vertex_t* >::iterator iter = list_vertices.begin();
       iter != list_vertices.end(); iter++) {
    vertex_t *vertex_curr = *iter;
    delete vertex_curr;
  }
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::insert_vertex (vertex_t *vertex_in) {
  
  // insert the vertex to the list of vertices
  vertex_in->vertex_id=num_vertices;
  list_vertices.push_back (vertex_in);
  num_vertices++;
  
  
#if _SMP_FAST_VERTEX_DELETE
  
  vertex_in->it_vertex_list = list_vertices.end();
  (vertex_in->it_vertex_list)--;
  
#endif

  // UPDATE ALL COMPONENTS
  sampler.sm_update_insert_vertex (vertex_in);
  distance_evaluator.de_update_insert_vertex (vertex_in);
  extender.ex_update_insert_vertex (vertex_in);
  collision_checker.cc_update_insert_vertex (vertex_in);
  model_checker.mc_update_insert_vertex (vertex_in);
  
  // Run all the update functions
  for (typename list<vertex_update_func_t>::iterator it_func = list_update_insert_vertex_functions.begin();
       it_func != list_update_insert_vertex_functions.end(); it_func++) {
    
    (*it_func) (vertex_in);
  }

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::delete_vertex (vertex_t *vertex_in) {
  

  // UPDATE ALL COMPONENTS
  sampler.sm_update_delete_vertex (vertex_in);
  distance_evaluator.de_update_delete_vertex (vertex_in);
  extender.ex_update_delete_vertex (vertex_in);
  collision_checker.cc_update_delete_vertex (vertex_in);
  model_checker.mc_update_delete_vertex (vertex_in);

  // Run all the update functions
  for (typename list<vertex_update_func_t>::iterator it_func = list_update_delete_vertex_functions.begin();
       it_func != list_update_delete_vertex_functions.end(); it_func++) {    
    (*it_func) (vertex_in);
  }
  

  list<edge_t*> edge_list; 

  edge_list = vertex_in->incoming_edges;
  for (typename list<edge_t*>::iterator it_edge = edge_list.begin();
       it_edge != edge_list.end(); it_edge++) {
    edge_t *edge_curr = *it_edge;
    this->delete_edge(edge_curr);
  }
  edge_list.clear();


  edge_list = vertex_in->outgoing_edges;
  for (typename list<edge_t*>::iterator it_edge = edge_list.begin();
       it_edge != edge_list.end(); it_edge++) {
    edge_t *edge_curr = *it_edge;
    this->delete_edge(edge_curr);
  }
  edge_list.clear();

#if _SMP_FAST_VERTEX_DELETE
  
  typename list<vertex_t*>::iterator it_vertex_begin = vertex_in->it_vertex_list;
  typename list<vertex_t*>::iterator it_vertex_end = it_vertex_begin;
  it_vertex_end++;
  list_vertices.erase (it_vertex_begin, it_vertex_end);

#else

  list_vertices.remove (vertex_in);

#endif

  num_vertices--;

  delete vertex_in;
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::insert_edge (vertex_t *vertex_src_in, edge_t *edge_in, vertex_t *vertex_dst_in) {


  // WARNING: Overriding pointed data. May cause memory leaks.
  edge_in->vertex_src = vertex_src_in;
  edge_in->vertex_dst = vertex_dst_in;

  vertex_src_in->outgoing_edges.push_back (edge_in);
  vertex_dst_in->incoming_edges.push_back (edge_in);

  // UPDATE ALL COMPONENTS
  sampler.sm_update_insert_edge (edge_in);
  distance_evaluator.de_update_insert_edge (edge_in);
  extender.ex_update_insert_edge (edge_in);
  collision_checker.cc_update_insert_edge (edge_in);
  model_checker.mc_update_insert_edge (edge_in);

  // Run all the update functions
  for (typename list<edge_update_func_t>::iterator it_func = list_update_insert_edge_functions.begin();
       it_func != list_update_insert_edge_functions.end(); it_func++) {
    (*it_func) (edge_in);
  }
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::delete_edge (edge_t *edge_in) {
    
  // UPDATE ALL COMPONENTS
  sampler.sm_update_delete_edge (edge_in);
  distance_evaluator.de_update_delete_edge (edge_in);
  extender.ex_update_delete_edge (edge_in);
  collision_checker.cc_update_delete_edge (edge_in);
  model_checker.mc_update_delete_edge (edge_in);

  // Run all the update functions
  for (typename list<edge_update_func_t>::iterator it_func = list_update_delete_edge_functions.begin();
       it_func != list_update_delete_edge_functions.end(); it_func++) {
    (*it_func) (edge_in);
  }


  edge_in->vertex_src->outgoing_edges.remove (edge_in);
  edge_in->vertex_dst->incoming_edges.remove (edge_in);
  
  delete edge_in;
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::insert_trajectory (vertex_t *vertex_src_in, trajectory_t *trajectory_in, 
		     list<state_t*> *intermediate_vertices_in, vertex_t *vertex_dst_in) {
  
  // TODO: take the intermediate vertices into account
    
  vertex_t *vertex_dst = vertex_dst_in;

  // If no vertex_dst_in is given, then create a new 
  //   vertex using the final state in trajectory_in 
  if (vertex_dst == NULL) {
    state_t *final_state = trajectory_in->list_states.back();
    trajectory_in->list_states.pop_back();
    
    vertex_dst = new vertex_t;
    vertex_dst->state = final_state;
    
    this->insert_vertex (vertex_dst);  // Insert the new vertex into the graph
  }
  
  // Create the new edge
  edge_t *edge = new edge_t;
  edge->trajectory_edge = trajectory_in;
  this->insert_edge (vertex_src_in, edge, vertex_dst);  // Insert the new edge into the graph

  if (intermediate_vertices_in)
    delete intermediate_vertices_in;
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::insert_trajectories (vertex_t *vertex_src_in, list< trajectory_t *> *list_trajectories_in, vertex_t *vertex_dst_in) {

  // Exit if there no trajectories in the list
  if (list_trajectories_in->size() == 0)
    return 1;

  // Initialize the previous vertex
  vertex_t *vertex_prev = vertex_src_in;

  for (typename list< trajectory_t * >::iterator iter = list_trajectories_in->begin(); 
       iter != list_trajectories_in->end(); iter++) {
    
    // Get current trajectory 
    trajectory_t *trajectory_curr = *iter;

    // Extract final state from the trajectory
    state_t *final_state = trajectory_curr->list_states.back();
    trajectory_curr->list_states.pop_back();

    // Create the edge data structure
    edge_t *edge_curr = new edge_t;    

    // Create the vertex data structure
    vertex_t *vertex_curr;
    // If this is the last trajectory and connection to a final vertex is required
    iter++;
    if ( (iter == list_trajectories_in->end()) && (vertex_dst_in != 0) ) {
      vertex_curr = vertex_dst_in;
    }
    else  {  // Otherwise create a new vertex
      vertex_curr = new vertex_t;
      vertex_curr->state = final_state;
    
      // Insert the new vertex into the graph
      this->insert_vertex (vertex_curr);
    }
    iter--;
    
    
    // Insert the new edge into the graph
    edge_curr->trajectory = trajectory_curr;
    this->insert_edge (vertex_prev, edge_curr, vertex_curr);

    // Update the previous vertex
    vertex_prev = vertex_curr;
  }

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::init_sampler (sampler_t &sampler_in) {
  
  sampler = sampler_in;

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::init_distance_evaluator (distance_evaluator_t &distance_evaluator_in) {
  
  distance_evaluator = distance_evaluator_in;

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::init_extender (extender_t &extender_in) {
  
  extender = extender_in;

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::init_collision_checker (collision_checker_t &collision_checker_in) {
  
  collision_checker = collision_checker_in;

  return 1;
}


template< class typeparams >
int smp::planner<typeparams> 
::init_model_checker (model_checker_t &model_checker_in) {
  
  model_checker = model_checker_in;

  return 1;
}



template< class typeparams >
int smp::planner<typeparams>
::clear_update_function_list_vertex_insert () {
  
  list_update_insert_vertex_functions.clear ();
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams>
::register_new_update_function_vertex_insert (vertex_update_func_t *vertex_update_func_in) {
  
  list_update_insert_vertex_functions.push_back (vertex_update_func_in);
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams>
::clear_update_function_list_vertex_delete ()  {
  
  list_update_delete_vertex_functions.clear ();
  
  return 1;
}


 
template< class typeparams >
int smp::planner<typeparams>
::register_new_update_function_vertex_delete (vertex_update_func_t *vertex_update_func_in)  {
  
  list_update_delete_vertex_functions.push_back (vertex_update_func_in);
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams>
::clear_update_function_list_edge_insert ()  {
  
  list_update_insert_edge_functions.clear ();
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams>
::register_new_update_function_edge_insert (edge_update_func_t *edge_update_func_in) {
  
  list_update_insert_edge_functions.push_back (edge_update_func_in);
  
  return 1;
}



template< class typeparams >
int smp::planner<typeparams>
::clear_update_function_list_edge_delete ()  {
  
  list_update_delete_edge_functions.clear ();
  
  return 1;
}


template< class typeparams >
int smp::planner<typeparams>
::register_new_update_function_edge_delete (edge_update_func_t *edge_update_func_in)  {
  
  list_update_delete_edge_functions.push_back (edge_update_func_in);
  
  return 1;
}


#endif
