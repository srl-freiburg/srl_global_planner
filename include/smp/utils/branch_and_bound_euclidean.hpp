#ifndef _SMP_BRANCH_AND_BOUND_EUCLIDEAN_HPP_
#define _SMP_BRANCH_AND_BOUND_EUCLIDEAN_HPP_

#include <smp/utils/branch_and_bound_euclidean.h>

#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/utils/branch_and_bound_base.hpp>

#include <cmath>

template< class typeparams, int NUM_DIMENSIONS >
smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::branch_and_bound_euclidean () {
  
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 1.0;
  }
  
  root_vertex = NULL;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::~branch_and_bound_euclidean () {

}



template< class typeparams, int NUM_DIMENSIONS >
int smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::add_children_to_list (list<vertex_t*> &list_vertices_in, vertex_t *vertex_in) {
  
  list_vertices_in.push_back (vertex_in);
  
  for (typename list<edge_t*>::iterator it_edge = vertex_in->outgoing_edges.begin(); 
       it_edge != vertex_in->outgoing_edges.end(); it_edge++) {
    
    edge_t *edge_curr = *it_edge;
    
    add_children_to_list (list_vertices_in, edge_curr->vertex_dst);
  }
  
  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::run_branch_and_bound () {
  
  if (this->upper_bound_cost < 0.0) 
    return 0;

  list<vertex_t*> list_obsolete_vertices;
  
  // Go through all the vertices and check the constraints
  for (typename list<vertex_t*>::iterator it_vertex = this->planner_bnb->list_vertices.begin();
       it_vertex != this->planner_bnb->list_vertices.end(); it_vertex++) {

    vertex_t *vertex_curr = *it_vertex;
   if(root_vertex == vertex_curr){
        ROS_INFO("Skip root_vertex");
        continue;
      }
    state_t *state_curr = vertex_curr->state;
    
    // Get the cost to get to vertex_curr
    double total_cost = vertex_curr->data.total_cost;
    
    // Compute the cost-to-go heuristic     
    
    // --- Find the Euclidean distance between the current point and the center of the goal box 
    double distance = 0.0;
    double increments[NUM_DIMENSIONS];
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      increments[i] = (*state_curr)[i] - region_goal.center[i];
      distance += increments[i] * increments[i];
    }
    distance = sqrt(distance);

    // --- Find the maximum weighted increment (when scaled by the size of the box)
    int max_weighted_increment_idx = -1;
    double max_weighted_increment = -1.0;
    for (int i = 0; i < NUM_DIMENSIONS; i++) {  // Normalize the direction vector 
      increments[i] /= distance;
      double weighted_increment_curr = fabs (increments[i] * region_goal.size[i]/2.0);
      if (max_weighted_increment < weighted_increment_curr) {
    	max_weighted_increment_idx = i;
    	max_weighted_increment = weighted_increment_curr;
      }
    }

    // --- Calculate the residual distance, i.e., the length of the vector that lies inside the goal box
    double distance_residual = 0.0;
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      double distance_residual_curr = increments[i] * region_goal.size[max_weighted_increment_idx]/2.0;
      distance_residual += distance_residual_curr * distance_residual_curr;
    }
    distance_residual = sqrt (distance_residual);
    
    
    // --- Compute the heuristic cost and normalize
    double heuristic_cost = distance - distance_residual;
    if (heuristic_cost <= 0.0)
      heuristic_cost = 0.0;

    
    // If necessary, delete the vertex
    if (total_cost + heuristic_cost > this->upper_bound_cost + 5.0)  {
      
      // Check whether the state is actuall in the goal region
      bool inside_goal_region = true;
      for (int i = 0 ; i < NUM_DIMENSIONS; i++) {
      	if (fabs ((*state_curr)[i] - region_goal.center[i]) > region_goal.size[i])  {
      	  inside_goal_region = false;
      	  break;
      	}
      }

      if (inside_goal_region == false)
	       list_obsolete_vertices.push_back (vertex_curr);
    }
  }
  

  // Delete all the obsolete vertices
  for (typename list<vertex_t*>::iterator it_vertex = list_obsolete_vertices.begin();
       it_vertex != list_obsolete_vertices.end(); it_vertex++) {
    
        vertex_t *vertex_curr = *it_vertex;
        this->planner_bnb->delete_vertex (vertex_curr);
  }


  // Delete all vertices that do not have a parent
  list_obsolete_vertices.clear ();
  for (typename list<vertex_t*>::iterator it_vertex = this->planner_bnb->list_vertices.begin();
       it_vertex != this->planner_bnb->list_vertices.end(); it_vertex++) {
    
    vertex_t *vertex_curr = *it_vertex;
   if(root_vertex == vertex_curr){
        ROS_INFO("Skip root_vertex");
        continue;
      }
    if ( (vertex_curr->incoming_edges.empty()) && (vertex_curr != root_vertex)) {
      
      add_children_to_list (list_obsolete_vertices, vertex_curr);
    }
  }


  // Delete all the descendants of obsolete vertices
  for (typename list<vertex_t*>::iterator it_vertex = list_obsolete_vertices.begin();
       it_vertex != list_obsolete_vertices.end(); it_vertex++) {
    
    vertex_t *vertex_curr = *it_vertex;
    this->planner_bnb->delete_vertex (vertex_curr);
  }
  

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::set_goal_region (region_t region_goal_in) {
  
  region_goal = region_goal_in;

  return 1;
}




template< class typeparams, int NUM_DIMENSIONS >
int smp::branch_and_bound_euclidean<typeparams,NUM_DIMENSIONS>
::set_root_vertex (vertex_t *root_vertex_in) {

  this->root_vertex = root_vertex_in;
  
  return 1;
}


#endif
