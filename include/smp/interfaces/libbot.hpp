#ifndef _SMP_INTERFACE_LIBBOT_HPP_
#define _SMP_INTERFACE_LIBBOT_HPP_

#include <smp/interfaces/libbot.h>

#include <smp/planners/base.hpp>

#include <smp/interfaces/base.hpp>
#include <smp/common/region.hpp>


smp::interface_libbot_environment
::interface_libbot_environment () {
  
  
}


smp::interface_libbot_environment
::~interface_libbot_environment () {

  // Clear the memory occupied by obstacles.
  for (list<region_t*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++) {
    region_t *obstacle_curr = *iter;
    delete obstacle_curr;
  }
}


int smp::interface_libbot_environment
::set_operating_region (const region_t &region_in) {
  
  operating = region_in;
  
  return 1;
}


int smp::interface_libbot_environment
::set_goal_region (const region_t &region_in) {
  
  goal = region_in;

  return 1;
}


int smp::interface_libbot_environment
::clear_obstacle_list () {
  
  obstacles.clear();

  return 1;
}


int smp::interface_libbot_environment
::add_obstacle (const region_t &region_in) {
  
  obstacles.push_back (new region_t(region_in));

  return 1;
}



template< class typeparams >
smp::interface_libbot<typeparams> 
::interface_libbot () {

  lcm = globals_get_lcm ();
  visualize_3d_on = 0;
  
  this->planner_int = 0;
  
}


template< class typeparams >
smp::interface_libbot<typeparams> 
::~interface_libbot () {
  
}


template< class typeparams >
int smp::interface_libbot<typeparams> 
::set_planner (planner_t *planner_in) {

  this->planner_int = planner_in;
  
  return 1;
}

template< class typeparams >
int smp::interface_libbot<typeparams> 
::visualize_2d () {
  
  visualize_3d_on = 0;
  
  return 1;
}

template< class typeparams >
int smp::interface_libbot<typeparams> 
::visualize_3d () {
  
  visualize_3d_on = 1;
  
  return 1;
}


template< class typeparams >
int smp::interface_libbot<typeparams> 
::publish_environment (const environment_t &environment_in) {
  
  lcmtypes_smp_environment_t *environment = (lcmtypes_smp_environment_t *) malloc (sizeof (lcmtypes_smp_environment_t));

  // Fill in the operating and goal regions.
  for (int j = 0; j < 3; j++) {
    environment->operating.center[j] = environment_in.operating.center[j];
    environment->operating.size[j] = environment_in.operating.size[j];
    
    environment->goal.center[j] = environment_in.goal.center[j];
    environment->goal.size[j] = environment_in.goal.size[j];
  }


  // Fill in the obstacles
  environment->num_obstacles = environment_in.obstacles.size();
  
  if (environment->num_obstacles > 0) {
    
    environment->obstacles = (lcmtypes_region_3d_t*) malloc (environment->num_obstacles * sizeof (lcmtypes_region_3d_t));
    
    int i = 0;
    
    for (list<region_t*>::const_iterator iter = environment_in.obstacles.begin(); 
	 iter != environment_in.obstacles.end(); iter++) {
      
      region_t *obstacle_curr = *iter;
      
      for (int j = 0; j < 3; j++) {
	environment->obstacles[i].center[j] = obstacle_curr->center[j];
	environment->obstacles[i].size[j] = obstacle_curr->size[j];
      }
      
      i++;
    }
  }

  // Publish the message
  lcmtypes_smp_environment_t_publish (this->lcm, "SMP_ENVIRONMENT", environment);
  
  // Free the associated memory
  lcmtypes_smp_environment_t_destroy (environment);
  
  return 1;
}


template< class typeparams >
int smp::interface_libbot<typeparams> 
::publish_data () {


  if (this->planner_int == 0)
    return 0;

  list <vertex_t *> *list_vertices = &(this->planner_int->list_vertices);

  lcmtypes_smp_graph_t graph;

  graph.num_vertices = list_vertices->size ();
  graph.vertices = (lcmtypes_smp_vertex_t *) malloc (graph.num_vertices * sizeof (lcmtypes_smp_vertex_t));

  int i = 0;
  int num_edges = 0;
  for (typename list<vertex_t *>::iterator iter = list_vertices->begin(); 
       iter != list_vertices->end(); iter++) {

    vertex_t *vertex_curr = *iter;
    state_t &state_ref = *(vertex_curr->state);

    graph.vertices[i].state.x = state_ref[0];
    graph.vertices[i].state.y = state_ref[1];
    if (visualize_3d_on)
      graph.vertices[i].state.z = state_ref[2];
    else
      graph.vertices[i].state.z = 0.0;
    i++;

    num_edges += (*iter)->incoming_edges.size();// + (*iter)->outgoing_edges.size();

  }
  
  graph.num_edges = num_edges;
  graph.edges = (lcmtypes_smp_edge_t *) malloc (graph.num_edges * sizeof (lcmtypes_smp_edge_t));

  i = 0;
  for (typename list<vertex_t *>::iterator it_vertex = list_vertices->begin(); 
       it_vertex != list_vertices->end(); it_vertex++) {
    
    vertex_t *vertex_curr = *it_vertex;
    
    list< edge_t* > *incoming_edges_curr = &(vertex_curr->incoming_edges);
    for (typename list<edge_t*>::iterator it_edge = incoming_edges_curr->begin();
  	 it_edge != incoming_edges_curr->end(); it_edge++) {

      edge_t *edge_curr = *it_edge;
      
      state_t &state_src_ref = *(edge_curr->vertex_src->state);

      graph.edges[i].vertex_src.state.x = state_src_ref[0];
      graph.edges[i].vertex_src.state.y = state_src_ref[1];
      if (visualize_3d_on) 
  	graph.edges[i].vertex_src.state.z = state_src_ref[2];
      else
  	graph.edges[i].vertex_src.state.z = 0.0;
      
      state_t &state_dst_ref = *(edge_curr->vertex_dst->state);

      graph.edges[i].vertex_dst.state.x = state_dst_ref[0];
      graph.edges[i].vertex_dst.state.y = state_dst_ref[1];
      if (visualize_3d_on) 
  	graph.edges[i].vertex_dst.state.z = state_dst_ref[2];
      else
  	graph.edges[i].vertex_dst.state.z = 0.0;
      
      
      list<state_t*> *list_states_curr = &(edge_curr->trajectory_edge->list_states);
      if (list_states_curr->size () == 0) {
  	graph.edges[i].trajectory.num_states = 0;
  	graph.edges[i].trajectory.states = NULL;
      }
      else {
	
  	graph.edges[i].trajectory.num_states = list_states_curr->size();

  	graph.edges[i].trajectory.states = (lcmtypes_smp_state_t *) malloc (graph.edges[i].trajectory.num_states
  									    * sizeof (lcmtypes_smp_state_t));
	int j = 0;
  	for (typename list<state_t*>::iterator it_state = list_states_curr->begin();
  	     it_state != list_states_curr->end(); it_state++) {
	  
	  state_t *state_traj_curr = *it_state;
	  state_t &state_traj_ref = *state_traj_curr;
	  
  	  graph.edges[i].trajectory.states[j].x = state_traj_ref[0];
  	  graph.edges[i].trajectory.states[j].y = state_traj_ref[1];
  	  if (visualize_3d_on) 
  	    graph.edges[i].trajectory.states[j].z = state_traj_ref[2];
  	  else
  	    graph.edges[i].trajectory.states[j].z = 0.0;
	  j++;
  	}
      }
      i++;
    }
  }
  
  
  lcmtypes_smp_graph_t_publish (lcm, "SMP_GRAPH", &graph);

  // lcmtypes_smp_graph_t_destroy (&graph);
  
  return 1;
}


template< class typeparams >
int smp::interface_libbot<typeparams> 
::publish_trajectory (trajectory_t &trajectory_in) {

  lcmtypes_smp_trajectory_t lcm_traj;

  lcm_traj.num_states = trajectory_in.list_states.size();
  lcm_traj.states = (lcmtypes_smp_state_t *) malloc (lcm_traj.num_states * sizeof (lcmtypes_smp_state_t));


  int i = 0;
  for (typename list<state_t*>::iterator it_state = trajectory_in.list_states.begin(); 
       it_state != trajectory_in.list_states.end(); it_state++) {

    state_t *state_curr = *it_state;
    
    lcm_traj.states[i].x = (*state_curr)[0];
    lcm_traj.states[i].y = (*state_curr)[1];
    if (visualize_3d_on)
      lcm_traj.states[i].z = (*state_curr)[2];
    else
      lcm_traj.states[i].z = 0.0;

    i++;
  }
  
  lcmtypes_smp_trajectory_t_publish (lcm, "SMP_TRAJECTORY_OPTIMAL", &lcm_traj);

  return 1;
}


#endif
