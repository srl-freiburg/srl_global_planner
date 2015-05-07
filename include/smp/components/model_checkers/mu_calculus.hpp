#ifndef _SMP_MODEL_CHECKER_MU_CALCULUS_HPP_
#define _SMP_MODEL_CHECKER_MU_CALCULUS_HPP_

#include <smp/components/model_checkers/mu_calculus.h>

#include <smp/components/model_checkers/base.hpp>


template< class typeparams >
smp::model_checker_mu_calculus<typeparams>
::model_checker_mu_calculus () {

  uid_counter = 0;
  found_solution = false;
  
  ms.pt.parseFormula ("Phi"); // Formula parse tree is hardcoded now.
};


template< class typeparams >
smp::model_checker_mu_calculus<typeparams>
::~model_checker_mu_calculus () {
  
  
};


template< class typeparams >
int smp::model_checker_mu_calculus<typeparams>
::mc_update_insert_vertex (vertex_t *vertex_in) {

  // Create a new state
  MS_state *ms_state_new = new MS_state;

  // Assign a unique id
  ms_state_new->identifier = uid_counter++;

  state_t *state_curr = vertex_in->state;
  
  // Add propositions to ms_state
  
  if ( ((*state_curr)[0] > -4.0) && ((*state_curr)[0] < -3.0) &&
       ((*state_curr)[1] > -4.0) && ((*state_curr)[1] < -3.0) ){
    // cout << "region 1" << endl;
    ms_state_new->addprop (1);
  }
  
  if ( ((*state_curr)[0] > 5.0) && ((*state_curr)[0] < 6.0) &&
       ((*state_curr)[1] > 5.0) && ((*state_curr)[1] < 6.0) ) {
    // cout << "region 2" << endl;
    ms_state_new->addprop (2);
  }
  
  if ( ((*state_curr)[0] > 0.1) && ((*state_curr)[0] < 4.0) &&
       ((*state_curr)[1] > 0.1) && ((*state_curr)[1] < 4.0) ) {
    // cout << "region 3" << endl; 
  }
  else {
    ms_state_new->addprop (3);
  }

  ms_state_new->data = (void *) vertex_in;

  // Add state pointer into vertex data
  vertex_in->data.state = ms_state_new;
  
  // Add the new state to the model checker
  ms.addState (ms_state_new);
  
  return 1;
};


template< class typeparams >
int smp::model_checker_mu_calculus<typeparams>
::mc_update_insert_edge (edge_t *edge_in) {

  vertex_t *vertex_src = edge_in->vertex_src;
  
  vertex_t *vertex_dst = edge_in->vertex_dst;

  if (ms.addTransition (vertex_src->data.state, vertex_dst->data.state)) {
    found_solution = true;
    cout << "Found a solution" << endl;
  }
  
  return 1;
};


template< class typeparams >
int smp::model_checker_mu_calculus<typeparams>
::mc_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams >
int smp::model_checker_mu_calculus<typeparams>
::mc_update_delete_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams >
int smp::model_checker_mu_calculus<typeparams>
::get_solution (trajectory_t &trajectory_out) {
  
  // TODO: also put in the inputs...

  if (found_solution == false)
    return 0;

  stateList ms_state_list = ms.getTrajectory();
  
  list<vertex_t*> list_vertices;

  for (stateList::iterator it_ms_state = ms_state_list.begin(); it_ms_state != ms_state_list.end(); it_ms_state++) {

    vertex_t *vertex_curr = (vertex_t*) ((*it_ms_state)->data);

    list_vertices.push_back(vertex_curr);
    
    // cout << "State : ";
    // for (int i = 0; i < 2; i++)
    //   cout << vertex_curr->state->state_vars[i] << " , ";
    // cout << endl;     

  }


  if (list_vertices.size() == 0)
    return 0;
  

  vertex_t *vertex_prev = list_vertices.front ();
  list_vertices.pop_front ();

  trajectory_out.list_states.push_back (vertex_prev->state);

  // printf ("[%d]", vertex_prev);
  // cout << "State : ";
  // for (int i = 0; i < 2; i++)
  //   cout << vertex_prev->state->state_vars[i] << " , ";
  // cout << endl;     


  for (typename list<vertex_t*>::iterator it_vertex = list_vertices.begin(); it_vertex != list_vertices.end(); it_vertex++) {

    vertex_t *vertex_curr = *it_vertex;

    if (vertex_curr == vertex_prev)
      continue;
    
    // printf ("[%d]", vertex_curr);
    // cout << "State : ";
    // for (int i = 0; i < 2; i++)
    //   cout << vertex_curr->state->state_vars[i] << " , ";
    // cout << endl;     

    
    // find the edge between these two vertices
    edge_t *edge_found = 0;
    for (typename list<edge_t*>::iterator it_edge = vertex_curr->incoming_edges.begin(); 
    	 it_edge != vertex_curr->incoming_edges.end(); it_edge++) {
      
      edge_t *edge_curr = *it_edge;
      
      // cout << "  -  Examining : ";      
      // printf ("[%d]", edge_curr->vertex_src);
      // cout << "State : ";
      // for (int i = 0; i < 2; i++)
      // 	cout << edge_curr->vertex_src->state->state_vars[i] << " , ";
      // printf ("[%d]", edge_curr->vertex_dst);
      // cout << "- State : ";
      // for (int i = 0; i < 2; i++)
      // 	cout << edge_curr->vertex_dst->state->state_vars[i] << " , ";
      // cout << endl;           

      // printf ("-- Checking : [%d]=[%d] and [%d]=[%d]\n", edge_curr->vertex_src, vertex_prev, edge_curr->vertex_dst, vertex_curr);


      if ( (edge_curr->vertex_src == vertex_prev) && (edge_curr->vertex_dst == vertex_curr) ){
    	edge_found = edge_curr;
    	break;
      }
    }
    
    if (edge_found == 0) {
      cout << "No such edge" << endl;
      return 0;
    }
    
    
    // Add all trajectory on this edge to the new trajectory
    for (typename list<state_t*>::iterator it_state = edge_found->trajectory_edge->list_states.begin();
	 it_state != edge_found->trajectory_edge->list_states.end(); it_state++) {
      
      state_t *state_curr = *it_state;
      
      trajectory_out.list_states.push_back (state_curr);
    }

    
    trajectory_out.list_states.push_back (vertex_curr->state);

    vertex_prev = vertex_curr;
	
  }
  

  return 1;
}


#endif
