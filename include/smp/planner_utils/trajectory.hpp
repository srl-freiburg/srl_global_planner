#ifndef _SMP_TRAJECTORY_HPP_
#define _SMP_TRAJECTORY_HPP_

#include <smp/planner_utils/trajectory.h>

template< class typeparams >
smp::trajectory< typeparams >
::trajectory () {

}



template< class typeparams >
smp::trajectory< typeparams >
::~trajectory () {
  
  this->clear_delete ();

}


template< class typeparams >
int smp::trajectory< typeparams >
::clear () {

  // Clear the list of states and the list of inputs.
  list_states.clear();
  list_inputs.clear();

  return 1;
}


template< class typeparams >
int smp::trajectory< typeparams >
::clear_delete () {

  // Free all the memory occupied by the states in the list. 
  for (typename list<state_t*>::iterator iter_state = list_states.begin(); 
       iter_state != list_states.end(); iter_state++) {
    state_t *state_curr = *iter_state;
    delete state_curr;
  }

  // Free all the memory occupied by the inputs in the list.
  for (typename list<input_t*>::iterator iter_input = list_inputs.begin(); 
       iter_input != list_inputs.end(); iter_input++) {
    input_t *input_curr = *iter_input;
    delete input_curr;
  }

  // Clear the list of states and the list of inputs.
  this->clear();
  
  return 1;
}




#endif
