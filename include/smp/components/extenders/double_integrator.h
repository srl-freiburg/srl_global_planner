/*! \file components/extenders/double_integrator.h
  \brief The double integrator system components. State, input, and extender definitions.
  
  This file implements the state, input, and extender classes for a 
  d-dimensional double integrator system, where d is a template parameter
  when appropriate. Currently, the implementation supports only d = 2.
  The author is working on the case when d > 2.
*/

#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_H_


#include <smp/components/extenders/state_array_double.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/base.h>


#include <list>

using namespace std;

namespace smp {


    //! Implementation of the state data structure for the double integrator dynamics
    /*!
      This class implements the state data structure for the double integrator dynamics. 
      The number of state variables is twice number of dimensions, since for each dimension
      both position and velocity has to be stored. The positions are stored in the usual order
      first, and then the all velocities are stored in their usual order, in the array.
    */
    template< int NUM_DIMENSIONS > 
    class state_double_integrator : public state_array_double<NUM_DIMENSIONS*2> {

    };



    //! Implementation of the input data structure for the double integrator dynamics
    /*!
      This class implements the input data structure for the double integrator dynamics. 
      The number of input variables is one plus the dimensions. The extra input variable,
      placed in the beginning of the array, is used to store the time it takes to 
      execute the trajectory segment. 
    */
    template< int NUM_DIMENSIONS >
    class input_double_integrator : public input_array_double<NUM_DIMENSIONS+1> {
    
    };



    //! Extender function with double integrator dynamics.
    /*!
      This class implements an extender with double integrator dynamics. It is intended
      that the number of dimensions of the state space is a template argument for the class.
      However, this feature is not implemented yet. Currently, the double integrator 
      system works only in two dimensions (NUM_DIMENSIONS = 2), i.e., two positions 
      and their two velocities.
      
      \ingroup extenders
    */
    template < class typeparams, int NUM_DIMENSIONS >
    class extender_double_integrator : public extender_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory< typeparams > trajectory_t;

    

        // // TODO: get back to appropriate velocity constraints.
        // double velocity_constraint_min[NUM_DIMENSIONS];
        // double velocity_constraint_max[NUM_DIMENSIONS];

       
    
        int extend_with_optimal_control (state_t *state_ini, state_t *state_fin, 
                                         list<state_t*> *list_states_out, list<input_t*> *list_inputs_out);


    public :

        extender_double_integrator ();
        ~extender_double_integrator ();

    
        int ex_update_insert_vertex (vertex_t *vertex_in) {return 1;}
    

        int ex_update_insert_edge (edge_t *edge_in)  {return 1;}


        int ex_update_delete_vertex (vertex_t *vertex_in) {return 1;}
    
    
        int ex_update_delete_edge (edge_t *edge_in) {return 1;}


    
        int extend (state_t *state_from_in, state_t *state_towards_in,
                    int *exact_connection_out, trajectory_t *trajectory_out,
                    list<state_t*> *intermediate_vertices_out);
    
    };


}

#endif
