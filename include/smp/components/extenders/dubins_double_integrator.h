/*! \file components/extenders/dubins_double_integrator.h
  \brief The combined dubins car and double integrator system components. 
  State, input, and extender definitions.
  
  This file implements the state, input, and extender classes for a 
  the combined dubins car and double integrator integrator system, which 
  constitutes a basic model of an airplane moving like a Dubins car on the
  plane governed with double integrator dynamics for its altitude
*/

#ifndef _SMP_SYSTEM_DUBINS_DOUBLE_INTEGRATOR_H_
#define _SMP_SYSTEM_DUBINS_DOUBLE_INTEGRATOR_H_


#include <smp/components/extenders/state_array_double.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/base.h>


#include <list>


using namespace std;


namespace smp {


    //! Implementation of the state data structure for the dubins double integrator airplane dynamics
    /*!
      This class implements the state data structure for the double integrator dynamics. 
      The number of state variables is twice number of dimensions, since for each dimension
      both position and velocity has to be stored. The positions are stored in the usual order
      first, and then the all velocities are stored in their usual order, in the array.
    */
    class state_dubins_double_integrator : public state_array_double<5> {

    };



    //! Implementation of the input data structure for the dubins double integrator airplane dynamics
    /*!
      This class implements the input data structure for the double integrator dynamics. 
      The number of input variables is one plus the dimensions. The extra input variable,
      placed in the beginning of the array, is used to store the time it takes to 
      execute the trajectory segment. 
    */
    class input_dubins_double_integrator : public input_array_double<3> {
    
    };



    //! Extender function with dubins double integrator airplane dynamics.
    /*!
      This class implements an extender with a combined dubins car and double integrator dynamics. 
            
      \ingroup extenders
    */
    template < class typeparams >
    class extender_dubins_double_integrator : public extender_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory< typeparams > trajectory_t;

    

        // // TODO: get back to appropriate velocity constraints.


        double extend_dubins_spheres_return_params (double x_s1, double y_s1, double t_s1, 
                                                    double x_s2, double y_s2, double t_s2, int comb_no, 
                                                    double *t_increment_s1_out, 
                                                    double *s_increment_out, 
                                                    double *t_increment_s2_out);

        
        double extend_with_time_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max, 
                                                          int *direction, int *traj_saturated,
                                                          double *x_intersect_beg, 
                                                          double *x_intersect_end,
                                                          double *v_intersect);


        int extend_with_effort_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max, 
                                                         double t_min, double t_goal, double t_eps,
                                                         int *dir, int *traj_saturated, double *max_control,
                                                         double *x_intersect_beg, double *x_intersect_end,
                                                         double *v_intersect);


        int extend_dubins_di (state_t* state_ini, state_t* state_fin, 
                              int* fully_extends,  list<state_t*> *list_states_out, list<input_t*> *list_inputs_out);



    public :

        extender_dubins_double_integrator ();
        ~extender_dubins_double_integrator ();

    
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
