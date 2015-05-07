/*! \file components/extenders/pos.h
  \brief The extend function component that implements a position control as described by Alessandro Astolfi paper Exponential Stabilization of a Wheeled Mobile Robot Via Discontinuous Control.
  
 the dimensionality of the state space is a template parameter
*/



// #include "/media/data/software/RRTstar/pedsimRRTstar/demoapp/src/smp/components/extenders/state_array_double.h"
// #include "/media/data/software/RRTstar/pedsimRRTstar/demoapp/src/smp/components/extenders/input_array_double.h"
// #include "/media/data/software/RRTstar/pedsimRRTstar/demoapp/src/smp/components/extenders/base.h"


#include <smp/components/extenders/state_array_double.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/base.h>
#include <list>

using namespace std;


namespace smp {

//! Implementation of the state data structure for the Position control described by Alessandro Astolfi in the paper
//!  Exponential Stabilization of a Wheeled Mobile Robot Via Discontinuous Control
/*!
      This class implements the state data structure.
      The number of state variables is three. The state variables indicate position
      in the x and y coordinates and the orientation, in this order.
      
      \ingroup states
    */
class state_pos : public state_array_double <3> {
    
};



//! Implementation of the input data structure
/*!

      The number of input variables is exactly two. The first input variable
      stores the Translational Velocity t, while the second variable stores the Steering input required.
      
      \ingroup inputs
    */
class input_pos : public input_array_double<2> {


};


//! s.
/*!


      \ingroup extenders
    */
template< class typeparams,  int NUM_DIMENSIONS>
class extender_pos : public extender_base<typeparams> {



    typedef typename typeparams::state state_t;
    typedef typename typeparams::input input_t;
    typedef typename typeparams::vertex_data vertex_data_t;
    typedef typename typeparams::edge_data edge_data_t;

    typedef vertex<typeparams> vertex_t;
    typedef edge<typeparams> edge_t;

    typedef trajectory<typeparams> trajectory_t;


    /** This function will generate a vector of double as output:

     *  [0] Vl velocity of the left wheel;
     *  [1] Vr velocity of the right wheel;
     *  [2] V translational Velocity;
     *  [3] W Angular Velocity.
     *  [4] EOT End Of Trajectory

    **/


    double set_angle_to_range(double alpha, double min);
    
    double diff_angle_unwrap(double alpha1, double alpha2);

    double f(double rho);

    double g(double t);




    double* posctrlstep (double x_c, double y_c, double t_c,
                         double x_end, double y_end, double t_end, double ct, double b, int dir);
    
    double posctrl(state_t *state_ini, state_t *state_fin,int dir,double b, double dt ,
                   list<state_t *> *list_states_out, list<input_t *> *list_inputs_out);

    double normangle(double a, double mina);


public :

    extender_pos ();
    ~extender_pos ();
    

    int ex_update_insert_vertex (vertex_t *vertex_in);
    

    int ex_update_insert_edge (edge_t *edge_in);


    int ex_update_delete_vertex (vertex_t *vertex_in);
    
    
    int ex_update_delete_edge (edge_t *edge_in);


    /**
     * Generates a trajectory, returned in the trajectory_out argument, that connects two
         * given states, provided with the state_from_in and state_towards_in arguments. If
         * the connection is exact, i.e., the trajectory reaches state_towards_in exactly,
         * then the output variable exact_connection_out is set to one. If, on the other hand,
         * the connection is approximate, then the same variable is set to zero.
         *
         * @param state_from_in The state that the new trajectory starts from.
         * @param state_towards_in The state that the new trajectory is shooted towards.
         * @param exact_connection_out Set to one if the connection is exact, otherwise
         *                             this variable is set to zero by this function.
         * @param trajectory_out The output variable that contians the resulting trajectory.
         * @param intermediate_vertices_out The list of states that will be individual vertices.
         *
         * @returns Returns 1 for success, a non-positive number to indicate error.
         */
    int extend (state_t *state_from_in, state_t *state_towards_in,
                int *exact_connection_out, trajectory_t *trajectory_out,
                list<state_t*> *intermediate_vertices_out);
    
    /// Result for each iteration in posctrlstep
    double *result;
    /// Saving the posctrlstep results in this during posctrl procedure
    double *intRes;

    double T;
    
    double xi,yi;

    double rho_end_condit;
};


}

