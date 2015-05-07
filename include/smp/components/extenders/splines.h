/*! \file components/extenders/splines.h
  \brief The extend function component that implements the \eta^3 splines unit generator
  
 
*/


/// Maximum size of Splines
#define PreNt 10000

#include <smp/components/extenders/state_array_double.h>
#include <smp/components/extenders/input_array_double.h>
#include <smp/components/extenders/base.h>
#include <list>

using namespace std;


namespace smp {

    /*!
      This class implements the state data structure.
      The number of state variables is three. The state variables indicate position
      in the x and y coordinates and the orientation, in this order.
      
      \ingroup states
    */
    class state_splines : public state_array_double <3> {
    
    };



    //! Implementation of the input data structure 
    /*!
      
      The number of input variables is exactly two. The first input variable 
      stores the Translational Velocity t, while the second variable stores the Steering input required.
      
      \ingroup inputs
    */
    class input_splines : public input_array_double<2> {


    };


    //! s.
    /*!
      
      
      \ingroup extenders
    */
    template< class typeparams,  int NUM_DIMENSIONS>
    class extender_splines : public extender_base<typeparams> {



        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory<typeparams> trajectory_t;
	
	
	
	
	
    
	double gen_splines(state_t *state_ini, state_t *state_fin,double va,double wa, double vb ,double wb, double dva, double dwa,
			  double dvb, double dwb,list<state_t *> *list_states_out, list<input_t *> *list_inputs_out);
	
        

    public :

        extender_splines ();
        ~extender_splines ();
    

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


        double v[PreNt];
        double w[PreNt];
        double x[PreNt];
        double y[PreNt];
        double th[PreNt];
        double dx[PreNt];
        double ddx[PreNt];
        double dy[PreNt];
        double ddy[PreNt];
        double u[PreNt];
    };


}

