/*! \file components/extenders/base.h
  \brief The abstract extender
  
  The extender (aka, the extension function) generates that exactly or approximately 
  connects two given states. 
*/
#ifndef _SMP_EXTENDER_BASE_H_
#define _SMP_EXTENDER_BASE_H_


#include <smp/planner_utils/trajectory.h>
#include <smp/planner_utils/vertex_edge.h>

#include <list>

using namespace std;


namespace smp {

    //! The abstract class that specifies the structure of the extender component.
    /*!
      An extender provides the function to generate a trajectory that connects two given 
      states. The extender can also provide a list of designated states, which become 
      vertices of their own when added to the graph maintained by the planning algorithm.
      
      \ingroup extenders_base
    */
    template< class typeparams >
    class extender_base {



        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory< typeparams > trajectory_t;
    

    public:

        /**
         * \brief Update function for vertex insertion
         *
         * This function is called by the planner whenever a new vertex is
         * added to the graph. A pointer to the new vertex is given as an argument.
         *
         * @param vertex_in A pointer to the new vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int ex_update_insert_vertex (vertex_t *vertex_in) = 0;
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever a new edge is
         * added to the graph. A pointer to the new edge is given as an argument.
         *
         * @param edge_in A pointer to the new edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
        virtual int ex_update_insert_edge (edge_t *edge_in) = 0;    

        /**
         * \brief Update function for vertex deletion
         *
         * This function is called by the planner whenever a vertex is deleted 
         * from the graph. A pointer to the vertex is given as an argument.
         *
         * @param vertex_in A pointer to deleted vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int ex_update_delete_vertex (vertex_t *vertex_in) = 0;
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever an edge is delete
         * from the graph. A pointer to the edge is given as an argument.
         *
         * @param edge_in A pointer to deleted edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
        virtual int ex_update_delete_edge (edge_t *edge_in) = 0;    

    
        /**
         * \brief Abstract function that generates a trajectory connecting two given states. 
         *
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
        virtual int extend (state_t *state_from_in, state_t *state_towards_in,
                            int *exact_connection_out, trajectory_t *trajectory_out,
                            list<state_t*> *intermediate_vertices_out) = 0;

        double dt_;
        double rho_endcondition;
        double L_axis;

    };

}

#endif
