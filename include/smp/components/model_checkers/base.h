/*! \file components/model_checkers/base.h
  \brief Definition of the model checker 

*/

#ifndef _SMP_MODEL_CHECKER_BASE_H_
#define _SMP_MODEL_CHECKER_BASE_H_

#include <smp/planner_utils/trajectory.h>
#include <smp/planner_utils/vertex_edge.h>


namespace smp {

    //! An abstract model checker
    /*!
      The model checker checks, possibly incrementally, whether the graph
      maintained by the planner algorithm contains a trajectory that 
      solves the problem at hand. In particular, a model checker may check
      whether there exists a trajectory that reaches a goal region. More
      generally, the model checker can evaluate whether the graph includes
      a trajectory that satisfies a temporal logic specification.
      
      \ingroup model_checkers_base
    */
    template< class typeparams >
    class model_checker_base {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
        
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef trajectory<typeparams> trajectory_t;

    
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
        virtual int mc_update_insert_vertex (vertex_t *vertex_in) = 0;
    

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
        virtual int mc_update_insert_edge (edge_t *edge_in) = 0;    


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
        virtual int mc_update_delete_vertex (vertex_t *vertex_in) = 0;
    

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
        virtual int mc_update_delete_edge (edge_t *edge_in) = 0;    

    
        /**
         * \brief Returns a trajectory, if one exists, that solves the problem.
         *
         * This function can be called by the user to get a trajectory that solves
         * the particular problem at hand. If the problem involves reaching a goal
         * set, then this function will return a trajectory that does so, if such a
         * trajectory currently exists in the graph.
         *
         * @param trajectory_out The trajectory output by the function. Set to an
         *                       empty trajectory if no trajectory that solves 
         *                       the problem is present in the graph.
         *
         *
         * @returns Returns 1 for success, a non-positive value to indiacate error.
         */
        virtual int get_solution (trajectory_t &trajectory_out) = 0;
        
    };


}

#endif
