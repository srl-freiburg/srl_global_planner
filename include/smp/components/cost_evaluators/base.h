/*! \file components/cost_evaluators/base.h
  \brief The abstract cost evaluator
  
  This file provides an implementation of the abstract class for the
  generic cost evaluator.
*/

#ifndef _SMP_COST_EVALUATOR_BASE_H_
#define _SMP_COST_EVALUATOR_BASE_H_

#include <smp/planner_utils/trajectory.h>
#include <smp/planner_utils/vertex_edge.h>


namespace smp {

    //! The abstract class that specifies the structure of the cost evalutor component.
    /*!
      This class implements the abstract cost evaluator class, which provides one main
      method that returns the cost of a trajectory starting from a given initial state
      a reaching a given final vertex.
      
      \ingroup cost_evaluators
    */
    template< class typeparams >
    class cost_evaluator_base {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory<typeparams> trajectory_t;

    public:

        /**
         * \brief Choose how the distance metric can be computed
         *
         * SELECT_FUNC == 0. True Distance Metric
         * SELECT_FUNC == 1. regression_nlm
         * SELECT_FUNC == 2. regression_nn
         */

        int SELECT_FUNC;

        /**
         * \brief Update function for vertex cost modification
         *
         * This function is called by the planner whenever a the cost associated
         * with a vertex is changed by the optimizing (incremental) planning algorithm.
         *
         * @param vertex_in A pointer to the vertex with modified cost.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int ce_update_vertex_cost (vertex_t *vertex_in) = 0;

    
        /**
         * \brief Update function for edge cost modification
         *
         * This function is called by the planner whenever the cost associated
         * with an edge is changed by the optimizing (incremental) planning algorithm.
         *
         * @param edge_in A pointer to the edge with modified cost.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int ce_update_edge_cost (edge_t *edge_in) = 0;
    
    
        /**
         * \brief Evaluates the cost of a trajectory.
         *
         * This function returns the cost of a given trajectory that starts from 
         * state_initial_in and reaches state_final_in. Sometimes the final state
         * is embedded in the trajectory itself, in which case state_final_in argument
         * can be set to NULL.
         *
         * @param state_initial_in Initial state that the trajectory starts from
         * @param trajectory_in Trajectory
         * @param state_final_in Final state that the trajectory reaches 
         *
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        virtual double evaluate_cost_trajectory (state_t *state_initial_in,
                                                 trajectory_t *trajectory_in,
                                                 state_t *state_final_in = 0) = 0;
    
    };


}

#endif
