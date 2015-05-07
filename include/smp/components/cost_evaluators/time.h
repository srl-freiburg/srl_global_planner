/*! \file components/cost_evaluators/time.h
  \brief The cost evaluator based on the execution time
  
  This file implements the smp_cost_evaluator_time class that computes the
  cost of the trajectory based on the time it takes to execute it.
*/

#ifndef _SMP_COST_EVALUATOR_TIME_H_
#define _SMP_COST_EVALUATOR_TIME_H_

#include <smp/components/cost_evaluators/base.h>


namespace smp {

    //! The cost evaluator class based on the trajectory execution time
    /*!
      This class computes the cost of a trajectory according to the time
      it takes to execute that particular trajectory.
      
      \ingroup cost_evaluators
    */
    template< class typeparams >
    class cost_evaluator_time : public cost_evaluator_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory<typeparams> trajectory_t;


    public:

        int ce_update_vertex_cost (vertex_t *vertex_in);

        int ce_update_edge_cost (edge_t *edge_in);
    
        double evaluate_cost_trajectory (state_t *state_initial_in,
                                         trajectory_t *trajectory_in,
                                         state_t *state_final_in = 0);
    
    };


}

#endif
