/*! \file branch_and_bound_base.h
  \brief Branch and bound heuristic for tree-based motion planners

  The branch and bound heuristic takes a sampling-based motion planner,
  an upper bound on the cost of an optimal trajectory, and a function 
  that can compute a lower bound on the cost-to-go (i.e., an admissible
  heuristic). The branch and bound module, then examines each vertex 
  in tree maintained by the planning algorithm and prunes all nodes that 
  can not possibly generate a trajectory with cost better than that of 
  the current best trajectory in the tree.
*/

#ifndef _SMP_BRANCH_AND_BOUND_BASE_H_
#define _SMP_BRANCH_AND_BOUND_BASE_H_

#include <smp/planner_utils/vertex_edge.h>
#include <smp/planners/base.h>


namespace smp{

    //! Branch and bound base class.
    /*!
     An abstract branch and bround heuristic class. All heuristics should inherit from this class.
     
     \ingroup bnb
     */
    template< class typeparams >
    class branch_and_bound_base {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
    
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
    
        typedef planner<typeparams> planner_t;

    protected:

        //! The planner on which the branch and bound heuristic is applied.
        /*!
          The branch and bound heuristic uses this variable to access, e.g., the 
          vertex list of the planner that the heuristic is being applied.
         */ 
        planner_t *planner_bnb;

        //! An upper bound on the cost of an optimal solution.
        /*!
          This variable can be used by any derived class to access the upper bound 
          on the cost of an optimal solution. This variable is usually frequently
          updated by the user using the set_upper_bound_cost function over the course 
          of an incremental planning process.
         */
        double upper_bound_cost;
    
    
    public :
    
        branch_and_bound_base ();
        ~branch_and_bound_base ();
        
        /**
         * \brief Sets the planner to which the branch and bound heuristic will be applied.
         *
         * The branch and bound heuristic works with the vertex list of the planner that it
         * is given as an input. The planner is provided to the branch and bound heuristic 
         * using this function.
         * 
         * @param planner_in New planner
         * 
         * @returns Returns 1 for success, and a non-positive value to indicate failure.
         */
        int set_planner (planner_t *planner_in);


        /**
         * \brief Sets an upper bound for the optimal cost.
         *
         * The branch and bound heuristic requires an upper bound on the cost of an 
         * optimal solution. This function can be used to set and update this upper bound.
         * 
         * @param upper_bound_cost_in upper_bound_cost_in
         * 
         * @returns Returns 1 for success, and a non-positive value to indicate failure.
         */        
        int set_upper_bound_cost (double upper_bound_cost_in);
        
        /**
         * \brief Runs the branch and bound algorithm
         *
         * Once this function is called, the branch and bound algorithm goes through 
         * all the vertices in the graph and deletes those that can not lead to (or 
         * unlikely to lead to) an optimal solution.
         * 
         * @returns Returns 1 for success, and a non-positive value to indicate failure.
         */        
        virtual int run_branch_and_bound () = 0;
        
    };
}

#endif
