/*! \file branch_and_bound_euclidean.h
  \brief Branch and bound with the Euclidean distance heuristic
*/

#ifndef _SMP_BRANCH_AND_BOUND_EUCLIDEAN_H_
#define _SMP_BRANCH_AND_BOUND_EUCLIDEAN_H_

#include <smp/utils/branch_and_bound_base.h>

#include <smp/planner_utils/vertex_edge.h>
#include <smp/common/region.h>


namespace smp{

    //! Branch and bound with the Euclidean distance admissible heuristic (not considering obstacles).
    /*!
      Currently, the heuristic is not computing the exact distance. The current heuristic is not even 
      admissible. It should be employed with care.
      
      \ingroup bnb
     */
    template< class typeparams, int NUM_DIMENSIONS >
    class branch_and_bound_euclidean : public branch_and_bound_base<typeparams> {
    

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef region<NUM_DIMENSIONS> region_t;
    
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
    
        typedef planner<typeparams> planner_t;

        region_t region_goal;
        
        vertex_t *root_vertex;
        
        int add_children_to_list (list<vertex_t*> &list_vertices_in, vertex_t *vertex_in);
                
    public :
    
        branch_and_bound_euclidean ();
        ~branch_and_bound_euclidean ();
    

        int run_branch_and_bound ();

        /**
         * \brief Sets the goal region to which the Euclidean distance will be computed.
         *
         * The branch and bound with the Euclidean distance heuristic considers the Euclidean 
         * distance neglegting the obstacles. It is assumed that the goal region is a box, which 
         * can be set or modified using this function.
         *
         * @param region_goal_in New goal region.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_goal_region (region_t region_goal_in);


        /**
         * \brief Sets the root vertex of the planner.
         *
         * This function can be used to provide the heuristic with the knowledge of the root vertex.
         * Since the heuristic is not admissible, it has the danger of deleting the root vertex, which 
         * can be avoided if a pointer to the root vertex is provided to the heuristic using 
         * this function.
         *
         * @param root_vertex_in A pointer to the root vertex of the incremental planner.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_root_vertex (vertex_t *root_vertex_in);
    };

}

#endif
