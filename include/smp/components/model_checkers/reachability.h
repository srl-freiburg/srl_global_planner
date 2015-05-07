/*! \file components/model_checkers/reachability.h
  \brief Definition of the reachability model checker 

  This file implements the vertex and edge data structures as well as the 
  model checker for the model checker based on reachability. The reachability
  model checker is designed to work with incremental planners that maintain a
  necessarily connected graph such that for any vertex in the graph there exists
  at least one trajectory that starts from the root vertex and reaches this
  particular vertex. RRT, RRT*, and RRG planners all satsify this criterion. 
*/


#ifndef _SMP_MODEL_CHECKER_REACHABILITY_H_
#define _SMP_MODEL_CHECKER_REACHABILITY_H_

#include <smp/components/model_checkers/base.h>
#include <smp/common/region.h>


namespace smp {

    //! Vertex data for reachability checking.
    /*!
      This data structure is attached to each vertex in the graph maintained 
      by the planner algorithm. The main component of
      the data structure is a variable that keeps track of whether this vertex
      is inside the goal region.
    */
    class model_checker_reachability_vertex_data {

    public:    
    
        //! Reachability of the goal region.
        /*!
          This variable that indicates whether the associated vertex 
          state is inside the goal region. This data structure is suitable
          when the planner includes a trajectory to reach every vertex in
          the graph. RRT, RRT*, and RRG planners satsify this criterion.
        */
        bool reaches_goal;
    };



    //! Edge data for reachability checking.
    /*!
      This empty class is implemented for the sake of completeness.
    */
    class model_checker_reachability_edge_data {
    
    };



    //! Reachability model checker.
    /*!
      The reachability model checker incrementally checks whether the graph 
      includes a vertex that reaches the goal region. The dimensionality of 
      the state space is provided as a template parameter. This reachability model 
      checker is designed to work with incremental planners that maintain a
      necessarily connected graph such that for any vertex in the graph there exists
      at least one trajectory that starts from the root vertex and reaches this
      particular vertex. RRT, RRT*, and RRG planners all satsify this criterion.
      
      \ingroup model_checkers  
    */
    template < class typeparams, int NUM_DIMENSIONS >
    class model_checker_reachability : public model_checker_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
    
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef trajectory<typeparams> trajectory_t;

        typedef region<NUM_DIMENSIONS> region_t;

        region_t region_goal;
    
    public:

        model_checker_reachability ();    
        ~model_checker_reachability ();

    
        /**
         * \brief Constructor that initializes the goal region.
         *
         * This constructor initializes the goal region. Note that the
         * there is a constructor with no arguments. If initiated that
         * constructor will initialize the goal region to its default 
         * values derived from the region class, which amounts 
         * to a point in the origin. 
         *
         * @param region_goal New goal region. 
         */
        model_checker_reachability (const region_t &region_goal);


        /**
         * \brief Modifies the goal region.
         *
         * This function sets the goal region to its new value given 
         * as an argument. 
         *
         * @param region_goal New goal region. 
         *
         * @returns Return 1 if succcess, and a non-positive value to indicate error.
         */
        int set_goal_region (const region_t &region_goal);


        int mc_update_insert_vertex (vertex_t *vertex_in);
    

        int mc_update_insert_edge (edge_t *edge_in);  


        int mc_update_delete_vertex (vertex_t *vertex_in);
    
    
        int mc_update_delete_edge (edge_t *edge_in);


        int get_solution (trajectory_t &trajectory_out);
        
    };


}

#endif
