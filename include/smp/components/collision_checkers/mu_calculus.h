/*! \file components/collision_checkers/mu_calculus.h
  \brief Collision checker for the mu-calculus model checker
  
  This file implements the collision checking process for the mu-calculus 
  model checker. The region class, which is used to describe rectangular 
  obstacles in the Euclidean space is defined in region.h
*/
#ifndef _SMP_COLLISION_CHECKER_MU_CALCULUS_H_
#define _SMP_COLLISION_CHECKER_MU_CALCULUS_H_

#include <smp/components/collision_checkers/base.h>
#include <smp/common/region.h>

#include <list>


namespace smp {

    //! mu-calculus collision checker
    /*!
      This class implements a standard collision checker for the mu-calculus
      model checker. The mu-calculus model checer requires each
      trajectory to traverse from one region to another at most twice. This 
      collision checker ensures that this property holds as an invariant.
      
      \ingroup collision_checkers
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class collision_checker_mu_calculus : public collision_checker_base<typeparams> {
        
        
        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;    
        typedef trajectory<typeparams> trajectory_t;

        typedef region<NUM_DIMENSIONS> region_t;

        int num_discretization_steps;
        double discretization_length;

        // 0: no discretization
        // 1: use steps discretization
        // 2: use length discretization 
        int discretization_method;     

        list< region_t* > list_regions;
        
        // returns the index for the region that the given state lies in
        inline int get_region_index (double state_vars[NUM_DIMENSIONS]);


    public:
        collision_checker_mu_calculus ();
        ~collision_checker_mu_calculus ();


        int cc_update_insert_vertex (vertex_t *vertex_in);
    

        int cc_update_insert_edge (edge_t *edge_in);  


        int cc_update_delete_vertex (vertex_t *vertex_in);
    
    
        int cc_update_delete_edge (edge_t *edge_in);


        int check_collision_state (state_t *state_in);


        int check_collision_trajectory (trajectory_t *trajectory_in);
    

        /**
         * \brief Sets the number of discretization steps.
         * 
         * This function can be used to set the number of intermediate states 
         * in the discretization process. In this case, the trajectory between
         * two consecutive states is approximated by a straight line. And this
         * line is discretized in such a way that the line includes 
         * number of states exactly equal to that provided to this function.
         *
         * @param num_discretization_steps_in Number of discretization steps.
         * 
         * @returns Returns 1 for success, a non-positive value to indicate error.
         */
        int set_discretization_steps (int num_discretization_steps_in);


        /**
         * \brief Sets the length for the discretization.
         *
         * This function can be used to set the length of the discretization.
         * In this case, the trajectory between two states is approximated by a line 
         * connecting them, and discretized in such a way that the maximum length
         * of any segment is at most the parameter provided to this function.
         * 
         * @param discretization_length_in Length of the discretization.
         * 
         * @returns Returns 1 for success, a non-positive value to indicate error.
         */
        int set_discretization_length (double discretization_length_in);


        /**
         * \brief Adds a new region to the list of regions.
         * 
         * This function adds a new region to the list of regions. Note that the 
         * NUM_DIMENSIONS template argument of the region and this class 
         * must match. Otherwise, compuilation errors will occur.
         *
         * @param region_in The reference to the new obstacle
         * 
         * @returns Returns 1 for success, a non-positive value to indicate error.
         */
        int add_region (region_t &region_in);

    };


}

#endif
