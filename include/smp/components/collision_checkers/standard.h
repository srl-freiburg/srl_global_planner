/*! \file components/collision_checkers/standard.h
  \brief The standard brute-force collision checker
  
  This file implements the standard collision checker class. The region
  class, which is used to describe rectangular obstacles in the Euclidean
  space is defined in region.h
*/
#ifndef _SMP_COLLISION_CHECKER_STANDARD_H_
#define _SMP_COLLISION_CHECKER_STANDARD_H_

#include <smp/components/collision_checkers/base.h>

#include <smp/common/region.h>

#include <list>


namespace smp {

    //! Standard collision checker
    /*!
      This class implements the standard collision checker. Standard collision
      checking procedure discretizes the trajectories connecting consecutive 
      states. The said trajectory is obtained by a linear interpolation between 
      the said states. Each interpolated state is, then, checked for collisioon
      with all the obstacles. This procedure is continued for all the states in 
      the trajectory. A single states is checked for collision by merely going
      through the list of obstacles to check whether the query state resides
      inside any of the obstacles.
      
      \ingroup collision_checkers
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class collision_checker_standard : public collision_checker_base<typeparams> {
        
        
        
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

        list< region_t* > list_obstacles;



    public:
        collision_checker_standard ();
        ~collision_checker_standard ();

	float size_robot;
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
         * \brief Adds a new obstacle to the list of obstacles.
         * 
         * This function adds a new obstacle to the list of obstacle, which 
         * must be a type of region<NUM_DIMENSIONS>. Note that the 
         * NUM_DIMENSIONS template argument of the region and this class 
         * must match. Otherwise, compuilation errors will occur.
         *
         * @param obstacle_in The pointer to the new obstacle
         * 
         * @returns Returns 1 for success, a non-positive value to indicate error.
         */
        int add_obstacle (region_t &obstacle_in);
	
	 /**
	  * Added to clean the list_obstacles
	  * @param a stupid number
          * 
          * @returns Returns 1 for success, a non-positive value to indicate error.
	  * 
	  */
	int clean_obs(int a);

    };


}

#endif
