/*! \file components/collision_checkers/standard.h
  \brief The standard brute-force collision checker
  
  This file implements the standard collision checker class. The region
  class, which is used to describe rectangular obstacles in the Euclidean
  space is defined in region.h
*/
#ifndef _SMP_COLLISION_CHECKER_COSTMAP_H_
#define _SMP_COLLISION_CHECKER_COSTMAP_H_

#include <smp/components/collision_checkers/base.h>

#include <srl_global_planner/costmap_model.h>
#include <geometry_msgs/Point.h>

#include <smp/common/region.h>

#include <list>


namespace smp {

    //! Cost Map Collision Checker
    /*!
     
      
      \ingroup collision_checkers
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class collision_checker_costmap : public collision_checker_base<typeparams> {
        
        
        
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
        collision_checker_costmap();
        ~collision_checker_costmap ();

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
        * @returns Returns 1 for success, a non-positive value to indicate error.
    	* 
    	*/
    	int clean_obs(int a);

        /**
        * Initializing the world model and the footprint_spec.
        * @param world_model, 
        * @param footprint_spec, footprint of the robot
        * @param  inscribed_radius The radius of the inscribed circle of the robot
        * @param  circumscribed_radius The radius of the circumscribed circle of the robot
        * @returns Returns 1 for success, a non-positive value to indicate error.
        * 
        */
        int initialize(base_local_planner::CostmapModel* world_model,std::vector<geometry_msgs::Point> footprint_spec, double inscribed_radius, double circumscribed_radius, costmap_2d::Costmap2DROS* costmap_ros, std::string planner_frame);


        double inscribed_radius_; ///<  @brief The radius of the inscribed circle of the robot

        double circumscribed_radius_; ///<  @brief The radius of the circumscribed circle of the robot

        base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

        std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

        costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
       
        std::string global_frame_; ///< @brief Global Frame of the CostMap

        tf::TransformListener *CollCheckerlistener; ///< @brief Trasform Listener used to apply a trasform between Odom and global cost map frame

        tf::StampedTransform transform_;  ///< @brief Trasform between Global Frame and Odom

        int LEVEL_OBSTACLE_;  ///< @brief Minimum cell cost to have to accept a cell as obstacle



    };


}

#endif
