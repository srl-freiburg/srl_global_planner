/*! \file components/multipurpose/minimum_time_reachability.h
  \brief An implementation of the vertex and edge components in the graph.

  Provides an implementation of the vertex and edge components in the graph. Both classes
  are defined as templates that take the types of the state, input, and the data stored in
  the vertices as well as the type of the data that is stored in the edges as an argument.
*/


#ifndef _SMP_MINIMUM_TIME_REACHABILITY_H_
#define _SMP_MINIMUM_TIME_REACHABILITY_H_

#include <smp/planners/rrtstar.h>
#include <smp/common/region.h>
#include <smp/components/model_checkers/base.h>
#include <smp/components/cost_evaluators/base.h>



namespace smp {


    //! Vertex data for minimum-time reachability.
    /*!
      This data structure is attached to each vertex in the graph maintained by the planner
      algorithm. The data structure includes two variables. One variable indicates whether the associated
      vertex lies inside the goal region. Another variables keeps track of the cost to reach
      this particular vertex starting from the root vertex. The latter variable is particularly
      created to work with teh RRT* algorithm.
    */
    class minimum_time_reachability_vertex_data : public rrtstar_vertex_data {

    public:

        //! Reachability of the goal region.
        /*!
          This variable that indicates whether the associated vertex
          state is inside the goal region.
        */
        bool reaches_goal;
    };



    //! Edge data for minimum-time reachability.
    /*!
      This empty class is implemented for the sake of completeness.
    */
    class minimum_time_reachability_edge_data : public rrtstar_edge_data {

    };



    //! A combination of the minimum-time cost evaluator and the reachability model checker
    /*!
      Combining the minimum-time cost evaluator and the reachability model checker, this class
      is able to keep track of the minimum-time that reaches the goal region. The class constitutes
      a good example of multiple-purpose algorithm component made possible with mutliple inheritance.

      \ingroup model_checkers
      \ingroup cost_evaluators
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class minimum_time_reachability : public model_checker_base<typeparams> , public cost_evaluator_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef trajectory<typeparams> trajectory_t;

        typedef region<NUM_DIMENSIONS> region_t;

        typedef int (*update_func_t)(trajectory_t *);


        list<update_func_t> list_update_functions;  // A list of functions that will be called in the
                                                    //    event of updating the minimum cost trajectory.


        vertex_t *min_cost_vertex;         // A pointer to the minimum cost vertex in the tree
        trajectory_t min_cost_trajectory;  // A copy of the mininum cost trajectory


        region_t region_goal;


    public:

        minimum_time_reachability ();
        ~minimum_time_reachability ();

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
        minimum_time_reachability (const region_t &region_goal);

        /**
         * \brief Modifies the goal region.
         *
         * This function sets the goal region to its new value given
         * as an argument.
         *
         * @param region_goal New goal region.
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int set_goal_region (const region_t &region_goal);

        int ce_update_vertex_cost (vertex_t *vertex_in);

        int ce_update_edge_cost (edge_t *edge_in);

        int mc_update_insert_vertex (vertex_t *vertex_in);

        int mc_update_insert_edge (edge_t *edge_in);

        int mc_update_delete_vertex (vertex_t *vertex_in);

        int mc_update_delete_edge (edge_t *edge_in);

        int get_solution (trajectory_t &trajectory_out);

        double evaluate_cost_trajectory (state_t *state_initial_in,
                                         trajectory_t *trajectory_in,
                                         state_t *state_final_in = 0);

        double set_angle_to_range(double alpha, double min);

        double diff_angle_unwrap(double alpha1, double alpha2);

        double regression_nlm(double xin,double yin,double thin,double xout,double yout,double thout);

        int update_trajectory (trajectory_t *trajectory_in);

        double cost_proj (double x, double y, double theta);

        /**
         * \brief Returns the cost of the best trajectory.
         *
         *  This function returns the cost of the minimum cost trajectory that reaches the goal,
         *  if such a trajectory exists. Otherwise, it returns -1.0.
         *
         * @returns Returns the cost of the minimum cost trajectory, or -1.0 to indicate no such trajectory exists.
         */
        double get_best_cost ();

        /**
         * \brief Clears the update function list for minimum cost update.
         *
         * Whenever an optimizing motion planning algorithm using this component finds
         * a better trajectory, this component calls a list of functions that are registered
         * for this call back. This method clears this list of functions.
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int clear_update_function_list ();

        /**
         * \brief Clears the update function list for minimum cost update.
         *
         * Whenever an optimizing motion planning algorithm using this component finds
         * a better trajectory, this component calls a list of functions that are registered
         * for this call back. This method registers a new update function, i.e., adds
         * the function given in the argument to the appropriate list.
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int register_new_update_function (update_func_t update_function);


        /**
        * Initializing the world model and the footprint_spec.
        * @param world_model,
        * @param footprint_spec, footprint of the robot
        * @param  inscribed_radius The radius of the inscribed circle of the robot
        * @param  circumscribed_radius The radius of the circumscribed circle of the robot
        * @returns Returns 1 for success, a non-positive value to indicate error.
        *
        */
        int initWorldModel(base_local_planner::CostmapModel* world_model,std::vector<geometry_msgs::Point> footprint_spec, double inscribed_radius, double circumscribed_radius, costmap_2d::Costmap2DROS* costmap_ros, std::string planner_frame);


        /**
        * Get the cost from the global cost map
        * @param x, x coordinate of the pose
        * @param y, y coordinate of the pose
        * @param theta_i, theta coordinate of the pose
        * @returns Returns the cost associated to the pose
        *
        */

        int getCostFromMap(double x_i, double y_i, double theta_i);

        clock_t begin_time;

        float end_time;

        int cntForClock;

        bool foundTraj ;

        double cost;

        trajectory_t path_support;

        double Kd;

        double Kdist;

        double Kth;

        double Kor;

        double Kangle;

        int n_dis_traj;

        int ADD_COST_THETASTAR;

        int ADD_COST_PATHLENGTH;

        int NOTLEARNED;

        int MODEL_COST;

        int ONLYTHETACOST;

        Eigen::MatrixXd path;

        Eigen::MatrixXd distances;

        double tot_dist;

        int cntUpdates;

        std::string file_name;

        bool ADD_COST_FROM_COSTMAP; ///<  @brief Choose if to add cost from costmap

        double inscribed_radius_; ///<  @brief The radius of the inscribed circle of the robot

        double circumscribed_radius_; ///<  @brief The radius of the circumscribed circle of the robot

        base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

        std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

        costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

        std::string global_frame_; ///< @brief Global Frame of the CostMap

        tf::TransformListener *CostEvaluatorlistener; ///< @brief Trasform Listener used to apply a trasform between Odom and global cost map frame

        tf::StampedTransform transform_;  ///< @brief Trasform between Global Frame and Odom



    };


}

#endif
