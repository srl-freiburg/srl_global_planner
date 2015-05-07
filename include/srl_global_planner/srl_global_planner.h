#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP





#include <cstdlib>
#include <fstream>
#include <iostream>
#include <srl_global_planner/srl_trajectory.h>

// Grid planner
#include <srl_global_planner/thetastar_leading_rrt.h>


// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

// messages and services
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_control_msgs/CollisionStatus.h>
#include <spencer_nav_msgs/SetDrivingDirection.h>
#include <spencer_nav_msgs/StopDriving.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Header.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Bool.h>


#include <srl_global_planner/world_model.h>
#include <srl_global_planner/costmap_model.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>



/// ==================================================================================
/// Rrt_planner
/// This node solves a motion planning problem given the current robot_state and the
/// goal. It is based on the Sampling-based Motion Planning Library from MIT, where you
/// can use three RRT variants.
/// ==================================================================================


//! SMP HEADER

// #include <smp/components/samplers/uniform.hpp>
//#include <smp/components/samplers/uniformc.hpp>
// #include <smp/components/samplers/gauss.hpp>
#include <smp/components/samplers/theta_star_in_regions.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/pos.hpp>




//#include <srl_global_planner/smp/components/collision_checkers/standard.hpp>
// #include <srl_global_planner/smp/components/collision_checkers/circle.hpp>
#include <smp/components/collision_checkers/collisionCostMap.hpp>
// #include <srl_global_planner/smp/components/collision_checkers/rectangle.hpp>


#include <smp/components/multipurpose/minimum_time_reachability_thetastar.hpp>



#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>

#include <smp/utils/branch_and_bound_euclidean.hpp>

#define HUMANMAX 20



namespace srl_global_planner {



typedef state_pos state_t;
typedef input_pos input_t;

typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;


// Create the typeparams structure
typedef struct _typeparams {
    typedef state_t state;
    typedef input_t input;
    typedef vertex_data_t vertex_data;
    typedef edge_data_t edge_data;
} typeparams;

//// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;





// Define all planner component types
//typedef sampler_opra<typeparams,3> sampler_t;
// typedef sampler_uniform<typeparams,3> sampler_t;
typedef theta_star_in_regions<typeparams,3> sampler_t;
//typedef sampler_uniformc<typeparams,3> sampler_t;
//typedef sampler_gauss<typeparams,3> sampler_t;




typedef distance_evaluator_kdtree<typeparams,3> distance_evaluator_t;

typedef extender_pos<typeparams,3> extender_t;



//typedef collision_checker_standard<typeparams,2> collision_checker_t;
// typedef collision_checker_circle<typeparams,2> collision_checker_t;
typedef collision_checker_costmap<typeparams,2> collision_checker_t;

// typedef collision_checker_rectangle <typeparams,2> collision_checker_t;

typedef minimum_time_reachability<typeparams,2> min_time_reachability_t;
// typedef smoothness_cost<typeparams,2>  smoothness_cost_t;

// Define all algorithm types
typedef rrtstar<typeparams>  rrtstar_t;

// Types to save the tree information
typedef vertex<typeparams> vertex_t;
typedef edge<typeparams> edge_t;


/// Define Branch and Bound Type
typedef branch_and_bound_euclidean<typeparams,2> branch_and_bound_t;





///<  @brief Struct that descrbes an Obstacle point
typedef struct ObstaclePoint {

    double x;
    double y;
    double z;
    double cell_width;
    double cell_height;

    ObstaclePoint() { x = 0.0; y = 0.0; z = 0.0; cell_width=0; cell_height=0;}
    ObstaclePoint(double xx, double yy) : x(xx), y(yy) { z = 0.0; cell_width=0; cell_height=0;}
    ObstaclePoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz)  { cell_width=0; cell_height=0; }
    ObstaclePoint(double xx, double yy, double zz, double cw, double ch) : x(xx), y(yy), z(zz), cell_width(cw), cell_height(ch)  { }
    ObstaclePoint(double* p) { x = p[0]; y = p[1]; z = p[2]; cell_width=p[3]; cell_height=p[4];}

    ObstaclePoint& operator=(ObstaclePoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        cell_width=copy.cell_width;
        cell_height=copy.cell_height;

        return *this;
    }

} Tobstacle;






///<  @brief Struct that descrbes a human point
typedef struct HumanPoint {

    // pose of a human being (agent)
    double x;
    double y;
    double z;
    // agent id and type
    int id;
    int type;

    // cell_width and cell_height used as agent's sizes
    double cell_width;
    double cell_height;

    HumanPoint() { x = 0.0; y = 0.0; z = 0.0; cell_width=0.3; cell_height=0.4; id=0; type=0;}
    HumanPoint(double xx, double yy) : x(xx), y(yy) { z = 0.0; cell_width=0.3; cell_height=0.4; id=0; type=0;}
    HumanPoint(double xx, double yy, double zz, double ii, double tt) : x(xx), y(yy), z(zz), id(ii), type(tt) { cell_width=0.3; cell_height=0.4; }
    HumanPoint(double* p) { x = p[0]; y = p[1]; z = p[2]; id=p[3]; type=p[4]; cell_width=0.3; cell_height=0.4;}

    HumanPoint& operator=(HumanPoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        cell_width=copy.cell_width;
        cell_height=copy.cell_height;
        id=copy.id;
        type=copy.type;

        return *this;
    }

} Thuman;









///<  @brief Srl_global Planner class
class Srl_global_planner : public nav_core::BaseGlobalPlanner
{





private:


    ros::NodeHandle nh_;
    // Publishers
    ros::Publisher pub_path_;    // WARNING: TO PUBLISH A PATH
    ros::Publisher pub_goal_;
    ros::Publisher pub_tree_;
    ros::Publisher pub_tree_dedicated_;
    ros::Publisher pub_path_dedicated_;
    ros::Publisher pub_samples_;
    ros::Publisher pub_graph_;
    ros::Publisher pub_no_plan_;
    ros::Publisher pub_obstacle_markers_;

    // ros::Publisher pub_sensor_range_;
    // subscribers
    ros::Subscriber sub_obstacles_;    // to read the obstacles' positions
    ros::Subscriber sub_all_agents_;   // to read the agents' poses AND   to read the current robot pose
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_daryl_odom_;



public:


    /**
    * @brief Constructor of the Srl_global_planner
    * @param node, Ros NodeHandle
    * @param world_model, Cost Map model to load informaiton of the Global Cost map
    * @param footprint_spec, footprint of the robot
    * @param costmap_ros, cost_map ros wrapper
    */
    Srl_global_planner() : costmap_ros_(NULL), initialized_(false),  world_model_(NULL) {

    }

    /**
    * @brief Constructor of the Srl_global_planner
    * @param name, name to associate to the node
    * @param costmap_ros, costmap_ros
    */
    Srl_global_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(costmap_ros)
    {

        initialize(name, costmap_ros);
    }

    /**
    * @brief Initialize the ros handle
    * @param name, Ros NodeHandle name
    * @param costmap_ros, cost_map ros wrapper
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief makePlan, follows the virtual method of the base class
    * @param start, Start pose
    * @param goal, goal pose
    * @param plan, generated path
    * @return bool, true
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan );

    /**
    * @brief callbackSetGoal, Set goal via the Rviz Topic
    * @return void
    */

    void callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);
    /**
    * @brief callbackObstacles, Read occupancy grid from global cost map
    * @return void
    */
    void callbackObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
    * @brief callbackAllTracks, Read the agents
    * @return void
    */
    void callbackAllTracks(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);

    /**
    * @brief callbackSetRobotPose, Read the robot odom not currently used
    * @return void
    */
    void callbackSetRobotPose(const nav_msgs::Odometry::ConstPtr& msg);

    /**
    * @brief publishPath, Publish path
    * @return void
    */
    void publishPath(Trajectory *t);

    /**
    * @brief publishGoal, Publish Goal
    * @return void
    */
    void publishGoal();

    /**
    * @brief publishSample, Publish sample of RRT
    * @param x, x coordinate of the sample
    * @param y, y coordinate of the sample
    * @param theta, orientation of the sample
    * @param ident, associate id to the sample published
    * @return void
    */
    void publishSample(double x,double y, double theta, int ident);

    /**
    * @brief publishNoPlan, Publish sample a No Plan bool msg
    * @param res, result of the planner
    * @return void
    */
    void publishNoPlan(int res);


    /**
    * @brief setGlobalPathSupport, set the global support to use in the planning phase
    * @return true, if the plan was set
    */
    bool setGlobalPathSupport(std::vector< geometry_msgs::PoseStamped > plan);


    /**
    * @brief plan a kinodynamic path using RRT
    * @param, traj Trajectory generated
    * @param, type of motion planner to use
    * @param, start pose
    * @return true, if the plan was found
    */
    int plan(Trajectory *traj, int type, geometry_msgs::PoseStamped& start);

    /**
    * @brief set the Goal region
    * @param, x coordinate of the goal pose
    * @param, y coordinate of the goal pose
    * @param, theta yaw angle of the goal pose
    * @param, toll yaw angle of the goal pose
    * @param, goal_frame goal frame
    * @return void
    */
    void setGoal(double x, double y, double theta, double toll, std::string goal_frame);


    /**
    * @brief Transform pose in planner_frame
    * @param, init_pose initial pose to transform
    * @return Pose transformed
    */
    geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped init_pose);


    /**
    * @brief Wrap the angle
    * @param, alpha, angle to wrap in the range [min min+2*M_PI]
    * @param, min, beginning of the range
    * @return angle in correct range
    */
    double set_angle_to_range(double alpha, double min);

    /**
    * @brief Saves the current tree
    * @param, list_vertices, vertices and branches of the tree
    * @return 1
    */
    int saveTree(list <vertex_t *> *list_vertices);

    /**
    * @brief Publish the tree
    * @return void
    */
    void publishTree();


    double cellwidth_;  ///<  @brief Cell width

    double cellheight_; ///<  @brief Cell height

    base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

    std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

    costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

    costmap_2d::Costmap2D* costmap_; ///< @brief The ROS wrapper for the costmap the controller will use

    std::string node_name_;  ///<  @brief Node name

    Grid_planner *grid_planner_; ///<  @brief Grid Planner that searches a discrete path

    double goal_x_; ///<  @brief x coordinate of the goal pose

    double goal_y_; ///<  @brief y coordinate of the goal pose

    double goal_theta_; ///<  @brief yaw angle of the goal pose

    double toll_goal_; ///<  @brief toll the goal region


    double rx,ry,rz; ///<  @brief robot coordinates

    int Nit_; ///<  @brief  Number of max iterations


    Trajectory *trajectory_; ///<  @brief Keep the path locally

    /// Size of the scene
    double xscene_; ///<  @brief Width of the scene in meters

    double yscene_; ///<  @brief Height of the scene in meters

    int n_hum_; ///<  @brief number of the humans in the scene (to use when the social cost need to be added)

    double hposes_[HUMANMAX][3]; ///<  @brief  humans in the scene


    std::vector<Tpoint> tree_; ///<  @brief tree and expansions occured

    std::vector<size_t> nexpasions_; ///<  @brief expansions done in the tree

    float numVertices_; ///<  @brief Number of iterations

    float timeIter_; ///<  @brief Time per iteration

    float timeSolution_; ///<  @brief Time to find a solution iteration

    double nrew_;  ///<  @brief Number of rewiring per iteration

    double xsupp_; ///<  @brief Width of the support

    double ysupp_;///<  @brief Width of the support

    std::vector<Tobstacle> obstacle_positions; ///<  @brief Obstacles

    std::vector<Thuman> agents_position; ///<  @brief agents


    int TIMECOUNTER ; ///<  @brief Flag to terminate the iterations after a certain amount of seconds

    int FIRSTSOLUTION ;  ///<  @brief Flag to stop the planner once you found the first solution

    int DEB_RRT; ///<  @brief If you want to show all the debug info

    int BOX; ///<  @brief  One if you want to select the neighbours in a Box which respects the diff-drive kinematics

    double RADIUS; ///<  @brief Radius where to select the nearest vertex in plane RRT

    bool goal_init_; ///<  @brief Flag that indicates if the goal is initialized

    double curr_cost_; ///<  @brief Current cost

    int no_fails;  ///<  @brief Number of fails

    double MAXTIME; ///<  @brief Max time of iterations for planning

    int NUMBER_UPDATE_TRAJ; ///<  @brief Number of times the planner improves the trajectory

    int NOANIM; ///<  @brief Activate Publishing markers

    int DISPLAY_METRICS; ///<  @brief Displaying metrics currently not used

    int first_sol; ///<  @brief First solution flag, not used currently

    int SELECT_FUNC; ///<  @brief To select different cost function in min_time_reachability, currently not used

    int WHATTOSHOW; ///<  @brief Selects what kind of info to display in Rviz

    int BRANCHBOUND; ///<  @brief Selects Branching and Bounding of the tree

    double THRS_BRANCHBOUND;  ///<  @brief Thrs for Branching and Bounding of the tree

    int BRANCHBOUND_RATIO; ///<  @brief Ratio for Branching and Bounding of the tree

    int GOAL_BIASING; ///<  @brief Flag to activate Goal Biasing

    double GOAL_BIASING_THS; ///<  @brief Goal Biasing THRS

    int COST_DISPLAY; ///<  @brief To display cost not used currently

    double RHO; ///<  @brief Stopping distance of the POSQ extend function

    double DT; ///<  @brief Integration time stamp for the POSQ extender

    double L_AXIS; ///<  @brief Axis length of the robot

    int READ_AGENTS; ///<  @brief If consider agents in the planner

    double SCALING_IRL; ///<  @brief To scale IRL currently not used

    geometry_msgs::Pose start_pose_; ///<  @brief Start pose

    geometry_msgs::Pose goal_pose_; ///<  @brief Goal pose

    //
    int PARALLEL; ///<  @brief Parallelize collision checker (only rectangular colllision checker)

    int K; ///<  @brief K nearest obstacles to consider in Collision checkin  (only rectangular colllision checker)

    double RAD_OBST; ///<  @brief Radius of the nearest neighbors to consider in Collision checkin  (only rectangular colllision checker)

    double robot_width_; ///<  @brief robot width

    double robot_length_; ///<  @brief robot length

    double collision_boundary; ///<  @brief enlarge the size of the robot

    std::string cost_file_name; ///<  @brief file where to save te costs

    //
    ros::Time begin; ///<  @brief fMap loading only during the first Nsecs

    double initialization_time; ///<  @brief initialization time for map

    double map_loading_time; ///<  @brief map loading time

    double max_map_loading_time; ///<  @brief max map loading time

    int cnt_map_readings; ///<  @brief counter of map readings

    double agents_size_; ///<  @brief Number of Agents

    double xmin_; ///<  @brief xmin for the grid

    double xmax_; ///<  @brief xmax for the grid

    double ymin_; ///<  @brief ymin for the grid

    double ymax_; ///<  @brief ymax for the grid

    int typepar; ///<  @brief type of the planner to use

    double inscribed_radius_; ///<  @brief inscribed radius for the robot shape

    double circumscribed_radius_ ; ///<  @brief circumscribe radius for the robot shape

    ///
    trajectory_t *support_; ///<  @brief Path to be used to generate the sampler support

    trajectory_t *support_bias_; ///<  @brief Path to be used to generate the sampler support

    int TYPE_SAMPLING;  ///<  @brief choose type of sampling

    int EXT_SIGMAS; ///<  @brief In case of Gaussian sampling load external sigmas

    int NUMBER_RUNS; ///<  @brief How many times to run the planner

    double sigmaxi_; ///<  @brief Sigma along the x axis

    double sigmayi_; ///<  @brief Sigma along the y axis


    double computed_sigma_x_; ///<  @brief Computed Sigma along the x axis

    double computed_sigma_y_; ///<  @brief Computed Sigma along the y axis

    double width_strip_; ///<  @brief Width of the Theta* sampling support


    int LEARNED; ///<  @brief Learned distance metric

    int FINDNEAREST; ///<  @brief find nearest vertex in RRT

    int NOTLEARNED;  ///<  @brief Not Learned distance metric

    int ONLYTHETACOST; ///<  @brief Consider only the Theta* cost

    double Kd; ///<  @brief Gain for Theta* cost

    int ADD_COST_THETASTAR; ///<  @brief Add theta* cost

    int AVERAGING; ///<  @brief Averaging among multiple segments

    double OR_RANGE; ///<  @brief Define orientation ranges for theta* sampling

    int MODEL_COST; ///<  @brief Type of cost to consider in the distance metrics

    double Kround; ///<  @brief Gain in Theta* Distance Metric

    double Kdist; ///<  @brief Gain in Theta* Distance Metric

    double Kth; ///<  @brief Gain in Theta* Distance Metric

    double Kor; ///<  @brief Gain in Theta* Distance Metric

    double Kangle; ///<  @brief Gain in Theta* Distance Metric

    double BIAS_PROB; ///<  @brief Probability of Classic Path Biasing technique

    double DISPERSION; ///<  @brief Dispersion in the Classic Path Biasing technique

    int n_dis_traj; ///<  @brief Number of points where to compute the Distance Metric (Not used at the moment)

    double LMAX; /// Thresold for the trapezoid function in the linear combination for the weights in the  Theta* Sampling

    int SRCPARENT;  ///<  @brief if to check for short cut in RRT connection (not used)

    bool initialized_; ///<  @brief check if the global planner is initialized

    int cnt_make_plan_; ///<  @brief counter of planning calls

    int TYPEPLANNER; ///<  @brief type of the planner to use

    std::string costmap_frame_;  ///<  @brief costmap frame

    std::string planner_frame_; ///<  @brief planner frame

    bool ADD_COST_FROM_COSTMAP; ///<  @brief Choose if to add cost from costmap

    int ADD_COST_PATHLENGTH; ///<  @brief Choose if to add cost associated to path length and changes of heading

    int LEVEL_OBSTACLE_; ///<  @brief Minimum cell cost to have to accept a cell as obstacle

    double GB_ORIENT_; ///<  @brief Size of the Orientation range during goal biasing

    double width_map_;  ///<  @brief Width of the 2D space where to sample

    double height_map_; ///<  @brief Height of the 2D space where to sample

    double center_map_x_; ///<  @brief x coordinate of the center of 2D space where to sample

    double center_map_y_; ///<  @brief y coordinate of the center of 2D space where to sample

    int cnt_no_plan_; ///<  @brief counter of no planning sol




};

}

#endif // Rrt_planner_H
