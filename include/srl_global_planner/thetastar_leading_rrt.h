#include <srl_global_planner/stl_thetastar.h>
#include <srl_global_planner/grid.h>
#include <srl_global_planner/srl_trajectory.h>
#include <srl_global_planner/costmap_model.h>
#include <srl_global_planner/world_model.h>

// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>



#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Header.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

///< @brief Forward declaration of the GNode class
class GNode;
///< @brief Forward declaration of the Grid class
class Grid;


///< @brief Struct that describes a Obstacle Point
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





///< @brief Grid Planner class
class Grid_planner{


    private:

        Trajectory *trajectory_; ///< @brief Trajectory to store path

        Tpoint start_; ///< @brief Start pose

        double minx_; ///< @brief x coordinate of the left bottom corner of the grid

        double miny_; ///< @brief y coordinate of the left bottom corner of the grid

        double grid_width_; ///< @brief Grid width in meters

        double grid_height_; ///< @brief Grid height in meters

        double cellwidth_; ///< @brief Cell width in meters

        double cellheight_; ///< @brief Cell height in meters

        int run_cnt_; ///< @brief Counters telling how many times the grid planner has been called

        int map_reading_cnt_; ///< @brief Counters telling how many times the map has been read

        int PUBLISH_GRID;  ///< @brief Publishing grid

        int THETASTARON_; ///< @brief Flag to choose which discrete search planner to use

        int MAX_LIMIT_SEARCH_STEPS_; ///< @brief Max number of steps allowed to Search for a path

        Tpoint goal_; ///< @brief Goal pose

        string costmap_frame_; ///< @brief string containing the costmap_frame_

        double Fs_gridplanner_; ///< @brief Frequency of the grid planner node

        tf::TransformListener* gridlistener_;  ///< @brief Tf listener

        int DEB_INFO_; ///< @brief Flag to activate debug info

        int LEVEL_OBSTACLE_; ///< @brief Minimum cell cost to have to accept a cell as obstacle

        double min_cost_p_; // thrs for the min obstacle to consider

        ros::NodeHandle nh_; ///< @brief Local node handle

        ros::Subscriber sub_obstacles_; ///< @brief subscriber to the global grid map

        ros::Publisher pub_discrete_path_; ///< @brief publisher of the dicrete path

        ros::Publisher pub_grid_; ///< @brief Publishing Grid

        ros::Publisher pub_discrete_path_marker_; ///< @brief publisher of the markers associated to the dicrete path

        ros::Rate* grid_planner_loop_rate_; ///< @brief Loop rate of the grid planner ros node

        std::string planner_frame_;

    public:

        /**
        * @brief  Constructor of the Grid planner
        * @param node, Ros NodeHandle
        * @param world_model, Cost Map model to load informaiton of the Global Cost map
        * @param footprint_spec, footprint of the robot
        * @param costmap_ros, cost_map ros wrapper
        */
        Grid_planner(const ros::NodeHandle& node, base_local_planner::CostmapModel* world_model, std::vector<geometry_msgs::Point> footprint_spec, costmap_2d::Costmap2DROS* costmap_ros);

        /**
        * @brief Initiliaze the object
        * @return true if no problem encountered
        */
        bool initialize();

        /**
        * @brief It runs the search over the grid
        * @param plan, the plan generated
        * @return 1 if the search has generated a path, zero otherwise
        */
        int run_grid_planner(std::vector< geometry_msgs::PoseStamped >& plan);


        /**
        * @brief Theta* search
        * @param paths, the discrete path generated by theta*
        * @param st, start node for the search
        * @return 1 if plan was found, 0 otherwise
        */
        int thetastar_search(std::vector<std::vector<GNode> >& paths, GNode st);


        /**
        * @brief A* search
        * @param paths, the discrete path generated by A*
        * @param st, start node for the search
        * @return 1 if plan was found, 0 otherwise
        */
        int astar_search(std::vector<std::vector<GNode> >& paths, GNode st);

        /**
        * @brief  Given a point x,y it will return the point of the corresponding cell
        * @param x, x coordinate of the point of interest
        * @param y, y coordinate of the point of interest
        * @return Tpoint containing the node of the grid where the point (x,y) lies
        */
        Tpoint getEnclosingCell(double x, double y);

        /**
        * @brief Initialize the grid
        * @param ax, x coordinate where the map starts
        * @param ay, y coordinate where the map starts
        * @param aw, grid width in meters
        * @param ay, grid height in meters
        * @param cwidth, cell width
        * @param cheight, cell height
        * @return void
        */
        void initGrid(double ax, double ay, double aw, double ah, double cwidth, double cheight);

        /**
        * @brief Reading the Occupangy grid associated to the Global Cost Map
        * @param msg, ROS OccupancyGrid
        * @return void
        */
        void callbackreadingObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg);

       /**
        * @brief  Constructor of the Grid planner
        * @param path, path to be published
        * @param replan, replan, not used at the moment
        * @return void
        */
        void publishGlobalPath(std::vector<GNode>& path, bool replan);


        /**
        * @brief setGoal sets the goal trasform it in the costmap frame
        * @param x, x coordinate of the goal
        * @param y, y coordinate of the goal
        * @param theta, yaw angle of the goal
        * @param goal_frame, frame of the received the goal
        * @return void
        */
        void setGoal(double x,double y, double theta, std::string goal_frame);

        /**
        * @brief setStart sets the start trasform it in the costmap frame
        * @param x, x coordinate of the start
        * @param y, y coordinate of the start
        * @param theta, yaw angle of the start
        * @param goal_frame, frame of the received the start
        * @return void
        */
        void setStart(double x, double y, double theta, std::string start_frame);

        /**
        * @brief getGrid returns the current grid
        * @return Grid
        */
        static Grid* getGrid();


        /// Attributes

        base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

        std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

        costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

        costmap_2d::Costmap2D* costmap_; ///< @brief The ROS wrapper for the costmap the controller will use

        static Grid* grid_; ///< @brief Grid used by the grid planner

        std::vector<Tobstacle> obstacle_positions_; ///< @brief Vector of Obstacles

        std::string map_frame_; ///< @brief Name of the map frame





};










/**
* @brief GNode class used in the search
*/
class GNode
{
public:


    int x, y;
    bool occupied;
    Grid* gd;
    double cellwidth;
    double cellheight;
    double theta;


     GNode()
    {
        x = 0.0;
        y = 0.0;
        gd=Grid_planner::getGrid();
        cellwidth=Grid_planner::getGrid()->cellwidth;
        cellheight=Grid_planner::getGrid()->cellheight;

    }




    GNode(double xx, double yy, Grid* g)

        : x((int)xx), y(int(yy))
    {

        gd = g;
        cellwidth=Grid_planner::getGrid()->cellwidth;
        cellheight=Grid_planner::getGrid()->cellheight;
    }



        GNode(double xx, double yy)
        : x((int)xx), y(int(yy))
   {

        gd=Grid_planner::getGrid();
        cellwidth=Grid_planner::getGrid()->cellwidth;
        cellheight=Grid_planner::getGrid()->cellheight;


   }




    bool operator==(const GNode& n)
    {
        return (x == n.x && y == n.y);
    }




    double costTo(const GNode& n)
    {


        double xs = fabs(n.x*cellwidth - x*cellwidth);
        double ys = fabs(n.y*cellheight - y*cellheight);
        return sqrt(xs*xs + ys*ys);
    }



    // the cell enclosing the node
    Tpoint getCell()
    {
        if (((x-Grid_planner::getGrid()->minx) < 0) || ((y-Grid_planner::getGrid()->miny) < 0))
            std::cerr << "Cell coordinates out of grid! getCell()" << std::endl;

        unsigned int cellx =((x-Grid_planner::getGrid()->minx)/cellwidth);
        unsigned int celly =((y-Grid_planner::getGrid()->miny)/cellheight);

        if ((cellx >= (unsigned int)Grid_planner::getGrid()->width || (celly >= (unsigned int)Grid_planner::getGrid()->height)))
            std::cerr << "Cell coordinates out of grid! getCell()" << std::endl;

        return Tpoint(cellx, celly, 0);
    }

    int GetMap(double x, double y)
    {
        if (Grid_planner::getGrid()->isOccupied(x, y)) return 1;
        else return 0;
    }


    int getMapCost( GNode &successor ){

        int total_cost=0;

        double x0,y0,y1,x1;

        x0=x;

        y0=y;

        x1=successor.x;

        y1=successor.y;


        double t=0;


        ///  go over the line that connects this node to the successor node

        double x_curr, y_curr;

        while(t<1){

            x_curr = (x1-x0)*t+x0;
            y_curr = (x1-x0)*t+x0;
            t=t+0.01;
            total_cost += Grid_planner::getGrid()->getCost(x_curr, y_curr);

        }


        return total_cost;


    }


    bool IsSameState( GNode &rhs )
    {

        // same state in a maze search is simply when (x,y) are the same
        if( (x == rhs.x) &&
                (y == rhs.y) )
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    void PrintNodeInfo()
    {
        char str[100];
        sprintf( str, "Node position : (%d,%d)\n", x,y );
        cout << str;
    }

    // Here's the heuristic function that estimates the distance from a Node
    // to the Goal.

    float GoalDistanceEstimate( GNode &nodeGoal )
    {
        float xd = float( ( (float)x - (float)nodeGoal.x ) );
        float yd = float( ( (float)y - (float)nodeGoal.y) );


        return sqrt(xd*xd+yd*yd);

    }

    bool IsGoal( GNode &nodeGoal )
    {

        if( (x == nodeGoal.x) &&
                (y == nodeGoal.y) )
        {
            return true;
        }

        return false;
    }

    // This generates the successors to the given Node. It uses a helper function called
    // AddSuccessor to give the successors to the Thetastar class. The Theta* specific initialisation
    // is done for each node internally, so here you just set the state information that
    // is specific to the application
    bool GetSuccessorsThetaStar( ThetaStarSearch<GNode> *thetastarsearch, GNode *parent_node )
    {

        int parent_x = -1;
        int parent_y = -1;

        if( parent_node )
        {
            parent_x = parent_node->x;
            parent_y = parent_node->y;
        }

        double unit;
        //mod
        unit =1;
        GNode NewNode;


        GNode *th1= new GNode(x,y);
        GNode *pr1= new GNode( x-unit, y );
        if(lineofsight(th1,pr1) && !((parent_x == x-unit) && (parent_y == y)))
        {
            NewNode = GNode( x-unit, y );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th1);delete(pr1);


        GNode *th2= new GNode(x,y);
        GNode *pr2= new GNode( x, y-unit);
        if(lineofsight(th2,pr2) && !((parent_x == x) && (parent_y == y-unit)))
        {

            NewNode = GNode( x, y-unit );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th2);delete(pr2);


        GNode *th3= new GNode(x,y);
        GNode *pr3= new GNode( (x+unit), y);
        if(lineofsight(th3,pr3) && !((parent_x == x+unit) && (parent_y == y)))
        {

            NewNode = GNode( x+unit, y );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th3);delete(pr3);




        GNode *th4= new GNode(x,y);
        GNode *pr4= new GNode(  x+unit, y+unit);
        if(lineofsight(th4,pr4) && !((parent_x == x+unit) && (parent_y == y+unit)))
        {

            NewNode = GNode( x+unit, y+unit );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th4);delete(pr4);


        GNode *th5= new GNode(x,y);
        GNode *pr5= new GNode( x-unit, y-unit);
        if(lineofsight(th5,pr5)  && !((parent_x == x-unit) && (parent_y == y-unit)))
        {

            NewNode = GNode( x-unit, y-unit );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th5);delete(pr5);


        GNode  *th6= new GNode(x,y);
        GNode *pr6= new GNode(x+unit, y-unit);
        if(lineofsight(th6,pr6) && !((parent_x == x+unit) && (parent_y == y-unit)))
        {

            NewNode = GNode( x+unit, y-unit );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th6);delete(pr6);



        GNode *th7= new GNode(x,y);
        GNode *pr7= new GNode( x-unit, y+unit);
        if(lineofsight(th7,pr7) && !((parent_x == x-unit) && (parent_y == y+unit)))
        {


            NewNode = GNode( x-unit, y+unit );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th7);delete(pr7);



        GNode *th8= new GNode(x,y);
        GNode *pr8= new GNode( x, (y+unit) );
        if(lineofsight(th8,pr8) && !((parent_x == x) && (parent_y == y+unit)))
        {


            NewNode = GNode( x, (y+unit) );
            thetastarsearch->AddSuccessor( NewNode );
        }
        delete(th8);delete(pr8);





        return true;
    }


    // This generates the successors to the given Node. It uses a helper function called
    // AddSuccessor to give the successors to the Thetastar class. The Theta* specific initialisation
    // is done for each node internally, so here you just set the state information that
    // is specific to the application
    bool GetSuccessors( ThetaStarSearch<GNode> *thetastarsearch, GNode *parent_node )
    {



        int parent_x = -1;
        int parent_y = -1;

        if( parent_node )
        {
            parent_x = parent_node->x;
            parent_y = parent_node->y;
        }

        double unit;
        //mod
        unit =1;
        GNode NewNode;


        if(!thetastarsearch->THETASTARON){



                if( !isblock(x-unit,y) && !((parent_x == x-unit) && (parent_y == y)))
                {
                    NewNode = GNode( x-unit, y );
                    thetastarsearch->AddSuccessor( NewNode );
                }


                if( !isblock(x,y-unit) && !((parent_x == x) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }




                if( !isblock(x+unit,y) && !((parent_x == x+unit) && (parent_y == y)))
                {

                    NewNode = GNode( x+unit, y );
                    thetastarsearch->AddSuccessor( NewNode );
                }





                if( !isblock(x+unit,y+unit) && !((parent_x == x+unit) && (parent_y == y+unit)))
                {

                    NewNode = GNode( x+unit, y+unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }



                if( !isblock(x-unit,y-unit)  && !((parent_x == x-unit) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x-unit, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }



                if( !isblock(x+unit,y-unit) && !((parent_x == x+unit) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x+unit, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }




                if( !isblock(x-unit,y+unit) && !((parent_x == x-unit) && (parent_y == y+unit)))
                {


                    NewNode = GNode( x-unit, y+unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }




                if( !isblock(x,y+unit) && !((parent_x == x) && (parent_y == y+unit)))
                {


                    NewNode = GNode( x, (y+unit) );
                    thetastarsearch->AddSuccessor( NewNode );
                }

        }else{

                GNode *th1= new GNode(x,y);
                GNode *pr1= new GNode( x-unit, y );
                if(lineofsight(th1,pr1) && !((parent_x == x-unit) && (parent_y == y)))
                {
                    NewNode = GNode( x-unit, y );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th1);delete(pr1);


                GNode *th2= new GNode(x,y);
                GNode *pr2= new GNode( x, y-unit);
                if(lineofsight(th2,pr2) && !((parent_x == x) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th2);delete(pr2);


                GNode *th3= new GNode(x,y);
                GNode *pr3= new GNode( (x+unit), y);
                if(lineofsight(th3,pr3) && !((parent_x == x+unit) && (parent_y == y)))
                {

                    NewNode = GNode( x+unit, y );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th3);delete(pr3);




                GNode *th4= new GNode(x,y);
                GNode *pr4= new GNode(  x+unit, y+unit);
                if(lineofsight(th4,pr4) && !((parent_x == x+unit) && (parent_y == y+unit)))
                {

                    NewNode = GNode( x+unit, y+unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th4);delete(pr4);


                GNode *th5= new GNode(x,y);
                GNode *pr5= new GNode( x-unit, y-unit);
                if(lineofsight(th5,pr5)  && !((parent_x == x-unit) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x-unit, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th5);delete(pr5);


                GNode  *th6= new GNode(x,y);
                GNode *pr6= new GNode(x+unit, y-unit);
                if(lineofsight(th6,pr6) && !((parent_x == x+unit) && (parent_y == y-unit)))
                {

                    NewNode = GNode( x+unit, y-unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th6);delete(pr6);



                GNode *th7= new GNode(x,y);
                GNode *pr7= new GNode( x-unit, y+unit);
                if(lineofsight(th7,pr7) && !((parent_x == x-unit) && (parent_y == y+unit)))
                {


                    NewNode = GNode( x-unit, y+unit );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th7);delete(pr7);



                GNode *th8= new GNode(x,y);
                GNode *pr8= new GNode( x, (y+unit) );
                if(lineofsight(th8,pr8) && !((parent_x == x) && (parent_y == y+unit)))
                {


                    NewNode = GNode( x, (y+unit) );
                    thetastarsearch->AddSuccessor( NewNode );
                }
                delete(th8);delete(pr8);



        }



        return true;
    }

    float GetCost( GNode &successor )
    {

        double dx= (successor.x-x);
        double dy= (successor.y-y);

        return sqrt(dx*dx+dy*dy);

    }



    bool lineofsight_br(GNode *successor,GNode *parent_node){

        double x0,y0,y1,x1;
        x0=parent_node->x;
        y0=parent_node->y;
        x1=successor->x;
        y1=successor->y;

        double unit;
        unit=1;
        bool res=true;



        double dx,dy,f,sy,sx;

        dy=(y1-y0);
        dx=(x1-x0);
        f=0;

        if(dy<0){
            dy=-dy;
            sy=-unit;
        }else{

            sy=unit;
        }

        if(dx<0){
            dx=-dx;
            sx=-unit;
        }else{

            sx=unit;
        }


        if(dx>=dy){
            while(x0!=x1){
                f=f+dy;


                if(f>=dx){
                    if(isblock(x0+((sx-unit)/2),y0+((sy-unit)/2))){
                        return false;
                    }
                    y0=y0+sy;
                    f=f-dx;
                }


                if(f!=0 && isblock(x0+((sx-unit)/2),y0+((sy-unit)/2))){
                    return false;
                }

                if(dy==0 && isblock(x0+((sx-unit)/2),y0) && isblock(x0+((sx-unit)/2),y0-unit)){
                    return false;
                }

                x0=x0+sx;

            }

        }
        else {

            while(y0!=y1){

                f=f+dx;

                if(f>=dy){

                    if(isblock(x0+((sx-unit)/2),y0+((sy-unit)/2))){
                        return false;
                    }
                    x0=x0+sx;
                    f=f-dy;
                }

                if(f!=0 && isblock(x0+((sx-unit)/2),y0+((sy-unit)/2))){
                    return false;
                }

                if(dx==0 && isblock(x0,y0+((sy-unit)/2)) && isblock(x0-unit,y0+((sy-unit)/2))){
                    return false;
                }

                y0=y0+sy;

            }



        }



        return res;
    }

    bool lineofsightAntiAliasing(GNode *successor,GNode *parent_node)
    {
        bool res=true;


        int x0,y0,y1,x1;
        int x,y;
        x0=parent_node->x;
        y0=parent_node->y;
        x1=successor->x;
        y1=successor->y;


        int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;

        int err = dx-dy, e2, x2;                       /* error value e_xy */

        int ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);

        for ( ; ; ){

            if(isblock(x0,y0))
                return false;

            e2 = err; x2 = x0;
            if (2*e2 >= -dx) {                                    /* x step */
                 if (x0 == x1) break;
                 if (e2+dy < ed) {

                    if(isblock(x0,y0+sy))
                        return false;

                 }
                 err -= dy; x0 += sx;
            }

            if (2*e2 <= dy) {                                     /* y step */
                 if (y0 == y1) break;

                 if (dx-e2 < ed) {

                    if(isblock(x2+sx,y0))
                        return false;
                 }

                 err += dx; y0 += sy;
            }
        }

        return res;
    }


    bool lineofsight(GNode *successor,GNode *parent_node)
    {

        bool res = true;
        int x0 = parent_node->x;
        int y0 = parent_node->y;
        int x1 = successor->x;
        int y1 = successor->y;
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int x = x0;
        int y = y0;
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            if(isblock(x,y))
              return false;

            if (error > 0)
            {
                x += x_inc;
                error -= dy;
            }
            else
            {
                y += y_inc;
                error += dx;
            }
        }

        return res;
    }


    bool lineofsight_coverbr(GNode *successor,GNode *parent_node)
    {
        bool res=true;


        int x1,y1,y2,x2;
        int x,y;
        x1=parent_node->x;
        y1=parent_node->y;
        x2=successor->x;
        y2=successor->y;



        int i;               // Loop counter
        int ystep, xstep;    // The step on y and x axes
        int error;           // The error accumulated during the increment
        int errorprev;       // Stores the previous value of the error variable
        int yy = y1, xx = x1;// The line points
        int  ddy, ddx;        // Compulsory variables: the double values of dy and dx
        int dx = x2 - x1;
        int dy = y2 - y1;

        // Check start and end coordinates directly to save possible execution time
        if( (isblock(x1,y1)) || (isblock(x2,y2)))
        return false;

        if (dy < 0){
        ystep = -1;
        dy = -dy;
        }else
        ystep = 1;

        if (dx < 0){
        xstep = -1;
        dx = -dx;
        }else
        xstep = 1;

        ddy = 2 * dy; // Work with double values for full precision
        ddx = 2 * dx;

        if (ddx >= ddy){ // First octant (0 <= slope <= 1)
        // Compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = dx; // Start in the middle of the square
        error = dx;

        for (i=0 ; i < dx ; i++){ // Do not use the first point (already done)
          xx += xstep;
          error += ddy;

          if (error > ddx){  // Increment y if AFTER the middle ( > )
            yy += ystep;
            error -= ddx;
            // Three cases (octant == right->right-top for directions below):
            if (error + errorprev < ddx) { // Bottom square also
              if(isblock(xx,yy-ystep))
                return false;
            }
            else if (error + errorprev > ddx) {  // Left square also
              if(isblock(xx-xstep,yy))
                return false;
            }
            else { // Corner: bottom and left squares also (line goes exactly through corner of cells)
              if(isblock(xx,yy-ystep) && isblock(xx-xstep,yy)) // If both positions are occupied we return false, the AND can be changed to OR
                return false;
            }
          }

          if(isblock(xx,yy))
                return false;

          errorprev = error;
        }
        } else {  // The same as above

        errorprev = dy;
        error = dy;
        for (i=0 ; i < dy ; i++){
          yy += ystep;
          error += ddx;
          if (error > ddy){
            xx += xstep;
            error -= ddy;
            if (error + errorprev < ddy) {
              if(isblock(xx-xstep,yy))
                return false;
            }
            else if (error + errorprev > ddy) {
              if(isblock(xx,yy-ystep))
                return false;
            }
            else {
              if(isblock(xx-xstep,yy) && isblock(xx,yy-ystep))
                return false;
            }
          }

          if(isblock(xx,yy))
                return false;
          errorprev = error;
        }
        }

        // We have reached the end point so return true, we have line of sight
        return true;


    }

    bool isblockS(double x, double y){

        int res= GetMap(x,y);
        if(res>=1){

            return true;}
        else
            return false;


    }

    bool isblock(double x, double y){

    // TODO: substitute the hand coded values with the cellwidht and cellheight

    if( GetMap(x,y) >= 1)
      return true;

    if( GetMap(x+0.21,y) >=1 ) 
      return true;

    if( GetMap(x,y+0.21) >=1 )
        return true;

    if( GetMap(x-0.21,y) >=1 )
          return true;

    if( GetMap(x,y-0.21) >=1 )
            return true;

    return false;

    }



};
