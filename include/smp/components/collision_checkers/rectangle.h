/*! \file components/collision_checkers/standard.h
  \brief The standard brute-force collision checker
  
  This file implements the standard collision checker class using a rectangle. The region
  class, which is used to describe rectangular obstacles in the Euclidean
  space is defined in region.h
*/
#ifndef _SMP_COLLISION_CHECKER_RECTANGLE_H_
#define _SMP_COLLISION_CHECKER_RECTANGLE_H_

#include <smp/components/collision_checkers/base.h>

#include <smp/common/region.h>

#include <list>


#include "nanoflann.hpp"

using namespace nanoflann;

namespace smp {

    //! Standard collision checker
    /*!
      This class implements the standard collision checker using the Separating Axis Theoremo for rectangles
      Standard collision checking procedure discretizes the trajectories connecting consecutive 
      states. The said trajectory is obtained by a linear interpolation between 
      the said states. Each interpolated state is, then, checked for collisioon
      with all the obstacles. This procedure is continued for all the states in 
      the trajectory. A single states is checked for collision by merely going
      through the list of obstacles to check whether the query state resides
      inside any of the obstacles.
      
      \ingroup collision_checkers
    */

    class Vector2D
        {
        public:
          Vector2D() : x(0), y(0) {}

          Vector2D(double x, double y) : x(x), y(y) {}

          Vector2D(double x, double y,double theta) : x(x), y(y) {


            this->x= this->x*cos(theta)-this->y*sin(theta);
            this->y= this->x*sin(theta)+this->y*cos(theta);     

          }

          Vector2D(double x, double y,double theta, double w, double h) : x(x), y(y) {


              this->transform(theta,w,h);

          }
          Vector2D operator-(const Vector2D &other) const
          {
            double x_,y_;
            x_=(x - other.x);
            y_=(y - other.y);
  

            return Vector2D(x_, y_);
          }

          double dot(const Vector2D &other) const
          {
            return x * other.x + y*other.y;
          }

          double proj(const Vector2D &other,const Vector2D &base) const
          {
            return x * (other.x -base.x)+ y*(other.y-base.y);
          }


          Vector2D perp() const
          {
            return Vector2D(-y, x);
          }

          double cross( const Vector2D &other) const
          {
            return x * other.y - y * other.x;
          }

          void negate(){

            this->x=-x;
            this->y=-y;


          }


          bool transform(double theta,double w, double h){

            this->x= w*cos(theta)-h*sin(theta) +this->x;
            this->y= w*sin(theta)+h*cos(theta) +this->y;     
          }


          double x,y;




        };



      template <typename T>
      struct PointCloud
      {
        struct ObstaclePoint
        {
          T  x,y,width,height;
        };

        std::vector<ObstaclePoint>  pts;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t size) const
        {
          const T d0=p1[0]-pts[idx_p2].x;
          const T d1=p1[1]-pts[idx_p2].y;
          return d0*d0+d1*d1;
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline T kdtree_get_pt(const size_t idx, int dim) const
        {
          if (dim==0) return pts[idx].x;
          else if (dim==1) return pts[idx].y;
        
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX &bb) const { return false; }

      };




    template< class typeparams, int NUM_DIMENSIONS >
    class collision_checker_rectangle : public collision_checker_base<typeparams> {
        
        
        
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
        collision_checker_rectangle();
        ~collision_checker_rectangle ();




        ros::NodeHandle nh_collisions_;
                       /// Publisher for the Trajectories
        ros::Publisher pub_rects_;

        ros::Publisher pub_bot_points;


	      float size_robot;

        double robot_width;

        double robot_length;

        
        int marker_id;

        PointCloud<double> cloud;

        size_t K;

        double RADIUS;


        double incr_mean;

        int n_iter;

        int PARALLEL;



        typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud<double> > , PointCloud<double>, 2> my_kd_tree_t;

        my_kd_tree_t   *index;



        int cc_update_insert_vertex (vertex_t *vertex_in);
    

        int cc_update_insert_edge (edge_t *edge_in);  


        int cc_update_delete_vertex (vertex_t *vertex_in);
    
    
        int cc_update_delete_edge (edge_t *edge_in);


        int check_collision_state (state_t *state_in);


        int check_collision_state_2D (state_t *state_in);



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

        bool isColliding(vector<Vector2D> &object1, vector<Vector2D> &object2,double ob1x, double ob1y,double ob2x,double ob2y);

        bool checkCollisionOneSided(vector<Vector2D> &object1, vector<Vector2D> &object2,double ob1x, double ob1y,double ob2x,double ob2y);
	
        bool checkCollisionOneSided_par(vector<Vector2D> &object1, vector<Vector2D> &object2,double ob1x, double ob1y,double ob2x,double ob2y);

        int clean_obs(int a);

        int readObstaclePoints(PointCloud<double> &point);

        void setParam(int k, double r, int par);

        void setRobotDim(double width, double length, double collision_boundary);

        double  build_index();






    };


}

#endif
