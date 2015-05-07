#ifndef _SMP_COLLISION_CHECKER_RECTANGLE_HPP_
#define _SMP_COLLISION_CHECKER_RECTANGLE_HPP_

#include <smp/components/collision_checkers/rectangle.h>

#include <smp/components/collision_checkers/base.hpp>
#include <smp/common/region.hpp>
#include <tf/transform_broadcaster.h>


#define N_CORES 4


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::collision_checker_rectangle () {
  
    num_discretization_steps = 20;

    discretization_length = 0.1;  

    discretization_method = 2;
    //introducing the size of the robot
    size_robot=0.30;

    robot_width=1.3; //90

    robot_length=1.59; // 155

    marker_id=0;

    RADIUS=0.5;

    K=5;

    incr_mean=0;

    n_iter=0;

    index=new my_kd_tree_t(2, cloud, KDTreeSingleIndexAdaptorParams(10) );

}


template< class typeparams, int NUM_DIMENSIONS > 
smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::~collision_checker_rectangle () {

  for (typename list<region_t*>::iterator iter = list_obstacles.begin();
       iter != list_obstacles.end(); iter++) {
    
    region_t *region_curr = *iter;
    
    delete region_curr;
  }
}



template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::clean_obs (int a) {
  list_obstacles.clear();
  

  a=1;
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS>
::cc_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS>
::cc_update_insert_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS>
::cc_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS>
::cc_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}



// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision

template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::check_collision_state_2D (state_t *state_in){





if (list_obstacles.empty()){
   // ROS_WARN("Empty obstacles");
    return 1;
  }
  

  vector<Vector2D> robot_stateI;
  vector<Vector2D> obstacleI;

  robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],-robot_width/2,+robot_length/2));
  robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],-robot_width/2,-robot_length/2));
    // robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],-robot_width/2,0/2));

  robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],+robot_width/2,+robot_length/2));
  robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],+robot_width/2,-robot_length/2));
    // robot_stateI.push_back(Vector2D((*state_in)[0],(*state_in)[1],(*state_in)[2],+robot_width/2,0/2));



  const double query_pt[2] = { (*state_in)[0],(*state_in)[1]};

  // radiusSearchSearch():  Perform a search for the N closest points
  // ----------------------------------------------------------------
  // const double search_radius = static_cast<double>(0.5);

  std::vector<std::pair<size_t,double> >   ret_matches;
  nanoflann::SearchParams params;
  const size_t nObs = index->radiusSearch(&query_pt[0],RADIUS, ret_matches, params);


    if(nObs==0)
      return 1;


    for(int k=0;k<nObs;k++){

       int ind=ret_matches[k].first;
       obstacleI.clear();


       obstacleI.push_back(Vector2D(cloud.pts[ind].x-cloud.pts[ind].width/2,cloud.pts[ind].y+cloud.pts[ind].height/2));
       obstacleI.push_back(Vector2D(cloud.pts[ind].x-cloud.pts[ind].width/2,cloud.pts[ind].y-cloud.pts[ind].height/2));
       obstacleI.push_back(Vector2D(cloud.pts[ind].x+cloud.pts[ind].width/2,cloud.pts[ind].y+cloud.pts[ind].height/2));
       obstacleI.push_back(Vector2D(cloud.pts[ind].x+cloud.pts[ind].width/2,cloud.pts[ind].y-cloud.pts[ind].height/2));
       

       // publishBotPoints(robot_stateI,1);


      if(!isColliding(robot_stateI, obstacleI,(*state_in)[0],(*state_in)[1],cloud.pts[ind].x,cloud.pts[ind].y)){
             
           // publishRects(cloud.pts[ind].x,cloud.pts[ind].y,0, 1);
          // publishBotPoints(obstacleI,1);
           // ROS_WARN("a collision, robot pose %f %f",(*state_in)[0],(*state_in)[1]);
           return 0;
            
      }
      else{
    
          // publishRects(cloud.pts[ind].x,cloud.pts[ind].y,0, 0);
          // publishBotPoints(obstacleI,0);
          // ROS_WARN("NO collision, robot pose %f %f",(*state_in)[0],(*state_in)[1]);
      }
  }


  

  // publishRects((*state_in)[0],(*state_in)[1],(*state_in)[2], 1);
  // publishRects((*state_in)[0],(*state_in)[1],(*state_in)[2], 0);
  // publishBotPoints(robot_stateI,1);
  

   return 1;

}

// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::check_collision_state (state_t *state_in) {

  if (list_obstacles.empty())
    return 1;
  



      if(check_collision_state_2D (state_in))
        return 1;
      else 
        return 0;

  
 
}




template< class typeparams, int NUM_DIMENSIONS > 
bool smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::checkCollisionOneSided(vector<Vector2D> &object1, vector<Vector2D> &object2, double ob1x, double ob1y,double ob2x,double ob2y )
{


  int num_one = object1.size();

  Vector2D normal;



  /// Check the first shape's axis
  for(int i=0; i<num_one; i++)
    {
      
      // ROS_INFO("=======================");
      // ROS_INFO("=========FIRST RECT==============");
      Vector2D edge = object1[ (i+1) % num_one] - object1[i];

      normal = edge.perp();


      // ROS_INFO("Edge between (%f,%f) (%f,%f)",object1[(i+1)%num_one].x,object1[(i+1)%num_one].y,object1[i].x,object1[i].y);
      // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
      double min1 = normal.dot(object1[0]);
      double min2 =  normal.dot(object2[0]) ;

      double max1 = min1;
      double max2 = min2;


      /// For each axis compute the projections

      for(int j=0; j<object1.size(); j++)
      {

        // double dot = normal.dot(object1[j]);

        double dot = normal.proj(object1[j],object1[i]);

        if (dot < min1) 
            min1 = dot; 

        if(dot > max1) 
            max1 = dot;

        // min1 = std::min(min1, dot);
        // max1 = std::max(max1, dot);
      
      }
    

      for(int j=0; j<object2.size(); j++)
      {
        // double dot = normal.dot(object2[j]);

        double dot = normal.proj(object2[j],object1[i]);


        if (dot < min2) 
            min2 = dot; 

        if(dot > max2) 
            max2 = dot;

        // min2 = std::min(min2, dot);
        // max2 = std::max(max2, dot);

      }


     /// Check if the projections overlap
   
      if((min2 > max1 || max2 < min1)){

              // ROS_INFO("Edge between (%f,%f) (%f,%f)",object1[(i+1)%num_one].x,object1[(i+1)%num_one].y,object1[i].x,object1[i].y);
              // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
              // ROS_INFO("Not Colliding Obst. 1 Min1 %f, Max1 %f, Min2 %f, Max2 %f",min1,max1,min2,max2);  
              return true;
      }

      // ROS_INFO("=======================");
    }




/// Check the second shape's axis

int num_two = object2.size();
for(int i=0; i<num_two; i++)
    {
      
      // // double m=tan(M_PI*i/180); 
      // ROS_INFO("=======================");
      // ROS_INFO("=========SECOND RECT==============");
      Vector2D edge = object2[ (i+1) % num_two]-object2[i] ;
      normal = edge.perp();
      // ROS_INFO("Edge between (%f,%f) (%f,%f)",object2[(i+1)%num_one].x,object2[(i+1)%num_one].y,object2[i].x,object2[i].y);
      // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
      

      double min1 = normal.dot(object1[0]);
      double min2 =  normal.dot(object2[0]) ;

      double max1 = min1;
      double max2 = min2;


  /// For each axis compute the projections

      for(int j=0; j<object1.size(); j++)
      {
        

        // double dot = normal.dot(object1[j]);
        
        double dot = normal.proj(object1[j],object2[i]);


        if (dot < min1) 
            min1 = dot; 

        if(dot > max1) 
            max1 = dot;

      }
    

      for(int j=0; j<object2.size(); j++)
      {
        // double dot = normal.dot(object2[j]);
        
        double dot = normal.proj(object2[j],object2[i]);

        if (dot < min2) 
            min2 = dot; 

        if(dot > max2) 
            max2 = dot;


      }


     /// Check if the projections overlap

      if((min2 > max1 || max2 < min1)){

              // ROS_INFO("Edge between (%f,%f) (%f,%f)",object2[(i+1)%num_one].x,object2[(i+1)%num_one].y,object2[i].x,object2[i].y);
              // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
              // ROS_INFO("Not Colliding Obst. 2 Min1 %f, Max1 %f, Min2 %f, Max2 %f",min1,max1,min2,max2);  
              return true;
      }



    }

  


// if we get here then we know that every axis had overlap on it
// so we can guarantee an intersection



  return false;
}


template< class typeparams, int NUM_DIMENSIONS > 
bool smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::checkCollisionOneSided_par(vector<Vector2D> &object1, vector<Vector2D> &object2, double ob1x, double ob1y,double ob2x,double ob2y )
{

  int num_one = object1.size();

  Vector2D normal;
  int i=0;
  int cnt=0;

  omp_set_nested(1);

    #pragma omp parallel num_threads(num_one)
    {


    #pragma omp for ordered schedule(static) private(i) reduction(+:cnt) 
      /// Check the first shape's axis
  for(i=0; i<num_one; i++)
    {
      
      // ROS_INFO("=======================");
      // ROS_INFO("=========FIRST RECT==============");
      Vector2D edge = object1[ (i+1) % num_one] - object1[i];

      normal = edge.perp();


      // ROS_INFO("Edge between (%f,%f) (%f,%f)",object1[(i+1)%num_one].x,object1[(i+1)%num_one].y,object1[i].x,object1[i].y);
      // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
      double min1 = normal.dot(object1[0]);
      double min2 =  normal.dot(object2[0]) ;

      double max1 = min1;
      double max2 = min2;


      /// For each axis compute the projections
      for(int j=0; j<object1.size(); j++)
      {

        // double dot = normal.dot(object1[j]);

        double dot = normal.proj(object1[j],object1[i]);

        if (dot < min1) 
            min1 = dot; 

        if(dot > max1) 
            max1 = dot;


      
      }
    
      for(int j=0; j<object2.size(); j++)
      {
        // double dot = normal.dot(object2[j]);

        double dot = normal.proj(object2[j],object1[i]);


        if (dot < min2) 
            min2 = dot; 

        if(dot > max2) 
            max2 = dot;

      }


     /// Check if the projections overlap
   
      if((min2 > max1 || max2 < min1)){

              // ROS_INFO("Edge between (%f,%f) (%f,%f)",object1[(i+1)%num_one].x,object1[(i+1)%num_one].y,object1[i].x,object1[i].y);
              // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
              // ROS_INFO("Not Colliding Obst. 1 Min1 %f, Max1 %f, Min2 %f, Max2 %f",min1,max1,min2,max2);  
              // return true;
              cnt++;
      }

      // ROS_INFO("=======================");
    }


}

/// Check the second shape's axis

if(cnt>0)
  return true;

int num_two = object2.size();

    #pragma omp parallel num_threads(num_one)
    {


    #pragma omp for ordered schedule(static) private(i) reduction(+:cnt) 
    for(int i=0; i<num_two; i++)
    {
      
      // // double m=tan(M_PI*i/180); 
      // ROS_INFO("=======================");
      // ROS_INFO("=========SECOND RECT==============");
      Vector2D edge = object2[ (i+1) % num_two]-object2[i] ;
      normal = edge.perp();
      // ROS_INFO("Edge between (%f,%f) (%f,%f)",object2[(i+1)%num_one].x,object2[(i+1)%num_one].y,object2[i].x,object2[i].y);
      // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
      

      double min1 = normal.dot(object1[0]);
      double min2 =  normal.dot(object2[0]) ;

      double max1 = min1;
      double max2 = min2;


  /// For each axis compute the projections
      for(int j=0; j<object1.size(); j++)
      {
        

        // double dot = normal.dot(object1[j]);
        
        double dot = normal.proj(object1[j],object2[i]);


        if (dot < min1) 
            min1 = dot; 

        if(dot > max1) 
            max1 = dot;

      }
    
      for(int j=0; j<object2.size(); j++)
      {
        // double dot = normal.dot(object2[j]);
        
        double dot = normal.proj(object2[j],object2[i]);

        if (dot < min2) 
            min2 = dot; 

        if(dot > max2) 
            max2 = dot;


      }


     /// Check if the projections overlap

      if((min2 > max1 || max2 < min1)){

              // ROS_INFO("Edge between (%f,%f) (%f,%f)",object2[(i+1)%num_one].x,object2[(i+1)%num_one].y,object2[i].x,object2[i].y);
              // ROS_INFO("Current Axis (%f,%f)", normal.x, normal.y);
              // ROS_INFO("Not Colliding Obst. 2 Min1 %f, Max1 %f, Min2 %f, Max2 %f",min1,max1,min2,max2);  
              // return true;
            cnt++;
      }



    }

  }

if(cnt>0)
  return true;
// if we get here then we know that every axis had overlap on it
// so we can guarantee an intersection



  return false;

}


template< class typeparams, int NUM_DIMENSIONS > 
bool smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::isColliding(vector<Vector2D> &object1, vector<Vector2D> &object2, double ob1x, double ob1y,double ob2x,double ob2y)
{


    bool r=false;
    double wt ;

    if(PARALLEL){
      
       wt = omp_get_wtime();
       r=checkCollisionOneSided_par(object1, object2, ob1x,ob1y,ob2x, ob2y);

       return r;
        

    }else{
          
          wt = omp_get_wtime();
          r=checkCollisionOneSided(object1, object2, ob1x,ob1y,ob2x, ob2y);

          return r;
    }




}
 



// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::check_collision_trajectory (trajectory_t *trajectory_in) {
  
  
//  if (list_obstacles.size() == 0)
if (list_obstacles.empty())
    return 1;
  
  if (trajectory_in->list_states.size() == 0)
    return 1;


  typename list<state_t*>::iterator iter = trajectory_in->list_states.begin();


  state_t *state_prev = *iter;


  if (this->check_collision_state (state_prev) == 0)
    return 0;
  
  iter++;
  

  for (; iter != trajectory_in->list_states.end(); iter++) {
    
    state_t *state_curr = *iter;
    
    if (discretization_method != 0) { 
      // Compute the increments 
      double dist_total = 0.0;
      double increments[NUM_DIMENSIONS];
      for (int i = 0; i < NUM_DIMENSIONS; i++) {
    double increment_curr = (*state_curr)[i] - (*state_prev)[i];
    dist_total += increment_curr * increment_curr;
    increments[i] = increment_curr;
      }
      dist_total = sqrt(dist_total);


      // Compute the number of increments
      int num_increments;    
      if (discretization_method == 1) {
    num_increments = num_discretization_steps;
      }
      else if (discretization_method == 2){
    num_increments = (int) floor(dist_total/discretization_length);
      }


      if (num_increments > 0) { // Execute the remaining only if the discretization is required.

    for (int i = 0; i < NUM_DIMENSIONS; i++)  // Normalize the increments.
      increments[i] = increments[i]/((double)(num_increments+1));

    for (typename list<region_t *>::iterator iter = list_obstacles.begin(); 
         iter != list_obstacles.end(); iter++) {

      region_t *region_curr = *iter;

      for (int idx_state = 1; idx_state <= num_increments; idx_state++){
        bool collision = true;

        for (int i = 0; i < NUM_DIMENSIONS; i++) {
          if (fabs((*state_prev)[i] + increments[i]*idx_state - region_curr->center[i]) 
          >= (region_curr->size[1]/2)) {
        collision = false;
          }
        }
        if (collision == true) {
          return 0;
        }
      }
    }
      }
    }
    
    if (check_collision_state (state_curr) == 0){
      return 0;
    }

    state_prev = state_curr;
  }      
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::set_discretization_steps (int num_discretization_steps_in) {
  
  if (num_discretization_steps <= 0) {
    num_discretization_steps = 0;
    discretization_length = 0;  
    discretization_method = 0;
  }
  else {
    num_discretization_steps = num_discretization_steps_in;
    discretization_method = 1;
  }
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::set_discretization_length (double discretization_length_in) {
  
  if (discretization_length <= 0.0) {
    num_discretization_steps = 0;
    discretization_length = 0.05;  
    discretization_method = 0;
  }
  else {
    discretization_length = discretization_length_in;
    discretization_method = 2;
  }
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::add_obstacle (region_t &obstacle_in) {
  
  list_obstacles.push_back (new region_t(obstacle_in));

  if(!readObstaclePoints(cloud))
        return 1;
      else
       return 0;

 
}



template< class typeparams, int NUM_DIMENSIONS > 
int smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::readObstaclePoints(PointCloud<double> &point){




  if (list_obstacles.empty())
    return 1;
  
  int n_obst=list_obstacles.size();

  point.pts.resize(n_obst);

  int i=0;

  for (typename list<region_t *>::iterator iter = list_obstacles.begin(); iter != list_obstacles.end(); iter++) {
          region_t *region_curr = *iter;
    
            point.pts[i].x=region_curr->center[0];
            point.pts[i].y=region_curr->center[1];
            point.pts[i].width=region_curr->size[1];
            point.pts[i].height=region_curr->size[1]; 

           i++;
    }

 return 0;

}

template< class typeparams, int NUM_DIMENSIONS > 
void smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::setParam(int k, double r, int par){


    this->K=k;

    this->RADIUS=r;

    this->PARALLEL=par;


}


template< class typeparams, int NUM_DIMENSIONS > 
void smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS> 
::setRobotDim(double width, double length,double collision_boundary){

    /// x axis, robot length+Collision error area
    this->robot_length=length+collision_boundary*2;
    /// Y axis, robot width+Collision error area
    this->robot_width=width+collision_boundary*2; 



}

template< class typeparams, int NUM_DIMENSIONS > 
double smp::collision_checker_rectangle<typeparams,NUM_DIMENSIONS>
::build_index(){

  index->buildIndex();


}

#endif
