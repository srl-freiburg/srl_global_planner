#ifndef _SMP_MINIMUM_TIME_REACHABILITY_HPP_
#define _SMP_MINIMUM_TIME_REACHABILITY_HPP_

#include <smp/components/multipurpose/minimum_time_reachability_thetastar.h>

#include <smp/planners/rrtstar.hpp>
#include <smp/common/region.hpp>
#include <smp/components/model_checkers/base.hpp>
#include <smp/components/cost_evaluators/base.hpp>


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::minimum_time_reachability () {
  begin_time = clock();
  end_time = 0;
  cntForClock=0;
  foundTraj=0;
  cost=100;
  min_cost_vertex = NULL;
  NOTLEARNED=0;
  ONLYTHETACOST=0;
  Kd=10;
  Kangle=10;
  cntUpdates=0;

  Kdist=0.1;
  Kth=0.8;
  Kor=0.1;

  n_dis_traj = 10;

  ADD_COST_THETASTAR  = 0;
  ADD_COST_PATHLENGTH = 1;
  ADD_COST_FROM_COSTMAP = false;

  MODEL_COST=0;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 0.0;
  }

}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::~minimum_time_reachability () {


}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::minimum_time_reachability (const region_t &region_in) {

  region_goal = region_in;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::initWorldModel(base_local_planner::CostmapModel* world_model,std::vector<geometry_msgs::Point> footprint_spec, double inscribed_radius, double circumscribed_radius, costmap_2d::Costmap2DROS* costmap_ros, std::string planner_frame, tf::TransformListener *listener){

  this->world_model_ = world_model;
  this->footprint_spec_ = footprint_spec;
  ROS_DEBUG("CollisionChecker Size of the footprint_spec %d", (int )footprint_spec_.size());

  this->circumscribed_radius_= circumscribed_radius;
  this->inscribed_radius_ = inscribed_radius;
  this->costmap_ros_ = costmap_ros; ///< @brief The ROS wrapper for the costmap the controller will use

  CostEvaluatorlistener = listener;

  global_frame_ = costmap_ros_->getGlobalFrameID();

  rob_foot_print_ = costmap_ros_->getRobotFootprint();

  try{

      ROS_DEBUG("Getting Transform from %s to %s", planner_frame.c_str(), global_frame_.c_str() );

      CostEvaluatorlistener->waitForTransform( global_frame_, planner_frame, ros::Time(0), ros::Duration(0.20));
      CostEvaluatorlistener->lookupTransform(global_frame_, planner_frame, ros::Time(0), transform_);
    }
    catch(tf::TransformException){

        ROS_ERROR("Failed to receive transform from CostMap");
        return 0;
  }


  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::getCostFromMap (double x_i, double y_i, double theta_i) {


    tf::Pose source;

    tf::Quaternion q= tf::createQuaternionFromRPY(0,0,theta_i);

    tf::Matrix3x3 base(q);

    source.setOrigin(tf::Vector3(x_i,y_i,0));

    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result=transform_*source;

    double x_bot = result.getOrigin().x();
    double y_bot = result.getOrigin().y();
    double orient_bot = tf::getYaw( result.getRotation());

    // ROS_DEBUG("From (%f,%f,%f) to (%f,%f,%f) ", x_i, y_i, theta_i, x_bot, y_bot, orient_bot);

    std::vector<geometry_msgs::Point> footprint_spec;

    footprint_spec.clear();

    // Given a pose, build the oriented footprint of the robot.
    // costmap_ros_->getOrientedFootprint(x_bot , y_bot , orient_bot, footprint_spec);


        // build the oriented footprint at the robot's current location
        double cos_th = cos(orient_bot);
        double sin_th = sin(orient_bot);
        for (unsigned int i = 0; i < rob_foot_print_.size(); ++i){
            geometry_msgs::Point new_pt;
            new_pt.x = x_bot + (rob_foot_print_[i].x * cos_th - rob_foot_print_[i].y * sin_th);
            new_pt.y = y_bot + (rob_foot_print_[i].x * sin_th + rob_foot_print_[i].y * cos_th);
            footprint_spec.push_back(new_pt);

        }

    // ROS_DEBUG("CollisionCheckerState Size of the footprint_spec %d", (int )footprint_spec.size());
    geometry_msgs::Point robot_point;

    robot_point.x = x_bot;
    robot_point.y = y_bot;
    robot_point.z = 0;


    double cost = world_model_->footprintCost(robot_point, footprint_spec, inscribed_radius_, circumscribed_radius_);


    if(cost < 0)
        cost = 255;

  return cost;
}





template < class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::set_goal_region (const region_t &region_in) {

  region_goal = region_in;

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::set_angle_to_range(double alpha, double min)
{

    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}

template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::diff_angle_unwrap(double alpha1, double alpha2)
{
    double delta;

    // normalize angles alpha1 and alpha2
    alpha1 = set_angle_to_range(alpha1, 0);
    alpha2 = set_angle_to_range(alpha2, 0);

    // take difference and unwrap
    delta = alpha1 - alpha2;
    if (alpha1 > alpha2) {
        while (delta > M_PI) {
            delta -= 2.0 * M_PI;
        }
    } else if (alpha2 > alpha1) {
        while (delta < -M_PI) {
            delta += 2.0 * M_PI;
        }
    }
    return delta;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::ce_update_vertex_cost (vertex_t *vertex_in) {


  if (vertex_in->data.reaches_goal == true) {



    bool update_trajectory = false;

    if (min_cost_vertex == NULL) {
      min_cost_vertex = vertex_in;
      ROS_INFO_STREAM("RRT* Trajectory Cost : " << vertex_in->data.total_cost << endl);
      /// saving the cost
      cost= vertex_in->data.total_cost;
      update_trajectory = true;
            cntUpdates++;

    }


    if ( (vertex_in->data.total_cost < min_cost_vertex->data.total_cost) ) {
      
      ROS_INFO_STREAM("RRT* Trajectory Cost : " << vertex_in->data.total_cost << endl);
      /// saving the cost
      cost=vertex_in->data.total_cost;
      min_cost_vertex = vertex_in;
      update_trajectory = true;
            cntUpdates++;

    }




    if (update_trajectory == true) {
            // Update the time only for the firt Found trajectory
        if(cntForClock==0){
                foundTraj=true;
                end_time=float( clock () - begin_time ) /  CLOCKS_PER_SEC;
            }
        cntForClock++;

      min_cost_trajectory.clear_delete ();

      vertex_t *vertex_ptr = min_cost_vertex;
      while (1) {

	      edge_t *edge_curr = vertex_ptr->incoming_edges.back();

	      trajectory_t *trajectory_curr = edge_curr->trajectory_edge;

	      min_cost_trajectory.list_states.push_back (new state_t(*(vertex_ptr->state)));

	if (vertex_ptr->incoming_edges.size() == 0)
	  break;



    for (typename list<state_t*>::reverse_iterator it_state = trajectory_curr->list_states.rbegin();
         it_state != trajectory_curr->list_states.rend(); ++it_state) {
      min_cost_trajectory.list_states.push_back (new state_t(**it_state));
    }


    for (typename list<input_t*>::reverse_iterator it_input = trajectory_curr->list_inputs.rbegin();
         it_input != trajectory_curr->list_inputs.rend(); ++it_input) {
      min_cost_trajectory.list_inputs.push_back (new input_t(**it_input));
    }

	vertex_ptr = edge_curr->vertex_src;
      }

      // Call all the update functions
      for (typename list<update_func_t>::iterator it_func = list_update_functions.begin();
	   it_func != list_update_functions.end(); it_func++) {

	(*it_func)(&min_cost_trajectory);
      }


    }
  }


  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::ce_update_edge_cost (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::mc_update_insert_vertex (vertex_t *vertex_in) {

  for (int i = 0; i < NUM_DIMENSIONS; i++)  {
    if (fabs (vertex_in->state->state_vars[i] - region_goal.center[i]) > region_goal.size[i]) {
      vertex_in->data.reaches_goal = false;
      return 1;
    }
  }

  vertex_in->data.reaches_goal = true;

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::mc_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::mc_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::mc_update_delete_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::get_solution (trajectory_t &trajectory_out) {


  if (!min_cost_vertex)
    return 1;

  trajectory_out.clear();

  for (typename list<state_t*>::iterator it_state = min_cost_trajectory.list_states.begin();
       it_state != min_cost_trajectory.list_states.end(); it_state++) {

    trajectory_out.list_states.push_front (new state_t(**it_state));
  }

  for (typename list<input_t*>::iterator it_input = min_cost_trajectory.list_inputs.begin();
       it_input != min_cost_trajectory.list_inputs.end(); it_input++) {
    trajectory_out.list_inputs.push_front (new input_t(**it_input));
  }



  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::update_trajectory (trajectory_t *trajectory_in){


   path_support.clear_delete ();

    for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin(); it_state != trajectory_in->list_states.end(); it_state++) {
        state_t *state_curr = *it_state;
        path_support.list_states.push_back (new state_t(*state_curr));

    }

    for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin(); it_input != trajectory_in->list_inputs.end(); it_input++) {
        input_t *input_curr = *it_input;
        path_support.list_inputs.push_back (new input_t(*input_curr));
    }




      int N;
      N= path_support.list_states.size();

      this->path.resize(N,2);
      this->distances.resize(N,1);

      double xi,yi,thetai;
      int i=0;
      tot_dist=0;
      // save the means from the trajectory
      for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

        state_t *state_curr = *it_state;
        xi = (state_curr)->state_vars[0];
        yi = (state_curr)->state_vars[1];
        thetai = (state_curr)->state_vars[2];
        this->path(i,0)=xi;
        this->path(i,1)=yi;

        if(i>0){
          this->distances(i-1,0)=sqrt((this->path(i,0)-this->path(i-1,0))*(this->path(i,0)-this->path(i-1,0))+(this->path(i,1)-this->path(i-1,1))*(this->path(i,1)-this->path(i-1,1)));
          tot_dist+=this->distances(i-1,0);
        }

        i++;
      }


}


template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::cost_proj (double x, double y, double theta){


      int N;
      N= path_support.list_states.size();


      Eigen::MatrixXd relativ_dist(N,1);

      double sx_,sy_,dsx_,dsy_,ux_,uy_,vx_,vy_,w_,rp_;


      double dt=0.1;



    for(int j=0;j<N-1;j++){

        Eigen::MatrixXd p1(1,2);
        Eigen::MatrixXd p2(1,2);


        p1(0,0)=this->path(j,0);
        p1(0,1)=this->path(j,1);

        p2(0,0)=this->path(j+1,0);
        p2(0,1)=this->path(j+1,1);

        ux_=(p2(0,0)-p1(0,0))/sqrt((p2(0,0)-p1(0,0))*(p2(0,0)-p1(0,0))+(p2(0,1)-p1(0,1))*(p2(0,1)-p1(0,1)));
        uy_=(p2(0,1)-p1(0,1))/sqrt((p2(0,0)-p1(0,0))*(p2(0,0)-p1(0,0))+(p2(0,1)-p1(0,1))*(p2(0,1)-p1(0,1)));
        vx_=(x-p1(0,0));
        vy_=(y-p1(0,1));

        //        cout<<"Vector U, norm U "<<ux_<< " "<<uy_<<", "<<sqrt(ux_*ux_+uy_*uy_)<<endl;
        //        cout<<"Vector V, norm V "<<vx_<<" "<<vy_<<", "<<sqrt(vx_*vx_+vy_*vy_)<<endl;
        w_=(ux_*vx_+uy_*vy_);
        sx_=p1(0,0)+ux_*w_;
        sy_=p1(0,1)+uy_*w_;

        relativ_dist(j,0)=sqrt((x-sx_)*(x-sx_)+(y-sy_)*(y-sy_));

      }



    double min_v=relativ_dist(0,0);
    int min_i=0;
    for(int j=1;j<N-1;j++){

      if(  min_v  >relativ_dist(j,0)){


        min_v=relativ_dist(j,0);
        min_i=j;
      }

    }



    double d;
    d=relativ_dist(min_i,0);

    double res=0;

    double or_rob_succ;
    or_rob_succ=atan2((this->path(min_i+1,1)-this->path(min_i,1))/dt,(this->path(min_i+1,0)-this->path(min_i,0))/dt);

    /// Repulsive Theta* Path Force
    if(MODEL_COST==0){


      res=1/((d*d)*0.5*Kd);
    }

    /// Attractive Theta* Path Force
    if(MODEL_COST==1)
    {

      res=((d*d)*0.5*Kd);

    }


    double orientation_factor=0;
    if(MODEL_COST==2){

      orientation_factor=fabs(1-cos(diff_angle_unwrap(or_rob_succ,theta)));

      res=((d)*Kd)+Kangle*orientation_factor;



    }

    return res;



}



template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::evaluate_cost_trajectory (state_t *state_initial_in,
			    trajectory_t *trajectory_in,
			    state_t *state_final_in) {

  double total_cost = 0.0;
  double initialCost_=0;


  //! FOR POSITION CONTROLLER
  int cnt;
  double a;
  a=1000;
  double old_x,old_y,new_x,new_y,s,sw,vmax, old_theta, new_theta;
  vmax=2;
  old_x=0;
  old_y=0;
  new_x=0;
  new_y=0;
  cnt=0;
  /// Initial cost introduced as discount factor to penilize multi steps trajectories
  total_cost+=initialCost_;

  if(ONLYTHETACOST){


        double xin, yin, thin, xout, yout, thout;
        double xmid,ymid,thmid;
        double xmidt,ymidt,thmidt;
        ///state_t *final=trajectory_in->list_states.back();

        xin=(*state_initial_in)[0];
        yin=(*state_initial_in)[1];
        thin=(*state_initial_in)[2];


        if(trajectory_in->list_states.size()>0){


            state_t *final=trajectory_in->list_states.back();
            xout=(*final)[0];
            yout=(*final)[1];
            thout=(*final)[2];

        }
        else{


            xout=(*state_final_in)[0];
            yout=(*state_final_in)[1];
            thout=(*state_final_in)[2];

        }

        total_cost+=cost_proj(xout,yout,thout)+cost_proj(xin,yin,thin);

        return total_cost;

  }



  if(NOTLEARNED){

        for (typename list<state_t*>::iterator iter=trajectory_in->list_states.begin();
          iter!= trajectory_in->list_states.end();iter++){
           state_t *state_curr=*iter;
          if(cnt==0){
              old_x=(*state_curr)[0];
              old_y=(*state_curr)[1];
              old_theta=(*state_curr)[2];
              // Initial value
              total_cost+=0.1;
          }
          else{

            new_x=(*state_curr)[0];
            new_y=(*state_curr)[1];
            new_theta=(*state_curr)[2];
      //      s=sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))/vmax;
            s=Kdist*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+Kor*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)));

            if(ADD_COST_PATHLENGTH)
              total_cost+=s;

            old_y=new_y;
            old_x=new_x;
            old_theta=new_theta;

            if(ADD_COST_FROM_COSTMAP)
              total_cost+= getCostFromMap(new_x,new_y,new_theta);

          }
          cnt++;

        }


      if(ADD_COST_THETASTAR){

        double xin, yin, thin, xout, yout, thout;
        ///state_t *final=trajectory_in->list_states.back();

        xin=(*state_initial_in)[0];
        yin=(*state_initial_in)[1];
        thin=(*state_initial_in)[2];


        if(trajectory_in->list_states.size()>0){


            state_t *final=trajectory_in->list_states.back();
            xout=(*final)[0];
            yout=(*final)[1];
            thout=(*final)[2];



          }
          else{


            xout=(*state_final_in)[0];
            yout=(*state_final_in)[1];
            thout=(*state_final_in)[2];

          }

          if(trajectory_in->list_states.size()>0){
              total_cost+=cost_proj(xout,yout,thout)+cost_proj(xin,yin,thin);

          }else{

              total_cost+=cost_proj(xout,yout,thout)+cost_proj(xin,yin,thin);

          }


      }



  }else{

      double xin, yin, thin, xout, yout, thout;


      ///state_t *final=trajectory_in->list_states.back();

      xin=(*state_initial_in)[0];
      yin=(*state_initial_in)[1];
      thin=(*state_initial_in)[2];

      if(trajectory_in->list_states.size()>0){


        state_t *final=trajectory_in->list_states.back();
        xout=(*final)[0];
        yout=(*final)[1];
        thout=(*final)[2];

      }
      else{


        xout=(*state_final_in)[0];
        yout=(*state_final_in)[1];
        thout=(*state_final_in)[2];
      }

    if(ADD_COST_THETASTAR){

          if(trajectory_in->list_states.size()>0){
              total_cost+=regression_nlm( xin, yin, thin, xout, yout, thout)+cost_proj(xout,yout,thout);

            }else{

              total_cost+=regression_nlm( xin, yin, thin, xout, yout, thout)+cost_proj(xout,yout,thout)+cost_proj(xin,yin,thin);

            }
    }


    if(ADD_COST_PATHLENGTH)
      total_cost+=regression_nlm( xin, yin, thin, xout, yout, thout);



    if(ADD_COST_FROM_COSTMAP){


          for (typename list<state_t*>::iterator iter=trajectory_in->list_states.begin();
            iter!= trajectory_in->list_states.end();iter++){

            state_t *state_curr=*iter;
            total_cost+= getCostFromMap((*state_curr)[0],(*state_curr)[1],(*state_curr)[2]);

          }

    }


    // Scale the cost properly
    total_cost=total_cost/3;


  }


  return total_cost;

}





template< class typeparams, int NUM_DIMENSIONS>
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::regression_nlm(double xin,double yin,double thin,double xout,double yout,double thout){

  int NFEATURES=14;
  int NPARAMS=29;

  double feat_in[NFEATURES];
  // vector of paramas
  double p[NPARAMS];



    if((yout-yin)>=0){




        feat_in[0]= xout-xin;
        feat_in[1]= yout-yin;
        feat_in[2]= diff_angle_unwrap(thout,thin);
        feat_in[3]= sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));
        feat_in[4]= cos(diff_angle_unwrap(thout,thin));
        feat_in[5]= sin(diff_angle_unwrap(thout,thin));
        feat_in[6]= ( diff_angle_unwrap(thout,thin)*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[7]= ( cos(diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[8]= ( sin(diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[9]=  diff_angle_unwrap(atan2(yout-yin,xout-xin),0);
        feat_in[10]= diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin));

        if(diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin))!=0)

            feat_in[11]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)/diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin));
        else
            feat_in[11]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)/0.0000001;

        feat_in[12]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));
        feat_in[13]= diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));

    }
    else{

           // given the simmetry
        yout=-yout;
        thout=-thout;
        feat_in[0]= xout-xin;
        feat_in[1]= yout-yin;
        feat_in[2]= diff_angle_unwrap(thout,thin);
        feat_in[3]= sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));
        feat_in[4]= cos(diff_angle_unwrap(thout,thin));
        feat_in[5]= sin(diff_angle_unwrap(thout,thin));
        feat_in[6]= ( diff_angle_unwrap(thout,thin)*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[7]= ( cos(diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[8]= ( sin(diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin)) );
        feat_in[9]=  diff_angle_unwrap(atan2(yout-yin,xout-xin),0);
        feat_in[10]= diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin));

        if(diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin))!=0)
          feat_in[11]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)/diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin));
        else
          feat_in[11]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)/0.0000001;

        feat_in[12]= diff_angle_unwrap(atan2(yout-yin,xout-xin),0)*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));
        feat_in[13]= diff_angle_unwrap(atan2(yout-yin,xout-xin),diff_angle_unwrap(thout,thin))*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin));





    }


    /// Filling the params... HAND MADE SOLUTION
    p[0]=-2.8717*0.00001;
    p[1]=-46.884;
    p[2]=7.031*0.000001;
    p[3]= 35.032;
    p[4]=-0.0021241;
    p[5]=1.3666;
    p[6]=-0.010636;
    p[7]=43.701;
    p[8]=-.0077482;
    p[9]=-1.2206;
    p[10]=0.0063052;
    p[11]=0.66874;
    p[12]=6.6832*0.00001;
    p[13]=5.7338;
    p[14]=0.011268;
    p[15]=-0.15003;
    p[16]=0.010104;
    p[17]=-0.037837;
    p[18]=0.00015487;
    p[19]=-9.2476;
    p[20]=0.0010415;
    p[21]=-4.6392;
    p[22]=.000000014361;
    p[23]=-3822.7;
    p[24]=0.000002983;
    p[25]=-42.046;
    p[26]=-0.000010996;
    p[27]=-50.86;
    p[28]=20.238;

    double acc;
    int par_ind;
    par_ind=0;
    acc=0;

    for(int i=0;i<NFEATURES;i++){

      par_ind=i*2;
      acc=acc+p[par_ind]*(feat_in[i]-p[par_ind+1])*(feat_in[i]-p[par_ind+1]);

    }

return (acc+p[28]);

}






template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::get_best_cost () {

  if (min_cost_vertex == NULL)
    return -1.0;
  else
    return (double)(min_cost_vertex->data.total_cost);
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::clear_update_function_list () {

  list_update_functions.clear();

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::register_new_update_function (update_func_t update_function) {

  if (update_function == NULL)
    return 0;

  list_update_functions.push_back (update_function);

  return 1;
}





#endif
