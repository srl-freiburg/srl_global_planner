#ifndef _SMP_MINIMUM_TIME_REACHABILITY_HPP_
#define _SMP_MINIMUM_TIME_REACHABILITY_HPP_

#include <smp/components/multipurpose/minimum_time_reachability.h>
#include <smp/planners/rrtstar.hpp>
#include <smp/common/region.hpp>
#include <smp/components/model_checkers/base.hpp>
#include <smp/components/cost_evaluators/base.hpp>
#include <smp/components/multipurpose/costtogo/costotogo.c>


#define NFEATURES 14
#define NPARAMS 29



template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::minimum_time_reachability () {
  begin_time = clock();
  end_time = 0;
  cntForClock=0;
  foundTraj=0;
  cost=100;
  min_cost_vertex = NULL;
  cntUpdates=0;
  relations_.clear();
  people_.clear();
  map_people_.clear();
  SCALING_IRL=10;
  SCALING_SMOOTHCOST=100;
  lock_callbacks=0;
  COST_DISPLAY=0;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 0.0;
  }


}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::~minimum_time_reachability () {

  sub_tracked_persons_.shutdown();
  sub_tracked_groups_.shutdown();
  pub_irl_actions_.shutdown();
  pub_relations_.shutdown();
  relations_file_.close();
}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::minimum_time_reachability (const region_t &region_in) {

  region_goal = region_in;
}

template < class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::mapRange(double value, double mina, double maxa, double mint, double maxt)
{
    return mint + ((value - mina) * (maxt - mint) / (maxa - mina));
}


template < class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::init_ros_units(ros::NodeHandle node){

  nh_cost_ev_=node;

  ROS_INFO("Initializing ROSNODE INTO minimum_time_reachability");
  people_.clear();
  relations_.clear();

  // set up subscribers
  sub_tracked_persons_ = nh_cost_ev_.subscribe ("/pedsim/tracked_persons", 1, &smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>::callbackTrackedPersons, this);
  sub_tracked_groups_ = nh_cost_ev_.subscribe ("/pedsim/tracked_groups", 1, &smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>::callbackTrackedGroups, this);

  // set up publishers
  pub_irl_actions_ = nh_cost_ev_.advertise<visualization_msgs::MarkerArray> ("/pedsim/synthetic_actions", 1);
  pub_relations_ = nh_cost_ev_.advertise<visualization_msgs::MarkerArray> ("/pedsim/relations", 1);
  pub_tracked_persons_ = nh_cost_ev_.advertise<pedsim_msgs::TrackedPersons> ("/planning/tracked_persons", 1);
  pub_tracked_groups_ = nh_cost_ev_.advertise<pedsim_msgs::TrackedGroups> ("/planning/tracked_groups", 1);

  // read config
  ros::param::param<std::string> ("/irl_engine/reward_type", reward_type_, "polite" );

  /// Initializing
  Array4D g = {{27.0, 27.0, 0, 0}};

  goal_ = g;


  nh_cost_ev_.getParam("irl_engine/reward_weights_polite", polite_);
  nh_cost_ev_.getParam("irl_engine/reward_weights_rude", rude_);
  nh_cost_ev_.getParam("irl_engine/reward_weights_friendly", friendly_);
  ROS_INFO("Params sizes polite_ %d, rude_ %d, friendly_ %d ",(int)polite_.size(),(int)rude_.size(),(int)friendly_.size());
         // world dimension
  nh_cost_ev_.getParam("irl_engine/world_width", world_width_);
  nh_cost_ev_.getParam("irl_engine/world_height", world_height_);

  if((int)polite_.size()==0 && (int)rude_.size()==0 &&(int)friendly_.size() ==0){
    
    ROS_ERROR("Error Params not read!!!!!");
    ROS_ERROR("Error Params not read!!!!!");
    ROS_ERROR("Error Params not read!!!!!");
    ROS_ERROR("Error Params not read!!!!!");



    return 0;

  }

  ros::Rate loop_rate(20);
  ros::spinOnce();
  if(this->SELECT_FUNC>=4)
  {
    
  ROS_WARN("  Waitin people_ and relations_ information to load, this->SELECT_FUNC = %d", this->SELECT_FUNC);

  while(people_.size()==0)
    {
      ROS_WARN(" Waitin people_ information to load, %d ",(int)people_.size());
      ros::spinOnce();
    }

  if(people_.size()>1){
  while(relations_.size()==0  )
    {
      ROS_WARN(" Waitin relations_ information to load, %d", (int)relations_.size());
      ros::spinOnce();
    }
  }
    ROS_WARN(" Not Waitin people_ and relations_ information to load, %d %d",(int)people_.size(), (int)relations_.size());
    // Locking CallBacks!!
    lock_callbacks=1;
  }
 
  /// Saving Relations
  for (size_t i = 0; i < relations_.size(); i++)
    {
            Array2I rel = relations_[i];
            geometry_msgs::Point pt1, pt2;
            

            Array4D p1 =map_people_[rel[0]];
            Array4D p2 =map_people_[rel[1]];
            // Array4D p1 = people_[rel[0]-1];
            // Array4D p2 = people_[rel[1]-1];

            pt1.x = p1[0]*world_width_; pt1.y = p1[1]*world_height_; pt1.z = 0.0;
            pt2.x = p2[0]*world_width_; pt2.y = p2[1]*world_height_; pt2.z = 0.0;
            ROS_INFO("[%f, %f, %f, %f]",pt1.x, pt1.y,pt2.x,pt2.y);


    }



}

template < class typeparams, int NUM_DIMENSIONS >
void smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::callbackTrackedPersons(const pedsim_msgs::TrackedPersons::ConstPtr &msg)
{

    // Read Only before all the people and relations have been loaded once!!!
    if(lock_callbacks==0){
    ROS_INFO("Reading TrackedPersons");

      people_.clear();

      /// Copying in the map
      BOOST_FOREACH(pedsim_msgs::TrackedPerson p, msg->tracks){
        
        Array4D pd = {{p.pose.pose.position.x, p.pose.pose.position.y,
              p.twist.twist.linear.x, p.twist.twist.linear.y}};
        
        map_people_.insert(std::pair<int,Array4D>(p.track_id,pd));

      }


      /// Copying in the vector

      BOOST_FOREACH(pedsim_msgs::TrackedPerson p, msg->tracks)
      {
          Array4D pd = {{p.pose.pose.position.x, p.pose.pose.position.y,
              p.twist.twist.linear.x, p.twist.twist.linear.y}};
          people_.push_back(pd);

      }

      ROS_INFO("TrackedPersons Size %d", (int)people_.size());
   
    ROS_INFO("STORING the TRACKS!!");
    received_tracks_= *msg;

   }


}





template < class typeparams, int NUM_DIMENSIONS >
void smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::callbackTrackedGroups(const pedsim_msgs::TrackedGroups::ConstPtr &msg)
{
  // Read Only before all the people and relations have been loaded once!!!
  if(lock_callbacks==0){

    ROS_INFO("Reading TrackedGroups");
    std::vector<std::vector<int> > associations;
    relations_.clear();

    BOOST_FOREACH(pedsim_msgs::TrackedGroup g, msg->groups)
    {
        std::set<PairInt> groupings;

        BOOST_FOREACH(int id, g.track_ids)
        {
            BOOST_FOREACH(int idx, g.track_ids)
            {
                if (id == idx)
                    continue;

                auto result = groupings.insert(PairInt(id, idx));

                // true if isertion succesful (the link does not already exist)
                if (result.second)
                {
                    Array2I r = {{id, idx}};
                    relations_.push_back(r);
                }
            }
        }
    }


    ROS_INFO("TrackedGroups Size %d", (int)relations_.size());


    if(COST_DISPLAY){

        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < relations_.size(); i++)
        {
            Array2I rel = relations_[i];
            // relations_.push_back(rel);
            // ROS_INFO("%d, %d, %d, %d, [%d, %d]", i, j, connection[i+1], connection[j+i+1], rel[0], rel[1]);
            // ROS_INFO("[%d, %d]", rel[0], rel[1]);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.type = visualization_msgs::Marker::LINE_LIST;


            geometry_msgs::Point pt1, pt2;
             // Array4D p1 = people_[rel[0]-1];
            // Array4D p2 = people_[rel[1]-1];

            Array4D p1 = map_people_[rel[0]];
            Array4D p2 = map_people_[rel[1]];
            pt1.x = p1[0]*world_width_; pt1.y = p1[1]*world_height_; pt1.z = 0.0;
            pt2.x = p2[0]*world_width_; pt2.y = p2[1]*world_height_; pt2.z = 0.0;


            relations_file_<<pt1.x<<" "<<pt1.y<<" "<<pt2.x<<" "<<pt2.y<<" ";

            marker.points.push_back(pt1);
            marker.points.push_back(pt2);

            marker_array.markers.push_back(marker);

        }



        pub_relations_.publish(marker_array);

    }


  ROS_INFO("STORING the GROUPS!!");
  received_groups_=*msg;

  }     


}


/// -----------------------------------------------------------
/// \brief Process RRT actions by computing IRL costs on them
/// and visualize the costs using markers
/// -----------------------------------------------------------
template < class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::computeIRLcost(std::vector<Array4D> act)
{

    // std::vector<double> features = computeTrajectoryFeatures(act, people_, relations_, goal_, 0.99, 0.35);
    std::vector<double> features = computeTrajectoryFeatures(act, map_people_, relations_, goal_, 1, 0.04);
    double action_cost = 0.0;

    if (reward_type_ == "polite")
        action_cost = computeActionCost(features, polite_);
    else if (reward_type_ == "rude")
        action_cost = computeActionCost(features, rude_);
    else if (reward_type_ == "friendly")
        action_cost = computeActionCost(features, friendly_);
    else {
        ROS_WARN("Invalid reward type");
        return 0;
    }

    return action_cost;
}


template < class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::set_goal_region (const region_t &region_in) {


  region_goal = region_in;
  ROS_INFO("Setting Regions %f %f",region_goal.center[0], region_goal.center[1]);

  Array4D g = {{region_goal.center[0], region_goal.center[1],0,0}};
  goal_ = g;

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
      cout << "COST -- : " << vertex_in->data.total_cost << endl;
      /// saving the cost
      cost= vertex_in->data.total_cost;
      update_trajectory = true;
      cntUpdates++;
    }


    if ( (vertex_in->data.total_cost < min_cost_vertex->data.total_cost) ) {
      cout << "COST -- : " << vertex_in->data.total_cost << endl;
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

  ROS_INFO("Publishing GROUPS and PERSONS");
  pub_tracked_persons_.publish(received_tracks_);
  pub_tracked_groups_.publish(received_groups_);


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

  // Setting Lock_callbacks to Zero!!!
  lock_callbacks=0;


  return 1;
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
  double old_x,old_y,new_x,new_y,s,sw,vmax, old_theta, new_theta;
  vmax=2;
  old_x=0;
  old_y=0;
  new_x=0;
  new_y=0;
  cnt=0;
  /// Initial cost introduced as discount factor to penilize multi steps trajectories
  total_cost+=initialCost_;


  if(this->SELECT_FUNC==0){



    //bla


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
          s=0.5*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+0.5*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)));

          total_cost+=s;
          old_y=new_y;
          old_x=new_x;
          old_theta=new_theta;
        }
        cnt++;

      }

  }



  if(this->SELECT_FUNC==1){


    double xin, yin, thin, xout, yout, thout;


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




    total_cost=regression_nlm( xin, yin, thin, xout, yout, thout);





  }





  if(this->SELECT_FUNC==2){


        double xin, yin, thin, xout, yout, thout;


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



    total_cost=regression_nn( xin, yin, thin, xout, yout, thout);



  }




  if(this->SELECT_FUNC==3){

     double xin, yin, thin, xout, yout, thout;


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

       total_cost=0.5*sqrt((xout-xin)*(xout-xin)+(yout-yin)*(yout-yin))+0.5*fabs(1-cos(diff_angle_unwrap(thout,thin)));

  }


  // compute cost using IRL
  if(this->SELECT_FUNC==4){

    std::vector<Array4D> traj_curr;
    traj_curr.clear();

    double smooth_cost=0;

    for (typename list<state_t*>::iterator iter=trajectory_in->list_states.begin();
          iter!= trajectory_in->list_states.end();iter++){

            state_t *state_curr=*iter;
            Array4D pt = {{(*state_curr)[0], (*state_curr)[1],0,0}};
            traj_curr.push_back(pt);

            if(cnt==0){
              old_x=(*state_curr)[0];
              old_y=(*state_curr)[1];
              old_theta=(*state_curr)[2];
              // Initial value
              smooth_cost+=0.1;
            }
            else{

              new_x=(*state_curr)[0];
              new_y=(*state_curr)[1];
              new_theta=(*state_curr)[2];
        //      s=sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))/vmax;
              s=0.5*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+0.5*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)));

              smooth_cost+=s;
              old_y=new_y;
              old_x=new_x;
              old_theta=new_theta;
            }
            cnt++;

    }
    
    if(traj_curr.size()>0)
      total_cost=computeIRLcost(traj_curr);
    else
      total_cost=0;



  }


  // compute cost using IRL with a scalar and smoothness
  if(this->SELECT_FUNC==5){

    std::vector<Array4D> traj_curr;
    traj_curr.clear();

    double smooth_cost=0;

    for (typename list<state_t*>::iterator iter=trajectory_in->list_states.begin();
          iter!= trajectory_in->list_states.end();iter++){

            state_t *state_curr=*iter;
            Array4D pt = {{(*state_curr)[0], (*state_curr)[1],0,0}};
            traj_curr.push_back(pt);

            if(cnt==0){
              old_x=(*state_curr)[0];
              old_y=(*state_curr)[1];
              old_theta=(*state_curr)[2];
              // Initial value
              smooth_cost+=0.1;
            }
            else{

              new_x=(*state_curr)[0];
              new_y=(*state_curr)[1];
              new_theta=(*state_curr)[2];
        //      s=sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))/vmax;
              s=0.5*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+0.5*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)));

              smooth_cost+=s;
              old_y=new_y;
              old_x=new_x;
              old_theta=new_theta;
            }
            cnt++;

    }
    if(traj_curr.size()>0){
      total_cost=computeIRLcost(traj_curr)/SCALING_IRL+smooth_cost/SCALING_SMOOTHCOST;
      // ROS_INFO("IRLCost: %f",computeIRLcost(traj_curr)/SCALING_IRL);
      // ROS_INFO("Pl+Smoothness Cost: %f",smooth_cost/SCALING_SMOOTHCOST);
    }
    else
      total_cost=0;



  }

  return total_cost;

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


template< class typeparams, int NUM_DIMENSIONS>
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::regression_nlm(double xin,double yin,double thin,double xout,double yout,double thout){

    double feat_in[NFEATURES];
    // vector of paramas
    double p[NPARAMS];

   /* Computing features

+     1. x2-x1;
+     2. y2-y1;
+     3. dangle;
+     4. eucl_dist ;
+     5. cos(dangle);
+     6. sin(dangle);
+     7. dangle.*eucl_dist;
+     8. cos(dangle).*eucl_dist;
+     9. sin(dangle).*eucl_dist;
+     10. diffangle(atan2(y2-y1,x2-x1),th1);
+     11. diffangle(atan2(y2-y1,x2-x1),th2);
+     12. diffangle(atan2(y2-y1,x2-x1),th1)./diffangle(atan2(y2-y1,x2-x1),th2);
+     13. diffangle(atan2(y2-y1,x2-x1),th1).*eucl_dist;
+     14. diffangle(atan2(y2-y1,x2-x1),th2).*eucl_dist;

+    */

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




/// method to compute the cost by using the learned function
template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS>
::regression_nn(double xin,double yin,double thin,double xout,double yout,double thout){

double feat_in[NFEATURES];

/* Computing features

 1. x2-x1;
 2. y2-y1;
 3. dangle;
 4. eucl_dist ;
 5. cos(dangle);
 6. sin(dangle);
 7. dangle.*eucl_dist;
 8. cos(dangle).*eucl_dist;
 9. sin(dangle).*eucl_dist;
 10. diffangle(atan2(y2-y1,x2-x1),th1);
 11. diffangle(atan2(y2-y1,x2-x1),th2);
 12. diffangle(atan2(y2-y1,x2-x1),th1)./diffangle(atan2(y2-y1,x2-x1),th2);
 13. diffangle(atan2(y2-y1,x2-x1),th1).*eucl_dist;
 14. diffangle(atan2(y2-y1,x2-x1),th2).*eucl_dist;

*/



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

/*



Now apply regression
*/

double res;
res=costotogo(feat_in);

return res;


}


#endif
