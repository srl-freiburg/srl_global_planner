#include <srl_global_planner/srl_global_planner.h>
#include <pluginlib/class_list_macros.h>


#define TESTM 0



using namespace smp;

using namespace std;

using namespace base_local_planner;


//extern Config config;
tf::TransformListener* listener;

ros::Rate *loop_rate;


namespace srl_global_planner {


/// ==================================================================================
/// Srl_global_planner::transformPose(geometry_msgs::PoseStamped init_pose)
/// Transforms the init_pose in the planner_frame
/// ==================================================================================
geometry_msgs::PoseStamped Srl_global_planner::transformPose(geometry_msgs::PoseStamped init_pose){

    geometry_msgs::PoseStamped res;

    ROS_DEBUG("Transform Pose in RRT, in Frame %s", planner_frame_.c_str());

    tf::StampedTransform transform;

    try{

        // will transform data in the goal_frame into the planner_frame_
        listener->waitForTransform( planner_frame_, init_pose.header.frame_id, ros::Time::now(), ros::Duration(0.20));
        listener->lookupTransform(  planner_frame_,  init_pose.header.frame_id, ros::Time::now(), transform);

    }
    catch(tf::TransformException){

        ROS_ERROR("Failed to transform the given pose in the RRT Planner frame_id");
        return init_pose;
    }

    tf::Pose source;

    tf::Quaternion q= tf::createQuaternionFromRPY(0,0,tf::getYaw(init_pose.pose.orientation));

    tf::Matrix3x3 base(q);

    source.setOrigin(tf::Vector3(init_pose.pose.position.x, init_pose.pose.position.y, 0));

    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = transform*source;



    res.pose.position.x = result.getOrigin().x() ;
    res.pose.position.y = result.getOrigin().y() ;
    res.pose.position.z = result.getOrigin().z() ;

    tf::quaternionTFToMsg(result.getRotation(), res.pose.orientation);

    res.header = init_pose.header;

    res.header.frame_id = planner_frame_;

    return res;

}


/// ==================================================================================
/// setGoal(double x, double y, double theta,double toll)
/// Method to store the Goal region description into the instance of the class
/// ==================================================================================
void Srl_global_planner::setGoal(double x, double y, double theta, double toll, std::string goal_frame){

  ROS_DEBUG("Setting Goal in RRT, Goal Frame %s", goal_frame.c_str());

    tf::StampedTransform transform;

    try{

        // will transform data in the goal_frame into the planner_frame_
        listener->waitForTransform( planner_frame_, goal_frame, ros::Time::now(), ros::Duration(0.20));

        listener->lookupTransform( planner_frame_, goal_frame, ros::Time::now(), transform);

    }
    catch(tf::TransformException){

        ROS_ERROR("Failed to transform Gaol Transform in RRT Planner");
        return;
    }

    tf::Pose source;

    tf::Quaternion q = tf::createQuaternionFromRPY(0,0, theta);

    tf::Matrix3x3 base(q);

    source.setOrigin(tf::Vector3(x, y, 0));

    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = transform*source;


    this->goal_theta_= tf::getYaw( result.getRotation());
    this->goal_x_ = result.getOrigin().x();
    this->goal_y_ = result.getOrigin().y();
    this->toll_goal_ = toll;
    this->goal_init_ = true;

    ROS_DEBUG("GOAL SET!!");
    ROS_DEBUG("GOAL SET!!");
    ROS_DEBUG("GOAL SET!!");

    goal_pose_.position.x = result.getOrigin().x();
    goal_pose_.position.y = result.getOrigin().y();
    goal_pose_.position.z = result.getOrigin().z();

    /// if goal was set the first time keep it as previous goal pose
    if(initialize_previous_goal_ == 0){

      previous_goal_pose_.position.x = goal_pose_.position.x;
      previous_goal_pose_.position.y = goal_pose_.position.y;
      previous_goal_pose_.position.z = goal_pose_.position.z;
      initialize_previous_goal_ ++;
      last_path_length_ = 0;

    }else{

      /// if previous goal is different from the new one, reset the last path
      /// generated
      if( previous_goal_pose_.position.y != goal_pose_.position.y &&
                previous_goal_pose_.position.y != goal_pose_.position.y){

                  ROS_DEBUG("New goal, resetting path length");
                  last_path_length_ = 0;

      }


    }

    tf::quaternionTFToMsg(result.getRotation(), goal_pose_.orientation);

    this->goal_init_=true;

}

/// ==================================================================================
/// publishBlockedPlanner()
/// Method to publish the status of the planner, if blocked by agents
/// ==================================================================================
void Srl_global_planner::publishBlockedPlanner(bool status){

  std_msgs::Bool d;
  d.data = status;

  pub_planner_blocked_.publish(d);

}
/// ==================================================================================
/// publishTree()()
/// Method to publish the current Tree
/// ==================================================================================

void Srl_global_planner::publishTree(){

    visualization_msgs::Marker tree_marker_;


    tree_marker_.header.frame_id = planner_frame_;
    tree_marker_.header.stamp = ros::Time();
    tree_marker_.ns = "rrt_planner";
    tree_marker_.id = 1;

    tree_marker_.type = visualization_msgs::Marker::POINTS;
    tree_marker_.color.a = 1;
    tree_marker_.color.r = 0.0;
    tree_marker_.color.g = 0.0;
    tree_marker_.color.b = 1.0;

    tree_marker_.scale.x = 0.1;
    tree_marker_.scale.y = 0.1;
    tree_marker_.scale.z = 0.1;


    tree_marker_.action = 0;  // add or modify


    for (vector<Tpoint>::const_iterator iter =this->tree_.begin(); iter !=this->tree_.end(); ++iter) {
        Tpoint a = (*iter);
        geometry_msgs::Point p;
        p.x = a.x;
        p.y = a.y;
        p.z = 0.5;

        tree_marker_.points.push_back(p);

    }




    pub_tree_.publish(tree_marker_);


    visualization_msgs::Marker dedicated_tree_marker_;



    /// pub_tree_dedicated_ publishes a tree on a topic showed by a separate rviz session

    dedicated_tree_marker_.header.frame_id = planner_frame_;
    dedicated_tree_marker_.header.stamp = ros::Time();
    dedicated_tree_marker_.ns = "rrt_planner";
    dedicated_tree_marker_.id = 1;

    dedicated_tree_marker_.type = visualization_msgs::Marker::POINTS;
    dedicated_tree_marker_.color.a = 1;
    dedicated_tree_marker_.color.r = 1.0;
    dedicated_tree_marker_.color.g = 0.0;
    dedicated_tree_marker_.color.b = 0.0;

    dedicated_tree_marker_.scale.x = 0.1;
    dedicated_tree_marker_.scale.y = 0.1;
    dedicated_tree_marker_.scale.z = 0.1;


    dedicated_tree_marker_.action = 0;  // add or modify


    for (vector<Tpoint>::const_iterator iter =this->tree_.begin(); iter !=this->tree_.end(); ++iter) {
        Tpoint a = (*iter);
        geometry_msgs::Point p;
        p.x = a.x;
        p.y = a.y;
        p.z = 0.05;

        dedicated_tree_marker_.points.push_back(p);

    }

    pub_tree_dedicated_.publish(dedicated_tree_marker_);


}

/// ==================================================================================
/// publishSample()()
/// Method to publish the current sample
/// ==================================================================================
void Srl_global_planner::publishSample(double x,double y, double theta, int ident){

    visualization_msgs::Marker sample_marker_;
    sample_marker_.header.frame_id = planner_frame_;
    sample_marker_.header.stamp = ros::Time::now();
    sample_marker_.ns = "Srl_global_planner";
    sample_marker_.id = ident;


    sample_marker_.type = visualization_msgs::Marker::ARROW;
    sample_marker_.color.a = 1;
    sample_marker_.color.r = 1.0;
    sample_marker_.color.g = 0.10;
    sample_marker_.color.b = 0.10;

    sample_marker_.scale.x = 1;
    sample_marker_.scale.y = 0.1;
    sample_marker_.scale.z = 0.01;


    sample_marker_.action = 0;  // add or modify

    sample_marker_.pose.position.x = x;
    sample_marker_.pose.position.y = y;
    sample_marker_.pose.position.z = 0; /// needed and
    sample_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);




    pub_samples_.publish(sample_marker_);
}


/// ==================================================================================
/// publishGoal()
/// Method to publish the Goal Marker
/// ==================================================================================

void Srl_global_planner::publishGoal(){

    visualization_msgs::Marker goal_marker_;

    goal_marker_.header.frame_id = planner_frame_;
    goal_marker_.header.stamp = ros::Time();
    goal_marker_.ns = "Srl_global_planner";
    goal_marker_.id = 1;

    goal_marker_.type = visualization_msgs::Marker::CUBE;
    goal_marker_.color.a = 0.20;
    goal_marker_.color.r = 1.0;
    goal_marker_.color.g = 0.10;
    goal_marker_.color.b = 0.10;

    goal_marker_.scale.x = this->toll_goal_;
    goal_marker_.scale.y = this->toll_goal_;
    goal_marker_.scale.z = 0.01;


    goal_marker_.action = 0;  // add or modify

    goal_marker_.pose.position.x =  this->goal_x_;
    goal_marker_.pose.position.y =  this->goal_y_;
    goal_marker_.pose.position.z = 0; /// needed and

    goal_marker_.pose.orientation.x = 0;
    goal_marker_.pose.orientation.y = 0;
    goal_marker_.pose.orientation.z = 0;
    goal_marker_.pose.orientation.w = 0;


    pub_goal_.publish(goal_marker_);


}



/// ==================================================================================
/// checkFrontLaserCollisionStatus(const CollisionStatus& collisionStatus)
/// Method to set the global Goal
/// ==================================================================================
void Srl_global_planner::checkFrontLaserCollisionStatus(const CollisionStatus::ConstPtr& msg) {

    collision_error_    = msg->collisionError;
    collision_warning_  = msg->collisionWarning;
    ROS_DEBUG("Srl Global Planner checking collision status, Error: %d, Warning: %d", collision_error_, collision_warning_);

}

/// ==================================================================================
/// checkRearLaserCollisionStatus(const CollisionStatus& collisionStatus)
/// Method to set the global Goal
/// ==================================================================================
void Srl_global_planner::checkRearLaserCollisionStatus(const CollisionStatus::ConstPtr& msg){

    collision_error_    = msg->collisionError;
    collision_warning_  = msg->collisionWarning;
    ROS_DEBUG("Srl Global Planner checking collision status, Error: %d, Warning: %d", collision_error_, collision_warning_);
}



/// ========================================================================================
/// SetDrivingDirection
/// Set The correct Driving Direction
/// ========================================================================================
bool Srl_global_planner::SetDrivingDirection(spencer_nav_msgs::SetDrivingDirection::Request &req,spencer_nav_msgs::SetDrivingDirection::Response &res){

  if(req.backward){

    ROS_WARN("Backward direction set");
    dir_planning_=-1*fabs(dir_planning_);


  }else
  {

    ROS_WARN("Forward  direction set");
    dir_planning_=1*fabs(dir_planning_);

  }

  return true;


}
/// ==================================================================================
/// callbackSetGoal(const geometry_msgs::PointStamped& msg)
/// Method to set the new Goal
/// TODO: Currently not used
/// ==================================================================================

void Srl_global_planner::callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){


}


/// ==================================================================================
/// callbackSetRobotPose(const nav_msgs::Odometry::ConstPtr& msg)
/// Method to set odom not needed
/// TODO: Currently not used
/// ==================================================================================

void Srl_global_planner::callbackSetRobotPose(const nav_msgs::Odometry::ConstPtr& msg){



}



/// ==================================================================================
/// csetGlobalPathSupport
/// Global Support used in RRT
/// ==================================================================================
bool Srl_global_planner::setGlobalPathSupport( std::vector< geometry_msgs::PoseStamped > plan){

    support_bias_->clear_delete();
    support_->clear_delete();

    ROS_DEBUG("Srl_global_planner setGlobalPathSupport");

    size_t dim;

    dim = plan.size();

    double t=0;

    double old_x,old_y,old_th;

    ROS_DEBUG("Srl_global_planner setGlobalPathSupport dim path %d", (int)dim);

    for (size_t i=0; i<dim; i++){


        geometry_msgs::PoseStamped posei = transformPose(plan[i]);

        double th = tf::getYaw(plan[i].pose.orientation);
        double x = plan[i].pose.position.x;
        double y = plan[i].pose.position.y;


        // double th = tf::getYaw(posei.pose.orientation);
        // double x = posei.pose.position.x;
        // double y = posei.pose.position.y;


        state_t *state_i = new state_t;

        (*state_i)[0]=x;
        (*state_i)[1]=y;
        (*state_i)[2]=th;

        ROS_DEBUG("Reading points >> %f %f %f", x, y, th);
        ROS_DEBUG("Reading points transformed >> %f %f %f", posei.pose.position.x, posei.pose.position.y, tf::getYaw(posei.pose.orientation));

        support_->list_states.push_back (new state_t(*state_i));


        if(i==0){

        state_t *state_i = new state_t;

        (*state_i)[0]=x;
        (*state_i)[1]=y;
        (*state_i)[2]=th;

        support_bias_->list_states.push_back (new state_t(*state_i));

        old_x=x;
        old_y=y;
        old_th=th;

        }
        else{


            while(t<1){

                state_t *state_i = new state_t;

                (*state_i)[0]=(x-old_x)*t+old_x;
                (*state_i)[1]=(y-old_y)*t+old_y;
                (*state_i)[2]=(th-old_th)*t+old_th;
                support_bias_->list_states.push_back (new state_t(*state_i));
                t=t+0.01;

            }


            state_t *state_i = new state_t;

            (*state_i)[0]=x;
            (*state_i)[1]=y;
            (*state_i)[2]=th;

            support_bias_->list_states.push_back (new state_t(*state_i));

            old_x=x;
            old_y=y;
            old_th=th;
            t=0;

        }


    }


    return true;
}




/// ==================================================================================
/// callbackObstacles(const nav_msgs::GridCells::ConstPtr& msg)
/// callback to read the obstacles positions
/// TODO: Currently not used
/// ==================================================================================
void Srl_global_planner::callbackObstacles(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    if(cnt_map_readings==0){

        begin=ros::Time::now();
        initialization_time=begin.toSec();
        map_loading_time=initialization_time;
    }else{

        map_loading_time=ros::Time::now().toSec();
    }

    // TODO: if you want
    if((map_loading_time-initialization_time)>max_map_loading_time && obstacle_positions.size()!=0 && cnt_map_readings<0){

        if(DEB_RRT)
        ROS_DEBUG("Obstacles loaded");

        ROS_ERROR("Stop Loading Map!!!");
        return;
    }


    /// Read transform planner_frame_ <-- Map
    ROS_WARN("Trying to read transform from %s to %s ", planner_frame_.c_str(), msg->header.frame_id.c_str() );
    tf::StampedTransform transform;
    try{
        ROS_WARN("Trying to read transform from %s to %s ", planner_frame_.c_str(), msg->header.frame_id.c_str() );
        listener->waitForTransform(planner_frame_, msg->header.frame_id,msg->header.stamp, ros::Duration(0.20));
        listener->lookupTransform(planner_frame_, msg->header.frame_id,msg->header.stamp, transform);
        costmap_frame_ = msg->header.frame_id;
    }
    catch(tf::TransformException){

        ROS_ERROR(" Global Planner -- > Failed to transform obstacles");
        return;
    }
    /// Read the obstacles in the messagge and save them in the vector obstacle_positions in the map frame

    obstacle_positions.clear();

   if(DEB_RRT)
   ROS_DEBUG("Reading Obstacles positions..");

    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    if(DEB_RRT)
    ROS_DEBUG("Got map %d %d", info.width, info.height);

    visualization_msgs::Marker obstacle_marker_;
    obstacle_marker_.header.frame_id = planner_frame_;
    obstacle_marker_.header.stamp = ros::Time::now();
    obstacle_marker_.ns = "Srl_global_planner";
    obstacle_marker_.id = 0;

    // set the state space
    center_map_x_ = info.origin.position.x + info.width/2;

    center_map_y_ = info.origin.position.y + info.height/2;

    height_map_ = info.height ;

    width_map_ = info.width ;
    ROS_DEBUG("Reading Map (%f, %f, %f, %f)..", center_map_x_, center_map_x_, height_map_, width_map_);

    int occup_prob, n_occupied;
    n_occupied=0;
    double xo, yo;
    double min_prob=0.1;
    for (unsigned int x = 0; x < info.width; x++)
        for (unsigned int y = 0; y < info.height; y++){
            /// acces the data in the map, in row-major order
                       occup_prob=msg->data[x+ info.width * y];
                       if(occup_prob>min_prob){


                            xo= (x+0.5)*info.resolution+info.origin.position.x;
                            yo= (y+0.5)*info.resolution+info.origin.position.y;


                            /// Transform the point of the obstacle from map to odom frames
                            tf::Pose source;
                            source.setOrigin(tf::Vector3(xo,yo,0));
                            tf::Matrix3x3 identity;
                            identity.setIdentity();
                            source.setBasis(identity);
                            /// Apply the proper transform
                            tf::Pose result=transform*source;
                            obstacle_positions.push_back(Tobstacle(result.getOrigin().x(),result.getOrigin().y(),0,info.resolution,info.resolution));

                            n_occupied++;




        }

    }



    cnt_map_readings++;

    if(DEB_RRT)
    ROS_DEBUG("N_obstacles %d",n_occupied);

}

/// ==================================================================================
/// callbackAllAgents(const pedsim_msgs::AllAgentsState::ConstPtr& msg)
/// callback to read the agents positions
/// TODO: Currently not used
/// ==================================================================================
void Srl_global_planner::callbackAllTracks(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg){

    /// Read Robot pose
    double x_curr = 0;
    double y_curr = 0;
    double theta_curr = 0;

    tf::StampedTransform transform;
    try{
        listener->lookupTransform("odom", "base_footprint", ros::Time(0.0), transform);
    }
    catch (tf::TransformException ex){
            ROS_ERROR("Didn't read robot pose via trasform listener in callbackAllTracks - RRT %s",ex.what());
            ros::Duration(1.0).sleep();
            return;
    }


    x_curr = transform.getOrigin().x();
    y_curr = transform.getOrigin().y();
    theta_curr = tf::getYaw(transform.getRotation());
    if(dir_planning_ == -1){
        theta_curr = set_angle_to_range(M_PI+theta_curr,0);
    }
    else{
        theta_curr = set_angle_to_range(theta_curr,0);
    }

    cnt_agents_in_wall_ = 0;

    /// Reading Tracking Poses
    agents_position.clear();

    double curr_or=0;
    double x = 0;
    double y = 0;

    double vy = 0;
    double vx = 0;

    double x_oriented = 0;
    double y_oriented = 0;
    double theta_oriented = 0;
    double vx_oriented = 0;
    double vy_oriented = 0;

    tf::StampedTransform  transformH;
    try{

        // will transform data in the tracker_frame_ into the planner_frame_
       // listener->waitForTransform( planner_frame_, msg->header.frame_id, ros::Time::now(), ros::Duration(0.20));

        //listener->lookupTransform( planner_frame_, msg->header.frame_id, ros::Time::now(), transformH);
        listener->lookupTransform("odom", msg->header.frame_id, ros::Time(0.0), transformH);


    }
    catch(tf::TransformException){

        ROS_ERROR("Read agents in RRT, Failed to transform  from Planner frame %s to tracker frame %s", planner_frame_.c_str(), msg->header.frame_id.c_str() );
        return;
    }


    for(size_t i=0; i < msg->tracks.size(); i++){

            vy = msg->tracks[i].twist.twist.linear.y;
            vx = msg->tracks[i].twist.twist.linear.x;
            curr_or=atan2(vy,vx);
            x = msg->tracks[i].pose.pose.position.x;
            y = msg->tracks[i].pose.pose.position.y;

            agents_position.push_back(Thuman(x, y, theta_oriented,msg->tracks[i].track_id, 0));


            double dist_human_i = sqrt((x-x_curr)*(x-x_curr) + (y-y_curr)*(y-y_curr));


            if(dist_human_i < dist_wall_){

             // ROS_WARN("Agents i distance %f ", dist_human_i);
              cnt_agents_in_wall_++;

            }
     }


//     if(cnt_agents_in_wall_ > 0)
  //      ROS_DEBUG("Agents blocking the robot %d ", cnt_agents_in_wall_);




}



/// ==================================================================================
/// publishNoPlan
/// ==================================================================================
void Srl_global_planner::publishNoPlan(int res){

    std_msgs::Bool noPlan;

    if(res==1){
        noPlan.data=true;
    }
    else{
        noPlan.data=false;
    }

    pub_no_plan_.publish(noPlan);


}

/// ==================================================================================
/// publishPlan(std::vector<geometry_msgs::PoseStamped>& plan)
/// publish Final plan
/// ==================================================================================
void Srl_global_planner::publishPlan(std::vector<geometry_msgs::PoseStamped>& plan){

  /// To IMPLEMENT
      nav_msgs::Path path_;
      ROS_DEBUG("Publishing a path");



      path_.header.frame_id = planner_frame_;
      path_.header.stamp = ros::Time();
      path_.poses.resize((int)plan.size());


      for (size_t i = 0; i < plan.size(); i++) {
          geometry_msgs::PoseStamped posei;
          path_.poses[i].header.stamp = ros::Time();
          path_.poses[i].header.frame_id = plan[i].header.frame_id;
          path_.poses[i].pose = plan[i].pose;

      }

      if(plan.size()>0){

          pub_plan_.publish(path_);
          ROS_DEBUG("Plan Published");

      }

}

/// ==================================================================================
/// publishPath(Trajectory *t)
/// method to publish the trajectory t
/// ==================================================================================
void Srl_global_planner::publishPath(Trajectory *t){

/// To IMPLEMENT
    nav_msgs::Path path_;
    ROS_DEBUG("Publishing a path");

    std::vector<Tpoint> path = t->getPath();
    ROS_DEBUG("Path Size :%d",(int)path.size());

    path_.header.frame_id = planner_frame_;
    path_.header.stamp = ros::Time();
    path_.poses.resize((int)path.size());

    visualization_msgs::Marker path_marker_;
    path_marker_.header.frame_id = planner_frame_;
    path_marker_.header.stamp = ros::Time();
    path_marker_.ns = "Srl_global_planner";
    path_marker_.id = 1;

    path_marker_.type = visualization_msgs::Marker::POINTS;
    path_marker_.color.a = 1;
    path_marker_.color.r = 0.0;
    path_marker_.color.g = 1.0;
    path_marker_.color.b = 0.0;

    path_marker_.scale.x = 0.5;
    path_marker_.scale.y = 0.5;
    path_marker_.scale.z = 0.5;


    path_marker_.action = 0;  // add or modify


    for (size_t i = 0; i < path.size(); i++) {


        geometry_msgs::PoseStamped posei;

        path_.poses[i].header.stamp = ros::Time();
        path_.poses[i].header.frame_id = planner_frame_;


        path_.poses[i].pose.position.x = path[i].x;
        path_.poses[i].pose.position.y = path[i].y;
        path_.poses[i].pose.position.z = 0;


        path_.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);


        /// Path_marker
        geometry_msgs::Point p;
        p.x = path[i].x;
        p.y = path[i].y;
        p.z = 0.05;
        path_marker_.points.push_back(p);


    }

    if(path.size()>0){

        pub_path_.publish(path_);
        pub_path_dedicated_.publish(path_marker_);

        ROS_DEBUG("Path Published");

    }
}




/// ==================================================================================
/// saveTree(list <vertex_t *> *list_vertices)
/// method to save for the tree
/// =================================================================================
int Srl_global_planner::saveTree(list <vertex_t *> *list_vertices){
/// WARNING THE TRAJECTORY OBJECT NEED TO BE DEFINED as TREE

  //  construct the graph and save all the trajectories samples and the vertice
        this->tree_.clear();

        int i = 0;
        int num_edges = 0;
        for (typename list<vertex_t *>::iterator iter = list_vertices->begin();
             iter != list_vertices->end(); iter++) {

            vertex_t *vertex_curr = *iter;
            state_t &state_ref = *(vertex_curr->state);

            i++;
            num_edges += (*iter)->incoming_edges.size();//
        }

        i = 0;

        // file stream to save the number of cartesian points for each edge
        ofstream ind;
        ind.open("ind.txt",std::fstream::in | std::fstream::out | std::fstream::app);

        for (typename list<vertex_t *>::iterator it_vertex = list_vertices->begin();
             it_vertex != list_vertices->end(); it_vertex++) {

            vertex_t *vertex_curr = *it_vertex;

            list< edge_t* > *incoming_edges_curr = &(vertex_curr->incoming_edges);
            for (typename list<edge_t*>::iterator it_edge = incoming_edges_curr->begin();
                 it_edge != incoming_edges_curr->end(); it_edge++) {

                edge_t *edge_curr = *it_edge;

                // ROS_DEBUG("New Edge from (%f,%f,%f) ",edge_curr->vertex_src->state->state_vars[0],edge_curr->vertex_src->state->state_vars[1],edge_curr->vertex_src->state->state_vars[2]);
                // ROS_DEBUG("to (%f,%f,%f)",edge_curr->vertex_dst->state->state_vars[0],edge_curr->vertex_dst->state->state_vars[1],edge_curr->vertex_dst->state->state_vars[2]);

                list<state_t*> *list_states_curr = &(edge_curr->trajectory_edge->list_states);

                if (list_states_curr->size () == 0) {
                    {
                        ind<<list_states_curr->size ()<<endl;
                        this->nexpasions_.push_back(list_states_curr->size ());
                    }


                }
                else {


                    {
                        ind<<list_states_curr->size()<<endl;
                        this->nexpasions_.push_back(list_states_curr->size ());
                    }

                    int j = 0;
                    for (typename list<state_t*>::iterator it_state = list_states_curr->begin();
                         it_state != list_states_curr->end(); it_state++) {

                        state_t *state_traj_curr = *it_state;
                        state_t &state_traj_ref = *state_traj_curr;
                        this->tree_.push_back(Tpoint(state_traj_ref[0],state_traj_ref[1],state_traj_ref[2]));

                        j++;

                    }
                }

                i++;
            }
        }


        ind.close();



}


/// ==================================================================================
/// set_angle_to_range(double alpha, double min)
/// wrap the angle
/// ==================================================================================
double Srl_global_planner::set_angle_to_range(double alpha, double min)
{

    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}



/// ==================================================================================
/// plan(Trajectory *traj,int type)
/// method to solve a planning probleme.
/// ==================================================================================

int Srl_global_planner::plan(Trajectory *traj,int type, geometry_msgs::PoseStamped& start){
    // TODO: could use dyn param
    nh_.getParam("/move_base_node/planner_frequency", this->planner_frequency_);

    if(DEB_RRT>0)
    ROS_DEBUG("Startin to plan!!!");


    // RRT* section
    // for each movement clear the obstacle list and refill it with the static ones and the agents
    // 1. CREATE PLANNING OBJECTS
    //cout<<"Debug: Initiliaze components";
    // 1.a Create the components
    sampler_t sampler;
    // State, input, vertex_data, and edge_data definitions
    distance_evaluator_t distance_evaluator;

    extender_t extender;
    collision_checker_t collision_checker;

    collision_checker.size_robot = robot_length_+collision_boundary*2;
    collision_checker.LEVEL_OBSTACLE_ = this->LEVEL_OBSTACLE_;
    /// IF RECTANGULAR COLLISION CHECKER
    // collision_checker.setParam(K,RAD_OBST,PARALLEL);
    // collision_checker.setRobotDim(robot_width_,robot_length_,collision_boundary);

    extender.L_axis=this->L_AXIS;
    // smoothness_cost_t min_smoothness;
    min_time_reachability_t min_time_reachability;

    min_time_reachability.file_name=cost_file_name;

    branch_and_bound_t branch_and_bound;



    rrtstar_t planner (sampler,distance_evaluator,extender,collision_checker,
                          min_time_reachability,min_time_reachability);

    collision_checker.initialize(world_model_, footprint_spec_, inscribed_radius_, circumscribed_radius_, costmap_ros_, planner_frame_, listener);

    min_time_reachability.initWorldModel(world_model_, footprint_spec_, inscribed_radius_, circumscribed_radius_, costmap_ros_, planner_frame_, listener);

    distance_evaluator.set_list_vertices(&(planner.list_vertices));

    if(DEB_RRT>0)
    ROS_DEBUG("Planner created");
        //added for single_integrator_extender
    //    extender.set_max_length(2);
    double side;
    int multiplier_side=sqrt(5);
    //considering a square inscribed in a circle, let's calculate a radius and then multiplie it for 2 (conservative way)
    side=(sqrt((goal_x_-rx)*(goal_x_-rx)+(goal_y_-ry)*(goal_y_-ry)))*multiplier_side;

    if(DEB_RRT>0)
    ROS_DEBUG("Planner setting parameters");

    planner.parameters.set_phase (type);   // The phase parameter can be used to run the algorithm as an RRT,
    // See the documentation of the RRG algorithm for more information.
    if(type==0){

            planner.parameters.set_fixed_radius(RADIUS);

    }
    planner.BOX=0;

    planner.parameters.set_gamma (side);    // Set this parameter should be set at least to the side length of
    //   the (bounded) state space. E.g., if the state space is a box
    //   with side length L, then this parameter should be set to at
    //   least L for rapid and efficient convergence in trajectory space.
    planner.parameters.set_dimension (3);
    planner.parameters.set_max_radius (2*side);  // This parameter should be set to a high enough value. In practice,
    //   one can use smaller values of this parameter to get a good
    //   solution quickly, while preserving the asymptotic optimality.

    if(RADIUS>0)
        planner.parameters.set_fixed_radius(RADIUS);


    /// setting RHO end condition
    planner.RHO=this->RHO;
    planner.DT=this->DT;

    if(DEB_RRT>0)
    ROS_DEBUG("Planner setting learning and support");

    planner.LEARNED=this->LEARNED;
    planner.FINDNEAREST=this->FINDNEAREST;
    planner.SRCPARENT=this->SRCPARENT;
    // planner.updatepathsupport(support_);

    if(DEB_RRT>0)
    ROS_DEBUG("setting cost evaluator");

    /// setting cost evaluator
    min_time_reachability.NOTLEARNED=this->NOTLEARNED;
    min_time_reachability.MODEL_COST=this->MODEL_COST;
    min_time_reachability.ONLYTHETACOST=this->ONLYTHETACOST;

    if(DEB_RRT>0)
    ROS_DEBUG("setting cost evaluator - update_trajectory");

    min_time_reachability.update_trajectory(support_);
    min_time_reachability.Kd=this->Kd;
    min_time_reachability.Kangle=this->Kangle;
    // min_time_reachability.set_goal_region (region_goal);
    min_time_reachability.Kdist = this->Kdist;
    min_time_reachability.Kth = this->Kth;
    min_time_reachability.Kor = this->Kor;
    min_time_reachability.ADD_COST_THETASTAR = this->ADD_COST_THETASTAR;
    min_time_reachability.ADD_COST_FROM_COSTMAP = this->ADD_COST_FROM_COSTMAP;
    min_time_reachability.ADD_COST_PATHLENGTH = this->ADD_COST_PATHLENGTH;

    min_time_reachability.n_dis_traj = this->n_dis_traj;


    /// Reading current robot pose from cost map
    if(DEB_RRT>0)
    ROS_DEBUG("Reading current robot pose from cost map");


    rx = start.pose.position.x;
    ry = start.pose.position.y;
    rz = tf::getYaw(start.pose.orientation);
    rz = set_angle_to_range(rz,0);

    //! 2 Set the Sampler Support
    if(DEB_RRT>0)
    ROS_DEBUG("Setting Sampler");

    /// For a Rectangle as Sampling Support
    region<3> sampler_support;
    double ws = (goal_x_- rx)*(goal_x_- rx) + (goal_y_- ry)*(goal_y_- ry);


    if(TYPE_SAMPLING == 4){

      sampler_support.center[0] = rx;
      sampler_support.center[1] = rx;
      sampler_support.center[2] = goal_theta_;
      sampler_support.size[0] = ws*2 ;
      sampler_support.size[1] = ws*2 ;
      sampler_support.size[2] = 2*M_PI;
      ROS_DEBUG("Size sampling support %f", 2*ws);

    }else{
      sampler_support.center[0] = rx;
      sampler_support.center[1] = rx;
      sampler_support.center[2] = goal_theta_;
      sampler_support.size[0] = width_map_ ;
      sampler_support.size[1] = height_map_ ;
      sampler_support.size[2] = 2*M_PI;
   }


    /// Not Uniform
    sampler.set_support(sampler_support);
    sampler.AVERAGING=this->AVERAGING;
    sampler.OR_RANGE=this->OR_RANGE;
    sampler.LMAX=this->LMAX;

    if(DEB_RRT>0)
    ROS_DEBUG("Local Planner -->  SupportCenter, SupportSize: (%f ,%f, %f) - (%f ,%f, %f)",sampler_support.center[0],sampler_support.center[1],sampler_support.center[2],sampler_support.size[0],sampler_support.size[1],sampler_support.size[2]);


    if(DEB_RRT>0)
    ROS_DEBUG("Define type of sampling unit");

    sampler.use_type(TYPE_SAMPLING);

    // In case of biasing
    region<3> region_goal_sup;

    region_goal_sup.center[0] = goal_x_ ;
    region_goal_sup.center[1] = goal_y_ ;
    region_goal_sup.center[2] = goal_theta_ ;

    region_goal_sup.size[0] = toll_goal_;
    region_goal_sup.size[1] = toll_goal_;
    region_goal_sup.size[2] = this->GB_ORIENT_;
    sampler.set_goal(region_goal_sup);
    sampler.set_goal_biasing(GOAL_BIASING);
    sampler.set_goal_biasing_ths(GOAL_BIASING_THS);


    if(DEB_RRT>0)
    ROS_DEBUG("Set Type Sampling");

    if(TYPE_SAMPLING==1){
        sampler.setsigmas(sigmaxi_,sigmayi_);
        sampler.use_extsigmas(EXT_SIGMAS);
            /// Insert the path obtained by ThetaStar
       if(DEB_RRT>0)
       ROS_DEBUG("Importing Theta* path into the sampling unit");

        sampler.update_trajectory(support_);
        sampler.Kround=this->Kround;
    }
    else if(TYPE_SAMPLING==0 || TYPE_SAMPLING==3 ) {

        sampler.set_width_strip(width_strip_);
            /// Insert the path obtained by ThetaStar
        if(DEB_RRT>0)
        ROS_DEBUG("Importing Theta* path into the sampling unit");

        sampler.update_trajectory(support_);
        sampler.Kround=this->Kround;

    }else if(TYPE_SAMPLING==5){

    sampler.update_trajectory_bias(support_bias_);
    sampler.set_bias_probability(BIAS_PROB);
    sampler.set_sample_dispersion(DISPERSION);

    }

if(DEB_RRT>0){
    ROS_DEBUG_STREAM("Sampler support center...."<< sampler_support.center[0]<< " "<<sampler_support.center[1]<< " "<< sampler_support.center[2]<< " ");
    ROS_DEBUG_STREAM("Sampler Sides :"<< sampler_support.size[0]<<" " <<sampler_support.size[1]<<" " <<sampler_support.size[2]);

}

if(DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: collision checker");

if( DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: Add Pedestrians");

    // 3.b Initialize the model checker and the cost evaluator
if (DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: goal region");

    region<2> region_goal;
    region_goal.center[0] =goal_x_ ;
    region_goal.center[1] =goal_y_ ;
    region_goal.size[0] = toll_goal_/2;
    region_goal.size[1] = toll_goal_/2;

    min_time_reachability.set_goal_region (region_goal);

if(DEB_RRT>0)
    ROS_DEBUG_STREAM("Goal Region: x:"<<region_goal.center[0]<<" y:"<<region_goal.center[1]);

if(DEB_RRT>0)
ROS_DEBUG("Goal Region: x: %f y: %f",region_goal.center[0],region_goal.center[1]);


    // Set branch and bound util
    branch_and_bound.set_planner (&planner);
    branch_and_bound.set_goal_region (region_goal);
    branch_and_bound.set_root_vertex (planner.get_root_vertex());
    planner.WHATTOSHOW=WHATTOSHOW;

if(DEB_RRT>0)
    ROS_DEBUG("Loading Robot Pose");

    // 3.c Initialize the planner to commanded Agent position ** TODO ** We can get the initial position
    state_t *state_initial = new state_t;

    state_initial->state_vars[0] = rx;
    state_initial->state_vars[1] = ry;
    state_initial->state_vars[2] = rz;

    if( DEB_RRT>0)
    ROS_DEBUG_STREAM("Robot position-> x:"<<state_initial->state_vars[0]<<" y:"<<state_initial->state_vars[1]<<" z:"<<state_initial->state_vars[2]);

    planner.initialize (state_initial);

    if( DEB_RRT>0)
      ROS_DEBUG_STREAM("Debug: RRT* iterations");

    min_time_reachability.begin_time=clock();
    float timep=0;
    double rw=0;

    int iterations=1;
    int it=0;

    double begin_time_ext_ros;
    begin_time_ext_ros=ros::WallTime::now().toSec();

    double begin_time_solution,end_time_solution;
    begin_time_solution=ros::WallTime::now().toSec();

    double curr_MAXTIME;
    double curr_NUM_UPDATES;
    {

        // curr_MAXTIME = MAXTIME;
        curr_MAXTIME = 1/planner_frequency_ - 0.1;
        ROS_DEBUG("current curr_MAXTIME %f ", curr_MAXTIME);
        curr_NUM_UPDATES = NUMBER_UPDATE_TRAJ;
    }



    if(TIMECOUNTER == 1){

    while(timep<curr_MAXTIME && min_time_reachability.cntUpdates<curr_NUM_UPDATES ){


        if(timep>curr_MAXTIME)
            break;

        begin_time_ext_ros=ros::WallTime::now().toSec();
        planner.iteration ();
        timep+=  ros::WallTime::now().toSec() - begin_time_ext_ros;
        it++;

        publishSample(planner.statex,planner.statey,planner.statetheta,it);
        iterations=it;

        rw+=planner.nrewiring;
        if (it%100000 == 0) {

            if(DEB_RRT>0)
              ROS_DEBUG("Seconds: %f", timep);

            iterations=it;

        }


        if(FIRSTSOLUTION>0){

          if(min_time_reachability.foundTraj==true){

            curr_cost_=min_time_reachability.cost;
            if(first_sol==0){
                end_time_solution=ros::WallTime::now().toSec() -begin_time_solution;

                if( DEB_RRT>0);
                ROS_DEBUG("First Solution after Seconds: %f", end_time_solution);

                first_sol++;
             }


            }
        }
        else
        {

             if(min_time_reachability.foundTraj==true){

                curr_cost_=min_time_reachability.cost;
                }

        }

    }
}

else {
    for (int i = 0; i < Nit_; i++){
        begin_time_ext_ros=ros::WallTime::now().toSec();
        planner.iteration ();
        timep+=  ros::WallTime::now().toSec() - begin_time_ext_ros;
        it++;

        publishSample(planner.statex,planner.statey,planner.statetheta,it);
        rw+=planner.nrewiring;

            iterations=i;

        if (i%100 == 0) {

          if(DEB_RRT>0)
            ROS_DEBUG("pLANNER Iteration: %d" , i);
            iterations=i;

        }
       double min_cost = min_time_reachability.get_best_cost ();

       if(BRANCHBOUND){

           if (min_cost>THRS_BRANCHBOUND && (i%BRANCHBOUND_RATIO == 0)) {

             if(DEB_RRT>0)
                  ROS_DEBUG("Min Cost %f, Running branch_and_bound",min_cost );

                  branch_and_bound.set_upper_bound_cost (min_cost);
                  branch_and_bound.run_branch_and_bound ();
           }

       }

        if(FIRSTSOLUTION>0){
        /// If solution is found then stop to iterate
            if(min_time_reachability.foundTraj==true){
                curr_cost_=min_time_reachability.cost;
                end_time_solution=ros::WallTime::now().toSec() -begin_time_solution;

        // if(min_smoothness.foundTraj==true){
            break;
            }
        }

    }
}


    if(FIRSTSOLUTION==0){
        end_time_solution=ros::WallTime::now().toSec()-begin_time_solution;

        if( DEB_RRT>0)
          ROS_DEBUG("%d Solution after Seconds: %f",NUMBER_UPDATE_TRAJ, end_time_solution);
    }


    nrew_=rw/iterations;
    timeIter_=timep/iterations;

    trajectory_t trajectory_final;

    if(DEB_RRT>0)
      ROS_DEBUG( "Try to Get Solution " );

    min_time_reachability.get_solution (trajectory_final);



    /// IF NO TRAJECTORY SAVE THE TREE
    if( trajectory_final.list_states.size()==0){
        typedef rrtstar_t planner_t;

    if(!NOANIM)
        publishTree();


        planner_t *planner_int;
        planner_int =&planner;
        list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
        /// Metrics (timeIter_ saved before)
        numVertices_=list_vertices->size();
        timeSolution_=end_time_solution;
        return 0;

    }


    float x,y,theta;
    traj->reset();
    ///take the last point of the trajectory and connect it to the goal.
    state_t *start_state = trajectory_final.list_states.back();
    double b_local_planner_ = 0.1;
    state_t *goal_state = new state_t;
    (goal_state)->state_vars[0] = goal_x_+ b_local_planner_*cos(goal_theta_);
    (goal_state)->state_vars[1] = goal_y_+ b_local_planner_*sin(goal_theta_);
    (goal_state)->state_vars[2] = goal_theta_;

    if(length_path_optimize_<0)
      planner.connect_goal(start_state, goal_state, &trajectory_final);


    for (typename list<state_t*>::iterator it_state = trajectory_final.list_states.begin();
         it_state != trajectory_final.list_states.end(); it_state++){
        state_t *state_curr = *it_state;
        x = (state_curr)->state_vars[0];
        y = (state_curr)->state_vars[1];
        theta = (state_curr)->state_vars[2];
        traj->addPointEnd(Tpoint(x,y,theta));

    }


    /// Publish the obtained path
    if(!NOANIM)
        publishPath(traj);



    double v,w ;




    for (typename list<input_t*>::iterator it_input = trajectory_final.list_inputs.begin();
         it_input != trajectory_final.list_inputs.end(); it_input++){
        input_t *input_curr = *it_input;
        v=(input_curr)->input_vars[0];
        w=(input_curr)->input_vars[1];
        traj->addVelocities(v,w);

    }




    typedef rrtstar_t planner_t;
    planner_t *planner_int;
    planner_int =&planner;


    list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
    numVertices_=list_vertices->size();
    timeSolution_=end_time_solution;



    if(!NOANIM)
        publishTree();

    return 1;

}


bool Srl_global_planner::generatePlanOptimizeLocally(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){


            this->trajectory_->reset();

            ros::spinOnce();

            nh_.getParam("TYPE_SAMPLING",this->TYPE_SAMPLING);
            nh_.getParam("GOAL_BIASING",this->GOAL_BIASING);
            nh_.getParam("type_planner", this->TYPEPLANNER);

            geometry_msgs::PoseStamped start_pose = start;

            // if backward motion add pi to the current orientation
            if(dir_planning_ == -1){

              start_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0,  M_PI + tf::getYaw(start.pose.orientation)  );
            }

            int status;

            if(this->initialized_){


              if(collision_error_){
                  ROS_ERROR("RRT* could not generate a path, collision error on");
                  return false;

              }


              ROS_DEBUG("Setting Goal (%f,%f,%f) and Start (%f, %f, %f)",(double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation),
                            start_pose.pose.position.x, start_pose.pose.position.y, tf::getYaw(start_pose.pose.orientation));

                            ROS_DEBUG("Start pose not flipped (%f,%f,%f)", start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));

                //status = system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflater enabled false");

                this->setGoal( (double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation), toll_goal_ , goal.header.frame_id );
                /// Grid Planning
                /// To check the frame.. Reading in Odom Frame ...
                if(!grid_planner_->setStart(start_pose.pose.position.x, start_pose.pose.position.y, tf::getYaw(start_pose.pose.orientation), start_pose.header.frame_id ))
                  return false;

                if(!grid_planner_->setGoal(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation), goal.header.frame_id  ))
                  return false;

                ros::spinOnce();
                // loop_rate->sleep();

                std::vector< geometry_msgs::PoseStamped > grid_plan;
                std::vector< geometry_msgs::PoseStamped > support_grid_plan;
                std::vector< geometry_msgs::PoseStamped > remaining_grid_plan;
                int index_goal_path_to_optimize = 0;

                grid_plan.clear();

                if(TYPE_SAMPLING!=4){

                    /// try several time to find a global path with grid search
                    while( cnt_run_grid_planner_ <  20 &&  (int)grid_plan.size() == 0){

                      if(!grid_planner_->run_grid_planner(grid_plan)){

                      }else{

                        break;
                      }

                      cnt_run_grid_planner_ = cnt_run_grid_planner_+1;

                   }

                    cnt_run_grid_planner_ = 0;

                    if((int)grid_plan.size()==1){

                        ROS_WARN("DISCRETE PATH OF SINGLE CELL, STAY HERE");
                        return false;
                    }

                    if((int)grid_plan.size()>1){

                      if(return_only_discrete_path_){
                        ROS_WARN("Giving back the grid path, size: %d", (int)grid_plan.size());
                        for (size_t i = 0; i < grid_plan.size(); i++) {
                          geometry_msgs::PoseStamped posei;
                          posei.header.seq = cnt_make_plan_;
                          posei.header.stamp = ros::Time::now();
                          posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                          posei.pose.position.x = grid_plan[i].pose.position.x;
                          posei.pose.position.y = grid_plan[i].pose.position.y;
                          posei.pose.position.z = 0;
                          posei.pose.orientation = grid_plan[i].pose.orientation;
                          plan.push_back(posei);

                       }

                       return true;

                      }

                      double current_grid_path_length = 0;

                      for (size_t i = 0; i < grid_plan.size(); i++) {

                        geometry_msgs::PoseStamped posei;
                        posei.header.seq = cnt_make_plan_;
                        posei.header.stamp = ros::Time::now();
                        posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                        posei.pose.position.x = grid_plan[i].pose.position.x;
                        posei.pose.position.y = grid_plan[i].pose.position.y;
                        posei.pose.position.z = 0;
                        posei.pose.orientation = grid_plan[i].pose.orientation;


                        if(i>0){
                          double dx = grid_plan[i].pose.position.x - grid_plan[i-1].pose.position.x;
                          double dy = grid_plan[i].pose.position.y - grid_plan[i-1].pose.position.y;
                          double ds = sqrt(dx*dx+dy*dy);
                          current_grid_path_length += ds;
                        }
                        if(length_path_optimize_> current_grid_path_length)
                          {
                            /// adding poses to the support for the sampling unit
                            support_grid_plan.push_back(posei);
                            /// update the goal pose
                            this->setGoal( (double)posei.pose.position.x, (double)posei.pose.position.y, (double)tf::getYaw(posei.pose.orientation), 0.2 , posei.header.frame_id );
                            /// keep memory to which index we need to start to copy the optimized path
                            index_goal_path_to_optimize = i;
                          }
                        else{
                            remaining_grid_plan.push_back(posei);

                        }

                     }
                      // ROS_INFO("Setting support, current grid plan size %d, current remain path size %d", support_grid_plan.size(), remaining_grid_plan.size());
                      // ROS_INFO("current_grid_path_length %f, length path to optimize %f", current_grid_path_length, length_path_optimize_);

                      setGlobalPathSupport(support_grid_plan);

                    }else
                    {
                      ROS_WARN("NO PATH FOUND FROM THE GRID PLANNER");
                      // return false;
                      TYPE_SAMPLING = 4; /// sample everywhere
                      GOAL_BIASING = 0; // NO Goal biasing
                      TYPEPLANNER = 1; /// use RRT*
                      ROS_WARN("Sample every where in the global costmap");

                    }

                  }


                geometry_msgs::PoseStamped s;
                s = transformPose(start_pose);
                n_agents_in_wall_planning_ = cnt_agents_in_wall_;

                if(this->plan(trajectory_, this->TYPEPLANNER, s)){


                                std::vector<Tpoint> path = trajectory_->getPath();

                                plan.clear();
                                cnt_no_plan_= 0;
                                cnt_make_plan_++ ;

                                double distIntialPathToDiscard = 0;
                                double oldx = 0;
                                double oldy = 0;


                                distToTheGoal_ = 0;
                                double oldxp = path[0].x;
                                double oldyp = path[0].y;
                                for (size_t i = 1; i < path.size(); i++) {

                                      double x_p = path[i].x;
                                      double y_p = path[i].y;

                                      distToTheGoal_ += sqrt( (x_p-oldxp)*(x_p-oldxp) + (y_p-oldyp)*(y_p-oldyp)  );
                                      oldxp = path[i].x;
                                      oldyp = path[i].y;
                                }

                                ROS_DEBUG("Path Length or path distance to the goal %f", distToTheGoal_);
                                ROS_DEBUG("Last path length %f, Thrs %f", last_path_length_, gain_path_length_ * last_path_length_);
                                ROS_DEBUG("Thrs on path length %f", thrs_initial_part_path_);
                                ROS_DEBUG("Number agents into wall %d", n_agents_in_wall_planning_);


                                last_path_length_ = distToTheGoal_;


                                // if the length of the path is higher than 0.5 so you can
                                // filter the glorbal path otherwise send it as it is
                                if ( distToTheGoal_>thrs_initial_part_path_){

                                  for (size_t i = 0; i < path.size(); i++) {


                                      if(i == 0){
                                        oldx = path[i].x;
                                        oldy = path[i].y;
                                        continue;
                                      }
                                      else{

                                        distIntialPathToDiscard += sqrt( (path[i].x-oldx)*(path[i].x-oldx) + (path[i].y-oldy )*(path[i].y-oldy ));
                                        oldx = path[i].x;
                                        oldy = path[i].y;

                                        if(distIntialPathToDiscard>thrs_initial_part_path_  ){

                                            ROS_DEBUG("Adding point %d to the distance %f", (int)i, distIntialPathToDiscard);

                                            geometry_msgs::PoseStamped posei;
                                            posei.header.seq = cnt_make_plan_;
                                            posei.header.stamp = ros::Time::now();
                                            posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                            posei.pose.position.x = path[i].x;
                                            posei.pose.position.y = path[i].y;
                                            posei.pose.position.z = 0;
                                            posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);

                                            plan.push_back(posei);
                                        }
                                     }
                                  }
                                }else{

                                  ROS_DEBUG("Not filtering!!");
                                  for (size_t i = 0; i < path.size(); i++) {

                                      geometry_msgs::PoseStamped posei;
                                      posei.header.seq = cnt_make_plan_;
                                      posei.header.stamp = ros::Time::now();
                                      posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                      posei.pose.position.x = path[i].x;
                                      posei.pose.position.y = path[i].y;
                                      posei.pose.position.z = 0;
                                      posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);
                                      plan.push_back(posei);

                                  }

                                }

                                ROS_WARN("RRT found a path.. ");
                                /// Adding last part of the grid path
                                for (size_t i = 0; i < remaining_grid_plan.size(); i++) {

                                    geometry_msgs::PoseStamped posei;
                                    posei.header.seq = cnt_make_plan_;
                                    posei.header.stamp = ros::Time::now();
                                    posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                    posei.pose.position.x =remaining_grid_plan[i].pose.position.x;
                                    posei.pose.position.y = remaining_grid_plan[i].pose.position.y;
                                    posei.pose.position.z = 0;
                                    posei.pose.orientation = remaining_grid_plan[i].pose.orientation;
                                    plan.push_back(posei);

                                }


                                return true;



                }else{
                                ROS_WARN("RRT did not find a path.. ");
                                cnt_no_plan_++;

                                /// Check if there are humans very close to the robot
                                /// which are blocking it!
                                if(n_agents_in_wall_planning_ > 0){

                                  /// contact the supervisor!!! need to unblock the planner!!
                                  ROS_DEBUG("Blocked Planner True, cnt agents in wall %d", n_agents_in_wall_planning_);

                                }
                                else
                                {

                                  ROS_DEBUG("Blocked Planner false, cnt agents in wall %d", n_agents_in_wall_planning_);
                                  grid_planner_->run_grid_planner(grid_plan);

                                 if( grid_plan.size()>0 && return_discrete_path_ == true){

                                    ROS_WARN("Giving back the grid path, size: %d", (int)grid_plan.size());
                                    for (size_t i = 0; i < grid_plan.size(); i++) {

                                      geometry_msgs::PoseStamped posei;
                                      posei.header.seq = cnt_make_plan_;
                                      posei.header.stamp = ros::Time::now();
                                      posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                      posei.pose.position.x = grid_plan[i].pose.position.x;
                                      posei.pose.position.y = grid_plan[i].pose.position.y;
                                      posei.pose.position.z = 0;
                                      posei.pose.orientation = grid_plan[i].pose.orientation;
                                      plan.push_back(posei);

                                   }


                                   return true;
                                 }

                                }

                                return false;

                }


            }else{

                return false;
            }


}


/// ==================================================================================
/// generatePlan()
/// ==================================================================================
bool Srl_global_planner::generatePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){



      // TODO -> Implement a replanning architecture!!!!
      this->trajectory_->reset();

      ros::spinOnce();

      nh_.getParam("TYPE_SAMPLING",this->TYPE_SAMPLING);
      nh_.getParam("GOAL_BIASING",this->GOAL_BIASING);
      nh_.getParam("type_planner", this->TYPEPLANNER);

      geometry_msgs::PoseStamped start_pose = start;

      // if backward motion add pi to the current orientation
      if(dir_planning_ == -1){

        start_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0,  M_PI + tf::getYaw(start.pose.orientation)  );
      }

      int status;

      if(this->initialized_){


        if(collision_error_){
            ROS_ERROR("RRT* could not generate a path, collision error on");
            return false;

        }


        ROS_DEBUG("Setting Goal (%f,%f,%f) and Start (%f, %f, %f)",(double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation),
                      start_pose.pose.position.x, start_pose.pose.position.y, tf::getYaw(start_pose.pose.orientation));

                      ROS_DEBUG("Start pose not flipped (%f,%f,%f)", start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));

          //status = system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflater enabled false");

          this->setGoal( (double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation), toll_goal_ , goal.header.frame_id );
          /// Grid Planning
          /// To check the frame.. Reading in Odom Frame ...
          if(!grid_planner_->setStart(start_pose.pose.position.x, start_pose.pose.position.y, tf::getYaw(start_pose.pose.orientation), start_pose.header.frame_id ))
            return false;

          if(!grid_planner_->setGoal(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation), goal.header.frame_id  ))
            return false;

          ros::spinOnce();
          // loop_rate->sleep();

          std::vector< geometry_msgs::PoseStamped > grid_plan;

          grid_plan.clear();

          if(TYPE_SAMPLING!=4){

              /// try several time to find a global path with grid search
              while( cnt_run_grid_planner_ <  20 &&  (int)grid_plan.size() == 0){

                if(!grid_planner_->run_grid_planner(grid_plan)){

                }else{

                  break;
                }

                cnt_run_grid_planner_ = cnt_run_grid_planner_+1;

             }

              cnt_run_grid_planner_ = 0;

              if((int)grid_plan.size()==1){

                  ROS_WARN("DISCRETE PATH OF SINGLE CELL, STAY HERE");
                  return false;
              }

              if((int)grid_plan.size()>1){

                if(return_only_discrete_path_){
                  ROS_WARN("Giving back the grid path, size: %d", (int)grid_plan.size());
                  for (size_t i = 0; i < grid_plan.size(); i++) {

                    geometry_msgs::PoseStamped posei;
                    posei.header.seq = cnt_make_plan_;
                    posei.header.stamp = ros::Time::now();
                    posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                    posei.pose.position.x = grid_plan[i].pose.position.x;
                    posei.pose.position.y = grid_plan[i].pose.position.y;
                    posei.pose.position.z = 0;
                    posei.pose.orientation = grid_plan[i].pose.orientation;
                    plan.push_back(posei);

                 }

                 ROS_DEBUG("Removin inflation layer into the global cost map");
                 ros::spinOnce();
                 return true;

                }

                setGlobalPathSupport(grid_plan);

              }else
              {
                ROS_WARN("NO PATH FOUND FROM THE GRID PLANNER");
                // return false;
                TYPE_SAMPLING = 4; /// sample everywhere
                GOAL_BIASING = 0; // NO Goal biasing
                TYPEPLANNER = 1; /// use RRT*
                ROS_WARN("Sample every where in the global costmap");

              }

            }


          geometry_msgs::PoseStamped s;
          s = transformPose(start_pose);
          ros::spinOnce();

          n_agents_in_wall_planning_ = cnt_agents_in_wall_;


          // if(n_agents_in_wall_planning_==0){
          //     publishBlockedPlanner(false);
          //     ros::spinOnce();
          //     // loop_rate->sleep();
          // }

          if(this->plan(trajectory_, this->TYPEPLANNER, s)){


                          std::vector<Tpoint> path = trajectory_->getPath();

                          plan.clear();
                          cnt_no_plan_= 0;
                          cnt_make_plan_++ ;

                          double distIntialPathToDiscard = 0;
                          double oldx = 0;
                          double oldy = 0;


                          distToTheGoal_ = 0;
                          double oldxp = path[0].x;
                          double oldyp = path[0].y;
                          for (size_t i = 1; i < path.size(); i++) {

                                double x_p = path[i].x;
                                double y_p = path[i].y;

                                distToTheGoal_ += sqrt( (x_p-oldxp)*(x_p-oldxp) + (y_p-oldyp)*(y_p-oldyp)  );
                                oldxp = path[i].x;
                                oldyp = path[i].y;
                          }

                          ROS_DEBUG("Path Length or path distance to the goal %f", distToTheGoal_);
                          ROS_DEBUG("Last path length %f, Thrs %f", last_path_length_, gain_path_length_ * last_path_length_);
                          ROS_DEBUG("Thrs on path length %f", thrs_initial_part_path_);
                          ROS_DEBUG("Number agents into wall %d", n_agents_in_wall_planning_);

                          /// Case in which we are blocked: there is a agents wall
                          /// and the path length is hi
                          // if( distToTheGoal_ > gain_path_length_ * last_path_length_  && n_agents_in_wall_planning_ > 0){
                          //
                          //   /// contact the supervisor!!! need to unblock the planner!!
                          //   publishBlockedPlanner(true);
                          //   ros::spinOnce();
                          //   // loop_rate->sleep();
                          //
                          // }else{
                          //
                          //   publishBlockedPlanner(false);
                          //   ros::spinOnce();
                          //   // loop_rate->sleep();
                          //
                          // }



                          last_path_length_ = distToTheGoal_;


                          // if the length of the path is higher than 0.5 so you can
                          // filter the glorbal path otherwise send it as it is
                          if ( distToTheGoal_>thrs_initial_part_path_){

                            for (size_t i = 0; i < path.size(); i++) {


                                if(i == 0){
                                  oldx = path[i].x;
                                  oldy = path[i].y;
                                  continue;
                                }
                                else{

                                  distIntialPathToDiscard += sqrt( (path[i].x-oldx)*(path[i].x-oldx) + (path[i].y-oldy )*(path[i].y-oldy ));
                                  oldx = path[i].x;
                                  oldy = path[i].y;

                                  if(distIntialPathToDiscard>thrs_initial_part_path_  ){

                                      ROS_DEBUG("Adding point %d to the distance %f", (int)i, distIntialPathToDiscard);

                                      geometry_msgs::PoseStamped posei;
                                      posei.header.seq = cnt_make_plan_;
                                      posei.header.stamp = ros::Time::now();
                                      posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                      posei.pose.position.x = path[i].x;
                                      posei.pose.position.y = path[i].y;
                                      posei.pose.position.z = 0;
                                      posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);

                                      plan.push_back(posei);
                                  }
                               }
                            }
                          }else{

                            ROS_DEBUG("Not filtering!!");
                            for (size_t i = 0; i < path.size(); i++) {

                                geometry_msgs::PoseStamped posei;
                                posei.header.seq = cnt_make_plan_;
                                posei.header.stamp = ros::Time::now();
                                posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                posei.pose.position.x = path[i].x;
                                posei.pose.position.y = path[i].y;
                                posei.pose.position.z = 0;
                                posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);



                                plan.push_back(posei);

                            }

                          }

                          ROS_WARN("RRT found a path.. ");
                          return true;



          }else{
                          ROS_WARN("RRT did not find a path.. ");
                          cnt_no_plan_++;

                          /// Check if there are humans very close to the robot
                          /// which are blocking it!
                          if(n_agents_in_wall_planning_ > 0){

                            /// contact the supervisor!!! need to unblock the planner!!
                            ROS_DEBUG("Blocked Planner True, cnt agents in wall %d", n_agents_in_wall_planning_);
                            // publishBlockedPlanner(true);
                            // ros::spinOnce();

                          }
                          else
                          {

                            ROS_DEBUG("Blocked Planner false, cnt agents in wall %d", n_agents_in_wall_planning_);
                            // publishBlockedPlanner(false);
                            // ros::spinOnce();


                            // if (ros::param::has("/move_base_node/global_costmap/inflater/enabled"))
                            // {
                            //   //status = system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflater enabled true");
                            //
                            //   ros::spinOnce();
                            //   ROS_DEBUG("Setting inflation layer into the global cost map");
                            // }else
                            // {
                            //   ROS_WARN("inflater enabled param not found");
                            // }

                            grid_planner_->run_grid_planner(grid_plan);


                           if( grid_plan.size()>0 && return_discrete_path_ == true){

                              ROS_WARN("Giving back the grid path, size: %d", (int)grid_plan.size());
                              for (size_t i = 0; i < grid_plan.size(); i++) {

                                geometry_msgs::PoseStamped posei;
                                posei.header.seq = cnt_make_plan_;
                                posei.header.stamp = ros::Time::now();
                                posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                                posei.pose.position.x = grid_plan[i].pose.position.x;
                                posei.pose.position.y = grid_plan[i].pose.position.y;
                                posei.pose.position.z = 0;
                                posei.pose.orientation = grid_plan[i].pose.orientation;
                                plan.push_back(posei);

                             }

                             ROS_DEBUG("Removin inflation layer into the global cost map");
                            // status = system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflater enabled false");
                             ros::spinOnce();

                             return true;
                           }

                          }
                          ros::spinOnce();
                          // loop_rate->sleep();
                          ROS_DEBUG("Removin inflation layer into the global cost map");
                          //status = system("rosrun dynamic_reconfigure dynparam set /move_base_node/global_costmap/inflater enabled false");
                          return false;

          }


      }else{

          return false;
      }




}



/// ==================================================================================
/// checkLayersAndPlan(), this method is meant to enable and disable the layers of the
/// costmap following this principles:
// - the global planner will initially try to generate a global-plan without the obstacle layer
// - if no global-plan was generated, lower the threshold for social layer.
// - if still no global-plan, remove the social layer.
// - if still no global-plan, static layer is causing problem, there's really no plan possible. return failure.
/// ==================================================================================
bool Srl_global_planner::checkLayersAndPlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){


      bool obstacle_layer_disabled = false;
      bool social_layer_range_reduced = false;
      bool social_layer_removed = false;


      // - the global planner will initially try to generate a global-plan without the obstacle layer
      if(costmap_layers_handler_->isObstacleLayerEnabled() && !obstacle_layer_off_ ){
        if(!costmap_layers_handler_->enableObstacleLayer(false))
        {
          ROS_ERROR("Could not plan due to issues during disabling of the obstacle layer");
          obstacle_layer_off_ = true;
          costmap_layers_handler_->enableObstacleLayer(true);
          return false;

        }
        obstacle_layer_disabled = true;
      }

      bool result_no_obstacle_layer = generatePlan(start, goal, plan);

      // if result was found reset obstacle layer, as original configuration was
      if(result_no_obstacle_layer){

        if(obstacle_layer_disabled){
          /// resetting obstacle layer, TODO use parameter to turn off obstacle layer check
          costmap_layers_handler_->enableObstacleLayer(true);
        }

        return true;
      }
      // - if no global-plan was generated, lower the threshold for social layer (say 5m).
      if(costmap_layers_handler_->isSocialLayerEnabled()){
        if(!costmap_layers_handler_->setSocialLayerMaxRange(reduced_social_range_)) //TODO: to be parametrized
        {
          ROS_ERROR("Could not plan due to issues during setting of max Range of Social layer");
          costmap_layers_handler_->setSocialLayerMaxRange(initial_social_range_);
          return false;

        }
        social_layer_range_reduced = true;
      }

      bool result_reduced_social_range = generatePlan(start, goal, plan);

      // if result was found reset obstacle layer and origina social layer range, as original configuration was
      if(result_reduced_social_range){

        if(obstacle_layer_disabled){
          costmap_layers_handler_->enableObstacleLayer(true);
        }

        if(social_layer_range_reduced)
          costmap_layers_handler_->setSocialLayerMaxRange(initial_social_range_);

        return true;

      }

      // - if still no global-plan, remove the social layer. now a global plan is available. In this case the local planner should trigger the slow mode. Keep measuring robot speed in this mode. if not moving for n-seconds, declare human wall. then supervisor asks for space.
      if(costmap_layers_handler_->isSocialLayerEnabled()){
        if(!costmap_layers_handler_->enableSocialLayer(false))
        {
          ROS_ERROR("Could not plan due to issues during disabling of the Social layer");
          costmap_layers_handler_->enableSocialLayer(true);
          return false;
        }
        social_layer_removed = true;
      }

      bool result_no_social_layer = generatePlan(start, goal, plan);

      // - if still no global-plan, static layer is causing problem, there's really no plan possible. return failure.
      // if result was found reset obstacle layer and  social layer, as original configuration was
      /// TODO: in this case need to notice the local planner that we removed the social compliant layer.. need to move slowly
      if(social_layer_removed){
        costmap_layers_handler_->enableSocialLayer(true);
      }

      if(obstacle_layer_disabled){
        costmap_layers_handler_->enableObstacleLayer(true);
      }

      if(social_layer_range_reduced)
        costmap_layers_handler_->setSocialLayerMaxRange(initial_social_range_);

      if(result_no_social_layer)
        return true;
      else
        return false;


}


/// ==================================================================================
/// makePlan()
/// ==================================================================================
bool Srl_global_planner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){


      if(CHECK_LAYERS_ON_){

        bool result_checkLayersAndPlanning = checkLayersAndPlan(start, goal, plan);
        publishPlan(plan);
        return result_checkLayersAndPlanning;
      }
      else{
          if(length_path_optimize_<0){
            bool result_planning = generatePlan(start, goal, plan);
            publishPlan(plan);
            return result_planning;
          }else{

            bool result_planning = generatePlanOptimizeLocally(start, goal, plan);
            publishPlan(plan);
            return result_planning;
          }
      }


}



/// ==================================================================================
/// initialize()
/// Method to initialize all the publishers and subscribers
/// ==================================================================================
void  Srl_global_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){


    if(!initialized_){

            // number of RRT* iterations

        // ** RRT* Section **
        // 1 Create the planner algorithm -- Note that the min_time_reachability variable acts both
        //                                       as a model checker and a cost evaluator.
        // cout<<"Debug: Initiliaze planner"<<endl;
        /// Need to be set
        node_name_ = name;
        this->initialized_ = false;
        this->goal_theta_=0;
        this->goal_x_=0;
        this->goal_y_=0;
        this->cellheight_=1.5;
        this->cellwidth_=1.5;
        this->xsupp_=0;
        this->ysupp_=0;
        this->timeIter_=0;
        this->nrew_=0;
        this->goal_init_=false;
        this->DISPLAY_METRICS=0;
        this->ADD_COST_PATHLENGTH=1;
        this->no_fails=0;
        this->curr_cost_=0;
        this->MAXTIME=0;
        this->NUMBER_UPDATE_TRAJ=0;
        this->NOANIM=0;
        this->first_sol=0;
        this->SELECT_FUNC=0;
        this->WHATTOSHOW=0;
        this->RHO=0.15;
        this->DT=0.1;
        this->inscribed_radius_ = 1;
        this->circumscribed_radius_ =1;
        this->max_map_loading_time=20;
        this->support_ = new trajectory_t();
        this->support_bias_ = new trajectory_t();
        trajectory_ = new Trajectory();
        this->cnt_make_plan_ = 0;
        this->cnt_no_plan_ = 0;
        this->thrs_initial_part_path_ = 0.5;
        this->cnt_run_grid_planner_ = 0;
        this->dir_planning_ = -1;
        this->dist_wall_ = 0.5;
        this->cnt_agents_in_wall_ = 0;
        this->last_path_length_ = 0;
        this->initialize_previous_goal_ = 0;
        this->gain_path_length_ = 1.6;
        this->n_agents_in_wall_planning_ = 0;
        this->return_discrete_path_ = true;
        this->return_only_discrete_path_ = false;
        this->collision_error_  = false;
        this->collision_warning_ = false;
        this->reduced_social_range_ = 4.5;
        this->CHECK_LAYERS_ON_ = false;
        this->obstacle_layer_off_ = false;
        this->global_costmap_height_ = 50;
        this->global_costmap_width_ = 50;
        ros::NodeHandle node("~/Srl_global_planner");
        nh_ =  node;

        ROS_DEBUG("Srl_global_planner ŝtart initializing");


        try {

        costmap_ros_ = costmap_ros;

        ROS_INFO("getting costmap from costmap_ros...");

        costmap_ = costmap_ros_->getCostmap();

        /// TODO: !! getting the transform listener from costmap2DROS... only in our spencer software
        ROS_INFO("getting listner from costmap...");

        listener = costmap_ros_->getTransformListener();

        ROS_INFO("creating world model...");
        world_model_ = new CostmapModel(*costmap_);


      }catch (exception& e)
      {
          ROS_ERROR("An exception occurred during creation of the costmap. Exception Nr. %s", e.what() );
      }

      try{
          ROS_INFO("returning footprint...");
          footprint_spec_.clear();
          // std::vector<geometry_msgs::Point> fp;
          //
          // fp = costmap_ros_->getUnpaddedRobotFootprint();


          footprint_spec_ = costmap_ros_->getRobotFootprint();

          // for(int i=0;i<fp.size();i++){
          //   footprint_spec_.push_back(fp[i]);
          // }
          if( (int)(footprint_spec_.size()) > 0)
              ROS_DEBUG("footprint_spec_ loaded with %d elements", (int)footprint_spec_.size());

          }
      catch (exception& e)
          {
            ROS_ERROR("An exception occurred during reading of the footprint. Exception Nr. %s", e.what() );
          }


        try{

             grid_planner_ = new Grid_planner(node, new CostmapModel(*costmap_) , footprint_spec_, costmap_ros_, listener);

              grid_planner_->initialize();

        }
        catch (exception& e)
        {
            ROS_ERROR("An exception occurred during creation of the global planner. Exception Nr. %s", e.what() );
        }


        /// all the ROS DATA
        // setup publishers
        pub_planner_blocked_ = nh_.advertise<std_msgs::Bool>("/spencer/navigation/planner_blocked",5);

        pub_path_ = nh_.advertise<nav_msgs::Path>("rrt_planner_path", 5);

        pub_plan_ = nh_.advertise<nav_msgs::Path>("global_plan", 5);

        pub_goal_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_goal",1000);

        pub_tree_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_tree",1000);

        pub_tree_dedicated_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_tree_dedicated",1000);

        pub_path_dedicated_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_path_dedicated",1000);

        pub_samples_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_samples",1000);

        // pub_graph_=nh_.advertise<pedsim_msgs::Tree>("/rrt_planner/graph",1);
        pub_no_plan_ = nh_.advertise<std_msgs::Bool>("rrt_planner/no_plan",1);

        // sub_obstacles_= nh_.subscribe("/move_base_node/global_costmap/costmap",1, &Srl_global_planner::callbackObstacles,this);

        // sub_all_agents_= nh_.subscribe("/spencer/perception/tracked_persons",1, &Srl_global_planner::callbackAllTracks,this);

        frontLaserCollisionStatus_listener_ = nh_.subscribe("/spencer/control/collision/aggregated_front", 1, &Srl_global_planner::checkFrontLaserCollisionStatus, this);

        rearLaserCollisionStatus_listener_  = nh_.subscribe("/spencer/control/collision/aggregated_rear", 1, &Srl_global_planner::checkRearLaserCollisionStatus, this);


        // subscriberss TODO!!!!
        sub_goal_=nh_.subscribe("/move_base_simple/goal",1,&Srl_global_planner::callbackSetGoal,this);


        sub_daryl_odom_= nh_.subscribe("/spencer/sensors/odom",5,&Srl_global_planner::callbackSetRobotPose,this);

        srv_drivingdir = nh_.advertiseService("/set_driving_direction", &Srl_global_planner::SetDrivingDirection,this);

        ROS_INFO("ROS publishers and subscribers initialized");


        /// Initializing number of read maps
        cnt_map_readings=0;

        loop_rate = new ros::Rate(2/DT);



        /// ==================
        /// READING PARAMETERS
        /// ==================
        /// define dim of scene
        double x1,x2, y1,y2, csx,csy;

        nh_.getParam("x1", this->xmin_);
        nh_.getParam("x2", this->xmax_);
        nh_.getParam("y1", this->ymin_);
        nh_.getParam("y2", this->ymax_);
        nh_.getParam("cell_size_x", csx);
        nh_.getParam("cell_size_y", csy);

        int tcount,firstsol,deburrt;
        nh_.getParam("TIMECOUNTER", this->TIMECOUNTER);
        nh_.getParam("FIRSTSOLUTION", this->FIRSTSOLUTION);
        nh_.getParam("DEB_RRT", this->DEB_RRT);
        nh_.getParam("BOX",this->BOX);
        nh_.getParam("WHATTOSHOW",this->WHATTOSHOW);
        nh_.getParam("RADIUS",this->RADIUS);
        nh_.getParam("RHO",this->RHO);
        nh_.getParam("DT",this->DT);
        nh_.getParam("L_AXIS",this->L_AXIS);
        nh_.getParam("THRS_BRANCHBOUND",this->THRS_BRANCHBOUND);
        nh_.getParam("BRANCHBOUND",this->BRANCHBOUND);
        nh_.getParam("BRANCHBOUND_RATIO",this->BRANCHBOUND_RATIO);
        nh_.getParam("GOAL_BIASING",this->GOAL_BIASING);
        nh_.getParam("GOAL_BIASING_THS",this->GOAL_BIASING_THS);
        nh_.getParam("SCALING_IRL",this->SCALING_IRL);
        nh_.getParam("PARALLEL",this->PARALLEL);
        nh_.getParam("K",this->K);
        nh_.getParam("RAD_OBST",this->RAD_OBST);
        nh_.getParam("ROBOT_LENGTH",this->robot_length_);
        nh_.getParam("ROBOT_WIDTH",this->robot_width_);
        nh_.getParam("READ_AGENTS",this->READ_AGENTS);
        nh_.getParam("file_name",this->cost_file_name);
        nh_.getParam("front_rear_error_distance",this->collision_boundary);
        nh_.getParam("max_map_loading_time",this->max_map_loading_time);
        nh_.getParam("agents_size",this->agents_size_);
        nh_.getParam("MAXTIME",this->MAXTIME);
        nh_.getParam("NUMBER_UPDATE_TRAJ",this->NUMBER_UPDATE_TRAJ);
        nh_.getParam("NOANIM",this->NOANIM);
        nh_.getParam("SELECT_FUNC",this->SELECT_FUNC);
        nh_.getParam("DISPLAY_METRICS",this->DISPLAY_METRICS);
        nh_.getParam("COST_DISPLAY",this->COST_DISPLAY);
        nh_.getParam("max_iterations", this->Nit_);
        nh_.getParam("inscribed_radius",this->inscribed_radius_);
        nh_.getParam("circumscribed_radius",this->circumscribed_radius_);
        nh_.getParam("TYPE_SAMPLING",this->TYPE_SAMPLING);
        nh_.getParam("LEARNED", this->LEARNED);
        nh_.getParam("FINDNEAREST", this->FINDNEAREST);
        nh_.getParam("NOTLEARNED", this->NOTLEARNED);
        nh_.getParam("ONLYTHETACOST", this->ONLYTHETACOST);
        nh_.getParam("OR_RANGE",this->OR_RANGE);
        nh_.getParam("AVERAGING",this->AVERAGING);
        nh_.getParam("Kd",this->Kd);
        nh_.getParam("Kangle",this->Kangle);
        nh_.getParam("Kdist",this->Kdist);
        nh_.getParam("Kor",this->Kor);
        nh_.getParam("WIDTH_STRIP",this->width_strip_);
        nh_.getParam("LMAX",this->LMAX);
        nh_.getParam("Kth",this->Kth);
        nh_.getParam("n_dis_traj",this->n_dis_traj);
        nh_.getParam("ADD_COST_THETASTAR",this->ADD_COST_THETASTAR);;
        nh_.getParam("Kround",this->Kround);
        nh_.getParam("MODEL_COST",this->MODEL_COST);
        nh_.getParam("BIAS_PROB",this->BIAS_PROB);
        nh_.getParam("DISPERSION",this->DISPERSION);
        nh_.getParam("type_planner", this->TYPEPLANNER);
        nh_.getParam("planner_frame",this->planner_frame_);
        nh_.getParam("ADD_COST_FROM_COSTMAP", this->ADD_COST_FROM_COSTMAP);
        nh_.getParam("LEVEL_COLLCHECKER_OBSTACLE_", this->LEVEL_OBSTACLE_);
        nh_.getParam("GOAL_BIASING_ORIENTATION_RANGE", this->GB_ORIENT_);
        nh_.getParam("ADD_COST_PATHLENGTH", this->ADD_COST_PATHLENGTH);
        nh_.getParam("thrs_initial_part_path", this->thrs_initial_part_path_);
        nh_.getParam("GOAL_TOLL", this->toll_goal_);
        nh_.getParam("dist_wall", this->dist_wall_);
        nh_.getParam("gain_path_length", this->gain_path_length_);
        nh_.getParam("return_discrete_path", this->return_discrete_path_);
        nh_.getParam("return_only_discrete_path", this->return_only_discrete_path_);
        nh_.getParam("reduced_social_range", this->reduced_social_range_);
        nh_.getParam("check_layers_on", this->CHECK_LAYERS_ON_);
        nh_.getParam("obstacle_layer_off", this->obstacle_layer_off_);
        nh_.getParam("global_costmap_width", this->global_costmap_width_);
        nh_.getParam("global_costmap_height", this->global_costmap_height_);
        nh_.getParam("length_path_optimize", this->length_path_optimize_);
        nh_.getParam("/move_base_node/planner_frequency", this->planner_frequency_);

        ROS_INFO("Is Check on costmap Layers ON? %d", this->CHECK_LAYERS_ON_ );
        /// store dim of scene
        this->xscene_=x2-x1;

        this->yscene_=y2-y1;
        /// store sizes
        this->cellwidth_=csx;

        this->cellheight_=csy;

        ROS_INFO("RRT_planner initialized");

        int size_footprint = (int)footprint_spec_.size();

        ROS_INFO("Current size of the footprint %d", size_footprint);

        this->costmap_layers_handler_ = new costmapLayersDynRecHandler(nh_);

        initial_social_range_ = costmap_layers_handler_->getSocialLayerMaxRange();

        // TODO: the global cost map service call is blocking.. not sure why!
        // ROS_INFO("setGlobalCostMapSize initialized");
        //
        // bool on_map_size = costmap_layers_handler_->setGlobalCostMapSize((int)global_costmap_width_, (int)global_costmap_height_);
        //
        // ROS_INFO("setGlobalCostMapSize finished");

        initialized_ = true;

    }else{


        ROS_WARN("RRT planner not initialized");
    }

}


}



 //register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(srl_global_planner::Srl_global_planner, nav_core::BaseGlobalPlanner);
