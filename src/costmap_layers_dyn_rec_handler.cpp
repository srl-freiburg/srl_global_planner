#include <srl_global_planner/costmap_layers_dyn_rec_handler.h>

using namespace std;




costmapLayersDynRecHandler::costmapLayersDynRecHandler(const ros::NodeHandle& node){

  nh_layers_ = node;

  obstacle_layer_enabled_ = false;
  inflater_layer_enabled_ = false;
  social_layer_maxrange_ = 8.0;
  social_layer_minrange_ = 1.0;

  // Readin initial status of the layers from the parameter server
  nh_layers_.getParam("/move_base_node/global_costmap/social_compliance/update_range", social_layer_maxrange_) ;
  nh_layers_.getParam("/move_base_node/global_costmap/social_compliance/min_range", social_layer_minrange_);
  nh_layers_.getParam("/move_base_node/global_costmap/social_compliance/enabled", social_layer_enabled_);
  nh_layers_.getParam("/move_base_node/global_costmap/obstacles/enabled", obstacle_layer_enabled_);
  nh_layers_.getParam("/move_base_node/global_costmap/inflater/enabled", inflater_layer_enabled_);
  // Showing info
  ROS_INFO("Costmap Layers Handler started, current parameters:");
  ROS_INFO("Social Layer enabled : %d ", social_layer_enabled_);
  ROS_INFO("Obstacle Layer enabled : %d ", obstacle_layer_enabled_);
  ROS_INFO("Inflater Layer enabled : %d ", inflater_layer_enabled_);
  ROS_INFO("Max Range of the social layer: %f", social_layer_maxrange_ );
  ROS_INFO("Min Range of the social layer: %f", social_layer_minrange_ );

}

costmapLayersDynRecHandler::~costmapLayersDynRecHandler(){

}

/// ==================================================================================
/// enableObstacleLayer()
/// ==================================================================================
bool costmapLayersDynRecHandler::enableObstacleLayer(bool enable){


  try{

      if(ros::service::exists("/move_base_node/global_costmap/obstacles/set_parameters",true)){

      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::BoolParameter bool_param;
      dynamic_reconfigure::Config conf;
      bool_param.name = "enabled";
      bool_param.value = enable;
      conf.bools.push_back(bool_param);
      srv_req.config = conf;
      ros::service::call("/move_base_node/global_costmap/obstacles/set_parameters", srv_req, srv_resp);
      if(!enable)
        ROS_WARN("Obstacle layer removed");
      else
        ROS_WARN("Obstacle layer added");
      boost::unique_lock<boost::mutex> lock(obstacle_layer_enabled_mutex_);
      obstacle_layer_enabled_ = enable;
      lock.unlock();
      return true;
    }else{
      ROS_ERROR("Obstacles set_parameters service not available");
      return false;
    }

  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in enableObstacleLayer. Exception Nr. %s", e.what() );
      return false;
  }



}



/// ==================================================================================
/// enableSocialLayer()
/// ==================================================================================
bool costmapLayersDynRecHandler::enableSocialLayer(bool enable){

  try{

      if(ros::service::exists("/move_base_node/global_costmap/social_compliance/set_parameters",true)){

      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::BoolParameter bool_param;
      dynamic_reconfigure::Config conf;
      bool_param.name = "enabled";
      bool_param.value = enable;
      conf.bools.push_back(bool_param);
      srv_req.config = conf;
      ros::service::call("/move_base_node/global_costmap/social_compliance/set_parameters", srv_req, srv_resp);
      if(!enable)
        ROS_WARN("Social layer removed");
      else
        ROS_WARN("Social layer added");
      boost::unique_lock<boost::mutex> lock(social_layer_enabled_mutex_);
      social_layer_enabled_ = enable;
      lock.unlock();
      return true;
    }else{
      ROS_ERROR("Social set_parameters service not available");
      return false;
    }
  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in enableSocialLayer. Exception Nr. %s", e.what() );
      return false;
  }

}




/// ==================================================================================
/// enableInflaterLayer()
/// ==================================================================================
bool costmapLayersDynRecHandler::enableInflaterLayer(bool enable){

  try{

    if(ros::service::exists("/move_base_node/global_costmap/inflater/set_parameters",true)){

      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::BoolParameter bool_param;
      dynamic_reconfigure::Config conf;
      bool_param.name = "enabled";
      bool_param.value = enable;
      conf.bools.push_back(bool_param);
      srv_req.config = conf;
      ros::service::call("/move_base_node/global_costmap/inflater/set_parameters", srv_req, srv_resp);
        if(!enable)
          ROS_WARN("Inflater layer removed");
        else
          ROS_WARN("Inflater layer added");
      boost::unique_lock<boost::mutex> lock(inflater_layer_enabled_mutex_);
      inflater_layer_enabled_ = enable;
      lock.unlock();
      return true;
    }else{
      ROS_ERROR("inflater set_parameters service not available");
      return false;
    }
  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in enableInflaterLayer. Exception Nr. %s", e.what() );
      return false;
  }
}



/// ============================================================================
/// setSocialLayerMaxRadius()
/// ============================================================================
bool costmapLayersDynRecHandler::setSocialLayerMaxRange(double new_range){

  try{

      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::DoubleParameter double_param;
      dynamic_reconfigure::Config conf;
      double_param.name = "update_range";
      double_param.value = new_range;
      conf.doubles.push_back(double_param);
      srv_req.config = conf;
      ros::service::call("/move_base_node/global_costmap/social_compliance/set_parameters", srv_req, srv_resp);
      ROS_WARN("Social layer --> changing update_range to %f", new_range);
      boost::unique_lock<boost::mutex> lock(social_layer_maxrange_mutex_);
      social_layer_maxrange_ = new_range;
      lock.unlock();
      return true;

  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in setSocialLayerMaxRange. Exception Nr. %s", e.what() );
      return false;
  }


}

/// ============================================================================
/// setGlobalCostMapSize(double width, double height)
/// ============================================================================
bool costmapLayersDynRecHandler::setGlobalCostMapSize(int width, int height){


  try {

      dynamic_reconfigure::ReconfigureRequest srv_req_width;
      dynamic_reconfigure::ReconfigureResponse srv_resp_width;
      dynamic_reconfigure::IntParameter int_param_width;
      dynamic_reconfigure::Config conf_width;
      int_param_width.name = "width";
      int_param_width.value = width;
      conf_width.ints.push_back(int_param_width);
      srv_req_width.config = conf_width;
      ROS_WARN("Global Cost Map --> Calling Service");

      ros::service::call("/move_base_node/global_costmap/set_parameters", srv_req_width, srv_resp_width);
      ROS_WARN("Global Cost Map --> changing width to %d", width);

      return true;
  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in setGlobalCostMapSize. Exception Nr. %s", e.what() );
      return false;
  }

  try {
      dynamic_reconfigure::ReconfigureRequest srv_req_height;
      dynamic_reconfigure::ReconfigureResponse srv_resp_height;
      dynamic_reconfigure::IntParameter int_param_height;
      dynamic_reconfigure::Config conf_height;
      int_param_height.name = "height";
      int_param_height.value = height;
      conf_height.ints.push_back(int_param_height);
      srv_req_height.config = conf_height;
      ROS_WARN("Global Cost Map --> Calling Service");

      ros::service::call("/move_base_node/global_costmap/set_parameters", srv_req_height, srv_resp_height);
      ROS_WARN("Global Cost Map --> changing height to  to %d", height);



      return true;
  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in setGlobalCostMapSize. Exception Nr. %s", e.what() );
      return false;
  }


}




/// ============================================================================
/// setSocialLayerMinRange()
/// ============================================================================
bool costmapLayersDynRecHandler::setSocialLayerMinRange(double new_range){

  try {
      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::DoubleParameter double_param;
      dynamic_reconfigure::Config conf;
      double_param.name = "min_range";
      double_param.value = new_range;
      conf.doubles.push_back(double_param);
      srv_req.config = conf;
      ros::service::call("/move_base_node/global_costmap/social_compliance/set_parameters", srv_req, srv_resp);
      ROS_WARN("Social layer --> changing min_range to %f", new_range);
      boost::unique_lock<boost::mutex> lock(social_layer_minrange_mutex_);
      social_layer_minrange_ = new_range;
      lock.unlock();
      return true;
  }
  catch (exception& e)
  {
      ROS_ERROR("An exception occurred in setSocialLayerMinRange. Exception Nr. %s", e.what() );
      return false;
  }

}
