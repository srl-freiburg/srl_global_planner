/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Freiburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Luigi Palmieri
 *********************************************************************/
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <ros/console.h>


class costmapLayersDynRecHandler {


public:

  costmapLayersDynRecHandler(const ros::NodeHandle& node);

  ~costmapLayersDynRecHandler();


  /**
  * @brief enableObstacleLayer, call service to enable or disable Obstacle Layer
  * @param enable, If true enable the Obstacle Layer
  * @return bool, true if no exception
  */
  bool enableObstacleLayer(bool enable);

  /**
  * @brief enableSocialLayer, call service to enable or disable Social Compliance Layer
  * @param enable, If true enable the Social Compliance Layer
  * @return bool, true if no exception
  */
  bool enableSocialLayer(bool enable);


  /**
  * @brief enableInflaterLayer, call service to enable or disable Inflater Layer
  * @param enable, If true enable the Inflater Layer
  * @return bool, true if no exception
  */
  bool enableInflaterLayer(bool enable);


  /**
  * @brief isObstacleLayerEnabled, check if the Obstacle layer is enabled
  * @param none,
  * @return bool, true if obstacle layer is enabled
  */
  bool isObstacleLayerEnabled(){return obstacle_layer_enabled_;};

  /**
  * @brief isInflaterLayerEnabled, check if the Inflater layer is enabled
  * @param none,
  * @return bool, true if Inflater layer is enabled
  */
  bool isInflaterLayerEnabled(){return inflater_layer_enabled_;};

  /**
  * @brief isSocialLayerEnabled, check if the Social Compliance layer is enabled
  * @param none,
  * @return bool, true if the Social Compliance layer is enabled
  */
  bool isSocialLayerEnabled(){return social_layer_enabled_;};

  /**
  * @brief getSocialLayerMinRange, returns the min range where the Social Compliance layer is applied
  * @param none,
  * @return double, it returns the value of the Social Compliance min range value
  */
  double getSocialLayerMinRange(){return social_layer_minrange_;};

  /**
  * @brief getSocialLayerMaxRange, returns the max range where the Social Compliance layer is applied
  * @param none,
  * @return double, it returns the value of the Social Compliance max range value
  */
  double getSocialLayerMaxRange(){return social_layer_maxrange_;};

  /**
  * @brief setSocialLayerMaxRange, sets the max range where the Social Compliance layer is applied
  * @param none,
  * @return bool, it returns true if no exception
  */
  bool setSocialLayerMaxRange(double new_range);


  /**
  * @brief setSocialLayerMinRange, sets the min range where the Social Compliance layer is applied
  * @param none,
  * @return bool, it returns true if no exception
  */
  bool setSocialLayerMinRange(double new_range);


  /**
  * @brief setGlobalCostMapSize, sets the width and the height of the global cost map
  * @param none,
  * @return bool, it returns true if no exception
  */
  bool setGlobalCostMapSize(int width, int height);

private:

  bool obstacle_layer_enabled_; ///<  @brief True if obstacle layer is enabled

  bool inflater_layer_enabled_; ///<  @brief True if inflater layer is enabled

  bool social_layer_enabled_; ///<  @brief True if social layer is enabled

  double social_layer_minrange_;  ///<  @brief Value of the min range of the social compliance layer

  double social_layer_maxrange_; ///<  @brief Value of the max range of the social compliance layer

  boost::mutex obstacle_layer_enabled_mutex_;

  boost::mutex inflater_layer_enabled_mutex_;

  boost::mutex social_layer_enabled_mutex_;

  boost::mutex social_layer_minrange_mutex_;

  boost::mutex social_layer_maxrange_mutex_;

  ros::NodeHandle nh_layers_;


};
