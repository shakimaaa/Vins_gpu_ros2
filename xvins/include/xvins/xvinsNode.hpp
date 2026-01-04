#ifndef XVINS_LIFECYCLE_NODE_HPP_
#define XVINS_LIFECYCLE_NODE_HPP_

/*******************************************************
 * Copyright (C) 2025, Xion Control
 * 
 * This file is part of XVoyager.
 * 
 * Licensed under the MIT License; you may not use this file
 * except in compliance with the License.
 * 
 * See the LICENSE file distributed with this work for details.
 * 
 * Developed by Xion Control.
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "xvins/xvoyager_param.hpp"
#include "xvins/estimator/estimator.hpp"
#include "xvins/estimator/parameters.hpp"
#include "xvins/utility/visualization.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include "xion_msg/msg/extended_odometry.hpp"

namespace xvins {

class XVinsLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit XVinsLifecycleNode(const rclcpp::NodeOptions & options);
  virtual ~XVinsLifecycleNode() = default;

  // Lifecycle callbacks (to be implemented later)
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);



protected:
  // Callback function declarations
  void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
  void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr & img_msg);
  void sync_process();
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  // void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg);
  // void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg);
  

  // Callbacl group
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr img0_callback_group_;
  rclcpp::CallbackGroup::SharedPtr img1_callback_group_;

  // Updated global queues: using SharedPtr instead of deprecated ConstPtr.
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
    // std::queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> img0_buf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> img1_buf;
    std::mutex m_buf;

    //global estimator
    Estimator estimator;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_feature_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1_;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart_;

    std::atomic_bool thread_running_;
    std::atomic_bool node_active_;

    // Thread for syncing
    std::thread sync_thread_;
    
    //main parameter
    Parameter_t params_;

};

} // namespace xvins

#endif // XVINS_LIFECYCLE_NODE_HPP_
