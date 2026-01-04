#ifndef XVINS_VISUALIZATION_HPP_
#define XVINS_VISUALIZATION_HPP_

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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/image_encodings.h>
#include "xvins/utility/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "xvins/utility/camera_pose_visualization.hpp"
#include <Eigen/Dense>
#include "xvins/estimator/estimator.hpp"
#include "xvins/estimator/parameters.hpp"
#include "xvins/xvoyager_param.hpp"
#include <fstream>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include "xion_msg/msg/extended_odometry.hpp"

namespace xvins {

extern rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr  pub_odometry;
extern rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path;
// extern rclcpp_lifecycle::LifecyclePublisher<xion_msg::msg::ExtendedOdometry>::SharedPtr pub_odometry_source;
// extern ros::Publisher pub_cloud, pub_map;
extern rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
// extern ros::Publisher pub_ref_pose, pub_cur_pose;
// extern ros::Publisher pub_key;
extern nav_msgs::msg::Path path;
//extern rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph; // maybe not used ??
extern int IMAGE_ROW, IMAGE_COL;

void registerPubs(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

// void pubOdometrySource(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator &estimator, const std_msgs::msg::Header &header);

void activatePubs();
void deactivatePubs();
void cleanupPubs();

} // namespace xvins

#endif // XVINS_VISUALIZATION_HPP_
