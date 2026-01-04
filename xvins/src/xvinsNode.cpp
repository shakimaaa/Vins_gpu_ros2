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
#include "xvins/xvinsNode.hpp"
#include <rclcpp/rclcpp.hpp>
using std::placeholders::_1;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
namespace xvins {

XVinsLifecycleNode::XVinsLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("xvins_lifecycle_node", options)
{
    // Declar parameter here please bohao
     // this->declare_parameter
     // refer to zhever_fsm params_.declare_all_parameters(shared_from_this());
  
    //  params_.declare_all_parameters(shared_from_this());

     
  
  thread_running_.store(false); // Control the lifecycle of the background thread (start/stop).
  node_active_.store(false); // Mark whether a node is active (to avoid performing calculations or publishing data in an inactive state)

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XVinsLifecycleNode::on_configure(const rclcpp_lifecycle::State & state)
{  
    
  RCLCPP_INFO(get_logger(), "on_configure() called");
  // Step 1: Read config file parameter
//refer to zherver_fsm params_.read_all_parameters(shared_from_this());
  
  // Read parameters
  //std::cout << "start load" << std::endl;
  RCLCPP_INFO(get_logger(),"start declare parameters");
  params_.declare_all_parameters(shared_from_this());

  RCLCPP_INFO(get_logger(),"start read parameters");
  try {
    params_.read_all_parameters(shared_from_this());
    RCLCPP_INFO(get_logger(), "Parameters loaded successfully.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error loading parameters: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(),"read parameters successfully");

  params_.print(shared_from_this());
  RCLCPP_INFO(get_logger(),"initializing estimator");
  //initialize estimator
  // estimator.readParameterFromROS(params_);
  xvins::readParametersFromROS(params_); // Read config parameters

  estimator.setParameter(); // assume parameter is read already in parameter.cpp


  // Step 2: Register publishers (they are global in visualization.cpp)
  registerPubs(shared_from_this());

  // Step 3: Create subscriptions
  rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
  // rclcpp::QoS feature_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  // rclcpp::QoS restart_qos = rclcpp::QoS(rclcpp::KeepLast(100));

  imu_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  img0_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  img1_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_sub_opt;
    imu_sub_opt.callback_group = imu_callback_group_;
  rclcpp::SubscriptionOptions img0_sub_opt;
    img0_sub_opt.callback_group = img0_callback_group_;
  rclcpp::SubscriptionOptions img1_sub_opt;
    img1_sub_opt.callback_group = img1_callback_group_;

if (params_.USE_IMU) {
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    params_.IMU_TOPIC, imu_qos, std::bind(&XVinsLifecycleNode::imu_callback, this, _1),
  imu_sub_opt);
}

// sub_feature_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
//   "/feature_tracker/feature", feature_qos, std::bind(&XVinsLifecycleNode::feature_callback, this, _1));

sub_img0_ = this->create_subscription<sensor_msgs::msg::Image>(
  params_.IMAGE0_TOPIC, image_qos, std::bind(&XVinsLifecycleNode::img0_callback, this, _1),
img0_sub_opt);

if (params_.STEREO) {
  sub_img1_ = this->create_subscription<sensor_msgs::msg::Image>(
    params_.IMAGE1_TOPIC, image_qos, std::bind(&XVinsLifecycleNode::img1_callback, this, _1),
  img1_sub_opt);
}

// sub_restart_ = this->create_subscription<std_msgs::msg::Bool>(
//   "/vins_restart", restart_qos, std::bind(&XVinsLifecycleNode::restart_callback, this, _1));
    
node_active_.store(false);


RCLCPP_INFO(this->get_logger(), "XVinsLifecycleNode configured successfully!");


return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XVinsLifecycleNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_activate() called");
  activatePubs();

node_active_.store(true);
thread_running_.store(true);

// Start your sync_process thread if needed
  sync_thread_ = std::thread(&XVinsLifecycleNode::sync_process, this);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XVinsLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() called");
  deactivatePubs();

  // Set the thread flag to false
  thread_running_.store(false);
  node_active_.store(false);


  // close thread
  try {
    if (sync_thread_.joinable()) {
      sync_thread_.join();
    }
  } catch (const std::system_error& e) {
    RCLCPP_ERROR(get_logger(), "Thread join failed: %s", e.what());
  }

  estimator.stopThreadAndJoin();
  //clear estimator
  estimator.clearState();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XVinsLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() called");

  // 1. Cleanup publishers
  cleanupPubs();

  // 2. Reset subscriptions (destroy them)
  sub_imu_.reset();
  // sub_feature_.reset();
  sub_img0_.reset();
  sub_img1_.reset();
  // sub_restart_.reset();

  // Reset callback groups
  imu_callback_group_.reset();
  img0_callback_group_.reset();
  img1_callback_group_.reset();

  // 3. Stop sync_thread if somehow still running (super safe)
  thread_running_.store(false);
  node_active_.store(false);

  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }

  // 4. Clear estimator state completely (optional)
  estimator.clearState();

  RCLCPP_INFO(get_logger(), "XVinsLifecycleNode cleanup done!");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// Callback function definitions (empty for now)
void XVinsLifecycleNode::img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{   
    if (!node_active_.load()) {
        return;
    }
    // static double last_img0_time = 0;
    // double current_img0_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
    // if (current_img0_time - last_img0_time < 1.0 / 15) {
    //     // If the time difference is less than 1/15 seconds, skip this image
    //     // This is to avoid processing images that are too close in time
    //     // printf("skip img0\n");
    //     // ROS_INFO("skip img0");
    //     return;
    // }
    // last_img0_time = current_img0_time;

    std::lock_guard<std::mutex> lock(m_buf);
    img0_buf.push(img_msg);
}

// Image callback function
void XVinsLifecycleNode::img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{   
    if (!node_active_.load()) {
        return;
    }
    // static double last_img1_time = 0;
    // double current_img1_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
    // if (current_img1_time - last_img1_time < 1.0 / 15) {
    //     // If the time difference is less than 1/15 seconds, skip this image
    //     // This is to avoid processing images that are too close in time
    //     // printf("skip img1\n");
    //     // ROS_INFO("skip img1");
    //     return;
    // }
    // last_img1_time = current_img1_time;

    std::lock_guard<std::mutex> lock(m_buf);
    img1_buf.push(img_msg);
}

// Convert ros images to opencv
cv::Mat XVinsLifecycleNode::getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr & img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    return ptr->image.clone();
}

// synchronization thread
void XVinsLifecycleNode::sync_process()
{
  // double last_image_time = 0;
  while (thread_running_.load())
    {
        if (params_.STEREO)
        {
            // ROS_INFO("STEREO sync_process");
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            {
                std::lock_guard<std::mutex> lock(m_buf);
                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * 1e-9;
                    double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * 1e-9;

                    // 0.003s sync tolerance
                    if (time0 < time1 - 0.003)
                    {
                        img0_buf.pop();
                        printf("throw img0\n");
                        ROS_INFO("throw img0");
                    }
                    else if (time0 > time1 + 0.003)
                    {
                        img1_buf.pop();
                        printf("throw img1\n");
                        ROS_INFO("throw img1");
                    }
                    else
                    {
                        time = time0;
                        header = img0_buf.front()->header;
                        image0 = getImageFromMsg(img0_buf.front());
                        img0_buf.pop();
                        image1 = getImageFromMsg(img1_buf.front());
                        img1_buf.pop();
                        // printf("find img0 and img1\n");
                    }
                }
            }
            if (!image0.empty())
                {
                  // double dt = time - last_image_time;
                  // if(dt < 1.0 / 15 )
                  // {
                  //   continue;
                  // }
                  // last_image_time = time;
                  estimator.inputImage(time, image0, image1); // Image feature tracking function interface
                  // ROS_INFO("input time, image0, image1");
                }
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            {
                std::lock_guard<std::mutex> lock(m_buf);
                if (!img0_buf.empty())
                {
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * 1e-9;
                    header = img0_buf.front()->header;
                    image = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                }
            }
            if (!image.empty())
                estimator.inputImage(time, image);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// imu callback function
void XVinsLifecycleNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{   
    if (!node_active_.load()) {
        return;
    }
      double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    Vector3d acc(imu_msg->linear_acceleration.x,
                 imu_msg->linear_acceleration.y,
                 imu_msg->linear_acceleration.z);
    Vector3d gyr(imu_msg->angular_velocity.x,
                 imu_msg->angular_velocity.y,
                 imu_msg->angular_velocity.z);

    // Eigen::Matrix3d R_BA;
    // R_BA << 0, -1, 0,
    //         0,  0, -1,
    //         1,  0, 0;
    // acc = R_BA * acc;
    // gyr = R_BA * gyr;

    // imu added to buffer
    estimator.inputIMU(t, acc, gyr);
}

// void XVinsLifecycleNode::feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
// {   
//     if (!node_active_.load()) {
//         return;
//     }
//     std::cout << "feature cb" << std::endl;
//     std::cout << "Feature: " << feature_msg->points.size() << std::endl;

//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//     for (unsigned int i = 0; i < feature_msg->points.size(); i++)
//     {
//         int feature_id = feature_msg->channels[0].values[i];
//         int camera_id = feature_msg->channels[1].values[i];
//         double x = feature_msg->points[i].x;
//         double y = feature_msg->points[i].y;
//         double z = feature_msg->points[i].z;
//         double p_u = feature_msg->channels[2].values[i];
//         double p_v = feature_msg->channels[3].values[i];
//         double velocity_x = feature_msg->channels[4].values[i];
//         double velocity_y = feature_msg->channels[5].values[i];
//         if (feature_msg->channels.size() > 5)
//         {
//             double gx = feature_msg->channels[6].values[i];
//             double gy = feature_msg->channels[7].values[i];
//             double gz = feature_msg->channels[8].values[i];
//             pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
//             // printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
//         }
//         assert(z == 1);
//         Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//         xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
//     }
//     double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * 1e-9;
//     estimator.inputFeature(t, featureFrame);
// }

// void XVinsLifecycleNode::restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
// {   
//     if (!node_active_.load()) {
//         return;
//     }
//     if (restart_msg->data == true)
//     {
//         ROS_WARN("restart the estimator!");
//         estimator.clearState();
//         estimator.setParameter();
//     }
// }

} // namespace xvins

// Main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xvins::XVinsLifecycleNode>(rclcpp::NodeOptions());
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node->get_node_base_interface());
  rclcpp::spin(node->get_node_base_interface());
  // executor.spin();
  rclcpp::shutdown();
  return 0;
}
