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
#include "xvins/estimator/estimator.hpp"
#include "xvins/estimator/parameters.hpp"
#include "xvins/utility/visualization.hpp"

using namespace xvins;

// Global estimator instance
Estimator estimator;

// Updated global queues: using SharedPtr instead of deprecated ConstPtr.
std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
std::queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
std::queue<sensor_msgs::msg::Image::SharedPtr> img0_buf;
std::queue<sensor_msgs::msg::Image::SharedPtr> img1_buf;
std::mutex m_buf;

// Callback for the left image topic.
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(m_buf);
    // std::cout << "Left : " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << std::endl;
    img0_buf.push(img_msg);
}

// Callback for the right image topic.
void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(m_buf);
    // std::cout << "Right: " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << std::endl;
    img1_buf.push(img_msg);
}

// Converts a sensor_msgs::msg::Image message to a cv::Mat.
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr &img_msg)
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

// Synchronizes and processes images from the stereo camera topics.
void sync_process()
{
    while (true)
    {
        if (STEREO)
        {
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
                    }
                    else if (time0 > time1 + 0.003)
                    {
                        img1_buf.pop();
                        printf("throw img1\n");
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
                estimator.inputImage(time, image0, image1);
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

// IMU callback: processes incoming IMU data.
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    Vector3d acc(imu_msg->linear_acceleration.x,
                 imu_msg->linear_acceleration.y,
                 imu_msg->linear_acceleration.z);
    Vector3d gyr(imu_msg->angular_velocity.x,
                 imu_msg->angular_velocity.y,
                 imu_msg->angular_velocity.z);
    estimator.inputIMU(t, acc, gyr);
}

// Feature callback: processes incoming feature data.
void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    std::cout << "feature cb" << std::endl;
    std::cout << "Feature: " << feature_msg->points.size() << std::endl;

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if (feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            // printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * 1e-9;
    estimator.inputFeature(t, featureFrame);
}

// Restart callback: clears and resets the estimator.
void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("vins_estimator");

    if (argc != 2)
    {
        printf("please input: ros2 run xvins xvins_node [config file]\n"
               "for example: ros2 run xvins xvins_node ~/path/to/config.yaml\n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = nullptr;
    if (USE_IMU)
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, qos_profile, imu_callback);
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img0_callback);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1 = nullptr;
    if (STEREO)
    {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img1_callback);
    }
    
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);

    std::thread sync_thread{sync_process};
    rclcpp::spin(n);

    return 0;
}
