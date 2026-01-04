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

#include "xvins/utility/visualization.hpp"

namespace xvins {

// using namespace ros;
using namespace Eigen;
rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry, pub_latest_odometry;
// rclcpp_lifecycle::LifecyclePublisher<xion_msg::msg::ExtendedOdometry>::SharedPtr pub_odometry_source;
rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path;
 rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud, pub_margin_cloud;
rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_image_track;

nav_msgs::msg::Path path;


CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPubs(shared_ptr<rclcpp_lifecycle::LifecycleNode> n)
{
    pub_latest_odometry = n->create_publisher<nav_msgs::msg::Odometry>("imu_propagate", rclcpp::QoS(100).best_effort());
    pub_path = n->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(100));
    pub_odometry = n->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(100));
    pub_point_cloud = n->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", rclcpp::QoS(100));
    pub_margin_cloud = n->create_publisher<sensor_msgs::msg::PointCloud>("margin_cloud", rclcpp::QoS(100));
    pub_key_poses = n->create_publisher<visualization_msgs::msg::Marker>("key_poses", rclcpp::QoS(1000));
    pub_camera_pose = n->create_publisher<nav_msgs::msg::Odometry>("camera_pose", rclcpp::QoS(1000));
    pub_camera_pose_visual = n->create_publisher<visualization_msgs::msg::MarkerArray>("camera_pose_visual", rclcpp::QoS(100));
    pub_keyframe_pose = n->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", rclcpp::QoS(1000));
    pub_keyframe_point = n->create_publisher<sensor_msgs::msg::PointCloud>("keyframe_point", rclcpp::QoS(100));
    pub_extrinsic = n->create_publisher<nav_msgs::msg::Odometry>("extrinsic", rclcpp::QoS(100));
    pub_image_track = n->create_publisher<sensor_msgs::msg::Image>("image_track", rclcpp::QoS(10));
    // pub_odometry_source = n->create_publisher<xion_msg::msg::ExtendedOdometry>("/position/slam_odom", rclcpp::QoS(100));

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}

void activatePubs()
{
    if (pub_latest_odometry) pub_latest_odometry->on_activate();
    if (pub_path) pub_path->on_activate();
    if (pub_odometry) pub_odometry->on_activate();
    if (pub_point_cloud) pub_point_cloud->on_activate();
    if (pub_margin_cloud) pub_margin_cloud->on_activate();
    if (pub_key_poses) pub_key_poses->on_activate();
    if (pub_camera_pose) pub_camera_pose->on_activate();
    if (pub_camera_pose_visual) pub_camera_pose_visual->on_activate();
    if (pub_keyframe_pose) pub_keyframe_pose->on_activate();
    if (pub_keyframe_point) pub_keyframe_point->on_activate();
    if (pub_extrinsic) pub_extrinsic->on_activate();
    if (pub_image_track) pub_image_track->on_activate();
    // if (pub_odometry_source) pub_odometry_source->on_activate();
}

void deactivatePubs()
{
    if (pub_latest_odometry) pub_latest_odometry->on_deactivate();
    if (pub_path) pub_path->on_deactivate();
    if (pub_odometry) pub_odometry->on_deactivate();
    if (pub_point_cloud) pub_point_cloud->on_deactivate();
    if (pub_margin_cloud) pub_margin_cloud->on_deactivate();
    if (pub_key_poses) pub_key_poses->on_deactivate();
    if (pub_camera_pose) pub_camera_pose->on_deactivate();
    if (pub_camera_pose_visual) pub_camera_pose_visual->on_deactivate();
    if (pub_keyframe_pose) pub_keyframe_pose->on_deactivate();
    if (pub_keyframe_point) pub_keyframe_point->on_deactivate();
    if (pub_extrinsic) pub_extrinsic->on_deactivate();
    if (pub_image_track) pub_image_track->on_deactivate();
    // if (pub_odometry_source) pub_odometry_source->on_deactivate();
}


void cleanupPubs()
{
    pub_latest_odometry.reset();
    pub_path.reset();
    pub_odometry.reset();
    pub_point_cloud.reset();
    pub_margin_cloud.reset();
    pub_key_poses.reset();
    pub_camera_pose.reset();
    pub_camera_pose_visual.reset();
    pub_keyframe_pose.reset();
    pub_keyframe_point.reset();
    pub_extrinsic.reset();
    pub_image_track.reset();
    // pub_odometry_source.reset();
}


void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::msg::Odometry odometry;

    int sec_ts = (int)t;
    uint nsec_ts = (uint)((t - sec_ts) * 1e9);
    odometry.header.stamp.sec = sec_ts;
    odometry.header.stamp.nanosec = nsec_ts;

    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry->publish(odometry);
}

void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::msg::Header header;
    header.frame_id = "world";

    int sec_ts = (int)t;
    uint nsec_ts = (uint)((t - sec_ts) * 1e9);
    header.stamp.sec = sec_ts;
    header.stamp.nanosec = nsec_ts;

    // sensor_msgs::msg::ImagePtr 
    sensor_msgs::msg::Image::SharedPtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track->publish(*imgTrackMsg);
}


void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    // ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            // ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
            // ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

// void pubOdometrySource(const Estimator &estimator, const std_msgs::msg::Header &header)
// {
//     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//     {
//         xion_msg::msg::ExtendedOdometry odometry_source;
//         odometry_source.header = header;
//         odometry_source.header.frame_id = "world";
//         odometry_source.child_frame_id = "world";
//         Quaterniond tmp_Q;
//         tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
//         odometry_source.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
//         odometry_source.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
//         odometry_source.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
//         odometry_source.pose.orientation.x = tmp_Q.x();
//         odometry_source.pose.orientation.y = tmp_Q.y();
//         odometry_source.pose.orientation.z = tmp_Q.z();
//         odometry_source.pose.orientation.w = tmp_Q.w();
//         odometry_source.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
//         odometry_source.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
//         odometry_source.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
//         odometry_source.bas.x = estimator.Bas[WINDOW_SIZE].x();
//         odometry_source.bas.y = estimator.Bas[WINDOW_SIZE].y();
//         odometry_source.bas.z = estimator.Bas[WINDOW_SIZE].z();
//         odometry_source.bgs.x = estimator.Bgs[WINDOW_SIZE].x();
//         odometry_source.bgs.y = estimator.Bgs[WINDOW_SIZE].y();
//         odometry_source.bgs.z = estimator.Bgs[WINDOW_SIZE].z();

//         pub_odometry_source->publish(odometry_source);
//     }
// }


void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        if (estimator.latest_pose_covariance_valid)
        {
            for (int r = 0; r < 6; ++r)
            {
                for (int c = 0; c < 6; ++c)
                    odometry.pose.covariance[r * 6 + c] = estimator.latest_pose_covariance(r, c);
            }
        }
        else
        {
            odometry.pose.covariance[0] = -1.0;
        }
        odometry.twist.covariance[0] = -1.0;
        pub_odometry->publish(odometry);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path->publish(path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.sec + header.stamp.nanosec * (1e-9) << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.sec + header.stamp.nanosec * (1e-9),
                                                          tmp_T.x(), tmp_T.y(), tmp_T.z(),
                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = rclcpp::Duration(std::chrono::seconds(0));

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses->publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose->publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if(STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;


    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::msg::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud->publish(point_cloud);


    // pub margined potin
    sensor_msgs::msg::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::msg::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud->publish(margin_cloud);
}



void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    // return; // tmp.


    cout << "tf 1" << endl;
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    // std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    // geometry_msgs::msg::TransformStamped transform, transform_cam;

    // tf2::Quaternion q;
    // // body frame
    // Vector3d correct_t;
    // Quaterniond correct_q;
    
    // cout << "tf 2" << endl;
    // correct_t = estimator.Ps[WINDOW_SIZE];
    // correct_q = estimator.Rs[WINDOW_SIZE];

    // cout << "tf 3" << endl;

    
    // cout << header.stamp.sec + header.stamp.nanosec * (1e-9) << endl;
    // cout << correct_t << endl;
    // cout << correct_q.w() << " " << correct_q.x() << " " << correct_q.y() << " " << correct_q.z() << endl;


    // // transform.header.stamp = header.stamp;
    // transform.header.frame_id = "world";
    // transform.child_frame_id = "body";

    // transform.transform.translation.x = correct_t(0);
    // transform.transform.translation.y = correct_t(1);
    // transform.transform.translation.z = correct_t(2);

    // cout << "tf 4" << endl;


    // q.setW(correct_q.w());
    // q.setX(correct_q.x());
    // q.setY(correct_q.y());
    // q.setZ(correct_q.z());
    // transform.transform.rotation.x = q.x();
    // transform.transform.rotation.y = q.y();
    // transform.transform.rotation.z = q.z();
    // transform.transform.rotation.w = q.w();

    // cout << "tf 5" << endl;

    // br->sendTransform(transform);


    // cout << "tf 6" << endl;



    // camera frame
    // transform_cam.header.stamp = header.stamp;
    // transform_cam.header.frame_id = "body";
    // transform_cam.child_frame_id = "camera";


    // transform_cam.transform.translation.x = estimator.tic[0].x();
    // transform_cam.transform.translation.y = estimator.tic[0].y();
    // transform_cam.transform.translation.z = estimator.tic[0].z();

    // q.setW(Quaterniond(estimator.ric[0]).w());
    // q.setX(Quaterniond(estimator.ric[0]).x());
    // q.setY(Quaterniond(estimator.ric[0]).y());
    // q.setZ(Quaterniond(estimator.ric[0]).z());

    // transform_cam.transform.rotation.x = q.x();
    // transform_cam.transform.rotation.y = q.y();
    // transform_cam.transform.rotation.z = q.z();
    // transform_cam.transform.rotation.w = q.w();

    // br->sendTransform(transform_cam);

    cout << "tf 7" << endl;

    
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();

    cout << "tf 8" << endl;
    pub_extrinsic->publish(odometry);
    cout << "tf 9" << endl;

}


// void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
// {
//     if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
//         return;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> br;
//     tf2::Transform transform;
//     tf2::Quaternion q;
//     // body frame
//     Vector3d correct_t;
//     Quaterniond correct_q;
//     correct_t = estimator.Ps[WINDOW_SIZE];
//     correct_q = estimator.Rs[WINDOW_SIZE];

//     transform.setOrigin(tf2::Vector3(correct_t(0),
//                                     correct_t(1),
//                                     correct_t(2)));
//     q.setW(correct_q.w());
//     q.setX(correct_q.x());
//     q.setY(correct_q.y());
//     q.setZ(correct_q.z());
//     transform.setRotation(q);
//     // br->sendTransform(tf2::StampedTransform(transform, header.stamp, "world", "body"));
//     br->sendTransform(tf2::StampedTransform(transform, header.stamp, "world", "body"));

//     // camera frame
//     transform.setOrigin(tf2::Vector3(estimator.tic[0].x(),
//                                     estimator.tic[0].y(),
//                                     estimator.tic[0].z()));
//     q.setW(Quaterniond(estimator.ric[0]).w());
//     q.setX(Quaterniond(estimator.ric[0]).x());
//     q.setY(Quaterniond(estimator.ric[0]).y());
//     q.setZ(Quaterniond(estimator.ric[0]).z());
//     transform.setRotation(q);
//     // br->sendTransform(tf2::StampedTransform(transform, header.stamp, "body", "camera"));
//     br->sendTransform(tf2::StampedTransform(transform, header.stamp, "body", "camera"));

    
//     nav_msgs::msg::Odometry odometry;
//     odometry.header = header;
//     odometry.header.frame_id = "world";
//     odometry.pose.pose.position.x = estimator.tic[0].x();
//     odometry.pose.pose.position.y = estimator.tic[0].y();
//     odometry.pose.pose.position.z = estimator.tic[0].z();
//     Quaterniond tmp_q{estimator.ric[0]};
//     odometry.pose.pose.orientation.x = tmp_q.x();
//     odometry.pose.pose.orientation.y = tmp_q.y();
//     odometry.pose.pose.orientation.z = tmp_q.z();
//     odometry.pose.pose.orientation.w = tmp_q.w();
//     pub_extrinsic->publish(odometry);

// }

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::msg::Odometry odometry;

        int sec_ts = (int)estimator.Headers[WINDOW_SIZE - 2];
        uint nsec_ts = (uint)((estimator.Headers[WINDOW_SIZE - 2] - sec_ts) * 1e9);
        odometry.header.stamp.sec = sec_ts;
        odometry.header.stamp.nanosec = nsec_ts;

        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.sec, P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose->publish(odometry);


        sensor_msgs::msg::PointCloud point_cloud;

        sec_ts = (int)estimator.Headers[WINDOW_SIZE - 2];
        nsec_ts = (uint)((estimator.Headers[WINDOW_SIZE - 2] - sec_ts) * 1e9);
        point_cloud.header.stamp.sec = sec_ts;
        point_cloud.header.stamp.nanosec = nsec_ts;

        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::msg::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point->publish(point_cloud);
    }
}

} // namespace xvins
