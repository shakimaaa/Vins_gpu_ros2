/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "xvins/estimator/parameters.hpp"

namespace xvins
{

    double INIT_DEPTH;
    double MIN_PARALLAX;
    double ACC_N, ACC_W;
    double GYR_N, GYR_W;

    std::vector<Eigen::Matrix3d> RIC;
    std::vector<Eigen::Vector3d> TIC;

    Eigen::Vector3d G{0.0, 0.0, 9.8};

    double BIAS_ACC_THRESHOLD;
    double BIAS_GYR_THRESHOLD;
    double SOLVER_TIME;
    int NUM_ITERATIONS;
    int ESTIMATE_EXTRINSIC;
    int ESTIMATE_TD;
    int ROLLING_SHUTTER;
    std::string EX_CALIB_RESULT_PATH;
    std::string VINS_RESULT_PATH;
    std::string OUTPUT_FOLDER;
    std::string IMU_TOPIC;
    int ROW, COL;
    double TD;
    int NUM_OF_CAM;
    int STEREO;
    int USE_IMU;
    int MULTIPLE_THREAD;
    int USE_GPU;
    int USE_GPU_ACC_FLOW;
    int USE_GPU_CERES;
    int PUB_RECTIFY;
    Eigen::Matrix3d rectify_R_left;
    Eigen::Matrix3d rectify_R_right;
    map<int, Eigen::Vector3d> pts_gt;
    std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
    std::string FISHEYE_MASK;
    std::vector<std::string> CAM_NAMES;
    int MAX_CNT;
    int MIN_DIST;
    double F_THRESHOLD;
    int SHOW_TRACK;
    int FLOW_BACK;
    int LK_SIZE;
    int LK_N;
    xvins::TrackingMethod TRACK_METHOD;

    int NFEATURES;
    float SCALE_FACTOR;
    int NLEVELS;
    int EDGE_THRESHOLD;
    int FIRST_LEVEL;
    int WTA_K;
    int PATCH_SIZE;
    int FAST_THRESHOLD;
    double ORB_MIN_DIST;

    void readParametersFromROS(const Parameter_t &params)
    {
        INIT_DEPTH = params.INIT_DEPTH;
        MIN_PARALLAX = params.MIN_PARALLAX;
        ACC_N = params.ACC_N;
        ACC_W = params.ACC_W;
        GYR_N = params.GYR_N;
        GYR_W = params.GYR_W;
        RIC = params.RIC;
        TIC = params.TIC;
        G = params.G;
        BIAS_ACC_THRESHOLD = params.BIAS_ACC_THRESHOLD;
        BIAS_GYR_THRESHOLD = params.BIAS_GYR_THRESHOLD;
        SOLVER_TIME = params.SOLVER_TIME;
        NUM_ITERATIONS = params.NUM_ITERATIONS;
        ESTIMATE_EXTRINSIC = params.ESTIMATE_EXTRINSIC;
        ESTIMATE_TD = params.ESTIMATE_TD;
        ROLLING_SHUTTER = params.ROLLING_SHUTTER;
        EX_CALIB_RESULT_PATH = params.EX_CALIB_RESULT_PATH;
        VINS_RESULT_PATH = params.VINS_RESULT_PATH;
        OUTPUT_FOLDER = params.OUTPUT_FOLDER;
        IMU_TOPIC = params.IMU_TOPIC;
        ROW = params.ROW;
        COL = params.COL;
        TD = params.TD;
        NUM_OF_CAM = params.NUM_OF_CAM;
        STEREO = params.STEREO;
        USE_IMU = params.USE_IMU;
        MULTIPLE_THREAD = params.MULTIPLE_THREAD;
        USE_GPU = params.USE_GPU;
        USE_GPU_ACC_FLOW = params.USE_GPU_ACC_FLOW;
        USE_GPU_CERES = params.USE_GPU_CERES;
        PUB_RECTIFY = params.PUB_RECTIFY;
        rectify_R_left = params.rectify_R_left;
        rectify_R_right = params.rectify_R_right;
        pts_gt = params.pts_gt;
        IMAGE0_TOPIC = params.IMAGE0_TOPIC;
        IMAGE1_TOPIC = params.IMAGE1_TOPIC;
        FISHEYE_MASK = params.FISHEYE_MASK;
        CAM_NAMES = params.CAM_NAMES;
        MAX_CNT = params.MAX_CNT;
        MIN_DIST = params.MIN_DIST;
        F_THRESHOLD = params.F_THRESHOLD;
        SHOW_TRACK = params.SHOW_TRACK;
        FLOW_BACK = params.FLOW_BACK;
        LK_SIZE = params.LK_SIZE;
        LK_N = params.LK_N;
        TRACK_METHOD = params.TRACK_METHOD;

        NFEATURES = params.NFEATURES;
        SCALE_FACTOR = params.SCALE_FACTOR;
        NLEVELS = params.NLEVELS;
        EDGE_THRESHOLD = params.EDGE_THRESHOLD;
        FIRST_LEVEL= params.FIRST_LEVEL;
        WTA_K = params.WTA_K;
        PATCH_SIZE = params.PATCH_SIZE;
        FAST_THRESHOLD = params.FAST_THRESHOLD;
        ORB_MIN_DIST = params.ORB_MIN_DIST;

        RCLCPP_INFO(rclcpp::get_logger("readParametersFromROS"), "Parameters read from ROS successfully.");
    }

} // namespace xvins
