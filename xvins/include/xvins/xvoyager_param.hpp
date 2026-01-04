#ifndef XVOYAGER_PARAM_HPP_
#define XVOYAGER_PARAM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <Eigen/Dense>
#include "xvins/utility/utility.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std;

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR

// Global constants and macros remain in the global scope.

const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
const double FOCAL_LENGTH = 460.0;
// #define UNIT_SPHERE_ERROR

namespace xvins
{
    enum TrackingMethod {
        OPTICAL_FLOW = 0,  // OPTICAL_FLOW
        ORB = 1            // ORB
    };

    class Parameter_t
    {
    public:
        // 变量声明
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
        std::string CONFIG_PATH;
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

        // ORB tracking parameters
        int NFEATURES;
        float SCALE_FACTOR;
        int NLEVELS;
        int EDGE_THRESHOLD;
        int FIRST_LEVEL;
        int WTA_K;
        int PATCH_SIZE;
        int FAST_THRESHOLD;
        double ORB_MIN_DIST;


        xvins::TrackingMethod TRACK_METHOD;

        // Enums for parameter sizes and state/ noise ordering
        enum SIZE_PARAMETERIZATION
        {
            SIZE_POSE = 7,
            SIZE_SPEEDBIAS = 9,
            SIZE_FEATURE = 1
        };

        enum StateOrder
        {
            O_P = 0,
            O_R = 3,
            O_V = 6,
            O_BA = 9,
            O_BG = 12
        };

        enum NoiseOrder
        {
            O_AN = 0,
            O_GN = 3,
            O_AW = 6,
            O_GW = 9
        };

        Parameter_t();
        void read_all_parameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node);
        void declare_all_parameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node);
        void print(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) const;

    private:
        template <typename TName, typename TVal>
        void read_essential_param(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, const TName &name, TVal &val)
        {
            if (node->get_parameter(name, val))
            {
                // Parameter retrieved successfully
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Read param: %s failed.", name);
                 throw std::runtime_error(std::string("Essential parameter not set: ") + name);
            }
        }
    };

}

#endif