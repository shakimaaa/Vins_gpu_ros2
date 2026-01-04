#include "xvins/xvoyager_param.hpp"

namespace xvins
{

  Parameter_t::Parameter_t() {}

  void Parameter_t::declare_all_parameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node)
  {
    if (!node->has_parameter("imu"))
      node->declare_parameter("imu", 1);
    if (!node->has_parameter("num_of_cam"))
      node->declare_parameter("num_of_cam", 2);
    if (!node->has_parameter("imu_topic"))
      node->declare_parameter("imu_topic", "/mav/imu/data_raw");
    if (!node->has_parameter("image0_topic"))
      node->declare_parameter("image0_topic", "/camera/camera/infra1/image_rect_raw");
    if (!node->has_parameter("image1_topic"))
      node->declare_parameter("image1_topic", "/camera/camera/infra2/image_rect_raw");
    if (!node->has_parameter("output_path"))
      node->declare_parameter("output_path", "/home/alexmanson/altivision/vins_config");
    if (!node->has_parameter("cam0_calib"))
      node->declare_parameter("cam0_calib", "left.yaml"); 
    if (!node->has_parameter("cam1_calib"))
      node->declare_parameter("cam1_calib", "right.yaml");
    if (!node->has_parameter("image_width"))
      node->declare_parameter("image_width", 640);
    if (!node->has_parameter("image_height"))
      node->declare_parameter("image_height", 480);
    if (!node->has_parameter("estimate_extrinsic"))
      node->declare_parameter("estimate_extrinsic", 0);
    if (!node->has_parameter("body_T_cam0"))
      node->declare_parameter("body_T_cam0", std::vector<double>());
    if (!node->has_parameter("body_T_cam1"))
      node->declare_parameter("body_T_cam1", std::vector<double>());
    if (!node->has_parameter("multiple_thread"))
      node->declare_parameter("multiple_thread", 1);
    if (!node->has_parameter("max_cnt"))
      node->declare_parameter("max_cnt", 80);
    if (!node->has_parameter("min_dist"))
      node->declare_parameter("min_dist", 40);
    if (!node->has_parameter("freq"))
      node->declare_parameter("freq", 1);
    if (!node->has_parameter("F_threshold"))
      node->declare_parameter("F_threshold", 1.0);
    if (!node->has_parameter("show_track"))
      node->declare_parameter("show_track", 1);
    if (!node->has_parameter("flow_back"))
      node->declare_parameter("flow_back", 1);
    if (!node->has_parameter("max_solver_time"))
      node->declare_parameter("max_solver_time", 0.015);
    if (!node->has_parameter("max_num_iterations"))
      node->declare_parameter("max_num_iterations", 2);
    if (!node->has_parameter("keyframe_parallax"))
      node->declare_parameter("keyframe_parallax", 10.0);
    if (!node->has_parameter("acc_n"))
      node->declare_parameter("acc_n", 0.3);
    if (!node->has_parameter("gyr_n"))
      node->declare_parameter("gyr_n", 0.06);
    if (!node->has_parameter("acc_w"))
      node->declare_parameter("acc_w", 0.0001);
    if (!node->has_parameter("gyr_w"))
      node->declare_parameter("gyr_w", 0.00001);
    if (!node->has_parameter("g_norm"))
      node->declare_parameter("g_norm", 9.805);
    if (!node->has_parameter("estimate_td"))
      node->declare_parameter("estimate_td", 0);
    if (!node->has_parameter("td"))
      node->declare_parameter("td", 0.009121);
    if (!node->has_parameter("load_previous_pose_graph"))
      node->declare_parameter("load_previous_pose_graph", 0);
    if (!node->has_parameter("pose_graph_save_path"))
      node->declare_parameter("pose_graph_save_path", "/home/fast/savedfiles/output/pose_graph/");
    if (!node->has_parameter("save_image"))
      node->declare_parameter("save_image", 1);
    if (!node->has_parameter("use_gpu_acc_flow"))
      node->declare_parameter("use_gpu_acc_flow", 1);
    if (!node->has_parameter("use_gpu"))
      node->declare_parameter("use_gpu", 1);
    if (!node->has_parameter("publish_rectify"))
      node->declare_parameter("publish_rectify", 0);
    if (!node->has_parameter("config_path"))
      node->declare_parameter("config_path", "./xvins/config");
    if (!node->has_parameter("init_depth"))
      node->declare_parameter("init_depth", 0.5);
    if (!node->has_parameter("bias_acc_threshold"))
      node->declare_parameter("bias_acc_threshold", 0.1);
    if (!node->has_parameter("bias_gry_threshold"))
      node->declare_parameter("bias_gry_threshold", 0.1);
    if (!node->has_parameter("nfeatures"))
      node->declare_parameter("nfeatures", 200);
    if (!node->has_parameter("scale_factor"))
      node->declare_parameter("scale_factor", 1.2);
    if (!node->has_parameter("nlevels"))
      node->declare_parameter("nlevels", 8);
    if (!node->has_parameter("edge_threshold"))
      node->declare_parameter("edge_threshold", 31);
    if (!node->has_parameter("first_level"))
      node->declare_parameter("first_level", 0);
    if (!node->has_parameter("wta_k"))
      node->declare_parameter("wta_k", 2);
    if (!node->has_parameter("patch_size"))
      node->declare_parameter("patch_size", 31);
    if (!node->has_parameter("fast_threshold"))
      node->declare_parameter("fast_threshold", 20);
    if (!node->has_parameter("orb_min_dist"))
      node->declare_parameter("orb_min_dist", 20.0); // Default value for LK_SIZE
    if (!node->has_parameter("track_method"))
      node->declare_parameter("track_method", 0); // 0 for optical flow, 1 for ORB
    if (!node->has_parameter("use_gpu_ceres"))
      node->declare_parameter("use_gpu_ceres", 1); // 0 for cpu ceres, 1 for gpu ceres
    if (!node->has_parameter("lk_n"))
      node->declare_parameter("lk_n", 3); // Default value for LK_N
    if (!node->has_parameter("lk_size"))
      node->declare_parameter("lk_size", 21); // Default value for LK_SIZE

  }

  void Parameter_t::read_all_parameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node)
  {
    RCLCPP_INFO(node->get_logger(),"开始read");

    CAM_NAMES.clear();
    RIC.clear();
    TIC.clear();
    
    // 读取基础参数
    read_essential_param(node, "image0_topic", IMAGE0_TOPIC);
    read_essential_param(node, "image1_topic", IMAGE1_TOPIC);
    read_essential_param(node, "max_cnt", MAX_CNT);
    read_essential_param(node, "min_dist", MIN_DIST);
    read_essential_param(node, "F_threshold", F_THRESHOLD);
    read_essential_param(node, "show_track", SHOW_TRACK);
    read_essential_param(node, "flow_back", FLOW_BACK);
    read_essential_param(node, "multiple_thread", MULTIPLE_THREAD);
    read_essential_param(node, "use_gpu", USE_GPU);
    read_essential_param(node, "use_gpu_acc_flow", USE_GPU_ACC_FLOW);
    // orb tracking parameters
    read_essential_param(node, "nfeatures", NFEATURES);
    read_essential_param(node, "scale_factor", SCALE_FACTOR);
    read_essential_param(node, "nlevels", NLEVELS);
    read_essential_param(node, "edge_threshold", EDGE_THRESHOLD);
    read_essential_param(node, "first_level", FIRST_LEVEL);
    read_essential_param(node, "wta_k", WTA_K);
    read_essential_param(node, "patch_size", PATCH_SIZE);
    read_essential_param(node, "fast_threshold", FAST_THRESHOLD);
    read_essential_param(node, "orb_min_dist", ORB_MIN_DIST);
    read_essential_param(node, "use_gpu_ceres", USE_GPU_CERES);
    read_essential_param(node, "lk_n", LK_N);
    read_essential_param(node, "lk_size", LK_SIZE);

    int method_int;
    read_essential_param(node, "track_method", method_int);
    if (method_int == 0)
    {
      TRACK_METHOD = xvins::TrackingMethod::OPTICAL_FLOW;
    }
    else if (method_int == 1)
    {
      TRACK_METHOD = xvins::TrackingMethod::ORB;
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid tracking method: %d", method_int);
      throw std::runtime_error("Invalid tracking method");
    }
    
    // RCLCPP_INFO(node->get_logger(),"topic ok");
    read_essential_param(node, "imu", USE_IMU);
    // RCLCPP_INFO(node->get_logger(),"imu ok");

    // 如果 USE_IMU 为 1，读取与 IMU 相关的参数
    if (USE_IMU)
    {
      read_essential_param(node, "imu_topic", IMU_TOPIC);
      RCLCPP_INFO(node->get_logger(), "IMU_TOPIC: %s", IMU_TOPIC.c_str());
      read_essential_param(node, "acc_n", ACC_N);
      read_essential_param(node, "acc_w", ACC_W);
      read_essential_param(node, "gyr_n", GYR_N);
      read_essential_param(node, "gyr_w", GYR_W);
      read_essential_param(node, "g_norm", G.z());
    }

    read_essential_param(node, "max_solver_time", SOLVER_TIME);
    read_essential_param(node, "max_num_iterations", NUM_ITERATIONS);
    read_essential_param(node, "keyframe_parallax", MIN_PARALLAX);
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    read_essential_param(node, "output_path", OUTPUT_FOLDER);
    VINS_RESULT_PATH =  OUTPUT_FOLDER + "/vio.csv";

    read_essential_param(node, "num_of_cam", NUM_OF_CAM);
    read_essential_param(node, "imu_topic", IMU_TOPIC);
    
    // RCLCPP_INFO(node->get_logger(),"output_path ok");
    // read_essential_param(node, "cam0_calib", CAM_NAMES[0]);
    // read_essential_param(node, "cam1_calib", CAM_NAMES[1]);
    // RCLCPP_INFO(node->get_logger(),"cam_calib ok");
    read_essential_param(node, "config_path", CONFIG_PATH);

    RCLCPP_INFO(node->get_logger(),"读取基础参数");

    // 处理LK_SIZE和LK_N
    if (!node->has_parameter("lk_size") || node->get_parameter("lk_size", LK_SIZE) == false)
    {
      LK_SIZE = 21; // 默认值
    }
    if (!node->has_parameter("lk_n") || node->get_parameter("lk_n", LK_N) == false)
    {
      LK_N = 3; // 默认值
    }


    read_essential_param(node, "estimate_extrinsic", ESTIMATE_EXTRINSIC);
    // 处理ESTIMATE_EXTRINSIC
    if (ESTIMATE_EXTRINSIC == 2)
    {
      RCLCPP_WARN(node->get_logger(), "Have no prior about extrinsic parameters, calibrating extrinsic parameters...");
      RIC.push_back(Eigen::Matrix3d::Identity());
      TIC.push_back(Eigen::Vector3d::Zero());
      EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else
    {
      if (ESTIMATE_EXTRINSIC == 1)
      {
        RCLCPP_WARN(node->get_logger(), "Optimize extrinsic parameters around the initial guess...");
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
      }
      if (ESTIMATE_EXTRINSIC == 0)
      {
        RCLCPP_WARN(node->get_logger(), "Fixing extrinsic parameters...");
      }

      // 读取 body_T_cam0 和 body_T_cam1
      std::vector<double> body_T_cam0;
      read_essential_param(node, "body_T_cam0", body_T_cam0);
      if (body_T_cam0.size() == 16)
      {
        Eigen::Matrix4d T;
        for (int i = 0; i < 16; ++i)
        {
          T(i / 4, i % 4) = body_T_cam0[i];
        }
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
      }

      // std::vector<double> body_T_cam1;
      // read_essential_param(node, "body_T_cam1", body_T_cam1);
      // if (body_T_cam1.size() == 16) {
      //     Eigen::Matrix4d T;
      //     for (int i = 0; i < 16; ++i) {
      //         T(i / 4, i % 4) = body_T_cam1[i];
      //     }
      //     RIC.push_back(T.block<3, 3>(0, 0));
      //     TIC.push_back(T.block<3, 1>(0, 3));
      // }
    }

    // 验证 NUM_OF_CAM
    if (NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
      printf("NUM_OF_CAM should be 1 or 2\n");
      assert(0);
    }

    std::string cam0Calib;
    read_essential_param(node, "cam0_calib", cam0Calib);
    std::string cam0Path = CONFIG_PATH + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    // 双目相机
    if (NUM_OF_CAM == 2)
    {
      STEREO = 1;
      std::string cam1Calib;
      read_essential_param(node, "cam1_calib", cam1Calib);
      std::string cam1Path = CONFIG_PATH + "/" + cam1Calib;
      CAM_NAMES.push_back(cam1Path);

      std::vector<double> body_T_cam1;
      read_essential_param(node, "body_T_cam1", body_T_cam1);
      if (body_T_cam1.size() == 16)
      {
        Eigen::Matrix4d T;
        for (int i = 0; i < 16; ++i)
        {
          T(i / 4, i % 4) = body_T_cam1[i];
        }
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
      }

      read_essential_param(node, "publish_rectify", PUB_RECTIFY);
    } else
    {
      STEREO = 0;
    
    }

    read_essential_param(node, "init_depth", INIT_DEPTH);
    read_essential_param(node, "bias_acc_threshold", BIAS_ACC_THRESHOLD);
    read_essential_param(node, "bias_gry_threshold", BIAS_GYR_THRESHOLD);
    read_essential_param(node, "td", TD);
    read_essential_param(node, "estimate_td", ESTIMATE_TD);


    // 处理时间延迟估计（ESTIMATE_TD）
    if (ESTIMATE_TD)
    {
      RCLCPP_INFO(node->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: %f", TD);
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Synchronized sensors, fix time offset: %f", TD);
    }

    // 图像
    read_essential_param(node, "image_height", ROW);
    read_essential_param(node, "image_width", COL);
    RCLCPP_INFO(node->get_logger(), "ROW: %d COL: %d", ROW, COL);

    // 如果没有启用 IMU，则禁用其他功能
    if (!USE_IMU)
    {
      ESTIMATE_EXTRINSIC = 0;
      ESTIMATE_TD = 0;
      printf("No IMU, fix extrinsic parameters; no time offset calibration\n");
    }
  }

  void Parameter_t::print(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) const
  {
    RCLCPP_INFO(node->get_logger(), "=================== Parameter Values ===================");
    // 打印基础参数
    RCLCPP_INFO(node->get_logger(), "USE_IMU: %d", USE_IMU);
    RCLCPP_INFO(node->get_logger(), "IMU_TOPIC: %s", IMU_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMAGE0_TOPIC: %s", IMAGE0_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMAGE1_TOPIC: %s", IMAGE1_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "OUTPUT_FOLDER: %s", OUTPUT_FOLDER.c_str());
    RCLCPP_INFO(node->get_logger(), "CAM_NAMES: ");
    for (const auto &cam : CAM_NAMES)
    {
      RCLCPP_INFO(node->get_logger(), "  %s", cam.c_str());
    }

    RCLCPP_INFO(node->get_logger(), "TRACK_METHOD: %d", static_cast<int>(TRACK_METHOD));

    RCLCPP_INFO(node->get_logger(), "NUM_OF_CAM: %d", NUM_OF_CAM);
    RCLCPP_INFO(node->get_logger(), "MULTIPLE_THREAD: %d\n", MULTIPLE_THREAD);
    RCLCPP_INFO(node->get_logger(), "----------------------GPU Mode----------------------");
    RCLCPP_INFO(node->get_logger(), "USE_GPU: %d", USE_GPU);
    RCLCPP_INFO(node->get_logger(), "USE_GPU_ACC_FLOW: %d", USE_GPU_ACC_FLOW);
    RCLCPP_INFO(node->get_logger(), "USE_GPU_CERES: %d", USE_GPU_CERES);
    RCLCPP_INFO(node->get_logger(), "---------------------GPU Mode----------------------\n");
    RCLCPP_INFO(node->get_logger(), "SOLVER_TIME: %f", SOLVER_TIME);
    RCLCPP_INFO(node->get_logger(), "MIN_PARALLAX: %f", MIN_PARALLAX);

    RCLCPP_INFO(node->get_logger(), "STEREO: %d", STEREO);
    RCLCPP_INFO(node->get_logger(), "MAX_CNT: %d", MAX_CNT);
    RCLCPP_INFO(node->get_logger(), "MIN_DIST: %d", MIN_DIST);
    RCLCPP_INFO(node->get_logger(), "F_THRESHOLD: %f", F_THRESHOLD);
    RCLCPP_INFO(node->get_logger(), "SHOW_TRACK: %d", SHOW_TRACK);
    RCLCPP_INFO(node->get_logger(), "FLOW_BACK: %d", FLOW_BACK);
    RCLCPP_INFO(node->get_logger(), "LK_SIZE: %d", LK_SIZE);
    RCLCPP_INFO(node->get_logger(), "LK_N: %d", LK_N);

    RCLCPP_INFO(node->get_logger(), "ACC_N: %f", ACC_N);
    RCLCPP_INFO(node->get_logger(), "GYR_N: %f", GYR_N);
    RCLCPP_INFO(node->get_logger(), "ACC_W: %f", ACC_W);
    RCLCPP_INFO(node->get_logger(), "GYR_W: %f", GYR_W);
    RCLCPP_INFO(node->get_logger(), "G_NORM: %f", G.z());

    RCLCPP_INFO(node->get_logger(), "TD: %f", TD);
    RCLCPP_INFO(node->get_logger(), "ESTIMATE_TD: %d", ESTIMATE_TD);

    RCLCPP_INFO(node->get_logger(), "=================== Parameter End ===================");
  }

}