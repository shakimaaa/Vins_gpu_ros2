cmake_minimum_required(VERSION 3.8)
project(xvins)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_models_vinx REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.10.0 EXACT REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Ceres REQUIRED)
# find_package(xion_msg REQUIRED)

#include(/home/dji/opencv/build/OpenCVConfig.cmake)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  include
)


# Libraries
add_library(xvins_lib
    src/xvoyager_param.cpp
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/camera_pose_visualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp
    )

target_link_libraries(xvins_lib  ${OpenCV_LIBS} ${CERES_LIBRARIES})

ament_target_dependencies(xvins_lib rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models_vinx image_transport rclcpp_lifecycle )

add_executable(xvins_node src/xvinsNode.cpp)
ament_target_dependencies(xvins_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models_vinx image_transport rclcpp_lifecycle )
target_link_libraries(xvins_node xvins_lib)


# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install nodes
install(
  TARGETS xvins_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Ament package
ament_package()