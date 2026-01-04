import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    # 动态获取 vins 包的 share 目录路径
    # 注意：请确保您的 CMakeLists.txt 已配置将 config 文件夹安装到 share 目录
    vins_share_dir = get_package_share_directory("vins")
    config_path = os.path.join(vins_share_dir, "config", "testNode.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_path,
                description="VINS config file",
            ),
            Node(
                package="vins",
                executable="vins_node",
                name="vins_node",
                arguments=[config_file],
                output="screen",
            ),
        ]
    )
