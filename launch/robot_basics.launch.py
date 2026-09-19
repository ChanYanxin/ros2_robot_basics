import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ros2_robot_basics")
    params_file = os.path.join(package_share, "config", "controller.yaml")

    record_bag = LaunchConfiguration("record_bag")
    bag_name = LaunchConfiguration("bag_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description="Record the main tracking topics with rosbag2.",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value="rosbag2_joint_tracking",
                description="Output directory used by rosbag2.",
            ),
            Node(
                package="ros2_robot_basics",
                executable="target_joint_publisher",
                name="target_joint_publisher",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="ros2_robot_basics",
                executable="joint_pd_simulator",
                name="joint_pd_simulator",
                output="screen",
                parameters=[params_file],
            ),
            ExecuteProcess(
                condition=IfCondition(record_bag),
                cmd=[
                    "ros2", "bag", "record",
                    "--storage", "sqlite3",
                    "-o", bag_name,
                    "/target_joint_state",
                    "/joint_states",
                    "/control_effort",
                    "/tracking_error",
                ],
                output="screen",
            ),
        ]
    )
