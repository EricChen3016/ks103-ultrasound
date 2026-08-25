from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ks103_ultrasound",
            executable="ks103_node",
            name="ks103_ultrasound",
            parameters=[os.path.join(
                get_package_share_directory("ks103_ultrasound"),
                "config", "example.yaml")],
            output="screen",
        )
    ])
