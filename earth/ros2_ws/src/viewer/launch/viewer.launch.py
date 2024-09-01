from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rqt_plot",
            executable="rqt_plot",
            arguments=[
                "/gps_topic/altitude",
                "/gps_topic/speed",
                "/accelerometer_topic/y",
            ]
        ),
        Node(
            package="viewer",
            executable="viewer",
        ),
        Node(
            package="rqt_graph",
            executable="rqt_graph",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(get_package_share_directory('viewer'), 'launch/rviz.rviz')]
        ),
    ])
