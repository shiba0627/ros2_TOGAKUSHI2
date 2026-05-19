#v2の障害物検知
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_whill', executable='whill_modelc_controller', output='screen'),
        Node(package='ros2_whill', executable='whill_modelc_publisher', output='screen'),
        Node(package='logger_pkg', executable='odom_logger', output='screen')
    ])
