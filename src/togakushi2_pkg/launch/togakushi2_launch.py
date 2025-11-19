from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='communication', executable='LiDAR', output='screen'),     
        Node(package='ros2_whill', executable='whill_modelc_controller', output='screen'),
        Node(package='ros2_whill', executable='whill_modelc_publisher', output='screen'),
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen'),
        Node(package='patlite_pypkg', executable='color_node', output = 'screen'),
        Node(package='state_manager_pkg', executable='state_manager_node', output='screen'),
        Node(package='udp_server_pkg', executable='udp_server_node_2', output='screen'),
        Node(package='camera_pkg', executable='camera_node', output='screen'),
        Node(package='game_pkg', executable='game_node', output='screen'),
        Node(package='logger_pkg', executable='compress_node', output='screen'),
        Node(package='logger_pkg', executable='joy_logger', output='screen'),
        Node(package='logger_pkg', executable='image_logger', output='screen'),
        Node(package='logger_pkg', executable='obstacle_logger', output='screen'),
        Node(package='logger_pkg', executable='odom_logger', output='screen')
    ])
