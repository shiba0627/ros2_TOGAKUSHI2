from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','1','world','odom'],
        ),
        Node(
            package='ros2_whill',
            namespace='',
            executable='whill_modelc_controller',
            remappings=[
            ],
            output="screen",
            parameters=[{
              'serialport': '/dev/ttyUSB4'
            }],
        ),
    ])
