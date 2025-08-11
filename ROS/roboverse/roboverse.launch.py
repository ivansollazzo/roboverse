import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    endpoint_launch_path = os.path.join(
        current_dir, 'src', 'ros_tcp_endpoint', 'launch', 'endpoint.py'
    )

    return LaunchDescription([
        # INCLUDE ROS TCP CONNECTOR ENDPOINT LAUNCH
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(endpoint_launch_path)
        ),

        # UNICYCLE 0 NODES
        Node(
            package='unicycle',
            executable='kinematics_controller',
            name='unicycle_0_kinematics_controller',
            parameters=[{'unicycle_id': 'unicycle_0'}],
            output='screen'
        )
    ])

