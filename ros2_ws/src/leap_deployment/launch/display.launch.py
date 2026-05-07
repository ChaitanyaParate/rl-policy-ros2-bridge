import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'leap_deployment'
    pkg_share = get_package_share_directory(pkg_name)
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'leap_hand.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    # Check if rviz config exists, else start without it
    rviz_args = []
    if os.path.exists(rviz_config_path):
        rviz_args = ['-d', rviz_config_path]
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
        
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package=pkg_name,
            executable='policy_node',
            name='policy_node',
            # Pass use_sine_wave true or false based on preference
            parameters=[{'use_sine_wave': False}] # Set to False to load the trained SB3 model
        ),
        Node(
            package=pkg_name,
            executable='interface_node',
            name='interface_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args
        )
    ])
