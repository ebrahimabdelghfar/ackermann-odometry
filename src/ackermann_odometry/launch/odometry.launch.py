import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ackermann_odometry')
    
    # Path to the default parameters file
    default_params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch argument for custom parameters file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file'
    )
    
    # Create the odometry publisher node
    odom_publisher_node = Node(
        package='ackermann_odometry',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    return LaunchDescription([
        params_file_arg,
        odom_publisher_node
    ])
