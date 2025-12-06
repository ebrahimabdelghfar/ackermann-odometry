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
    
    # Declare launch arguments with default values (for override)
    axle_length_arg = DeclareLaunchArgument(
        'axle_length',
        default_value='',
        description='Distance between left and right wheels (meters). Overrides config file if set.'
    )
    
    wheelbase_length_arg = DeclareLaunchArgument(
        'wheelbase_length',
        default_value='',
        description='Distance between front and rear axles (meters). Overrides config file if set.'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='',
        description='Radius of the wheels (meters). Overrides config file if set.'
    )
    
    center_of_mass_offset_arg = DeclareLaunchArgument(
        'center_of_mass_offset',
        default_value='',
        description='Offset of center of mass from rear axle (meters). Overrides config file if set.'
    )
    
    damping_factor_arg = DeclareLaunchArgument(
        'damping_factor',
        default_value='',
        description='Damping factor for position updates. Overrides config file if set.'
    )
    
    # Create parameter overrides dictionary
    param_overrides = {}
    
    # Add overrides only if they are not empty
    axle_length = LaunchConfiguration('axle_length')
    wheelbase_length = LaunchConfiguration('wheelbase_length')
    wheel_radius = LaunchConfiguration('wheel_radius')
    center_of_mass_offset = LaunchConfiguration('center_of_mass_offset')
    damping_factor = LaunchConfiguration('damping_factor')
    
    # Create the odometry publisher node
    odom_publisher_node = Node(
        package='ackermann_odometry',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[
            LaunchConfiguration('params_file'),
            # Command line parameters will override config file
            {
                'axle_length': axle_length,
                'wheelbase_length': wheelbase_length,
                'wheel_radius': wheel_radius,
                'center_of_mass_offset': center_of_mass_offset,
                'damping_factor': damping_factor,
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        params_file_arg,
        axle_length_arg,
        wheelbase_length_arg,
        wheel_radius_arg,
        center_of_mass_offset_arg,
        damping_factor_arg,
        odom_publisher_node
    ])
