from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with default values
    axle_length_arg = DeclareLaunchArgument(
        'axle_length',
        default_value='1.0',
        description='Distance between left and right wheels (meters)'
    )
    
    wheelbase_length_arg = DeclareLaunchArgument(
        'wheelbase_length',
        default_value='2.0',
        description='Distance between front and rear axles (meters)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Radius of the wheels (meters)'
    )
    
    center_of_mass_offset_arg = DeclareLaunchArgument(
        'center_of_mass_offset',
        default_value='0.0',
        description='Offset of center of mass from rear axle (meters)'
    )
    
    damping_factor_arg = DeclareLaunchArgument(
        'damping_factor',
        default_value='1',
        description='Damping factor for position updates'
    )
    
    # Create the odometry publisher node
    odom_publisher_node = Node(
        package='ackermann_odometry',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[{
            'axle_length': LaunchConfiguration('axle_length'),
            'wheelbase_length': LaunchConfiguration('wheelbase_length'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'center_of_mass_offset': LaunchConfiguration('center_of_mass_offset'),
            'damping_factor': LaunchConfiguration('damping_factor')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        axle_length_arg,
        wheelbase_length_arg,
        wheel_radius_arg,
        center_of_mass_offset_arg,
        damping_factor_arg,
        odom_publisher_node
    ])
