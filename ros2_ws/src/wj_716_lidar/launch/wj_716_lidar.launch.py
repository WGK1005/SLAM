import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('wj_716_lidar')
    default_params_file = os.path.join(pkg_dir, 'config', 'wj_716_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to ROS 2 parameters YAML file'
    )

    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.100.104',
        description='LiDAR IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2110',
        description='LiDAR TCP port'
    )

    lidar_node = Node(
        package='wj_716_lidar',
        executable='wj_716_lidar_node',
        name='wj_716_lidar_01',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'hostname': LaunchConfiguration('hostname'),
                'port': LaunchConfiguration('port')
            }
        ]
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['--frame-id', 'map', '--child-frame-id', 'laser'],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        hostname_arg,
        port_arg,
        tf_node,
        lidar_node
    ])
