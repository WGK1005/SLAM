#!/usr/bin/env python3
"""
SCU-TDT-V1: RMUC 2025 3D Gazebo 11 仿真 + VLP-16 & HI12 机器人 + RViz2 三层膨胀闭环导航一键启动文件
用法：
  ros2 launch scu_tdt_v1 gazebo_rmuc_sim.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root_dir = "/home/wgk/decision_planning/SCU-TDT-V1"
    world_path = os.path.join(root_dir, "worlds", "rmuc2025.world")
    urdf_path = os.path.join(root_dir, "urdf", "scu_sentry_vlp16.urdf.xacro")
    rviz_path = os.path.join(root_dir, "rviz", "scu_tdt_rmuc.rviz")
    pgm_path = os.path.join(root_dir, "maps", "rmuc2025.pgm")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    use_gui = LaunchConfiguration("gui", default="true")
    use_rviz = LaunchConfiguration("rviz", default="true")
    omni_mode = LaunchConfiguration("omni_mode", default="true")

    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": use_gui,
            "verbose": "false",
        }.items(),
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "scu_sentry_vlp16",
            "-topic", "robot_description",
            "-x", "2.50",
            "-y", "2.00",
            "-z", "0.05",
            "-Y", "0.0",
        ],
        output="screen",
    )

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    nav_node = Node(
        package="scu_tdt_v1",
        executable="scu_tdt_nav_node",
        name="scu_tdt_nav_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "pgm_map_path": pgm_path,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "z_floor": 0.0,
            "init_x": 2.50,
            "init_y": 2.00,
            "init_yaw": 0.0,
            "auto_goal_x": 25.20,
            "auto_goal_y": 13.00,
            "enable_auto_goal": True,
            "max_speed": 2.20,
            "max_acc": 1.50,
            "robot_radius": 0.25,
            "squeeze_radius": 0.55,
            "omni_mode": omni_mode,
            "odom_topic": "/odom",
            "cloud_topic": "/velodyne_points",
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI (gzclient)"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("omni_mode", default_value="true", description="True for omni chassis, False for diff drive"),
        gazebo_launch,
        rsp_node,
        spawn_entity,
        static_map_to_odom,
        nav_node,
        rviz_node,
    ])

