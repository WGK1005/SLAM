
#!/usr/bin/env python3
"""
SCU-TDT-V1: 2027 足式机器人挑战赛 障碍赛场地 (14m x 6m, 1:1) Gazebo 仿真 + RViz2 闭环导航

世界与栅格由 tools/generate_rc_obstacle_world.py 生成，坐标原点为场地左下角，
机器人默认出生在启动区1 (场地中心 7.0, 3.0)。

用法：
  ros2 launch /home/wgk/decision_planning/SCU-TDT-V1/launch/gazebo_rc_obstacle.launch.py
  ros2 launch .../gazebo_rc_obstacle.launch.py start_zone:=2     # 从启动区2 (沟壑组合斜坡侧平台) 出生
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT_DIR = "/home/wgk/decision_planning/SCU-TDT-V1"

# 启动区坐标 (m)：启动区1 = 场地中心；启动区2 = 沟壑组合斜坡侧平台顶面 (高 0.36m)
START_ZONES = {
    "1": (7.00, 3.00, 0.05, 0.0),
    "2": (1.10, 1.00, 0.42, 0.0),
}


def _launch_setup(context, *args, **kwargs):
    world_path = os.path.join(ROOT_DIR, "worlds", "rc_obstacle_2027.world")
    urdf_path = os.path.join(ROOT_DIR, "urdf", "unitree_go2_vlp16.urdf.xacro")
    rviz_path = os.path.join(ROOT_DIR, "rviz", "scu_tdt_rmuc.rviz")
    pgm_path = os.path.join(ROOT_DIR, "maps", "rc_obstacle_2027.pgm")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    zone = LaunchConfiguration("start_zone").perform(context)
    sx, sy, sz, syaw = START_ZONES.get(zone, START_ZONES["1"])

    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
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
        arguments=["-entity", "unitree_go2_vlp16", "-topic", "robot_description",
                   "-x", str(sx), "-y", str(sy), "-z", str(sz), "-Y", str(syaw)],
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
            "init_x": sx,
            "init_y": sy,
            "init_yaw": syaw,
            "enable_auto_goal": False,      # 在 RViz2 中用 2D Goal Pose 指点
            "max_speed": 1.50,
            "max_acc": 1.00,
            "robot_radius": 0.25,           # 规则收拢尺寸 800x600mm 内的典型足式机器人
            "squeeze_radius": 0.50,
            "omni_mode": LaunchConfiguration("omni_mode"),
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
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return [gazebo_launch, rsp_node, spawn_entity, static_map_to_odom, nav_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI (gzclient)"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("omni_mode", default_value="true", description="True for omni chassis"),
        DeclareLaunchArgument("start_zone", default_value="1", description="Spawn at start zone 1 or 2"),
        OpaqueFunction(function=_launch_setup),
    ])

