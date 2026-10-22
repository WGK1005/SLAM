"""open3d_loc 适配版：Velodyne VLP-16 + 我们自己的 FAST-LIO

与原版 open3d_loc_g1.launch.py 的区别：
  1. 地图路径可传参（原版写死 /home/sax/GO2_Localization_ROS2/...）
  2. 话题 remap：/Odometry_loc <- /Odometry、/cloud_registered_1 <- /cloud_registered
     （我们的 FAST-LIO 发这两个名字，不改代码，只在 launch 里 remap）
  3. TF 链多接一环：FAST-LIO 发的是 camera_init -> body，而 open3d_loc 内部会
     lookupTransform("base_link","imu_link") 和 ("motion_link","base_link")，
     所以要把 body 接到 imu_link 上。完整链：
         map -[open3d_loc]-> odom -[静态]-> camera_init -[FAST-LIO]-> body
             -[静态]-> imu_link -[静态]-> base_link -[静态]-> motion_link
  4. 修正原版的两个体素参数（coarse 0.01 / fine 0.2 是笔误，ROS1 版是 0.15 / 0.1）
     voxelsize_coarse 只影响 /map 可视化，voxelsize_fine 才是配准分辨率

用法：
    ros2 launch open3d_loc loc_vlp16.launch.py
    ros2 launch open3d_loc loc_vlp16.launch.py map_file:=$HOME/maps/pc_base2.pcd
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('open3d_loc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    config_file = PathJoinSubstitution([share, 'config', 'loc_param_vlp16.yaml'])

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='回放 bag 且加了 --clock 时才设 true'),
        DeclareLaunchArgument('map_file',
                              default_value=os.path.expanduser('~/maps/pc_base2.pcd'),
                              description='先验点云地图（binary PCD 或 PLY）'),
    ]

    # ── TF 链（全部是静态的，动态部分由 open3d_loc 和 FAST-LIO 各自发）──
    tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_init2odom',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']),
        # ★ 我们多出来的这一环：FAST-LIO 的 body(IMU系) -> open3d_loc 期望的 imu_link
        Node(package='tf2_ros', executable='static_transform_publisher', name='body2imulink',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='imulink2baselink',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'imu_link', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_center',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'motion_link']),
    ]

    loc_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        remappings=[
            ('/Odometry_loc', '/Odometry'),                # ← FAST-LIO 里程计
            ('/cloud_registered_1', '/cloud_registered'),  # ← FAST-LIO 当前帧（世界系）
        ],
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.15,   # 只用于 /map 可视化
                'voxelsize_fine': 0.1,      # ★ 配准分辨率
                'threshold_fitness': 0.4,        # 常规配准阈值，超过才更新 odom->map
                'threshold_fitness_init': 0.5,   # 初始化配准阈值
                'loc_frequence': 2.5,            # 定位频率 Hz
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,            # 当前子地图半径
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # ★ 延时 3 秒再起定位节点：它初始化时会 lookupTransform("motion_link","base_link")，
    #   如果静态 TF 还没发布完，查询失败会让内部矩阵变成 NaN（已同时改成默认单位阵兜底）
    delayed_loc = TimerAction(period=3.0, actions=[loc_node])

    return LaunchDescription(args + tf_nodes + [delayed_loc])
