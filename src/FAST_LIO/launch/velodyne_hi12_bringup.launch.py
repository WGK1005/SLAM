"""VLP-16 + 超核 HI12 + FAST-LIO 一键 bringup（FAST-LIO ros2 分支 / ROS 2 Humble）

顺序：velodyne_driver(原始包) → velodyne_transform(点云+逐点时间) → hipnuc_imu(IMU) → fastlio_mapping

用法：
  ros2 launch fast_lio velodyne_hi12_bringup.launch.py
  ros2 launch fast_lio velodyne_hi12_bringup.launch.py rviz:=false start_imu:=false
  ros2 launch fast_lio velodyne_hi12_bringup.launch.py imu_port:=/dev/ttyUSB0 imu_baudrate:=230400

★ IMU 实测工作在 230400（设备输出率 200Hz），不是 115200：
  115200 带宽只有 ~11.5KB/s，200Hz 会溢出，现象是
  "bytes received but no valid frames"（很容易误判成接线问题）。

注意：本 launch 里两个驱动的关键参数写在 config/velodyne_driver.yaml：
  timestamp_first_packet: true（雷达 scan 时间戳取第一个包）
  organize_cloud: false（避免 NaN 填充点进入 FAST-LIO）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fastlio_share = get_package_share_directory('fast_lio')
    velodyne_share = get_package_share_directory('velodyne_pointcloud')

    driver_params = os.path.join(fastlio_share, 'config', 'velodyne_driver.yaml')
    calibration = os.path.join(velodyne_share, 'params', 'VLP16db.yaml')

    rviz = LaunchConfiguration('rviz')
    start_imu = LaunchConfiguration('start_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    config_file = LaunchConfiguration('config_file')

    actions = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('start_imu', default_value='true',
                              description='是否启动 hipnuc_imu 串口驱动'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0',
                              description='HI12 串口设备（usbipd attach 后通常是 /dev/ttyUSB0）'),
        DeclareLaunchArgument('imu_baudrate', default_value='230400',
                              description='必须与 HI12 实际配置一致！当前设备：200Hz / 230400（CHCenter 里确认）'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link',
                              description='IMU 机体坐标系名字'),
        DeclareLaunchArgument('config_file', default_value='velodyne.yaml',
                              description='fast_lio 的配置文件（位于 fast_lio/config）'),
        DeclareLaunchArgument('fix_timestamp', default_value='true',
                              description='是否启动时间戳规范化节点（消除雷达内部时钟跳变，见 tools/timestamp_normalizer.py）'),
        DeclareLaunchArgument('start_mapping', default_value='true',
                              description='是否启动 FAST-LIO 主节点。录动态包时设 false',
                              choices=['true', 'false']),
    ]

    # 1) Velodyne 驱动：发布 /velodyne_packets
    actions.append(Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='both',
        parameters=[driver_params],
    ))

    # 2) 点云转换：发布 /velodyne_points（字段 x,y,z,intensity,ring,time）
    actions.append(Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='both',
        parameters=[driver_params, {'calibration': calibration}],
    ))

    # 2.5) 时间戳规范化：用主机时钟给点云重打戳（/velodyne_points → /velodyne_points_norm）
    #      VLP-16 没接 GPS/PPS，驱动的时间戳由雷达内部自由运行时钟重建，实测 100s 内倒退 6 次（最大 1.75s），
    #      会让 FAST-LIO 打 "lidar loop back" 并破坏 LiDAR↔IMU 的时间对齐。
    # ★ 路径解析踩坑记录（2026-10-17）：
    #   os.path.realpath 必须作用在 __file__ 本身上，才能穿透 install/ 里的符号链接。
    #   如果写成 realpath(join(dirname(abspath(__file__)), '..','..','..','tools',...))，
    #   那么 '..' 在 join 阶段就已经离开 launch/ 目录，符号链接根本不会被解析，
    #   结果算成 install/fast_lio/tools/... → 找不到脚本 → 规范化节点不启动
    #   → FAST-LIO 订阅的 /velodyne_points_norm 无人发布 → 一帧数据都收不到、建不出图。
    _here_src = os.path.dirname(os.path.realpath(__file__))          # 真实源码路径（穿透符号链接）
    _here_raw = os.path.dirname(os.path.abspath(__file__))           # install 里的路径
    _cands = [
        os.path.join(_here_src, '..', '..', '..', 'tools', 'timestamp_normalizer.py'),
        os.path.join(_here_raw, '..', '..', '..', '..', '..', 'tools', 'timestamp_normalizer.py'),
        os.path.expanduser('~/fast_lio_ws/tools/timestamp_normalizer.py'),
    ]
    norm_script = next((os.path.realpath(c) for c in _cands if os.path.exists(c)), _cands[0])
    if os.path.exists(norm_script):
        actions.append(ExecuteProcess(
            cmd=['python3', norm_script, '--in', '/velodyne_points', '--out', '/velodyne_points_norm'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('fix_timestamp')),
        ))
    else:
        actions.append(LogInfo(msg=(
            f'[bringup] ❌ 未找到时间戳规范化脚本，尝试过: {_cands}\n'
            f'[bringup]    但 config 里 lid_topic 是 /velodyne_points_norm，'
            f'不启动它 = FAST-LIO 收不到任何点云、建不出图！\n'
            f'[bringup]    修法: 确认 ~/fast_lio_ws/tools/timestamp_normalizer.py 存在，'
            f'或用 fix_timestamp:=false 并在配置里把 lid_topic 改成 /velodyne_points')))

    # 3) 超核 HI12 IMU（官方 hipnuc_imu 包，发布 /imu/data）；没编译就跳过
    try:
        hipnuc_share = get_package_share_directory('hipnuc_imu')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hipnuc_share, 'launch', 'serial.launch.py')),
            launch_arguments={
                'port': imu_port,
                'baudrate': imu_baudrate,
                'frame_id': imu_frame_id,
            }.items(),
            condition=IfCondition(start_imu),
        ))
    except Exception:
        actions.append(LogInfo(
            msg='[bringup] 未找到 hipnuc_imu 包，跳过 IMU 驱动（单独启动即可，注意话题要是 /imu/data）'))

    # 4) FAST-LIO 主节点（复用上游 mapping.launch.py，换配置为 velodyne.yaml）
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={'config_file': config_file, 'rviz': rviz}.items(),
        condition=IfCondition(LaunchConfiguration('start_mapping')),
    ))

    return LaunchDescription(actions)
