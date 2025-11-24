#!/usr/bin/env python3
"""
RoboMaster 仿真环境启动文件
用于启动 Gazebo 仿真环境，支持 RMUC 和 RMUL 两种比赛场景
"""

import os

# ROS2 包管理和路径相关导入
from ament_index_python.packages import get_package_share_directory, get_package_share_path

# Launch 系统核心模块导入
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable

# 世界类型枚举定义
class WorldType:
    RMUC = 'RMUC'  # RoboMaster 大学生机器人大赛场地
    RMUL = 'RMUL'  # RoboMaster 大学生机器人联盟场地

def get_world_config(world_type):
    """
    获取不同比赛场地的配置信息
    
    Args:
        world_type: 世界类型 (RMUC 或 RMUL)
    
    Returns:
        dict: 包含机器人初始位置和世界文件路径的配置字典
    """
    world_configs = {
        WorldType.RMUC: {
            'x': '6.35',      # 机器人初始 X 坐标
            'y': '7.6',       # 机器人初始 Y 坐标
            'z': '0.2',       # 机器人初始 Z 坐标
            'yaw': '0.0',     # 机器人初始偏航角
            'world_path': 'world/RMUC2024_world/RMUC2024_world.world'  # RMUC 世界文件路径
        },
        WorldType.RMUL: {
            'x': '4.3',       # 机器人初始 X 坐标
            'y': '3.35',      # 机器人初始 Y 坐标
            'z': '1.16',      # 机器人初始 Z 坐标
            'yaw': '0.0',     # 机器人初始偏航角
            'world_path': 'world/RMUL2024_world/RMUL2024_world.world'  # RMUL 世界文件路径
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'  # 带动态障碍物的世界
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    """
    生成 Launch 描述文件
    
    Returns:
        LaunchDescription: 完整的 launch 配置
    """
    # 获取包的共享目录路径
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 指定 xacro 文件路径，用于生成机器人描述
    default_robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # 创建 launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')        # 是否使用仿真时间
    use_rviz = LaunchConfiguration('rviz', default='false')  # 是否启动 RViz
    robot_description = LaunchConfiguration('robot_description')  # 机器人描述

    # 设置 Gazebo 插件路径（用于障碍物插件）
    append_enviroment = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(os.path.join(get_package_share_directory('pb_rm_simulation'), 'meshes', 'obstacles', 'obstacle_plugin', 'lib'))
    )

    # 声明 launch 参数：是否使用仿真时间
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # 声明 launch 参数：选择世界类型
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUC,
        description='Choose <RMUC> or <RMUL>'
    )

    # 声明 launch 参数：RViz 配置文件路径
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    # 声明 launch 参数：机器人描述
    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # 定义 Launch 动作节点
    
    # 启动 Gazebo 客户端（图形界面）
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # 启动关节状态发布器节点
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,        # 使用仿真时间
            'robot_description': robot_description  # 机器人描述参数
        }],
        output='screen'
    )

    # 启动机器人状态发布器节点
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,        # 使用仿真时间
            'robot_description': robot_description  # 机器人描述参数
        }],
        output='screen'
    )

    # 启动 RViz2 可视化工具（可选）
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),         # 只有在 rviz 参数为 true 时才启动
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]  # 指定 RViz 配置文件
    )

    def create_gazebo_launch_group(world_type):
        """
        创建特定世界类型的 Gazebo 启动组
        
        Args:
            world_type: 世界类型 (RMUC 或 RMUL)
        
        Returns:
            GroupAction: 包含 Gazebo 服务器和机器人生成的动作组
        """
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),  # 只有当世界类型匹配时才执行
            actions=[
                # 在 Gazebo 中生成机器人实体
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',                    # 实体名称
                        '-topic', 'robot_description',         # 机器人描述话题
                        '-x', world_config['x'],               # 初始 X 位置
                        '-y', world_config['y'],               # 初始 Y 位置
                        '-z', world_config['z'],               # 初始 Z 位置
                        '-Y', world_config['yaw']              # 初始偏航角
                    ],
                ),
                # 启动 Gazebo 服务器（物理仿真引擎）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, world_config['world_path'])}.items(),
                )
            ]
        )

    # 创建不同世界类型的启动组
    bringup_RMUC_cmd_group = create_gazebo_launch_group(WorldType.RMUC)  # RMUC 世界启动组
    bringup_RMUL_cmd_group = create_gazebo_launch_group(WorldType.RMUL)  # RMUL 世界启动组

    # 创建 launch 描述对象并添加所有动作
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(append_enviroment)

    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)     # 仿真时间参数
    ld.add_action(declare_world_cmd)            # 世界类型参数
    ld.add_action(declare_rviz_config_file_cmd) # RViz 配置文件参数
    ld.add_action(declare_robot_description_cmd) # 机器人描述参数
    
    # 添加核心节点
    ld.add_action(gazebo_client_launch)         # Gazebo 客户端
    ld.add_action(start_joint_state_publisher_cmd)  # 关节状态发布器
    ld.add_action(start_robot_state_publisher_cmd)  # 机器人状态发布器
    
    # 添加世界启动组（根据参数选择启动哪个世界）
    ld.add_action(bringup_RMUL_cmd_group) # type: ignore  # RMUL 世界启动组
    ld.add_action(bringup_RMUC_cmd_group) # type: ignore  # RMUC 世界启动组

    # 添加可视化工具（可选启动）
    ld.add_action(start_rviz_cmd)               # RViz2 可视化工具

    return ld
