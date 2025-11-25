#!/usr/bin/env python3
"""
Mid-360 激光雷达仿真启动文件
用于启动 Gazebo 仿真环境，包含 Mid-360 激光雷达的机器人
"""

import os

# ROS2 包管理和路径相关导入
from ament_index_python.packages import get_package_share_directory

# Launch 系统核心模块导入
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    """
    生成 Mid-360 仿真 Launch 描述文件
    
    Returns:
        LaunchDescription: 完整的 launch 配置
    """
    # 获取包的共享目录路径
    pkg_dir = get_package_share_directory('mid360_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 指定 xacro 文件路径，用于生成机器人描述
    robot_description_content = Command(['xacro ', os.path.join(
        pkg_dir, 'urdf', 'mid360_robot.xacro')])

    # AprilTag 独立模型描述 (通过xacro处理)
    apriltag_description_content = Command(['xacro ', os.path.join(
        pkg_dir, 'urdf', 'apriltag_standalone.xacro')])

    # 创建 launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')        # 是否使用仿真时间
    use_rviz = LaunchConfiguration('use_rviz')              # 是否启动 RViz
    world_file = LaunchConfiguration('world')               # 世界文件路径
    robot_description = LaunchConfiguration('robot_description')  # 机器人描述

    # 声明 launch 参数：是否使用仿真时间
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # 声明 launch 参数：是否启动 RViz
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2'
    )



    # 声明 launch 参数：世界文件
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'world', 'simple_world.world'),
        description='Full path to world file to load'
    )

    # 声明 launch 参数：机器人描述
    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='Robot description'
    )

    # 声明 launch 参数：RViz 配置文件路径
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'mid360_simulation.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    # 定义 Launch 动作节点
    
    # 启动 Gazebo 服务器（物理仿真引擎）
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # 启动 Gazebo 客户端（图形界面）
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 启动关节状态发布器节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,        # 使用仿真时间
            'robot_description': ParameterValue(robot_description_content, value_type=str)  # 机器人描述参数
        }],
        output='screen'
    )

    # 启动机器人状态发布器节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,        # 使用仿真时间
            'robot_description': ParameterValue(robot_description_content, value_type=str)  # 正确包装机器人描述参数
        }],
        output='screen'
    )

    # 在 Gazebo 中生成机器人实体
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'mid360_robot',           # 实体名称
            '-topic', 'robot_description',       # 机器人描述话题
            '-x', '0.0',                        # 初始 X 位置
            '-y', '0.0',                        # 初始 Y 位置
            '-z', '0.5',                        # 初始 Z 位置（稍微抬高）
            '-R', '0.0',                        # 初始 Roll 角
            '-P', '0.0',                        # 初始 Pitch 角
            '-Y', '0.0'                         # 初始 Yaw 角
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 在 Gazebo 中生成 AprilTag 实体
    spawn_apriltag_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_apriltag',
        arguments=[
            '-entity', 'apriltag_board',        # 实体名称
            '-topic', '/apriltag_description',  # 从话题读取
            '-x', '3.0',                        # 初始 X 位置 (机器人前方3米)
            '-y', '0.0',                        # 初始 Y 位置
            '-z', '1.0',                        # 初始 Z 位置 (提高到1米,更明显)
            '-R', '0.0',                        # 初始 Roll 角
            '-P', '0.0',                        # 初始 Pitch 角
            '-Y', '0.0'                         # 初始 Yaw 角
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # AprilTag描述发布器
    apriltag_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='apriltag_state_publisher',
        namespace='apriltag',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(apriltag_description_content, value_type=str)
        }],
        remappings=[('robot_description', '/apriltag_description')],
        output='screen'
    )

    # 启动 RViz2 可视化工具（可选）
    rviz_node = Node(
        condition=IfCondition(use_rviz),         # 只有在 use_rviz 参数为 true 时才启动
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],  # 指定 RViz 配置文件
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 创建 launch 描述对象并添加所有动作
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)     # 仿真时间参数
    ld.add_action(declare_use_rviz_cmd)         # RViz 启动参数
    ld.add_action(declare_world_cmd)            # 世界文件参数
    ld.add_action(declare_robot_description_cmd) # 机器人描述参数
    ld.add_action(declare_rviz_config_cmd)      # RViz 配置文件参数
    
    # 添加 Gazebo 相关节点
    ld.add_action(gazebo_server_launch)         # Gazebo 服务器
    ld.add_action(gazebo_client_launch)         # Gazebo 客户端
    
    # 添加机器人相关节点
    ld.add_action(joint_state_publisher_node)   # 关节状态发布器
    ld.add_action(robot_state_publisher_node)   # 机器人状态发布器
    ld.add_action(apriltag_state_publisher_node) # AprilTag 描述发布器
    ld.add_action(spawn_entity_node)            # 机器人实体生成器
    ld.add_action(spawn_apriltag_node)          # AprilTag 实体生成器

    # 添加可视化工具
    ld.add_action(rviz_node)                    # RViz2 可视化工具

    return ld