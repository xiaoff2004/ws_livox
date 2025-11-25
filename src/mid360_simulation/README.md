# Mid-360 激光雷达仿真包

这个包提供了 Livox Mid-360 激光雷达在 baselink 上的 Gazebo 仿真环境。

## 功能特性

- 完整的 Mid-360 激光雷达仿真
- 激光雷达安装在机器人 baselink 上
- 预配置的 RViz 可视化
- 简单的测试世界环境
- 完整的 ROS2 Humble 支持

## 包结构

```
mid360_simulation/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # 包依赖配置
├── README.md                   # 使用说明
├── launch/
│   └── mid360_simulation.launch.py  # 主启动文件
├── urdf/
│   └── mid360_robot.xacro     # 机器人描述文件
├── rviz/
│   └── mid360_simulation.rviz # RViz 配置文件
└── world/
    └── simple_world.world     # 测试世界文件
```

## 依赖

确保你已经安装了以下依赖：

```bash
# ROS2 Humble 基础包
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2

# Livox 仿真包（确保在同一工作空间中）
# ros2_livox_simulation 包
```

## 使用方法

### 1. 构建工作空间

```bash
cd ~/ws_livox
colcon build
source install/setup.bash
```

### 2. 启动仿真

```bash
# 启动完整仿真（包括 Gazebo 和 RViz）
ros2 launch mid360_simulation mid360_simulation.launch.py

# 只启动 Gazebo（不启动 RViz）
ros2 launch mid360_simulation mid360_simulation.launch.py use_rviz:=false

# 启动仿真并开启激光可视化（在 Gazebo 中显示激光束）
ros2 launch mid360_simulation mid360_simulation.launch.py laser_visualize:=true

# 启动仿真但关闭激光可视化
ros2 launch mid360_simulation mid360_simulation.launch.py laser_visualize:=false

# 组合参数使用
ros2 launch mid360_simulation mid360_simulation.launch.py use_rviz:=true laser_visualize:=true world:=/path/to/your/world.world
```

### 3. 查看激光雷达数据

启动后，Mid-360 激光雷达会发布点云数据到以下话题：

```bash
# 查看激光雷达点云数据
ros2 topic echo /livox/lidar

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /livox/lidar
```

### 4. 参数配置

Launch 文件支持以下参数：

- `use_sim_time`: 是否使用仿真时间（默认：true）
- `use_rviz`: 是否启动 RViz（默认：true）  
- `world`: 世界文件路径（默认：simple_world.world）
- `rviz_config`: RViz 配置文件路径
- `laser_visualize`: 是否在 Gazebo 中显示激光束可视化（默认：false）

#### 激光可视化说明

`laser_visualize` 参数控制是否在 Gazebo 仿真环境中显示激光束：

- `true`: 显示红色激光束，便于调试和演示
- `false`: 隐藏激光束，提高仿真性能

**注意**: 激光可视化仅影响 Gazebo 中的视觉效果，不影响点云数据的发布。

## 机器人描述

机器人包含以下组件：

- **base_link**: 机器人基座（蓝色立方体，0.3x0.3x0.1m）
- **lidar_mount**: 激光雷达支架（灰色圆柱体）
- **mid360_lidar**: Mid-360 激光雷达传感器

激光雷达安装在基座上方约 0.125m 处。

## 可视化

### RViz 配置

RViz 配置包含：

- 机器人模型显示
- 激光雷达点云可视化（彩色强度映射）
- TF 坐标系显示
- 网格参考

### Gazebo 激光可视化

通过 `laser_visualize` 参数，你可以控制在 Gazebo 中是否显示激光束：

```bash
# 显示激光束（调试模式）
ros2 launch mid360_simulation mid360_simulation.launch.py laser_visualize:=true

# 隐藏激光束（正常模式，性能更好）
ros2 launch mid360_simulation mid360_simulation.launch.py laser_visualize:=false
```

当激光可视化开启时，你会在 Gazebo 中看到从激光雷达发出的红色激光束，这对于理解激光雷达的扫描范围和调试非常有用。

## 自定义配置

### 修改激光雷达位置

编辑 `urdf/mid360_robot.xacro` 文件中的 `origin` 标签：

```xml
<xacro:mid360 
  name="mid360_lidar" 
  parent="lidar_mount" 
  topic="/livox/lidar">
  <origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- 修改这里 -->
</xacro:mid360>
```

### 修改激光雷达参数

Mid-360 的参数在 `ros2_livox_simulation` 包中的 `urdf/mid360.xacro` 文件中定义。

### 添加新的世界环境

在 `world/` 目录下创建新的 `.world` 文件，然后使用 `world` 参数指定。

## 故障排除

### 1. 找不到 ros2_livox_simulation 包

确保 `ros2_livox_simulation` 包在同一工作空间中并已正确构建。

### 2. Gazebo 插件错误

确保 Gazebo 插件路径正确设置，检查环境变量：

```bash
echo $GAZEBO_PLUGIN_PATH
```

### 3. 没有点云数据

检查激光雷达话题是否正常发布：

```bash
ros2 topic hz /livox/lidar
ros2 node info /gazebo
```

### 4. 看不到激光束可视化

如果在 Gazebo 中看不到激光束：

1. 确保启动时设置了 `laser_visualize:=true`
2. 检查是否有障碍物供激光雷达扫描
3. 在 Gazebo 中调整视角，激光束为红色细线
4. 验证激光雷达是否正常工作：
   ```bash
   ros2 topic echo /livox/lidar --once
   ```

## 开发者信息

- 包名: `mid360_simulation`
- 版本: 1.0.0
- 许可证: Apache-2.0
- ROS2 发行版: Humble

## 相关包

- `ros2_livox_simulation`: Livox 激光雷达仿真核心包
- `pb_rm_simulation`: RoboMaster 仿真环境包