cmake_minimum_required(VERSION 3.10)
project(mid360_simulation)

## 使用 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 添加编译器警告和错误标志，确保代码质量
add_definitions(-Wall -Werror)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

#######################
## 查找依赖包 ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## 构建 ##
###########

# 当前包主要用于仿真配置，不包含 C++ 源码
# 如果需要添加自定义插件或节点，可以在此处添加

#############
## 测试 ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## 安装 ##
#############

# 安装仿真相关的配置文件和资源
ament_auto_package(
  INSTALL_TO_SHARE
  launch      # Launch 启动文件
  urdf        # 机器人描述文件
  meshes      # 3D 模型文件
  world       # Gazebo 世界文件
  rviz        # RViz 配置文件
)