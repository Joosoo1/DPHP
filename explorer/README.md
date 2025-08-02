# Explorer

Explorer是一个用于动态环境探索与路径规划的机器人系统，适用于多种复杂场景如校园、森林、车库、隧道等。

## 目录
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [支持场景](#支持场景)
- [安装说明](#安装说明)
- [使用方法](#使用方法)
- [模块介绍](#模块介绍)
- [测试](#测试)
- [许可证](#许可证)

## 功能特性

- 动态环境建模与感知
- 实时路径规划与避障
- 多种地图配置支持（campus, forest, garage 等）
- 点云数据管理与可视化
- TSP路径优化与传感器覆盖规划
- 基于图（Graph）的路径规划方法
- 支持多传感器融合与动态地图更新

## 系统架构

Explorer基于ROS（Robot Operating System）的模块化架构，采用C++实现核心功能。整个系统采用模块化设计，支持快速扩展与集成。

主要组件包括：
- **explorer_node**: 核心探索逻辑与路径规划
- **sensor_coverage_planner**: 传感器覆盖规划
- **pointcloud_manager**: 点云数据管理
- **planning_env**: 规划环境管理
- **graph**: 图搜索相关算法
- **tsp_solver**: TSP路径优化求解器
- **viewpoint**: 视点管理模块

系统采用发布-订阅模式进行模块间通信，使用状态机模式管理路径规划状态。

## 支持场景

Explorer支持多种场景配置：

- Campus（校园）
- Forest（森林）
- Garage（车库）
- Indoor（室内）
- Matterport（建筑物内部）
- Tunnel（隧道）

每种场景都有相应的配置文件，可以根据具体需求进行调整。

## 安装说明

### 依赖项

- ROS（Robot Operating System）
- Catkin构建系统
- PCL（Point Cloud Library）
- OR-Tools（优化算法库）

### 构建步骤

```bash
cd /path/to/your/catkin/workspace
catkin build explorer
```

## 使用方法

### 基本使用

启动默认场景（garage）的探索任务：

```bash
roslaunch explorer explore.launch
```

### 指定场景

启动特定场景的探索任务：

```bash
# 校园场景
roslaunch explorer explore.launch scenario:=campus

# 森林场景
roslaunch explorer explore.launch scenario:=forest

# 隧道场景
roslaunch explorer explore.launch scenario:=tunnel
```

### 其他参数

```bash
# 禁用RVIZ可视化
roslaunch explorer explore.launch rviz:=false

# 启用rosbag记录
roslaunch explorer explore.launch rosbag_record:=true
```

## 模块介绍

Explorer由多个模块组成，每个模块负责特定的功能：

- **exploration_path**: 探索路径管理
- **explorer_node**: 主节点，协调各个模块
- **explorer_visualizer**: 可视化模块
- **graph**: 图搜索相关算法实现
- **grid_world**: 网格世界相关实现
- **keypose_graph**: 关键姿态图
- **lidar_model**: 激光雷达模型
- **local_coverage_planner**: 局部覆盖规划器
- **planning_env**: 规划环境
- **pointcloud_manager**: 点云管理器
- **rolling_grid**: 滚动网格
- **rolling_occupancy_grid**: 滚动占用网格
- **sensor_coverage_planner**: 传感器覆盖规划器
- **tsp_solver**: TSP求解器
- **utils**: 工具函数
- **viewpoint**: 视点相关
- **viewpoint_manager**: 视点管理器

## 测试

项目包含多个单元测试和集成测试：

```bash
# 运行所有测试
rostest explorer integration_test.launch

# 或者运行特定测试
rosrun explorer graph_test
```

测试文件位于[test](test)目录中。

## 许证证

该项目目前标记为TODO license。请在使用前确认许可条款。

## 维护者

- joosoo <joosoo@buct.edu.cn>