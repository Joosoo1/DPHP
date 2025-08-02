# Dynamic Explorer

Dynamic Explorer是一个用于动态环境探索与路径规划的机器人系统，适用于多种复杂场景如校园、森林、室内、隧道等。该系统基于ROS（Robot Operating System）开发，采用模块化架构设计，支持快速扩展与集成。

## 目录
- [系统概述](#系统概述)
- [核心模块](#核心模块)
  - [Explorer](#explorer)
  - [Predictor](#predictor)
  - [Visualization Tools](#visualization-tools)
- [系统架构](#系统架构)
- [支持场景](#支持场景)
- [安装说明](#安装说明)
- [使用方法](#使用方法)
- [许可证](#许可证)

## 系统概述

Dynamic Explorer旨在解决在动态环境中实现高效、实时的探索与路径规划问题。该系统具有以下核心功能：

- 动态环境建模与感知
- 实时路径规划与避障
- 多种地图配置支持（campus, forest, garage 等）
- 点云数据管理与可视化
- TSP路径优化与传感器覆盖规划
- 支持多传感器融合与动态地图更新

## 核心模块

### Explorer

Explorer是系统的核心模块，负责探索逻辑与路径规划。它基于ROS的模块化架构，采用C++实现核心功能。

主要特性：
- 基于图（Graph）的路径规划方法
- 实时路径规划与避障
- 传感器覆盖规划
- 点云数据管理
- 多种场景配置支持

目录结构：
```
explorer/
├── config/            # 配置文件（支持campus、forest、garage等多种场景）
├── include/           # 头文件
├── launch/            # 启动文件
├── src/               # 源代码
├── msg/               # 自定义消息
├── srv/               # 自定义服务
└── test/              # 测试文件
```

### Predictor

Predictor是动态环境预测模块，基于Python实现，用于预测环境的动态变化。

主要特性：
- 动态环境预测模型
- 基于Python的机器学习实现
- 与ROS系统集成

目录结构：
```
predictor/
├── models/            # 模型定义
├── scripts/           # Python脚本
│   ├── datatools/     # 数据处理工具
│   ├── models/        # 模型实现
│   ├── rosinterfaces/ # ROS接口
│   ├── training_routines/ # 训练程序
│   └── utils/         # 工具函数
└── package.xml        # 包描述文件
```

### Visualization Tools

Visualization Tools提供实时可视化支持，帮助用户更好地理解系统运行状态。

主要特性：
- 实时绘图与可视化
- 与ROS系统集成
- 支持RVIZ插件

目录结构：
```
visualization_tools/
├── scripts/           # Python可视化脚本
├── src/               # C++源代码
└── launch/            # 启动文件
```

## 系统架构

整个系统采用模块化架构设计，基于ROS（Robot Operating System）构建，使用C++实现核心功能，Python用于脚本和训练接口。

```
+------------------+     +------------------+     +------------------+
|    Explorer      |<--->|    Predictor     |<--->| Visualization    |
| (核心探索逻辑)    |     | (动态环境预测)    |     |   (可视化工具)     |
+------------------+     +------------------+     +------------------+
         ^                        ^                        ^
         |                        |                        |
         +------------------------+------------------------+
                                  |
                            +-----+-----+
                            |    ROS    |
                            | (通信机制) |
                            +-----------+
```

关键技术决策：
- 使用Catkin构建系统进行项目管理
- 模块化设计支持快速扩展与集成
- 基于图（Graph）的路径规划方法
- 支持多传感器融合与动态地图更新
- 架构模式采用发布-订阅模式（ROS通信机制）和状态机模式（用于路径规划状态管理）

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
- Python 3.x（用于predictor模块）

### 构建步骤

```bash
cd /path/to/your/catkin/workspace
catkin build
```

或者针对特定模块构建：

```bash
# 构建explorer模块
catkin build explorer

# 构建predictor模块
catkin build predictor

# 构建visualization_tools模块
catkin build visualization_tools
```

## 使用方法

### 启动完整系统

```bash
roslaunch explorer explore.launch
```

### 指定场景

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

### 启动可视化工具

```bash
# 启动实时绘图工具
rosrun visualization_tools realTimePlot.py
```

## 许可证

- Explorer: TODO license
- Predictor: TODO license
- Visualization Tools: BSD

## 维护者

- Explorer: joosoo <joosoo@buct.edu.cn>
- Predictor: joosoo <joosoo@todo.todo>
- Visualization Tools: Hongbiao Zhu <hongbiaz@cmu.edu>