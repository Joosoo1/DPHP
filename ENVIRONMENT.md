# DPHP Planner 开发环境管理指南

## 简介

本项目提供了一个环境管理脚本 (`manage_env.sh`)，用于解决在使用 Anaconda 环境时可能出现的库冲突问题。该脚本可以隔离 Anaconda 环境，确保在编译 ROS 项目时使用系统默认的库。

## 使用方法

### 1. 查看帮助信息

```bash
./manage_env.sh help
```

### 2. 隔离环境并编译

```bash
./manage_env.sh build
```

或者直接运行（默认行为为 build）：

```bash
./manage_env.sh
```

### 3. 清理构建文件

```bash
./manage_env.sh clean
```

### 4. 清理并重新编译

```bash
./manage_env.sh clean-build
```

### 5. 查看环境信息

```bash
./manage_env.sh env
```

## 工作原理

脚本通过以下方式工作：

1. **识别 Anaconda 路径**：检测 PATH 和 LD_LIBRARY_PATH 中包含 "anaconda" 的路径
2. **移除 Anaconda 路径**：从环境变量中移除 Anaconda 相关路径
3. **清理环境变量**：取消可能干扰的 Conda 环境变量
4. **执行编译**：在清理后的环境中使用 catkin build 编译项目

## 注意事项

1. 该脚本仅临时修改当前 shell 会话的环境变量，不会影响系统的永久配置
2. 如果需要恢复 Anaconda 环境，请重新打开终端或 source activate 相应的环境
3. 脚本假设项目位于 `/home/joosoo/dphp_planner` 目录下
4. 如需修改项目路径，请编辑脚本中的相关路径配置

## 故障排除

如果遇到编译问题，请按以下步骤排查：

1. 确认脚本具有执行权限：`chmod +x manage_env.sh`
2. 检查环境信息：`./manage_env.sh env`
3. 尝试清理并重新编译：`./manage_env.sh clean-build`
4. 如果仍有问题，请检查系统库是否完整安装

## 贡献

如有改进建议或发现问题，请提交 Issue 或 Pull Request。