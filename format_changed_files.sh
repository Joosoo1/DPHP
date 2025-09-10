#!/bin/bash

# 只对已暂存的 C++ 文件进行格式化
echo "开始格式化已修改的 C++ 文件..."

# 定义已修改的 C++ 文件列表
cpp_files=(
    "explorer/src/grid_world.cpp"
    "explorer/src/sensor_coverage_planner_ground.cpp"
    "explorer/src/viewpoint.cpp"
    "explorer/src/viewpoint_manager.cpp"
)

header_files=(
    "explorer/include/explorer/grid_world.h"
    "explorer/include/explorer/sensor_coverage_planner_ground.h"
    "explorer/include/explorer/viewpoint.h"
    "explorer/include/explorer/viewpoint_manager.h"
)

# 格式化 .cpp 文件
echo "正在格式化 .cpp 文件..."
for file in "${cpp_files[@]}"; do
    if [ -f "$file" ]; then
        echo "格式化: $file"
        clang-format -i "$file"
    else
        echo "文件不存在: $file"
    fi
done

# 格式化 .h 文件
echo "正在格式化 .h 文件..."
for file in "${header_files[@]}"; do
    if [ -f "$file" ]; then
        echo "格式化: $file"
        clang-format -i "$file"
    else
        echo "文件不存在: $file"
    fi
done

echo "格式化完成!"