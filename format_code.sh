#!/bin/bash

# 格式化所有 C++ 源文件和头文件
echo "开始格式化 C++ 代码..."

# 查找并格式化 .cpp 文件
echo "正在格式化 .cpp 文件..."
find . -name "*.cpp" -not -path "./explorer/or-tools/*" | while read file; do
    echo "格式化: $file"
    clang-format -i "$file"
done

# 查找并格式化 .h 文件
echo "正在格式化 .h 文件..."
find . -name "*.h" -not -path "./explorer/or-tools/*" | while read file; do
    echo "格式化: $file"
    clang-format -i "$file"
done

# 查找并格式化 .hpp 文件
echo "正在格式化 .hpp 文件..."
find . -name "*.hpp" -not -path "./explorer/or-tools/*" | while read file; do
    echo "格式化: $file"
    clang-format -i "$file"
done

echo "格式化完成!"