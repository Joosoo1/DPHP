#!/bin/bash
# Explorer项目单元测试运行脚本
# 这个脚本用于编译和运行Explorer项目的所有单元测试

echo "=========================================="
echo "Explorer项目单元测试运行脚本"
echo "=========================================="

# 设置工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "工作目录: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# 检查是否在catkin工作空间中
if [ ! -f "src/Explorer/explorer/CMakeLists.txt" ]; then
  echo "错误: 未找到Explorer项目CMakeLists.txt文件"
  echo "请在catkin工作空间中运行此脚本"
  exit 1
fi

echo "1. 清理之前的构建..."
rm -rf build devel logs 2>/dev/null

echo "2. 创建catkin工作空间..."
mkdir -p build
cd build

echo "3. 配置CMake..."
cmake .. -DCATKIN_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

if [ $? -ne 0 ]; then
  echo "错误: CMake配置失败"
  exit 1
fi

echo "4. 编译项目..."
make -j$(nproc)

if [ $? -ne 0 ]; then
  echo "错误: 编译失败"
  exit 1
fi

echo "5. 编译测试..."
make tests

if [ $? -ne 0 ]; then
  echo "错误: 测试编译失败"
  exit 1
fi

echo "6. 运行单元测试..."
echo "=========================================="

# 设置环境变量
source devel/setup.bash

# 运行所有测试
ctest --verbose --output-on-failure

echo "=========================================="

# 单独运行新增的核心模块测试
echo "7. 运行新增的核心模块测试..."

# 定义测试列表
declare -a core_tests=(
  "sensor_coverage_planner_test"
  "viewpoint_manager_test"
  "grid_world_comprehensive_test"
  "local_coverage_planner_comprehensive_test"
  "viewpoint_comprehensive_test"
)

# 运行每个核心测试
for test_name in "${core_tests[@]}"; do
  echo "------------------------------------------"
  echo "运行测试: $test_name"

  if [ -f "devel/lib/explorer/$test_name" ]; then
    ./devel/lib/explorer/$test_name
    if [ $? -eq 0 ]; then
      echo "✓ $test_name 通过"
    else
      echo "✗ $test_name 失败"
    fi
  else
    echo "⚠ 未找到测试可执行文件: $test_name"
  fi
done

echo "=========================================="
echo "测试运行完成"

# 生成测试报告
echo "8. 生成测试报告..."
if command -v gcovr &>/dev/null; then
  echo "生成代码覆盖率报告..."
  gcovr --xml-pretty --exclude-unreachable-branches --print-summary -o coverage.xml
  gcovr --html-details --exclude-unreachable-branches -o coverage.html
  echo "覆盖率报告已生成: coverage.html"
fi

echo "=========================================="
echo "测试运行脚本完成"
echo "=========================================="

# 显示测试结果统计
echo "测试结果统计:"
echo "- 已添加的测试文件: 5个"
echo "- 传感器覆盖规划器测试: sensor_coverage_planner_test.cpp"
echo "- 视点管理器测试: viewpoint_manager_test.cpp"
echo "- 栅格世界综合测试: grid_world_comprehensive_test.cpp"
echo "- 局部覆盖规划器综合测试: local_coverage_planner_comprehensive_test.cpp"
echo "- 视点综合测试: viewpoint_comprehensive_test.cpp"
echo ""
echo "测试覆盖的核心功能:"
echo "- 传感器覆盖规划器的主要功能"
echo "- 视点管理器的状态管理和坐标转换"
echo "- 栅格世界的坐标系统和状态管理"
echo "- 局部覆盖规划器的路径规划算法"
echo "- 视点类的状态管理和覆盖计算"
