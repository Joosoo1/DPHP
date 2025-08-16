#!/bin/bash

# 脚本功能：用于DPHP Planner项目的开发环境管理
# 提供隔离Anaconda环境的功能，确保编译时不出现库冲突

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示帮助信息
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "选项:"
    echo "  build          - 隔离环境并编译项目"
    echo "  test           - 隔离环境并运行测试"
    echo "  clean          - 清理构建文件"
    echo "  clean-build    - 清理并重新编译"
    echo "  env            - 显示环境信息"
    echo "  help           - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 build       # 隔离环境并编译"
    echo "  $0 test        # 隔离环境并运行测试"
    echo "  $0 clean-build # 清理并重新编译"
}

# 显示环境信息
show_env_info() {
    echo -e "${BLUE}==================== 环境信息 ====================${NC}"
    echo "工作目录: $(pwd)"
    echo "ROS版本: $ROS_DISTRO"
    echo ""
    echo -e "${YELLOW}原始环境变量:${NC}"
    echo "PATH前200字符: ${PATH:0:200}..."
    echo "LD_LIBRARY_PATH前200字符: ${LD_LIBRARY_PATH:0:200}..."
    echo ""
    echo -e "${GREEN}清理后的环境变量:${NC}"
    echo "CLEANED_PATH前200字符: ${CLEANED_PATH:0:200}..."
    echo "CLEANED_LD_LIBRARY_PATH前200字符: ${CLEANED_LD_LIBRARY_PATH:0:200}..."
    echo -e "${BLUE}==================================================${NC}"
}

# 清理Anaconda相关的路径
clean_env_vars() {
    # 清理PATH
    CLEANED_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "anaconda" | tr '\n' ':' | sed 's/:$//' | sed 's/^://' | sed 's/::/:/g')
    
    # 清理LD_LIBRARY_PATH
    CLEANED_LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "anaconda" | tr '\n' ':' | sed 's/:$//' | sed 's/^://' | sed 's/::/:/g')
    
    # 清理PYTHONPATH
    CLEANED_PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "anaconda" | tr '\n' ':' | sed 's/:$//' | sed 's/^://' | sed 's/::/:/g')
    
    # 导出清理后的环境变量
    export PATH="$CLEANED_PATH"
    export LD_LIBRARY_PATH="$CLEANED_LD_LIBRARY_PATH"
    export PYTHONPATH="$CLEANED_PYTHONPATH"
    
    # 取消可能干扰的Python相关环境变量
    unset CONDA_DEFAULT_ENV
    unset CONDA_EXE
    unset CONDA_PREFIX
    unset CONDA_PYTHON_EXE
    unset CONDA_PROMPT_MODIFIER
}

# 清理构建文件
clean_build() {
    echo -e "${YELLOW}正在清理构建文件...${NC}"
    cd /home/joosoo/dphp_planner
    rm -rf build devel install
    echo -e "${GREEN}清理完成${NC}"
}

# 编译项目
build_project() {
    echo -e "${YELLOW}正在编译项目...${NC}"
    cd /home/joosoo/dphp_planner
    
    # 使用catkin build编译
    if catkin build explorer --no-deps; then
        echo -e "${GREEN}编译成功完成！${NC}"
        return 0
    else
        echo -e "${RED}编译失败！${NC}"
        return 1
    fi
}

# 运行测试
run_tests() {
    echo -e "${YELLOW}正在运行测试...${NC}"
    cd /home/joosoo/dphp_planner
    
    # 先编译测试
    if catkin build explorer --no-deps --make-args tests; then
        echo -e "${GREEN}测试编译成功！${NC}"
        
        # 运行测试
        if catkin run_tests explorer --no-deps; then
            echo -e "${GREEN}所有测试通过！${NC}"
            return 0
        else
            echo -e "${RED}部分测试失败！${NC}"
            return 1
        fi
    else
        echo -e "${RED}测试编译失败！${NC}"
        return 1
    fi
}

# 主函数
main() {
    case "$1" in
        build)
            echo -e "${BLUE}==================== 隔离环境并编译 ====================${NC}"
            clean_env_vars
            build_project
            ;;
        test)
            echo -e "${BLUE}==================== 隔离环境并测试 ====================${NC}"
            clean_env_vars
            run_tests
            ;;
        clean)
            echo -e "${BLUE}==================== 清理构建文件 ====================${NC}"
            clean_build
            ;;
        clean-build)
            echo -e "${BLUE}==================== 清理并重新编译 ====================${NC}"
            clean_build
            clean_env_vars
            build_project
            ;;
        env)
            show_env_info
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            if [ -z "$1" ]; then
                echo -e "${BLUE}==================== 隔离环境并编译 ====================${NC}"
                clean_env_vars
                build_project
            else
                echo -e "${RED}未知选项: $1${NC}"
                show_help
                exit 1
            fi
            ;;
    esac
}

# 执行主函数
main "$@"