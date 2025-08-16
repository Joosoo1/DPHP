cmake_minimum_required(VERSION 3.1)
project(explorer)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Fix PCL issue with NrDims
add_compile_definitions(PCL_NO_PRECOMPILE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PROJECT_DEPENDENCIES
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        tf
        tf_conversions
        pcl_ros
        pcl_conversions
        dynamic_reconfigure
        message_generation
        cv_bridge
)

find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# Include external dependencies
# Set OR-Tools paths
set(OR_TOOLS_ROOT ${PROJECT_SOURCE_DIR}/or-tools)
set(OR_TOOLS_INCLUDE_DIRS ${OR_TOOLS_ROOT}/include)
# Include absl headers for OR-Tools
include_directories(${OR_TOOLS_ROOT}/include)
link_directories(${OR_TOOLS_ROOT}/lib)

set(OR_TOOLS_LIBRARIES
        ${OR_TOOLS_ROOT}/lib/libortools.so
        ${OR_TOOLS_ROOT}/lib/libprotobuf.so
        ${OR_TOOLS_ROOT}/lib/libglog.so
        ${OR_TOOLS_ROOT}/lib/libgflags.so
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OR_TOOLS_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${OR_TOOLS_ROOT}/lib)

# Include project header files
include_directories(include/explorer)
include_directories(include)
include_directories(or-tools/include)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
add_message_files(
        FILES
        DualState.msg
        DualStateArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
        FILES
        PlannerTrigger.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        nav_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        tf
        tf_conversions
        pcl_ros
        pcl_conversions
        dynamic_reconfigure
        message_runtime
        cv_bridge
        DEPENDS PCL OpenCV GSL EIGEN3
)

# Build explorer library
add_library(${PROJECT_NAME}
        src/exploration_path.cpp
        src/explorer_visualizer.cpp
        src/graph.cpp
        src/grid_world.cpp
        src/keypose_graph.cpp
        src/lidar_model.cpp
        src/local_coverage_planner.cpp
        src/navigationBoundary.cpp
        src/planning_env.cpp
        src/pointcloud_manager.cpp
        src/rolling_grid.cpp
        src/rolling_occupancy_grid.cpp
        src/sensor_coverage_planner_ground.cpp
        src/tsp_solver.cpp
        src/misc_utils.cpp
        src/pointcloud_utils.cpp
        src/viewpoint.cpp
        src/viewpoint_manager.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GSL_LIBRARIES}
        ${OR_TOOLS_LIBRARIES}
)

# Explorer node
add_executable(explorer_node
        src/explorer_node.cpp)
target_link_libraries(explorer_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OR_TOOLS_LIBRARIES}
)

## Rename C++ executable without namespace
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(graph_test test/graph_test.cpp)
    target_link_libraries(graph_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    catkin_add_gtest(tsp_solver_test test/tsp_solver_test.cpp)
    target_link_libraries(tsp_solver_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main ${OR_TOOLS_LIBRARIES})

    catkin_add_gtest(keypose_graph_test test/keypose_graph_test.cpp)
    target_link_libraries(keypose_graph_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    catkin_add_gtest(exploration_path_test test/exploration_path_test.cpp)
    target_link_libraries(exploration_path_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    catkin_add_gtest(planning_env_test test/planning_env_test.cpp)
    target_link_libraries(planning_env_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    # 添加集成测试
    catkin_add_gtest(explorer_integration_test test/explorer_integration_test.cpp)
    target_link_libraries(explorer_integration_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)


    # 添加视点管理器测试
    catkin_add_gtest(viewpoint_manager_test test/viewpoint_manager_test.cpp)
    target_link_libraries(viewpoint_manager_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    # 添加栅格世界综合测试
    catkin_add_gtest(grid_world_comprehensive_test test/grid_world_comprehensive_test.cpp)
    target_link_libraries(grid_world_comprehensive_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)

    # 添加视点综合测试
    catkin_add_gtest(viewpoint_comprehensive_test test/viewpoint_comprehensive_test.cpp)
    target_link_libraries(viewpoint_comprehensive_test ${PROJECT_NAME} ${catkin_LIBRARIES} gtest gtest_main)
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#############################################
## Declare ROS tests
#############################################

# 添加集成测试到rostest
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/integration_test.launch)
endif ()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)