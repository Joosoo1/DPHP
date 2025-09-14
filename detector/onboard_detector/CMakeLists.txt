cmake_minimum_required(VERSION 3.0.2)
project(onboard_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetDynamicObstacles.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES onboard_detector
   CATKIN_DEPENDS roscpp rospy std_msgs message_generation geometry_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   include/${PROJECT_NAME}/dynamicDetector.cpp
   include/${PROJECT_NAME}/uvDetector.cpp
   include/${PROJECT_NAME}/dbscan.cpp
   include/${PROJECT_NAME}/kalmanFilter.cpp
   include/${PROJECT_NAME}/lidarDetector.cpp
)

# add_executable(${PROJECT_NAME}_node src/onboard_detector_node.cpp)
add_executable(detector_node src/detector_node.cpp)
#add_executable(fake_detector_node src/fake_detector_node.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(detector_node ${PROJECT_NAME})
#target_link_libraries(fake_detector_node ${PROJECT_NAME})
