cmake_minimum_required(VERSION 3.0.2)
project(track_lidar_front_end)


################################################
### 设置编译器标准为C++14,编译类型为Release
################################################
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)


################################################
### 找到库依赖
################################################
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
  jsk_recognition_msgs
  lidar_front_end_msgs
)

catkin_package(
  CATKIN_DEPENDS 
)


################################################
### PCL库
################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


################################################
### 设置工作根目录
################################################
set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
set(NODE_NAME ${PROJECT_NAME}_node)
configure_file(
  ${PROJECT_SOURCE_DIR}/include/PATH.h.in
  ${PROJECT_SOURCE_DIR}/include/PATH.h
)
configure_file(
  ${PROJECT_SOURCE_DIR}/launch/${PROJECT_NAME}_node.launch.in
  ${PROJECT_SOURCE_DIR}/launch/${PROJECT_NAME}_node.launch
)


################################################
### 将库文件存入TARGET_LIBRARIES变量
################################################
set(TARGET_LIBRARIES "")
list(APPEND TARGET_LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_LIBRARIES}
)

################################################
###  可执行节点
################################################
add_executable(${PROJECT_NAME}_node src/track_lidar_front_end_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${TARGET_LIBRARIES})
