cmake_minimum_required(VERSION 3.0.2)
project(track_lidar_front_end)


################################################
### 设置编译器标准为C++14,编译类型为Release
################################################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)


################################################
### 找到库依赖
################################################
find_package(PCL 1.11 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  nav_msgs
  jsk_recognition_msgs
  lidar_msgs
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
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})


################################################
### 将库文件存入TARGET_LIBRARIES变量
################################################
set(TARGET_LIBRARIES "")
list(APPEND TARGET_LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

################################################
###  可执行节点
################################################
add_executable(${PROJECT_NAME}_node src/track_lidar_front_end_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${TARGET_LIBRARIES})
