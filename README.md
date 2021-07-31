# track_lidar_front_end

## 简介

使用`roslaunch`加载参数并运行节点  
一个基于PCL的前端里程计，效果不错，但跑到最后终究还是有明显的误差累计，配合后端的位姿优化有不错的结果  
从老程序改来，有很多不完善的改动及测试中的遗留，更完善的前后端会在`lidar_slam`中完成  

## 依赖

- PCL
- Eigen3
- jsk_recognition_msgs
- [lidar_msgs](https://github.com/Zero-Swangel/lidar_front_end_msgs)

## 项目的构建和运行

### 构建

```bash
cp -r ${PROJECT_SOURCE_DIR} catkin_ws/src
cd catkin_ws/
catkin_make -DCATKIN_WHITELIST_PACKAGES="track_lidar_front_end" -DCMAKE_EXPORT_COMPILE_COMMANDS=YES
```

### 参数

所有参数的设置都保存在 `config/track_lidar_front_end.yaml` , 直接对其进行修改即可，修改之后重新启动节点生效。

|             部分参数               |          解释           |
|-----------------------------------|------------------------|
|local_point_cloud_publish_topic    |局部地图发布话题           |
|global_point_cloud_publish_topic   |全局地图发布话题           |
|bounding_box_publish_topic         |全局锥桶发布话题           |
|output_publish_topic               |前端输出                  |
|pass_through_*_*_preprocess        |截取视野范围              |
|pass_through_*_*_plane             |找地面范围                |

### 运行

会同时启动前端、作弊程序、rviz等

```bash
roslaunch track_lidar_front_end track_lidar_front_end_node.launch
```

## 已知问题

1. 预处理有ransac找地面过慢导致的少量闪烁，调整`pass_through_*_*_plane`可以解决
2. 在随手测试和改动中有大量莫名代码，整体结构很乱，在之后会修正

## 项目组织结构

```bash
.
├── src
│   └── track_lidar_front_end_node.cpp
├── CMakeLists.txt
├── package.xml
├── README.md
├── config
│   └── track_lidar_front_end.yaml
├── launch
│   ├── track_lidar_front_end_node.launch
└── rviz
    └── 1.rviz
```

- config: 项目配置文件, 包含一个yaml文件，存储着节点的各种参数
- launch: 加载参数，启动节点的roslaunch文件
- src: 源码
