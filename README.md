# track_lidar_front_end

> 实验性质的产品，源自track_lidar_front_end_with_imu

竟然跑起来了  
还很粗糙，主要需要调整的是lidar_to_imu参数、欧式聚类  

## 计划改进

- 加入圈数判定，在跑起来时也会有比较明显的误差累计(用八字测试的)，在顺逆时针圈lidarToImu应不同(拓展至左转/右转?)
- 有很多数据可以用`ros参数服务器`共享，如圈数、作弊程序启不启用等，可以用一个程序控制其他程序，同时减少一些重复计算
- 类封装，调整部分函数，使用MessageHelper同步消息(数据类型不同，要改很多，影响不是特别大，之后再加)，完成README
