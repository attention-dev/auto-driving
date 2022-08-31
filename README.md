# auto-driving
autonomous driving learning notes

参考[百度Apollo](https://github.com/ApolloAuto/apollo)源码进行自动驾驶相关技术的学习，包括规划控制算法，ROS，Protobuffer等，持续更新中

百度Apollo核心算法模块主要集中在[modules](https://github.com/ApolloAuto/apollo/tree/master/modules)中，包括
- [canbus](https://github.com/ApolloAuto/apollo/tree/master/modules/canbus) CAN控制器收发模块
- [control](https://github.com/ApolloAuto/apollo/tree/master/modules/control) 控制模块
- [localization](https://github.com/ApolloAuto/apollo/tree/master/modules/localization) 定位模块
- [map](https://github.com/ApolloAuto/apollo/tree/master/modules/map) 地图导航模块
- [perception](https://github.com/ApolloAuto/apollo/tree/master/modules/perception) 感知模块
- [planning](https://github.com/ApolloAuto/apollo/tree/master/modules/planning) 路径/运动规划模块
- [prediction](https://github.com/ApolloAuto/apollo/tree/master/modules/prediction) 预测模块
- [routing](https://github.com/ApolloAuto/apollo/tree/master/modules/prediction) 路由模块
- [dreamview](https://github.com/ApolloAuto/apollo/tree/master/modules/dreamview) 人机界面

此外，Apollo用到了大量的第三方库[third_party](https://github.com/ApolloAuto/apollo/tree/master/third_party)，如
- [protobuf](https://developers.google.com/protocol-buffers) 序列化结构数据协议，在Apollo中可以定义消息类数据
- [eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) 线性代数矩阵运算库
- [boost](https://www.boost.org/)
- [gflags](https://github.com/gflags/gflags) 
- [gtest](https://github.com/google/googletest) 测试库
- [opencv](https://opencv.org/)
- [pcl](https://pointclouds.org/) 点云算法库
- [ros](https://www.ros.org/)
- [tf2](https://github.com/ApolloAuto/apollo/tree/master/third_party/tf2) ROS tf2 坐标系转换库


## Reference
- https://cloud.tencent.com/developer/article/1418705
- https://blog.csdn.net/adamshan/article/details/80555174
- https://blog.csdn.net/AgingMoon/article/details/110311171
- https://blog.csdn.net/weixin_37395438/article/details/112973098
- https://adamshan.blog.csdn.net/article/details/80779615
- https://blog.csdn.net/robinvista/article/details/111400051
