<!-- <style>
table th:first-of-type {
    width: 55%;
}
table th:nth-of-type(2) {
    width: 15%;
}
table th:nth-of-type(3) {
    width: 15%;
}
table th:nth-of-type(3) {
    width: 15%;
}
</style> -->

## 控制器抽象基类

适用于所有控制器的接口，并由具体的控制器实现接口功能。

|<div style="width:100pt">函数</div> | <div style="width:100pt">功能</div> |<div style="width:100pt">输入</div> |<div style="width:100pt">输出</div> |
| --- | --- | --- | --- |
| `common::Status Init(std::shared_ptr<DependencyInjector> injector, const ControlConf *control_conf)` | 初始化控制器 | 控制器配置 | 初始化状态，即是否初始化成功 |
| `common::Status ComputeControlCommand(const localization::LocalizationEstimate *localization, const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory, control::ControlCommand *cmd)` | 基于当前的车辆状态和目标轨迹计算控制指令 | 车辆位置【读】，车辆状态（如速度、加速度等）【读】，由上层规划模块生成的参考轨迹【读】，计算的控制指令【写】| 状态标志，指令是否计算成功 | 
| `common::Status Reset()` | 重置控制器 | 无 | 重置标志 |
| `std::string Name() const` | 获取控制器名称 | 无 | 控制器名称 |
| `void Stop()` | 停止控制器 | 无 | 无 |

- 控制器配置 [`ControlConf`](https://github.com/ApolloAuto/apollo/blob/master/modules/control/proto/control_conf.proto) \
包括控制器类别，测试相关参数以及控制器本身的一些参数（如控制周期、横纵向控制器参数配置、轨迹/车辆状态/位置周期相关参数等）

- [`DependencyInjector`](https://github.com/ApolloAuto/apollo/blob/master/modules/control/common/dependency_injector.h) \
通过[`VehicleStateProvider`](https://github.com/ApolloAuto/apollo/blob/master/modules/common/vehicle_state/vehicle_state_provider.h)类获取车辆状态，[`VehicleStateProvider`](https://github.com/ApolloAuto/apollo/blob/master/modules/common/vehicle_state/vehicle_state_provider.h)类主要功能包括
    - 状态更新
    - 姿态获取
    - xyz坐标获取
    - kappa，曲线的曲率
    - 横滚角 roll angle
    - 俯仰角 pitch angle
    - 偏航角 yaw angle
    - 朝向角 heading，即车头朝向与x轴的夹角
    - 线速度 linear_velocity
    - 角速度 angular_velocity
    - 线加速度 linear_acceleration
    - 轮速 gear
    - 方向盘转角 steering_percentage
    - 速度下发，set_linear_velocity
    - 位置估计，EstimateFuturePosition
    - 车辆中心位置计算，ComputeCOMPosition
    - 获取车辆状态，vehicle_state

- 车辆位置 [`localization::LocalizetionEstimate`](https://github.com/ApolloAuto/apollo/blob/master/modules/localization/proto/localization.proto) \
数据结构包含3个消息结构和一个枚举
    - `message Uncertainty` 定义导航定位、位置坐标、线速度、线加速度、角速度等的标准差
    - `message LocalizationEstimate` 定义姿态、轨迹航点等
    - `message LocalizationStatus` 定义坐标融合、导航、Lidar状态等
    - `enum MeasureState` 定义定位成功标志以及误差等

- 车辆底盘状态 [`canhus::Chassis`](https://github.com/ApolloAuto/apollo/blob/master/modules/canbus/proto/chassis.proto)
    - **`message Chassis`** 定义底盘物理量
        - `enum DrivingMode` 定义驾驶模式，即5级自动驾驶级别
        - `enum ErrorCode` 定义车辆故障类型，包括控制指令周期错误，方向盘、刹车、挡位故障等
        - `enum GearPosition` 定义挂挡位置，包括空挡、正常行驶、倒车、泊车、低速行驶、无效挡位等
        - 其他项，包括引擎速度、车速、里程计、车灯信号等车辆底盘本身的一些状态量 \
    其嵌套以下消息结构
    - `message ChassisGPS` 定义GPS，包括经纬度、时间等 
    - `enum GpsQuality` 
    - `message WheelSpeed` 定义轮速，包括后右rr、后左rl、前右fr、前左fl，四个轮子的速度和方向
    - `message Sonar` 定义声纳相关物理量
    - `message Surround` 定义车辆周围环境，包括交叉通行和盲点警报，以及声纳状态等

- 规划轨迹 [`planning::ADCTrajectory`](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/proto/planning.proto)
    - **`message ADCTrajectory`** 定义路径长度/时间、轨迹点、路径点、决策结果等
    其嵌套以下消息结构
    - `message EStop` 定义紧急制动及原因
    - `message TaskStats` 定义任务统计数据，包括任务名称和时间
    - `message LatencyStats` 
    - `enum JucType` 定义车身主体在道路上的位置类型
    - `message RSSInfo` 

- 控制指令 [`control::ControlCommand`](https://github.com/ApolloAuto/apollo/blob/master/modules/control/proto/control_cmd.proto)


