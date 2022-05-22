<style>
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
</style>

控制器抽象基类，只提供适用于所有控制器的接口，不实现，由具体的控制器实现接口功能
| 函数 | 功能 | 输入 | 输出 |
| --- | --- | --- | --- |
| `common::Status Init(std::shared_ptr<DependencyInjector> injector, const ControlConf *control_conf)` | 初始化控制器 | 控制器配置 | 初始化状态，即是否初始化成功 |
| `common::Status ComputeControlCommand(const localization::LocalizationEstimate *localization, const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory, control::ControlCommand *cmd)` | 基于当前的车辆状态和目标轨迹计算控制指令 | 车辆位置【读】，车辆状态（如速度、加速度等）【读】，由上层规划模块生成的参考轨迹【读】，计算的控制指令【写】| 状态标志，指令是否计算成功 | 
| `common::Status Reset()` | 重置控制器 | 无 | 重置标志 |
| `std::string Name() const` | 获取控制器名称 | 无 | 控制器名称 |
| `void Stop()` | 停止控制器 | 无 | 无 |

- 控制器配置类 [`ControlConf`](https://github.com/ApolloAuto/apollo/blob/master/modules/control/proto/control_conf.proto)
包括控制器类别，测试相关参数以及控制器本身的一些参数（如控制周期、横纵向控制器参数配置、轨迹/车辆状态/位置周期相关参数等）

- [`DependencyInjector`](https://github.com/ApolloAuto/apollo/blob/master/modules/control/common/dependency_injector.h)
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

- 车辆位置 [`localization::LocalizetionEstimate`](https://github.com/ApolloAuto/apollo/blob/master/modules/localization/proto/localization.proto)
包含3个消息结构

- 车辆状态 [`canhus::Chassis`]()

- 规划轨迹 [`planning::ADCTrajectory`]()

- 控制指令 [`control::ControlCommand`]()

