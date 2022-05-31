# 横向控制

## LQR控制

## 控制器参数配置

## 横向控制器

采用LQR控制器

重要功能：

|<div style="width:100pt">函数</div> | <div style="width:100pt">功能</div> |<div style="width:100pt">输入</div> |<div style="width:100pt">输出</div> |
| --- | --- | --- | --- |
| `common::Status Init(std::shared_ptr<DependencyInjector> injector, const ControlConf *control_conf) override` | 重写控制器初始化函数 | 
| `common::Status ComputeControlCommand(const localization::LocalizationEstimate *localization, const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory, ControlCommand *cmd) override` | 重写控制指令计算函数 |
| `common::Status Reset() override` | 重写控制器重置函数 |
| `void Stop() override` | 重写控制器停止函数 | 
| `std::string Name() const override` | 重写控制器名称函数 | 
| `void UpdateState(SimpleLateralDebug *debug)` |
| `void UpdateDrivingOrientation()` |
| `void UpdateMatrix()` |
| `void UpdateMatrixCompound()` |
| `double ComputeFeedForward(double ref_curvature) const` |
| `void ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug)` |
| `bool LoadControlConf(const ControlConf *control_conf)` | 载入控制器参数配置 | 控制器参数配置 | 无 |
| `void InitializeFilters(const ControlConf *control_conf)` | 初始化滤波器（低通滤波器），包括数字滤波器、横向误差滤波器和航向误差滤波器等 | 控制器参数配置 | 无 |
| `void LoadLatGainScheduler(const LatControllerConf &lat_controller_conf)` |
| `void LogInitParameters()` | 打印车辆基本结构参数，包括车身质量、轮距和转动惯量等 | 无 | 无 |
| `void ProcessLogs(const SimpleLateralDebug *debug, const canbus::Chassis *chassis)` | 打印日志到文件 | 横向调试器【读】，车辆底盘状态【读】 | 无 |
|

## LQR 控制器
- 状态：横向误差，横向误差率，航向误差，航向误差率 (lateral error, lateral error rate, heading error, heading error rate)
- 系统矩阵、增益矩阵和权值矩阵
    - 系统矩阵
        - matrix_a_: 状态矩阵
        - matrix_ad_: 离散状态矩阵
        - matrix_adc_: 
        - matrix_b_:
        - matrix_bd_:
        - matrix_bdc_:
    - 增益矩阵
    - 权值矩阵
