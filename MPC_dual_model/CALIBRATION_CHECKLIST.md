# 实机标定与调参清单

以下量必须在水池、拖曳台、固定架或回放数据实验中确定。代码中的默认值仅用于软件仿真，不能视为 FineSUB 的实机参数。

## Unity UnderwaterVision 当前填写状态（2026-08-08）

结论：仿真接入所需参数已经填写，但 `CALIBRATION_CHECKLIST` 尚未完成实机标定。下表区分已验证值、仿真假设和当前直驱模式不使用的参数；“待实测”项目不能仅靠 Unity 补齐。

| 类别 | 当前值 | 状态与依据 |
|---|---|---|
| 控制坐标与可见量 | MPC 只接收机体系前/右/下的目标相对位置 `[x,z,-y]` | Unity 内部用目标与 ROV 变换生成无噪声理想相对量，但 ROS/MPC 不再接收两者的世界位姿。话题为 `/sim/finsrov/mpc/target_relative_body`，frame 为 `FinsROV/mpc_body_frd`。场景关闭 pose、DVL、depth 发布，只保留 IMU。 |
| `rotation_body_from_camera`、`camera_origin_in_body`、双目尺度 | 未填 | Unity 理想相对量直接输出 FRD；接真实视觉时必须用这些标定值把相机向量转到相同 FRD 接口。 |
| `position_std` | `(0.015, 0.015, 0.025) m` | 仿真假设，未由静止视觉数据统计。 |
| 时间与延迟 | `dt=0.05 s`，ROS 控制频率和理想相对位置发布频率均为 `20 Hz`，输入超时 `1.0 s` | 消息时间戳只是采样元数据；当前不从目标世界位姿求绝对速度/加速度。实机相机、网络和推理总延迟待实测。 |
| `M_t` | `diag(26.07276, 26.79684, 26.07276) kg` | UnderwaterVision 仿真辨识值，顺序为前/右/下（Unity `X/Z/-Y`）；速度拟合 `R²=0.9986/0.9956/0.9928`。它包含当前 Unity 环境表现出的等效质量，不是实机参数。 |
| `D_L` | `diag(93.88006, 143.69195, 280.86849) N/(m/s)` | 用 `+/-6 N`、`+/-12 N` 三轴阶跃的完整速度响应辨识；拟合得到的二次阻尼接近零，当前采用线性项。原始结果见 `dynamics_20260806_212129.json`。 |
| `acceleration_std` | `(0.35, 0.35, 0.45) m/s^2` | 滤波器默认假设，未由模型残差统计。 |
| `force_min/max` | `min=(-16.2968,-16.2968,-23.0472) N`，`max=(16.2968,16.2968,29.5236) N` | 由 V4 Pro1 canonical 8 台推进器的非对称 `force_n` 限额和 45 度水平安装几何计算；QP 同时逐台约束，斜向力不是独立轴盒限。 |
| `delta_force_min/max` | `+/- (4.0, 4.0, 5.6) N/周期` | 工程限幅，取各轴包络约 20%；执行器响应尚未辨识。 |
| 直驱映射 | `positive_force_at_limit=(16.2968,16.2968,29.5236)`，`signs=(1,1,1)` | 当前 `DirectWorldWrench` 路径已验证；Unity 不执行推进器动力分配，但 MPC QP 仍执行逐台可行域约束。 |
| 设备命令与分配器 | `command_limits=(99,99,45)`；分配器参数未标定 | 当前桥接节点把 `thruster_allocator` 置空，因此这些值不参与 Unity 直驱控制。 |
| MPC 任务 | 参考 `(0.8,0,0) m`，预测步数 `10`，预测时域 `0.5 s` | 已接入并用于仿真调参。 |
| MPC 状态权重 | 位置 `(10000,14000,25000)`，速度 `(2,20,12)`，终端倍数 `4` | 2026-08-08 UnderwaterVision `0.0975 m/s` 跑道轨迹最终值。 |
| MPC 输入权重 | 有效力 `(0.003,0.002,0.04)`，力变化 `(0.01,0.01,0.5)` | 横向权重按半圆动态段调整；逐台非对称硬限幅保持不变。 |
| 距离与视场 | 前向距离 `[0.25,1.50] m`；半视场 `(42,30) deg`；边界 `5 deg` | 约束已启用；实际镜头有效视场和双目工作距离待标定。 |
| 松弛变量 | 二次权重 `5e4`，一次权重 `100`，`slack_max=5.0` | 仿真调参值。 |
| 双模型融合 | 初始权重 `(0.8,0.8,0.8)`，最小权重 `0.01`，窗口 `6`，预测 `3`，遗忘 `0.8`，权重更新率 `0.35` | 当前启用。模型一使用 `tau-tau_base`，模型二使用绝对 `tau`；阶梯上限 `(3,3,2,2,1,1)`，预测权重 `(0.5,0.3,0.2)`；每个方向独立选择模型。权重更新率只平滑模型选择参数，不平滑控制力。候选差异小于噪声门槛时向模型一稳定先验恢复；匀速轴模型一可达到 `0.99`，快速变速且模型二残差更小时才切换。 |
| 融合保护 | 位置误差截断 `(0.5,0.5,0.5) m`，`epsilon=1e-10`，候选差异门槛 `(1e-7,1e-7,1e-7) m2` | `epsilon` 只防止除零；候选差异门槛用于拒绝无法区分的噪声级模型差异，避免匀速轴权重在 `0` 和 `1` 之间抖动。 |
| QP 求解器 | OSQP，`rho=10`，`sigma=1e-8`，最多 `1500` 次，容差 `2e-5/3e-4`，时限 `0.035 s` | 时限已修正为小于 `0.05 s` 控制周期；仍需持续记录最坏求解时间。 |
| `tau_base` 在线更新 | 初始 `(0,0,0) N`；`enabled=true`，稳态更新率 `0.01`，过渡更新率 `0.03`，逐轴门限 `0.20 m/0.08 m/s` | 只用相对位置和由其估计的相对速度，以及上一周期已知的饱和力命令缓慢更新；它只进入模型一，不进入模型二正常预测。没有真实推力传感器时，该命令只是 `tau_achieved` 的近似。 |

当前有效场景是水平 XZ 跑道闭环：`0.75 m` 直线、`0.20 m` 半圆半径、恒定深度，依次执行 `+X` 直线、半圆、`-X` 直线、半圆回到起点，速率恒为 `0.0975 m/s`，起跑延迟 `2.0 s` 与目标相对位置发布同步。最终 65 s 试验的匹配段误差均值/P95 为 `0.01237/0.02665 m`，低加速度段 P95 `0.01655 m`，稳态 P95 `0.01032 m`，QP 成功率 `100%`。水平逐推进器包络触顶率为 `23.1%`；`0.10 m/s` 即使用最终权重，匹配段 P95 仍为 `0.03205 m`，因此 `0.0975 m/s` 是当前限额下最高确认速度。

## 1. 坐标系与视觉

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `rotation_body_from_camera` | `camera_transform.py` | 无 | 采集多个已知相机点和机体系点，拟合刚体旋转；验证相机前/右/下分别映射到机体前/右/下。 |
| `camera_origin_in_body` | `camera_transform.py` | m | 测量左相机光心相对机体控制原点的前/右/下位移。 |
| 双目尺度、左右相机外参 | 视觉系统上游 | m | 用已知距离标定板或标尺验证三维输出尺度。 |
| `position_std` | `KalmanConfig` | m | ROV 和目标固定时采集位置序列，取各轴标准差。 |
| 相机时间戳与总延迟 | 集成层 | s | 记录曝光、传输、推理到控制命令的时间；用于实际 `dt` 和后续延时滤波。 |

## 2. ROV 平移动力学

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `M_t` | `FixedLinearDampingRelativeModel` | kg（含附加质量） | 三轴阶跃推力/自由衰减实验拟合；允许为 3x3 非对角矩阵。 |
| `D_L` | `FixedLinearDampingRelativeModel` | N/(m/s) | 三轴匀速或自由衰减数据拟合固定线性阻尼。 |
| `dt` | `FixedLinearDampingRelativeModel` | s | 使用相邻有效三维量测的真实时间间隔；不要只假设 0.05。 |
| `acceleration_std` | `KalmanConfig` | m/s² | 从鱼突转、水流和模型残差统计得到；它描述模型无法解释的加速度。 |

## 3. 推进器和低层控制链

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `force_min/max` | `MPCConfig` | N | 以连续可用推力，不是瞬时额定峰值，作为上限。 |
| `delta_force_min/max` | `MPCConfig` | N/控制周期 | 阶跃试验中由安全推力变化率和执行器响应确定。 |
| `positive_force_at_limit` | `ForceCommandAdapter`、分配器 | N | 分别标定 forward/right/down 的满命令实际综合力。 |
| `signs` | `ForceCommandAdapter` | ±1 | 固定 ROV、低推力逐轴测试正负方向。 |
| `command_limits` | `ForceCommandAdapter` | 设备命令 | 从 MCU 协议和安全限幅确认。 |
| `translation_channel_limits`、`attitude_channel_limits` | `FineSUBThrusterAllocator` | 归一化通道 | 在不发生单电机持续饱和的条件下调定。 |
| `deadband` | `FineSUBThrusterAllocator` | 归一化通道 | 测量推进器开始稳定产生推力的最小命令。 |
| 每台推进器油门-推力曲线、响应延迟 | 当前代码未实现 | N、s | 用推力计、RPM/电流或水池实验建立；用于估计 `tau_achieved`。 |

## 4. MPC 任务与安全参数

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `reference_position` | `MPCConfig` | m | 任务要求的鱼相对位置，例如 `[0.6,0,0]`。 |
| `horizon` | `MPCConfig` | 控制步 | 先用 8-12；与 `dt` 一起决定预见时间。 |
| `position_weights` | `MPCConfig` | 代价权重 | 由允许位置误差归一化后，在仿真中调节。 |
| `velocity_weights` | `MPCConfig` | 代价权重 | 由允许相对速度和超调容忍度调节。 |
| `force_weights` | `MPCConfig` | 代价权重 | 现在惩罚融合有效作用力；平衡跟踪与推力使用。 |
| `delta_force_weights` | `MPCConfig` | 代价权重 | 由允许的力变化和平滑性确定。 |
| `forward_distance_min/max` | `MPCConfig` | m | 由安全距离、双目有效工作距离确定。 |
| `horizontal_half_fov_deg`、`vertical_half_fov_deg` | `MPCConfig` | deg | 用实际镜头有效半视场，而不是宣传全视场。 |
| `fov_margin_deg` | `MPCConfig` | deg | 留给检测框误差、相机畸变和控制延迟的安全余量。 |
| 松弛权重与 `slack_max` | `MPCConfig` | 代价、m | 先在仿真中确认突转时可行，再限制可接受的违规程度。 |

## 5. 双模型融合参数

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `initial_model1_weight` | `FusionConfig` | 0-1 | 根据先验，可先偏向模型一；应由实测残差修正。 |
| `minimum_weight` | `FusionConfig` | 0-0.5 | 防止任一模型完全失去恢复能力。 |
| `window` | `FusionConfig` | 历史起点数 | 大则稳定、慢；小则快、噪声敏感。建议先从 20 开始。 |
| `prediction_horizon` | `FusionConfig` | 控制步 | 多步位置评分长度；建议先从 6-10 开始。 |
| `forgetting_factor` | `FusionConfig` | 0-1 | 越接近 1 越平稳，越小越迅速适应目标机动。 |
| `horizon_weight_decay` | `FusionConfig` | 0-1 | 多步预测越远是否降权；先从 0.9-1.0 开始。 |
| `position_error_clip` | `FusionConfig` | m | 基于正常视觉误差和异常值上界设置。 |
| `epsilon` | `FusionConfig` | 误差平方单位 | 防止两个模型难以区分时除零。 |

## 6. 求解器与运行时安全

| 参数 | 代码位置 | 单位 | 如何确定 |
|---|---|---:|---|
| `backend` | `QPSolverSettings` | 名称 | 实机优先 `osqp`；仿真可 `auto`。 |
| `time_limit_seconds` | `QPSolverSettings` | s | 小于控制周期并留给视觉、通信和安全逻辑时间。 |
| `max_iterations`、`absolute_tolerance`、`relative_tolerance` | `QPSolverSettings` | 无 | 通过最坏工况仿真平衡精度与时间。 |
| `BaselineAdaptationConfig` | `mpc_tracker.py` | 无、m、m/s | 仅在基础力可靠、匹配状态稳定时启用。 |

## 上机顺序

1. 固定 ROV、低推力确认相机坐标和三轴正负号。
2. 标定相机外参和双目尺度。
3. 标定推进器命令到实际综合力，并记录响应延迟。
4. 用固定目标和低推力辨识 `M_t`、`D_L`。
5. 在仿真中调 MPC 和融合权重。
6. 水池低限幅闭环，仅启用前向轴。
7. 依次加入右向、下向、目标机动、在线融合。
