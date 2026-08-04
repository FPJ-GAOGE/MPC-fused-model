# 加入 yaw 旋转的双模型融合 MPC（实验版）

本目录是基于同级 `MPC_dual_model` 的旋转实验层。坐标系仍为机体系 FRD：`x` 向前、`y` 向右、`z` 向下；正 yaw 表示艏部向右转。

控制输出扩展为：

```text
u = [X, Y, Z, N]
X/Y/Z: 机体系平移综合力 (N)
N:     绕机体 z 轴的艏摇力矩 (N*m)
```

## 1. 这版实现的数学结构

平移双模型仍为：

```text
模型1: x_bar+ = A_d x + B_d (tau - tau_previous)
模型2: x_bar+ = A_d x + B_d tau
```

但 `x_bar+` 仍表达在旧机体系。若本周期 IMU 测得艏摇增量 `delta_psi`，统一转换到新机体系：

```text
T(delta_psi) = blockdiag(Rz(-delta_psi), Rz(-delta_psi))
x+ = T(delta_psi) x_bar+
```

因此纯转头、没有相对平移时：

```text
p[k] = Rz(-delta_psi) p[k-1]
v_rel = (p[k] - Rz(-delta_psi)p[k-1]) / dt = 0
```

不会再把转头产生的图像运动当成鱼的横向速度。

Fossen 约化艏摇模型为：

```text
m_r r_dot + d_r r = N
psi_dot = r
m_r = I_z - N_rdot
```

代码对线性艏摇模型进行精确零阶保持离散化。视线角为：

```text
alpha = atan2(p_y, p_x)
alpha_dot = (p_x v_y - p_y v_x)/(p_x^2+p_y^2+epsilon) - r
```

MPC 在位置、速度原代价之外增加 `alpha^2`、`r^2`、`N^2` 与 `delta_N^2`，并增加艏摇力矩、力矩变化率和角速度约束。

## 2. 为什么仍能使用 QP

旋转矩阵中的 `sin/cos` 以及 `r*p`、`r*v` 会使完整模型成为非线性模型。第一版按设计对话中的“冻结旋转轨迹”实现：

1. 本周期第一步使用 IMU 当前 `r`；
2. 后续步使用上一轮 MPC 的预测 `r` 构造 `Rz(-dt*r)`；
3. 视线角速度在同一条上一轮轨迹附近线性化；
4. 本轮仍求解一个凸 QP；求解完成后更新下一轮冻结轨迹。

这是线性时变近似，不是完整非线性 MPC。实际 yaw 与历史模型评价始终使用 IMU 的真实角度增量，而不是冻结预测值。

## 3. 与平移版一致的二维历史阶梯评价

旋转版每个历史预测起点额外保存 `origin_index`。回放时，每一步都先使用实际执行力推进两个平移候选模型，再用该步实际 IMU yaw 增量把两个预测旋转到目标时刻机体系，最后将预测位置与同一目标时刻的滤波位置比较。

令当前时刻为 `k`、回放起点为 `t0`、预测目标时刻为 `s`：

```text
r = k-t0
h = s-t0
1 <= h <= min(r, H, H_cap(r))
```

默认值为：

```text
r:       1  2  3  4  5  6
H_cap:   3  3  2  2  1  1
omega_h: 0.5, 0.3, 0.2
H:       3
```

因此在 `k` 时刻，对于起点 `t0=k-3`：

- `k-2|k-3`：保留，`h=1`；
- `k-1|k-3`：保留，`h=2`；
- `k|k-3`：排除，因为 `H_cap(3)=2`；
- `k-3|k-3`：`h=0`，仅作为两个模型共同的滤波回放初值，不参与评分。

模型评分只在当前实际启用的阶梯格子上计算。时间衰减权重与 `omega_h` 相乘后，会在这些启用格子之间重新归一化，因此阶梯格子数量随时间变化不会无意改变 `M1/M2/C12` 的整体尺度。

## 4. 文件

- `yaw_relative_model.py`：离散旋转、旋转补偿差分、Fossen 艏摇动力学。
- `yaw_kalman.py`：用实际 yaw 增量传播均值和协方差的 KF。
- `yaw_mpc_controller.py`：四输入冻结旋转 LTV-MPC/QP。
- `yaw_tracker.py`：时间顺序、双模型历史回放、FineSUB yaw 通道映射。
- `live_integration_example.py`：构造控制器和单帧调用示例。
- `example_yaw_simulation.py`：不连接硬件的闭环演示。
- `tests/`：纯旋转、零旋转退化、QP 与推进器混控专项测试。

该实验包复用主融合目录中已经验证过的 `dense_qp.py`、固定阻尼平移模型、双模型权重、相机转换和 FineSUB 分配器。因此两个目录必须保持同级。

## 5. 安装、测试和仿真

在 `D:\FINSMCAT\Machine\MPC` 下运行：

```powershell
python -m pip install -r .\MPC_dual_model_yaw\requirements.txt
python -m unittest discover -s .\MPC_dual_model_yaw\tests -v
python -m MPC_dual_model_yaw.example_yaw_simulation
```

本 yaw QP 默认强制使用 OSQP。当前机器的完整 tracker 基准约为平均 6 ms、95% 约 7 ms；NumPy ADMM 后备在同一问题上约为平均 59 ms，无法满足 20 Hz，所以本实验版不会静默退回它。若 OSQP 未安装，控制器会在构造时直接报错。

## 6. 控制循环接口

```python
import numpy as np
from MPC_dual_model_yaw.live_integration_example import (
    build_tracker,
    one_control_update,
)

tracker = build_tracker()
last_force = np.zeros(3)
last_yaw_moment = 0.0

# AUTO 切入时，锁存实际保持力、实际 yaw 力矩和当前 IMU yaw。
tracker.latch_baseline(last_force, last_yaw_moment, imu_yaw_rad)

output = one_control_update(
    tracker=tracker,
    position_camera_xyz=position_camera_xyz,  # [right, down, forward], m
    imu_yaw_rad=imu_yaw_rad,
    imu_yaw_rate_rad_s=imu_yaw_rate_rad_s,
    last_achieved_force_body=last_force,
    last_achieved_yaw_moment=last_yaw_moment,
    roll_pitch_control=(roll_pid, pitch_pid),
    rotation_body_from_camera=R_bc,       # 实机标定的相机到机体系旋转
    camera_origin_in_body=r_bc_body,      # 机体原点到相机原点的杠杆臂
)

tau = output.mpc.force
N = output.mpc.yaw_moment
yaw_channel = output.yaw_channel
motor = output.thruster_allocation.throttles

# 没有力/力矩观测器时，下面只能作为近似。
last_force = tau.copy()
last_yaw_moment = N
```

相机位置、IMU yaw 和 IMU yaw rate 必须对齐到同一拍摄时刻。若相机有明显延迟，需先做时间戳历史更新和 IMU 姿态回放；当前实时入口尚未实现延迟图像回溯。

只能选择一种执行链路：要么发送高层平移命令并把 `yaw_channel` 送入 MCU 现有姿态混控，要么直接使用八路 `motor`。不能让 Python 与 MCU 对同一控制量重复混控。

## 7. 实机必须确定的参数

平移部分沿用主融合代码的标定项，yaw 新增：

1. `effective_inertia = I_z-N_rdot`；
2. `linear_damping = d_r`；
3. `yaw_moment_min/max`；
4. `delta_yaw_moment_min/max`；
5. `yaw_rate_min/max`；
6. `positive_yaw_moment_at_limit`；
7. yaw 正负号 `YawMomentChannelAdapter.sign`；
8. IMU yaw 与相机时间戳偏差、IMU 安装方向和角速度零偏；
9. `line_of_sight_angle_weight`、`yaw_rate_weight`、`yaw_moment_weight`、`delta_yaw_moment_weight`；
10. 平移与 yaw 同时工作时的推进器可达域。当前 QP 分别限制 `X/Y/Z/N`，尚未加入八推进器共同饱和形成的耦合多面体约束。
11. 相机到机体系旋转 `R_bc` 和机体原点到相机原点杠杆臂 `r_bc_body`；加入 yaw 后杠杆臂误差会直接表现为周期性的假相对运动。

所有示例数值只是软件占位值，不能直接用于实机。应先在纯软件仿真验证，再固定机体/拆桨确认六轴符号，然后在低限幅水池试验中辨识参数。

## 8. 当前实验版仍有的边界

- 只加入 yaw，不包含 roll/pitch 对相对坐标的旋转补偿；
- 艏摇动力学暂时只有线性阻尼，没有 `d_rr*|r|*r` 和 surge-sway-yaw 耦合；
- 冻结旋转轨迹是 QP 近似，大角速度或上一轮预测偏差大时应缩短步长或升级为 SQP/NMPC；
- 视觉延迟回溯尚未实现；
- yaw 力矩与平移力尚未使用完整 8 推进器控制分配可达域作联合约束；
- 没有实际力/力矩观测器时，上一拍饱和命令仍只是已执行控制的近似。
