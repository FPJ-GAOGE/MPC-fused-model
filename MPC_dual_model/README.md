# 固定线性阻尼双模型融合 MPC

本目录实现鱼目标三维相对位置跟踪。坐标为机体系 FRD：`x` 向前、`y` 向右、`z` 向下；MPC 输出综合平移力

```text
tau = [F_forward, F_right, F_down]  (N)
```

横滚、俯仰、偏航仍由 FineSUB 原姿态 PID 产生。平移 MPC 和姿态 PID 最后进入同一个 8 推进器分配器。

## 1. 双模型

固定 Fossen 低速模型为

```text
M_t v_dot + D_L v = -u
p_dot = v
```

精确零阶保持离散化得到 `A_d, B_d`。两个候选模型为：

```text
模型1: x[k+1] = A_d x[k] + B_d (tau[k] - tau[k-1])
模型2: x[k+1] = A_d x[k] + B_d tau[k]
```

模型1把上一拍力看作当前运动的匹配力，只把力的增量视为新作用，通常会给出更积极的追赶动作。模型2把当前综合力全部计入预测，通常更保守。它们都不是鱼加速度的直接测量模型，而是两种不同的未知运动/基础力假设。

各方向的融合权重由历史多步位置预测误差计算。每个历史时刻保存
模型一和模型二的状态预测起点；随后使用实际执行过的综合力逐步推进。未来
第 `h` 步位置到达时，再用滤波后的位置计算误差。于是：

```text
e_m(i,h) = p_filtered[i+h] - p_prediction_m[i+h | i]
M1 = sum(gamma_i * beta_h * e1(i,h)^2)
M2 = sum(gamma_i * beta_h * e2(i,h)^2)
C  = sum(gamma_i * beta_h * e1(i,h) * e2(i,h))
a1 = clip((M2-C+epsilon)/(M1+M2-2C+2epsilon), amin, 1-amin)
a2 = 1-a1
```

默认初始 `a1=0.8`，先偏向快速模型1；有足够观测后由在线误差自动修正。主跟踪器和实机入口统一采用 6 个起点、最大 3 步预测的阶梯历史：

```text
起点年龄 r:       1  2  3  4  5  6
允许最大步长 H_r: 3  3  2  2  1  1
预测步权重 omega: 0.5, 0.3, 0.2
```

令当前时刻为 `k`、预测起点为 `t0`、被预测的目标时刻为 `s`：

```text
r = k-t0
h = s-t0
只评价 1 <= h <= min(r,H_r)
```

例如起点为 `k-3` 时，保留 `k-2|k-3` 和 `k-1|k-3`，不评价三步的 `k|k-3`。`k-3|k-3` 是两个模型共同的滤波初值，`h=0`，只作为回放起点而不参与评分。每个当前时刻仅在实际启用的阶梯格子上重新归一化时间/步长权重。

`build_default_staircase_fusion()` 是默认阶梯参数的唯一构造入口，`MPCTracker` 和 `live_integration_example.py` 均调用它，避免一个入口使用阶梯而另一个入口又回到 20×8。`FusionConfig(staircase_horizon_caps=None)` 仍保留原来的全多步历史方式，仅供显式对照实验和现有调用兼容。预测时冻结当前模型权重，但每个预测步都滚动使用上一预测步的力，不把同一个 `tau_base` 错误地固定到整个时域。

为保持 QP 线性，把上一拍力增广为状态 `z=[x,tau_previous]`：

```text
x+            = A_d x + B_d tau - W B_d tau_previous
tau_previous+ = tau
```

其中 `W=diag(a1_x,a1_y,a1_z,a1_x,a1_y,a1_z)`。`a1=1` 就是纯模型1，`a1=0` 就是纯模型2。

## 2. FineSUB 推进器分配

电机顺序与 `TaskSUB.cpp` 一致：

```text
0 LFLower, 1 LFUpper, 2 LBUpper, 3 LBLower,
4 RBLower, 5 RBUpper, 6 RFUpper, 7 RFLower
```

上层输入 `[roll,pitch,down]`，下层输入 `[yaw,forward,right]`。`FineSUBThrusterAllocator` 原样采用 `V5_SUB.hpp` 的两个 4x3 矩阵及写电机时的正负号，并在最终对每台电机独立限幅到 `[-1,1]`。

检查到的源码把已经算出的深度 PID 输出替换成了常数 `0.0`。本实现默认 `enable_depth=True`，因为要完成三轴 MPC；若只想逐字复现那一行，可设为 `False`。

注意只能选择一种发送方式：

- 仍向现有 MCU 发送 forward/right/depth 高层命令：使用 `output.device_command`，由 MCU 完成混控。
- Python 直接控制 8 个 ESC：使用 `output.thruster_allocation.throttles`。

不能先在 Python 分配成 8 路后，再让 MCU 做第二次混控。

## 3. 文件

- `fossen_fixed_dl_model.py`：Fossen 模型和精确离散化。
- `relative_kalman.py`：仅用相对位置估计相对速度，无需 DVL。
- `model_fusion.py`：历史多步位置预测误差评分和在线融合权重。
- `mpc_controller.py`：代价函数、约束、双模型增广预测和 QP。
- `device_adapter.py`：现有高层命令映射及 FineSUB 8 推进器分配。
- `mpc_tracker.py`：测量、滤波、权重更新、MPC 和分配的总入口。
- `live_integration_example.py`：接入现有视频控制循环的示例。
- `example_simulation.py`：不连接实机的闭环演示。

## 4. 安装与测试

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python example_simulation.py
```

QP 默认选择 OSQP，并将单次求解时间限制为 40 ms；如果运行环境尚未安装
OSQP，会自动使用 NumPy ADMM 后备求解器。后备求解器已经把每轮迭代的
重复通用线性求解改成一次预计算后的矩阵向量乘法，但实机仍建议安装
`requirements.txt` 中的 OSQP。

融合模型的 `force_weights` 现在惩罚
`tau[k]-diag(a1)tau[k-1]`：纯模型一时等于力增量，纯模型二时等于绝对力。
这避免模型一把非零匹配力错误地当作需要压回零的控制代价。QP 超时或失败时，
控制器先保持上一份可行力；若还没有可行解，则限速返回锁存的基础力，而不是
直接返回零力。

## 5. 最小使用方式

```python
from live_integration_example import build_tracker, one_control_update
import numpy as np

tracker = build_tracker()
last_force_body = np.zeros(3)
tracker.latch_baseline(last_force_body)

output = one_control_update(
    tracker,
    position_camera_xyz,
    last_force_body,
)

tau = output.mpc.force
a1 = output.mpc.model1_weight
a2 = output.mpc.model2_weight
cmd = output.device_command
motor = output.thruster_allocation.throttles
last_force_body = tau.copy()  # 没有力反馈时的近似
```

`position_camera_xyz` 是 OpenCV 相机坐标 `[right,down,forward]`（米）。默认转换为机体系 `[forward,right,down]`。

## 6. 上机前必须标定

示例值只是软件占位值，必须标定：

1. `M_t`：三个方向的等效质量和附加质量。
2. `D_L`：固定线性阻尼。
3. `force_min/max` 与 `delta_force_min/max`。
4. `positive_force_at_limit`：各方向满通道对应的实际综合力。
5. 相机外参与测量噪声。
6. 电机正反号；先拆桨或固定机体逐方向核对。

建议先把力限幅降到额定值的 10%--20%，依次检查 forward、right、down 和 roll、pitch、yaw，再进行三轴水池测试。

完整的实机标定、调参和上机顺序见 `CALIBRATION_CHECKLIST.md`。
