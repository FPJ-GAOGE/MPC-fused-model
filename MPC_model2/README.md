# FineSUB 模型二 MPC

本目录是 `MPC_dual_model` 的“只使用模型二”版本，并按 `D:\浏览器下载\Untitled (1).pdf` 第 3-8 页整理。它保留固定线性阻尼 Fossen 模型、三轴 MPC 代价、力/力变化率约束、视场与前向距离软约束、相对位置卡尔曼滤波、设备命令映射和 FineSUB 八推进器分配。

唯一使用的预测模型是：

```text
x[k+1] = A_d x[k] + B_d tau[k]
```

其中 `x=[p_rel,v_rel]`，`p_rel=p_target-p_vehicle`，`tau` 是当前绝对三轴综合力。上一拍力 `tau[k-1]` 只用于力变化量代价和变化率约束，不参与模型二状态预测。

本版本没有：

- 模型一 `B_d(tau[k]-tau[k-1])`；
- 在线残差窗口和双模型融合权重；
- `model1_weight` / `model2_weight` 输入或输出。

矩阵指数、QP 求解器、坐标变换和 FineSUB 分配器复用相邻的双模型目录；本目录单独封装了模型二的 Fossen 预测和卡尔曼时间更新。运行时固定模型一权重为零，不实例化 `OnlineModelFusion`，也不计算模型一候选预测。

## 坐标与输入输出

- 相机输入：左相机光学坐标 `[right, down, forward]`，单位 m。
- MPC 状态：机体系 FRD `[forward, right, down, v_forward, v_right, v_down]`。
- MPC 输出：三轴平移综合力 `[F_forward, F_right, F_down]`，单位 N。
- 姿态力矩仍由现有姿态控制器负责；MPC 不控制旋转自由度。

## 运行

在 `D:\FINSMCAT\Machine\MPC` 下运行：

```powershell
python -m unittest discover -s MPC_model2/tests -v
python -m MPC_model2.example_simulation
```

实机接入入口见 `live_integration_example.py`。上机前必须重新标定 `M_t`、`D_L`、力与命令的比例、符号、相机外参、噪声和全部限幅。MCU 高层混控与 Python 八推进器直控只能选择一种，不能重复分配。
