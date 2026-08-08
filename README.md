# FineSUB MPC 控制

本仓库包含单模型、双模型融合以及带偏航控制的 MPC。四个模型均已接入统一
实机入口 `rov_track_control3.py`，并通过树莓派 TCP-UART 桥接到
`v4ROV2/FineSUB` 的 USART3。

## 克隆完整工程

`V4pro1_MPC` 固件以 Git 子模块接入本仓库，固件内部的 CMSIS-DSP 和 ETL
也保持为子模块。首次克隆时请递归拉取：

```powershell
git clone --recurse-submodules https://github.com/maerdofe-spec/MPC-fused-model.git
```

已有工作目录可执行：

```powershell
git submodule update --init --recursive
```

固件也可在独立仓库查看：
[V4pro1_MPC](https://github.com/maerdofe-spec/V4pro1_MPC)。

## 实机控制链路

控制坐标统一为机体系 FRD：x 向前、y 向右、z 向下，正偏航力矩使艇首
向右。MPC 输出物理量 `[Fx, Fy, Fz, N]`，
`MPC_dual_model/finesub_protocol.py` 按标定值转换为固件混控器的归一化
`[forward, right, down, yaw]` 通道。

v3 控制帧包含版本、显式解锁位、yaw 控制来源、随机会话号、16 位序号、发送
时间和 CRC-16/MODBUS，新会话必须先完成停桨握手。下位机以 20 Hz 回传完整
IMU、深度/压力、命令接受或拒绝原因、混控后真正下发的 8 路电机油门以及
8 路 DSHOT RPM。上位机将 8 路实际输出反解成上一拍实际力和力矩，供四套
MPC 的状态估计和模型预测使用。只有遥测、执行反馈和命令确认均新鲜时才能
进入自动 MPC；固件连续 250 ms 没有接受合法 MPC 帧时会清零并停桨。

完整帧格式、拒绝原因和安全状态机见
`docs/finesub_mpc_protocol_v3_zh-CN.md`。

| `--model` | 平移控制 | yaw 控制 |
|---|---|---|
| `model1` | 模型一 MPC | 下位机本地航向保持 |
| `model2` | 模型二 MPC | 下位机本地航向保持 |
| `dual` | 双模型融合 MPC | 下位机本地航向保持 |
| `dual-yaw` | 双模型融合 MPC | 上位机 yaw 状态机与双环控制 |

## 运行

在本目录安装依赖并启动：

```powershell
python -m pip install -r MPC_dual_model_yaw/requirements-live.txt
python rov_track_control3.py --model dual-yaw
```

将最后一个参数替换为 `model1`、`model2` 或 `dual` 即可运行另外三套模型；
不传 `--model` 时默认使用 `dual-yaw`。

通信方式、地址、安全时间和硬件换算参数位于：

```text
MPC_dual_model/finesub_v4pro1_mpc.json
```

也可以指定其他配置：

```powershell
python rov_track_control3.py --model dual --config path/to/finesub.json
```

配置中的 `transport.type` 支持 `tcp`、`udp`、`serial` 和 `dry_run`。

### 实际执行反馈的定义

目前 MPC 使用的是“下位机最终施加的电机输出”反解得到的已实现控制量；真实
RPM 也会回传和显示。由于暂时没有当前实艇每台推进器经过实测的“正反转
RPM—推力”标定曲线，所以不会直接把 RPM 当作牛顿推力。完成水池标定并把曲线
写入硬件适配配置后，才能进一步启用真正的 RPM—推力闭环。

当前闭环使用下位机经过混控、方向修正和限幅后最终写给8个电调的输出；RPM
目前用于执行诊断和后续标定。不得直接套用其他艇或其他推进器的占位曲线。

### 烧录新固件

“烧录新固件”指把本次修改后的 FineSUB 下位机 C++ 源码编译成
`Robomaster_A.hex`，再通过下载器写入实机的 STM32 控制板。它就是更新下位机
程序，不是只把源码复制到 `V4pro1_MPC` 文件夹。

协议 v3 的上下行帧格式都已改变，因此新上位机代码和新下位机固件必须配套
使用。只更新电脑上的 Python 而不重新烧录 STM32，实际执行回显不会生效。

视频输入还要求操作系统已安装 GStreamer、H.264 解码插件和 PyGObject (`gi`)；
这些组件应使用目标机的系统包管理器安装。

- `S`：框选目标；`Z`：停止跟踪；`Q`：退出。
- 手柄按钮 7：解锁/停桨。
- 手柄按钮 3：切换 MANUAL/AUTO。

程序退出时会发送显式停桨帧。没有视频、遥测过期、命令确认过期、网络中断或
上位机停止发帧时，下位机看门狗仍会停桨；通信恢复后不会自动重新解锁。

## 验证

```powershell
$env:PYTHONPATH="$PWD;$PWD/MPC_dual_model"
python -m unittest discover -s MPC_dual_model/tests -v
$env:PYTHONPATH="$PWD"
python -m unittest discover -s MPC_dual_model_yaw/tests -v
```

## 下水前必须标定

示例中的质量、附加质量、阻尼、最大三轴力、最大偏航力矩、推力符号、相机
视场角以及“目标框宽度到距离”的比例仍是占位/初始值。首次联调应卸桨验证
CRC、通道顺序、符号和 250 ms 失联停机，再系留低限幅测试；未经这些步骤不
应直接自动下水。
