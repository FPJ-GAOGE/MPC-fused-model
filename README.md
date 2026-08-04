# MPC fused model

用于 FineSUB 鱼目标跟踪的模型预测控制（MPC）实验代码。仓库包含单模型、双模型融合以及加入偏航控制的实现，用于仿真、测试和实机集成验证。

## 目录

- `MPC_model1/`：只使用模型一的三轴平移 MPC。
- `MPC_model2/`：只使用模型二的三轴平移 MPC。
- `MPC_dual_model/`：在线评估并融合两个平移模型的 MPC。
- `MPC_dual_model_yaw/`：在双模型平移控制基础上加入偏航控制的实验版本。
- `rov_track_control3.py`：相关跟踪控制脚本。

每个模型目录都有自己的说明、依赖清单、示例和测试。请先阅读相应目录下的 `README.md` 与 `CALIBRATION_CHECKLIST.md`。

## 基本用法

以双模型版本为例：

```powershell
python -m pip install -r MPC_dual_model/requirements.txt
Set-Location MPC_dual_model
python -m unittest discover -s tests -v
python example_simulation.py
```

其他版本可将上述目录名替换为目标模型目录。

## 安全提示

实机运行前必须重新检查质量与阻尼参数、力和设备命令的比例与符号、相机外参、噪声参数以及全部限幅。MCU 高层混控和 Python 八推进器直接控制只能选择一种，不能重复分配。
