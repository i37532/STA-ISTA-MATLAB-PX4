# 改进模型原样副本

| 模型 | 文件 | 来源 |
| --- | --- | --- |
| 改进 STA（XYZ） | `sta/drone_position_control_sta_z_tuned.slx` | `matlab_simulation/sta` 中最终保存的同名模型 |
| 改进 ISTA | `ista/drone_position_control_ista_tuned.slx` | `matlab_simulation/ista` 中最终保存的同名模型 |

两份文件均按字节复制，SHA-256 见 `../parameters/source_manifest.json`。没有为 PX4 移植而修改仿真模型。

从 MATLAB 打开对应 SLX 即可查看原来的完整位置、速度、姿态、电机与动力学连接。控制函数已经内嵌于模型，`../matlab` 中的同名 `.m` 是可读的参考计算副本；修改外部 `.m` 不会自动修改模型内嵌函数。

`../matlab` 与 `../cpp` 用于后续算法迁移和单步一致性检查。SLX 是 Simulink 文件，不能直接放进 PX4 固件编译。两模型中的电机模型、质量、推力系数和加速度到油门换算属于当前仿真对象，接入 PX4 时应继续使用 PX4 的机体参数和加速度到推力转换链路。
