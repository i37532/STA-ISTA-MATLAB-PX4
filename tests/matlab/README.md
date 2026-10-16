# MATLAB 单步参考数据

在 MATLAB 中进入本目录，运行：

```matlab
export_controller_reference
```

如已安装 Simulink，可先执行 `verify_model_parameters`，核对 JSON 中的参数与保存的两份 SLX 一致；此检查只加载临时改名副本读取参数，不进行仿真。

脚本从包内 `parameters/controller_parameters.json` 加载参数，使用 `matlab/sta`、`matlab/ista` 中与 SLX 内嵌计算一致的函数，生成同目录 `controller_reference.csv`。不需要运行完整 Simulink 仿真，也不依赖原 Windows 目录。

每行是独立的一次采样：`algorithm=0` 为改进 STA，`algorithm=1` 为改进 ISTA；后续列依次是 `k1,k2,epsilon,lo,hi,nu_max,dt,error,nu,accel_ff,expected_raw,expected_limited,expected_next`。`nu` 是采样开始时的积分状态，`expected_next` 是本次采样后保存给下一次的积分状态。

覆盖 XY/Z 两套参数、0.0025/0.005/0.01 s 三种采样周期、正负误差、零点、边界层和隐式分支边界两侧、上下输出饱和、积分状态上下限及退出饱和的方向。测试的 `epsilon` 均大于零，符合两个改进模型的实际设置；没有把 `epsilon=0` 作者核回退分支作为改进算法的测试对象。

限幅输出和下一积分状态直接由原 MATLAB wrapper 返回。XY 的 `expected_raw` 使用同一控制核补算；Z 的原函数直接返回该诊断量。前馈通过 `velocity=-accel_ff/lambda` 输入；Z 测试固定 `c33=1, vd_raw=0`，并把经过高度物理推力约束后的实际上下限写入 CSV。误差与前馈独立取样是为覆盖算法输入，并非每行都代表可同时出现的完整飞机状态。

这验证了 C++ 与 MATLAB 单步数值一致性，不替代 PX4 编译、SITL 或闭环测试。
