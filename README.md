# 改进 STA 与 ISTA 的 PX4 速度环移植准备包

本文件夹包含两份原样复制的改进模型、对应 MATLAB 源函数、可独立编译的 C++ 控制核心和 Ubuntu 接入说明。目标是后续替换 PX4 的**平移速度内环**，即 x、y、z 速度控制，不是机体角速度环。

目前已准备数学核心与测试；**尚未修改飞控固件、编译 Codev 或运行 Gazebo SITL**。接入说明已按你提供的 `Codev-autopilot.zip` 实际源码核对：分支 `main-v2_gps`，归档 HEAD 为 `b1e454edede00da196ed280a7c48a4faf4433616`。这份工程不能按官方 v1.16 的接口直接套用。

## 1 先找模型

| 方案 | 文件 |
| --- | --- |
| 改进 STA，XYZ 均改进 | [drone_position_control_sta_z_tuned.slx](models/sta/drone_position_control_sta_z_tuned.slx) |
| 改进 ISTA | [drone_position_control_ista_tuned.slx](models/ista/drone_position_control_ista_tuned.slx) |

两份 SLX 与 `matlab_simulation` 中对应文件字节一致，控制函数已内嵌。打开 SLX 需要 MATLAB / Simulink；Ubuntu 上测试和移植 C++ 核不需要 MATLAB。模型的默认外力、目标和仿真终止时间沿用原模型，查看前可先读 [模型说明](models/README.md)。

## 2 文件怎么用

| 目录或文件 | 用途 |
| --- | --- |
| `models/` | 保存可回看的 Simulink 参考模型 |
| `matlab/sta/` | 改进 STA 的 XY 与高度函数 |
| `matlab/ista/` | 改进 ISTA 的包装函数、补偿预测和隐式核 |
| `matlab/common/` | 仿真用的加速度到油门换算，供理解原模型 |
| `parameters/` | 原模型参数、单位与来源记录 |
| `cpp/velocity_sta_ista.hpp` | 单轴 C++11 核，支持 `float` / `double`，无 PX4 依赖 |
| `tests/cpp/` | 分支、限幅、积分状态及 MATLAB 一致性测试 |
| `tests/matlab/controller_reference.csv` | 已由 MATLAB 原包装函数生成的 7980 组单步参考值 |
| `CMakeLists.txt` | Ubuntu 等环境的独立构建入口 |
| [算法与参数](docs/算法与参数.md) | 数学核与当前仿真参数 |
| [PX4 速度环接入说明](docs/PX4速度环接入说明.md) | 对应源码、信号方向、前馈、限幅与后续开发步骤 |
| [Codev 源码接口核对](docs/CODEV源码接口核对.md) | 压缩包版本证据、实际文件路径及与官方参考的差别 |
| [验证记录](VALIDATION.md) | 本包实际完成的检查及范围 |

## 3 复制到 Ubuntu 后先运行独立测试

将整个文件夹复制到例如 `~/sta_ista_px4_package`。在有 C++ 编译器与 CMake 的环境中：

```bash
cd ~/sta_ista_px4_package
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
```

若缺少工具，可先安装独立测试需要的工具：

```bash
sudo apt update
sudo apt install build-essential cmake
```

测试会读取包内 CSV，不需要在 Ubuntu 再安装 MATLAB。本包不重复包含约 1.3 GB 的 `Codev-autopilot.zip`，后续在 Ubuntu 中单独解压你的源码包。需要重新导出 MATLAB 参考值时，从包根目录运行：

```matlab
addpath(fullfile(pwd,'tests','matlab'));
verify_model_parameters;
export_controller_reference;
```

## 4 C++ 核的接口

输入是同一局部坐标系中的 `速度目标 − 实际速度`、显式加速度前馈、实际更新间隔 `dt` 和参数；输出是加速度指令及积分状态。x、y、z 分别保存独立状态。

```cpp
#include "velocity_sta_ista.hpp"
using namespace velocity_control;

State<float> state_x; // 放在控制器对象中持久保存，不能每次循环重建
Parameters<float> p_x{3.f, 1.f, 0.1f, -1.7f, 1.7f, 1.7f};
// 参数顺序：k1, k2, epsilon, acceleration_min, acceleration_max, integral_max

// 每次更新：前馈与 dt 由调用层提供。
auto result = propose(Algorithm::STA, p_x, state_x,
                      Input<float>{velocity_error_x, acceleration_ff_x, dt});
if (result.valid()) {
    // 使用 result.acceleration_limited 参与下游推力/倾角约束。
    // 获得约束结果后，决定该轴是否允许积分，并且每周期只提交一次：
    commit(state_x, result, allow_integral_x);
}
```

上例中的速度误差、前馈、`dt` 与积分许可由实际 PX4 适配层提供，不是现成固件补丁。ISTA 选择 `Algorithm::ISTA` 并使用其参数。详见 [C++ 接口说明](cpp/README.md)。

## 5 后续移植最关键的对应关系

1. 使用局部 NED 速度误差；z 向下为正，上升速度为负。输入不是机体系角速度。
2. 新核只产生加速度层控制量，保留 PX4 的加速度到推力换算、姿态环和控制分配。不要把这里的电机系数、PWM 或固定悬停油门搬到 PX4。
3. 原 Simulink 的 `-lambda*v` 前馈针对分段恒定位置目标；PX4 轨迹可能变化，不能无条件照搬。独立 C++ 核把前馈留给调用层决定。
4. 标量加速度限幅不等于 PX4 最终推力可实现约束。接入时还需处理下游抗饱和、无效设定值、积分重置、悬停推力变化和模式切换。
5. 当前参数用于复现已有仿真，应在选定 PX4 机型上重新整定。先保留原 PID 分支，在相同 SITL 工况下比较。

后续修改以 `Codev-autopilot.zip` 中的源码为准；官方文档仅作对照。先阅读 [Codev 源码接口核对](docs/CODEV源码接口核对.md)，再按 [接入说明](docs/PX4速度环接入说明.md) 准备 Ubuntu 和原版 SITL。
