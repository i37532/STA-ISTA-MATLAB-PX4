# 移植准备包验证记录

验证日期：2026-10-16。验证对象是本包内的两份参考模型、参数和独立 C++ 数学核。

## 1 完成的检查

| 检查 | 方法 | 结果 |
| --- | --- | --- |
| 模型复制完整性 | 对源模型及副本计算 SHA-256 | 两份 SLX 均字节一致 |
| MATLAB 参考函数来源 | 与 SLX 中内嵌 MATLAB Function 的计算语句比较，忽略注释与空白 | 8 份参考函数的计算一致 |
| 参数一致性 | MATLAB R2022a / Simulink 加载临时模型副本，运行 `verify_model_parameters` | 两模型保存的参数与 JSON 一致 |
| MATLAB 单步基准 | 原 MATLAB 包装函数生成 `controller_reference.csv` | 共 7980 组，STA / ISTA 各 3990 组 |
| Windows 构建 | CMake 3.19.7 + Ninja + Zig 0.13.0 所带 Clang 18.1.6，C++11 | 编译和链接成功 |
| Windows 单元与参考测试 | 直接运行 CMake 生成的测试程序，分别执行默认模式及 `--oracle` 模式 | 两项均 `PASS: 0 failed checks` |
| Linux 单元与参考测试 | 同一 C++ 源码交叉编译为静态 x86-64 Linux ELF，在 WSL Ubuntu 24.04.5 中实际执行 | 两项均 `PASS: 0 failed checks` |
| Codev 接口核对 | 只读检查用户 ZIP 中实际源码与 Git 引用 | 分支 `main-v2_gps`；差异已记录于接口说明 |

Windows 所用 CMake 工具包没有 `ctest.exe`，因此本次在 Windows 直接执行了 CMake 注册的两个测试命令，没有声称运行过 CTest。Linux 本次执行的是交叉编译后的程序，没有在 Ubuntu 本机运行 CMake/GCC 构建；包内仍提供标准 CMake/CTest 入口，供用户在目标 Ubuntu 上复现。

## 2 数值一致性覆盖范围

每组 MATLAB 数据分别以 C++ `double` 与 `float` 运算，比较限幅前加速度、限幅后加速度、下一积分状态。包含 XY/Z 参数、三个采样间隔（0.0025 / 0.005 / 0.01 s）、正负及零误差、边界层与隐式分支边界、输出限幅、积分上限和解饱和方向。

通过判据为：

- `double`：绝对误差不超过 `5e-10 * (1 + abs(expected))`。
- `float`：绝对误差不超过 `5e-5 * (1 + abs(expected))`。

独立单元测试还覆盖外部积分冻结、重复或旧状态提交、重置、无效参数、非有限输入与数值溢出时状态保持。测试通过表示在这些用例和容限下实现一致，不表示两种浮点精度按位相同。

## 3 复现

在包根目录打开终端：

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
```

也可直接执行生成的测试程序：

```bash
./test_velocity_core
./test_velocity_core --oracle ../tests/matlab/controller_reference.csv
```

重新生成参考 CSV 需要 MATLAB；使用随包 CSV 运行 C++ 测试不需要 MATLAB。MATLAB 入口详见 [测试说明](tests/matlab/README.md)。

## 4 模型校验值

```text
drone_position_control_sta_z_tuned.slx
8cb0411d67ef2912fd6ac3b184cb0c260acf18584f7bbd19aece377d9b192253

drone_position_control_ista_tuned.slx
ab2550e576c486e25e14a714580b9e9ed8052063f3c609fb490e5afef590f04f
```

完整来源与函数校验值见 [source_manifest.json](parameters/source_manifest.json)。Codev 归档及关键源码的校验值见 [CODEV 源码接口核对](docs/CODEV源码接口核对.md)。

## 5 尚未验证的范围

本次没有修改或编译 Codev 固件，没有运行 Codev SITL，也没有重新执行整机 Simulink 闭环性能对比。独立核通过测试不能证明移植后的整机性能优于 PID；后续仍需完成信号接入、前馈与推力约束协调、状态管理，以及同工况闭环比较。

Ubuntu 24.04.5 仅是本次独立 ELF 测试环境，不代表已验证该旧 Codev/Gazebo 工程兼容 Ubuntu 24.04。Codev 环境准备应以包内 [接入说明](docs/PX4速度环接入说明.md) 和实际归档脚本为准。
