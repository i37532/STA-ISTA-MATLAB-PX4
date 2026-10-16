# 改进 STA / ISTA 的 C++ 速度控制核心

`velocity_sta_ista.hpp` 是独立的 C++11 头文件，无 PX4 依赖、无动态内存分配。模板支持 `float` 与 `double`；未来接入 PX4 时可用 `float`。它只把**速度误差变成期望加速度**，尚未替换 PX4 固件中的控制器。

## 接口和单位

输入使用同一局部 NED 坐标系，三个轴各自维护一个 `State`：

| 字段 | 含义 |
| --- | --- |
| `Input.error` | `v_sp - v`，m/s |
| `Input.acceleration_ff` | 外部明确提供的加速度前馈，m/s² |
| `Input.dt` | 本次实际控制更新间隔，s，必须有限且大于 0 |
| `State.nu` | 积分状态，m/s² |
| `Parameters.epsilon` | 速度误差边界层宽度，m/s，必须大于 0 |
| `Parameters.acceleration_min/max` | 当前轴局部加速度限幅，m/s² |
| `Parameters.integral_max` | 积分状态绝对值上限，m/s² |
| `Proposal.acceleration_raw/limited` | 限幅前/后的加速度指令，m/s² |
| `Proposal.nu_next` | 应保存到下一次更新的积分状态 |

算法核心不包含重力、质量、PWM、油门或姿态换算。`k1` 的量纲为 `sqrt(m)/s^(3/2)`，`k2` 为 m/s³。NED 中 z 向下为正，因此 z 误差与输出也按向下为正处理。

## 与改进 Simulink 算法对应的公式

令 `clip(x,l,h)` 表示限幅，`sat(x)=clip(x,-1,1)`，

$$
\psi(s)=\sqrt{\max(|s|,\epsilon)}\operatorname{sat}(s/\epsilon).
$$

改进 STA：

$$
a_{raw}=a_{ff}+k_1\psi(e)+\nu,\qquad
\nu_{candidate}=\nu+dt\,k_2\operatorname{sat}(e/\epsilon).
$$

改进 ISTA 同时求解：

$$
s_{pred}=e-dt(u_{ISTA}-\nu),\qquad
\nu_{candidate}=\nu+dt\,k_2\operatorname{sat}(s_{pred}/\epsilon),
$$

$$
u_{ISTA}=k_1\psi(s_{pred})+\nu_{candidate},\qquad
a_{raw}=a_{ff}+u_{ISTA}.
$$

通过内、外边界层的闭式解计算，无迭代。`s_pred` 是控制律内部预测量，不是读取到的下一步实际误差。两种算法均先限幅加速度；若输出饱和且积分增量会加重同方向饱和，则冻结该次积分，随后限制积分状态幅值。这是已加入边界层、限幅和离散更新的工程版本，不能仅凭控制律形式宣称有限时间收敛。

## 单轴最小用法

```cpp
#include "velocity_sta_ista.hpp"
using namespace velocity_control;

State<float> x_state; // 保持为控制器成员，每个控制周期不能重新创建
Parameters<float> x_params = {3.f, 1.f, .1f, -1.7f, 1.7f, 1.7f};

// 放在控制更新函数中；v_sp_x、v_x、a_ff_x、dt 来自实际输入。
Input<float> input = {v_sp_x - v_x, a_ff_x, dt};
auto result = step(Algorithm::STA, x_params, x_state, input);
if (result.valid()) {
    const float ax_sp = result.acceleration_limited;
    // 交给原有加速度→姿态/推力流程。
} else {
    // 交给上层已有的输入无效/安全降级处理；不要继续发送 NaN。
}
```

示例参数仅复现现有水平 STA 仿真，不能直接当作实机整定结果。改用 `Algorithm::ISTA` 时应同时使用对应 ISTA 参数，不能把算法枚举切换视为参数已经匹配。

输出无效时，`status` 标明输入/数值问题，`step` 保持旧状态，输出诊断为 NaN。NaN 不代表安全的加速度指令。实际 PX4 适配层还需保留失效处理、有效轴判断、模式转换和时间间隔检查。

## PX4 接入时使用两阶段更新

独立的 XYZ 加速度限幅不能描述最终总推力、倾角、推力余量和分配限制。因此提供 `propose` 和 `commit`：

```cpp
auto x = propose(Algorithm::ISTA, x_params, x_state, x_input);
auto y = propose(Algorithm::ISTA, y_params, y_state, y_input);
auto z = propose(Algorithm::ISTA, z_params, z_state, z_input);
if (x.valid() && y.valid() && z.valid()) {
    // 1. 使用 x/y/z.acceleration_limited 进入现有 PX4 推力/倾角限制。
    // 2. 根据最终耦合限幅和积分增量方向，计算每轴是否允许积分。
    //    allow_x/y/z 必须由适配层的实际下游抗饱和逻辑给出。
    commit(x_state, x, allow_x);
    commit(y_state, y, allow_y);
    commit(z_state, z, allow_z);
}
```

`external_integrate=false` 保持该轴当前积分状态；它不重新计算本次已产生的指令。`commit` 会拒绝重复提交或状态已被修改的旧 proposal。必须先检查所有相关轴有效，再提交本次更新。

需要完整替换速度环的反馈/积分部分，不能继续叠加原 PX4 I 项。加速度前馈必须保留一份：现有模型的 `-lambda*v` 只对应特定固定位置目标和未限速条件；实际 PX4 轨迹、手动速度指令和动态目标不能一律套用该公式。此头文件要求由外部明确提供前馈，因此没有硬编码该式。

在解锁/落地、控制模式切换、估计器复位或某轴不受控时，按适配层策略调用 `State::reset()` 或处理状态连续性。没有已验证的 PX4 适配层前，请勿把此核心等同于可直接飞行的固件补丁。

## Ubuntu 构建和测试

在包含 `CMakeLists.txt` 的 `PX4` 文件夹下：

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
```

需要 C++11 编译器和 CMake。`velocity_core_unit` 检查正负误差、两侧边界、饱和释放、局部/外部积分冻结、状态限幅、无效输入和重复提交；`velocity_core_matlab_oracle` 读取 `tests/matlab/controller_reference.csv`，分别用 float/double 与 MATLAB 的原模型控制函数输出交叉比较。该测试是算法核一致性测试，不是 PX4 飞行验证。
