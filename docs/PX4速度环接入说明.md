# 改进 STA / ISTA 接入 PX4 速度环的说明

这份材料用于把已验证的 Simulink 算法迁移到 Ubuntu 上的 PX4 开发环境。`models` 保留改进 STA、改进 ISTA 的完整仿真模型；`matlab` 中的函数便于核对数学计算；`cpp` 是独立算法原型。**本包尚未接入实际 PX4 固件，也没有完成 PX4 编译或 SITL 验证。** 独立核通过测试，只能说明这一层计算可复现，不能等同于飞控闭环已经验证。

## 1 你的实际工程与真正要改的位置

你已指定使用 **`Codev-autopilot.zip`**。本文已据本机 `D:/THINGS/Codev-autopilot.zip` 中的实际源文件重新核对：归档内 `.git/HEAD` 指向 `main-v2_gps`，该分支引用值为 `b1e454edede00da196ed280a7c48a4faf4433616`。这只能确定归档携带的 Git 引用，不能证明归档工作文件与该提交完全相同，也不能直接认定它就是某个官方版本。具体哈希、行号及与官方版本的差异见 [CODEV源码接口核对](CODEV源码接口核对.md)。

**后续以 CODEV 工程为主；官方 v1.16.0 仅作对照，不用新版本文件直接覆盖你的工程。**

| 路径，均相对 PX4-Autopilot 根目录 | 后续工作 |
| --- | --- |
| `src/modules/mc_pos_control/PositionControl/PositionControl.cpp` | 在 `_velocityControl(dt)` 中选择 PID、STA 或 ISTA；保留加速度到推力映射和推力约束 |
| `src/modules/mc_pos_control/PositionControl/PositionControl.hpp` | 声明每轴积分状态、参数、模式切换与重置接口 |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp` | 把参数、采样时间、起降状态、估计器信息接到新控制核 |
| `src/modules/mc_pos_control/MulticopterPositionControl.hpp` | 参数成员与模块接口；你的工程使用这个文件名 |
| `src/modules/mc_pos_control/mc_pos_control_params.c` | 你的工程将参数集中在此；新算法参数须正式声明并加载 |

在你的 `PositionControl.cpp` 中，速度环从第 123 行开始，第 126–127 行生成误差与 PID 加速度反馈，第 130 行合入加速度前馈，第 132 行进入推力映射。**这就是后续新控制核最直接的接入位置。** 本次仅定位并整理，没有修改归档中的固件。

推荐的改动范围是：

```text
位置 / 速度 / 加速度轨迹目标
              ↓
保留 PX4 位置外环、速度限制与轨迹前馈
              ↓
速度误差 s = v_sp - v
              ↓
PID / 改进 STA / 改进 ISTA → 加速度反馈
              ↓
加上有效的加速度前馈
              ↓
保留 PX4 加速度→推力方向、悬停推力、倾角和总推力约束
              ↓
保留姿态环、角速度环、控制分配、电机输出
```

这里改的是**平移速度环**，不是 roll / pitch / yaw 角速度环。最先只替换速度反馈计算，其他环节保持相同，才能判断性能变化来自哪里。

## 2 你的符号与 PX4 变量怎样对应

| 你的符号 / 含义 | PX4 参考变量 | 单位与方向 |
| --- | --- | --- |
| 位置、速度 | `_pos`、`_vel` | m、m/s，局部 NED |
| 经过位置环和限制后的目标速度 | `_vel_sp` | m/s，局部 NED |
| 速度误差 $s=v_d-v$ | `vel_error` | m/s；建议新核使用这个符号约定 |
| 实际速度导数 | `_vel_dot` | m/s²；你的工程使用 `BlockDerivative` 由速度计算，不能直接当作未经处理的 IMU 比力 |
| 轨迹加速度前馈，随后合入反馈 | `_acc_sp` | m/s²，是运动加速度，不是 PWM 或牛顿 |
| 原速度积分项 | `_vel_int` | 加速度量纲；换控制器时需明确由谁保存和更新积分状态 |
| 推力指令 | `_thr_sp` | 归一化推力向量；不是四电机油门之和 |

NED 的 x / y / z 分别是北 / 东 / 下；机体系 FRD 则是前 / 右 / 下。它们的 z 都向下，但不是同一套坐标轴。速度环用局部 NED，不能把机体前向速度直接填进 NED 的 x。上升对应 $v_z<0$；增加向上运动加速度对应 $a_z<0$。你的 `msg/vehicle_local_position_setpoint.msg` 第 1–18 行直接定义了 NED 单位，并让 `trajectory_setpoint` 复用同一种消息；不要照搬新版独立 `trajectory_setpoint_s` 的字段。

因此，移植核的基本接口应是：输入有效的速度误差、内部状态和本次 `dt`，输出加速度反馈。**不要把 Simulink 中的 `0.60899`、电机 PWM `1000..2000`、转速系数、推力平方律或重力补偿直接塞进 `_acc_sp`。** PX4 下游已有自己的悬停推力与姿态生成链路；重复补偿会改变物理含义。

## 3 为什么不能把完整 MATLAB 函数直接粘进去

### 3.1 固定目标前馈要与轨迹前馈分开

你的仿真在固定位置目标、速度未限幅时使用：

$$
v_d=\lambda(p_d-p),\qquad \dot v_d=-\lambda v,
\qquad a_d=-\lambda v+u_{\rm STA/ISTA}.
$$

PX4 的目标可能是运动轨迹，并且位置反馈会与速度前馈组合、再受速度限制。若忽略限幅，且输入为参考速度 $v_{\rm ff}$，则一般关系是：

$$
v_d=K_p(p_d-p)+v_{\rm ff},\qquad
\dot v_d=K_p(\dot p_d-v)+\dot v_{\rm ff}.
$$

只有固定位置、零速度前馈等条件下才退化为 $-K_pv$。**第一版固件适配建议保留 PX4 已有加速度前馈，只替换反馈核。** 若要复现 Simulink 的“完整目标速度导数补偿”，应另设实验分支，正确处理参考导数和限幅切换，再与只替换反馈的版本比较；不能无条件加上 $-\lambda v$。

### 3.2 `NaN` 不是普通零值

PX4 用 `NaN` 表示某层设定值未启用。例如只有加速度目标时，不应因为把无效速度目标改成 0，就额外启动“刹停至零速度”的反馈。接入时要逐分量检查有效性：速度控制有效才调用对应核；无效分量不更新其状态，并保留已有轨迹前馈。XY 的输入有效性还有成对要求。你的 `PositionControl.hpp` 第 135–140 行和 `PositionControl.cpp` 第 91–105、189–208 行是对应依据。当前 CODEV 在控制计算之后才完成有效性检查，因此新状态尤其应采用“试算成功后提交”。

### 3.3 时间步长使用 PX4 实际传入的 `dt`

Simulink 的控制采样周期是 0.005 s；你的 CODEV 在 `MulticopterPositionControl.cpp` 第 265–271 行根据 **`local_pos.timestamp`** 计算 `dt`，限制在 0.002–0.04 s，再传给控制与微分器。新核的显式积分、隐式预测、滤波全部应使用本次 `dt`；不能写死 0.005 s。第 421–440 行表明更新失败后会换用失效保护目标并在同一周期再次调用 `update(dt)`；先试算、成功后提交，可以避免失败重试重复积分。

### 3.4 抗饱和必须知道下游真正受到了什么限制

独立核的标量输出限幅，只能处理该核自己的加速度上下限。PX4 后面还有倾角、总推力和竖直优先分配约束，三个轴会相互影响。你的 `PositionControl.cpp` 第 134–173 行对 Z 使用饱和方向判断，对 XY 使用受限推力反算的加速度进行积分回算；没有新版的水平推力预留量。新 STA / ISTA 的状态也要收到这些约束的反馈；不能保留旧 PID 积分更新、又增加一套独立积分，造成两套状态同时作用。

建议在实际固件分支采用“计算候选输出 → 下游约束 → 回传可实现控制量 → 决定状态提交”的接口。条件积分、回算系数、ISTA 隐式状态在限幅后的处理需要重新验证，不能把原 PID 中依赖 P 增益的回算系数机械套用到 STA。

### 3.5 重置和悬停推力变化同样属于控制器

你的 CODEV 已有 `resetIntegral()` 和 `updateHoverThrust()`，**没有 `resetIntegralXY()`**。`PositionControl.hpp` 第 157 行的重置实际清除整个向量，虽然其注释写着 XY。上层第 372–378 行在未起飞/地面接触时重置；第 457–460 行退出位置控制分支没有相应清零。后续必须明确增加新核的按轴重置和模式切换策略，不能假定新版行为已经存在。

实际接入应逐项验证：未起飞、地面接触、退出位置控制、XY 不再受控、PID/STA/ISTA 切换、估计器重置、失效回退和悬停推力估计更新。新积分状态 $\nu$ 需要明确清零或平滑迁移策略。你的 `PositionControl.cpp` 第 68–71 行使用 $\Delta\nu_z=(T_{h,\rm new}-T_{h,\rm old})g/T_{h,\rm new}$ 补偿悬停推力变化；这不是 v1.16 的同一个公式，尤其不能宣称所有非悬停工况均严格无跳变。迁移后应验证新的状态补偿与输出连续性。

## 4 Ubuntu 上建议怎么开始

先把整个本包复制到 Ubuntu 的一个独立目录，例如 `~/sta_ista_px4_package`；SLX 供 MATLAB/Simulink 复现实验，C++ 核供移植和独立测试。它们不需要直接覆盖 PX4 现有源码。

你已经有 CODEV 工程，优先保留原开发环境、原分支和原仿真目标。下面命令只查询环境与代码状态，供后续适配记录：

```bash
cd ~/Codev-autopilot   # 改成你的实际目录
lsb_release -a
git branch --show-current
git rev-parse HEAD
git status --short
git submodule status
```

归档的 `Tools/setup/ubuntu.sh` 是旧环境脚本：显式处理 Ubuntu 18.04 / 20.04，并安装 Gazebo 9 / 11 等依赖。它不是当前 v1.16 的 Harmonic 安装脚本；本次没有运行它。若你已有能编译的环境，继续使用；若环境尚未搭好，需要根据真实 Ubuntu 版本处理依赖兼容性，不能默认执行新版本安装流程覆盖旧环境。

归档的 `platforms/posix/cmake/sitl_target.cmake` 明确列出旧式 `gazebo` 和 `jmavsim` 目标，并使用 `Tools/sitl_gazebo`。依赖已准备好后，先运行**未加 STA/ISTA 的 CODEV**：

```bash
make px4_sitl_default gazebo
```

这与新版本的 `make px4_sitl gz_x500` 是不同一代仿真接口。本次只根据归档构建文件核对目标，没有执行 CODEV 构建或 SITL，实际能否运行还需在 Ubuntu 验证。

## 5 接入、整定和比较的顺序

1. 保存原版 PID 的 SITL 日志，固定机型、质量、轨迹、速度/加速度/推力限制、估计器和采样配置。
2. 在实际 PX4 分支增加可选择的 PID / STA / ISTA 模式，默认保留 PID；先验证重置、有效分量、输出符号、有限值和上下限。
3. 先做 XY，再独立做 Z；Z 的增益和积分上限单独整定。Simulink 的参数是研究起点，不是 PX4 通用参数。
4. 对每种算法依次做定点、速度阶跃、平滑轨迹、同样的外部扰动，以及限幅退出和模式切换实验。
5. 最后做消融：仅换控制核、增加前馈适配、再改抗饱和/参数，分别比较，避免把多项改动的收益全归给算法名称。

指标沿用本项目的超调、调节时间、RMSE、IAE、扰动峰值、恢复时间；另外记录速度与加速度误差、推力饱和占比、控制指令波动、实际 `dt` 和执行时间。对所有方案使用相同限制。PX4 的归一化推力变化量可以作为控制平滑性指标，但不能直接称作能耗，也不能直接与原 Simulink PWM 数字比较。

**目标工程已明确为 CODEV；下一阶段需要核对 Ubuntu 中实际工作目录的版本、已有修改和可用仿真环境。** 然后把本包数学核接成可编译的固件改动，并完成相同条件下的 SITL 对比；当前模型和独立代码保留为数值参照。

## 6 官方新版只用来对照

研究时另核对过官方已发布 **PX4 v1.16.0**，提交 `6ea3539157ca358c70a515878b77077af7d4611d`。其独立轨迹消息、积分重置、水平推力预留量、悬停推力补偿和开发环境都有变化，不能直接视作 CODEV 的接口。官方资料：[发布页](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.0)、[PositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/6ea3539157ca358c70a515878b77077af7d4611d/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)、[上层模块](https://github.com/PX4/PX4-Autopilot/blob/6ea3539157ca358c70a515878b77077af7d4611d/src/modules/mc_pos_control/MulticopterPositionControl.cpp)、[Ubuntu 文档](https://docs.px4.io/v1.16/en/dev_setup/dev_env_linux_ubuntu)、[新版 Gazebo 文档](https://docs.px4.io/v1.16/en/sim_gazebo_gz/index)。
