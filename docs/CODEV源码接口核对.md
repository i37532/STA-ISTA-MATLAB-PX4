# CODEV 源码接口核对记录

核对日期：2026-10-16。方法：直接用 ZIP 读取 API 读取指定文本与 Git 引用；没有解压执行、没有运行归档脚本、没有改动固件，也没有把整个 1.3 GB 工程加入本移植包。

## 1 输入来源与可确认的版本信息

| 项目 | 核对结果 |
| --- | --- |
| 本机源文件 | `D:/THINGS/Codev-autopilot.zip` |
| ZIP 字节数 | `1317588254` |
| ZIP SHA-256 | `63a4520ab91e3269316303a481d857331d46ae76ff974c3287c1b4d47a4bf9ce` |
| 归档根目录 | `Codev-autopilot/` |
| `.git/HEAD` 第 1 行 | `ref: refs/heads/main-v2_gps` |
| `.git/refs/heads/main-v2_gps` 第 1 行 | `b1e454edede00da196ed280a7c48a4faf4433616` |
| `.git/packed-refs` | 包含 `v1.12.3-1.0.1`、`v1.12.3-1.0.4`、`v1.12.3-1.0.6` 等厂商命名标签；当前分支引用不等于这些标签的引用值 |

**准确描述是“CODEV 的 `main-v2_gps` 源码归档”。** 标签名字和 README 的 PX4 名称不能证明它等同官方 v1.12.3，也不能证明 ZIP 内工作文件没有未提交改动。本次未完整校验 Git 提交树；以下结论直接依据 ZIP 内当前文件的内容。Ubuntu 上如已有修改，应以实际目录再核对。

下文路径相对于归档内 `Codev-autopilot/`，行号按文件原文从 1 开始，适用于本 SHA 的 ZIP。

## 2 速度环接入位置

| 实际文件 | 行号 | 已核实的作用 |
| --- | --- | --- |
| `src/modules/mc_pos_control/PositionControl/PositionControl.cpp` | 82–89 | 输入类型为 `vehicle_local_position_setpoint_s`，字段使用 x/y/z、vx/vy/vz、acceleration |
| 同上 | 91–105 | XY 目标有效性成对检查；先执行控制，再用 `_updateSuccessful()` 判断输出 |
| 同上 | 107–120 | 位置 P、速度前馈相加、XY/上下速度限制 |
| 同上 | 123–132 | `s=_vel_sp-_vel`；PID 输出加速度反馈，与 `_acc_sp` 相加，再进入推力映射 |
| 同上 | 134–173 | Z 饱和方向抑制积分、竖直推力优先、XY tracking anti-windup、积分更新与 Z 状态限制 |
| 同上 | 176–186 | 加速度变为倾角/归一化推力，使用 g、悬停推力和倾角约束 |
| 同上 | 189–208 | 控制状态与最终加速度/推力有限值检查 |
| `src/modules/mc_pos_control/PositionControl/PositionControl.hpp` | 157 | `resetIntegral()` 清零全部 `_vel_int`；没有 `resetIntegralXY()` |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp` | 150–154 | 位置/速度增益从参数传给控制器 |
| 同上 | 194–246 | 估计器有效性检查，`BlockDerivative` 生成 `_vel_dot` 对应状态 |
| 同上 | 265–271 | `dt` 使用 `local_pos.timestamp`，限制为 0.002–0.04 s，传给微分器 |
| 同上 | 279–285 | 接收有效的悬停推力估计并更新控制器 |
| 同上 | 295–318 | 根据估计器重置量调整旧目标 |
| 同上 | 367–378 | 未进入飞行阶段移除 Z 前馈；未起飞/地面接触时重置目标与积分 |
| 同上 | 421–440 | 更新失败后用 failsafe 目标再次调用控制器，发生在同一个循环内 |
| 同上 | 457–460 | 位置控制停用分支更新起飞状态，未调用 `resetIntegral()` |
| `src/modules/mc_pos_control/MulticopterPositionControl.hpp` | 105、114 | 订阅名是 `trajectory_setpoint`，保存类型仍为 `vehicle_local_position_setpoint_s` |
| 同上 | 132–175 | 参数成员声明位置 |
| `src/modules/mc_pos_control/mc_pos_control_params.c` | 74、85 | `MPC_THR_HOVER`、`MPC_USE_HTE` 定义 |
| 同上 | 158、231、468 | `MPC_Z_VEL_P_ACC`、`MPC_XY_VEL_P_ACC`、`MPC_VELD_LP` 定义 |
| `msg/vehicle_local_position_setpoint.msg` | 1–18 | NED、m/s、m/s²；`vehicle_local_position_setpoint` 和 `trajectory_setpoint` 复用同一消息 |

推荐首版改动：在速度加速度反馈这一层增加可选择的算法分支，保留位置环、轨迹前馈、推力/倾角约束和下面的姿态环。新增状态需要参与所有重置、失效和悬停推力处理；只替换第 127 行公式还不够。

## 3 和官方 v1.16.0 的差异

对照基准：[官方 `6ea3539157ca358c70a515878b77077af7d4611d`](https://github.com/PX4/PX4-Autopilot/commit/6ea3539157ca358c70a515878b77077af7d4611d)。对照用于发现接口不同，不表示要将 CODEV 升级。

| 项目 | 此 CODEV 归档 | 官方 v1.16.0 对照 | 对移植的影响 |
| --- | --- | --- | --- |
| 目标类型 | `vehicle_local_position_setpoint_s` | `trajectory_setpoint_s` | 不能直接复制新版成员访问方式 |
| 输入/输出有效性 | `update()` 先算，末尾 `_updateSuccessful()` | 先 `_inputValid()`，再执行控制 | 独立核应检查有效分量并试算状态，失败不提交 |
| 时间来源 | `local_pos.timestamp` | `timestamp_sample` | 用本工程传来的 `dt`，不要自行另设采样周期 |
| 状态微分 | `control::BlockDerivative` | 速度/导数滤波器链 | 噪声与相位延迟不同，参数要重新整定 |
| XY 抗饱和 | 始终计算 tracking 修正项 | 增加特定饱和判据后启用 | 新算法不能机械复用依赖 P 增益的回算系数 |
| 水平推力预留 | 无单独预留量 | 有水平推力 margin | 保持 CODEV 原约束作为首轮公平基线 |
| 姿态生成的 Z 项 | 使用固定 g | 有水平/垂直加速度解耦选项 | 不应把新版本选项当作现成参数 |
| 积分重置 | 只有整个积分向量清零 | 另有 XY 清零；上层停用时也重置 | 新核需要显式添加逐轴与模式生命周期处理 |
| 悬停推力状态补偿 | 使用悬停附近的简化式 | 使用含当前 `_acc_sp(2)` 的补偿式 | 迁移后应验证推力连续性，不能说两版完全一样 |
| 参数定义 | 一个 `mc_pos_control_params.c` | 按功能拆成多个 `*_params.c` | 对实际文件声明、加载新参数 |
| SITL 工具链 | `Tools/sitl_gazebo`、Gazebo 9/11 脚本 | Gazebo Harmonic、`gz_x500` | CODEV 不直接使用新版仿真命令 |

CODEV 的悬停推力补偿在 `PositionControl.cpp` 第 68–71 行：

$$
\nu_z\leftarrow\nu_z+(T_{h,\mathrm{new}}-T_{h,\mathrm{old}})\frac{g}{T_{h,\mathrm{new}}}.
$$

对新的 STA/ISTA，必须明确补偿的是新积分状态还是旧 `_vel_int`，不能两处重复补偿，也不能使其失效。该式并不是所有运动加速度下严格保持原推力的完整公式，应在实际分支验证。

## 4 Ubuntu / SITL 现状核对

- `Tools/setup/ubuntu.sh` 第 3–9 行说明旧工具链；第 54–68 行显式区分 Ubuntu 18.04/20.04；第 192–227 行选择 Gazebo 9/11 和相应依赖。本次没有执行安装脚本，不能据此承诺它在当前 Ubuntu 上安装成功。
- `platforms/posix/cmake/sitl_target.cmake` 第 34–36 行使用 `Tools/sitl_gazebo`；第 89–93 行列出 `gazebo` / `jmavsim`；第 151–178 行生成目标。因此环境准备好后，原工程基线入口为 `make px4_sitl_default gazebo`，不是 `gz_x500`。
- ZIP 内 `Tools/sitl_gazebo` 有 510 个非目录条目，`Tools/jMAVSim` 有 141 个，`src/lib/matrix` 有 60 个；它们确实有内容，但文件存在不代表子模块状态正确或可编译。
- Ubuntu 侧先记录 `lsb_release -a`、`git branch --show-current`、`git rev-parse HEAD`、`git status --short` 与 `git submodule status`。复核是否与本归档相同，再开展适配。

## 5 关键原文哈希

哈希用于与 Ubuntu 文件比对，避免行号和接口随改动失效；Linux 可运行 `sha256sum 路径` 查看。

| 原文文件 | SHA-256 |
| --- | --- |
| `src/modules/mc_pos_control/PositionControl/PositionControl.cpp` | `8bc5a6509b0bbb8df05b2348552d0dc639e72280cc0e5448c6b082018bfad499` |
| `src/modules/mc_pos_control/PositionControl/PositionControl.hpp` | `fa0536781f6b69ba6ce5b1521f6b347ab30db4caea425572fb9eb420a5f55ec2` |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp` | `14def4699f764c1ec8ce1d775e5d0e49240a6aa213d51b2b16ddae6343fd8195` |
| `src/modules/mc_pos_control/MulticopterPositionControl.hpp` | `ce6b07488c5363aa9740df48e5704801e80ad22240df63e2dde97fa75fe50ce2` |
| `src/modules/mc_pos_control/mc_pos_control_params.c` | `9ce32fd677982cdc1979a0647b8cdfb043240341b74bc14fb7d74b659ccaef18` |
| `Tools/setup/ubuntu.sh` | `124b61a1c5a8ed333f1e9f1d033da0f93002baa7a06f572f47d293e1e807ac9a` |

本记录完成的是源码接口核对；尚未形成 CODEV 固件补丁，未执行固件构建、SITL 或实飞。模型复现、独立 C++ 测试和飞控集成是三个不同的验证层次。
