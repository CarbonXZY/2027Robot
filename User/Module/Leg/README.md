# 腿控制模块（Module::Leg）

本模块控制由 **4 个齿轮齿条** 组成的腿机构：前左（lf）、前右（rf）、后左（lr）、后右（rr），每个齿轮齿条由一台 C620 电机驱动。

控制目标不是"4 个电机转同样的角度"，而是：

- **前组** lf + rf：两条腿的真实高度完全同步；
- **后组** lr + rr：两条腿的真实高度完全同步；
- **前组与后组之间互不同步**，各自跟踪各自的目标高度。

目录中 `regular_test/` 是方案验证时的试验代码（`RegularMotor`，单套规划 + 单套耦合），保留作参考，不参与整机运行；实际模块是 [mod_leg.h](mod_leg.h) / [mod_leg.cpp](mod_leg.cpp)。

---

## 1. 任务空间与关节空间

齿轮齿条把电机旋转变成齿条的直线运动，因此"腿的高度"是直线量。

| 空间 | 本任务中的物理量 | 维数 | 代码中的载体 |
|---|---|---|---|
| **关节空间** | 4 台电机的转角 θ（rad） | 4 | `MotorGroup`、`offset_[]`、`output_.position_real[]` |
| **任务空间** | 前侧腿高 `h_front`、后侧腿高 `h_rear`（m） | 2 | `Height now_height_`、`Height target_`、两套 `Interpolator` |

这里是一个**冗余驱动**系统：任务空间只有 2 个自由度，关节空间有 4 个自由度，多出来的 2 个内部自由度由交叉耦合消除——即强制同组两条腿高度相同。

因此控制分工是：

- **规划、目标、速度、同步都在任务空间（高度）里做**；
- **关节空间只负责两件事**：把电机转角换算成高度（反馈），把高度指令换算回电机转角（输出）。

映射关系（下标 `i` 为 lf/rf/lr/rr）：

```text
反馈： h_i = (θ_i - θ_i,offset) · direction_sign[i] · equivalent_radius[i]
输出： θ_i = h_group · direction_sign[i] / equivalent_radius[i] + θ_i,offset
```

- `equivalent_radius`：电机转角到齿条直线位移的等效半径（m/rad），**已包含减速比**；
- `direction_sign`：±1，统一机构安装方向；
- `offset`：主动找限位堵转时的编码器角度，进入 `kRunning` 时按 `initial_height` 平移，使机械下限位对应高度 `initial_height`、目标 0 对应安全位置。

---

## 2. 代码组织

```text
User/Module/Leg/
├── mod_leg.h / mod_leg.cpp   腿模块（本文件描述的对象）
├── README.md                 本文档
└── regular_test/             试验代码，提供已验证的控制思路
    ├── regular_motor.h/.cpp  单组 N 电机同步（本模块的参照实现）
    └── regular_motor_instance.*  试验实例
```

模块内部使用的既有算法：

| 组件 | 位置 | 作用 |
|---|---|---|
| `Algorithm::Interpolator` | `User/Algorithm/alg_interpolator.*` | 两点间插值，生成虚拟主轴的位置轨迹（线性/多项式/S 型） |
| `Algorithm::CrossCouple<N>` | `User/Algorithm/alg_cross_couple.*` | 组内位置平均 + 按偏差输出同步补偿 |
| `Device::MotorDjiC620` | `User/Device/dvc_motor_dji.*` | 电机位置环、速度前馈、CAN 收发、掉线检测 |

`Module::Leg` 内部持有：

```text
motor_group_            4 台电机（lf/rf/lr/rr）
offset_[4]              校准偏移
front_master_axis_      前组虚拟主轴（插值到目标高度）
rear_master_axis_       后组虚拟主轴
cross_couple_front_     前组同步，CrossCouple<2>
cross_couple_rear_      后组同步，CrossCouple<2>
now_height_             当前前/后腿高（组内平均）
target_ / prev_target_  前/后目标高度
output_                 发给 4 台电机的位置目标与速度前馈
```

两个耦合器相互独立，前组电机只进入 `cross_couple_front_`，后组只进入 `cross_couple_rear_`，两组不存在任何数据交叉。

---

## 3. 1ms 控制流程

```text
TimCalculatePeriodElapsedCallback()
        │
        ├─ 状态机                kInit → kCalibrate（主动找下限位）→ kRunning
        ├─ UpdateData()          电机角度 → 高度；组内平均得到 now_height_
        ├─ NewTargetHandler()    目标高度变化时，按平均速度计算步数并重置本组插值器
        ├─ CalculateOutput()     插值器给出本组高度/速度，换算成电机位置与速度前馈，叠加同步补偿
        └─ OutputToMotor()       写入 4 台电机的目标/前馈（Calculate 统一在状态机之后执行）
```

数据流总览：


```text
┌──────────── 关节空间 ────────────┐
│ 4× C620  now_angle               │
└──────────────┬───────────────────┘
               │ 每台: − offset_[i]
               ▼
    h_i = (angle−offset)·sign·radius        单腿高度
               │
       ┌───────┴────────┐
       │ lf, rf         │ lr, rr
       ▼                ▼
 CrossCouple(2)   CrossCouple(2)             组内平均
       └───────┬────────┘
               ▼
    now_height_.front / .rear
               │
 target_.front/.rear ─► NewTargetHandler     任务空间规划
               │        步数 = abs(Δh) / (v·1ms)
               ▼
 front_master_axis_ / rear_master_axis_
               │ master_pos, master_speed
               ▼
 θ = master_pos·sign/radius + offset
 speed_kff = master_speed·sign/radius
               │ + CoupleOut·sign/radius    组内同步补偿
               ▼
 position_real / speed_kff  ─────────────►  4× C620
```

---

## 4. 各环节说明

### 4.1 状态机与主动回零

```text
                 Init()
   kInit ─────────────────────────► kCalibrate
     │                                 │
     │ 4 台都 kEnable                   │ 4 台都找到下限并记录 offset
     ▼                                 ▼
 （等待，不输出）                   kRunning ◄──── 目标跟踪 / 保持
                                       │
                                       │ 任意电机掉线
                                       ▼
                                    kError（锁存）
```

| 状态 | 行为 |
|---|---|
| `kInit` | 只等待 4 台电机 `GetStatus() == kEnable`，不输出、不调用 `Calculate()` |
| `kCalibrate` | 4 台切换到速度环参数，各自低速向下找机械限位；堵转后记录 `offset_` 并切回位置环保持 |
| `kRunning` | 正常执行 `UpdateData → NewTargetHandler → CalculateOutput → OutputToMotor` |
| `kError` | 锁存，停止更新目标；速度前馈清零，保持最后的 position 目标 |

- **零点由机械下限位决定**：找限位时每台电机各自堵转，堵转位置就是该腿的高度 0。因此目标是"从下限位往上的高度"，取值范围 `[0, Mechanical::max_height]`。
- 进入 `kRunning` 时零点会按 `initial_height`（默认 -0.01 m）平移：校准位置对应的高度变成 `initial_height`，随后 `ResetMotion()` 的目标 0 会把它抬高 `-initial_height`，离开机械限位到安全位置。`initial_height` 必须非正。
- 找限位方向必须与 `direction_sign` 一致：`motion_value = -homing.speed * direction_sign / equivalent_radius`。
- 找限位检测使用 `CalibrateDetectMode::kSpeed`：实测转速持续低于 `speed_threshold` 达 `debounce_ms` 即判定到底。阈值以高度单位 m/s 配置，下发前按各电机的 `equivalent_radius` 折算成输出轴角速度。
- 校准期间不运行交叉耦合，因为 offset 还没有全部建立，高度没有意义。
- `kError` 目前只锁存，清除与降级策略后续再实现。
- `GetHeight()` 在 `kInit` / `kCalibrate` 阶段无效，只有 `kRunning` 之后才有意义。

### 4.2 编码器偏移

```cpp
// 找限位阶段: 记录原始堵转角度, 用于等待其他腿时保持位置
offset_[i] = 找限位堵转时的角度;

// 进入 kRunning 时: 平移零点, 使校准位置对应 initial_height
offset_[i] -= initial_height * direction_sign[i] / equivalent_radius[i];
```

上电后必须先完成主动回零，`offset_` 才有效。校准期间 `offset_` 是原始堵转角度；进入 `kRunning` 后它表示"高度 0 对应的角度"，不再等于原始堵转角度。此时位置目标 `offset_[i]` 对应的高度就是 0（比校准点高 `-initial_height`）。

### 4.3 角度 → 高度

```cpp
height_i = (GetNowAngle() - offset_[i]) * direction_sign[i] * equivalent_radius[i];
```

`GetNowAngle()` 是弧度；乘 `equivalent_radius`（m/rad）后得到直线位移。`direction_sign` 用来统一同一组内两台电机的正方向，使"高度增大"对两台电机都是同一个物理方向。

### 4.4 组内平均与同步

```cpp
cross_couple_front_.SetNow(height, i);      // i = 0/1 → lf/rf
cross_couple_rear_.SetNow(height, i - 2);   // i = 2/3 → lr/rr，组内编号 0/1
```

两组分别求平均，得到 `now_height_.front` / `now_height_.rear`。规划器用的就是这个平均值，而不是某一条腿的位置。

耦合补偿在 `CalculateOutput()` 中计算：

```text
CoupleOut[i] = kp · (avg_height - height_i)
speed_kff[i] += CoupleOut[i] · direction_sign[i] / equivalent_radius[i]
```

它把"本腿落后/超前本组平均多少米"折算成一个速度修正，因此 `kp` 的物理量纲是 1/s（位置误差 → 速度）。`kp` 越大收敛越快，过大会振荡；前后两组可分别用 `SetFrontCrossCoupleKp()` / `SetRearCrossCoupleKp()` 整定。

### 4.5 目标与速度规划（任务空间）

- 目标单位是**高度 m**，不是电机角度。
- 目标变化时，按设定的平均速度 `average_speed`（m/s）算需要多少个 1ms 周期：

  ```text
  total_step = ceil(|curr_height - target| / (average_speed · 0.001))
  ```

  再用 `Interpolator::Reset(curr, target, total_step, type)` 建立本组轨迹。
- 前后两组独立判断、独立重置，所以可以一组运动、另一组保持不动。
- `Interpolator` 每拍给出 `master_pos`，相邻两拍之差除以 1ms 就是 `master_speed`；到点后差值自然为 0。
- 曲线类型可选 `kLinear` / `kPolynomial` / `kSigmoid`，推荐 `kPolynomial`（两端速度为 0，起停平滑）。

### 4.6 高度 → 电机指令（关节空间）

```text
ratio = direction_sign[i] / equivalent_radius[i]
position_real[i] = master_pos   · ratio + offset_[i]
speed_kff[i]     = master_speed · ratio
```

位置目标用于电机自身的位置环；速度前馈用于抵消位置环滞后。两者都按每台电机自己的等效半径和方向换算。

`LimitOutput()` 在每个周期把两者分别限制在 `2 × average_speed` 以内（按各电机 `equivalent_radius` 折算成输出轴 rad/s）：速度前馈直接限幅，位置环则通过 `pid_position.SetOutMax()` 限制其速度目标输出。两项是各自限幅，叠加后的速度目标最坏可到 4 倍平均速度。

### 4.7 100ms 回调

```cpp
Tim100msAlivePeriodElapsedCallback();
```

循环 4 台电机调用 `Tim100msAlivePeriodElapsedCallback()`，负责电机掉线检测。必须放在 100ms 分频里调用。

---

## 5. 使用示例

### 5.1 机械参数与实例

```cpp
#include "mod_leg.h"
#include "dvc_motor_instances.h"

// 顺序固定: lf, rf, lr, rr
// equivalent_radius 单位 m/rad, 已含减速比; direction_sign 取 ±1
// max_height 为前/后组各自允许的最大高度 (m)
Module::Leg::Mechanical g_leg_mechanical = {
    .equivalent_radius = {0.02f, 0.02f, 0.02f, 0.02f},
    .direction_sign   = {1, -1, 1, -1},
    .max_height       = {0.6f, 0.6f},
};

Module::Leg::MotorGroup g_leg_motors = {
    .lf = &Device::g_leg_front_left,
    .rf = &Device::g_leg_front_right,
    .lr = &Device::g_leg_rear_left,
    .rr = &Device::g_leg_rear_right,
};

// 机械参数在构造时固定, 之后不再修改
Module::Leg g_leg(g_leg_motors, g_leg_mechanical);

// 主动找限位参数: 下降速度、堵转速度阈值、消抖时间
Module::Leg::Homing g_leg_homing = {
    .speed = 0.02f,             // 0.02 m/s
    .speed_threshold = 0.002f,  // 0.002 m/s
    .debounce_ms = 150,         // 150 ms
};
```

> 请使用 `MotorGroup{...}` 或 `std::array<...>{...}` 显式指定构造类型，避免 `Leg{a, b, c, d}` 在两个构造函数之间产生歧义。

### 5.2 初始化与回调

```cpp
void TaskInit()
{
    // 必须在打开 CAN 接收中断之前完成电机对象创建
    g_leg.Init(
        0.05f,                                          // 平均速度 0.05 m/s
        g_leg_homing,                                   // 主动找限位参数
        Algorithm::Interpolator::Type::kPolynomial,     // 平滑起停
        150.0f);                                        // 交叉耦合 kp, 单位 1/s

    // 初始化后是 kInit: 等电机在线后自动 kCalibrate → kRunning
    // 可以通过 GetStatus() 查询进度, kRunning 之后才能设置目标

    Driver::FdcanInit(&hfdcan1, Task::Fdcan1Callback);
    Driver::TimInit(&htim5, Task::Tim1msCallback);
    HAL_TIM_Base_Start_IT(&htim5);
}

void Tim1msCallback()
{
    g_leg.TimCalculatePeriodElapsedCallback();
    Driver::Tim1msCanPeriodElapsedCallback();
    // 其余 1ms 任务...

    static uint16_t mod_alive = 0;
    if (++mod_alive >= 100)
    {
        mod_alive = 0;
        g_leg.Tim100msAlivePeriodElapsedCallback();
    }
}
```

### 5.3 设置目标

```cpp
// 前组抬到 0.10 m, 后组抬到 0.15 m (相对各自的下限位)
g_leg.SetTarget(0.10f, 0.15f);

// 也可以单独改一组
g_leg.SetTargetFront(0.12f);
g_leg.SetTargetRear(0.18f);

// 读取当前真实腿高
Module::Leg::Height now = g_leg.GetHeight();
float front = now.front;
float rear  = now.rear;

// 运行时调参
g_leg.SetAverageSpeed(0.08f);          // m/s
g_leg.SetCrossCouple(false);           // 关闭组内同步补偿
g_leg.SetFrontCrossCoupleKp(200.0f);   // 只调前组
g_leg.SetRearCrossCoupleKp(150.0f);    // 只调后组
```

### 5.4 上机调试建议

1. 先确认 4 台电机 ID 与实物 lf/rf/lr/rr 的对应关系，`direction_sign` 逐个验证：手动抬高一条腿，`GetHeight()` 应朝正方向变化。
2. 首次运行把 `average_speed` 设小（如 0.02 m/s）、目标行程设短（如 0.01 m），确认前后组方向正确、组内两条腿同步。
3. 再逐步加大行程与速度，最后再整定交叉耦合 `kp`：从 0 开始加大，直到组内偏差能快速收敛且不振荡。
4. 目标高度是相对**机械下限位**的增量（回零后下限位即 0）；如果需要绝对离地高度，需要在上层按机构安装高度另加偏置。
