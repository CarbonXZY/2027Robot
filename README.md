# R2_Embedded

R2 机器人的**下位机固件**，运行在 STM32H723VGT6（Cortex-M7，C++20）上。
一块 MCU 通过一条 **USB-CDC** 链路与上位机相连，链路之上复用出底盘、腿、遥控多路数据；
电机闭环（电流环/速度环/位置环）全部在下位机完成，上位机只负责指令下发与状态回读。

配套上位机工程 [`R2_PC`](../R2_PC) 基于 ROS 2 Humble + `ros2_control`。

---

## 1. 系统概览

```
        ┌──────────────────────── 上位机 R2_PC (ROS 2) ────────────────────────┐
        │  ros2_control_node ─┬─ chassis 硬件接口 ─ mecanum_drive_controller    │
        │                     ├─ leg 硬件接口     ─ leg_controller（自定义）    │
        │                     └─ Telecontrol 接口 ─ telecontrol_broadcaster     │
        │                       └── 共用同一个「通信中间件」实例 ──┐            │
        └──────────────────────────────────────────────────────────┼───────────┘
                                                                    │ USB-CDC
        ┌──────────────────────────────────────────────────────────┼───────────┐
        │  Task ─ Interaction ─ Module ─ Device ─ Driver            │            │
        │   周期分频   整车回调   控制律   电机抽象   外设           │            │
        │                        └── 各模块把帧注册进通信中间件 ───┘            │
        └───────────────────────────────────────────────────────────────────────┘
```

核心思想：把上下位机之间**一条物理链路抽象成一个多路帧引擎** —— 一条链路同时承载多路数据，
每路数据用一个 **id** 标识，由一张统一的**注册表**管理。腿、底盘、遥控就这样共用同一条链路。

---

## 2. 分层架构

| 层 | 目录 | 职责 |
| --- | --- | --- |
| **Task** | `User/Task` | `TaskInit()` 做驱动初始化与接线；`TaskLoop()` 前台循环。周期分频（1ms/100ms/1000ms）也在这里 |
| **Interaction** | `User/Interaction` | `Chariot` 把各机构绑到一起，对 Task 只暴露几个回调 |
| **Module** | `User/Module` | 机构控制律：`Leg`（腿）/`Chassis`（底盘）/`Telecontrol`（遥控）。各自 `Init()` 里把帧注册进中间件 |
| **Device** | `User/Device` | 设备抽象：`MotorBase` 基类 + DJI 电机实现、电机实例集中处、CRSF 解析、DWT 计时 |
| **Driver** | `User/Driver` | 外设驱动：FDCAN、UART、USB-CDC、TIM、数学工具 |
| **Middleware** | `User/Middleware` | `CommunicationInterface`：线格式、打包、解包、CRC、按 id 分发 |
| **Algorithm** | `User/Algorithm` | `PID`、`CRC16/CCITT`、环形缓冲 |

依赖方向自上而下：上层只依赖下层的抽象（`Module` 只认 `MotorBase`，不认 C620 的具体实现）。

---

## 3. 通信中间件（核心）

线格式（每帧）：

```
[id:1][data:N][crc16_lo][crc16_hi]
```

- `id` 1 字节；`data` 长度**不在线上传输**，收侧按 id 从注册表查出（`rx_size`）。
- CRC16（CCITT-FALSE）覆盖 `[id][data]`。

**收发模型：**

- **发送**：控制线程（1ms）调 `Send()`，把注册表里所有 tx 帧依次打包后交给绑定的发送出口；
  一包超过对端接收缓冲就**只在帧边界上**切成多段发。
- **接收**：底层 USB 收到一包就回调 `RxRptlCallback()`，就地逐帧解包 —— 帧长按 id 查出，
  CRC 不过的那一帧直接丢掉，不污染上层。

**架构优势：**

1. **一路 id，一条链路** —— 新增一路数据只需占用一个空闲 id，共用现有链路；
   不必为每路数据单独开设备、单独写一套帧格式。
2. **模块自注册，依赖倒置** —— 每个 Module 在自己的 `Init()` 里把帧注册进中间件，
   只知道「我这一帧长这样」；中间件也不认识任何具体模块。耦合点只有一张 id 表。
3. **传输方式可替换** —— 中间件只依赖一个「发送出口」的绑定；今天走 USB-CDC，
   明天换 CAN、以太网或无线，只改绑定处那一行（`TaskInit()` 里），模块和帧格式都不动。
4. **统一的收发模型** —— 发送端每拍把注册表里所有帧统一打包发出，模块不必关心
   「这一拍该不该发我」；接收端按 id 分发，坏帧静默丢弃。
5. **帧契约两端钉死** —— 每路帧的字段布局由两端契约固定（`#pragma pack(1)` + `static_assert`），
   谁改错在**编译期**就暴露，而不是等上板错位。

> 分层落点：中间件在 `User/Middleware`；各模块自己注册帧；`Interaction` 层统一触发收发；
> `Task` 层只负责把选定的物理链路绑到中间件上——换链路只动这一处。

---

## 4. 帧 id 约定

| id | 方向 | 设备 | 载荷 | 大小 |
|----|------|------|------|------|
| 1 | 双向 | 底盘 | 下行 `ChassisTx` 4 轮目标速度 / 上行 `ChassisRx` 4 轮速度+位置 | tx 16 / rx 32 |
| 3 | 双向 | 腿 | 下行 `LegTx` 4 腿目标位置 / 上行 `LegRx` 4 腿速度+位置 | tx 16 / rx 32 |
| 5 | 上行 | 遥控 | `TelecontrolTx` 摇杆/开关/链路状态 | tx 29 |

- id 值与上位机 `robot/urdf/robot.urdf` 的 `<param>` 一一对应，改动需**两端同步**。
- 上下行结构体名按「电机视角」：`Tx` = 下发给电机的指令，`Rx` = 电机回传的反馈。
- 麦轮运动学解算在上位机的 `mecanum_drive_controller` 里做，本板底盘模块只做执行。

---

## 5. 周期调度

TIM5 配成 1ms，`tsk_config_and_callback.cpp` 里做分频：

| 周期 | 动作 |
|------|------|
| **1ms** | DWT 计时更新 · 腿控制（标定/位置环）· 底盘控制 · 遥控刷帧 · CAN 发送 · USB 上行 |
| **100ms** | 电机自身掉线检测 · 遥控接收机（CRSF）掉线检测 |
| **1000ms** | 上位机存活窗口（PC 掉线判定） |

**几个关键取舍：**

- **标定期整机不动**：`Chariot` 每拍检查 `leg_.IsCalibrated()`，校准未完成前底盘不响应上位机轮速，
  腿反馈（以及遥控帧）也先不回传，免得上位机拿到一堆假的关节位置。
- **上位机判活窗口取 1000ms**：上位机是 `sleep_for(100ms)` 节奏，窗口跟它一般大就会相位打拍，
  偶发一个窗口一个包都不落而误判掉线。
- **失效保护留在电机侧**：PC 掉线也不失控，`failsafe` 同时通过遥控帧对上层可见。

---

## 6. 硬件链路

- **CAN**：`FDCAN1` 挂 8 台 DJI 电机（C620/C610），反馈 ID `0x201~0x204` 腿关节、`0x205~0x208` 底盘轮；
  发送按电机组分组 ID（`0x1ff/0x200/0x2ff/0x1fe/...`）统一在 1ms 里发出。
- **USB-CDC**：与上位机的主链路，异步环形缓冲发送（实测两次发送间隔至少 1ms，连续调用会丢包）。
- **CRSF**：遥控接收机接 `UART7`（420000 8N1），原始字节由 `Class_CRSF` 解包成摇杆/开关/链路状态。

---

## 7. 上位机与 ros2_control

R2 上位机整体建立在 `ros2_control` 标准框架上：**硬件是插件、控制器是插件**，由
`ros2_control_node` 统一加载和调度。三块硬件接口（chassis / leg / Telecontrol）共用同一个
设备句柄与中间件实例，因此跑在同一个进程里，不会出现多进程抢设备。

**优势：**

1. **硬件与算法解耦** —— 硬件接口只搬数据（帧 ↔ state/command interface），控制器只算控制律。
   换总线只动硬件层，换算法只动控制器。
2. **不重复造轮子** —— 底盘运动学直接用官方 `mecanum_drive_controller`，只有腿的控制律是自定义插件。
3. **统一生命周期** —— 所有硬件和控制器由 `ros2_control_node` 统一
   configure/activate/deactivate，启停时序和错误处理不用自己维护。
4. **遥控器接入即标准** —— 遥控链路做成只读的 `SensorInterface`，状态既被
   `TelecontrolBroadcaster` 打包成话题 `/telecontrol/rc_state`，
   也能被其它控制器直接当 state interface 读取（「遥控驾驶」控制器同时读遥控和底盘状态）。
5. **天然可观测** —— 所有关节与遥控状态都经 state interface 暴露，
   `joint_state_broadcaster` 等标准工具开箱即用。

> 详见上位机工程 [`R2_PC`](../R2_PC) 的 README。

---

## 8. 优势汇总

| # | 设计点 | 带来的优势 |
|---|---|---|
| 1 | 多路帧引擎 + id 注册表 | 一条链路承载多路数据，新增一路只占一个 id |
| 2 | 模块自注册，不感知传输层 | 依赖倒置，耦合点只剩一张 id 表 |
| 3 | 中间件只绑一个「发送出口」 | 换传输方式（USB / CAN / 以太网）只改绑定处，模块与帧格式不动 |
| 4 | 统一收发模型 | 发送方不必关心时机，接收方按 id 分发、坏帧静默丢弃 |
| 5 | 帧契约编译期钉死 | 两端改错在编译期暴露，不在上板后才错位 |
| 6 | 3 块 ros2_control 硬件接口 | 硬件层与控制器层解耦，生命周期统一管理 |
| 7 | 官方 `mecanum_drive_controller` | 底盘运动学不重复造轮子 |
| 8 | 遥控拆成 SensorInterface + Broadcaster | 遥控既是话题、又是可被其它控制器读取的 state |
| 9 | 失效保护留在电机侧 | PC 掉线也不失控，`failsafe` 同时可见 |
| 10 | 全量状态经 state interface 暴露 | 系统天然可观测 |

---

## 9. 构建

工程用 STM32CubeMX 生成骨架，CMake + `arm-none-eabi-gcc` 构建（C++20，`-Wall -Wextra`）。

**IDE**：VS Code + CMake Tools，选 `CMakePresets.json` 里的 preset（生成器为 Ninja，
`binaryDir` 为 `build/<presetName>`）。

**命令行（headless，用于验证工程编不编得过）**：

```bash
cmake -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=<repo>/cmake/gcc-arm-none-eabi.cmake \
  -S <repo> -B <build-dir> -G Ninja
cmake --build <build-dir>
```

注意：

- `User/*` 由 `GLOB_RECURSE CONFIGURE_DEPENDS` 自动收录，但 **include 目录是手写的** ——
  新增 `User/Module/<X>` 之类的子目录时，必须在 `CMakeLists.txt` 的
  `target_include_directories` 里补一行。
- 直接 `tail` 构建输出会被 `-Wmissing-field-initializers` 警告淹没（且管道会吞掉 ninja 的非零退出码），
  建议先 `grep -E "error:|FAILED"` 过滤。

---

## 10. 目录速览

```
User/
├─ Algorithm/    alg_pid · alg_crc · alg_circular_buffer
├─ Middleware/   Communication_Interface（多路帧引擎）
├─ Driver/       drv_can · drv_uart · drv_usb · drv_tim · drv_math
├─ Device/       dvc_motor_base/dji/instances · dvc_crsf · dvc_dwt
├─ Module/       Leg/ · Chassis/ · Telecontrol/
├─ Interaction/  ita_chariot（整车组合）
└─ Task/         tsk_config_and_callback（初始化 + 周期分频）
Core/            CubeMX 生成的中断向量、HAL 初始化
USB_DEVICE/      USB CDC 设备类与描述符
```

---

> 一句话总结：下位机的**通信中间件**解决了「帧怎么组织、怎么复用一条链路、怎么与模块解耦」，
> 上位机的 **`ros2_control`** 解决了「帧怎么变成关节和话题、控制器怎么被管理」，
> 二者共同指向一个目标 —— **任何一路数据都能被任意模块以标准方式读到，且错位在编译期就被拦住。**
