# R2_PC

R2 机器人的**上位机（PC 端）控制框架**，基于 ROS 2 Humble + `ros2_control`。
整车由一块 MCU 通过 USB-CDC 与 PC 相连，PC 上跑一个 `ros2_control_node`，
把底盘、腿、遥控三条链路统一挂进 `ros2_control`，再通过标准控制器暴露成 ROS 2 话题。

- 上位机只负责**指令下发与状态回读**，所有底层闭环（电机电流环/速度环）都在下位机固件里。
- 上下位机之间只有一条 USB-CDC 链路，一根线上按帧 id 复用多个设备的收发。

## 数据流

```
        ┌───────────── PC (ROS 2 Humble) ─────────────┐
        │                                              │
  MCU ──┤ USB-CDC ─ CommunicationInterface ─ ros2_control ─ controllers ─ topics
        │  (ttyACM)   (打包/CRC/分发)         (硬件接口)        (算法/转发)
        └──────────────────────────────────────────────┘
```

中间层 `CommunicationInterface` 是整条链路的核心：上层硬件接口只绑定几个收发结构体，
它负责打包成字节流、加 CRC、交给 USB；收到字节流时解包、校验、按 id 分发回结构体。
**整个底盘/四条腿/一帧遥控各自是线上的一帧**，不是每个电机一个 id。

## 目录结构

| 包 | 作用 |
| --- | --- |
| `shared_package` | 与 ROS 解耦的公共层：通信中间件、USB-CDC 驱动、任务接线、算法、工具。**无 ROS 运行时依赖**，可单独编译测试 |
| `chassis` | 麦克纳姆底盘 `ros2_control` 硬件接口（`SystemInterface`） |
| `leg` | 四条腿硬件接口 + `LegController`（自定义控制器 + 状态机） |
| `Telecontrol` | 遥控链路：`SensorInterface`（收帧）+ Broadcaster（→ 话题） |
| `telecontrol_msgs` | 遥控消息定义 `RcState.msg` |
| `robot` | 整机 bringup：URDF、launch、人机交互仲裁节点 |

### shared_package 内部

| 子目录 | 内容 |
| --- | --- |
| `Middleware/` | `Communication_Interface`：线格式、打包、解包、CRC 校验、id 分发。详见其 [README](src/shared_package/Middleware/README.md) |
| `Driver/usb_cdc/` | USB CDC ACM 虚拟串口的轻量封装（收发线程 + 回调）。详见其 [README](src/shared_package/Driver/usb_cdc/README.md) |
| `Task/` | `TaskInit()/TaskLoop()`：把 USB-CDC 接到中间件，链路存活检测 |
| `Algorithm/` | `alg_crc`（CRC16/CCITT-FALSE）、`alg_fsm`（状态机基类）、`alg_circular_buffer` |
| `Device/` | `MotorFeedbackFrame` 等公共数据结构 |
| `Utils/` | `RetryUntil`、`ReadParam`、`Debug_Log`（独立窗口调试打印） |

## 帧 id 约定

线上每帧为 `[id:1][data:N][crc16:2]`，data 长度不在线上传输，由收侧按 id 查表。
**收发两侧对每个 id 的尺寸必须严格一致**，详细约定见中间件 README。

| id | 方向 | 设备 | 载荷 | 大小 |
|----|------|------|------|------|
| 1 | 双向 | 底盘 | `ChassisTx` 4 轮目标速度 / `ChassisRx` 4 轮速度+位置 | tx 16 / rx 32 |
| 3 | 双向 | 腿 | `LegTx` 4 腿目标位置 / `LegRx` 4 腿速度+位置 | tx 16 / rx 32 |
| 5 | 仅收 | 遥控 | `TelecontrolRx` 摇杆/开关/链路状态 | rx 29 |

id 各自可在 `robot/urdf/robot.urdf` 的 `<param>` 里改，需与下位机固件约定一致。

## 构建 / 运行

依赖 ROS 2 Humble 与 ros2_control 相关包：

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

构建并启动：

```bash
colcon build
source install/setup.bash
ros2 launch robot robot.launch
```

或直接用一键脚本（自动 source 工作空间）：

```bash
./launch.sh
```

> USB 设备默认 `/dev/ttyACM0`。普通用户需加入 `dialout` 组才能访问，见 USB-CDC README。

`robot.launch` 会拉起：`robot_state_publisher`、`ros2_control_node`（update_rate 10 Hz）、
以及 `joint_state_broadcaster` / `mecanum_drive_controller` / `leg_controller` /
`telecontrol_broadcaster` 四个 spawner，最后启动人机交互仲裁节点。

## 主要话题

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/mecanum_drive_controller/reference_unstamped` | `geometry_msgs/Twist` | 底盘速度指令（vx 前后 / vy 横移 / wz 自转） |
| `/mecanum_drive_controller/odom` | `nav_msgs/Odometry` | 底盘里程计 |
| `/telecontrol/rc_state` | `telecontrol_msgs/RcState` | 遥控器状态（摇杆、开关、链路质量、failsafe） |
| `/cmd_vel` | `geometry_msgs/Twist` | 上位机（导航等）下发的速度，经仲裁后转给底盘 |

手动下发底盘速度：

```bash
ros2 topic pub /mecanum_drive_controller/reference_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

## 人机交互仲裁（robot 包）

`robot` 节点在遥控帧到达时决定底盘听谁：

- 遥控链路失效（`failsafe`）或 SA 打到失能档 → 一律发零，底盘不动。
- SD 打到上位机档 → 转发上位机最近一帧 `/cmd_vel`。
- 否则听遥控器摇杆，经死区处理后映射到 Twist。

发布节奏跟着遥控帧走（~10 Hz）。详见 [robot.hpp](src/robot/include/robot/robot.hpp)。

## 各模块文档

- 通信中间件与线格式：[`src/shared_package/Middleware/README.md`](src/shared_package/Middleware/README.md)
- USB-CDC 驱动：[`src/shared_package/Driver/usb_cdc/README.md`](src/shared_package/Driver/usb_cdc/README.md)
- 底盘硬件接口：[`src/chassis/README.md`](src/chassis/README.md)

## License

Apache-2.0，见 [LICENSE](LICENSE)。
