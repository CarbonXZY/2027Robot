# chassis

麦克纳姆底盘的 `ros2_control` 硬件接口包。硬件接口通过 `shared_package` 的全局
`Communication_Interface`（`Control_Frame::USB_Communication_Interface`）与底层（MCU/USB）收发数据。

## 结构

```
chassis/
├── chassis_plugin.xml                 # pluginlib 插件描述
├── include/chassis/chassis_system.hpp  # 硬件接口 + 收/发两个结构体
├── src/chassis_system.cpp             # SystemInterface 实现
├── urdf/chassis.urdf                  # 示例 URDF（4 轮 + ros2_control 标签）
├── config/chassis_controller.yaml     # mecanum_drive_controller 配置
├── launch/chassis.launch              # XML launch
└── README.md
```

## 依赖安装（Humble）

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## 数据收发

底盘在 `chassis_system.hpp` 里声明了两个结构体：

```cpp
// 发（下发指令）：4 个电机目标速度
struct Struct_Motor_Tx { float velocity[Wheel_Count]; };

// 收（电机回传）：4 个电机速度 + 位置
struct Struct_Motor_Rx { Control_Frame::Struct_Motor_Base motor[Wheel_Count]; };
```

`Struct_Motor_Base` 定义在共享包 `shared_package/Device/motor_base.hpp`：

```cpp
struct Struct_Motor_Base {
    float velocity;  // 速度 (rad/s)
    float position;  // 位置 (rad)
};
```

`on_configure` 里把这两个结构体绑到全局 `Control_Frame::USB_Communication_Interface`。
**整个底盘是线上的一帧**——一个结构体对一个 id，不是每个电机一个 id：

| 方向 | 帧 id | 载荷 |
|------|-------|------|
| 下行 tx | `chassis_id`（默认 1） | `Struct_Motor_Tx`，16 字节 |
| 上行 rx | 同上 | `Struct_Motor_Rx`，32 字节 |

线上每帧是 `[id][data][crc16]`，所以实际下发 19 字节、回传 35 字节，
都在单包上限 64 字节以内，各一个 USB 包。轮子之间的顺序就是结构体里数组的顺序，
不再由 id 区分。

- **write()**：只把指令速度写进 `Tx_Buffer.velocity[i]`，随后 `Send()` 打成包发出去。
- **read()**：只从 `Rx_Buffer.motor[i]` 读速度/位置——数据在接收线程到达时已由中间件
  分发进 `Rx_Buffer`，`read()` 本身不做 I/O。
- 本类不做位置积分（位置来自电机回传）。

两个结构体都带 `#pragma pack(push, 1)`。线格式没有长度字段，收发两侧的尺寸必须严格
一致，一旦有填充字节就会整体错位。

`chassis_id` 可在 URDF 的 `<param name="chassis_id">` 里改，需与下位机固件约定一致。

## 控制器

使用 `ros2_controllers` 自带的 `mecanum_drive_controller`（Humble 已提供），
它订阅 `cmd_vel`（Twist/TwistStamped），用麦克纳姆逆解算出 4 轮目标速度。

- 逆解参数 `sum_of_robot_center_projection_on_X_Y_axis = lx + ly`，
  `lx/ly` 是轮心到车体中心在 X/Y 轴的投影距离（半车长/半车宽），
  必须与 URDF 的轮子布置一致。
- 话题：`/mecanum_drive_controller/reference`（TwistStamped，`use_stamped_vel: true`）
  或 `/mecanum_drive_controller/reference_unstamped`（Twist，`use_stamped_vel: false`）。

## 构建 / 运行

```bash
cd ~/Program_Projects/Control_Frame
colcon build
source install/setup.bash
ros2 launch chassis chassis.launch

# 下发指令（Twist：vx 前后，vy 横移，wz 自转）
ros2 topic pub /mecanum_drive_controller/reference_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```
