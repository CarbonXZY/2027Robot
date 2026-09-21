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
struct Struct_Motor_Tx { float velocity[kWheelCount]; };

// 收（电机回传）：4 个电机速度 + 位置
struct Struct_Motor_Rx { Control_Frame::Struct_Motor_Base motor[kWheelCount]; };
```

`Struct_Motor_Base` 定义在共享包 `shared_package/Device/motor_base.hpp`：

```cpp
struct Struct_Motor_Base {
    float velocity;  // 速度 (rad/s)
    float position;  // 位置 (rad)
};
```

`on_configure` 里把这两个结构体绑到全局 `Control_Frame::USB_Communication_Interface`（每轮一帧）：

| 轮子 | 默认帧 id | 发送(tx)            | 接收(rx)              |
|------|----------|--------------------|----------------------|
| 前左 front_left  | 1 | velocity (4 字节)   | velocity + position (8 字节) |
| 前右 front_right | 2 | velocity (4 字节)   | velocity + position (8 字节) |
| 后左 rear_left   | 3 | velocity (4 字节)   | velocity + position (8 字节) |
| 后右 rear_right  | 4 | velocity (4 字节)   | velocity + position (8 字节) |

- **write()**：只把指令速度写进 `Tx_Buffer.velocity[i]`。
- **read()**：只从 `Rx_Buffer.motor[i]` 读速度/位置。
- 真正的打包/解包由 `Communication_Interface` 的 `Send()`/`Receive()` 完成，由传输层（USB）驱动；
  本类不直接做 I/O，也不做位置积分（位置来自电机回传）。

帧 id 可在 URDF 的 `<param name="xxx_wheel_id">` 里改。

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
