// chassis_system.hpp
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <Communication_Interface.hpp>
#include <motor_base.hpp>

/**
 * 麦克纳姆底盘轮子布局
 * [0][1]
 * [2][3]
 */

namespace chassis
{

constexpr size_t Wheel_Count = 4;  // 麦克纳姆底盘 4 个电机

#pragma pack(push, 1)
// 底盘发送结构体（下发指令）：4 个电机目标速度
struct Struct_Motor_Tx
{
  float velocity[Wheel_Count];
};

// 底盘接收结构体（电机回传）：4 个电机速度 + 位置
struct Struct_Motor_Rx
{
  Control_Frame::Struct_Motor_Base motor[Wheel_Count];
};
#pragma pack(pop)

// 麦克纳姆底盘硬件接口（ros2_control SystemInterface）。
// 收发交给 shared_package 的 Communication_Interface（绑定结构体后自动打包/解包），
// 本类 read/write 只读写 Tx_Buffer/Rx_Buffer 两个结构体。
class ChassisSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下来的关节名
  std::array<std::string, Wheel_Count> Joint_Names{};

  // 整个底盘绑成一帧，id 来自 URDF 的 chassis_id
  uint8_t Chassis_Id = 1;

  // ros2_control 侧（double）
  std::array<double, Wheel_Count> Target_Velocity{};
  std::array<double, Wheel_Count> Now_Velocity{};
  std::array<double, Wheel_Count> Now_Position{};

  // 绑定到 Communication_Interface 的两个结构体：发 / 收
  Struct_Motor_Tx Tx_Buffer;
  Struct_Motor_Rx Rx_Buffer;

  Control_Frame::Class_Communication_Interface * Communication_Interface = nullptr;  // 指向全局实例 USB_Communication_Interface
};

}  // namespace chassis
