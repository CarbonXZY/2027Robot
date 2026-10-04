/**
 * @file    chassis_system.hpp
 * @author  Carbon
 * @date    2026-10-04
 * @brief   麦克纳姆底盘硬件接口：4 轮速度指令下发 + 电机反馈回传，帧收发交给 CommunicationInterface
 */
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

constexpr size_t kWheelCount = 4;  // 麦克纳姆底盘 4 个电机

#pragma pack(push, 1)
// 底盘发送结构体（下发指令）：4 个电机目标速度
struct ChassisTx
{
  float velocity[kWheelCount];
};

// 底盘接收结构体（电机回传）：4 个电机速度 + 位置
struct ChassisRx
{
  Device::MotorFeedbackFrame motor[kWheelCount];
};
#pragma pack(pop)

// 麦克纳姆底盘硬件接口（ros2_control SystemInterface）。
// 收发交给 shared_package 的 CommunicationInterface（绑定结构体后自动打包/解包），
// 本类 read/write 只读写 tx_buffer_/rx_buffer_ 两个结构体。
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
  std::array<std::string, kWheelCount> joint_names_{};

  // ros2_control 侧（double）
  std::array<double, kWheelCount> target_velocity_{};
  std::array<double, kWheelCount> now_velocity_{};
  std::array<double, kWheelCount> now_position_{};

  // 绑定到 CommunicationInterface 的两个结构体：发 / 收
  ChassisTx tx_buffer_{};
  ChassisRx rx_buffer_{};

  Middleware::CommunicationInterface * communication_interface_ = nullptr;  // 指向全局实例 g_usb_communication_interface
};

}  // namespace chassis
