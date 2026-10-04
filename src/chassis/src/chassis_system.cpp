/**
 * @file    chassis_system.cpp
 * @author  Carbon
 * @date    2026-10-04
 * @brief   麦克纳姆底盘硬件接口实现（ros2_control SystemInterface）
 */
#include "chassis/chassis_system.hpp"

#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mid_config.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsk_config_and_callback.hpp>

namespace chassis
{

  using hardware_interface::CallbackReturn;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  CallbackReturn ChassisSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下关节名
    for (size_t i = 0; i < kWheelCount; ++i)
    {
      joint_names_[i] = info.joints[i].name;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ChassisSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> ifs;
    for (size_t i = 0; i < kWheelCount; ++i)
    {
      ifs.emplace_back(joint_names_[i], HW_IF_VELOCITY, &now_velocity_[i]);
      ifs.emplace_back(joint_names_[i], HW_IF_POSITION, &now_position_[i]);
    }
    return ifs;
  }

  std::vector<hardware_interface::CommandInterface> ChassisSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> ifs;
    for (size_t i = 0; i < kWheelCount; ++i)
    {
      ifs.emplace_back(joint_names_[i], HW_IF_VELOCITY, &target_velocity_[i]);
    }
    return ifs;
  }

  CallbackReturn ChassisSystem::on_configure(const rclcpp_lifecycle::State &)
  {
    // 指向全局实例，整个底盘绑成一帧：下行 tx_buffer_，上行 rx_buffer_
    communication_interface_ = &Middleware::g_usb_communication_interface;

    if (!communication_interface_->Register(
            static_cast<uint8_t>(Middleware::CommFrameId::kChassis),
            &tx_buffer_, &rx_buffer_, sizeof(tx_buffer_), sizeof(rx_buffer_)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ChassisSystem"), "绑定帧 id=%u 失败",
                   static_cast<uint8_t>(Middleware::CommFrameId::kChassis));
      return CallbackReturn::ERROR;
    }

    if (!Task::TaskInit())
    {
      RCLCPP_ERROR(rclcpp::get_logger("ChassisSystem"), "打开 USB-CDC 失败");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ChassisSystem"), "已绑定底盘帧 id=%u（tx %zu 字节 / rx %zu 字节）",
                static_cast<uint8_t>(Middleware::CommFrameId::kChassis), sizeof(tx_buffer_), sizeof(rx_buffer_));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ChassisSystem::on_activate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ChassisSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ChassisSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 数据在接收线程到达时已由 RxRptlCallback() 分发进 rx_buffer_，这里只做读取
    for (size_t i = 0; i < kWheelCount; ++i)
    {
      now_velocity_[i] = static_cast<double>(rx_buffer_.motor[i].velocity);
      now_position_[i] = static_cast<double>(rx_buffer_.motor[i].position);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ChassisSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < kWheelCount; ++i)
    {
      tx_buffer_.velocity[i] = static_cast<float>(target_velocity_[i]);
    }
    return hardware_interface::return_type::OK;
  }

} // namespace chassis

PLUGINLIB_EXPORT_CLASS(chassis::ChassisSystem, hardware_interface::SystemInterface)
