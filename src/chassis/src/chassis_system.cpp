// chassis_system.cpp
#include "chassis/chassis_system.hpp"

#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
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

    // 读取各轮帧 id（默认 1..4）
    const uint8_t ids[Wheel_Count] = {
        Control_Frame::ReadParam<uint8_t>(info.hardware_parameters, "front_left_wheel_id", 1),
        Control_Frame::ReadParam<uint8_t>(info.hardware_parameters, "front_right_wheel_id", 2),
        Control_Frame::ReadParam<uint8_t>(info.hardware_parameters, "rear_left_wheel_id", 3),
        Control_Frame::ReadParam<uint8_t>(info.hardware_parameters, "rear_right_wheel_id", 4),
    };

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）直接映射到 Wheels 下标
    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      Wheels[i] = WheelConfig{info.joints[i].name, ids[i]};
    }

    // 打开 USB-CDC，并把 Control_Frame 的收发回调接上
    if (!Control_Frame::Task_Init())
    {
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ChassisSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> ifs;
    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      ifs.emplace_back(Wheels[i].joint, HW_IF_VELOCITY, &Now_Velocity[i]);
      ifs.emplace_back(Wheels[i].joint, HW_IF_POSITION, &Now_Position[i]);
    }
    return ifs;
  }

  std::vector<hardware_interface::CommandInterface> ChassisSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> ifs;
    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      ifs.emplace_back(Wheels[i].joint, HW_IF_VELOCITY, &Target_Velocity[i]);
    }
    return ifs;
  }

  CallbackReturn ChassisSystem::on_configure(const rclcpp_lifecycle::State &)
  {
    // 指向全局实例，并手动绑定两个结构体（每轮一帧）
    Communication_Interface = &Control_Frame::USB_Communication_Interface;

    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      if (!Communication_Interface->Register(Wheels[i].frame_id, &Tx_Buffer.velocity[i], &Rx_Buffer.motor[i], sizeof(float), sizeof(Control_Frame::Struct_Motor_Base)))
      {
        RCLCPP_ERROR(rclcpp::get_logger("ChassisSystem"), "绑定帧 id=%u 失败", Wheels[i].frame_id);
        return CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("ChassisSystem"), "已绑定 4 个轮子帧（tx/rx 结构体）");
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
    // 数据在接收线程到达时已由 Rx_RptlCallback() 分发进 Rx_Buffer，这里只做读取
    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      Now_Velocity[i] = static_cast<double>(Rx_Buffer.motor[i].velocity);
      Now_Position[i] = static_cast<double>(Rx_Buffer.motor[i].position);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ChassisSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 先写 tx 结构体，再 Send 发出去
    for (size_t i = 0; i < Wheel_Count; ++i)
    {
      Tx_Buffer.velocity[i] = static_cast<float>(Target_Velocity[i]);
    }
    Communication_Interface->Send();
    return hardware_interface::return_type::OK;
  }

} // namespace chassis

PLUGINLIB_EXPORT_CLASS(chassis::ChassisSystem, hardware_interface::SystemInterface)
