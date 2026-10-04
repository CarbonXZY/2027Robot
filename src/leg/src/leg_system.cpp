// leg_system.cpp
#include "leg/leg_system.hpp"

#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <motor_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsk_config_and_callback.hpp>
#include <utils.hpp>

namespace leg
{

  using hardware_interface::CallbackReturn;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  CallbackReturn LegSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // 四条腿一帧，id 与下位机固件约定
    Leg_Id = Utils::ReadParam<uint8_t>(info.hardware_parameters, "leg_id", 3);

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下关节名
    for (size_t i = 0; i < Leg_Count; ++i)
    {
      Joint_Names[i] = info.joints[i].name;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> LegSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> ifs;
    for (size_t i = 0; i < Leg_Count; ++i)
    {
      ifs.emplace_back(Joint_Names[i], HW_IF_POSITION, &Now_Position[i]);
      ifs.emplace_back(Joint_Names[i], HW_IF_VELOCITY, &Now_Velocity[i]);
    }
    return ifs;
  }

  std::vector<hardware_interface::CommandInterface> LegSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> ifs;
    for (size_t i = 0; i < Leg_Count; ++i)
    {
      ifs.emplace_back(Joint_Names[i], HW_IF_POSITION, &Target_Position[i]);
    }
    return ifs;
  }

  CallbackReturn LegSystem::on_configure(const rclcpp_lifecycle::State &)
  {
    // 指向全局实例，四条腿绑成一帧：下行 Tx_Buffer，上行 Rx_Buffer
    Communication_Interface = &Middleware::USB_Communication_Interface;

    if (!Communication_Interface->Register(Leg_Id, &Tx_Buffer, &Rx_Buffer,
                                           sizeof(Tx_Buffer), sizeof(Rx_Buffer)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "绑定帧 id=%u 失败", Leg_Id);
      return CallbackReturn::ERROR;
    }

    if (!Task::Task_Init())
    {
      RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "打开 USB-CDC 失败");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("LegSystem"), "已绑定腿帧 id=%u（tx %zu 字节 / rx %zu 字节）",
                Leg_Id, sizeof(Tx_Buffer), sizeof(Rx_Buffer));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn LegSystem::on_activate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn LegSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type LegSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 数据在接收线程到达时已由 Rx_RptlCallback() 分发进 Rx_Buffer，这里只做读取
    for (size_t i = 0; i < Leg_Count; ++i)
    {
      Now_Position[i] = static_cast<double>(Rx_Buffer.motor[i].position);
      Now_Velocity[i] = static_cast<double>(Rx_Buffer.motor[i].velocity);
    }

    // 调试：表格更新式打印四条腿位置（windows_name "leg"，首次调用会自动弹独立窗口）
    Utils::Debug_Log::SetMode("leg", Utils::Debug_Log::Mode::Refresh);
    Utils::Debug_Log::Print("leg", "pos = %.3f %.3f %.3f %.3f",
        Rx_Buffer.motor[0].position, Rx_Buffer.motor[1].position,
        Rx_Buffer.motor[2].position, Rx_Buffer.motor[3].position);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type LegSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 测试：每条腿位置恒为 0 下发，供下位机验证下行链路是否收到
    for (size_t i = 0; i < Leg_Count; ++i)
    {
      Tx_Buffer.position[i] = 0.0f;
    }

    Task::Task_Loop();
    return hardware_interface::return_type::OK;
  }

} // namespace leg

PLUGINLIB_EXPORT_CLASS(leg::LegSystem, hardware_interface::SystemInterface)
