/**
 * @file telecontrol_system.cpp
 * @brief 遥控链路实现：SensorInterface（收帧→state interface）+ ControllerInterface（→话题）
 */

#include "telecontrol/telecontrol_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "tsk_config_and_callback.hpp"
#include "utils.hpp"

namespace Telecontrol
{

using hardware_interface::CallbackReturn;

/* ==================== TelecontrolSystem ==================== */

/**
 * @brief 读 URDF 参数：帧 id 与 sensor 名
 */
CallbackReturn TelecontrolSystem::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    frame_id_ = Utils::ReadParam<uint8_t>(info.hardware_parameters, "frame_id", kDefaultFrameId);

    if (info.sensors.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("TelecontrolSystem"), "URDF 里缺 <sensor> 声明");
        return CallbackReturn::ERROR;
    }
    sensor_name_ = info.sensors[0].name;

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 按 kStateNames 的顺序导出 state interface（接口全名 = sensor 名/后缀）
 */
std::vector<hardware_interface::StateInterface> TelecontrolSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < kStateCount; ++i)
    {
        interfaces.emplace_back(sensor_name_, kStateNames[i], &state_[i]);
    }
    return interfaces;
}

/**
 * @brief 绑定遥控帧并打开 USB-CDC
 */
CallbackReturn TelecontrolSystem::on_configure(const rclcpp_lifecycle::State &)
{
    // 指向全局实例；只收不发，所以 tx 传 nullptr、tx_size 传 0
    communication_interface_ = &Middleware::g_usb_communication_interface;

    if (!communication_interface_->Register(frame_id_, nullptr, &rx_buffer_, 0, sizeof(rx_buffer_)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("TelecontrolSystem"), "绑定遥控帧 id=%u 失败", frame_id_);
        return CallbackReturn::ERROR;
    }

    if (!Task::TaskInit())
    {
        RCLCPP_ERROR(rclcpp::get_logger("TelecontrolSystem"), "打开 USB-CDC 失败");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("TelecontrolSystem"), "已绑定遥控帧 id=%u（rx %zu 字节）",
                frame_id_, sizeof(rx_buffer_));
    return CallbackReturn::SUCCESS;
}

CallbackReturn TelecontrolSystem::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn TelecontrolSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 把 rx_buffer_ 刷进 state interface
 * 数据在接收线程到达时已由 RxRptlCallback() 分发进 rx_buffer_，这里只做搬运。
 */
hardware_interface::return_type TelecontrolSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    state_[0]  = static_cast<double>(rx_buffer_.right_x);
    state_[1]  = static_cast<double>(rx_buffer_.right_y);
    state_[2]  = static_cast<double>(rx_buffer_.left_x);
    state_[3]  = static_cast<double>(rx_buffer_.left_y);
    state_[4]  = static_cast<double>(rx_buffer_.s1);
    state_[5]  = static_cast<double>(rx_buffer_.sa);
    state_[6]  = static_cast<double>(rx_buffer_.sb);
    state_[7]  = static_cast<double>(rx_buffer_.sc);
    state_[8]  = static_cast<double>(rx_buffer_.sd);
    state_[9]  = static_cast<double>(rx_buffer_.se);
    state_[10] = static_cast<double>(rx_buffer_.rssi);
    state_[11] = static_cast<double>(rx_buffer_.link_quality);
    state_[12] = static_cast<double>(rx_buffer_.snr);
    state_[13] = static_cast<double>(rx_buffer_.failsafe);

    return hardware_interface::return_type::OK;
}

/* ==================== TelecontrolBroadcaster ==================== */

/**
 * @brief 声明参数
 */
controller_interface::CallbackReturn TelecontrolBroadcaster::on_init()
{
    sensor_name_ = auto_declare<std::string>("sensor_name", "rc");
    topic_ = auto_declare<std::string>("topic", "/telecontrol/rc_state");

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 只发布，不认领任何命令接口
 */
controller_interface::InterfaceConfiguration TelecontrolBroadcaster::command_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::NONE, {}};
}

/**
 * @brief 认领 TelecontrolSystem 导出的 sensor 的 state interface
 */
controller_interface::InterfaceConfiguration TelecontrolBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (size_t i = 0; i < kStateCount; ++i)
    {
        cfg.names.push_back(sensor_name_ + "/" + kStateNames[i]);
    }
    return cfg;
}

controller_interface::CallbackReturn TelecontrolBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
    publisher_ = get_node()->create_publisher<telecontrol_msgs::msg::RcState>(
        topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_node()->get_logger(), "遥控话题 %s（sensor %s）", topic_.c_str(), sensor_name_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TelecontrolBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TelecontrolBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 每拍把 state interface 打包成一帧发出去
 */
controller_interface::return_type TelecontrolBroadcaster::update(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 下位机断链：不发，话题静默，等下一次 alive
    if (!Task::g_mcu_alive)
    {
        return controller_interface::return_type::OK;
    }

    // INDIVIDUAL 下 state_interfaces_ 的顺序 = state_interface_configuration() 里 names 的顺序
    telecontrol_msgs::msg::RcState msg;
    msg.header.stamp = get_node()->now();

    msg.right_x = static_cast<float>(state_interfaces_[0].get_value());
    msg.right_y = static_cast<float>(state_interfaces_[1].get_value());
    msg.left_x = static_cast<float>(state_interfaces_[2].get_value());
    msg.left_y = static_cast<float>(state_interfaces_[3].get_value());
    msg.s1 = static_cast<float>(state_interfaces_[4].get_value());

    msg.sa = static_cast<uint8_t>(state_interfaces_[5].get_value());
    msg.sb = static_cast<uint8_t>(state_interfaces_[6].get_value());
    msg.sc = static_cast<uint8_t>(state_interfaces_[7].get_value());
    msg.sd = static_cast<uint8_t>(state_interfaces_[8].get_value());
    msg.se = static_cast<uint8_t>(state_interfaces_[9].get_value());

    msg.rssi = static_cast<uint8_t>(state_interfaces_[10].get_value());
    msg.link_quality = static_cast<uint8_t>(state_interfaces_[11].get_value());
    msg.snr = static_cast<int8_t>(state_interfaces_[12].get_value());
    msg.failsafe = state_interfaces_[13].get_value() > 0.5;

    publisher_->publish(msg);
    return controller_interface::return_type::OK;
}

} // namespace Telecontrol

PLUGINLIB_EXPORT_CLASS(Telecontrol::TelecontrolSystem, hardware_interface::SensorInterface)
PLUGINLIB_EXPORT_CLASS(Telecontrol::TelecontrolBroadcaster, controller_interface::ControllerInterface)
