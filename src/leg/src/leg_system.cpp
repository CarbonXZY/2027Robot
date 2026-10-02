/**
 * @file leg_system.cpp
 * @author Carbon
 * @brief BR的腿
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "leg/leg_system.hpp"

#include <cmath>
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

/**
 * @brief 初始化：读 URDF 参数与四条腿的关节名
 *
 * @param info 硬件描述
 * @return CallbackReturn 结果
 */
CallbackReturn LegSystem::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // 四条腿一帧，id 与下位机固件约定
    leg_id_ = Utils::ReadParam<uint8_t>(info.hardware_parameters, "leg_id", 3);

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下关节名
    for (size_t i = 0; i < kLegCount; ++i)
    {
        joint_names_[i] = info.joints[i].name;
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 借出四条腿的 position/velocity 状态接口
 *
 * @return std::vector<hardware_interface::StateInterface> 状态接口
 */
std::vector<hardware_interface::StateInterface> LegSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifs;
    for (size_t i = 0; i < kLegCount; ++i)
    {
        ifs.emplace_back(joint_names_[i], HW_IF_POSITION, &now_position_[i]);
        ifs.emplace_back(joint_names_[i], HW_IF_VELOCITY, &now_velocity_[i]);
    }
    return ifs;
}

/**
 * @brief 借出四条腿的 position 命令接口
 *
 * @return std::vector<hardware_interface::CommandInterface> 命令接口
 */
std::vector<hardware_interface::CommandInterface> LegSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifs;
    for (size_t i = 0; i < kLegCount; ++i)
    {
        ifs.emplace_back(joint_names_[i], HW_IF_POSITION, &target_position_[i]);
    }
    return ifs;
}

/**
 * @brief 绑定通信帧并打开 USB-CDC
 *
 * @return CallbackReturn 结果
 */
CallbackReturn LegSystem::on_configure(const rclcpp_lifecycle::State &)
{
    // 指向全局实例，四条腿绑成一帧：下行 tx_buffer_，上行 rx_buffer_
    communication_interface_ = &Middleware::g_usb_communication_interface;

    if (!communication_interface_->Register(leg_id_, &tx_buffer_, &rx_buffer_,
                                            sizeof(tx_buffer_), sizeof(rx_buffer_)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "绑定帧 id=%u 失败", leg_id_);
        return CallbackReturn::ERROR;
    }

    if (!Task::TaskInit())
    {
        RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "打开 USB-CDC 失败");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("LegSystem"), "已绑定腿帧 id=%u（tx %zu 字节 / rx %zu 字节）",
                leg_id_, sizeof(tx_buffer_), sizeof(rx_buffer_));
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活，暂无动作
 *
 * @return CallbackReturn 结果
 */
CallbackReturn LegSystem::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 去激活，暂无动作
 *
 * @return CallbackReturn 结果
 */
CallbackReturn LegSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 把 rx_buffer_ 里的值刷进状态接口
 * 数据在接收线程到达时已由 RxRptlCallback() 分发进 rx_buffer_，这里只做读取。
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < kLegCount; ++i)
    {
        now_position_[i] = static_cast<double>(rx_buffer_.motor[i].position);
        now_velocity_[i] = static_cast<double>(rx_buffer_.motor[i].velocity);
    }

    // 调试：表格更新式打印四条腿位置（windows_name "leg"，首次调用会自动弹独立窗口）
    Utils::Debug_Log::CurrentMode() = Utils::Debug_Log::Mode::Refresh;
    Utils::Debug_Log::Print("leg", "pos = %.3f %.3f %.3f %.3f",
                            rx_buffer_.motor[0].position, rx_buffer_.motor[1].position,
                            rx_buffer_.motor[2].position, rx_buffer_.motor[3].position);

    return hardware_interface::return_type::OK;
}

/**
 * @brief 把命令接口里的目标位置写进 tx_buffer_，并推一包
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 只写 tx 结构体；Send() 不在这里调 —— 它会把所有已注册的帧打成一包，
    for (size_t i = 0; i < kLegCount; ++i)
    {
        tx_buffer_.position[i] = static_cast<float>(target_position_[i]);
    }

    Task::TaskLoop();
    return hardware_interface::return_type::OK;
}

// ---------------- LegController ----------------

/**
 * @brief 读 `joints` 参数，开出各成员的槽位，初始化状态机
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn LegController::on_init()
{
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    now_position_.assign(joint_names_.size(), 0.0);
    now_velocity_.assign(joint_names_.size(), 0.0);
    target_position_.assign(joint_names_.size(), 0.0);

    fsm_leg_.controller_ = this;
    fsm_leg_.Init(static_cast<uint8_t>(LegStatus::kCount), static_cast<uint8_t>(LegStatus::kInit));

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 认领四条腿的 position 命令接口（写入硬件侧的 target_position_）
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration LegController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_)
    {
        cfg.names.push_back(name + "/" + HW_IF_POSITION);
    }
    return cfg;
}

/**
 * @brief 认领四条腿的 position/velocity 状态接口
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration LegController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_)
    {
        cfg.names.push_back(name + "/" + HW_IF_POSITION);
        cfg.names.push_back(name + "/" + HW_IF_VELOCITY);
    }
    return cfg;
}

/**
 * @brief 每拍：读状态 → 下发目标位置 → 走一拍状态机
 *
 * @return controller_interface::return_type 结果
 */
controller_interface::return_type LegController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 下位机断链：整个控制循环停手，状态机也冻结，等下一次 alive 再继续
    if (!Task::g_mcu_alive)
    {
        return controller_interface::return_type::OK;
    }

    // INDIVIDUAL 下 state_interfaces_ 的顺序 = state_interface_configuration() 里 names 的顺序
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        now_position_[i] = state_interfaces_[2 * i].get_value();
        now_velocity_[i] = state_interfaces_[2 * i + 1].get_value();
        command_interfaces_[i].set_value(target_position_[i]);
    }

    fsm_leg_.TimStatusPeriodElapsedCallback();

    return controller_interface::return_type::OK;
}

/**
 * @brief 按当前状态把目标位置刷进 target_position_
 */
void LegController::MoveToPosition()
{
    const uint8_t status = fsm_leg_.GetNowStatusSerial();

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        target_position_[i] = approach_target_[status][i];
    }
}

/**
 * @brief 四条腿是否都到位：速度小于速度阈值，位置误差小于距离阈值
 *
 * @return true 都到位
 * @return false 还有腿没到
 */
bool LegController::IsActionFinished()
{
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        if (std::abs(now_velocity_[i]) >= velocity_approach_threshold_ ||
            std::abs(now_position_[i] - target_position_[i]) >= distance_approach_threshold_)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 状态机周期回调，节奏由 launch 的 update_rate 决定（10 Hz）
 */
void LegFsm::TimStatusPeriodElapsedCallback()
{
    status_[now_status_serial_].count_time++;

    switch (now_status_serial_)
    {
        case static_cast<uint8_t>(LegStatus::kInit):
        {
            controller_->MoveToPosition();

            if (controller_->IsActionFinished())
            {
                SetStatus(static_cast<uint8_t>(LegStatus::kLift));
            }
            break;
        }

        case static_cast<uint8_t>(LegStatus::kLift):
        {
            controller_->MoveToPosition();

            if (controller_->IsActionFinished())
            {
                SetStatus(static_cast<uint8_t>(LegStatus::kInit));
            }
            break;
        }
    }
}

} // namespace leg

PLUGINLIB_EXPORT_CLASS(leg::LegSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(leg::LegController, controller_interface::ControllerInterface)
