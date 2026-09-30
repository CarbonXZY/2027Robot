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

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

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
    Leg_Id = Utils::ReadParam<uint8_t>(info.hardware_parameters, "leg_id", 3);

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下关节名
    for (size_t i = 0; i < Leg_Count; ++i)
    {
        Joint_Names[i] = info.joints[i].name;
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
    for (size_t i = 0; i < Leg_Count; ++i)
    {
        ifs.emplace_back(Joint_Names[i], HW_IF_POSITION, &Now_Position[i]);
        ifs.emplace_back(Joint_Names[i], HW_IF_VELOCITY, &Now_Velocity[i]);
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
    for (size_t i = 0; i < Leg_Count; ++i)
    {
        ifs.emplace_back(Joint_Names[i], HW_IF_POSITION, &Target_Position[i]);
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
 * @brief 把 Rx_Buffer 里的值刷进状态接口
 * 数据在接收线程到达时已由 Rx_RptlCallback() 分发进 Rx_Buffer，这里只做读取。
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < Leg_Count; ++i)
    {
        Now_Position[i] = static_cast<double>(Rx_Buffer.motor[i].position);
        Now_Velocity[i] = static_cast<double>(Rx_Buffer.motor[i].velocity);
    }

    // 调试：表格更新式打印四条腿位置（windows_name "leg"，首次调用会自动弹独立窗口）
    Utils::Debug_Log::CurrentMode() = Utils::Debug_Log::Mode::Refresh;
    Utils::Debug_Log::Print("leg", "pos = %.3f %.3f %.3f %.3f",
                            Rx_Buffer.motor[0].position, Rx_Buffer.motor[1].position,
                            Rx_Buffer.motor[2].position, Rx_Buffer.motor[3].position);

    return hardware_interface::return_type::OK;
}

/**
 * @brief 把命令接口里的目标位置写进 Tx_Buffer，并推一包
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 只写 tx 结构体；Send() 不在这里调 —— 它会把所有已注册的帧打成一包，
    for (size_t i = 0; i < Leg_Count; ++i)
    {
        Tx_Buffer.position[i] = static_cast<float>(Target_Position[i]);
    }

    Task::Task_Loop();
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
    Joint_Names = auto_declare<std::vector<std::string>>("joints", {});
    Now_Position.assign(Joint_Names.size(), 0.0);
    Now_Velocity.assign(Joint_Names.size(), 0.0);
    Target_Position.assign(Joint_Names.size(), 0.0);

    FSM_Leg.Controller = this;
    FSM_Leg.Init(MAX_LEG_STATUS, Leg_Status_Init);

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 认领四条腿的 position 命令接口（写入硬件侧的 Target_Position）
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration LegController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : Joint_Names)
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
    for (const auto &name : Joint_Names)
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
    if (!Task::MCU_Alive)
    {
        return controller_interface::return_type::OK;
    }

    // INDIVIDUAL 下 state_interfaces_ 的顺序 = state_interface_configuration() 里 names 的顺序
    for (size_t i = 0; i < Joint_Names.size(); ++i)
    {
        Now_Position[i] = state_interfaces_[2 * i].get_value();
        Now_Velocity[i] = state_interfaces_[2 * i + 1].get_value();
        command_interfaces_[i].set_value(Target_Position[i]);
    }

    FSM_Leg.Leg_TIM_Status_PeriodElapsedCallback();

    return controller_interface::return_type::OK;
}

/**
 * @brief 按当前状态把目标位置刷进 Target_Position
 */
void LegController::Move_To_Position()
{
    uint8_t status = FSM_Leg.Get_Now_Status_Serial();

    for (size_t i = 0; i < Joint_Names.size(); ++i)
    {
        Target_Position[i] = Approach_Target[status][i];
    }
}

/**
 * @brief 四条腿是否都到位：速度小于速度阈值，位置误差小于距离阈值
 *
 * @return true 都到位
 * @return false 还有腿没到
 */
bool LegController::Is_Action_Finished()
{
    for (size_t i = 0; i < Joint_Names.size(); ++i)
    {
        if (std::abs(Now_Velocity[i]) >= Velocity_Approach_Threthold ||
            std::abs(Now_Position[i] - Target_Position[i]) >= Distance_Approach_Threthold)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 状态机周期回调，节奏由 launch 的 update_rate 决定（10 Hz）
 */
void LegFSM::Leg_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
        case Leg_Status_Init:
        {
            Controller->Move_To_Position();

            if (Controller->Is_Action_Finished())
            {
                Set_Status(Leg_Status_Lift);
            }
            break;
        }

        case Leg_Status_Lift:
        {
            Controller->Move_To_Position();

            if (Controller->Is_Action_Finished())
            {
                Set_Status(Leg_Status_Init);
            }
            break;
        }
    }
}

} // namespace leg

PLUGINLIB_EXPORT_CLASS(leg::LegSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(leg::LegController, controller_interface::ControllerInterface)
