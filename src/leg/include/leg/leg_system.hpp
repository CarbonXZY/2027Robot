/**
 * @file leg_system.hpp
 * @author Carbon
 * @brief BR的腿
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026 Robopioneer
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "alg_fsm.h"
#include "motor_base.hpp"
#include <Communication_Interface.hpp>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

namespace leg
{

/**
 * @brief 四条腿的布局，每条腿一个电机
 * [0][1]
 * [2][3]
 */
constexpr size_t Leg_Count = 4;

#pragma pack(push, 1)
/**
 * @brief 腿发送结构体（下发指令）：每条腿的目标位置
 */
struct Struct_Motor_Tx
{
    float position[Leg_Count];
};

/**
 * @brief 腿接收结构体（电机回传）：每条腿速度 + 位置
 */
struct Struct_Motor_Rx
{
    Device::Struct_Motor_Base motor[Leg_Count];
};
#pragma pack(pop)

/**
 * @brief 腿状态机的状态
 */
enum Enum_Leg_Status
{
    Leg_Status_Init = 0,
    Leg_Status_Lift,

    MAX_LEG_STATUS
};

class LegController;

/**
 * @brief 腿部状态机
 */
class LegFSM : public Class_FSM
{
public:
    LegController *Controller;

    void Leg_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief 腿硬件接口（ros2_control SystemInterface）
 * 收发交给 shared_package 的 Communication_Interface（绑定结构体后自动打包/解包），
 * 本类 read/write 只读写 Tx_Buffer/Rx_Buffer 两个结构体。
 */
class LegSystem : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下来的关节名
    std::array<std::string, Leg_Count> Joint_Names{};

    // 四条腿绑成一帧，id 来自 URDF 的 leg_id
    uint8_t Leg_Id = 3;

    // ros2_control 侧（double）
    std::array<double, Leg_Count> Target_Position{};
    std::array<double, Leg_Count> Now_Position{};
    std::array<double, Leg_Count> Now_Velocity{};

    // 绑定到 Communication_Interface 的两个结构体：发 / 收
    Struct_Motor_Tx Tx_Buffer{};
    Struct_Motor_Rx Rx_Buffer{};

    Middleware::Class_Communication_Interface *Communication_Interface = nullptr; // 指向全局实例 USB_Communication_Interface
};

/**
 * @brief 腿部控制器（自定义 ros2_control 控制器）
 * 它认领四条腿的 position/velocity state interface，每拍把值读进自己的成员；
 * command 侧认领 position 接口，把 Target_Position 喂给硬件。
 */
class LegController : public controller_interface::ControllerInterface
{
public:
    // ControllerInterfaceBase 的四个纯虚，必须实现
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    friend class LegFSM;

private:
    // 关节名来自 `joints` 参数，顺序即 state_interfaces_ 的顺序
    std::vector<std::string> Joint_Names;
    std::vector<double> Now_Position;
    std::vector<double> Now_Velocity;
    std::vector<double> Target_Position;

    LegFSM FSM_Leg;

    // 到位判定阈值：四条腿的速度、位置误差都要小于它才算到
    const float Velocity_Approach_Threthold = 0.1f;
    const float Distance_Approach_Threthold = 0.1f;

    // 每个状态的目标位置，按 Enum_Leg_Status 索引
    const float Approach_Target[MAX_LEG_STATUS][Leg_Count] = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // Init
        {1.0f, 1.0f, 1.0f, 1.0f}, // Lift
    };

    // 按当前状态把目标位置刷进 Target_Position
    void Move_To_Position();

    // 四条腿是否都到位
    bool Is_Action_Finished();
};

} // namespace leg

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
