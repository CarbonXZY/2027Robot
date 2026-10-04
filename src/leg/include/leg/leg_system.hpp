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

/* Exported types ------------------------------------------------------------*/

namespace leg
{

/**
 * @brief 四条腿的布局，每条腿一个电机
 * [0][1]
 * [2][3]
 */
constexpr size_t kLegCount = 4;

#pragma pack(push, 1)
/**
 * @brief 腿发送结构体（下发指令）：每条腿的目标位置
 */
struct LegTx
{
    float position[kLegCount];
};

/**
 * @brief 腿接收结构体（电机回传）：每条腿速度 + 位置
 */
struct LegRx
{
    Device::MotorFeedbackFrame motor[kLegCount];
};
#pragma pack(pop)

/**
 * @brief 腿状态机的状态
 */
enum class LegStatus
{
    kInit = 0,
    kLift,

    kCount,
};

class LegController;

/**
 * @brief 腿部状态机
 */
class LegFsm : public Algorithm::Fsm
{
public:
    LegController *controller_;

    void TimStatusPeriodElapsedCallback();
};

/**
 * @brief 腿硬件接口（ros2_control SystemInterface）
 * 收发交给 shared_package 的 CommunicationInterface（绑定结构体后自动打包/解包），
 * 本类 read/write 只读写 tx_buffer_/rx_buffer_ 两个结构体。
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
    std::array<std::string, kLegCount> joint_names_{};

    // ros2_control 侧（double）
    std::array<double, kLegCount> target_position_{};
    std::array<double, kLegCount> now_position_{};
    std::array<double, kLegCount> now_velocity_{};

    // 绑定到 CommunicationInterface 的两个结构体：发 / 收
    LegTx tx_buffer_{};
    LegRx rx_buffer_{};

    Middleware::CommunicationInterface *communication_interface_ = nullptr; // 指向全局实例 g_usb_communication_interface
};

/**
 * @brief 腿部控制器（自定义 ros2_control 控制器）
 * 它认领四条腿的 position/velocity state interface，每拍把值读进自己的成员；
 * command 侧认领 position 接口，把 target_position_ 喂给硬件。
 */
class LegController : public controller_interface::ControllerInterface
{
public:
    // ControllerInterfaceBase 的四个纯虚，必须实现
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    friend class LegFsm;

private:
    // 关节名来自 `joints` 参数，顺序即 state_interfaces_ 的顺序
    std::vector<std::string> joint_names_;
    std::vector<double> now_position_;
    std::vector<double> now_velocity_;
    std::vector<double> target_position_;

    LegFsm fsm_leg_;

    // 到位判定阈值：四条腿的速度、位置误差都要小于它才算到
    const float velocity_approach_threshold_ = 0.1f;
    const float distance_approach_threshold_ = 0.1f;

    // 每个状态的目标位置，按 LegStatus 索引
    const float approach_target_[static_cast<size_t>(LegStatus::kCount)][kLegCount] = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // Init
        {1.0f, 1.0f, 1.0f, 1.0f}, // Lift
    };

    // 按当前状态把目标位置刷进 target_position_
    void MoveToPosition();

    // 四条腿是否都到位
    bool IsActionFinished();
};

} // namespace leg

/* Exported function declarations --------------------------------------------*/
