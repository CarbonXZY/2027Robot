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
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <leg_msgs/action/lift.hpp>

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
    uint8_t front_completion_flag;
    uint8_t back_completion_flag;
};
#pragma pack(pop)

/**
 * @brief 腿状态机的状态
 */
enum class LegStatus
{
    kInit = 0,
    kLift_Front,
    kLift_Back,
    kLift_All,

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
    // 到位标志（0/1）：前两条腿（0/1）共享 front 标志，后两条（2/3）共享 back 标志
    std::array<double, kLegCount> completion_{};

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
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
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
    // 到位标志（double，0/1）：前两条（0/1）=front，后两条（2/3）=back
    std::vector<double> completion_;

    LegFsm fsm_leg_;

    // 抬腿动作名 / 到位超时（秒），launch 可覆盖
    std::string lift_action_name_ = "/leg/lift";
    double lift_timeout_s_ = 8.0;
    // Action 服务端：导航发 goal 切状态，到位后回 result
    rclcpp_action::Server<leg_msgs::action::Lift>::SharedPtr lift_action_server_;
    // 监控 goal 的线程：等到位后回 result（不卡 executor）
    std::thread monitor_thread_;
    std::atomic<bool> shutdown_{false};
    // 待切换的状态请求（kCount 表示无请求），goal 回调写、update() 消费
    std::atomic<uint8_t> pending_status_{static_cast<uint8_t>(LegStatus::kCount)};
    // RT 线程置位：请求的状态已到位；监控线程取走后清零
    std::atomic<bool> lift_done_{false};
    // 防重入：正在执行时拒绝新 goal
    std::atomic<bool> busy_{false};

    // 每个状态的目标位置，按 LegStatus 索引
    const float approach_target_[static_cast<size_t>(LegStatus::kCount)][kLegCount] = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // Init
        {0.2f, 0.2f, 0.0f, 0.0f}, // Lift_Front
        {0.0f, 0.0f, 0.2f, 0.2f}, // Lift_Back
        {0.2f, 0.2f, 0.2f, 0.2f}, // Lift_All
    };

    // 按当前状态把目标位置刷进 target_position_
    void MoveToPosition();

    // 当前状态要动的那组腿是否到位：直接读下位机回传的 completion 标志
    bool IsActionFinished();

    using LiftGoalHandle = rclcpp_action::ServerGoalHandle<leg_msgs::action::Lift>;

    // goal 回调：校验 + 防重入 + 写 pending，立刻返回 ACCEPT/REJECT
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const leg_msgs::action::Lift::Goal> goal);
    // cancel 回调：第一版不支持中途取消
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<LiftGoalHandle> goal_handle);
    // accepted 回调：起监控线程
    void HandleAccepted(const std::shared_ptr<LiftGoalHandle> goal_handle);
    // 监控线程体：等 lift_done_（带超时）后回 result
    void MonitorGoal(const std::shared_ptr<LiftGoalHandle> goal_handle);
};

} // namespace leg

/* Exported function declarations --------------------------------------------*/
