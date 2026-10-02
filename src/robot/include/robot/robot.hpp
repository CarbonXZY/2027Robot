/**
 * @file robot.hpp
 * @brief 人机交互口子：遥控 / 上位机 Twist 二选一，仲裁后发给底盘
 *
 * 仲裁规则：遥控帧的 SD 开关打到 pc_switch_value（默认 HIGH）就听上位机下发的
 * Twist，否则听遥控器摇杆。遥控链路失效（failsafe）、或 SA 打到 sa_disable_value
 * （默认 HIGH）时一律发零，底盘不动。
 * 摇杆映射：left_x 自转，right_x 横移，right_y 前后。
 *
 * 发布节奏跟着遥控帧走（广播器 ~10 Hz，与 ros2_control update_rate 同档），
 * 上位机的 Twist 只做采样。
 */

#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <telecontrol_msgs/msg/rc_state.hpp>

/**
 * @brief 遥控器三档开关档位，取值与 telecontrol_msgs/RcState.msg 的常量一致
 */
enum class RcSwitch : uint8_t
{
    kLow = 0,
    kMiddle = 1,
    kHigh = 2,
};

class Robot : public rclcpp::Node
{
public:
    Robot();

private:
    /**
     * @brief 摇杆死区：进死区归零，出死区重新归一化，避免边缘突跳
     */
    double ApplyDeadzone(double value, double deadzone) const;

    /**
     * @brief 遥控帧到达：按 SD / SA 位置选信号源，打包成 Twist 发出去
     */
    void OnRcState(const telecontrol_msgs::msg::RcState &msg);

    /**
     * @brief 上位机 Twist 到达：只记下最近一帧，等遥控帧那一拍再决定用不用
     */
    void OnPcTwist(const geometry_msgs::msg::Twist &msg);

    double max_linear_x_ = 1.0;
    double max_linear_y_ = 1.0;
    double max_angular_z_ = 2.0;
    double deadzone_ = 0.05;

    // SD 打到这一档就把底盘交给上位机
    RcSwitch pc_switch_value_ = RcSwitch::kHigh;
    // SA 打到这一档就失能底盘（发零）
    RcSwitch sa_disable_value_ = RcSwitch::kHigh;

    // 上位机最近一帧 Twist；还没收到过就是全零，正好当安全兜底
    geometry_msgs::msg::Twist pc_twist_{};

    rclcpp::Subscription<telecontrol_msgs::msg::RcState>::SharedPtr rc_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pc_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
