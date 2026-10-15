/**
 * @file robot.hpp
 * @brief 人机交互口子：按 SA 两位开关二选一（遥控器 / 上位机），定时打包成 Twist 发给底盘
 *
 * 每个周期 OnTimer：先 ResolveControlSource() 判定这一拍听谁（kNone / kPc / kRc），
 * 再 BuildTwist() 按信号源打包成 Twist 发布。
 * 默认上位机模式（遥控不在、读不到 SA 时）。遥控健康（rc_alive_timeout 内有帧且无
 * failsafe）时按 SA 区分：LOW = 遥控器，HIGH = 上位机（上位机失联则发零），其余
 * 不存在档位一律失能（发零）。
 * 摇杆映射：left_x 自转，right_x 横移，right_y 前后。
 */

#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <telecontrol/telecontrol_system.hpp>
#include <telecontrol_msgs/msg/rc_state.hpp>

/**
 * @brief 底盘信号源：失能 / 上位机 / 遥控器
 */
enum class ControlMode : uint8_t
{
    kNone = 0,  // 失能，发零
    kPc = 1,    // 上位机 Twist
    kRc = 2,    // 遥控器摇杆
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
     * @brief 定时仲裁：判定信号源 → 打包 Twist → 发布
     */
    void OnTimer();

    /**
     * @brief 判定这一拍听谁，结果写进 control_mode_：遥控不在默认上位机；遥控健康时 SA=LOW 遥控、HIGH 上位机、其余失能
     */
    void ResolveControlSource();

    /**
     * @brief 按信号源打包对应 Twist 并发布
     */
    void BuildTwist(ControlMode source);

    /**
     * @brief 遥控帧到达：只记下最近一帧
     */
    void OnRcState(const telecontrol_msgs::msg::RcState &msg);

    /**
     * @brief 上位机 Twist 到达：记下最近一帧与到达时刻
     */
    void OnPcTwist(const geometry_msgs::msg::Twist &msg);

    double max_linear_x_ = 1.0;
    double max_linear_y_ = 1.0;
    double max_angular_z_ = 2.0;
    double deadzone_ = 0.05;

    // 上位机超过这个时间没发 Twist 就算失联（上位机模式下发零保护）
    double pc_alive_timeout_ = 0.5;
    // 遥控超过这个时间没发帧就算不在，读不到 SA，默认上位机模式
    double rc_alive_timeout_ = 0.5;
    // 仲裁周期（毫秒）
    int timer_period_ms_ = 100;

    // 这一拍听谁，由 ResolveControlSource() 写入，BuildTwist() 读取
    ControlMode control_mode_ = ControlMode::kNone;

    // 遥控最近一帧；默认全零，正好当安全兜底
    telecontrol_msgs::msg::RcState last_rc_state_{};
    // 上位机最近一帧 Twist；还没收到过就是全零
    geometry_msgs::msg::Twist pc_twist_{};
    // 上位机最近一次 Twist 到达时刻（默认 0，表示从未收到；显式 ROS 时钟，和 now() 同源）
    rclcpp::Time last_pc_twist_time_{0, 0, RCL_ROS_TIME};
    // 遥控最近一帧到达时刻（默认 0，表示从未收到；显式 ROS 时钟，和 now() 同源）
    rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<telecontrol_msgs::msg::RcState>::SharedPtr rc_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pc_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
