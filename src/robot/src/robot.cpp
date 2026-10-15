/**
 * @file robot.cpp
 * @brief 人机交互口子实现：定时仲裁上位机 / 遥控器后发给底盘
 */

#include "robot/robot.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

Robot::Robot() : Node("robot")
{
    const std::string rc_topic = declare_parameter<std::string>("rc_topic", "/telecontrol/rc_state");
    const std::string pc_twist_topic = declare_parameter<std::string>("pc_twist_topic", "/cmd_vel");
    const std::string twist_topic = declare_parameter<std::string>("twist_topic", "/mecanum_drive_controller/reference_unstamped");

    max_linear_x_ = declare_parameter<double>("max_linear_x", 1.0);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 1.0);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 2.0);
    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    pc_alive_timeout_ = declare_parameter<double>("pc_alive_timeout", 0.5);
    rc_alive_timeout_ = declare_parameter<double>("rc_alive_timeout", 0.5);
    timer_period_ms_ = declare_parameter<int>("timer_period_ms", 100);

    // 遥控广播器用 SensorDataQoS 发，这里跟着用它收
    rc_subscription_ = create_subscription<telecontrol_msgs::msg::RcState>(
        rc_topic, rclcpp::SensorDataQoS(),
        [this](telecontrol_msgs::msg::RcState::SharedPtr msg)
        { OnRcState(*msg); });

    // 上位机发什么 QoS 都可能，best_effort 收最宽容
    pc_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        pc_twist_topic, rclcpp::QoS(10).best_effort(),
        [this](geometry_msgs::msg::Twist::SharedPtr msg)
        { OnPcTwist(*msg); });

    // 控制器侧是普通订阅，用默认 reliable QoS 发最保险
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);

    // 定时仲裁：发布节奏不再依赖遥控帧，上位机活着也能独立驱动底盘
    timer_ = create_wall_timer(
        std::chrono::milliseconds(timer_period_ms_),
        [this]() { OnTimer(); });
}

double Robot::ApplyDeadzone(double value, double deadzone) const
{
    if (std::abs(value) < deadzone)
    {
        return 0.0;
    }

    const double sign = (value > 0.0) ? 1.0 : -1.0;
    return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
}

void Robot::OnTimer()
{
    ResolveControlSource();
    BuildTwist(control_mode_);
}

void Robot::ResolveControlSource()
{
    // 上位机存活：pc_alive_timeout 内收到过 Twist
    const bool pc_alive =
        (this->now() - last_pc_twist_time_) < rclcpp::Duration::from_seconds(pc_alive_timeout_);
    // 遥控健康：rc_alive_timeout 内有帧，且链路没 failsafe（否则 SA/摇杆不可信）
    const bool rc_ok =
        (this->now() - last_rc_time_) < rclcpp::Duration::from_seconds(rc_alive_timeout_)
        && !last_rc_state_.failsafe;

    // 默认上位机模式；遥控不在（读不到可信的 SA）→ 上位机，上位机也没有才失能。
    if (!rc_ok)
    {
        control_mode_ = pc_alive ? ControlMode::kPc : ControlMode::kNone;
        return;
    }

    // 遥控健康：按 SA 区分。LOW = 遥控器，HIGH = 上位机（上位机失联则失能），
    // 其余不存在档位一律失能（发零）。
    switch (static_cast<Telecontrol::RcSwitch>(last_rc_state_.sa))
    {
    case Telecontrol::RcSwitch::kLow:
        control_mode_ = ControlMode::kRc;
        return;
    case Telecontrol::RcSwitch::kHigh:
        control_mode_ = pc_alive ? ControlMode::kPc : ControlMode::kNone;
        return;
    default:
        control_mode_ = ControlMode::kNone;
        return;
    }
}

void Robot::BuildTwist(ControlMode source)
{
    geometry_msgs::msg::Twist twist;

    switch (source)
    {
    case ControlMode::kPc:
        twist = pc_twist_;  // 还没收到过就是全零
        break;
    case ControlMode::kRc:
        twist.linear.x = ApplyDeadzone(last_rc_state_.right_y, deadzone_) * max_linear_x_;
        twist.linear.y = -ApplyDeadzone(last_rc_state_.right_x, deadzone_) * max_linear_y_;
        twist.angular.z = -ApplyDeadzone(last_rc_state_.left_x, deadzone_) * max_angular_z_;
        break;
    case ControlMode::kNone:
        break;  // 失能：发零，别让底盘照最后一条指令接着跑
    }

    publisher_->publish(twist);
}

void Robot::OnRcState(const telecontrol_msgs::msg::RcState &msg)
{
    last_rc_state_ = msg;
    last_rc_time_ = this->now();
}

void Robot::OnPcTwist(const geometry_msgs::msg::Twist &msg)
{
    pc_twist_ = msg;
    last_pc_twist_time_ = this->now();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}
