/**
 * @file robot.cpp
 * @brief 人机交互口子实现：遥控 / 上位机 Twist 仲裁后发给底盘
 */

#include "robot/robot.hpp"

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
    pc_switch_value_ = static_cast<RcSwitch>(declare_parameter<int>("pc_switch_value", 2));
    sa_disable_value_ = static_cast<RcSwitch>(declare_parameter<int>("sa_disable_value", 2));

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
}

double Robot::ApplyDeadzone(double value, double deadzone) const
{
    if (std::abs(value) < deadzone)
    {
        return 0.0;
    }

    const double sign = (value > 0.0) ? 1.0 : -1.0;
    return sign * (value - deadzone) / (1.0 - deadzone);
}

void Robot::OnRcState(const telecontrol_msgs::msg::RcState &msg)
{
    // 遥控链路失效、或 SA 打到失能档：发零，别让底盘照最后一条指令接着跑
    geometry_msgs::msg::Twist twist;

    if (!msg.failsafe && msg.sa != static_cast<uint8_t>(sa_disable_value_))
    {
        if (msg.sd == static_cast<uint8_t>(pc_switch_value_))
        {
            // SD 打到上位机档：放上位机最近一帧，没收到过就是零
            twist = pc_twist_;
        }
        else
        {
            twist.linear.x = ApplyDeadzone(msg.right_y, deadzone_) * max_linear_x_;
            twist.linear.y = ApplyDeadzone(msg.right_x, deadzone_) * max_linear_y_;
            twist.angular.z = ApplyDeadzone(msg.left_x, deadzone_) * max_angular_z_;
        }
    }

    publisher_->publish(twist);
}

void Robot::OnPcTwist(const geometry_msgs::msg::Twist &msg)
{
    pc_twist_ = msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}
