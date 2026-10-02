/**
 * @file telecontrol_system.hpp
 * @brief 遥控链路：MCU 上行的遥控帧 → state interface → ROS2 话题
 *
 * 分两层，和 leg / chassis 的分工一致：
 * - TelecontrolSystem：SensorInterface，只读。绑定 CommunicationInterface 的帧 id=5，
 *   read() 把收到的遥控数据刷进 state interface。不碰话题、也没有 node。
 * - TelecontrolBroadcaster：ControllerInterface，认领上面那些 state interface，
 *   打包成 telecontrol_msgs/RcState 发出去。
 *
 * 遥控帧的线格式必须与 R2_Embedded 固件逐字节一致。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <telecontrol_msgs/msg/rc_state.hpp>

#include "Communication_Interface.hpp"

namespace Telecontrol
{

/* Exported types ------------------------------------------------------------*/

// 与下位机固件约定的遥控帧 id（chassis=1、leg=3）
constexpr uint8_t kDefaultFrameId = 5U;

#pragma pack(push, 1)
/**
 * @brief 遥控帧载荷（线格式，布局不能动）
 *
 * float 全部排在前面，避免 pack(1) 下出现非对齐的 float 成员。
 * 字段顺序/大小/对齐必须与 R2_Embedded 逐字节一致。
 */
struct TelecontrolRx
{
    float   right_x;       // 右摇杆 X，-1 ~ +1
    float   right_y;       // 右摇杆 Y，-1 ~ +1
    float   left_x;        // 左摇杆 X，-1 ~ +1
    float   left_y;        // 左摇杆 Y，-1 ~ +1
    float   s1;            // 滑块，-100 ~ +100
    uint8_t sa;            // 开关三档：0=低 1=中 2=高
    uint8_t sb;
    uint8_t sc;
    uint8_t sd;
    uint8_t se;
    uint8_t rssi;          // 链路 RSSI
    uint8_t link_quality;  // 链路质量
    int8_t  snr;           // 信噪比
    uint8_t failsafe;      // 遥控链路失效标志
};
#pragma pack(pop)

static_assert(sizeof(TelecontrolRx) == 29, "线格式必须与 R2_Embedded 固件一致");

/**
 * @brief 导出的 state interface 后缀名，顺序即 state_interfaces_ 的顺序
 *
 * 两个类都按这张表来：TelecontrolSystem 按它导出，TelecontrolBroadcaster 按它认领。
 * 表改了要一起改，URDF 里 <sensor> 的 <state_interface> 也要同步。
 */
inline constexpr const char * kStateNames[] = {
    "right_x", "right_y", "left_x", "left_y", "s1",
    "sa", "sb", "sc", "sd", "se",
    "rssi", "link_quality", "snr", "failsafe",
};
inline constexpr size_t kStateCount = sizeof(kStateNames) / sizeof(kStateNames[0]);

/**
 * @brief 遥控链路硬件层（ros2_control SensorInterface，只读）
 *
 * 只做数据搬运：CommunicationInterface 收帧 → read() 刷进 state interface。
 * state interface 是 double，所以遥控帧的整数字段在这里也按 double 存。
 */
class TelecontrolSystem : public hardware_interface::SensorInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // URDF <param name="frame_id">，需与下位机固件约定一致
    uint8_t frame_id_ = kDefaultFrameId;
    // URDF <sensor name="...">，state interface 的前缀
    std::string sensor_name_;

    // 绑定到 CommunicationInterface 的接收结构体
    TelecontrolRx rx_buffer_{};

    // 借出去的 state interface 后备存储，按 kStateNames 顺序
    double state_[kStateCount]{};

    // 指向全局实例 g_usb_communication_interface
    Middleware::CommunicationInterface * communication_interface_ = nullptr;
};

/**
 * @brief 遥控链路广播器（ros2_control ControllerInterface）
 *
 * 认领 TelecontrolSystem 导出的 rc/* state interface，每拍打包成
 * telecontrol_msgs/RcState 发到话题上。
 */
class TelecontrolBroadcaster : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // 参数：sensor_name 要与 URDF <sensor name="..."> 一致
    std::string sensor_name_ = "rc";
    std::string topic_ = "/telecontrol/rc_state";

    rclcpp::Publisher<telecontrol_msgs::msg::RcState>::SharedPtr publisher_;
};

} // namespace Telecontrol
