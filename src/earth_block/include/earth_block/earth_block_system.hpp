/**
 * @file    earth_block_system.hpp
 * @author  Carbon
 * @date    2026-11-19
 * @brief   地块机构：1 路 C610 导轨电机 + 2 路气泵
 *
 * 对应下位机固件 R2_Embedded 的 id=2 帧，线格式逐字节一致：
 *   下行 PC -> MCU：EarthBlockTx  6B  = float motor_position_mm + uint8 pump_target[2]
 *   上行 MCU -> PC：EarthBlockRx 10B  = MotorFeedbackFrame(velocity, position) + uint8 pump_status[2]
 *
 * 分两层，和 leg 的分工一致：
 * - EarthBlockSystem：SystemInterface，只做通信帧与 state/command interface 的搬运。
 * - EarthBlockController：ControllerInterface，认领接口，内部跑 EarthBlockFsm。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <earth_block_msgs/action/command.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "alg_fsm.h"
#include "Communication_Interface.hpp"
#include "motor_base.hpp"

namespace earth_block
{

/**
 * @brief 气泵数量：pump_target[0]/pump_status[0] 为吸盘，[1] 为气动导轨
 */
constexpr size_t kPumpCount = 2;

#pragma pack(push, 1)
/**
 * @brief 地块发送结构体（下发指令，PC → MCU，6B）
 *
 * motor_position_mm: 导轨目标位置 (mm)
 * pump_target[0]:    吸盘气泵目标，0=释放，1=吸附
 * pump_target[1]:    气动导轨气泵目标，0=底部，1=顶部
 */
struct EarthBlockTx
{
    float motor_position_mm;
    uint8_t pump_target[kPumpCount];
};

/**
 * @brief 地块接收结构体（MCU 回传，MCU → PC，10B）
 *
 * motor.velocity: 导轨速度 (mm/s)
 * motor.position: 导轨位置 (mm)
 * pump_status[0]: 吸盘气泵状态，PumpStatus
 * pump_status[1]: 气动导轨气泵状态，PumpStatus
 */
struct EarthBlockRx
{
    Device::MotorFeedbackFrame motor;
    uint8_t pump_status[kPumpCount];
};
#pragma pack(pop)

/**
 * @brief 气泵状态，取值与固件 Enum_AIRPUMP_Status 一致
 */
enum class PumpStatus : uint8_t
{
    kNone = 0,
    kOpen = 1,
    kClose = 2,
};

/**
 * @brief 地块硬件层（ros2_control SystemInterface）
 *
 * 收发交给 shared_package 的 CommunicationInterface（绑定结构体后自动打包/解包），
 * 本类 read/write 只读写 tx_buffer_/rx_buffer_ 两个结构体。
 *
 * URDF 只声明一个电机 joint（earth_block_motor_rail_joint）；
 * 两路气泵作为自定义接口挂在这个 joint 上：
 *   command: position / suction / air_rail
 *   state:   position / velocity / suction / air_rail
 */
class EarthBlockSystem : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // URDF 里唯一的电机 joint 名
    std::string joint_name_;

    // 命令接口后备存储
    double target_position_mm_ = 0.0;  // 导轨目标位置 (mm)
    double target_suction_ = 0.0;      // 吸盘目标：0=释放，1=吸附
    double target_air_rail_ = 0.0;     // 气动导轨目标：0=底部，1=顶部

    // 状态接口后备存储
    double now_position_mm_ = 0.0;     // 导轨位置 (mm)
    double now_velocity_mmps_ = 0.0;   // 导轨速度 (mm/s)
    double suction_status_ = 0.0;      // 吸盘气泵状态，PumpStatus
    double air_rail_status_ = 0.0;     // 气动导轨气泵状态，PumpStatus

    // 绑定到 CommunicationInterface 的两个结构体：发 / 收
    EarthBlockTx tx_buffer_{};
    EarthBlockRx rx_buffer_{};

    Middleware::CommunicationInterface * communication_interface_ = nullptr;  // 指向全局实例 g_usb_communication_interface
};

/**
 * @brief 地块状态机的内部状态
 *
 * Action 对外只有 PLACE 和 PICKUP 两个，每个 Action 内部是两段联动：
 *   PLACE  先走 kPlace，到位后切到 kInit
 *   PICKUP 先走 kPick，到位后切到 kUp
 */
enum class EarthBlockStatus : uint8_t
{
    kInit = 0,
    kPick,
    kUp,
    kPlace,
    kCount
};

/**
 * @brief Action 里的动作，取值与 earth_block_msgs/action/Command 的常量一致
 */
enum class EarthBlockAction : uint8_t
{
    kPlace = 0,
    kPickup,
    kCount
};

class EarthBlockController;

/**
 * @brief 地块状态机
 */
class EarthBlockFsm : public Algorithm::Fsm
{
public:
    EarthBlockController * controller_ = nullptr;

    void TimStatusPeriodElapsedCallback();
};

/**
 * @brief 地块控制器（自定义 ros2_control 控制器）
 *
 * 认领一个电机 joint 的 position/suction/air_rail command interface
 * 和 position/velocity/suction/air_rail state interface。
 * 每拍按 FSM 当前状态把目标刷进 command interface。
 */
class EarthBlockController : public controller_interface::ControllerInterface
{
public:
    ~EarthBlockController() override;

    // ControllerInterfaceBase 的纯虚，必须实现
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    friend class EarthBlockFsm;

private:
    // 状态数量，三个目标数组的大小都按它来
    static constexpr size_t kStatusCount = static_cast<size_t>(EarthBlockStatus::kCount);

    // 每个状态的目标，按 EarthBlockStatus 索引；新增状态时同步补数组元素
    const double motor_rail_target_[kStatusCount] = {
        100.0,  // kInit
        50.0,  // kPick
        100.0,  // kUp
        100.0,  // kPlace
    };
    const double suction_target_[kStatusCount] = {
        0.0,  // kInit
        1.0,  // kPick
        1.0,  // kUp
        1.0,  // kPlace
    };
    const double air_rail_target_[kStatusCount] = {
        0.0,  // kInit
        0.0,  // kPick
        1.0,  // kUp
        0.0,  // kPlace
    };

    // 参数：joint / action_name / command_timeout
    std::string joint_name_ = "earth_block_motor_rail_joint";
    std::string action_name_ = "/earth_block/command";
    double command_timeout_s_ = 8.0;

    // 状态接口读回来的值
    double now_position_mm_ = 0.0;
    double now_velocity_mmps_ = 0.0;
    double now_suction_ = 0.0;
    double now_air_rail_ = 0.0;

    // 每拍写给命令接口的目标
    double target_position_mm_ = 0.0;
    double target_suction_ = 0.0;
    double target_air_rail_ = 0.0;

    // 电机导轨到位误差阈值 (mm)
    double motor_rail_error_threshold_mm_ = 2.0;
    // 电机导轨到位速度阈值 (mm/s)
    double motor_rail_speed_threshold_mmps_ = 0.2;

    // 气泵到位延时阈值 (ms)
    double pump_arrive_delay_ms_ = 200.0;

    // 两路气泵各自的到位计时器
    struct PumpArriveTimer
    {
        bool timing = false;
        double last_target = -1.0;
        std::chrono::steady_clock::time_point start{};
    };
    PumpArriveTimer suction_arrive_timer_;
    PumpArriveTimer air_rail_arrive_timer_;

    EarthBlockFsm fsm_;

    // FSM 当前状态的原子镜像，供 HandleGoal() 在回调线程里做合法性检查
    std::atomic<uint8_t> current_status_{static_cast<uint8_t>(EarthBlockStatus::kInit)};

    // Action 服务端：导航发 goal 切状态，到位后回 result
    rclcpp_action::Server<earth_block_msgs::action::Command>::SharedPtr command_action_server_;
    // 监控 goal 的线程：等到位后回 result（不卡 executor）
    std::thread monitor_thread_;
    std::atomic<bool> shutdown_{false};
    // 收到的 Action 只写进这里，后续状态切换/到位判断逻辑自己写
    // 初值取 kCount，表示当前没有待处理的 Action
    std::atomic<uint8_t> action_status_{static_cast<uint8_t>(EarthBlockAction::kCount)};
    // kUp 下收到 PICKUP：标记为重新抓取，FSM 收到后回到 kPick 重走 Pickup
    std::atomic<bool> pickup_retry_{false};
    // RT 线程置位：请求的状态已到位；监控线程取走后清零
    std::atomic<bool> command_done_{false};
    // 防重入：正在执行时拒绝新 goal
    std::atomic<bool> busy_{false};

    using CommandGoalHandle = rclcpp_action::ServerGoalHandle<earth_block_msgs::action::Command>;

    // 把目标刷进对应的 command interface 后备存储
    void Move_To_Position(double x);
    void Suction_To_Position(double x);
    void Air_Rail_To_Position(double x);

    // 当前状态是否到位（目前只调三个空实现）
    bool IsActionFinished();

    // 到位判断：气泵比对反馈与目标并保持 pump_arrive_delay_ms_，导轨看位置/速度
    bool IsSuctionArrived();
    bool IsAirRailArrived();
    bool IsMotorRailArrived();
    bool PumpArrived(double now_status, double target, PumpArriveTimer &timer);

    // goal 回调：校验 + 防重入 + 写 pending，立刻返回 ACCEPT/REJECT
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID & uuid,
                                           std::shared_ptr<const earth_block_msgs::action::Command::Goal> goal);
    // cancel 回调：暂不支持中途取消
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<CommandGoalHandle> goal_handle);
    // accepted 回调：起监控线程
    void HandleAccepted(const std::shared_ptr<CommandGoalHandle> goal_handle);
    // 监控线程体：等 command_done_（带超时）后回 result
    void MonitorGoal(const std::shared_ptr<CommandGoalHandle> goal_handle);
};

} // namespace earth_block
