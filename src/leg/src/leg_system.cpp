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

#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mid_config.h>
#include <motor_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsk_config_and_callback.hpp>
#include <utils.hpp>

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

    // 按 URDF 关节声明顺序（前左/前右/后左/后右）存下关节名
    for (size_t i = 0; i < kLegCount; ++i)
    {
        joint_names_[i] = info.joints[i].name;
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 借出四条腿的 position/velocity/completion 状态接口
 *
 * @return std::vector<hardware_interface::StateInterface> 状态接口
 */
std::vector<hardware_interface::StateInterface> LegSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifs;
    for (size_t i = 0; i < kLegCount; ++i)
    {
        ifs.emplace_back(joint_names_[i], HW_IF_POSITION, &now_position_[i]);
        ifs.emplace_back(joint_names_[i], HW_IF_VELOCITY, &now_velocity_[i]);
        ifs.emplace_back(joint_names_[i], "completion", &completion_[i]);
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
    for (size_t i = 0; i < kLegCount; ++i)
    {
        ifs.emplace_back(joint_names_[i], HW_IF_POSITION, &target_position_[i]);
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
    // 指向全局实例，四条腿绑成一帧：下行 tx_buffer_，上行 rx_buffer_
    communication_interface_ = &Middleware::g_usb_communication_interface;

    if (!communication_interface_->Register(
            static_cast<uint8_t>(Middleware::CommFrameId::kLeg),
            &tx_buffer_, &rx_buffer_, sizeof(tx_buffer_), sizeof(rx_buffer_)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "绑定帧 id=%u 失败",
                     static_cast<uint8_t>(Middleware::CommFrameId::kLeg));
        return CallbackReturn::ERROR;
    }

    // 传入"进程正在关闭"的谓词：设备不在时 ros2_control_node 会卡死在构造期，
    // 收到 Ctrl+C 就据此结束等待并退出（详见 TaskInit 里的说明）
    if (!Task::TaskInit([] { return !rclcpp::ok(); }))
    {
        RCLCPP_ERROR(rclcpp::get_logger("LegSystem"), "打开 USB-CDC 失败");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("LegSystem"), "已绑定腿帧 id=%u（tx %zu 字节 / rx %zu 字节）",
                static_cast<uint8_t>(Middleware::CommFrameId::kLeg), sizeof(tx_buffer_), sizeof(rx_buffer_));
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
 * @brief 把 rx_buffer_ 里的值刷进状态接口
 * 数据在接收线程到达时已由 RxRptlCallback() 分发进 rx_buffer_，这里只做读取。
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < kLegCount; ++i)
    {
        now_position_[i] = static_cast<double>(rx_buffer_.motor[i].position);
        now_velocity_[i] = static_cast<double>(rx_buffer_.motor[i].velocity);
        // 前两条腿（0/1）共享 front 标志，后两条（2/3）共享 back 标志
        const uint8_t flag = (i < kLegCount / 2) ? rx_buffer_.front_completion_flag
                                                  : rx_buffer_.back_completion_flag;
        completion_[i] = static_cast<double>(flag);
    }

    // // 调试：表格更新式打印四条腿位置（windows_name "leg"，首次调用会自动弹独立窗口）
    // Utils::Debug_Log::SetMode("leg", Utils::Debug_Log::Mode::Refresh);
    // Utils::Debug_Log::Print("leg", "pos = %.3f %.3f %.3f %.3f",
    //                         rx_buffer_.motor[0].position, rx_buffer_.motor[1].position,
    //                         rx_buffer_.motor[2].position, rx_buffer_.motor[3].position);

    return hardware_interface::return_type::OK;
}

/**
 * @brief 把命令接口里的目标位置写进 tx_buffer_，并推一包
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type LegSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 只写 tx 结构体；Send() 不在这里调 —— 它会把所有已注册的帧打成一包，
    for (size_t i = 0; i < kLegCount; ++i)
    {
        tx_buffer_.position[i] = static_cast<float>(target_position_[i]);
    }

    Task::TaskLoop();
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
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    lift_action_name_ = auto_declare<std::string>("lift_action_name", lift_action_name_);
    lift_timeout_s_ = auto_declare<double>("lift_timeout", lift_timeout_s_);
    now_position_.assign(joint_names_.size(), 0.0);
    now_velocity_.assign(joint_names_.size(), 0.0);
    target_position_.assign(joint_names_.size(), 0.0);
    completion_.assign(joint_names_.size(), 0.0);

    fsm_leg_.controller_ = this;
    fsm_leg_.Init(static_cast<uint8_t>(LegStatus::kCount), static_cast<uint8_t>(LegStatus::kInit));

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 建抬腿 Action 服务端：导航发 goal 切状态，到位后回 result
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn LegController::on_configure(const rclcpp_lifecycle::State &)
{
    lift_action_server_ = rclcpp_action::create_server<leg_msgs::action::Lift>(
        get_node(), lift_action_name_,
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const leg_msgs::action::Lift::Goal> goal)
        { return HandleGoal(uuid, goal); },
        [this](const std::shared_ptr<LiftGoalHandle> goal_handle)
        { return HandleCancel(goal_handle); },
        [this](const std::shared_ptr<LiftGoalHandle> goal_handle)
        { HandleAccepted(goal_handle); });

    RCLCPP_INFO(get_node()->get_logger(), "抬腿动作 %s", lift_action_name_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 停掉监控线程并释放 Action 服务端，方便重配置
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn LegController::on_cleanup(const rclcpp_lifecycle::State &)
{
    shutdown_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    lift_action_server_.reset();
    shutdown_.store(false);
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 认领四条腿的 position 命令接口（写入硬件侧的 target_position_）
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration LegController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_)
    {
        cfg.names.push_back(name + "/" + HW_IF_POSITION);
    }
    return cfg;
}

/**
 * @brief 认领四条腿的 position/velocity/completion 状态接口
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration LegController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_)
    {
        cfg.names.push_back(name + "/" + HW_IF_POSITION);
        cfg.names.push_back(name + "/" + HW_IF_VELOCITY);
        cfg.names.push_back(name + "/completion");
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
    if (!Task::g_mcu_alive)
    {
        return controller_interface::return_type::OK;
    }

    // 应用 goal 请求的状态切换（在实时线程里改状态机，避免与 goal 回调线程竞争）
    const uint8_t pending = pending_status_.exchange(static_cast<uint8_t>(LegStatus::kCount));
    if (pending != static_cast<uint8_t>(LegStatus::kCount))
    {
        fsm_leg_.SetStatus(pending);
    }

    // INDIVIDUAL 下 state_interfaces_ 的顺序 = state_interface_configuration() 里 names 的顺序
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        now_position_[i] = state_interfaces_[3 * i].get_value();
        now_velocity_[i] = state_interfaces_[3 * i + 1].get_value();
        completion_[i]   = state_interfaces_[3 * i + 2].get_value();
        command_interfaces_[i].set_value(target_position_[i]);
    }

    fsm_leg_.TimStatusPeriodElapsedCallback();

    // 有 goal 在跑，且腿已到位 → 通知监控线程
    if (busy_.load() && IsActionFinished())
    {
        lift_done_.store(true);
    }

    return controller_interface::return_type::OK;
}

/**
 * @brief 按当前状态把目标位置刷进 target_position_
 */
void LegController::MoveToPosition()
{
    const uint8_t status = fsm_leg_.GetNowStatusSerial();

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        target_position_[i] = approach_target_[status][i];
    }
}

/**
 * @brief 当前状态要动的腿是否到位：直接读下位机回传的 completion 标志
 *
 * @return true 到位
 * @return false 还没到
 */
bool LegController::IsActionFinished()
{
    // 前两条腿（0/1）共享 front 标志，后两条（2/3）共享 back 标志，各取一个即可
    const bool front_done = completion_[0] > 0.5;
    const bool back_done  = completion_[2] > 0.5;

    // 只判断本状态实际要动的那组腿，避免另一组残留的旧标志误判
    switch (fsm_leg_.GetNowStatusSerial())
    {
    case static_cast<uint8_t>(LegStatus::kLift_Front):
        return front_done;
    case static_cast<uint8_t>(LegStatus::kLift_Back):
        return back_done;
    case static_cast<uint8_t>(LegStatus::kInit):
    case static_cast<uint8_t>(LegStatus::kLift_All):
        return front_done && back_done;
    default:
        return true;
    }
}

/**
 * @brief goal 回调：校验 + 防重入 + 写 pending，立刻返回
 */
rclcpp_action::GoalResponse LegController::HandleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const leg_msgs::action::Lift::Goal> goal)
{
    // 越界丢弃，防止 SetStatus 写到状态数组外
    if (goal->status >= static_cast<uint8_t>(LegStatus::kCount))
    {
        RCLCPP_WARN(rclcpp::get_logger("LegController"), "收到非法腿状态 %u", goal->status);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 防重入：正在执行时拒绝新 goal
    if (busy_.exchange(true))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 开始新动作：清上次结果，把目标状态交给 update() 的实时线程
    lift_done_.store(false);
    pending_status_.store(goal->status);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief cancel 回调：第一版不支持中途取消
 */
rclcpp_action::CancelResponse LegController::HandleCancel(const std::shared_ptr<LiftGoalHandle>)
{
    return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief accepted 回调：起一个监控线程等结果，不阻塞 executor
 */
void LegController::HandleAccepted(const std::shared_ptr<LiftGoalHandle> goal_handle)
{
    // 上一条 goal 的监控线程此时已结束（busy_ 已释放），先回收再起新的；
    // 否则给 joinable 的 std::thread 赋新值会触发 std::terminate
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    monitor_thread_ = std::thread([this, goal_handle]() { MonitorGoal(goal_handle); });
}

/**
 * @brief 监控线程体：等到位（带超时）后回 result
 */
void LegController::MonitorGoal(const std::shared_ptr<LiftGoalHandle> goal_handle)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(lift_timeout_s_);

    while (!lift_done_.load() && !shutdown_.load() && rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 关闭中：直接放弃，别回 result
    if (shutdown_.load() || !rclcpp::ok())
    {
        busy_.store(false);
        return;
    }

    // 取走结果并清零（收到即清除），释放 busy
    const bool done = lift_done_.exchange(false);
    busy_.store(false);

    auto result = std::make_shared<leg_msgs::action::Lift::Result>();
    result->success = done;
    result->message = done ? "done" : "timeout";

    if (done)
    {
        goal_handle->succeed(result);
    }
    else
    {
        goal_handle->abort(result);
    }
}

/**
 * @brief 状态机周期回调，节奏由 launch 的 update_rate 决定（10 Hz）
 */
void LegFsm::TimStatusPeriodElapsedCallback()
{
    status_[now_status_serial_].count_time++;

    // 只按当前状态刷新目标位置；状态切换由外部指令驱动
    controller_->MoveToPosition();
}

} // namespace leg

PLUGINLIB_EXPORT_CLASS(leg::LegSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(leg::LegController, controller_interface::ControllerInterface)
