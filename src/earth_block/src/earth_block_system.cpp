/**
 * @file    earth_block_system.cpp
 * @author  Carbon
 * @date    2026-11-19
 * @brief   地块机构硬件层 + 控制层实现
 */

/* Includes ------------------------------------------------------------------*/

#include "earth_block/earth_block_system.hpp"

#include <chrono>
#include <cmath>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mid_config.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tsk_config_and_callback.hpp>

namespace earth_block
{

using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

// ---------------- EarthBlockSystem（硬件层） ----------------

/**
 * @brief 初始化：读 URDF 里唯一的电机 joint 名
 *
 * @param info 硬件描述
 * @return CallbackReturn 结果
 */
CallbackReturn EarthBlockSystem::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    if (info.joints.size() != 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("EarthBlockSystem"),
                     "地块只支持一个电机 joint，URDF 里声明了 %zu 个", info.joints.size());
        return CallbackReturn::ERROR;
    }

    joint_name_ = info.joints[0].name;
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 借出电机导轨和气泵的状态接口
 *
 * @return std::vector<hardware_interface::StateInterface> 状态接口
 */
std::vector<hardware_interface::StateInterface> EarthBlockSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifs;
    ifs.emplace_back(joint_name_, HW_IF_POSITION, &now_position_mm_);
    ifs.emplace_back(joint_name_, HW_IF_VELOCITY, &now_velocity_mmps_);
    ifs.emplace_back(joint_name_, "suction", &suction_status_);
    ifs.emplace_back(joint_name_, "air_rail", &air_rail_status_);
    return ifs;
}

/**
 * @brief 借出电机导轨和气泵的命令接口
 *
 * @return std::vector<hardware_interface::CommandInterface> 命令接口
 */
std::vector<hardware_interface::CommandInterface> EarthBlockSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifs;
    ifs.emplace_back(joint_name_, HW_IF_POSITION, &target_position_mm_);
    ifs.emplace_back(joint_name_, "suction", &target_suction_);
    ifs.emplace_back(joint_name_, "air_rail", &target_air_rail_);
    return ifs;
}

/**
 * @brief 绑定 id=2 通信帧并打开 USB-CDC
 *
 * @return CallbackReturn 结果
 */
CallbackReturn EarthBlockSystem::on_configure(const rclcpp_lifecycle::State &)
{
    // 指向全局实例，整个地块绑成一帧：下行 tx_buffer_，上行 rx_buffer_
    communication_interface_ = &Middleware::g_usb_communication_interface;

    if (!communication_interface_->Register(
            static_cast<uint8_t>(Middleware::CommFrameId::kEarthBlock),
            &tx_buffer_, &rx_buffer_, sizeof(tx_buffer_), sizeof(rx_buffer_)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("EarthBlockSystem"), "绑定帧 id=%u 失败",
                     static_cast<uint8_t>(Middleware::CommFrameId::kEarthBlock));
        return CallbackReturn::ERROR;
    }

    // 传入"进程正在关闭"的谓词：设备不在时 ros2_control_node 会卡死在构造期，
    // 收到 Ctrl+C 就据此结束等待并退出（详见 TaskInit 里的说明）
    if (!Task::TaskInit([] { return !rclcpp::ok(); }))
    {
        RCLCPP_ERROR(rclcpp::get_logger("EarthBlockSystem"), "打开 USB-CDC 失败");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("EarthBlockSystem"), "已绑定地块帧 id=%u（tx %zu 字节 / rx %zu 字节）",
                static_cast<uint8_t>(Middleware::CommFrameId::kEarthBlock), sizeof(tx_buffer_), sizeof(rx_buffer_));
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活，暂无动作
 *
 * @return CallbackReturn 结果
 */
CallbackReturn EarthBlockSystem::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 去激活，暂无动作
 *
 * @return CallbackReturn 结果
 */
CallbackReturn EarthBlockSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 把 rx_buffer_ 里的值刷进状态接口
 * 数据在接收线程到达时已由 RxRptlCallback() 分发进 rx_buffer_，这里只做读取。
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type EarthBlockSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    now_position_mm_ = static_cast<double>(rx_buffer_.motor.position);
    now_velocity_mmps_ = static_cast<double>(rx_buffer_.motor.velocity);
    suction_status_ = static_cast<double>(rx_buffer_.pump_status[0]);
    air_rail_status_ = static_cast<double>(rx_buffer_.pump_status[1]);
    return hardware_interface::return_type::OK;
}

/**
 * @brief 把命令接口里的目标写进 tx_buffer_
 *
 * @note 发送由 LegSystem::write() 里的 Task::TaskLoop() 统一触发，
 *       这里只更新本帧数据，避免每帧被重复发送。
 *
 * @return hardware_interface::return_type 结果
 */
hardware_interface::return_type EarthBlockSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    tx_buffer_.motor_position_mm = static_cast<float>(target_position_mm_);
    tx_buffer_.pump_target[0] = target_suction_ > 0.5 ? 1U : 0U;
    tx_buffer_.pump_target[1] = target_air_rail_ > 0.5 ? 1U : 0U;
    return hardware_interface::return_type::OK;
}

// ---------------- EarthBlockController（控制层） ----------------

/**
 * @brief 析构：停掉监控线程，避免 std::thread 在 joinable 状态下析构触发 std::terminate
 *
 * ControllerManager 退出时可能直接销毁 controller 插件，不走 on_cleanup()；
 * 只要发过 action，monitor_thread_ 就一直是 joinable 的，这里必须兜底 join。
 */
EarthBlockController::~EarthBlockController()
{
    shutdown_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
}

/**
 * @brief 读参数并初始化状态机
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn EarthBlockController::on_init()
{
    joint_name_ = auto_declare<std::string>("joint", joint_name_);
    action_name_ = auto_declare<std::string>("action_name", action_name_);
    command_timeout_s_ = auto_declare<double>("command_timeout", command_timeout_s_);

    fsm_.controller_ = this;
    fsm_.Init(static_cast<uint8_t>(EarthBlockStatus::kCount),
              static_cast<uint8_t>(EarthBlockStatus::kInit));
    current_status_.store(fsm_.GetNowStatusSerial());

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 建地块 Action 服务端：导航发 goal 切状态，到位后回 result
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn EarthBlockController::on_configure(const rclcpp_lifecycle::State &)
{
    command_action_server_ = rclcpp_action::create_server<earth_block_msgs::action::Command>(
        get_node(), action_name_,
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const earth_block_msgs::action::Command::Goal> goal)
        { return HandleGoal(uuid, goal); },
        [this](const std::shared_ptr<CommandGoalHandle> goal_handle)
        { return HandleCancel(goal_handle); },
        [this](const std::shared_ptr<CommandGoalHandle> goal_handle)
        { HandleAccepted(goal_handle); });

    RCLCPP_INFO(get_node()->get_logger(), "地块动作 %s", action_name_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 停掉监控线程并释放 Action 服务端，方便重配置
 *
 * @return controller_interface::CallbackReturn 结果
 */
controller_interface::CallbackReturn EarthBlockController::on_cleanup(const rclcpp_lifecycle::State &)
{
    shutdown_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    command_action_server_.reset();
    shutdown_.store(false);
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 认领电机 joint 的 position/suction/air_rail 命令接口
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration EarthBlockController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.push_back(joint_name_ + "/" + HW_IF_POSITION);
    cfg.names.push_back(joint_name_ + "/suction");
    cfg.names.push_back(joint_name_ + "/air_rail");
    return cfg;
}

/**
 * @brief 认领电机 joint 的 position/velocity/suction/air_rail 状态接口
 *
 * @return controller_interface::InterfaceConfiguration 配置
 */
controller_interface::InterfaceConfiguration EarthBlockController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.push_back(joint_name_ + "/" + HW_IF_POSITION);
    cfg.names.push_back(joint_name_ + "/" + HW_IF_VELOCITY);
    cfg.names.push_back(joint_name_ + "/suction");
    cfg.names.push_back(joint_name_ + "/air_rail");
    return cfg;
}

/**
 * @brief 每拍：读状态 → 走一拍状态机 → 写目标
 *
 * @return controller_interface::return_type 结果
 */
controller_interface::return_type EarthBlockController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 下位机断链：整个控制循环停手，状态机也冻结，等下一次 alive 再继续
    if (!Task::g_mcu_alive)
    {
        return controller_interface::return_type::OK;
    }

    // INDIVIDUAL 下 state_interfaces_ 的顺序 = state_interface_configuration() 里 names 的顺序
    now_position_mm_ = state_interfaces_[0].get_value();
    now_velocity_mmps_ = state_interfaces_[1].get_value();
    now_suction_ = state_interfaces_[2].get_value();
    now_air_rail_ = state_interfaces_[3].get_value();

    fsm_.TimStatusPeriodElapsedCallback();
    current_status_.store(fsm_.GetNowStatusSerial());

    command_interfaces_[0].set_value(target_position_mm_);
    command_interfaces_[1].set_value(target_suction_);
    command_interfaces_[2].set_value(target_air_rail_);

    // TODO: 后续状态切换逻辑在这里写：
    //   读 action_status_ / fsm_.GetNowStatusSerial()，按到位判断切换状态，
    //   动作全部完成时 command_done_.store(true);

    return controller_interface::return_type::OK;
}

/**
 * @brief 把导轨目标刷进 command interface 后备存储
 */
void EarthBlockController::Move_To_Position(double x)
{
    target_position_mm_ = x;
}

/**
 * @brief 把吸盘目标刷进 command interface 后备存储
 */
void EarthBlockController::Suction_To_Position(double x)
{
    target_suction_ = x;
}

/**
 * @brief 把气动导轨目标刷进 command interface 后备存储
 */
void EarthBlockController::Air_Rail_To_Position(double x)
{
    target_air_rail_ = x;
}

/**
 * @brief 当前状态是否到位
 *
 * @return true 到位
 * @return false 还没到
 */
bool EarthBlockController::IsActionFinished()
{
    return IsSuctionArrived() && IsAirRailArrived() && IsMotorRailArrived();
}

/**
 * @brief 气泵到位判断：反馈状态与目标对应状态一致，并持续 pump_arrive_delay_ms_
 *
 * 目标值：0=Close，1=Open；反馈值 PumpStatus：1=Open，2=Close。
 * 反馈与目标不匹配时计时器清零，重新匹配后重新计 200ms。
 */
bool EarthBlockController::PumpArrived(double now_status, double target, PumpArriveTimer &timer)
{
    // 目标变化后重新计时
    if (std::fabs(timer.last_target - target) > 0.5)
    {
        timer.last_target = target;
        timer.timing = false;
    }

    const double expected = target > 0.5 ? static_cast<double>(PumpStatus::kOpen) : static_cast<double>(PumpStatus::kClose);

    if (std::fabs(now_status - expected) >= 0.5)
    {
        timer.timing = false;
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!timer.timing)
    {
        timer.timing = true;
        timer.start = now;
        return false;
    }

    return std::chrono::duration<double, std::milli>(now - timer.start).count() >= pump_arrive_delay_ms_;
}

/**
 * @brief 吸盘到位判断
 */
bool EarthBlockController::IsSuctionArrived()
{
    return PumpArrived(now_suction_, target_suction_, suction_arrive_timer_);
}

/**
 * @brief 气动导轨到位判断
 */
bool EarthBlockController::IsAirRailArrived()
{
    return PumpArrived(now_air_rail_, target_air_rail_, air_rail_arrive_timer_);
}

/**
 * @brief 电机导轨到位判断：位置误差在阈值内，且速度小于阈值
 */
bool EarthBlockController::IsMotorRailArrived()
{
    return std::fabs(now_position_mm_ - target_position_mm_) <=
               motor_rail_error_threshold_mm_ &&
           std::fabs(now_velocity_mmps_) < motor_rail_speed_threshold_mmps_;
}

/**
 * @brief goal 回调：校验 + 防重入 + 写 pending，立刻返回
 */
rclcpp_action::GoalResponse EarthBlockController::HandleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const earth_block_msgs::action::Command::Goal> goal)
{
    // 越界丢弃，防止后续查表越界（Action 只有 PLACE / PICKUP 两个状态）
    if (goal->status >= static_cast<uint8_t>(EarthBlockAction::kCount))
    {
        RCLCPP_WARN(rclcpp::get_logger("EarthBlockController"), "收到非法地块 Action 状态 %u", goal->status);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // PICKUP 允许从 kInit 开始，也允许在 kUp 重新抓取；
    // PLACE 只允许从 kUp 开始。
    const uint8_t now_status = current_status_.load();
    const bool is_pickup = goal->status == static_cast<uint8_t>(EarthBlockAction::kPickup);
    const bool is_place = goal->status == static_cast<uint8_t>(EarthBlockAction::kPlace);
    const bool pickup_from_up = is_pickup && now_status == static_cast<uint8_t>(EarthBlockStatus::kUp);

    if ((is_pickup &&
         now_status != static_cast<uint8_t>(EarthBlockStatus::kInit) &&
         now_status != static_cast<uint8_t>(EarthBlockStatus::kUp)) ||
        (is_place && now_status != static_cast<uint8_t>(EarthBlockStatus::kUp)))
    {
        RCLCPP_WARN(rclcpp::get_logger("EarthBlockController"),
                    "地块 Action 与当前状态不匹配：action=%u, state=%u",
                    goal->status, now_status);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 防重入：正在执行时拒绝新 goal
    if (busy_.exchange(true))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 先记重试标记和 Action，再清上次结果，避免中间被状态机误判成已完成
    pickup_retry_.store(pickup_from_up);
    action_status_.store(goal->status);
    command_done_.store(false);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief cancel 回调：暂不支持中途取消
 */
rclcpp_action::CancelResponse EarthBlockController::HandleCancel(const std::shared_ptr<CommandGoalHandle>)
{
    return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief accepted 回调：起一个监控线程等结果，不阻塞 executor
 */
void EarthBlockController::HandleAccepted(const std::shared_ptr<CommandGoalHandle> goal_handle)
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
void EarthBlockController::MonitorGoal(const std::shared_ptr<CommandGoalHandle> goal_handle)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(command_timeout_s_);

    while (!command_done_.load() && !shutdown_.load() && rclcpp::ok() &&
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
    const bool done = command_done_.exchange(false);
    busy_.store(false);

    auto result = std::make_shared<earth_block_msgs::action::Command::Result>();
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
 * @brief 状态机周期回调，节奏由 launch 的 update_rate 决定（100 Hz）
 */
void EarthBlockFsm::TimStatusPeriodElapsedCallback()
{
    status_[now_status_serial_].count_time++;

    // 只按当前状态刷新目标；状态切换由外部指令驱动
    if (controller_ == nullptr)
    {
        return;
    }

    controller_->Move_To_Position(controller_->motor_rail_target_[now_status_serial_]);
    controller_->Suction_To_Position(controller_->suction_target_[now_status_serial_]);
    controller_->Air_Rail_To_Position(controller_->air_rail_target_[now_status_serial_]);

    switch (static_cast<EarthBlockStatus>(now_status_serial_))
    {
    case EarthBlockStatus::kInit:
    {
        if (controller_->IsActionFinished())
        {
            const uint8_t action = controller_->action_status_.load();
            if (action == static_cast<uint8_t>(EarthBlockAction::kPickup))
            {
                // Pickup 从 kInit 起第一段
                SetStatus(static_cast<uint8_t>(EarthBlockStatus::kPick));
            }
            else if (action == static_cast<uint8_t>(EarthBlockAction::kPlace))
            {
                // Place 第二段（kInit）到位，整个 Action 完成
                controller_->action_status_.store(static_cast<uint8_t>(EarthBlockAction::kCount));
                controller_->command_done_.store(true);
            }
        }
        break;
    }
    case EarthBlockStatus::kPick:
    {
        if (controller_->IsActionFinished())
        {
            SetStatus(static_cast<uint8_t>(EarthBlockStatus::kUp));
        }
        break;
    }
    case EarthBlockStatus::kUp:
    {
        // kUp 收到 PICKUP 重试：重新从 kPick 开始抓取
        if (controller_->pickup_retry_.exchange(false))
        {
            SetStatus(static_cast<uint8_t>(EarthBlockStatus::kPick));
            break;
        }

        if (controller_->IsActionFinished())
        {
            const uint8_t action = controller_->action_status_.load();
            if (action == static_cast<uint8_t>(EarthBlockAction::kPlace))
            {
                // Place 从 kUp 起第一段
                SetStatus(static_cast<uint8_t>(EarthBlockStatus::kPlace));
            }
            else if (action == static_cast<uint8_t>(EarthBlockAction::kPickup))
            {
                // Pickup 第二段（kUp）到位，整个 Action 完成
                controller_->action_status_.store(static_cast<uint8_t>(EarthBlockAction::kCount));
                controller_->command_done_.store(true);
            }
        }
        break;
    }
    case EarthBlockStatus::kPlace:
    {
        if (controller_->IsActionFinished())
        {
            SetStatus(static_cast<uint8_t>(EarthBlockStatus::kInit));
        }
        break;
    }
    default:
        break;
    }

}

} // namespace earth_block

PLUGINLIB_EXPORT_CLASS(earth_block::EarthBlockSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(earth_block::EarthBlockController, controller_interface::ControllerInterface)
