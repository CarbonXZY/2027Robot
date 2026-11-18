/**
 * @file mod_earth_block.h
 * @brief Earth Block 机构：两路吸盘 + 气动导轨 + C610 电机导轨
 *
 * 由 KFC 机构移植。气动件使用 dvc_airtool 的 Class_AIRPUMP：
 *   吸盘：Open=吸，Close=放
 *   气动导轨：Open=顶部，Close=底部（高电平=顶部）
 *
 * C610 导轨电机集中放在 Device::g_motor，由 MotorInstances::Init() 初始化，
 * 本模块只持有引用。通信层 EarthBlockInteract 作为成员，用 this 绑定，
 * 在 Init() 里自动注册 id=2 的收发帧。用法（须先调 Device::g_motor.Init()）：
 *   Module::EarthBlock earth_block(Device::g_motor.earth_block);
 *   earth_block.Init(hardware);
 *   1ms 调 TimCalculate1msCallback()，100ms 调 TimAlive100msCallback()，
 *   CAN 反馈调 FdcanRxCpltCallback()。
 */

#ifndef MOD_EARTH_BLOCK_H
#define MOD_EARTH_BLOCK_H

#include <cmath>
#include <cstdint>

#include "dvc_airtool.h"
#include "dvc_dwt.h"
#include "dvc_motor_dji.h"

namespace Module
{

class EarthBlock;

#pragma pack(push, 1)
/**
 * @brief Earth Block 发送结构体（下发指令，上位机 → 本板，6B）
 *
 * motor_position_mm: 导轨目标位置 (mm)
 * pump_target[0]:    吸盘气泵目标，0=释放，1=吸附
 * pump_target[1]:    气动导轨气泵目标，0=底部，1=顶部
 */
struct EarthBlockTx
{
    float motor_position_mm;
    uint8_t pump_target[2];
};

/**
 * @brief Earth Block 接收结构体（本板回传，本板 → 上位机，10B）
 *
 * motor.position: 导轨毫米位置 (mm)
 * motor.velocity: 导轨速度 (mm/s)
 * pump_status[0]: 吸盘气泵状态，Enum_AIRPUMP_Status
 * pump_status[1]: 气动导轨气泵状态，Enum_AIRPUMP_Status
 */
struct EarthBlockRx
{
    Device::MotorFeedbackFrame motor;
    uint8_t pump_status[2];
};
#pragma pack(pop)

/**
 * @brief Earth Block 通信层
 *
 * 只持有 EarthBlock 指针，不接触电机/GPIO 细节：
 * 下行把上位机目标翻译成 EarthBlock 动作，上行回填导轨反馈和气泵状态。
 * 实例由 EarthBlock 持有，在 EarthBlock::Init() 里用 Init(this) 绑定。
 */
class EarthBlockInteract
{
public:
    EarthBlockInteract() = default;
    EarthBlockInteract(const EarthBlockInteract &) = delete;
    EarthBlockInteract &operator=(const EarthBlockInteract &) = delete;

    /**
     * @brief 绑定 EarthBlock 并注册 id=2 的收发帧
     * @param earth_block 被控机构，由 EarthBlock::Init() 传入 this
     */
    void Init(EarthBlock *earth_block);

    /**
     * @brief 1ms：下发目标 → 推进 EarthBlock 状态机 → 回填反馈
     */
    void TimCalculate1msCallback();

    // 下行：上位机下发的目标，通信中间件直接写入
    EarthBlockTx tx_buffer_{};
    // 上行：回传给上位机的电机反馈和气泵状态
    EarthBlockRx rx_buffer_{};

protected:
    EarthBlock *earth_block_ = nullptr; // 被控机构，本类不拥有

    uint8_t last_pump_target_[2] = {0, 0}; // 上次已接受的气泵目标
    float last_motor_target_mm_ = 0.0f;    // 上次已接受的导轨目标 (mm)

    void UsbRegister();
    void UsbSetTarget();
    void UsbFeedback();
};

/**
 * @brief Earth Block 机构
 *
 * 只管理机构动作和吸盘/气动导轨，C610 导轨的校准、
 * 单位换算和反馈均由本模块内部完成。
 */
class EarthBlock
{
public:
    static constexpr uint8_t kCommandQueueSize = 8; // 环形队列容量，实际最多保存 7 条

    /**
     * @brief 机构状态
     */
    enum class Status
    {
        kDisable = 0,          // 未初始化或已停止
        kIdle,                 // 已初始化，等待新动作
        kQueued,               // 命令已出队，等待本周期执行
        kSuctionWait,          // 吸盘命令已输出，等待吸附/释放稳定
        kAirRailWait,          // 气动导轨命令已输出，等待动作稳定
        kMotorRailMoving,      // C610 导轨正在移动，等待位置到位
        kMotorRailCalibrating, // C610 导轨正在执行堵转回零
        kFinished,             // 当前动作完成，可启动下一个动作
        kFault,                // 超时或错误，电机已停止
    };

    /**
     * @brief 原子动作
     */
    enum class Action
    {
        kNone = 0,             // 没有有效动作
        kSuctionOn,            // 指定吸盘开始吸附
        kSuctionOff,           // 指定吸盘停止吸附并释放
        kAirRailUp,            // 气动导轨到顶部
        kAirRailDown,          // 气动导轨到底部
        kMotorRailMove,        // C610 导轨移动到指定毫米位置
        kMotorRailCalibrate,   // C610 导轨执行堵转回零
        kMotorRailSetZero,     // 将 C610 当前角度直接设为零点
    };

    /**
     * @brief 引脚配置
     */
    struct HardwareConfig
    {
        GPIO_TypeDef *suction_0_port = nullptr; // 左吸盘 GPIO 端口
        uint16_t suction_0_pin = 0;             // 左吸盘引脚位掩码

        GPIO_TypeDef *suction_1_port = nullptr; // 右吸盘 GPIO 端口；nullptr=临时停用
        uint16_t suction_1_pin = 0;             // 右吸盘引脚位掩码；0=临时停用

        GPIO_TypeDef *air_rail_port = nullptr;  // 气动导轨 GPIO 端口
        uint16_t air_rail_pin = 0;              // 气动导轨引脚位掩码
    };

    /**
     * @brief 构造 Earth Block，绑定 Device::g_motor 中的 C610
     * @param motor 已由 MotorInstances::Init() 初始化的导轨电机
     */
    explicit EarthBlock(Device::MotorDjiC610 &motor);

    EarthBlock() = delete;
    EarthBlock(const EarthBlock &) = delete;
    EarthBlock &operator=(const EarthBlock &) = delete;

    /**
     * @brief 初始化吸盘、气动导轨，并检查 C610 已由实例层初始化
     * @param hardware 引脚配置
     * @return true 初始化成功
     */
    bool Init(const HardwareConfig &hardware);

    /**
     * @brief 1ms 周期入口，建议每 1 ms 调用一次
     *
     * 内部转给 EarthBlockInteract：下发上位机目标 → 推进状态机 → 回填反馈。
     */
    void TimCalculate1msCallback();

    /**
     * @brief 将 C610 反馈转交给内部电机
     */
    void FdcanRxCpltCallback(uint8_t *rx_data);

    /**
     * @brief 执行 C610 100 ms 在线检测
     */
    void TimAlive100msCallback();

    /**
     * @brief 吸取指定吸盘
     * @param cup_index 0 或 1
     */
    bool SuckCup(uint8_t cup_index);

    /**
     * @brief 松开指定吸盘
     * @param cup_index 0 或 1
     */
    bool ReleaseCup(uint8_t cup_index);

    /**
     * @brief 控制气动导轨到顶部
     */
    bool AirRailUp();

    /**
     * @brief 控制气动导轨到底部
     */
    bool AirRailDown();

    /**
     * @brief 将 C610 电机导轨移动到指定毫米位置
     */
    bool MoveMotorRailTo(float position_mm);

    /**
     * @brief 启动 C610 电机导轨堵转校准
     */
    bool CalibrateMotorRail();

    /**
     * @brief 将当前电机位置直接设为零点
     */
    bool SetCurrentMotorRailPositionAsZero();

    /**
     * @brief 立即停止电机导轨
     */
    void StopMotorRail();

    inline Status GetStatus() const;
    inline Action GetLastAction() const;
    inline bool IsInitialized() const;
    inline bool IsBusy() const;
    inline bool IsActionFinished() const;
    inline bool IsMotorRailCalibrated() const;
    inline float GetMotorRailPositionMm() const;
    inline float GetMotorRailSpeedMmps() const;
    inline float GetMotorRailTargetMm() const;
    inline float GetMotorRailRawPosition() const;
    // 注意：Class_AIRPUMP::Get_Status() 本身不是 const，故这两个接口也不能是 const
    inline Enum_AIRPUMP_Status GetSuctionStatus(uint8_t cup_index);
    inline Enum_AIRPUMP_Status GetAirRailStatus();

    uint8_t GetCommandCount() const;
    bool IsCommandQueueFull() const;
    void ClearCommandQueue();
    void ClearFault();

protected:
    friend class EarthBlockInteract; // 允许通信层推进状态机

    struct Command
    {
        Action action = Action::kNone;  // 要执行的动作
        uint8_t suction_index = 0;      // 吸盘命令使用：0=左，1=右
        float target_mm = 0.0f;         // 电机导轨移动目标 (mm)
    };

    struct MotorRailConfig
    {
        float mm_per_motor_rad = 20.0f;  // 电机轴每 1 rad 对应导轨移动 20 mm
        float direction_sign = 1.0f;     // 1=电机正向使坐标增大，-1=方向相反
        float min_mm = 0.0f;             // 软件最小坐标 (mm)
        float max_mm = 140.0f;           // 7 rad × 20 mm/rad = 140 mm
        float arrive_position_mm = 1.0f; // 到点位置误差阈值 (mm)
        float arrive_speed_mmps = 2.0f;  // 到点速度阈值 (mm/s)

        // 法一：大误差定速接近，小误差切位置闭环
        float approach_speed_radps = 20.0f; // 大误差接近速度 (rad/s)
        float switch_error_rad = 0.1f;      // 小于该误差后切入位置闭环 (rad)

        Device::CalibrateParams calibration = { // 恒速撞限位，持续低速后记录零点
            .motion_mode = Device::CalibrateMotionMode::kSpeed,  // 恒速回零
            .motion_value = -3.0f,                               // 回零速度 (rad/s)
            .detect_mode = Device::CalibrateDetectMode::kSpeed,  // 按速度判断堵转
            .detect_threshold = 0.1f,                            // 堵转速度阈值 (rad/s)
            .debounce_us = 20000,                                // 低速持续时间 (us)
        };
    };

    struct MotionConfig
    {
        uint32_t suction_settle_us = 500000;            // 吸取后等待真空建立 (us)
        uint32_t release_settle_us = 200000;            // 松开后等待吸盘脱离 (us)
        uint32_t air_rail_settle_us = 500000;           // 气动导轨换向等待时间 (us)
        uint32_t motor_rail_move_timeout_us = 10000000; // 电机导轨移动超时 (us)
        MotorRailConfig motor_rail = {};                // C610 导轨机械换算和回零参数
    };

    Device::MotorDjiC610 *motor_ = nullptr; // 指向 Device::g_motor.earth_block

    EarthBlockInteract interact_; // 通信层，Init(this) 时绑定本对象

    Class_AIRPUMP suction_cup_[2]; // [0]左吸盘，[1]右吸盘
    Class_AIRPUMP air_rail_;       // 气动导轨，高电平=顶部，低电平=底部

    HardwareConfig hardware_ = {}; // 外部硬件配置副本
    MotionConfig motion_ = {};     // 机构内部运动参数

    bool suction_enabled_[2] = {false, false}; // 对应吸盘是否接线启用
    bool initialized_ = false;                 // GPIO 和电机导轨参数全部初始化成功后置 true

    Status status_ = Status::kDisable; // 当前机构状态
    Command current_command_ = {};     // 当前正在执行或最近执行的完整命令
    uint32_t action_start_time_us_ = 0; // 动作出队开始执行的时刻 (us)

    Command command_queue_[kCommandQueueSize] = {}; // 待执行命令环形队列
    volatile uint8_t command_queue_head_ = 0;       // 生产者下一次写入位置
    volatile uint8_t command_queue_tail_ = 0;       // 消费者下一次读取位置

    MotorRailConfig motor_rail_param_ = {};          // 导轨参数运行副本，方向统一为 ±1
    float motor_rail_offset_rad_ = 0.0f;             // 机械零点对应的电机原始角度 (rad)
    float motor_rail_current_mm_ = 0.0f;             // 相对机械零点的当前位置 (mm)
    float motor_rail_current_speed_mmps_ = 0.0f;     // 当前坐标方向上的速度 (mm/s)
    float motor_rail_target_mm_ = 0.0f;              // 当前毫米目标

    bool motor_rail_initialized_ = false;        // 导轨机械参数合法
    bool motor_rail_calibrated_ = false;         // 已建立毫米坐标
    bool motor_rail_calibration_active_ = false; // 正在回零
    bool motor_rail_control_enabled_ = false;    // 允许输出位置目标

    void Calculate(); // 状态机推进，由 EarthBlockInteract 调用

    bool PushCommand(const Command &command);
    bool PopCommand(Command &command);
    void LoadCommand(const Command &command, uint32_t now_us);
    void ExecuteQueuedAction();
    void ApplySuction(uint8_t cup_index, bool enable);
    bool IsActionTimeout(uint32_t now_us) const;
    void FinishAction();
    void SetFault();

    void StartMotorRailCalibration();
    bool UpdateMotorRailCalibration();
    bool SetMotorRailCurrentPositionAsZero();
    void UpdateMotorRailFeedback();
    void CalculateMotorRail();
    void StopMotorRailControl();
    bool IsMotorRailArrived() const;
};

inline EarthBlock::Status EarthBlock::GetStatus() const
{
    return status_;
}

inline EarthBlock::Action EarthBlock::GetLastAction() const
{
    return current_command_.action;
}

inline bool EarthBlock::IsInitialized() const
{
    return initialized_;
}

inline bool EarthBlock::IsBusy() const
{
    return status_ == Status::kQueued ||
           status_ == Status::kSuctionWait ||
           status_ == Status::kAirRailWait ||
           status_ == Status::kMotorRailMoving ||
           status_ == Status::kMotorRailCalibrating;
}

inline bool EarthBlock::IsActionFinished() const
{
    return status_ == Status::kFinished;
}

inline bool EarthBlock::IsMotorRailCalibrated() const
{
    return initialized_ && motor_rail_calibrated_;
}

inline float EarthBlock::GetMotorRailPositionMm() const
{
    return motor_rail_current_mm_;
}

inline float EarthBlock::GetMotorRailSpeedMmps() const
{
    return motor_rail_current_speed_mmps_;
}

inline float EarthBlock::GetMotorRailTargetMm() const
{
    return motor_rail_target_mm_;
}

inline float EarthBlock::GetMotorRailRawPosition() const
{
    return motor_->GetPosition();
}

inline Enum_AIRPUMP_Status EarthBlock::GetSuctionStatus(uint8_t cup_index)
{
    if (cup_index >= 2 || !suction_enabled_[cup_index])
    {
        return AIRPUMP_Status_None;
    }
    return suction_cup_[cup_index].Get_Status();
}

inline Enum_AIRPUMP_Status EarthBlock::GetAirRailStatus()
{
    return air_rail_.Get_Status();
}

} // namespace Module

#endif // MOD_EARTH_BLOCK_H
