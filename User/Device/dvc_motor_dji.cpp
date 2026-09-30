#include "dvc_motor_dji.h"

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
}

namespace Device
{

/**
 * @brief 构造函数 — 带控制参数版本
 *
 * 工作流程：
 * 1. 校验参数合法性（gearbox_rate > 0、current_max > 0、PID 参数有限）
 * 2. 绑定 FDCAN 句柄到对应的 FdcanManageObject
 * 3. 从 FDCAN 共享发送缓冲区分配 2 字节 tx_data_
 * 4. 清零所有状态和目标值
 *
 * 构造函数在静态初始化期运行，因此句柄只做指针比较，不解引用。
 */
MotorDjiC610::MotorDjiC610(
    FDCAN_HandleTypeDef *hfdcan,
    MotorDjiId fdcan_rx_id,
    const Parameters &parameters,
    float gearbox_rate,
    float current_max)
{
    initialized_ = false;

    fdcan_manage_object_ = nullptr;
    tx_data_ = nullptr;

    if (hfdcan == nullptr)
    {
        return;
    }

    if (!IsFinite(gearbox_rate) ||
        !IsFinite(current_max) ||
        gearbox_rate <= 0.0f ||
        current_max <= 0.0f)
    {
        return;
    }

    if (!CheckParameters(parameters))
    {
        return;
    }

    if (hfdcan == &hfdcan1)
    {
        fdcan_manage_object_ = &Driver::g_fdcan1_manage_object;
    }
    else if (hfdcan == &hfdcan2)
    {
        fdcan_manage_object_ = &Driver::g_fdcan2_manage_object;
    }
    else if (hfdcan == &hfdcan3)
    {
        fdcan_manage_object_ = &Driver::g_fdcan3_manage_object;
    }
    else
    {
        return;
    }

    tx_data_ = AllocateTxData(hfdcan, fdcan_rx_id);

    if (tx_data_ == nullptr)
    {
        return;
    }

    fdcan_rx_id_ = fdcan_rx_id;
    gearbox_rate_ = gearbox_rate;
    current_max_ = current_max;
    param_ = parameters;

    control_method_ = MotorControlMethod::kCurrent;

    target_current_ = 0.0f;
    target_speed_ = 0.0f;
    target_position_ = 0.0f;

    feedback_current_ = 0.0f;
    feedback_speed_ = 0.0f;
    feedback_position_ = 0.0f;

    feedforward_speed_ = 0.0f;
    feedforward_current_ = 0.0f;

    out_ = 0.0f;

    flag_ = 0;
    pre_flag_ = 0;
    motor_status_ = MotorStatus::kDisable;
    rx_data_ = {};

    // PID 初始化
    pid_position.Init(parameters.pid_position.k_p,
                      parameters.pid_position.k_i,
                      parameters.pid_position.k_d,
                      parameters.pid_position.k_f,
                      parameters.pid_position.i_out_max,
                      parameters.pid_position.out_max,
                      parameters.pid_position.d_t,
                      parameters.pid_position.dead_zone,
                      parameters.pid_position.i_variable_speed_a,
                      parameters.pid_position.i_variable_speed_b,
                      parameters.pid_position.i_separate_threshold,
                      parameters.pid_position.d_first);

    pid_omega.Init(parameters.pid_omega.k_p,
                   parameters.pid_omega.k_i,
                   parameters.pid_omega.k_d,
                   parameters.pid_omega.k_f,
                   parameters.pid_omega.i_out_max,
                   parameters.pid_omega.out_max,
                   parameters.pid_omega.d_t,
                   parameters.pid_omega.dead_zone,
                   parameters.pid_omega.i_variable_speed_a,
                   parameters.pid_omega.i_variable_speed_b,
                   parameters.pid_omega.i_separate_threshold,
                   parameters.pid_omega.d_first);

    initialized_ = true;
}

/**
 * @brief 校验控制参数合法性
 *
 * 检查项：PID 参数必须为有限值，积分限幅非负，输出限幅为正。
 */
bool MotorDjiC610::CheckParameters(const Parameters &parameters) const
{
    if (!IsFinite(parameters.pid_position.k_p) ||
        !IsFinite(parameters.pid_position.k_i) ||
        !IsFinite(parameters.pid_position.k_d) ||
        !IsFinite(parameters.pid_position.i_out_max) ||
        !IsFinite(parameters.pid_position.out_max))
    {
        return false;
    }

    if (parameters.pid_position.i_out_max < 0.0f ||
        parameters.pid_position.out_max <= 0.0f)
    {
        return false;
    }

    return true;
}

/**
 * @brief 为指定 CAN 接口和电机 ID 分配 tx_data_ 缓冲区
 *
 * DJI C610 每个电机占用 2 字节（电流目标高/低各 1 字节），
 * 同一 ID 组（0x200 / 0x1FF）的 4 个电机共享一个 CAN 帧。
 */
uint8_t *AllocateTxData(
    FDCAN_HandleTypeDef *hfdcan,
    MotorDjiId fdcan_rx_id)
{
    if (hfdcan == nullptr)
    {
        return nullptr;
    }

    if (hfdcan == &hfdcan1)
    {
        switch (fdcan_rx_id)
        {
        case MotorDjiId::kId0x201: return &Driver::g_fdcan1_0x200_tx_data[0];
        case MotorDjiId::kId0x202: return &Driver::g_fdcan1_0x200_tx_data[2];
        case MotorDjiId::kId0x203: return &Driver::g_fdcan1_0x200_tx_data[4];
        case MotorDjiId::kId0x204: return &Driver::g_fdcan1_0x200_tx_data[6];

        case MotorDjiId::kId0x205: return &Driver::g_fdcan1_0x1ff_tx_data[0];
        case MotorDjiId::kId0x206: return &Driver::g_fdcan1_0x1ff_tx_data[2];
        case MotorDjiId::kId0x207: return &Driver::g_fdcan1_0x1ff_tx_data[4];
        case MotorDjiId::kId0x208: return &Driver::g_fdcan1_0x1ff_tx_data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan2)
    {
        switch (fdcan_rx_id)
        {
        case MotorDjiId::kId0x201: return &Driver::g_fdcan2_0x200_tx_data[0];
        case MotorDjiId::kId0x202: return &Driver::g_fdcan2_0x200_tx_data[2];
        case MotorDjiId::kId0x203: return &Driver::g_fdcan2_0x200_tx_data[4];
        case MotorDjiId::kId0x204: return &Driver::g_fdcan2_0x200_tx_data[6];

        case MotorDjiId::kId0x205: return &Driver::g_fdcan2_0x1ff_tx_data[0];
        case MotorDjiId::kId0x206: return &Driver::g_fdcan2_0x1ff_tx_data[2];
        case MotorDjiId::kId0x207: return &Driver::g_fdcan2_0x1ff_tx_data[4];
        case MotorDjiId::kId0x208: return &Driver::g_fdcan2_0x1ff_tx_data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan3)
    {
        switch (fdcan_rx_id)
        {
        case MotorDjiId::kId0x201: return &Driver::g_fdcan3_0x200_tx_data[0];
        case MotorDjiId::kId0x202: return &Driver::g_fdcan3_0x200_tx_data[2];
        case MotorDjiId::kId0x203: return &Driver::g_fdcan3_0x200_tx_data[4];
        case MotorDjiId::kId0x204: return &Driver::g_fdcan3_0x200_tx_data[6];

        case MotorDjiId::kId0x205: return &Driver::g_fdcan3_0x1ff_tx_data[0];
        case MotorDjiId::kId0x206: return &Driver::g_fdcan3_0x1ff_tx_data[2];
        case MotorDjiId::kId0x207: return &Driver::g_fdcan3_0x1ff_tx_data[4];
        case MotorDjiId::kId0x208: return &Driver::g_fdcan3_0x1ff_tx_data[6];

        default: return nullptr;
        }
    }

    return nullptr;
}



/**
 * @brief 更新反馈值到 MotorBase 统一接口
 *
 * - 电流和速度始终从 C610 内部编码器反馈读取
 * - 位置根据 use_external_position_feedback 决定来自 C610 还是外部绝对编码器
 */
void MotorDjiC610::UpdateFeedback()
{
    if (!initialized_)
    {
        return;
    }

    feedback_current_ = rx_data_.now_current;
    feedback_speed_ = rx_data_.now_omega;

    /*
     * 舵轮建议使用外置绝对编码器位置反馈。
     * 如果 use_external_position_feedback = true，
     * 则 feedback_position_ 由 SetFeedbackPosition() 写入，不在这里覆盖。
     */
    if (!param_.use_external_position_feedback)
    {
        feedback_position_ = rx_data_.now_angle;
    }
}

/**
 * @brief 控制周期主入口：反馈更新 → PID 计算 → 输出限幅
 */
void MotorDjiC610::Calculate()
{
    if (!initialized_)
    {
        return;
    }

    UpdateFeedback();

    PidCalculate();

    LimitOutput();

    Output();
}

/**
 * @brief 根据控制模式执行对应 PID 计算
 *
 * 三种控制模式：
 * - CURRENT：直接使用上层电流目标（+ 前馈电流），不经 PID
 * - SPEED：速度 PID 输出电流目标（+ 速度前馈）
 * - POSITION：位置外环 → 速度目标 → 速度内环 → 电流目标（双环串级 PID）
 *
 * 前馈叠加：
 * - feedforward_speed_ 叠加到速度环目标
 * - feedforward_current_ 叠加到最终电流输出
 */
void MotorDjiC610::PidCalculate()
{
    switch (control_method_)
    {
    case MotorControlMethod::kCurrent:
    {
        /*
         * C610 电调内部有电流环，这里直接使用上层电流目标。
         */
        target_current_ = target_current_ + feedforward_current_;
        target_speed_ = 0.0f;
        break;
    }

    case MotorControlMethod::kSpeed:
    {
        /*
         * 速度 PID 输出电流目标。
         */
        feedback_speed_ = rx_data_.now_omega;
        pid_omega.SetTarget(target_speed_ + feedforward_speed_);
        pid_omega.SetNow(feedback_speed_);
        pid_omega.TimCalculatePeriodElapsedCallback();

        target_current_ = pid_omega.GetOut();
        break;
    }

    case MotorControlMethod::kPosition:
    {
        /*
         * 舵向位置模式：
         * 位置外环使用 feedback_position_。
         * 若 use_external_position_feedback = true，
         * 该值来自外置绝对编码器。
         */
        if (!param_.use_external_position_feedback)
        {
            feedback_position_ = rx_data_.now_angle;
        }
        feedback_speed_ = rx_data_.now_omega;



        pid_position.SetTarget(target_position_);
        pid_position.SetNow(feedback_position_);
        pid_position.TimCalculatePeriodElapsedCallback();

        target_speed_ = pid_position.GetOut();

        pid_omega.SetTarget(target_speed_ + feedforward_speed_);
        pid_omega.SetNow(feedback_speed_);
        pid_omega.TimCalculatePeriodElapsedCallback();

        target_current_ = pid_omega.GetOut();
        break;
    }

    case MotorControlMethod::kDisable:
    default:
    {
        target_current_ = 0.0f;
        target_speed_ = 0.0f;
        break;
    }
    }
}

/**
 * @brief 输出限幅与单位转换
 *
 * CURRENT 模式下 PidCalculate() 已叠加 feedforward_current_，
 * 此处再次叠加确保所有模式最终都能接受前馈。
 * 限幅后清零前馈值，避免重复叠加。
 */
void MotorDjiC610::LimitOutput()
{
    float tmp_value = target_current_ + feedforward_current_;

    Driver::MathConstrain(&tmp_value, -current_max_, current_max_);

    out_ = tmp_value * current_to_out_;

    feedforward_current_ = 0.0f;
    feedforward_speed_ = 0.0f;
}

void MotorDjiC610::Output()
{
    if (!initialized_)
    {
        return;
    }

    OutputCanData();
}

/**
 * @brief 写入 CAN 发送缓冲区（2 字节，大端字节序）
 *
 * DJI 协议：tx_data_[0] = 高字节，tx_data_[1] = 低字节
 * 实际 CAN 帧由 FDCAN 定期发送任务统一打包发出。
 */
void MotorDjiC610::OutputCanData()
{
    if (tx_data_ == nullptr)
    {
        return;
    }

    tx_data_[0] = (int16_t)out_ >> 8;
    tx_data_[1] = (int16_t)out_;

}

/**
 * @brief FDCAN 接收回调 — 更新电机状态标志和反馈数据
 *
 * 每次收到反馈帧后 flag_ 自增，用于后续心跳检测。
 * rx_data 参数未使用，直接从 fdcan_manage_object_ 读取最新帧。
 */
void MotorDjiC610::FdcanRxCpltCallback(uint8_t *rx_data)
{
    (void)rx_data;

    if (!initialized_)
    {
        return;
    }

    flag_ += 1;

    DataProcess();
}

/**
 * @brief 100ms 心跳检测定时器回调
 *
 * 如果连续 100ms 内 flag_ 未变化（未收到新反馈帧），
 * 判定电机离线：禁用状态、清零 PID 积分、清零输出。
 * 在线时重置 PID 积分可防止恢复连接后积分突跳。
 */
void MotorDjiC610::Tim100msAlivePeriodElapsedCallback()
{
    if (!initialized_)
    {
        return;
    }

    if (flag_ == pre_flag_)
    {
        motor_status_ = MotorStatus::kDisable;

        pid_position.SetIntegralError(0.0f);
        pid_omega.SetIntegralError(0.0f);

        target_current_ = 0.0f;
        target_speed_ = 0.0f;
        out_ = 0.0f;
    }
    else
    {
        motor_status_ = MotorStatus::kEnable;
    }

    pre_flag_ = flag_;
}

/**
 * @brief 解析 CAN 原始反馈数据
 *
 * 处理流程：
 * 1. 字节序反转（DJI 为大端）
 * 2. 多圈追踪：通过相邻帧编码器差值判断是否过零，更新 total_round
 * 3. 计算减速箱输出端角度：total_encoder / 分辨率 * 2π / 减速比
 * 4. 计算角速度：RPM → rad/s / 减速比
 * 5. 计算电流和温度
 *
 * DJI C610 编码器分辨率：8192 线/圈（13 bit 绝对值编码器）
 */
void MotorDjiC610::DataProcess()
{
    if (fdcan_manage_object_ == nullptr)
    {
        return;
    }

    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega;
    int16_t tmp_current;

    MotorDjiCanRxData *tmp_buffer =
        (MotorDjiCanRxData *)fdcan_manage_object_->rx_buffer_.Data;

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->encoder_reverse,
        (void *)&tmp_encoder
    );

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->omega_reverse,
        (void *)&tmp_omega
    );

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->current_reverse,
        (void *)&tmp_current
    );

    delta_encoder = tmp_encoder - rx_data_.pre_encoder;

    if (delta_encoder < -(int16_t)(encoder_num_per_round_ / 2))
    {
        rx_data_.total_round++;
    }
    else if (delta_encoder > (int16_t)(encoder_num_per_round_ / 2))
    {
        rx_data_.total_round--;
    }

    rx_data_.total_encoder =
        rx_data_.total_round * encoder_num_per_round_ + tmp_encoder;

    rx_data_.now_angle =
        (float)rx_data_.total_encoder /
        (float)encoder_num_per_round_ *
        kTwoPi /
        gearbox_rate_;

    rx_data_.now_omega =
        (float)tmp_omega *
        Driver::kRpmToRadps /
        gearbox_rate_;

    rx_data_.now_current =
        (float)tmp_current /
        current_to_out_;

    rx_data_.now_temperature =
        (float)tmp_buffer->temperature + Driver::kCelsiusToKelvin;

    rx_data_.pre_encoder = tmp_encoder;
}



bool MotorDjiC610::IsInitialized() const
{
    return initialized_;
}



float MotorDjiC610::NormalizeAngle(float angle)
{
    if (!IsFinite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, kTwoPi);

    if (angle > kPi)
    {
        angle -= kTwoPi;
    }
    else if (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}


/**
 * @brief 判断输入值是否为有限数
 *
 * @param value 输入值
 * @return true 输入值为有限数
 * @return false 输入值是无限数
 */
bool MotorDjiC610::IsFinite(float value)
{
    return isfinite(value);
}
/**
 * @brief 大疆电机统一发送接口
 * C610 C620
 * @return
 */
void DjiTimSendGroup(FDCAN_HandleTypeDef *hfdcan, CanTxId can_tx_id)
{
    if (hfdcan == nullptr)
    {
        return;
    }

    // C610 C620共用 0x200 和 0x1FF 两组 ID，分别对应 Driver::g_fdcan1_0x200_tx_data 和 Driver::g_fdcan1_0x1ff_tx_data 共享缓冲区
    // 0x201 ~ 0x204 使用 0x200 组，0x205 ~ 0x208 使用 0x1FF 组
    // DJI电调使用标准帧 ID，数据长度固定为 8 字节，每个电机占用 2 字节（高字节 + 低字节）
    uint8_t *tx_data = nullptr;
    switch (can_tx_id)
    {
    case CanTxId::kTx0x200Only:
    {
        //绑定0x201~0x204的电机ID到0x200组发送缓冲区
        tx_data = AllocateTxData(hfdcan, MotorDjiId::kId0x201);
        if (tx_data != nullptr)
        Driver::FdcanSendData(hfdcan, 0x200, tx_data, Driver::FdcanIdType::kStandard);
        break;
    }

    case CanTxId::kTx0x1ffOnly:
    {
        //绑定0x205~0x208的电机ID到0x1FF组发送缓冲区
        tx_data = AllocateTxData(hfdcan, MotorDjiId::kId0x205);
        if (tx_data != nullptr)
        Driver::FdcanSendData(hfdcan, 0x1FF, tx_data, Driver::FdcanIdType::kStandard);
        break;
    }

    case CanTxId::kTxBoth:
    {
        //绑定0x201~0x204和0x205~0x208的电机ID到0x200组和0x1FF组发送缓冲区
        tx_data = AllocateTxData(hfdcan, MotorDjiId::kId0x201);

        if (tx_data != nullptr)
        Driver::FdcanSendData(hfdcan, 0x200, tx_data, Driver::FdcanIdType::kStandard);

        tx_data = AllocateTxData(hfdcan, MotorDjiId::kId0x205);

        if (tx_data != nullptr)
        Driver::FdcanSendData(hfdcan, 0x1FF, tx_data, Driver::FdcanIdType::kStandard);
        break;
    }

    default:
        break;
    }
}

/* ====================================================================== */
/*  MotorDjiC620 实现                                                     */
/* ====================================================================== */

namespace
{
    float PowerCalculateC620(float k_0, float k_1, float k_2, float a, float current, float omega)
    {
        return (k_0 * current * omega + k_1 * omega * omega + k_2 * current * current + a);
    }
}

MotorDjiC620::MotorDjiC620(
    FDCAN_HandleTypeDef *hfdcan,
    MotorDjiId fdcan_rx_id,
    const Parameters &parameters,
    float gearbox_rate,
    float current_max)
{
    initialized_ = false;

    fdcan_manage_object_ = nullptr;
    tx_data_ = nullptr;

    if (hfdcan == nullptr)
    {
        return;
    }

    if (!IsFinite(gearbox_rate) ||
        !IsFinite(current_max) ||
        gearbox_rate <= 0.0f ||
        current_max <= 0.0f)
    {
        return;
    }

    if (!CheckParameters(parameters))
    {
        return;
    }

    if (hfdcan == &hfdcan1)
    {
        fdcan_manage_object_ = &Driver::g_fdcan1_manage_object;
    }
    else if (hfdcan == &hfdcan2)
    {
        fdcan_manage_object_ = &Driver::g_fdcan2_manage_object;
    }
    else if (hfdcan == &hfdcan3)
    {
        fdcan_manage_object_ = &Driver::g_fdcan3_manage_object;
    }
    else
    {
        return;
    }

    tx_data_ = AllocateTxData(hfdcan, fdcan_rx_id);

    if (tx_data_ == nullptr)
    {
        return;
    }

    fdcan_rx_id_ = fdcan_rx_id;
    gearbox_rate_ = gearbox_rate;
    current_max_ = current_max;
    param_ = parameters;

    control_method_ = MotorControlMethod::kCurrent;

    target_current_ = 0.0f;
    target_speed_ = 0.0f;
    target_position_ = 0.0f;

    feedback_current_ = 0.0f;
    feedback_speed_ = 0.0f;
    feedback_position_ = 0.0f;

    feedforward_speed_ = 0.0f;
    feedforward_current_ = 0.0f;

    out_ = 0.0f;
    power_estimate_ = 0.0f;
    power_factor_ = 1.0f;

    flag_ = 0;
    pre_flag_ = 0;
    motor_status_ = MotorStatus::kDisable;
    rx_data_ = {};

    pid_position.Init(parameters.pid_position.k_p,
                      parameters.pid_position.k_i,
                      parameters.pid_position.k_d,
                      parameters.pid_position.k_f,
                      parameters.pid_position.i_out_max,
                      parameters.pid_position.out_max,
                      parameters.pid_position.d_t,
                      parameters.pid_position.dead_zone,
                      parameters.pid_position.i_variable_speed_a,
                      parameters.pid_position.i_variable_speed_b,
                      parameters.pid_position.i_separate_threshold,
                      parameters.pid_position.d_first);

    pid_omega.Init(parameters.pid_omega.k_p,
                   parameters.pid_omega.k_i,
                   parameters.pid_omega.k_d,
                   parameters.pid_omega.k_f,
                   parameters.pid_omega.i_out_max,
                   parameters.pid_omega.out_max,
                   parameters.pid_omega.d_t,
                   parameters.pid_omega.dead_zone,
                   parameters.pid_omega.i_variable_speed_a,
                   parameters.pid_omega.i_variable_speed_b,
                   parameters.pid_omega.i_separate_threshold,
                   parameters.pid_omega.d_first);

    initialized_ = true;
}

bool MotorDjiC620::CheckParameters(const Parameters &parameters) const
{
    if (!IsFinite(parameters.pid_position.k_p) ||
        !IsFinite(parameters.pid_position.k_i) ||
        !IsFinite(parameters.pid_position.k_d) ||
        !IsFinite(parameters.pid_position.i_out_max) ||
        !IsFinite(parameters.pid_position.out_max))
    {
        return false;
    }

    if (parameters.pid_position.i_out_max < 0.0f ||
        parameters.pid_position.out_max <= 0.0f)
    {
        return false;
    }

    if (!IsFinite(parameters.power_k_0) ||
        !IsFinite(parameters.power_k_1) ||
        !IsFinite(parameters.power_k_2) ||
        !IsFinite(parameters.power_a))
    {
        return false;
    }

    return true;
}

void MotorDjiC620::UpdateFeedback()
{
    if (!initialized_)
    {
        return;
    }

    feedback_current_ = rx_data_.now_current;
    feedback_speed_ = rx_data_.now_omega;

    if (!param_.use_external_position_feedback)
    {
        feedback_position_ = rx_data_.now_angle;
    }
}

void MotorDjiC620::Calculate()
{
    if (!initialized_)
    {
        return;
    }

    UpdateFeedback();

    PidCalculate();

    LimitOutput();

    Output();

    if (param_.power_limit_status == MotorDjiC620PowerLimitStatus::kDisable)
    {
        feedforward_current_ = 0.0f;
        feedforward_speed_ = 0.0f;
    }
}

void MotorDjiC620::PidCalculate()
{
    switch (control_method_)
    {
    case MotorControlMethod::kCurrent:
    {
        target_current_ = target_current_ + feedforward_current_;
        target_speed_ = 0.0f;
        break;
    }

    case MotorControlMethod::kSpeed:
    {
        pid_omega.SetTarget(target_speed_ + feedforward_speed_);
        pid_omega.SetNow(rx_data_.now_omega);
        pid_omega.TimCalculatePeriodElapsedCallback();

        target_current_ = pid_omega.GetOut();
        break;
    }

    case MotorControlMethod::kPosition:
    {
        if (!param_.use_external_position_feedback)
        {
            feedback_position_ = rx_data_.now_angle;
        }
        feedback_speed_ = rx_data_.now_omega;

        pid_position.SetTarget(target_position_);
        pid_position.SetNow(feedback_position_);
        pid_position.TimCalculatePeriodElapsedCallback();

        target_speed_ = pid_position.GetOut();

        pid_omega.SetTarget(target_speed_ + feedforward_speed_);
        pid_omega.SetNow(feedback_speed_);
        pid_omega.TimCalculatePeriodElapsedCallback();

        target_current_ = pid_omega.GetOut();
        break;
    }

    case MotorControlMethod::kDisable:
    default:
    {
        target_current_ = 0.0f;
        target_speed_ = 0.0f;
        break;
    }
    }
}

void MotorDjiC620::LimitOutput()
{
    float tmp_value = target_current_ + feedforward_current_;

    Driver::MathConstrain(&tmp_value, -current_max_, current_max_);

    out_ = tmp_value * current_to_out_;
}

void MotorDjiC620::Output()
{
    if (!initialized_)
    {
        return;
    }

    OutputCanData();
}

void MotorDjiC620::OutputCanData()
{
    if (tx_data_ == nullptr)
    {
        return;
    }

    tx_data_[0] = (int16_t)out_ >> 8;
    tx_data_[1] = (int16_t)out_;
}

void MotorDjiC620::FdcanRxCpltCallback(uint8_t *rx_data)
{
    (void)rx_data;

    if (!initialized_)
    {
        return;
    }

    flag_ += 1;

    DataProcess();
}

void MotorDjiC620::Tim100msAlivePeriodElapsedCallback()
{
    if (!initialized_)
    {
        return;
    }

    if (flag_ == pre_flag_)
    {
        motor_status_ = MotorStatus::kDisable;

        pid_position.SetIntegralError(0.0f);
        pid_omega.SetIntegralError(0.0f);

        target_current_ = 0.0f;
        target_speed_ = 0.0f;
        out_ = 0.0f;
    }
    else
    {
        motor_status_ = MotorStatus::kEnable;
    }

    pre_flag_ = flag_;
}

void MotorDjiC620::DataProcess()
{
    if (fdcan_manage_object_ == nullptr)
    {
        return;
    }

    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega;
    int16_t tmp_current;

    MotorDjiCanRxData *tmp_buffer =
        (MotorDjiCanRxData *)fdcan_manage_object_->rx_buffer_.Data;

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->encoder_reverse,
        (void *)&tmp_encoder
    );

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->omega_reverse,
        (void *)&tmp_omega
    );

    Driver::MathEndianReverse16(
        (void *)&tmp_buffer->current_reverse,
        (void *)&tmp_current
    );

    delta_encoder = tmp_encoder - rx_data_.pre_encoder;

    if (delta_encoder < -(int16_t)(encoder_num_per_round_ / 2))
    {
        rx_data_.total_round++;
    }
    else if (delta_encoder > (int16_t)(encoder_num_per_round_ / 2))
    {
        rx_data_.total_round--;
    }

    rx_data_.total_encoder =
        rx_data_.total_round * encoder_num_per_round_ + tmp_encoder;

    rx_data_.now_angle =
        (float)rx_data_.total_encoder /
        (float)encoder_num_per_round_ *
        kTwoPi /
        gearbox_rate_;

    rx_data_.now_omega =
        (float)tmp_omega *
        Driver::kRpmToRadps /
        gearbox_rate_;

    rx_data_.now_current =
        (float)tmp_current /
        current_to_out_;

    rx_data_.now_temperature =
        (float)tmp_buffer->temperature + Driver::kCelsiusToKelvin;

    rx_data_.now_power = PowerCalculateC620(
        param_.power_k_0, param_.power_k_1, param_.power_k_2, param_.power_a,
        rx_data_.now_current, rx_data_.now_omega);

    rx_data_.pre_encoder = tmp_encoder;
}

void MotorDjiC620::TimPowerLimitAfterCalculatePeriodElapsedCallback()
{
    if (!initialized_)
    {
        return;
    }

    if (param_.power_limit_status == MotorDjiC620PowerLimitStatus::kEnable)
    {
        PowerLimitControl();
    }

    Driver::MathConstrain(&target_current_, -current_max_, current_max_);
    out_ = target_current_ * current_to_out_;

    Output();

    feedforward_current_ = 0.0f;
    feedforward_speed_ = 0.0f;
}

void MotorDjiC620::PowerLimitControl()
{
    power_estimate_ = PowerCalculateC620(
        param_.power_k_0, param_.power_k_1, param_.power_k_2, param_.power_a,
        target_current_, rx_data_.now_omega);

    if (power_estimate_ > 0.0f)
    {
        if (power_factor_ >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            float a = param_.power_k_2;
            float b = param_.power_k_0 * rx_data_.now_omega;
            float c = param_.power_a + param_.power_k_1 * rx_data_.now_omega * rx_data_.now_omega - power_factor_ * power_estimate_;
            float delta = b * b - 4.0f * a * c;

            if (delta < 0.0f)
            {
                target_current_ = 0.0f;
            }
            else
            {
                float h = sqrtf(delta);
                float result_1 = (-b + h) / (2.0f * a);
                float result_2 = (-b - h) / (2.0f * a);

                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((target_current_ > 0.0f && result_1 > 0.0f) || (target_current_ < 0.0f && result_1 < 0.0f))
                    {
                        target_current_ = result_1;
                    }
                    else
                    {
                        target_current_ = result_2;
                    }
                }
                else
                {
                    if (Driver::MathAbs(result_1) < Driver::MathAbs(result_2))
                    {
                        target_current_ = result_1;
                    }
                    else
                    {
                        target_current_ = result_2;
                    }
                }
            }
        }
    }
}

bool MotorDjiC620::IsInitialized() const
{
    return initialized_;
}

float MotorDjiC620::NormalizeAngle(float angle)
{
    if (!IsFinite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, kTwoPi);

    if (angle > kPi)
    {
        angle -= kTwoPi;
    }
    else if (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}

bool MotorDjiC620::IsFinite(float value)
{
    return isfinite(value);
}

} // namespace Device
