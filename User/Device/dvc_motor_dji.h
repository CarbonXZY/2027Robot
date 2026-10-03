#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

#include <stdint.h>
#include <math.h>

#include "alg_pid.h"
#include "dvc_motor_base.h"
#include "drv_can.h"
#include "drv_math.h"

namespace Device
{
    /**
     * @brief DJI 电机CAN发送ID
     */
    enum class CanTxId
    {
        kTx0x200Only = 0,
        kTx0x1ffOnly,
        kTxBoth
    };

    /**
     * @brief DJI 电机 CAN 原始反馈数据
     */
    struct MotorDjiCanRxData
    {
        uint16_t encoder_reverse;
        int16_t omega_reverse;
        int16_t current_reverse;
        uint8_t temperature;
        uint8_t reserved;
    } __attribute__((packed));

    /**
     * @brief DJI 电机 CAN ID（C610/C620/GM6020 通用）
     */
    enum class MotorDjiId
    {
        kId0x201 = 0x201,
        kId0x202 = 0x202,
        kId0x203 = 0x203,
        kId0x204 = 0x204,
        kId0x205 = 0x205,
        kId0x206 = 0x206,
        kId0x207 = 0x207,
        kId0x208 = 0x208,
    };

    /**
     * @brief DJI C610 处理后的反馈数据
     */
    struct MotorDjiC610RxData
    {
        float now_angle = 0.0f;
        float now_omega = 0.0f;
        float now_current = 0.0f;
        float now_temperature = 0.0f;

        uint32_t pre_encoder = 0;
        int32_t total_encoder = 0;
        int32_t total_round = 0;
    };

    /**
     * @brief C620 功率控制使能状态
     */
    enum class MotorDjiC620PowerLimitStatus
    {
        kDisable = 0,
        kEnable,
    };

    /**
     * @brief DJI C620 处理后的反馈数据
     */
    struct MotorDjiC620RxData
    {
        float now_angle = 0.0f;
        float now_omega = 0.0f;
        float now_current = 0.0f;
        float now_temperature = 0.0f;
        float now_power = 0.0f;

        uint32_t pre_encoder = 0;
        int32_t total_encoder = 0;
        int32_t total_round = 0;
    };

    /**
     * @brief DJI C610 电机类，直接继承 MotorBase
     *
     * 设计目标：
     * 1. 只使用 MotorBase 的控制模式 MotorControlMethod
     * 2. 不再保留独立的控制模式枚举，避免两套模式同步
     * 3. CURRENT 模式：直接下发电流目标
     * 4. SPEED 模式：速度 PID 输出电流目标
     * 5. POSITION 模式：外部位置环输出速度目标，再由速度 PID 输出电流目标
     *
     * 舵轮舵向建议：
     * - POSITION 模式使用外部绝对编码器反馈
     * - 通过 SetFeedbackPosition(absolute_angle_rad) 输入舵向真实角度
     */

    uint8_t *AllocateTxData(
        FDCAN_HandleTypeDef *hfdcan,
        MotorDjiId fdcan_rx_id);

    void DjiTimSendGroup(FDCAN_HandleTypeDef *hfdcan, CanTxId can_tx_id);

    class MotorDjiC610 : public MotorBase
    {
    public:
        struct Parameters
        {
            Algorithm::PidParameters pid_position;

            Algorithm::PidParameters pid_omega;

            /*
             * true:
             *   POSITION 模式使用外部绝对编码器反馈 feedback_position_
             * false:
             *   POSITION 模式使用 C610 自身编码器反馈 rx_data_.now_angle
             */
            bool use_external_position_feedback = false;
        };


        Algorithm::Pid pid_omega;
        Algorithm::Pid pid_position;


        /**
         * @brief 硬件绑定, 只管底层外设绑定, 不负责电控逻辑
         *
         * @param hfdcan FDCAN 句柄
         * @param fdcan_rx_id 电机反馈 ID，0x201 ~ 0x208
         * @param gearbox_rate 减速比，舵向若无减速箱可设 1
         * @param current_max 最大输出电流，A
         */
        MotorDjiC610(
            FDCAN_HandleTypeDef *hfdcan,
            MotorDjiId fdcan_rx_id,
            float gearbox_rate = 36.0f,
            float current_max = 10.0f);

        // 禁止无参构造
        MotorDjiC610() = delete;

        // 禁止赋值构造
        MotorDjiC610& operator=(const MotorDjiC610&) = delete;

        /**
         * @brief 电机控制初始化
         * @param method 控制模式
         * @param parameters 控制参数
         */
        void Init(MotorControlMethod method, const Parameters &parameters);

        inline void SetControlMethod(MotorControlMethod method) override;

        inline void SetTargetCurrent(float target_current) override;
        inline void SetTargetSpeed(float target_speed) override;
        inline void SetTargetPosition(float target_position) override;

        inline void SetFeedbackCurrent(float feedback_current) override;
        inline void SetFeedbackSpeed(float feedback_speed) override;
        inline void SetFeedbackPosition(float feedback_position) override;

        inline float GetCurrent() const override;
        inline float GetSpeed() const override;
        inline float GetPosition() const override;

        /**
         * @brief 从 CAN 帧更新 MotorBase 统一反馈值
         */
        void UpdateFeedback() override;

        /**
         * @brief 控制周期主入口：反馈更新 → PID 计算 → 输出限幅
         */
        void Calculate() override;

        /**
         * @brief 将计算结果写入 CAN 发送缓冲区
         */
        void Output() override;

        /**
         * @brief CAN 接收回调
         *
         * 外部 FDCAN 分发时，在收到对应电机反馈后调用。
         */
        void FdcanRxCpltCallback(uint8_t *rx_data);

        /**
         * @brief 100ms 心跳检测定时器回调
         *
         * 丢帧时判定电机离线，清零状态和积分。
         */
        void Tim100msAlivePeriodElapsedCallback();

        bool IsInitialized() const;
        MotorStatus GetStatus() const;

        /* ===== DJI C610 原生反馈值 ===== */
        inline float GetNowAngle() const;
        inline float GetNowOmega() const;
        inline float GetNowCurrent() const;
        inline float GetNowtemperature() const;

        /* ===== 内部状态读取 ===== */
        inline float GetTargetCurrentInternal() const;
        inline float GetTargetOmegaInternal() const;
        inline float GetOutput() const;

        /**
         * @brief 速度前馈 — 叠加到速度环目标，LimitOutput() 中自动清零
         */
        inline void SetFeedforwardOmega(float feedforward_omega);
        inline void SetFeedforwardCurrent(float feedforward_current);

    protected:
        /*
         * CAN / DJI 底层相关
         */
        Driver::FdcanManageObject *fdcan_manage_object_ = nullptr;
        MotorDjiId fdcan_rx_id_ = MotorDjiId::kId0x201;
        uint8_t *tx_data_ = nullptr;

        float gearbox_rate_ = 36.0f;
        float current_max_ = 10.0f;

        uint16_t encoder_num_per_round_ = 8192;
        float current_to_out_ = 10000.0f / 10.0f;
        float theoretical_output_current_max_ = 10.0f;

        /*
         * 心跳检测：flag_ 在每次收到反馈时自增，
         * pre_flag_ 在 100ms 定时器中对比判断是否丢帧。
         */
        uint32_t flag_ = 0;
        uint32_t pre_flag_ = 0;

        /* DJI CAN 协议待发送的电流值（已转换为协议格式） */
        float out_ = 0.0f;

        MotorDjiC610RxData rx_data_;

        /*
         * MotorBase 统一控制层
         */
        Parameters param_;
        MotorStatus motor_status_ = MotorStatus::kDisable;
        MotorControlMethod control_method_ = MotorControlMethod::kCurrent;

        float target_current_ = 0.0f;  // A, 上层目标电流
        float target_speed_ = 0.0f;    // rad/s, 上层目标速度
        float target_position_ = 0.0f; // rad, 上层目标位置

        /* 电机接受到的反馈值，单位同 target_* 单位 */
        float feedback_current_ = 0.0f;  // A
        float feedback_speed_ = 0.0f;    // rad/s
        float feedback_position_ = 0.0f; // rad

        bool initialized_ = false;

        /* 前馈叠加量：由上层在 Calculate() 前写入，LimitOutput() 中清零 */
        float feedforward_speed_ = 0.0f;   // rad/s
        float feedforward_current_ = 0.0f; // A

    private:
        bool CheckParameters(const Parameters &parameters) const;

        void DataProcess();

        /**
         * @brief 使用 MotorBase 的 control_method_ 直接计算 C610 PID
         */
        void PidCalculate();

        void LimitOutput();
        void OutputCanData();

        /**
         * @brief 将角度归一化到 [-2π, 2π] 区间
         */
        static float NormalizeAngle(float angle);
        static bool IsFinite(float value);
    };

    /* ===== inline 实现：状态读取 ===== */

    inline MotorStatus MotorDjiC610::GetStatus() const
    {
        return motor_status_;
    }

    inline float MotorDjiC610::GetNowAngle() const
    {
        return rx_data_.now_angle;
    }

    inline float MotorDjiC610::GetNowOmega() const
    {
        return rx_data_.now_omega;
    }

    inline float MotorDjiC610::GetNowCurrent() const
    {
        return rx_data_.now_current;
    }

    inline float MotorDjiC610::GetNowtemperature() const
    {
        return rx_data_.now_temperature;
    }

    inline float MotorDjiC610::GetTargetCurrentInternal() const
    {
        return target_current_;
    }

    inline float MotorDjiC610::GetTargetOmegaInternal() const
    {
        return target_speed_;
    }

    inline float MotorDjiC610::GetOutput() const
    {
        return out_;
    }

    /* ===== inline 实现：前馈写入（含 NaN/Inf 防护） ===== */

    inline void MotorDjiC610::SetFeedforwardOmega(float feedforward_omega)
    {
        if (IsFinite(feedforward_omega))
        {
            feedforward_speed_ = feedforward_omega;
        }
    }

    inline void MotorDjiC610::SetFeedforwardCurrent(float feedforward_current)
    {
        if (IsFinite(feedforward_current))
        {
            feedforward_current_ = feedforward_current;
        }
    }

    /* ===== inline 实现：MotorBase 接口 ===== */

    inline void MotorDjiC610::SetControlMethod(MotorControlMethod method)
    {
        control_method_ = method;
    }

    inline void MotorDjiC610::SetTargetCurrent(float target_current)
    {
        if (IsFinite(target_current))
        {
            target_current_ = target_current;
        }
    }

    inline void MotorDjiC610::SetTargetSpeed(float target_speed)
    {
        if (IsFinite(target_speed))
        {
            /* C610 对外速度单位：rad/s */
            target_speed_ = target_speed;
        }
    }

    inline void MotorDjiC610::SetTargetPosition(float target_position)
    {
        if (IsFinite(target_position))
        {
            /* C610 对外位置单位：rad */
            target_position_ = target_position;
        }
    }

    inline void MotorDjiC610::SetFeedbackCurrent(float feedback_current)
    {
        if (IsFinite(feedback_current))
        {
            feedback_current_ = feedback_current;
        }
    }

    inline void MotorDjiC610::SetFeedbackSpeed(float feedback_speed)
    {
        if (IsFinite(feedback_speed))
        {
            feedback_speed_ = feedback_speed;
        }
    }

    inline void MotorDjiC610::SetFeedbackPosition(float feedback_position)
    {
        if (IsFinite(feedback_position))
        {
            /* 外部位置反馈写入前归一化到 [-π, π] */
            feedback_position_ = feedback_position;
        }
    }

    inline float MotorDjiC610::GetCurrent() const
    {
        return feedback_current_;
    }

    inline float MotorDjiC610::GetSpeed() const
    {
        return feedback_speed_;
    }

    inline float MotorDjiC610::GetPosition() const
    {
        return feedback_position_;
    }

    /**
     * @brief DJI C620 电机类，继承 MotorBase
     *
     * 与 C610 的主要区别：
     * 1. 默认减速比 3591/187（M3508 原装减速箱）
     * 2. 最大电流 20A，current_to_out_ = 16384/20
     * 3. 支持功率限制控制
     * 4. 反馈数据包含功率估计
     */
    class MotorDjiC620 : public MotorBase
    {
    public:
        struct Parameters
        {
            Algorithm::PidParameters pid_position;
            Algorithm::PidParameters pid_omega;

            bool use_external_position_feedback = false;

            MotorDjiC620PowerLimitStatus power_limit_status =
                MotorDjiC620PowerLimitStatus::kDisable;

            float power_k_0 = 0.2962f;
            float power_k_1 = 0.0000f;
            float power_k_2 = 0.1519f;
            float power_a = 1.3544f;
        };

        Algorithm::Pid pid_omega;
        Algorithm::Pid pid_position;

        /**
         * @brief 硬件绑定, 只管底层外设绑定, 不负责电控逻辑
         *
         * @param hfdcan FDCAN 句柄
         * @param fdcan_rx_id 电机反馈 ID，0x201 ~ 0x208
         * @param gearbox_rate 减速比
         * @param current_max 最大输出电流，A
         */
        MotorDjiC620(
            FDCAN_HandleTypeDef *hfdcan,
            MotorDjiId fdcan_rx_id,
            float gearbox_rate = 3591.0f / 187.0f,
            float current_max = 20.0f);

        MotorDjiC620() = delete;

        // 禁止赋值构造
        MotorDjiC620& operator=(const MotorDjiC620&) = delete;

        /**
         * @brief 电机控制初始化
         * @param method 控制模式
         * @param parameters 控制参数
         */
        void Init(MotorControlMethod method, const Parameters &parameters);

        inline void SetControlMethod(MotorControlMethod method) override;
        inline void SetTargetCurrent(float target_current) override;
        inline void SetTargetSpeed(float target_speed) override;
        inline void SetTargetPosition(float target_position) override;

        inline void SetFeedbackCurrent(float feedback_current) override;
        inline void SetFeedbackSpeed(float feedback_speed) override;
        inline void SetFeedbackPosition(float feedback_position) override;

        inline float GetCurrent() const override;
        inline float GetSpeed() const override;
        inline float GetPosition() const override;

        void UpdateFeedback() override;
        void Calculate() override;
        void Output() override;

        void FdcanRxCpltCallback(uint8_t *rx_data);
        void Tim100msAlivePeriodElapsedCallback();

        /**
         * @brief 功率限制后重新计算输出
         *
         * 在所有电机 PID 计算完成后调用，
         * 根据功率因数限制电流目标并重新输出。
         */
        void TimPowerLimitAfterCalculatePeriodElapsedCallback();

        bool IsInitialized() const;
        MotorStatus GetStatus() const;

        inline float GetNowAngle() const;
        inline float GetNowOmega() const;
        inline float GetNowCurrent() const;
        inline float GetNowTemperature() const;
        inline float GetNowPower() const;
        inline float GetPowerEstimate() const;

        inline float GetTargetPosition() const;
        inline float GetTargetOmega() const;
        inline float GetTargetCurrent() const;

        inline float GetOutput() const;

        inline void SetFeedforwardOmega(float feedforward_omega);
        inline void SetFeedforwardCurrent(float feedforward_current);

        inline void SetPowerFactor(float power_factor);
        inline void SetOut(float out);

    protected:
        Driver::FdcanManageObject *fdcan_manage_object_ = nullptr;
        MotorDjiId fdcan_rx_id_ = MotorDjiId::kId0x201;
        uint8_t *tx_data_ = nullptr;

        float gearbox_rate_ = 3591.0f / 187.0f;
        float current_max_ = 20.0f;

        uint16_t encoder_num_per_round_ = 8192;
        float current_to_out_ = 16384.0f / 20.0f;
        float theoretical_output_current_max_ = 20.0f;

        uint32_t flag_ = 0;
        uint32_t pre_flag_ = 0;

        float out_ = 0.0f;

        MotorDjiC620RxData rx_data_;

        Parameters param_;
        MotorStatus motor_status_ = MotorStatus::kDisable;
        MotorControlMethod control_method_ = MotorControlMethod::kCurrent;

        float target_current_ = 0.0f;
        float target_speed_ = 0.0f;
        float target_position_ = 0.0f;

        float feedback_current_ = 0.0f;
        float feedback_speed_ = 0.0f;
        float feedback_position_ = 0.0f;

        bool initialized_ = false;

        float feedforward_speed_ = 0.0f;
        float feedforward_current_ = 0.0f;

        float power_estimate_ = 0.0f;
        float power_factor_ = 1.0f;

    private:
        bool CheckParameters(const Parameters &parameters) const;

        void DataProcess();
        void PidCalculate();
        void LimitOutput();
        void OutputCanData();
        void PowerLimitControl();

        static float NormalizeAngle(float angle);
        static bool IsFinite(float value);
    };

    /* ===== C620 inline 实现：状态读取 ===== */

    inline MotorStatus MotorDjiC620::GetStatus() const
    {
        return motor_status_;
    }

    inline float MotorDjiC620::GetNowAngle() const
    {
        return rx_data_.now_angle;
    }

    inline float MotorDjiC620::GetNowOmega() const
    {
        return rx_data_.now_omega;
    }

    inline float MotorDjiC620::GetNowCurrent() const
    {
        return rx_data_.now_current;
    }

    inline float MotorDjiC620::GetNowTemperature() const
    {
        return rx_data_.now_temperature;
    }

    inline float MotorDjiC620::GetNowPower() const
    {
        return rx_data_.now_power;
    }

    inline float MotorDjiC620::GetPowerEstimate() const
    {
        return power_estimate_;
    }

    inline float MotorDjiC620::GetTargetPosition() const
    {
        return target_position_;
    }

    inline float MotorDjiC620::GetTargetOmega() const
    {
        return target_speed_;
    }

    inline float MotorDjiC620::GetTargetCurrent() const
    {
        return target_current_;
    }

    inline float MotorDjiC620::GetOutput() const
    {
        return out_;
    }

    /* ===== C620 inline 实现：前馈写入 ===== */

    inline void MotorDjiC620::SetFeedforwardOmega(float feedforward_omega)
    {
        if (IsFinite(feedforward_omega))
        {
            feedforward_speed_ = feedforward_omega;
        }
    }

    inline void MotorDjiC620::SetFeedforwardCurrent(float feedforward_current)
    {
        if (IsFinite(feedforward_current))
        {
            feedforward_current_ = feedforward_current;
        }
    }

    inline void MotorDjiC620::SetPowerFactor(float power_factor)
    {
        if (IsFinite(power_factor))
        {
            power_factor_ = power_factor;
        }
    }

    inline void MotorDjiC620::SetOut(float out)
    {
        if (IsFinite(out))
        {
            out_ = out;
        }
    }

    /* ===== C620 inline 实现：MotorBase 接口 ===== */

    inline void MotorDjiC620::SetControlMethod(MotorControlMethod method)
    {
        control_method_ = method;
    }

    inline void MotorDjiC620::SetTargetCurrent(float target_current)
    {
        if (IsFinite(target_current))
        {
            target_current_ = target_current;
        }
    }

    inline void MotorDjiC620::SetTargetSpeed(float target_speed)
    {
        if (IsFinite(target_speed))
        {
            target_speed_ = target_speed;
        }
    }

    inline void MotorDjiC620::SetTargetPosition(float target_position)
    {
        if (IsFinite(target_position))
        {
            target_position_ = target_position;
        }
    }

    inline void MotorDjiC620::SetFeedbackCurrent(float feedback_current)
    {
        if (IsFinite(feedback_current))
        {
            feedback_current_ = feedback_current;
        }
    }

    inline void MotorDjiC620::SetFeedbackSpeed(float feedback_speed)
    {
        if (IsFinite(feedback_speed))
        {
            feedback_speed_ = feedback_speed;
        }
    }

    inline void MotorDjiC620::SetFeedbackPosition(float feedback_position)
    {
        if (IsFinite(feedback_position))
        {
            feedback_position_ = feedback_position;
        }
    }

    inline float MotorDjiC620::GetCurrent() const
    {
        return feedback_current_;
    }

    inline float MotorDjiC620::GetSpeed() const
    {
        return feedback_speed_;
    }

    inline float MotorDjiC620::GetPosition() const
    {
        return feedback_position_;
    }
}
#endif
