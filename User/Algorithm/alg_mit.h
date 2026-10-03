/**
 * @file    alg_mit.h
 * @author  lyh
 * @date    2026-10-02
 * @brief   MIT控制, 即弹簧阻尼模型
 */
#ifndef ALG_MIT_H
#define ALG_MIT_H

namespace Algorithm
{
/**
 * MIT控制模型
 *      t_out = kp * (x_des - x) + kd * (v_des -v) + t_ff
 *      kp:     弹性系数
 *      kd:     阻尼系数
 *      x_des:  目标位置
 *      x:      当前位置
 *      v_des:  目标速度
 *      v:      当前速度
 *      t_ff:   前馈补偿
 */
class Mit
{
public:

    void Init(float kp, float kd) { kp_ = kp; kd_ = kd; }

    void SetKp(float kp) { kp_ = kp; }
    void SetKd(float kd) { kd_ = kd; }
    void SetXdes(float x_des) { x_des_ = x_des; }
    void SetVdes(float v_des) { v_des_ = v_des; }
    void SetX(float x) { x_ = x; }
    void SetV(float v) { v_ = v; }
    void SetTff(float t_ff) { t_ff_ = t_ff; }

    float GetKp() const { return kp_; }
    float GetKd() const { return kd_; }
    float GetXdes() const { return x_des_; }
    float GetVdes() const { return v_des_; }
    float GetX() const { return x_; }
    float GetV() const { return v_; }
    float GetTff() const { return t_ff_; }

    float GetOutput() const { return t_out_; }

    float Calculate();

protected:
    // 控制参数
    float kp_ = 0.0f;
    float kd_ = 0.0f;
    // 目标值
    float x_des_ = 0.0f;
    float v_des_ = 0.0f;
    // 当前值
    float x_ = 0.0f;
    float v_ = 0.0f;
    // 前馈值
    float t_ff_ = 0.0f;
    // 输出值
    float t_out_ = 0.0f;

};

}

#endif // ALG_MIT_H
