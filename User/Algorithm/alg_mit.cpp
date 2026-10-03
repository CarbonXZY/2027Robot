/**
 * @file    alg_mit.cpp
 * @author  lyh
 * @date    2026-10-02
 * @brief   
 */
#include "alg_mit.h"

namespace Algorithm
{
//

float Mit::Calculate()
{
    t_out_ = kp_ * (x_des_ - x_) + kd_ * (v_des_ - v_) + t_ff_;
    return t_out_;
}

//
} // namespace Algorithm
