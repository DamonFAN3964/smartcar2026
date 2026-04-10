/*******************************************************************************
 * pid.cpp — PID 控制器实现
 ******************************************************************************/
#include "pid.h"
#include <cmath>

static inline float fclamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(PIDController *pid,
              float kp, float ki, float kd,
              float i_limit, float out_limit,
              bool incremental)
{
    pid->kp              = kp;
    pid->ki              = ki;
    pid->kd              = kd;
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev2_error     = 0.0f;
    pid->integral_limit  = i_limit;
    pid->output_limit    = out_limit;
    pid->use_incremental = incremental;
    pid->output          = 0.0f;
}

float pid_compute(PIDController *pid, float error)
{
    float out = 0.0f;

    if (!pid->use_incremental) {
        // ────── 位置式 PID ──────
        // 积分累加
        pid->integral += error;
        // 积分限幅（抗积分饱和）
        pid->integral = fclamp(pid->integral,
                               -pid->integral_limit,
                                pid->integral_limit);

        float deriv = error - pid->prev_error;

        out = pid->kp * error
            + pid->ki * pid->integral
            + pid->kd * deriv;

    } else {
        // ────── 增量式 PID ──────
        // Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
        float delta = pid->kp * (error - pid->prev_error)
                    + pid->ki * error
                    + pid->kd * (error - 2.0f * pid->prev_error
                                       +        pid->prev2_error);
        pid->output += delta;
        pid->output  = fclamp(pid->output,
                              -pid->output_limit,
                               pid->output_limit);
        out = pid->output;
    }

    pid->prev2_error = pid->prev_error;
    pid->prev_error  = error;

    // 输出限幅
    out = fclamp(out, -pid->output_limit, pid->output_limit);
    return out;
}

void pid_reset(PIDController *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output      = 0.0f;
}

void pid_tune(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}