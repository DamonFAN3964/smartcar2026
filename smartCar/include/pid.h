#pragma once
/*******************************************************************************
 * pid.h — PID 控制器（接口不变，内容与原版相同）
 ******************************************************************************/

struct PIDController {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev2_error;
    float integral_limit;
    float output_limit;
    bool  use_incremental;
    float output;
};

void  pid_init(PIDController* pid,
               float kp, float ki, float kd,
               float i_limit, float out_limit,
               bool incremental = false);

float pid_compute(PIDController* pid, float error);

void  pid_reset(PIDController* pid);

void  pid_tune(PIDController* pid, float kp, float ki, float kd);