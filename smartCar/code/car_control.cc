/*******************************************************************************
 * car_control.cpp — 车辆底层控制实现
 ******************************************************************************/
#include "car_control.h"
#include "headfile.h"
#include <algorithm>
#include <cmath>

// 全局 Motor 对象（wuwu 库）
static Motor g_motor;

// ─────────────────────── 内部辅助 ───────────────────────────────────

static inline int iclamp(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// speed(-100~100) → duty(0~MOTOR_MAX_DUTY)
static int speed_to_duty(int speed)
{
    int abs_spd = abs(iclamp(speed, -100, 100));
    return abs_spd * MOTOR_MAX_DUTY / 100;
}

// steer(-100~100) → servo duty(SERVO_LEFT_DUTY~SERVO_RIGHT_DUTY)
static int steer_to_servo(int steer)
{
    steer = iclamp(steer, -100, 100);
    // 线性插值：-100 → LEFT_DUTY, 0 → CENTER_DUTY, +100 → RIGHT_DUTY
    float t = (float)steer / 100.0f;       // -1.0 ~ +1.0
    int duty;
    if (t >= 0.0f) {
        duty = SERVO_CENTER_DUTY +
               (int)(t * (SERVO_RIGHT_DUTY - SERVO_CENTER_DUTY));
    } else {
        duty = SERVO_CENTER_DUTY +
               (int)(t * (SERVO_CENTER_DUTY - SERVO_LEFT_DUTY));
    }
    return iclamp(duty, SERVO_LEFT_DUTY, SERVO_RIGHT_DUTY);
}

// ─────────────────────── 接口实现 ───────────────────────────────────

int car_init()
{
    if (g_motor.motor_init() < 0) {
        return -1;
    }
    // 上电后舵机回中，电机停止
    g_motor.set_pwm_duty(SERVO_PWM_CH, SERVO_CENTER_DUTY);
    g_motor.set_motor1(MOTOR_FORWARD_LEVEL, 0);
    return 0;
}

void car_set_speed(int speed)
{
    speed = iclamp(speed, -100, 100);
    int duty = speed_to_duty(speed);
    int dir  = (speed >= 0) ? MOTOR_FORWARD_LEVEL : MOTOR_BACKWARD_LEVEL;
    g_motor.set_motor1(dir, duty);
}

void car_set_steer(int steer)
{
    int servo_duty = steer_to_servo(steer);
    g_motor.set_pwm_duty(SERVO_PWM_CH, servo_duty);
}

void car_drive(int speed, int steer)
{
    // 一次调用同时设置，减少总线访问次数
    car_set_speed(speed);
    car_set_steer(steer);
}

void car_stop()
{
    g_motor.set_motor1(MOTOR_FORWARD_LEVEL, 0);
    g_motor.set_pwm_duty(SERVO_PWM_CH, SERVO_CENTER_DUTY);
}