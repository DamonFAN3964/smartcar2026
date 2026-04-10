#pragma once
/*******************************************************************************
 * car_control.h — 车辆底层控制（与原版相同，不修改）
 ******************************************************************************/

/* ─── 舵机 PWM 通道 ───
 * 根据实际硬件接线修改
 */
#define SERVO_PWM_CH        2
#define SERVO_CENTER_DUTY   1500   // 舵机回中（微秒/占空比，依实际调整）
#define SERVO_LEFT_DUTY     1100   // 舵机最左
#define SERVO_RIGHT_DUTY    1900   // 舵机最右

/* ─── 电机方向 ─── */
#define MOTOR_FORWARD_LEVEL  0
#define MOTOR_BACKWARD_LEVEL 1
#define MOTOR_MAX_DUTY       10000

/* ─── 接口 ─── */

/**
 * @brief 初始化电机和舵机
 * @return 0=成功，-1=失败
 */
int  car_init();

/**
 * @brief 设置行驶速度
 * @param speed  -100~100，正值前进，负值后退
 */
void car_set_speed(int speed);

/**
 * @brief 设置转向
 * @param steer  -100~100，负值左转，正值右转
 */
void car_set_steer(int steer);

/**
 * @brief 同时设置速度和转向（效率高于分开调用）
 */
void car_drive(int speed, int steer);

/**
 * @brief 停车（速度=0，舵机回中）
 */
void car_stop();