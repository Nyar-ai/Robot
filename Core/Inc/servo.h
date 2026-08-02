/**
 * @file    servo.h
 * @brief   舵机驱动层 (PWM 脉宽控制)
 *
 * 硬件映射:
 *   夹爪: PA0 / TIM2_CH1
 *   旋转: PA2 / TIM2_CH3
 *
 * 工作原理:
 *   - TIM2 配置为 50Hz (20ms 周期), 1MHz 计数频率
 *   - 脉宽 500~2500μs → 夹爪0°~270° / 旋转0°~270°
 *   - 一次调用即时生效, 无需 tick 推进
 *
 * 与 clamp 层对接:
 *   clamp_gripper_open()  → Servo_SetAngle(SERVO_GRIPPER, CLAMP_GRIPPER_OPEN_DEG)
 *   clamp_gripper_close() → Servo_SetAngle(SERVO_GRIPPER, CLAMP_GRIPPER_CLOSE_DEG)
 *   clamp_rotate_set(deg) → Servo_SetAngle(SERVO_ROTATE, deg)
 */
#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>
#include <stdbool.h>

/* 舵机编号 */
#define SERVO_GRIPPER  0   /* PA0 / TIM2_CH1  (0 - 270deg) */
#define SERVO_ROTATE   1   /* PA2 / TIM2_CH3  (0 - 270deg) */
#define SERVO_NUM      2

/* ---- 舵机 PWM 参数 ---- */
#define SERVO_TIMER_FREQ_HZ      1000000UL   /* 1MHz (1us/tick) */
#define SERVO_PWM_PERIOD_US      20000UL     /* 20ms 周期 -> 50Hz */
#define SERVO_MIN_PULSE_US       500         /* 0deg   0.5ms */
#define SERVO_MAX_PULSE_US       2500        /* max deg 2.5ms */

/* ---- 各舵机角度上限 ---- */
#define SERVO_MAX_DEG_GRIPPER    270         /* 夹爪 (0-270度舵机) */
#define SERVO_MAX_DEG_ROTATE     270         /* 旋转 */

/**
 * @brief 初始化舵机底层:
 *        配置 TIM2 PSC=83(->1MHz)、ARR=19999(->20ms),
 *        启动 CH1/CH3 PWM, 初始角度: 夹爪张开(180deg), 旋转中位(135deg).
 * @note  必须在 MX_TIM2_Init() 之后调用; 由 clamp_init() 内部调用.
 */
void Servo_Init(void);

/**
 * @brief 设定舵机角度
 * @param id  舵机编号 SERVO_GRIPPER / SERVO_ROTATE
 * @param deg 目标角度: 夹爪 0~270deg, 旋转 0~270deg, 超出自动钳位
 */
void Servo_SetAngle(uint8_t id, uint16_t deg);

/**
 * @brief 设定舵机脉宽(us)
 * @param id 舵机编号
 * @param us 脉宽 500~2500
 */
void Servo_SetPulseUs(uint8_t id, uint16_t us);

#endif /* __SERVO_H */


