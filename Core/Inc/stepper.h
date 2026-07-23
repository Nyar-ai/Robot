/**
 * @file    stepper.h
 * @brief   4 路 42 步进电机驱动层 (PWM 脉冲 + DIR 方向)
 *
 * 硬件映射(基于本工程 CubeMX 已配置):
 *   M1: stp=PA6(TIM3_CH1)   DIR=PE13
 *   M2: stp=PD12(TIM4_CH1)  DIR=PE12
 *   M3: stp=PC8(TIM8_CH3)   DIR=PB5
 *   M4: stp=PE5(TIM9_CH1)   DIR=PD7
 *
 * 工作原理:
 *   - 步进电机转速 = PWM 频率(步/秒)
 *   - 通过动态改定时器 ARR 来改变 PWM 频率, 从而改变转速
 *   - 通过 DIR 引脚电平控制方向
 *
 * 注意: TIM3/4/8/9 都是 16 位定时器(ARR 上限 65535).
 *   为兼顾低速(麦轮蠕动)和高速, 把各定时器 Prescaler 设为统一工作频率 1MHz:
 *     TIM3/4 时钟 84MHz  → Prescaler=83   (84M/(83+1)=1MHz)
 *     TIM8/9 时钟 168MHz → Prescaler=167  (168M/(167+1)=1MHz)
 *   1MHz 下 16 位 ARR 可覆盖约 16 ~ 500000 步/秒, 对麦轮车足够.
 *
 * 与 chassis 层对接:
 *   chassis_get_wheel_speed(w);              // 4 轮线速度 mm/s
 *   for(i..4) Stepper_SetWheelSpeed(i, w[i]); // mm/s → 步/s → PWM
 */
#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "mecanum.h"

/* 电机编号(与 mecanum.h 的 LF/RF/LB/RB 对应, 但此处仅作 ID) */
#define STEPPER_M1   0   /* PA6 / TIM3_CH1 / DIR PE13 */
#define STEPPER_M2   1   /* PD12/ TIM4_CH1 / DIR PE12 */
#define STEPPER_M3   2   /* PC8 / TIM8_CH3 / DIR PB5  */
#define STEPPER_M4   3   /* PE5 / TIM9_CH1 / DIR PD7  */
#define STEPPER_M5   4   /* PE14/ TIM1_CH4 / DIR PE15 */
#define STEPPER_NUM  5

/* ---- 步进电机参数(按你的实车/驱动板拨码改) ---- */
#define STEPPER_STEPS_PER_REV      3200.0f   /* 200步/圈 × 16细分 = 3200 步/圈 */
#define STEPPER_WHEEL_RADIUS_MM    MECANUM_WHEEL_RADIUS_MM     /* 麦轮半径(必须与 mecanum.h 一致) */
/* 线速度→步速换算: step_s = mm_s / (2π·r) × steps_per_rev */

/* ---- 定时器统一工作频率 ---- */
#define STEPPER_TIMER_FREQ_HZ      1000000UL  /* 1 MHz (TIM3/4/8/9 经 Prescaler 后) */

/* ---- 速度限幅(避免丢步/超出16位ARR范围) ---- */
#define STEPPER_MIN_STEP_S         16.0f     /* 1MHz/65535 ≈ 15.26, 取16留余量 */
#define STEPPER_MAX_STEP_S         50000.0f  /* 最高步/秒(按电机/驱动板能力调) */

/* ---- 速度变化率限幅(平滑PWM频率跳变, 抑制启动抖动) ----
 * 相邻两次 SetSpeed 调用(1ms周期)间, 允许的最大步速变化量(步/s)
 * 超过则限幅, 相当于底层加一道低通; 0=不限幅 */
#define STEPPER_MAX_DELTA_STEP_S   250     /* 步/s (整数, 用于#if预处理) */

/**
 * @brief 初始化 4 个电机:
 *        设置各定时器 Prescaler(统一1MHz)、ARR 初值、占空比、启动 PWM、置 DIR 默认方向.
 * @note  必须在 MX_TIMx_Init() 之后调用.
 */
void Stepper_Init(void);

/**
 * @brief 设置某电机转速(步/秒, 带方向)
 * @param id     电机编号 STEPPER_M1..M4
 * @param step_s 步/秒. 正=正转, 负=反转, 0=停.
 *               超出限幅会被钳位; |step_s| < STEPPER_MIN_STEP_S 时停 PWM.
 */
void Stepper_SetSpeed(uint8_t id, float step_s);

/**
 * @brief 设置某电机轮缘线速度(mm/s, 带方向) —— 直接对接 chassis 输出
 * @param id  电机编号
 * @param mm_s 轮缘线速度 mm/s, 正负代表方向
 */
void Stepper_SetWheelSpeed(uint8_t id, float mm_s);

/**
 * @brief 全部电机停止(关 PWM)
 */
void Stepper_StopAll(void);

/**
 * @brief 某电机急停(关 PWM, 保留 DIR 状态)
 */
void Stepper_Brake(uint8_t id);

/**
 * @brief 批量下发 4 轮线速度(常见用法, 一次性更新 4 个电机)
 * @param mm_s 长度>=4 的数组, 对应 M1..M4
 */
void Stepper_SetWheelSpeedAll(const float mm_s[STEPPER_NUM]);

#endif /* __STEPPER_H */