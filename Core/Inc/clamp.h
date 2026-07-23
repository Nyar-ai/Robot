/**
 * @file    clamp.h
 * @brief   夹具控制层: 高度(带轮步进电机) / 夹爪(舵机) / 旋转(舵机)
 *
 * 设计:
 *   - 上层(FreeRTOS)以"发起-轮询"方式使用, 接口返回 bool 表示是否达到目标
 *   - 高度控制用 Scurve_Planner 实现梯形加减速
 *   - 舵机直接设定角度, 无需 tick 推进
 *
 * 典型用法(非阻塞):
 *   clamp_set_height(50.0f);                  // 发起
 *   while (!clamp_set_height(50.0f)) {        // 轮询, 直到返回 true
 *       osDelay(1);
 *   }
 */
#ifndef __CLAMP_H
#define __CLAMP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- 带轮步进电机参数 ---- */
#define CLAMP_STEPS_PER_REV        3200.0f   /* 200步/圈 × 16细分 */
#define CLAMP_PULLEY_DIAMETER_MM   10.0f     /* 带轮节圆直径 mm */
#define CLAMP_PULLEY_CIRCUM_MM     (3.14159265358979323846f * CLAMP_PULLEY_DIAMETER_MM)  /* 周长 ≈31.416mm */

/* ---- 到达阈值 ---- */
#define CLAMP_HEIGHT_TOL_MM        0.05f     /* 高度到达阈值(mm) */

/* ---- 默认梯形曲线参数(高度, 量纲 mm) ---- */
#define CLAMP_HEIGHT_MAX_SPEED     50.0f     /* mm/s   */
#define CLAMP_HEIGHT_MAX_ACCEL     80.0f     /* mm/s^2 */
#define CLAMP_HEIGHT_MIN_SPEED     5.0f      /* mm/s   (启动起步速度, 跳过电机死区) */

/* ---- 控制周期 ---- */
#define CLAMP_TICK_DT_S            0.001f    /* clamp_tick 步长 1ms */

/**
 * @brief 初始化夹具控制层(清空状态, 配置 S 曲线参数, 进入 IDLE)
 */
void clamp_init(void);

/**
 * @brief 1ms 周期推进(放 FreeRTOS clampTask 里调用)
 *        推进高度 S 曲线 → 下发给步进电机驱动层
 */
void clamp_tick(void);

/**
 * @brief 发起/推进"设定夹具高度到 target_mm"
 * @param target_mm 目标高度(绝对位置, mm)
 * @return true 表示已到达(且已停下); false 表示运动中
 * @note  非阻塞. 目标不变时重复调用仅轮询状态; 目标变化时自动重新规划.
 */
bool clamp_set_height(float target_mm);

/**
 * @brief 紧急停车(立即清零速度, 退出运动状态)
 */
void clamp_stop(void);

/**
 * @brief 当前高度电机是否处于空闲(非运动)状态
 */
bool clamp_is_idle(void);

/**
 * @brief 获取当前高度(mm)
 */
float clamp_get_height(void);

/**
 * @brief 设置当前高度(用于归零/校准)
 */
void clamp_set_height_now(float pos_mm);

/* ================================================================
 * 后续扩展接口(舵机, 占空比控制, 无需 tick 推进)
 *   void clamp_gripper_open(void);          // 夹爪张开
 *   void clamp_gripper_close(void);         // 夹爪闭合
 *   void clamp_gripper_set(uint8_t angle);  // 夹爪精确角度
 *   void clamp_rotate_set(uint8_t angle);   // 夹具旋转角度
 * ================================================================ */

#endif /* __CLAMP_H */
