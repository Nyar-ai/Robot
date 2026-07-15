/**
 * @file    chassis.h
 * @brief   底盘控制层: 世界坐标系下"跑到坐标" / "转向到绝对角度"
 *
 * 设计:
 *   - 上层(FreeRTOS)以"发起-轮询"方式使用, 接口返回 bool 表示是否达到目标
 *   - 内部用两个 Scurve_Planner: 一个管平移位移(单位 mm), 一个管旋转角度(单位 deg)
 *   - 世界固定坐标系: (x 向前, y 向左), 目标点/角度都是世界系绝对量
 *   - 角度由"陀螺仪注入"提供; 没注入则退化用里程计积分 θ
 *
 * 典型用法(非阻塞):
 *   move_to_coordinate(1000, 500);             // 发起
 *   while (!move_to_coordinate(1000, 500)) {   // 轮询, 直到返回 true
 *       osDelay(1);
 *   }
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* NULL (ARMCC 5 不自动引入) */

/* ---- 到达阈值 ---- */
#define CHASSIS_POS_TOL_MM        0.05f    /* 位置到达阈值(mm)   */
#define CHASSIS_ANG_TOL_DEG       0.1f    /* 角度到达阈值(deg)  */
         
/* ---- 默认梯形曲线参数(平移, 量纲 mm) —— 对齐 project 3 参数模型 ----
 * MAX_SPEED: 匀速段最大速度
 * MAX_ACCEL: 加/减速度(恒定值)
 * MIN_SPEED: 启动起步速度, 跳过低速死区(对应 project 的 MinStartSpeed) */
#define CHASSIS_TRANS_MAX_SPEED   200.0f    /* mm/s   */
#define CHASSIS_TRANS_MAX_ACCEL   100.0f    /* mm/s^2 */
#define CHASSIS_TRANS_MIN_SPEED   25.0f     /* mm/s   (启动起步速度, 跳过电机死区≈0.94) */

/* ---- 默认梯形曲线参数(旋转, 量纲 deg) —— 对齐 project 3 参数模型 ---- */
#define CHASSIS_ROT_MAX_SPEED     90.0f    /* deg/s   */
#define CHASSIS_ROT_MAX_ACCEL     45.0f    /* deg/s	^2 (旋转惯量大需更缓) */
#define CHASSIS_ROT_MIN_SPEED     5.0f     /* deg/s   (旋转启动起步速度) */

/* ---- 控制周期 ---- */
#define CHASSIS_TICK_DT_S         0.001f    /* chassis_tick 步长 1ms */

/**
 * @brief 初始化底盘层(清零位姿, 配置 S 曲线参数, 进入 IDLE)
 */
void chassis_init(void);

/**
 * @brief 注入陀螺仪航向角(世界系绝对角度)
 * @param gyro_deg 陀螺仪航向角(deg), 与 headturn/move 的角度量纲一致
 * @note  一旦调用, 内部 θ 全部改用陀螺仪; 不调用则退化为里程计积分 θ
 */
void chassis_feed_gyro(float gyro_deg);

/**
 * @brief 清除陀螺仪注入(退化为纯里程计)
 */
void chassis_clear_gyro(void);

/**
 * @brief 发起/推进"移动到世界坐标 (tx, ty)"
 * @return true 表示已到达(且已停下); false 表示运动中
 * @note  非阻塞. 上层需周期轮询同一目标, 或在 1ms 任务里查询状态
 */
bool move_to_coordinate(float tx, float ty);

/**
 * @brief 发起/推进"转向到世界系绝对角度 angle(deg)"
 * @param angle 目标绝对角度(deg)
 * @return true 表示已到达(且已停下); false 表示运动中
 */
bool headturn(int16_t angle);

/**
 * @brief 底盘周期推进(放 FreeRTOS 1ms 任务里调用)
 *        跑 S 曲线 → 速度分解 → 麦轮逆解 → 更新 4 轮目标速度 + 里程计
 */
void chassis_tick(void);

/**
 * @brief 取 4 轮目标线速度(mm/s), 供电机层换算成步/秒后下发 PWM
 */
void chassis_get_wheel_speed(float w[4]);

/**
 * @brief 取当前世界系位姿
 */
void chassis_get_pose(float *x, float *y, float *theta_deg);

/**
 * @brief 设置当前位姿(用于对齐起点/重置原点)
 */
void chassis_set_pose(float x, float y, float theta_deg);

/**
 * @brief 紧急停车(立即清零速度, 退出运动状态)
 */
void chassis_stop(void);

/**
 * @brief 当前是否处于空闲(非运动)状态
 */
bool chassis_is_idle(void);

#endif /* __CHASSIS_H */