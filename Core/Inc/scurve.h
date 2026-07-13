/**
 * @file    scurve.h
 * @brief   多实例位置型梯形(三段式)速度规划器
 *
 * 纯梯形加减速: 加速段(恒定加速度)→匀速段→减速段(恒定减速度).
 * 参数集对齐 project/StepMotor.c 的 3 参数模型:
 *   StepMotor_Init(PHandle, MaxSpeed, MinStartSpeed, ACCValue)
 *   ⇒ {max_speed, min_speed, max_accel}
 *
 * 工作流程:
 *   1) Scurve_MoveTo(planner, target)   设定目标位移(从当前位置算起), 从 min_speed 起步
 *   2) 周期调用 Scurve_Update(planner, dt) 推进状态机, 返回当前速度
 *   3) Scurve_IsIdle(planner) == true 表示已到达目标并停下
 *
 * 量纲: 位置/速度/加速度由调用方约定(可 mm / 步 / 度 ...), 保持三者一致即可。
 */
#ifndef __SCURVE_H
#define __SCURVE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* NULL (ARMCC 5 不自动引入) */

/* 梯形曲线状态机(三段式) */
typedef enum {
    SCURVE_STOP = 0,
    SCURVE_ACCEL,        /* 加速段: 加速度=+MAX_ACCEL, 速度线性上升 */
    SCURVE_CONST_SPEED,  /* 匀速段: 速度=MAX_SPEED */
    SCURVE_DECEL         /* 减速段: 加速度=-MAX_ACCEL, 速度线性下降到 0 */
} Scurve_State;

/* 曲线参数集(平移/旋转可各用一套) —— 对齐 project 的 3 参数模型 */
typedef struct {
    float max_speed;  /* 最大速度(量纲与位置一致/s)   */
    float max_accel;  /* 最大加速度(/s^2)            */
    float min_speed;  /* 启动最小速度(/s, 跳过低速死区, 0=从0起步) */
} Scurve_Config;

/* 规划器实例 */
typedef struct {
    Scurve_State   state;
    Scurve_Config  cfg;
    float current_speed;  /* 当前速度 */
    float current_accel;  /* 当前加速度 */
    float target_pos;     /* 目标位移(相对启动时刻) */
    float current_pos;    /* 已完成位移 */
    float direction;      /* +1 / -1 方向 */
} Scurve_Planner;

/**
 * @brief 初始化规划器(清零状态,设置参数)
 */
void Scurve_Init(Scurve_Planner *p, const Scurve_Config *cfg);

/**
 * @brief 重新设置参数(可在运行中调整)
 */
void Scurve_SetConfig(Scurve_Planner *p, const Scurve_Config *cfg);

/**
 * @brief 发起一次"走到 target_pos 位移"的运动(相对当前位置)
 * @param target_pos 目标位移(可正可负,符号代表方向)
 * @note  会清零当前位移/速度/加速度, 从 min_speed 起步进入加速段.
 */
void Scurve_MoveTo(Scurve_Planner *p, float target_pos);

/**
 * @brief 周期推进状态机
 * @param dt 时间步长(秒), 例如 0.001f
 * @return 当前速度(带符号: target_pos>0 时为正)
 */
float Scurve_Update(Scurve_Planner *p, float dt);

/**
 * @brief 立即停车(速度/加速度清零,状态切到 STOP)
 */
void Scurve_Stop(Scurve_Planner *p);

/**
 * @brief 是否处于空闲(STOP)状态
 */
bool Scurve_IsIdle(const Scurve_Planner *p);

/**
 * @brief 获取已完成位移(带符号)
 */
float Scurve_GetPos(const Scurve_Planner *p);

/**
 * @brief 获取当前速度(带符号)
 */
float Scurve_GetSpeed(const Scurve_Planner *p);

#endif /* __SCURVE_H */