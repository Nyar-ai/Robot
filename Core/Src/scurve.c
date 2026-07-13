/**
 * @file    scurve.c
 * @brief   多实例位置型 S 形(七段式)速度规划器实现
 *
 * 移植自 SMotor 工程(Motor.c),核心七段式状态机与 jerk/accel 限幅逻辑保留,
 * 改造点:
 *   - 多实例(StepperCtrl 全局变量 → Scurve_Planner *p)
 *   - 纯算法,去掉硬编码 htim3/GPIO(PWM 输出由上层电机驱动负责)
 *   - 支持双向(direction = ±1),速度带符号
 *
 * 减速逻辑修复(方案A, 4个Bug):
 *   Bug1: 任意加速态(含 ACCEL_JERK_UP/CONST)都能进入减速,不再落空 default
 *   Bug2: 减速距离加 jerk 延迟补偿,提前减速点,避免末端冲过目标
 *   Bug3: 删除减速入口的无效 fallthrough 死代码
 *   Bug4: 全程用绝对值域积分(speed_abs),避免 accel*direction 符号叠加错误
 *
 * 启动抖动抑制(min_speed):
 *   - Scurve_MoveTo 从 min_speed 起步(而非0), 跳过电机低速死区, 避免启动顿挫
 *   - 减速末端停在 min_speed 再清零, 避免低速抖动
 */
#include "scurve.h"
#include <math.h>

/* ---- 对外接口 ----------------------------------------------------------- */

void Scurve_SetConfig(Scurve_Planner *p, const Scurve_Config *cfg)
{
    if (p == NULL || cfg == NULL) return;
    /* 防止除零 / 负值 */
    p->cfg.max_speed = cfg->max_speed > 0.0f ? cfg->max_speed : 1.0f;
    p->cfg.max_accel = cfg->max_accel > 0.0f ? cfg->max_accel : 1.0f;
    p->cfg.max_jerk  = cfg->max_jerk  > 0.0f ? cfg->max_jerk  : 1.0f;
    /* min_speed: 跳过低速死区; 必须小于 max_speed, 否则钳到 max_speed/4 */
    p->cfg.min_speed = cfg->min_speed > 0.0f ? cfg->min_speed : 0.0f;
    if (p->cfg.min_speed >= p->cfg.max_speed) p->cfg.min_speed = p->cfg.max_speed * 0.25f;
}

void Scurve_Init(Scurve_Planner *p, const Scurve_Config *cfg)
{
    if (p == NULL) return;
    p->state         = SCURVE_STOP;
    p->current_speed = 0.0f;
    p->current_accel = 0.0f;
    p->target_pos    = 0.0f;
    p->current_pos   = 0.0f;
    p->direction     = 1.0f;
    Scurve_SetConfig(p, cfg);
}

void Scurve_MoveTo(Scurve_Planner *p, float target_pos)
{
    if (p == NULL) return;
    /* 目标位移过小则直接判定完成,避免数值抖动 */
    float target_abs = target_pos < 0.0f ? -target_pos : target_pos;
    if (target_abs < 1e-3f) {
        Scurve_Stop(p);
        return;
    }
    p->target_pos    = target_pos;
    p->current_pos   = 0.0f;
    p->current_accel = 0.0f;
    p->direction     = target_pos >= 0.0f ? 1.0f : -1.0f;
    /* 启动从 min_speed 起步(跳过低速死区), 避免启动顿挫/抖动 */
    p->current_speed = p->cfg.min_speed * p->direction;
    p->state         = SCURVE_ACCEL_JERK_UP;
}

void Scurve_Stop(Scurve_Planner *p)
{
    if (p == NULL) return;
    p->state         = SCURVE_STOP;
    p->current_speed = 0.0f;
    p->current_accel = 0.0f;
    /* 不清零 current_pos,便于上层查看已完成量 */
}

bool Scurve_IsIdle(const Scurve_Planner *p)
{
    return (p != NULL) && (p->state == SCURVE_STOP);
}

float Scurve_GetPos(const Scurve_Planner *p)
{
    return (p != NULL) ? p->current_pos : 0.0f;
}

float Scurve_GetSpeed(const Scurve_Planner *p)
{
    return (p != NULL) ? p->current_speed : 0.0f;
}

float Scurve_Update(Scurve_Planner *p, float dt)
{
    if (p == NULL || p->state == SCURVE_STOP || dt <= 0.0f) {
        return 0.0f;
    }

    /* ---- 绝对值域: 位置/速度按 direction 折算成正方向处理 (Bug4) ---- */
    float target_abs  = p->target_pos  < 0.0f ? -p->target_pos  : p->target_pos;
    float current_abs = p->current_pos < 0.0f ? -p->current_pos : p->current_pos;
    float remaining   = target_abs - current_abs;
    if (remaining < 0.0f) remaining = 0.0f;

    float speed_abs = p->current_speed < 0.0f ? -p->current_speed : p->current_speed;

    /* ---- 减速所需距离(补偿 jerk 延迟, Bug2) ----
     * S曲线减速 = 减速启动jerk段 + 匀减速段 + 收减速jerk段
     *   t1 = (accel_now + max_accel)/jerk : 加速度从当前值降到 -max_accel 的耗时
     *   匀减速: v²/(2a)
     *   收减速jerk段由"预测速度过0"触发,不单独估算(保守)
     * 加 0.5*v*dt 一拍裕量, 保证减速点足够提前 */
    float t1 = (p->current_accel + p->cfg.max_accel) / p->cfg.max_jerk;
    if (t1 < 0.0f) t1 = 0.0f;
    float required_decel = (speed_abs * speed_abs) / (2.0f * p->cfg.max_accel)
                         + speed_abs * t1
                         + 0.5f * speed_abs * dt;

    /* ---- 决定本拍是加速还是减速 (Bug1: 不再依赖"先切状态", 任意态都能进减速) ---- */
    bool need_decel = (remaining <= required_decel);

    if (need_decel) {
        /* ---- 减速阶段 ----
         * Bug1+Bug3: 任何加速/匀速态直接当 DECEL_JERK_UP 入口, 删除无效 fallthrough */
        if (p->state == SCURVE_ACCEL_JERK_UP  ||
            p->state == SCURVE_ACCEL_CONST    ||
            p->state == SCURVE_ACCEL_JERK_DOWN||
            p->state == SCURVE_CONST_SPEED) {
            p->state = SCURVE_DECEL_JERK_UP;
        }

        switch (p->state) {
        case SCURVE_DECEL_JERK_UP:
            /* 加速度从当前值(可能>0)经 jerk 线性减小到 -max_accel */
            p->current_accel -= p->cfg.max_jerk * dt;
            if (p->current_accel <= -p->cfg.max_accel) {
                p->current_accel = -p->cfg.max_accel;
                p->state = SCURVE_DECEL_CONST;
            }
            break;
        case SCURVE_DECEL_CONST:
            /* 预测下一拍速度是否到 min_speed, 是则进入收减速 jerk */
            if (speed_abs + p->current_accel * dt <= p->cfg.min_speed) {
                p->state = SCURVE_DECEL_JERK_DOWN;
            }
            break;
        case SCURVE_DECEL_JERK_DOWN:
            p->current_accel += p->cfg.max_jerk * dt;
            if (p->current_accel >= 0.0f) {
                p->current_accel   = 0.0f;
                p->current_speed   = 0.0f;
                /* 钳到目标精确停车 */
                p->current_pos     = p->target_pos;
                p->state           = SCURVE_STOP;
                return 0.0f;
            }
            break;
        default:
            break;
        }
    } else {
        /* ---- 加速/匀速阶段 ---- */
        switch (p->state) {
        case SCURVE_ACCEL_JERK_UP:
            p->current_accel += p->cfg.max_jerk * dt;
            if (p->current_accel >= p->cfg.max_accel) {
                p->current_accel = p->cfg.max_accel;
                p->state = SCURVE_ACCEL_CONST;
            }
            break;
        case SCURVE_ACCEL_CONST:
            /* 预测下一拍到 max_speed 就提前收 jerk */
            if (speed_abs + p->current_accel * dt >= p->cfg.max_speed) {
                p->state = SCURVE_ACCEL_JERK_DOWN;
            }
            break;
        case SCURVE_ACCEL_JERK_DOWN:
            p->current_accel -= p->cfg.max_jerk * dt;
            if (p->current_accel <= 0.0f) {
                p->current_accel = 0.0f;
                /* 速度钳到 max_speed,消除舍入误差 */
                speed_abs = p->cfg.max_speed;
                p->state = SCURVE_CONST_SPEED;
            }
            break;
        case SCURVE_CONST_SPEED:
            /* 匀速, 等减速点 */
            break;
        default:
            /* 异常: 处于减速态但不需要减速(remaining又变大), 重置为匀速 */
            p->current_accel = 0.0f;
            p->state = SCURVE_CONST_SPEED;
            break;
        }
    }

    /* ---- 积分更新(绝对值域, Bug4) ----
     * accel>0 加速(speed_abs增大), accel<0 减速(speed_abs减小)
     * 用 speed_abs 积分避免 accel*direction 符号叠加错误 */
    speed_abs += p->current_accel * dt;
    /* 减速反向钳位: 不许冲过 min_speed 之下太多(留余量), 最终停车由状态机精确处理 */
    if (speed_abs < p->cfg.min_speed * 0.5f) speed_abs = p->cfg.min_speed * 0.5f;
    /* 速度上限 */
    if (speed_abs > p->cfg.max_speed) speed_abs = p->cfg.max_speed;

    /* 还原带符号速度 + 积分位置 */
    p->current_speed = speed_abs * p->direction;
    p->current_pos  += p->current_speed * dt;

    /* 到达/越过目标 → 精确停车 */
    float done_abs = p->current_pos < 0.0f ? -p->current_pos : p->current_pos;
    if (done_abs >= target_abs) {
        p->current_pos   = p->target_pos;
        p->current_speed = 0.0f;
        p->current_accel = 0.0f;
        p->state         = SCURVE_STOP;
    }

    return p->current_speed;
}