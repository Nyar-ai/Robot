/**
 * @file    scurve.c
 * @brief   多实例位置型梯形(三段式)速度规划器实现
 *
 * 纯梯形加减速, 参数集对齐 project/StepMotor.c 的 3 参数模型:
 *   {max_speed, min_speed, max_accel}
 *
 * 梯形加减速核心公式(与 project 一致): v² = v0² + 2·a·x
 *   - 加速段: 速度线性上升, 加速度 = +max_accel
 *   - 减速段: 速度线性下降, 加速度 = -max_accel
 *   - 匀速段: 速度 = max_speed (短距离三角形曲线无此段)
 *
 * 阶段切换判据(project StepMotor_Pos_Run_Function 的等价实现):
 *   - 进入减速: 剩余距离 ≤ 当前速度减速到 0 所需距离 v²/(2a)
 *     (等价于 project 用 REAL_DEC_STEPS 判断 "步数计数器 ≥ ABS_POSSHIFT - REAL_DEC_STEPS")
 *   - 进入匀速: 加速段速度达到 max_speed
 *     (等价于 project 用 ACC_STEPS 判断加速段完成)
 *   - 短距离无匀速段: 加速中途即触发减速判据 → 三角形曲线
 *     (等价于 project 的 "ABS_POSSHIFT < ACC_ADD_DEC_STEPS ⇒ REAL_ACC=POSSHIFT/2")
 *
 * 启动抖动抑制(min_speed, 对应 project 的 MinStartSpeed):
 *   - Scurve_MoveTo 从 min_speed 起步(而非0), 跳过电机低速死区
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

    float v0 = p->cfg.min_speed;
    /* 短距离起步限幅: 保证 v0 低于"半程匀速"对应速度 */
    float v_short = sqrtf(2.0f * p->cfg.max_accel * target_abs * 0.4f);
    if (v_short < v0) v0 = v_short;
    if (v0 < 1.0f) v0 = 1.0f;

    p->target_pos    = target_pos;
    p->current_pos   = 0.0f;
    p->current_accel = 0.0f;
    p->direction     = target_pos >= 0.0f ? 1.0f : -1.0f;
    /* 启动从 min_speed 起步(跳过低速死区), 进入加速段 */
    p->current_speed = v0 * p->direction;
    p->state         = SCURVE_ACCEL;
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

    /* ---- 绝对值域: 位置/速度按 direction 折算成正方向处理 ---- */
    float target_abs  = p->target_pos  < 0.0f ? -p->target_pos  : p->target_pos;
    float current_abs = p->current_pos < 0.0f ? -p->current_pos : p->current_pos;
    float remaining   = target_abs - current_abs;
    if (remaining < 0.0f) remaining = 0.0f;

    float speed_abs = p->current_speed < 0.0f ? -p->current_speed : p->current_speed;

    /* ---- 减速所需距离(梯形: v²/(2a), 加一拍裕量) ---- */
    float required_decel = (speed_abs * speed_abs) / (2.0f * p->cfg.max_accel)
                         + 0.5f * speed_abs * dt;

    /* ---- 决定本拍状态: 加速 / 匀速 / 减速 ----
     * 优先级: 减速判据(冲过目标风险) > 峰值判据(是否到 max_speed) > 匀速 */
    if (remaining <= required_decel) {
        /* ---- 减速段 ----
         * 任意状态(加速/匀速)一旦满足减速判据即切入减速.
         * 短距离时加速中途即触发 → 三角形曲线(无匀速段) */
        p->state = SCURVE_DECEL;
    } else if (p->state == SCURVE_ACCEL && speed_abs >= p->cfg.max_speed) {
        /* ---- 达到最大速度, 切匀速 ---- */
        speed_abs = p->cfg.max_speed;  /* 钳到峰值消除舍入误差 */
        p->state  = SCURVE_CONST_SPEED;
    }

    /* ---- 按当前状态施加加速度 ---- */
    switch (p->state) {
    case SCURVE_ACCEL:
        p->current_accel = p->cfg.max_accel;
        break;
    case SCURVE_CONST_SPEED:
        p->current_accel = 0.0f;
        break;
    case SCURVE_DECEL:
        p->current_accel = -p->cfg.max_accel;
        break;
    default:
        p->current_accel = 0.0f;
        break;
    }

    /* ---- 积分更新(绝对值域) ----
     * accel>0 加速(speed_abs增大), accel<0 减速(speed_abs减小)
     * 用 speed_abs 积分避免 accel*direction 符号叠加错误 */
    speed_abs += p->current_accel * dt;
    /* 减速过 0: 速度已降到底(后向欧拉离散残差使位置略差 v·dt/2),
     * 此时强制钳位到目标停车, 消除末端死锁(否则 remaining>0 而 speed=0 永远卡住) */
    if (speed_abs < 0.0f) {
        p->current_pos   = p->target_pos;
        p->current_speed = 0.0f;
        p->current_accel = 0.0f;
        p->state         = SCURVE_STOP;
        return 0.0f;
    }
    /* 速度上限(max_speed) */
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