/**
 * @file    clamp.c
 * @brief   夹具控制层实现: 高度(带轮步进电机)梯形加减速控制
 *
 * 数据流:
 *   clamp_set_height(target_mm) ──► [状态机] ──► Scurve_Planner(量纲:步)
 *                                                      │
 *   clamp_tick (1ms, clampTask) ────────────────────────┘
 *                                                      │
 *                                            Scurve_Update → 步速
 *                                                      │
 *                                          Stepper_SetSpeed(M5, step_s)
 *
 * 高度换算: step = mm / CLAMP_PULLEY_CIRCUM_MM * CLAMP_STEPS_PER_REV
 */
#include "clamp.h"
#include "scurve.h"
#include "stepper.h"
#include <math.h>

/* ---- 内部状态 ----------------------------------------------------------- */
typedef enum {
    CL_STATE_IDLE = 0,
    CL_STATE_MOVING
} Clamp_State;

typedef struct {
    /* 高度(绝对位置, mm) */
    float height_mm;

    /* 运动状态 */
    Clamp_State  state;
    bool         arrived;

    /* 梯形规划器(量纲: 步) */
    Scurve_Planner planner;

    /* 本次目标(mm), 用于检测目标是否变化 */
    float target_mm;
} Clamp_Ctrl;

static Clamp_Ctrl g_clamp;

/* ---- 内部辅助 ----------------------------------------------------------- */

/* mm → steps 换算 */
static float mm_to_steps(float mm)
{
    return (mm / CLAMP_PULLEY_CIRCUM_MM) * CLAMP_STEPS_PER_REV;
}

/* steps → mm 换算 */
static float steps_to_mm(float steps)
{
    return (steps / CLAMP_STEPS_PER_REV) * CLAMP_PULLEY_CIRCUM_MM;
}

static float fabsf_local(float v) { return v < 0.0f ? -v : v; }

/* ---- 对外接口 ----------------------------------------------------------- */

void clamp_init(void)
{
    Scurve_Config cfg = {
        .max_speed = mm_to_steps(CLAMP_HEIGHT_MAX_SPEED),   /* mm/s → 步/s */
        .max_accel = mm_to_steps(CLAMP_HEIGHT_MAX_ACCEL),   /* mm/s² → 步/s² */
        .min_speed = mm_to_steps(CLAMP_HEIGHT_MIN_SPEED)    /* mm/s → 步/s */
    };
    Scurve_Init(&g_clamp.planner, &cfg);

    g_clamp.height_mm = 0.0f;
    g_clamp.state     = CL_STATE_IDLE;
    g_clamp.arrived   = true;
    g_clamp.target_mm = 0.0f;
}

void clamp_tick(void)
{
    float dt = CLAMP_TICK_DT_S;
    float step_s = 0.0f;

    switch (g_clamp.state) {
    case CL_STATE_MOVING: {
        /* S 曲线推进，返回当前速度(步/s, 带符号) */
        step_s = Scurve_Update(&g_clamp.planner, dt);

        /* 积分高度(已完成位移对应的 mm) */
        float delta_mm = steps_to_mm(step_s) * dt;
        g_clamp.height_mm += delta_mm;

        /* 到达判定: 规划器空闲 */
        if (Scurve_IsIdle(&g_clamp.planner)) {
            float err = fabsf_local(g_clamp.target_mm - g_clamp.height_mm);
            if (err < CLAMP_HEIGHT_TOL_MM * 3.0f) {
                g_clamp.state   = CL_STATE_IDLE;
                g_clamp.arrived = true;
                g_clamp.height_mm = g_clamp.target_mm;  /* 精确对齐 */
                step_s = 0.0f;
            } else {
                /* 规划器已停但没到(积分漂移), 重新规划剩余距离 */
                float remaining_mm = g_clamp.target_mm - g_clamp.height_mm;
                float remaining_steps = mm_to_steps(remaining_mm);
                if (fabsf_local(remaining_steps) < 1e-3f) {
                    g_clamp.state   = CL_STATE_IDLE;
                    g_clamp.arrived = true;
                    g_clamp.height_mm = g_clamp.target_mm;
                } else {
                    Scurve_MoveTo(&g_clamp.planner, remaining_steps);
                }
                step_s = 0.0f;
            }
        }
        break;
    }
    case CL_STATE_IDLE:
    default:
        step_s = 0.0f;
        break;
    }

    /* 下发步速到驱动层 */
    Stepper_SetSpeed(STEPPER_M5, step_s);
}

bool clamp_set_height(float target_mm)
{
    /* 目标变化 → 重新规划 */
    bool target_changed = (g_clamp.state != CL_STATE_MOVING) ||
                          (fabsf_local(target_mm - g_clamp.target_mm) > 0.5f);

    if (target_changed) {
        float delta_mm = target_mm - g_clamp.height_mm;
        float delta_steps = mm_to_steps(delta_mm);

        g_clamp.target_mm = target_mm;

        if (fabsf_local(delta_mm) < CLAMP_HEIGHT_TOL_MM) {
            /* 已在目标范围内 */
            g_clamp.state   = CL_STATE_IDLE;
            g_clamp.arrived = true;
        } else {
            Scurve_MoveTo(&g_clamp.planner, delta_steps);
            g_clamp.state   = CL_STATE_MOVING;
            g_clamp.arrived = false;
        }
    }

    return g_clamp.arrived;
}

void clamp_stop(void)
{
    Scurve_Stop(&g_clamp.planner);
    g_clamp.state   = CL_STATE_IDLE;
    g_clamp.arrived = true;
    Stepper_SetSpeed(STEPPER_M5, 0.0f);
}

bool clamp_is_idle(void)
{
    return g_clamp.state == CL_STATE_IDLE;
}

float clamp_get_height(void)
{
    return g_clamp.height_mm;
}

void clamp_set_height_now(float pos_mm)
{
    g_clamp.height_mm = pos_mm;
    g_clamp.target_mm = pos_mm;
}