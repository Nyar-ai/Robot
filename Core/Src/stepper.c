/**
 * @file    stepper.c
 * @brief   4 路 42 步进电机驱动实现 (PWM 脉冲 + DIR)
 *
 * 数据流(自上而下):
 *   chassis_get_wheel_speed(mm/s)  →  Stepper_SetWheelSpeed  →  Stepper_SetSpeed(步/s)
 *        │                                                              │
 *        │                                                       方向(DIR引脚)
 *        │                                                              │
 *        └──────────── ARR = timer_freq / |step_s|, 占空比 50% ◄─────────┘
 */
#include "stepper.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"

/* ---- 每个电机的硬件绑定 ---- */
typedef struct {
    TIM_HandleTypeDef *htim;        /* 定时器句柄 */
    uint32_t           channel;     /* PWM 通道 */
    GPIO_TypeDef      *dir_port;    /* DIR 引脚端口 */
    uint16_t           dir_pin;     /* DIR 引脚 */
    uint8_t            is_advanced; /* 是否高级定时器(TIM1/8, 需 MOE) */
} Stepper_Bind;

static const Stepper_Bind s_bind[STEPPER_NUM] = {
    /* M1: PA6 / TIM3_CH1 / DIR PE13 */
    { &htim3, TIM_CHANNEL_1, GPIOE, GPIO_PIN_13, 0 },
    /* M2: PD12/ TIM4_CH1 / DIR PE12 */
    { &htim4, TIM_CHANNEL_1, GPIOE, GPIO_PIN_12, 0 },
    /* M3: PC8 / TIM8_CH3 / DIR PB5  (TIM8 高级定时器, 需 MOE) */
    { &htim8, TIM_CHANNEL_3, GPIOB, GPIO_PIN_5,  1 },
    /* M4: PE5 / TIM9_CH1 / DIR PD7  */
    { &htim9, TIM_CHANNEL_1, GPIOD, GPIO_PIN_7,  0 },
};

/* 每个电机上一次下发的速度(步/s, 带符号), 用于底层速度变化率限幅 */
static float s_last_step_s[STEPPER_NUM];

/* ---- 内部辅助 ---- */

/* 启动某通道 PWM(对 TIM8 额外使能主输出 MOE) */
static void Stepper_StartPWM(const Stepper_Bind *b)
{
    HAL_TIM_PWM_Start(b->htim, b->channel);
    if (b->is_advanced) {
        /* TIM1/TIM8 高级定时器: 必须使能 MOE 才有 PWM 输出 */
        __HAL_TIM_MOE_ENABLE(b->htim);
    }
}

/* 停止某通道 PWM */
static void Stepper_StopPWM(const Stepper_Bind *b)
{
    HAL_TIM_PWM_Stop(b->htim, b->channel);
    if (b->is_advanced) {
        __HAL_TIM_MOE_DISABLE(b->htim);
    }
}

/* 局部取绝对值(避免依赖 fabsf) */
static float stepper_fabs(float v) { return v < 0.0f ? -v : v; }

/* ---- 对外接口 ---- */

void Stepper_Init(void)
{
    /* 1) 统一各定时器工作频率到 1MHz
     *    TIM3/4 在 APB1, 时钟=84MHz  → Prescaler=83
     *    TIM8/9 在 APB2, 时钟=168MHz → Prescaler=167
     *    (注意: __HAL_TIM_SET_PRESCALER 写的是"除以(PSC+1)") */
    __HAL_TIM_SET_PRESCALER(&htim3, 83);
    __HAL_TIM_SET_PRESCALER(&htim4, 83);
    __HAL_TIM_SET_PRESCALER(&htim8, 167);
    __HAL_TIM_SET_PRESCALER(&htim9, 167);

    /* 2) 各通道: ARR 预装载使能(改 ARR 时影子寄存器在更新事件生效, 避免毛刺)
     *    占空比 50%(下次 SetSpeed 会重设), 先设一个安全的中速 */
    for (uint8_t i = 0; i < STEPPER_NUM; ++i)
    {
        TIM_HandleTypeDef *h = s_bind[i].htim;
        /* 使能 ARR 预装载 */
        __HAL_TIM_ENABLE_OCxPRELOAD(h, s_bind[i].channel);
        /* 初始 ARR: 1MHz/1000 = 1000 步/秒 (只是个安全初值, 之后会被 SetSpeed 覆盖) */
        __HAL_TIM_SET_AUTORELOAD(h, 1000 - 1);
        __HAL_TIM_SET_COMPARE(h, s_bind[i].channel, 500);

        /* DIR 默认置 RESET(方向由用户定义, 这里只给个确定态) */
        HAL_GPIO_WritePin(s_bind[i].dir_port, s_bind[i].dir_pin, GPIO_PIN_RESET);

        /* 先不启动 PWM, 等 SetSpeed 时再启 */
    }
}

void Stepper_SetSpeed(uint8_t id, float step_s)
{
    if (id >= STEPPER_NUM) return;
    const Stepper_Bind *b = &s_bind[id];

    /* ---- 底层速度变化率限幅(平滑PWM跳变, 抑制抖动) ---- */
#if STEPPER_MAX_DELTA_STEP_S > 0
    float last = s_last_step_s[id];
    float delta = step_s - last;
    float d_abs = delta < 0.0f ? -delta : delta;
    if (d_abs > STEPPER_MAX_DELTA_STEP_S) {
        step_s = last + (delta < 0.0f ? -STEPPER_MAX_DELTA_STEP_S : STEPPER_MAX_DELTA_STEP_S);
    }
#endif
    s_last_step_s[id] = step_s;

    float v = stepper_fabs(step_s);

    /* 速度过低 → 停 PWM */
    if (v < STEPPER_MIN_STEP_S) {
        Stepper_StopPWM(b);
        return;
    }

    /* 速度上限钳位 */
    if (v > STEPPER_MAX_STEP_S) v = STEPPER_MAX_STEP_S;

    /* 方向: step_s>0 → DIR=RESET; step_s<0 → DIR=SET (具体极性按你驱动板, 可对调) */
    if (step_s >= 0.0f)
        HAL_GPIO_WritePin(b->dir_port, b->dir_pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(b->dir_port, b->dir_pin, GPIO_PIN_SET);

    /* ARR = timer_freq / v - 1;  占空比 50% */
    uint32_t arr = (uint32_t)((float)STEPPER_TIMER_FREQ_HZ / v);
    if (arr < 2) arr = 2;                 /* ARR 至少 2, 保证有完整脉冲 */
    if (arr > 65535) arr = 65535;         /* 16 位定时器保护 */

    __HAL_TIM_SET_AUTORELOAD(b->htim, arr - 1);
    __HAL_TIM_SET_COMPARE(b->htim, b->channel, arr / 2);

    Stepper_StartPWM(b);
}

void Stepper_SetWheelSpeed(uint8_t id, float mm_s)
{
    if (id >= STEPPER_NUM) return;
    /* step_s = mm_s / (2π·r) × steps_per_rev */
    const float PI = 3.14159265358979323846f;
    float circumference = 2.0f * PI * STEPPER_WHEEL_RADIUS_MM;  /* 周长 mm */
    float step_s = (mm_s / circumference) * STEPPER_STEPS_PER_REV;
    Stepper_SetSpeed(id, step_s);
}

void Stepper_SetWheelSpeedAll(const float mm_s[STEPPER_NUM])
{
    for (uint8_t i = 0; i < STEPPER_NUM; ++i)
        Stepper_SetWheelSpeed(i, mm_s[i]);
}

void Stepper_StopAll(void)
{
    for (uint8_t i = 0; i < STEPPER_NUM; ++i)
        Stepper_StopPWM(&s_bind[i]);
}

void Stepper_Brake(uint8_t id)
{
    if (id >= STEPPER_NUM) return;
    Stepper_StopPWM(&s_bind[id]);
}