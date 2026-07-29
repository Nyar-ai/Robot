/**
 * @file    servo.c
 * @brief   舵机驱动层实现 (TIM2 PWM 脉宽控制)
 *
 * 数据流:
 *   clamp_gripper_open/close/rotate_set
 *          │
 *          └── Servo_SetAngle(id, deg)
 *                │
 *                └── deg→CCR → __HAL_TIM_SET_COMPARE(htim2, channel, ccr)
 *
 * 角度-脉宽映射 (按 id 区分):
 *   夹爪 (SERVO_GRIPPER):  0-180deg  → CCR 500-2500  (斜率 2000/180)
 *   旋转 (SERVO_ROTATE):   0-270deg  → CCR 500-2500  (斜率 2000/270)
 */
#include "servo.h"
#include "tim.h"
#include "clamp.h"

/* ---- 每个舵机的硬件绑定 ---- */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} Servo_Bind;

static const Servo_Bind s_servo[SERVO_NUM] = {
    /* 夹爪: PA0 / TIM2_CH1 */
    { &htim2, TIM_CHANNEL_1 },
    /* 旋转: PA2 / TIM2_CH3 */
    { &htim2, TIM_CHANNEL_3 },
};

/* ---- 各舵机角度上限 ---- */
static const uint16_t s_max_deg[SERVO_NUM] = {
    SERVO_MAX_DEG_GRIPPER,   /* 夹爪 180deg */
    SERVO_MAX_DEG_ROTATE,    /* 旋转 270deg */
};

/* ---- 对外接口 ---- */

void Servo_Init(void)
{
    static bool inited = false;
    if (inited) return;
    inited = true;

    /* TIM2 时钟 84MHz (APB1)
     * PSC=83 → 1MHz (1us/tick)
     * ARR=19999 → 20ms 周期 (50Hz 舵机标准)
     * 必须生成更新事件使 PSC/ARR 从影子寄存器立即生效 */
    __HAL_TIM_SET_PRESCALER(&htim2, 83);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 19999);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

    /* 启动 PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    /* 初始安全角度 */
    Servo_SetAngle(SERVO_GRIPPER, 180);   /* 夹爪张开 */
    Servo_SetAngle(SERVO_ROTATE,   CLAMP_ROTATE_ZERO_DEG);  /* 旋转起始方向 (逻辑零点) */
}

void Servo_SetAngle(uint8_t id, uint16_t deg)
{
    if (id >= SERVO_NUM) return;

    uint16_t max_deg = s_max_deg[id];
    if (deg > max_deg) deg = max_deg;

    /* CCR = 500 + deg * 2000 / max_deg
     * max_deg = 180  → 斜率 2000/180 ≈ 11.11 us/deg
     * max_deg = 270  → 斜率 2000/270 ≈  7.41 us/deg */
    uint32_t ccr = SERVO_MIN_PULSE_US + (uint32_t)deg * 2000UL / max_deg;
    __HAL_TIM_SET_COMPARE(s_servo[id].htim, s_servo[id].channel, ccr);
}

void Servo_SetPulseUs(uint8_t id, uint16_t us)
{
    if (id >= SERVO_NUM) return;
    if (us < SERVO_MIN_PULSE_US) us = SERVO_MIN_PULSE_US;
    if (us > SERVO_MAX_PULSE_US) us = SERVO_MAX_PULSE_US;

    __HAL_TIM_SET_COMPARE(s_servo[id].htim, s_servo[id].channel, us);
}