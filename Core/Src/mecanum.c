/**
 * @file    mecanum.c
 * @brief   麦克纳姆轮(X 布局)运动学实现
 */
#include "mecanum.h"

/* 运行时可覆盖的几何参数(默认来自宏) */
static float g_lx  = MECANUM_LX_MM;
static float g_ly  = MECANUM_LY_MM;
static float g_R   = MECANUM_WHEEL_RADIUS_MM;

void Mecanum_SetGeometry(float lx_mm, float ly_mm, float wheel_r_mm)
{
    if (lx_mm > 0.0f) g_lx = lx_mm;
    if (ly_mm > 0.0f) g_ly = ly_mm;
    if (wheel_r_mm > 0.0f) g_R = wheel_r_mm;
}

void Mecanum_Inverse(float vx, float vy, float omega, float w[MECANUM_NUM])
{
    /* 应用左右方向极性(若整车 y 轴反向则取负) */
    vy = vy * MECANUM_FLIP_Y;

    /* (Lx+Ly) 单位 mm, omega 单位 rad/s → 该项单位 mm/s */
    float k = TURN_FLIP_Y*(g_lx + g_ly) * omega;

    /* X 布局(LF/RB 滚子 \, RF/LB 滚子 /) */
    w[MECANUM_LF] = vx - vy - k;
    w[MECANUM_RF] = vx + vy + k;
    w[MECANUM_LB] = vx + vy - k;
    w[MECANUM_RB] = vx - vy + k;
}

void Mecanum_Forward(const float w[MECANUM_NUM], float *vx, float *vy, float *omega)
{
    if (w == 0) {
        if (vx)    *vx = 0.0f;
        if (vy)    *vy = 0.0f;
        if (omega) *omega = 0.0f;
        return;
    }
    /* 逆解公式的反演(4 个方程 3 个未知数, 取最小二乘的等权平均) */
    float vxf = (w[MECANUM_LF] + w[MECANUM_RF] + w[MECANUM_LB] + w[MECANUM_RB]) * 0.25f;
    float vyf = (-w[MECANUM_LF] + w[MECANUM_RF] + w[MECANUM_LB] - w[MECANUM_RB]) * 0.25f;
    vyf *= MECANUM_FLIP_Y;
    float of = (-w[MECANUM_LF] + w[MECANUM_RF] - w[MECANUM_LB] + w[MECANUM_RB]) /
               (4.0f * (g_lx + g_ly));

    if (vx)    *vx = vxf;
    if (vy)    *vy = vyf;
    if (omega) *omega = of;
    (void)g_R; /* 线速度正/逆解不直接用轮径, 留给电机层换算 */
}