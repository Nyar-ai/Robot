/**
 * @file    mecanum.h
 * @brief   麦克纳姆轮(X 布局)运动学: 逆解 / 正解
 *
 * 约定:
 *   - 车体坐标系: x 向前, y 向左, z 向上; ω 绕 z 轴(逆时针为正)
 *   - w[4] 为 4 个轮子"沿驱动方向"的线速度(mm/s), 正方向=前进
 *   - 轮子编号顺序: [0]=左前LF [1]=右前RF [2]=左后LB [3]=右后RB
 *   - 几何参数 Lx(半轴距) / Ly(半轮距), 单位 mm
 *
 * X 布局逆解公式(不除以轮径, 直接输出线速度):
 *   vLF = vx - vy - (Lx+Ly)*ω
 *   vRF = vx + vy + (Lx+Ly)*ω
 *   vLB = vx + vy - (Lx+Ly)*ω
 *   vRB = vx - vy + (Lx+Ly)*ω
 *
 * 注: 若整车左右方向运动反向, 把 vy 项符号整体取反即可(见 MECANUM_FLIP_Y)。
 */
#ifndef __MECANUM_H
#define __MECANUM_H

#include <stdint.h>
#include <stddef.h>   /* NULL (ARMCC 5 不自动引入) */

/* 轮子编号 */
#define MECANUM_LF   0   /* 左前 */
#define MECANUM_RF   1   /* 右前 */
#define MECANUM_LB   2   /* 左后 */
#define MECANUM_RB   3   /* 右后 */
#define MECANUM_NUM  4

/* ---- 车体几何参数(按你的实车修改) ---- */
#define MECANUM_WHEEL_RADIUS_MM   37.5f    /* 轮半径(电机层换算步/秒时用) */
#define MECANUM_LX_MM             61.0f   /* 半轴距(中心到前后轮) */
#define MECANUM_LY_MM             86.0f   /* 半轮距(中心到左右轮) */

/* 若实测左右平移方向反了, 改为 -1.0f */
#define MECANUM_FLIP_Y            -1.0f
/* 若实测旋转方向反了, 改为 -1.0f */
#define TURN_FLIP_Y               -1.0f

/**
 * @brief 设置几何参数(不想用宏可运行时覆盖)
 */
void Mecanum_SetGeometry(float lx_mm, float ly_mm, float wheel_r_mm);

/**
 * @brief 逆解: 底盘速度 → 4 轮线速度
 * @param vx    车体系 x 速度 mm/s (前为正)
 * @param vy    车体系 y 速度 mm/s (左为正)
 * @param omega 车体系角速度 rad/s (逆时针为正)
 * @param w     输出 4 轮线速度 mm/s, 长度>=4
 */
void Mecanum_Inverse(float vx, float vy, float omega, float w[MECANUM_NUM]);

/**
 * @brief 正解: 4 轮线速度 → 底盘速度 (里程计用)
 * @param w     4 轮线速度 mm/s (通常用上一拍命令或编码器实测值)
 * @param vx    输出车体系 x 速度 mm/s
 * @param vy    输出车体系 y 速度 mm/s
 * @param omega 输出角速度 rad/s
 */
void Mecanum_Forward(const float w[MECANUM_NUM], float *vx, float *vy, float *omega);

#endif /* __MECANUM_H */