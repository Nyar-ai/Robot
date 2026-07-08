/**
 * @file    mpu6050.h
 * @brief   MPU6050 陀螺仪驱动(I2C + DMA 异步读取) + yaw 纯积分
 *
 * 硬件:
 *   - I2C1: PB8=SCL, PB9=SDA, 400kHz, DMA 已配(hdma_i2c1_rx/tx)
 *   - MPU6050 地址: 0x68 (AD0 接地), HAL 8位格式 = 0xD0
 *
 * 姿态方案:
 *   - 纯陀螺积分 yaw = Σ(gyro_z · dt) (MPU6050 加速度计测不了 yaw, 不做互补滤波)
 *   - 上电静止校准零漂(读 N 次取平均)
 *   - 水平贴装(+X 指车头) → 用 GYRO_Z, 符号 +1
 *
 * 与 chassis 对接: gyroTask 周期调 chassis_feed_gyro(yaw_deg)
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- 寄存器地址(沿用参考代码 MPU6050_Reg.h) ---- */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_WHO_AM_I        0x75

/* ---- I2C 地址 ---- */
#define MPU6050_ADDR_8BIT       0xD0   /* 7位 0x68 左移1位, HAL 用 8 位格式 */

/* ---- 轴选择(按安装方向) ---- */
/* 水平贴装 +X 指车头: yaw = GYRO_Z, 符号 +1 (逆时针自旋为正) */
#define MPU6050_YAW_USE_AXIS   2       /* 0=X, 1=Y, 2=Z */
#define MPU6050_YAW_SIGN       1.0f    /* 实测若 yaw 递减则改 -1.0f */

/* ---- 量程灵敏度 ---- */
/* GYRO_CONFIG=0x18 → ±2000°/s, 灵敏度 16.4 LSB/(°/s) */
#define MPU6050_GYRO_SENSITIVITY   16.4f

/* ---- 一次读取的数据块(从 0x3B 起 14 字节) ---- */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_RawData;

/**
 * @brief 初始化 MPU6050(沿用参考代码寄存器序列)
 * @return 0=成功, 非0=失败(读 WHO_AM_I 不对)
 */
uint8_t MPU6050_Init(void);

/**
 * @brief 上电静止校准 yaw 零漂(读 N 次取平均)
 * @note  调用时车必须静止! 会阻塞约 N×(单次读耗时)
 */
void MPU6050_CalibrateYaw(uint16_t sample_count);

/**
 * @brief 启动一次 DMA 异步读取(非阻塞, 立即返回)
 * @return 0=已启动, 非0=上一次还没读完或出错
 */
uint8_t MPU6050_StartReadDMA(void);

/**
 * @brief DMA 读取是否完成(供 gyroTask 轮询, 或用信号量等待)
 */
bool MPU6050_IsReadComplete(void);

/**
 * @brief 取最近一次 DMA 读到的原始数据(含 6 轴 + 温度)
 */
void MPU6050_GetRaw(MPU6050_RawData *raw);

/**
 * @brief 取当前陀螺 yaw 角速度(deg/s, 已去零漂, 已乘符号)
 */
float MPU6050_GetYawRate(void);

/**
 * @brief 取当前累积的 yaw 角(deg, 已归一到 [-180,180])
 */
float MPU6050_GetYaw(void);

/**
 * @brief yaw 积分一步(由 gyroTask 周期调用)
 * @param dt 步长(秒), 例如 0.001f
 */
void MPU6050_IntegrateYaw(float dt);

/**
 * @brief 强制设置 yaw(用于校准/重置)
 */
void MPU6050_SetYaw(float yaw_deg);

#endif /* __MPU6050_H */