/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动 + DMA 异步读取 + yaw 纯积分
 *
 * 初始化序列沿用参考代码(STM32Project-有注释版/10-2 硬件I2C读写MPU6050):
 *   PWR_MGMT_1=0x01, PWR_MGMT_2=0x00, SMPLRT_DIV=0x09,
 *   CONFIG=0x06, GYRO_CONFIG=0x18, ACCEL_CONFIG=0x18
 *
 * DMA 异步读取流程:
 *   1) MPU6050_StartReadDMA()  → HAL_I2C_Mem_Read_DMA 启动, 立即返回
 *   2) DMA 完成中断 → HAL_I2C_MemRxCpltCallback → 置位 g_dma_done
 *   3) gyroTask 查询 MPU6050_IsReadComplete() 取数据
 */
#include "mpu6050.h"
#include "i2c.h"
#include <string.h>

/* ---- 内部状态 ---- */
static volatile bool    g_dma_busy;       /* DMA 是否在忙 */
static volatile bool    g_dma_done;       /* 本次 DMA 读取完成标志 */
static uint8_t          g_rx_buf[14];     /* DMA 接收缓冲(6轴+温度) */
static MPU6050_RawData  g_raw_last;       /* 最近一次解析的原始数据 */

static float g_yaw_deg;                   /* 累积 yaw 角(deg) */
static float g_yaw_rate;                  /* 当前 yaw 角速度(deg/s, 已去零漂+符号) */
static float g_bias_raw;                  /* yaw 轴零漂(原始 LSB) */

/* ---- 内部辅助 ---- */

/* 从 14 字节缓冲解析成结构体 */
static void parse_raw(const uint8_t *buf, MPU6050_RawData *r)
{
    r->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    r->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    r->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    r->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    r->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    r->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    r->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
}

/* 从原始数据取当前 yaw 轴的 LSB 值(按 MPU6050_YAW_USE_AXIS 选择) */
static int16_t get_yaw_axis_raw(const MPU6050_RawData *r)
{
#if   MPU6050_YAW_USE_AXIS == 0
    return r->gyro_x;
#elif MPU6050_YAW_USE_AXIS == 1
    return r->gyro_y;
#else
    return r->gyro_z;
#endif
}

/* 阻塞式读单个寄存器(初始化/校准用, 不走 DMA) */
static HAL_StatusTypeDef read_reg_blocking(uint8_t reg, uint8_t *val, uint16_t timeout)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_8BIT, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, timeout);
}

/* 阻塞式写单个寄存器(初始化用) */
static HAL_StatusTypeDef write_reg_blocking(uint8_t reg, uint8_t val, uint16_t timeout)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_8BIT, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, timeout);
}

/* 阻塞式连续读 N 字节(校准用) */
static HAL_StatusTypeDef read_bytes_blocking(uint8_t reg, uint8_t *buf, uint16_t len, uint16_t timeout)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_8BIT, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, timeout);
}

/* ---- 对外接口 ---- */

uint8_t MPU6050_Init(void)
{
    g_dma_busy = false;
    g_dma_done = false;
    g_yaw_deg = 0.0f;
    g_yaw_rate = 0.0f;
    g_bias_raw = 0.0f;
    memset(&g_raw_last, 0, sizeof(g_raw_last));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));

    /* 先读 WHO_AM_I 验证芯片在线 */
    uint8_t id = 0;
    if (read_reg_blocking(MPU6050_WHO_AM_I, &id, 100) != HAL_OK) return 1;
    if (id != 0x68) return 2;   /* MPU6050 应答 0x68 */

    /* 沿用参考代码的初始化序列 */
    if (write_reg_blocking(MPU6050_PWR_MGMT_1,   0x01, 100) != HAL_OK) return 3;  /* 取消睡眠, X陀螺时钟 */
    HAL_Delay(10);   /* 等电源稳定 */
    if (write_reg_blocking(MPU6050_PWR_MGMT_2,   0x00, 100) != HAL_OK) return 4;  /* 所有轴不待机 */
    if (write_reg_blocking(MPU6050_SMPLRT_DIV,   0x09, 100) != HAL_OK) return 5;  /* 采样率分频 */
    if (write_reg_blocking(MPU6050_CONFIG,       0x06, 100) != HAL_OK) return 6;  /* DLPF */
    if (write_reg_blocking(MPU6050_GYRO_CONFIG,  0x18, 100) != HAL_OK) return 7;  /* ±2000°/s */
    if (write_reg_blocking(MPU6050_ACCEL_CONFIG, 0x18, 100) != HAL_OK) return 8;  /* ±16g */

    return 0;
}

void MPU6050_CalibrateYaw(uint16_t sample_count)
{
    uint8_t buf[14];
    float sum = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < sample_count; ++i)
    {
        if (read_bytes_blocking(MPU6050_ACCEL_XOUT_H, buf, 14, 50) == HAL_OK)
        {
            MPU6050_RawData r;
            parse_raw(buf, &r);
            sum += (float)get_yaw_axis_raw(&r);
            valid++;
        }
        HAL_Delay(2);   /* 间隔 2ms, 避免连续读太快 */
    }

    if (valid > 0) g_bias_raw = sum / valid;   /* 原始 LSB 零漂 */
}

uint8_t MPU6050_StartReadDMA(void)
{
    if (g_dma_busy) return 1;   /* 上一次还没读完 */

    g_dma_done = false;
    g_dma_busy = true;

    /* 从 0x3B(ACCEL_XOUT_H) 连续读 14 字节, DMA 异步 */
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(
        &hi2c1, MPU6050_ADDR_8BIT, MPU6050_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT, g_rx_buf, 14);

    if (st != HAL_OK) {
        g_dma_busy = false;
        return 2;
    }
    return 0;
}

bool MPU6050_IsReadComplete(void)
{
    return g_dma_done;
}

void MPU6050_GetRaw(MPU6050_RawData *raw)
{
    if (raw) *raw = g_raw_last;
}

float MPU6050_GetYawRate(void)
{
    return g_yaw_rate;
}

float MPU6050_GetYaw(void)
{
    return g_yaw_deg;
}

void MPU6050_SetYaw(float yaw_deg)
{
    g_yaw_deg = yaw_deg;
}

void MPU6050_IntegrateYaw(float dt)
{
    /* 由 DMA 完成后解析的最新数据驱动 */
    g_yaw_deg += g_yaw_rate * dt;
    /* 归一到 (-180, 180] */
    while (g_yaw_deg > 180.0f)  g_yaw_deg -= 360.0f;
    while (g_yaw_deg <= -180.0f) g_yaw_deg += 360.0f;
}

/* ---- DMA 完成回调(在 HAL 内部被调用, 运行在 I2C1_EV 中断上下文) ----
 * 这是关键的"异步通知"点: DMA 把 14 字节搬完后, HAL 会调本函数,
 * 在这里解析数据 + 置完成标志. gyroTask 查到标志即可取最新 yaw_rate. */
void MPU6050_OnDMAComplete(void)
{
    parse_raw(g_rx_buf, &g_raw_last);

    /* 计算 yaw 角速度(去零漂 + 乘灵敏度倒数 + 乘符号) */
    int16_t raw = get_yaw_axis_raw(&g_raw_last);
    float rate_dps = ((float)raw - g_bias_raw) / MPU6050_GYRO_SENSITIVITY;
    g_yaw_rate = rate_dps * MPU6050_YAW_SIGN;

    g_dma_done = true;
    g_dma_busy = false;
}