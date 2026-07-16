/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动 + DMA 异步读取 + yaw 纯积分(多层零漂抑制)
 *
 * 零漂抑制策略(分层, 见 mpu6050.h 宏配置):
 *   1) DLPF 降噪(CONFIG=DLPF_CFG) + 1kHz 采样匹配: 大幅降低角度随机游走
 *   2) 上电校准: 预热丢弃 + 3σ 去异常值 + 截尾均值, 记录校准温度基准
 *   3) 死区: |角速度|<阈值 视为0, 掐断残余零漂的纯积分
 *   4) 温度补偿: bias(T)=bias0 + k·(T-T0), k 默认0(标定后填值)
 *   5) 运行期自适应: 滑窗判静止 → 大τ一阶低通跟踪零偏(仅静止时更新)
 *   6) 静止自动重校准: 持续静止≥N ms 用窗口均值覆盖零偏并刷新温度基准
 *
 * 初始化序列:
 *   PWR_MGMT_1=0x01, PWR_MGMT_2=0x00, SMPLRT_DIV=0x00,
 *   CONFIG=DLPF_CFG(默认3=44Hz), GYRO_CONFIG=0x18, ACCEL_CONFIG=0x18
 *
 * DMA 异步读取流程:
 *   1) MPU6050_StartReadDMA()  → HAL_I2C_Mem_Read_DMA 启动, 立即返回
 *   2) DMA 完成中断 → HAL_I2C_MemRxCpltCallback → 置位 g_dma_done
 *   3) gyroTask 查询 MPU6050_IsReadComplete() 取数据
 */
#include "mpu6050.h"
#include "i2c.h"
#include <string.h>
#include <math.h>

/* ---- 内部状态 ---- */
static volatile bool    g_dma_busy;       /* DMA 是否在忙 */
static volatile bool    g_dma_done;       /* 本次 DMA 读取完成标志 */
static uint8_t          g_rx_buf[14];     /* DMA 接收缓冲(6轴+温度) */
static MPU6050_RawData  g_raw_last;       /* 最近一次解析的原始数据 */

static float g_yaw_deg;                   /* 累积 yaw 角(deg) */
static float g_yaw_rate;                  /* 当前 yaw 角速度(deg/s, 已去零漂+死区+符号) */
static volatile float g_bias_raw;         /* yaw 轴零漂(原始 LSB), task/中断共享 → volatile */
static float g_bias_temp_c;               /* 校准/重校准时的温度基准(°C) */
static float g_temp_c;                    /* 最近一次片上温度(°C) */

/* 静止检测环形缓冲(存去偏角速度 deg/s) + 增量统计 */
static float    g_ring[MPU6050_STATIC_WINDOW];
static uint16_t g_ring_head;
static uint16_t g_ring_count;
static float    g_ring_sum;
static float    g_ring_sumsq;
static volatile bool g_is_static;         /* 当前是否判为静止 */
static uint32_t g_static_ms;              /* 连续静止累计(ms, @1kHz 即样本数) */

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

/* 温度换算: T(°C) = raw/340 + 36.53 (MPU6050 数据手册公式) */
static float raw_to_temp_c(int16_t raw)
{
    return (float)raw / 340.0f + 36.53f;
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

/* 读取片上温度(阻塞, 校准/重校准用); 失败回退到基准温度避免补偿错乱 */
static float read_temp_blocking(void)
{
    uint8_t buf[2];
    if (read_bytes_blocking(MPU6050_TEMP_OUT_H, buf, 2, 50) == HAL_OK)
        return raw_to_temp_c((int16_t)((buf[0] << 8) | buf[1]));
    return g_bias_temp_c;
}

/* 对一组 yaw 轴 LSB 样本做"3σ 剔除"后重算均值/方差(鲁棒估计, 抑制异常样本)
 * 用 (x-mean)^2 > 9·var 等价判定 |x-mean|>3σ, 避免调用 sqrtf, 更快且无库依赖 */
static void robust_mean_var(const float *arr, uint16_t n,
                            float *out_mean, float *out_var)
{
    /* 第一趟: 全样本均值与方差 */
    float sum = 0.0f, sumsq = 0.0f;
    for (uint16_t i = 0; i < n; ++i) { sum += arr[i]; sumsq += arr[i] * arr[i]; }
    float mean = sum / n;
    float var  = sumsq / n - mean * mean;
    if (var < 0.0f) var = 0.0f;
    float thresh = 9.0f * var;   /* (3σ)^2 */

    /* 第二趟: 剔除 (x-mean)^2>9σ^2 后重算 */
    sum = 0.0f; sumsq = 0.0f; uint16_t m = 0;
    for (uint16_t i = 0; i < n; ++i) {
        float d = arr[i] - mean;
        if (d * d > thresh) continue;
        sum += arr[i]; sumsq += arr[i] * arr[i]; ++m;
    }
    if (m == 0) { *out_mean = mean; *out_var = var; return; }   /* 全被剔(极端), 回退 */
    *out_mean = sum / m;
    *out_var  = sumsq / m - (*out_mean) * (*out_mean);
    if (*out_var < 0.0f) *out_var = 0.0f;
}

/* 清空运行期滑窗与静止累计(在 task 端临界区内调用) */
static void reset_runtime_window(void)
{
    g_ring_head   = 0;
    g_ring_count  = 0;
    g_ring_sum    = 0.0f;
    g_ring_sumsq  = 0.0f;
    g_static_ms   = 0;
    g_is_static   = false;
}

/* ---- 对外接口 ---- */

uint8_t MPU6050_Init(void)
{
    g_dma_busy = false;
    g_dma_done = false;
    g_yaw_deg = 0.0f;
    g_yaw_rate = 0.0f;
    g_bias_raw = 0.0f;
    g_bias_temp_c = 25.0f;
    g_temp_c = 25.0f;
    reset_runtime_window();
    memset(&g_raw_last, 0, sizeof(g_raw_last));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));

    /* 先读 WHO_AM_I 验证芯片在线 */
    uint8_t id = 0;
    if (read_reg_blocking(MPU6050_WHO_AM_I, &id, 100) != HAL_OK) return 1;
    if (id != 0x68) return id;   /* MPU6050 应答 0x68(HAL 左移后比较位), 兼容旧判断 */

    /* 初始化序列: DLPF 开启降噪, 1kHz 采样匹配 1ms 任务 */
    if (write_reg_blocking(MPU6050_PWR_MGMT_1,   0x01, 100) != HAL_OK) return 3;  /* 取消睡眠, X陀螺时钟 */
    HAL_Delay(10);   /* 等电源稳定 */
    if (write_reg_blocking(MPU6050_PWR_MGMT_2,   0x00, 100) != HAL_OK) return 4;  /* 所有轴不待机 */
    if (write_reg_blocking(MPU6050_SMPLRT_DIV,   0x00, 100) != HAL_OK) return 5;  /* 1kHz(DLPF≠0时基带1kHz) */
    if (write_reg_blocking(MPU6050_CONFIG,       MPU6050_DLPF_CFG, 100) != HAL_OK) return 6;  /* DLPF 降噪 */
    if (write_reg_blocking(MPU6050_GYRO_CONFIG,  0x18, 100) != HAL_OK) return 7;  /* ±2000°/s */
    if (write_reg_blocking(MPU6050_ACCEL_CONFIG, 0x18, 100) != HAL_OK) return 8;  /* ±16g */

    return 0;
}

void MPU6050_CalibrateYaw(uint16_t sample_count)
{
    static float buf[MPU6050_CALIB_BUF];   /* 静态分配, 避免占栈 */
    uint8_t raw[14];
    uint16_t idx = 0;

    if (sample_count > MPU6050_CALIB_BUF) sample_count = MPU6050_CALIB_BUF;
    if (sample_count == 0) return;

    /* 预热: 丢弃上电瞬态样本 */
    for (uint16_t i = 0; i < MPU6050_CALIB_WARMUP; ++i) {
        (void)read_bytes_blocking(MPU6050_ACCEL_XOUT_H, raw, 14, 50);
        HAL_Delay(2);
    }

    /* 采集 */
    for (uint16_t i = 0; i < sample_count; ++i) {
        if (read_bytes_blocking(MPU6050_ACCEL_XOUT_H, raw, 14, 50) == HAL_OK) {
            MPU6050_RawData r;
            parse_raw(raw, &r);
            buf[idx++] = (float)get_yaw_axis_raw(&r);
        }
        HAL_Delay(2);
    }
    if (idx == 0) return;

    /* 3σ 剔除后取均值 */
    float mean, var;
    robust_mean_var(buf, idx, &mean, &var);

    /* 临界区写入共享零偏 + 记录温度基准 + 清滑窗 */
    float t = read_temp_blocking();
    __disable_irq();
    g_bias_raw     = mean;
    g_bias_temp_c  = t;
    reset_runtime_window();
    __enable_irq();
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

float MPU6050_GetTempC(void)
{
    return g_temp_c;
}

bool MPU6050_IsStatic(void)
{
    return g_is_static;
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

void MPU6050_ForceRecalibrate(void)
{
    static float buf[MPU6050_STATIC_WINDOW];   /* 静态分配 */
    uint8_t raw[14];
    uint16_t idx = 0;

    for (uint16_t i = 0; i < MPU6050_STATIC_WINDOW; ++i) {
        if (read_bytes_blocking(MPU6050_ACCEL_XOUT_H, raw, 14, 50) == HAL_OK) {
            MPU6050_RawData r;
            parse_raw(raw, &r);
            buf[idx++] = (float)get_yaw_axis_raw(&r);
        }
        HAL_Delay(2);
    }
    if (idx == 0) return;

    float mean, var;
    robust_mean_var(buf, idx, &mean, &var);

    float t = read_temp_blocking();
    __disable_irq();
    g_bias_raw     = mean;
    g_bias_temp_c  = t;
    reset_runtime_window();
    __enable_irq();
}

/* ---- DMA 完成回调(在 HAL 内部被调用, 运行在 I2C1_EV 中断上下文) ----
 * 这是关键的"异步通知"点: DMA 把 14 字节搬完后, HAL 会调本函数,
 * 在这里解析数据 + 零漂抑制 + 置完成标志. gyroTask 查到标志即可取最新 yaw_rate. */
void MPU6050_OnDMAComplete(void)
{
    parse_raw(g_rx_buf, &g_raw_last);

    int16_t raw = get_yaw_axis_raw(&g_raw_last);
    g_temp_c = raw_to_temp_c(g_raw_last.temp);

    /* 有效零偏 = 基准零偏 + 温度补偿(默认 k=0 不生效, 标定后启用) */
    float bias_eff = g_bias_raw + MPU6050_BIAS_TEMP_COEF * (g_temp_c - g_bias_temp_c);

    /* 去偏角速度(死区前), 用于滑窗统计/静止判定 */
    float rate_dps = ((float)raw - bias_eff) / MPU6050_GYRO_SENSITIVITY;

    /* 推入环形缓冲(去偏角速度域) + 增量统计 */
    if (g_ring_count < MPU6050_STATIC_WINDOW) {
        g_ring[g_ring_head] = rate_dps;
        g_ring_sum   += rate_dps;
        g_ring_sumsq += rate_dps * rate_dps;
        g_ring_head = (uint16_t)((g_ring_head + 1) % MPU6050_STATIC_WINDOW);
        g_ring_count++;
    } else {
        float old = g_ring[g_ring_head];
        g_ring_sum   -= old;
        g_ring_sumsq -= old * old;
        g_ring[g_ring_head] = rate_dps;
        g_ring_sum   += rate_dps;
        g_ring_sumsq += rate_dps * rate_dps;
        g_ring_head = (uint16_t)((g_ring_head + 1) % MPU6050_STATIC_WINDOW);
    }

    /* 缓冲满则判静止 + 自适应/重校准 */
    if (g_ring_count >= MPU6050_STATIC_WINDOW) {
        float n    = (float)MPU6050_STATIC_WINDOW;
        float mean = g_ring_sum / n;
        float var  = g_ring_sumsq / n - mean * mean;
        if (var < 0.0f) var = 0.0f;

        bool is_static = (fabsf(mean) < MPU6050_STATIC_RATE_THR) &&
                         (var         < MPU6050_STATIC_VAR_THR);
        g_is_static = is_static;

        if (is_static) {
            if (g_static_ms < 0xFFFFFFFF) g_static_ms++;

            /* 自适应低通更新零偏(LSB 域): 仅静止时缓慢跟踪温漂/慢漂
             * raw_avg_lsb = bias + mean_dps·sens;  bias += α·(raw_avg_lsb - bias) */
            float alpha = 0.001f / MPU6050_BIAS_TRACK_TAU;   /* dt=1ms */
            if (alpha > 1.0f) alpha = 1.0f;
            float raw_avg_lsb = g_bias_raw + mean * MPU6050_GYRO_SENSITIVITY;
            g_bias_raw += alpha * (raw_avg_lsb - g_bias_raw);

            /* 静止自动重校准: 累计达阈值则硬覆盖 + 刷新温度基准 + 清累计 */
            if (g_static_ms >= MPU6050_RECALIB_STATIC_MS) {
                g_bias_raw    = raw_avg_lsb;
                g_bias_temp_c = g_temp_c;
                g_static_ms   = 0;
            }
        } else {
            g_static_ms = 0;
        }
    }

    /* 死区: 输出给积分的角速度(注意: 滑窗用死区前的 rate_dps, 否则静止判定失真) */
    if (fabsf(rate_dps) < MPU6050_DEADZONE_DPS) rate_dps = 0.0f;
    g_yaw_rate = rate_dps * MPU6050_YAW_SIGN;

    g_dma_done = true;
    g_dma_busy = false;
}
/* 文件末尾保留换行 */
