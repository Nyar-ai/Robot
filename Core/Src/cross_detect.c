/**
 * @file    cross_detect.c
 * @brief   十字检测纠偏模块实现 — 非阻塞 DMA+空闲中断+信号量
 *
 * 与 K230 双向通讯流程:
 *   1. CrossDetect_Calibrate() 启动 HAL_UART_Receive_DMA
 *   2. 发 "DETECT\r\n" → K230 连拍 10 帧 → 回复 "CROSS,dx,dy,conf\r\n"
 *   3. DMA 后台搬运字节到 g_rx_buf (完全不占 CPU)
 *   4. 帧结束后 USART 空闲中断触发 → CrossDetect_UART_IDLE_IRQHandler
 *   5. 停止 DMA, 读取接收长度, 发信号量唤醒 defaultTask
 *   6. defaultTask 被唤醒, 解析响应帧 → 航向补偿 → chassis_set_pose
 *
 * 在等待期间 defaultTask 真正挂起(xSemaphoreTake), CPU 100% 让给控制环.
 */
#include "cross_detect.h"
#include "chassis.h"
#include "usart.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ---- 内部状态 ------------------------------------------------------------- */

/* USART2 DMA 接收缓冲 */
static uint8_t  g_rx_buf[CROSS_UART_RX_BUF_SIZE];

/* 空闲中断设置的标志 (volatile — ISR 写入, task 读取) */
static volatile bool    g_frame_ready;
static volatile uint8_t g_frame_len;

/* FreeRTOS 信号量 (由 freertos.c 创建后通过 CrossDetect_SetSemaphore 注入) */
static SemaphoreHandle_t g_sem = NULL;

/* 十字地图 */
static CrossPoint_t g_map[CROSS_MAP_MAX];
static uint8_t      g_map_count;

/* ---- 内部辅助 ------------------------------------------------------------- */

static float deg2rad(float d) { return d * (float)(3.14159265358979323846 / 180.0); }

static const CrossPoint_t *find_cross(uint8_t id)
{
    for (uint8_t i = 0; i < g_map_count; ++i) {
        if (g_map[i].id == id) return &g_map[i];
    }
    return NULL;
}

/* 发送字符串到 USART2 (阻塞, 但消息很短 <10 字节) */
static void uart_tx(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
}

/* 解析 "CROSS,dx,dy,conf" 或 "NOCROSS"
 * 返回: 0=无十字, 1=有十字(填充 dx/dy/conf), -1=解析失败 */
static int parse_line(const uint8_t *buf, uint8_t len, int *dx, int *dy, int *conf)
{
    if (buf == NULL || len < 3) return -1;

    /* 临时复制一份加 '\0' 方便 sscanf */
    char tmp[50];
    uint8_t n = (len < sizeof(tmp) - 1) ? len : (uint8_t)(sizeof(tmp) - 1);
    memcpy(tmp, buf, n);
    tmp[n] = '\0';

    /* NOCROSS */
    if (strncmp(tmp, "NOCROSS", 7) == 0) return 0;

    /* CROSS,dx,dy,conf */
    if (strncmp(tmp, "CROSS,", 6) == 0) {
        if (sscanf(tmp, "CROSS,%d,%d,%d", dx, dy, conf) == 3) return 1;
    }
    return -1;
}

/* ---- 对外接口 ------------------------------------------------------------- */

void CrossDetect_Init(void)
{
    g_map_count    = 0;
    g_frame_ready  = false;
    g_frame_len    = 0;
    memset(g_rx_buf, 0, sizeof(g_rx_buf));

    /* 注册默认十字地图 (按实际测量修改坐标) */
    CrossDetect_AddPoint(1,    0.0f,    0.0f);   /* 原点 */
    CrossDetect_AddPoint(2,  500.0f,    0.0f);   /* 十字2 */
    CrossDetect_AddPoint(3,  500.0f,  500.0f);   /* 十字3 */
    CrossDetect_AddPoint(4,    0.0f,  500.0f);   /* 十字4 */
}

void CrossDetect_SetSemaphore(SemaphoreHandle_t sem)
{
    g_sem = sem;
}

bool CrossDetect_AddPoint(uint8_t id, float x_mm, float y_mm)
{
    if (g_map_count >= CROSS_MAP_MAX) return false;
    g_map[g_map_count].id   = id;
    g_map[g_map_count].x_mm = x_mm;
    g_map[g_map_count].y_mm = y_mm;
    g_map_count++;
    return true;
}

void CrossDetect_ClearMap(void)
{
    g_map_count = 0;
}

bool CrossDetect_Calibrate(uint8_t cross_id)
{
    const CrossPoint_t *cp = find_cross(cross_id);
    if (cp == NULL) return false;

    /* 确认底盘已停稳 */
    if (!chassis_is_idle()) return false;

    /* 确认信号量已注入 */
    if (g_sem == NULL) return false;

    for (int retry = 0; retry < CROSS_RETRY_MAX; retry++) {

        /* ---- 1. 启动 DMA 接收 + 发送 DETECT 请求 ---- */
        g_frame_ready = false;
        g_frame_len   = 0;
        memset(g_rx_buf, 0, sizeof(g_rx_buf));

        HAL_UART_Receive_DMA(&huart2, g_rx_buf, CROSS_UART_RX_BUF_SIZE);
        uart_tx("DETECT\r\n");

        /* ---- 2. 挂起等待信号量 (DMA 搬运 + IDLE 中断唤醒) ----
         * 这里 defaultTask 真正挂起, CPU 100% 让给 chassisTask/gyroTask.
         * 超时 500ms 防止 K230 无响应导致永久阻塞. */
        if (xSemaphoreTake(g_sem, pdMS_TO_TICKS(CROSS_DETECT_TIMEOUT_MS)) != pdTRUE) {
            /* 超时: 停止 DMA, 重试 */
            HAL_UART_DMAStop(&huart2);
            continue;
        }

        /* ---- 3. 被唤醒, 检查帧是否有效 ---- */
        /* 停止 DMA (如果空闲中断还没停) */
        HAL_UART_DMAStop(&huart2);

        if (!g_frame_ready || g_frame_len == 0) {
            /* 被打断或异常, 短暂等后重试 */
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        /* ---- 4. 解析响应 ---- */
        int dx, dy, conf;
        int result = parse_line(g_rx_buf, g_frame_len, &dx, &dy, &conf);

        if (result == 0) {
            /* NOCROSS → 短暂等后重试 */
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        if (result < 0) {
            /* 解析失败 → 重试 */
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        /* ---- 5. 偏差合理性检查 ---- */
        if (dx > CROSS_MAX_OFFSET_PX || dx < -CROSS_MAX_OFFSET_PX ||
            dy > CROSS_MAX_OFFSET_PX || dy < -CROSS_MAX_OFFSET_PX) {
            uart_tx("RETRY\r\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* ---- 6. 像素偏差 → 物理偏差 → 车体坐标系偏差 (mm) ----
         * 摄像头画面映射到车体 (X+前, Y+左):
         *   dx>0→十字在画面右→车在十字左→车体Y为负
         *   dy>0→十字在画面下→车在十字后→车体X为负
         *   因此: body_x(前) = -dy_mm,  body_y(左) = -dx_mm */
        float dx_mm = (float)dx * CROSS_PX_TO_MM;
        float dy_mm = (float)dy * CROSS_PX_TO_MM;
        float body_x = -dy_mm;
        float body_y = -dx_mm;

        /* ---- 7. 航向补偿: 车体系偏差 → 世界系偏差 ----
         * ΔXw = body_x·cosθ - body_y·sinθ
         * ΔYw = body_x·sinθ + body_y·cosθ
         * 车世界坐标 = 十字坐标 - 世界系偏移 */
        float theta_deg;
        chassis_get_pose(NULL, NULL, &theta_deg);
        float theta_rad = deg2rad(theta_deg);
        float cos_t = cosf(theta_rad);
        float sin_t = sinf(theta_rad);
        float dx_w = body_x * cos_t - body_y * sin_t;
        float dy_w = body_x * sin_t + body_y * cos_t;

        float x_corrected = cp->x_mm - dx_w;
        float y_corrected = cp->y_mm - dy_w;

        /* ---- 8. 覆盖底盘位姿 (θ 不变) ---- */
        chassis_set_pose(x_corrected, y_corrected, theta_deg);

        /* ---- 9. 发送 OK 确认 ---- */
        uart_tx("OK\r\n");

        return true;
    }

    return false;
}

/* ---- 中断服务入口 (在 USART2_IRQHandler 的 USER CODE 区域调用) ---- */

void CrossDetect_UART_IDLE_IRQHandler(void)
{
    /* 检测空闲中断 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        /* 清空闲标志 (读 SR 后读 DR) */
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        /* 停止 DMA, 获取已接收字节数 */
        HAL_UART_DMAStop(&huart2);

        uint16_t rx_len = CROSS_UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
        if (rx_len > 0 && rx_len <= CROSS_UART_RX_BUF_SIZE) {
            g_frame_len  = (uint8_t)rx_len;
            g_frame_ready = true;
        }

        /* 唤醒 defaultTask */
        if (g_sem != NULL) {
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(g_sem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}