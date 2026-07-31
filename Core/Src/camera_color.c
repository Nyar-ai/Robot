/**
 * @file    camera_color.c
 * @brief   摄像头物块颜色检测实现(USART3 + DMA 异步请求-应答)
 *
 * 仿 camera_align.c 异步模式: 发送/接收均用 DMA, 任务在信号量上挂起等待,
 * 不占用 CPU, 不干扰 1ms 控制环.
 *
 * 帧格式见 camera_color.h.
 * HAL 回调(TxCpltCallback/RxCpltCallback/ErrorCallback)统一放在 usart.c 中,
 * 本模块通过 extern 暴露信号量和忙标志供 usart.c 使用.
 */
#include "camera_color.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* ==================== 内部状态 ==================== */

/* DMA 收/发完成信号量(任务与中断同步), 供 usart.c 的 HAL 回调释放 */
SemaphoreHandle_t g_color_tx_sem = NULL;
SemaphoreHandle_t g_color_rx_sem = NULL;

/* 标记一次收/发是否"正在进行"(避免误释放信号量), 供 usart.c 的 HAL 回调判断 */
volatile bool g_color_tx_busy = false;
volatile bool g_color_rx_busy = false;

/* 发送/接收缓冲(DMA 用, 静态分配) */
static uint8_t g_color_tx_buf[CAM_COLOR_REQ_LEN];
static uint8_t g_color_rx_buf[CAM_COLOR_ACK_LEN];

/* 最近一次解析结果(调试/日志用) */
static volatile uint8_t  g_last_color = 0;
static volatile uint16_t g_last_mx    = 0;
static volatile uint16_t g_last_my    = 0;

/* ==================== 对外接口 ==================== */

void camera_color_init(void)
{
    if (g_color_tx_sem == NULL) g_color_tx_sem = xSemaphoreCreateBinary();
    if (g_color_rx_sem == NULL) g_color_rx_sem = xSemaphoreCreateBinary();
    g_color_tx_busy = false;
    g_color_rx_busy = false;
    g_last_color = 0;
    g_last_mx    = 0;
    g_last_my    = 0;
}

bool camera_color_detect(uint8_t *out_color,
                         uint16_t *out_mx, uint16_t *out_my,
                         uint32_t timeout_ms)
{
    if (g_color_tx_sem == NULL || g_color_rx_sem == NULL) return false;
    if (timeout_ms == 0) timeout_ms = CAM_COLOR_DEFAULT_TIMEOUT_MS;

    /* 清空信号量(防止上一次残留, 循环清空确保无累积) */
    while (xSemaphoreTake(g_color_tx_sem, 0) == pdTRUE);
    while (xSemaphoreTake(g_color_rx_sem, 0) == pdTRUE);

    /* ---- 1. 组请求帧 ---- */
    g_color_tx_buf[0] = CAM_COLOR_FRAME_SOF0;
    g_color_tx_buf[1] = CAM_COLOR_FRAME_SOF1;
    g_color_tx_buf[2] = 0x00;

    /* ---- 2. 先启动 DMA 接收(收满 8 字节应答) ---- */
    g_color_rx_busy = true;
    HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&huart3, g_color_rx_buf, CAM_COLOR_ACK_LEN);
    if (st != HAL_OK) {
        g_color_rx_busy = false;
        return false;
    }

    /* ---- 3. 启动 DMA 发送请求 ---- */
    g_color_tx_busy = true;
    st = HAL_UART_Transmit_DMA(&huart3, g_color_tx_buf, CAM_COLOR_REQ_LEN);
    if (st != HAL_OK) {
        g_color_tx_busy = false;
        g_color_rx_busy = false;
        HAL_UART_AbortReceive(&huart3);
        return false;
    }

    /* ---- 4. 等待发送完成(DMA TC → HAL 回调 → 释放信号量) ---- */
    if (xSemaphoreTake(g_color_tx_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        g_color_tx_busy = false;
        g_color_rx_busy = false;
        HAL_UART_AbortTransmit(&huart3);
        HAL_UART_AbortReceive(&huart3);
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
        return false;
    }
    g_color_tx_busy = false;

    /* ---- 5. 等待接收完成(DMA 收满 → HAL 回调 → 释放信号量) ---- */
    bool rx_ok = (xSemaphoreTake(g_color_rx_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
    g_color_rx_busy = false;
    if (!rx_ok) {
        HAL_UART_AbortReceive(&huart3);
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
        return false;
    }

    /* ---- 6. 校验帧头 + XOR ---- */
    if (g_color_rx_buf[0] != CAM_COLOR_FRAME_SOF0 ||
        g_color_rx_buf[1] != CAM_COLOR_FRAME_SOF1) {
        return false;
    }
    uint8_t xor_calc = 0;
    for (int i = 0; i < CAM_COLOR_ACK_LEN - 1; ++i) xor_calc ^= g_color_rx_buf[i];
    if (xor_calc != g_color_rx_buf[CAM_COLOR_ACK_LEN - 1]) {
        return false;
    }

    /* ---- 7. 解析 COLOR + MX/MY (uint16 小端) ---- */
    uint8_t  color = g_color_rx_buf[2];
    uint16_t mx    = (uint16_t)g_color_rx_buf[3] | ((uint16_t)g_color_rx_buf[4] << 8);
    uint16_t my    = (uint16_t)g_color_rx_buf[5] | ((uint16_t)g_color_rx_buf[6] << 8);

    g_last_color = color;
    g_last_mx    = mx;
    g_last_my    = my;

    if (color == CAM_COLOR_NONE) {
        return false;   /* 摄像头明确报告"未检测到色块" */
    }

    if (out_color) *out_color = color;
    if (out_mx)    *out_mx    = mx;
    if (out_my)    *out_my    = my;

    return true;
}

void camera_color_get_last_raw(uint8_t *color,
                               uint16_t *mx, uint16_t *my)
{
    if (color) *color = g_last_color;
    if (mx)    *mx    = g_last_mx;
    if (my)    *my    = g_last_my;
}
