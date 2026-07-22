/**
 * @file    camera_align.c
 * @brief   摄像头黑色十字地标校准实现(USART2 + DMA 异步请求-应答)
 *
 * 仿陀螺仪异步模式: 发送/接收均用 DMA, 任务在信号量上挂起等待,
 * 不占用 CPU, 不干扰 1ms 控制环.
 *
 * 帧格式见 camera_align.h. 几何换算见 camera_align_pixel_to_world().
 */
#include "camera_align.h"
#include "usart.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <math.h>

/* ==================== 内部状态 ==================== */

/* DMA 收/发完成信号量(任务与中断同步) */
static SemaphoreHandle_t g_cam_tx_sem = NULL;
static SemaphoreHandle_t g_cam_rx_sem = NULL;

/* 标记一次收/发是否"正在进行"(避免误释放信号量) */
static volatile bool g_tx_busy = false;
static volatile bool g_rx_busy = false;

/* 发送/接收缓冲(DMA 用, 静态分配) */
static uint8_t g_tx_buf[CAM_REQ_LEN];
static uint8_t g_rx_buf[CAM_ACK_LEN];

/* 最近一次解析结果(调试/日志用) */
static volatile int16_t g_last_dx_px = 0;
static volatile int16_t g_last_dy_px = 0;
static volatile uint8_t g_last_status = 0;

/* ==================== 内部辅助 ==================== */

static float deg2rad(float d) { return d * (float)(3.14159265358979323846 / 180.0); }

/**
 * @brief 像素偏移 + 当前 θ → 反推车的真实世界坐标
 *
 * 几何模型:
 *   1. 摄像头视野中心相对车体原点的偏移(ebx, eby)由像素偏移 + 安装偏移得到
 *   2. 车体→世界旋转(与 chassis.c 里程计积分一致):
 *        off_wx = ebx*cos(θ) - eby*sin(θ)
 *        off_wy = ebx*sin(θ) + eby*cos(θ)
 *   3. 十字世界坐标 = 车位姿 + 世界系偏移
 *      => 真实车位姿 = 已知十字世界坐标 − 世界系偏移
 *
 * @param dx_px     十字相对图像中心的 x 像素偏移(右为正)
 * @param dy_px     十字相对图像中心的 y 像素偏移(下为正)
 * @param theta_deg 当前车体航向(信陀螺仪, deg)
 * @param out_x     输出: 反推的车世界系 x(mm)
 * @param out_y     输出: 反推的车世界系 y(mm)
 */
static void camera_align_recover_pose(int16_t dx_px, int16_t dy_px,
                                      float theta_deg,
                                      float *out_x, float *out_y)
{
    /* 像素 → 车体系 mm (注意像素 x↔车体y, 像素y↔车体x) */
    float ebx_mm = CAM_PIX_X_SIGN * ((float)dy_px * CAM_MM_PER_PIX_Y) + CAM_OFFSET_X_MM;
    float eby_mm = CAM_PIX_Y_SIGN * ((float)dx_px * CAM_MM_PER_PIX_X) + CAM_OFFSET_Y_MM;

    /* 车体 → 世界旋转(与 chassis.c 里程计一致) */
    float thr = deg2rad(theta_deg);
    float c = cosf(thr), s = sinf(thr);
    float off_wx =  ebx_mm * c - eby_mm * s;
    float off_wy =  ebx_mm * s + eby_mm * c;

    /* 十字世界坐标 = 车位姿 + 世界系偏移
     * => 车位姿 = 十字世界坐标 − 世界系偏移                (由调用方提供十字世界坐标) */
    (void)off_wx; (void)off_wy;
    /* 真正的换算在 camera_align_at 内完成(需要 cross_wx/cross_wy) */
    *out_x = off_wx;
    *out_y = off_wy;
}

/* ==================== 对外接口 ==================== */

void camera_align_init(void)
{
    if (g_cam_tx_sem == NULL) g_cam_tx_sem = xSemaphoreCreateBinary();
    if (g_cam_rx_sem == NULL) g_cam_rx_sem = xSemaphoreCreateBinary();
    g_tx_busy = false;
    g_rx_busy = false;
    g_last_dx_px = 0;
    g_last_dy_px = 0;
    g_last_status = 0;
}

bool camera_align_at(uint8_t id, float cross_wx, float cross_wy, uint32_t timeout_ms)
{
    if (g_cam_tx_sem == NULL || g_cam_rx_sem == NULL) return false;
    if (timeout_ms == 0) timeout_ms = CAM_DEFAULT_TIMEOUT_MS;

    /* 清空信号量(防止上一次残留, 循环清空确保无累积) */
    while (xSemaphoreTake(g_cam_tx_sem, 0) == pdTRUE);
    while (xSemaphoreTake(g_cam_rx_sem, 0) == pdTRUE);

    /* ---- 1. 组请求帧 ---- */
    g_tx_buf[0] = CAM_FRAME_SOF0;
    g_tx_buf[1] = CAM_FRAME_SOF1;
    g_tx_buf[2] = id;

    /* ---- 2. 先启动 DMA 接收(收满 8 字节应答) ---- */
    /* 在发送请求之前启动接收, 避免发送完成到接收启动之间的竞态窗口
     * 导致应答首字节丢失. USART2 全双工, TX/RX DMA 独立工作, 可同时进行. */
    g_rx_busy = true;
    HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&huart2, g_rx_buf, CAM_ACK_LEN);
    if (st != HAL_OK) {
        g_rx_busy = false;
        return false;
    }

    /* ---- 3. 启动 DMA 发送请求 ---- */
    g_tx_busy = true;
    st = HAL_UART_Transmit_DMA(&huart2, g_tx_buf, CAM_REQ_LEN);
    if (st != HAL_OK) {
        g_tx_busy = false;
        g_rx_busy = false;
        HAL_UART_AbortReceive(&huart2);
        return false;
    }

    /* ---- 4. 等待发送完成(DMA TC → HAL_UART_TxCpltCallback → 释放信号量) ---- */
    if (xSemaphoreTake(g_cam_tx_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        g_tx_busy = false;
        g_rx_busy = false;
        /* 发送超时, 中止 DMA 传输, 并清除 UART 残留错误标志,
         * 防止 HAL 因 ORE/FE/NE 未清除而在 Abort 内部卡死 */
        HAL_UART_AbortTransmit(&huart2);
        HAL_UART_AbortReceive(&huart2);
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
        return false;
    }
    g_tx_busy = false;

    /* ---- 5. 等待接收完成(DMA 收满 → HAL_UART_RxCpltCallback → 释放信号量) ---- */
    bool rx_ok = (xSemaphoreTake(g_cam_rx_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
    g_rx_busy = false;
    if (!rx_ok) {
        /* 接收超时, 中止 DMA 并清除 UART 残留错误标志 */
        HAL_UART_AbortReceive(&huart2);
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
        return false;
    }

    /* ---- 6. 校验帧头 + XOR ---- */
    if (g_rx_buf[0] != CAM_FRAME_SOF0 || g_rx_buf[1] != CAM_FRAME_SOF1) {
        return false;
    }
    uint8_t xor_calc = 0;
    for (int i = 0; i < CAM_ACK_LEN - 1; ++i) xor_calc ^= g_rx_buf[i];
    if (xor_calc != g_rx_buf[CAM_ACK_LEN - 1]) {
        return false;
    }

    /* ---- 7. 解析 STATUS + DX/DY(int16 小端) ---- */
    uint8_t status = g_rx_buf[2];
    int16_t dx_px = (int16_t)((uint16_t)g_rx_buf[3] | ((uint16_t)g_rx_buf[4] << 8));
    int16_t dy_px = (int16_t)((uint16_t)g_rx_buf[5] | ((uint16_t)g_rx_buf[6] << 8));

    g_last_status = status;
    g_last_dx_px  = dx_px;
    g_last_dy_px  = dy_px;

    if (status != CAM_ACK_STATUS_OK) {
        /* 摄像头明确报告"未检测到十字" */
        return false;
    }

    /* ---- 8. 像素偏移 → 车体系 mm → 世界系偏移 → 反推车位姿 ---- */
    float off_wx, off_wy;
    float x_now, y_now, theta_now;
    chassis_get_pose(&x_now, &y_now, &theta_now);   /* θ 仍信陀螺仪, 不改 */

    camera_align_recover_pose(dx_px, dy_px, theta_now, &off_wx, &off_wy);
    float car_x = cross_wx - off_wx;
    float car_y = cross_wy - off_wy;

    /* ---- 9. 原子注入 chassis(与 1ms chassisTask 读竞争保护) ---- */
    taskENTER_CRITICAL();
    chassis_set_pose(car_x, car_y, theta_now);
    taskEXIT_CRITICAL();

    return true;
}

void camera_align_get_last_raw(int16_t *dx_px, int16_t *dy_px, uint8_t *status)
{
    if (dx_px)   *dx_px   = g_last_dx_px;
    if (dy_px)   *dy_px   = g_last_dy_px;
    if (status)  *status  = g_last_status;
}

/* ==================== HAL UART 回调(运行在中断上下文, 保持简短) ==================== */

/* HAL 在 DMA 发送完成后调用本函数. 这里释放信号量唤醒 camera_align_at. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        BaseType_t hpw = pdFALSE;
        if (g_cam_tx_sem != NULL) {
            xSemaphoreGiveFromISR(g_cam_tx_sem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* HAL 在 DMA 接收收满后调用本函数. 这里释放信号量唤醒 camera_align_at. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        BaseType_t hpw = pdFALSE;
        if (g_cam_rx_sem != NULL) {
            xSemaphoreGiveFromISR(g_cam_rx_sem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* UART 错误(帧错/过冲/噪声等): 先清除硬件错误标志, 再释放信号量唤醒上层处理.
 * 关键: 必须先清除 ORE/FE/NE/PE 标志再释放信号量, 否则上层 Abort 时
 *       HAL 内部可能因检测到未清除的错误标志而陷入死循环. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* 1. 先清除 UART 硬件错误标志, 防止后续 Abort 操作中 HAL 状态机卡死 */
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);

        /* 2. 释放信号量唤醒上层任务, 让它走超时/重试路径 */
        BaseType_t hpw = pdFALSE;
        bool woke = false;
        if (g_rx_busy && g_cam_rx_sem != NULL) {
            xSemaphoreGiveFromISR(g_cam_rx_sem, &hpw);
            woke = true;
        }
        if (g_tx_busy && g_cam_tx_sem != NULL) {
            xSemaphoreGiveFromISR(g_cam_tx_sem, &hpw);
            woke = true;
        }
        if (woke) {
            portYIELD_FROM_ISR(hpw);
        }
    }
}
