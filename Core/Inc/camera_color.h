/**
 * @file    camera_color.h
 * @brief   摄像头物块颜色检测模块(USART3 + DMA, 异步请求-应答模式)
 *
 * 设计目标:
 *   仿照 camera_align(十字地标校准)的通讯逻辑, 通过 USART3 向 OpenART mini
 *   摄像头发起一次颜色检测请求, 摄像头返回检测到的色块中心像素坐标,
 *   单片机解析后获得颜色编号和位置信息.
 *
 * 通讯模型(与 camera_align 完全一致):
 *   STM32 → K230:  [0xAA][0x55][ID]                                      (3 字节请求)
 *   K230  → STM32: [0xAA][0x55][COLOR][MX_L][MX_H][MY_L][MY_H][XOR]     (8 字节应答)
 *     COLOR: 0 = 未检测到, 1 = 红, 2 = 绿, 3 = 蓝, 4 = 白, 5 = 黑
 *     MX/MY: uint16 色块中心像素坐标(小端)
 *     XOR  : 前 7 字节异或校验
 *
 * 数据流(对照 camera_align_at):
 *   camera_color_detect(&color, &mx, &my, timeout)
 *     ├─ HAL_UART_Transmit_DMA(请求)  → 信号量等发送完成
 *     ├─ HAL_UART_Receive_DMA(应答)   → 信号量等接收完成
 *     └─ 校验帧头 + XOR → 解析 COLOR/MX/MY
 *
 * 注意: 本模块所有阻塞操作均通过 FreeRTOS 信号量挂起任务, 不占用 CPU,
 *       不会干扰 1ms 控制环(chassisTask/gyroTask).
 */
/*
调用示例：
{
    uint8_t color;
    uint16_t mx, my;
    bool ok = camera_color_detect(&color, &mx, &my, 500);
    if (ok) {
         // color: 1=红, 2=绿, 3=蓝, 4=白, 5=黑
        // mx, my 为色块在图像中的像素坐标
    }
}
*/
#ifndef __CAMERA_COLOR_H
#define __CAMERA_COLOR_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 颜色编号 ==================== */
#define CAM_COLOR_NONE     0    /* 未检测到色块        */
#define CAM_COLOR_RED      1    /* 红色                */
#define CAM_COLOR_GREEN    2    /* 绿色                */
#define CAM_COLOR_BLUE     3    /* 蓝色                */
#define CAM_COLOR_WHITE    4    /* 白色                */
#define CAM_COLOR_BLACK    5    /* 黑色                */

/* ==================== 通讯协议宏 ==================== */
#define CAM_COLOR_REQ_LEN           3   /* 请求帧长度: [AA 55 ID]                  */
#define CAM_COLOR_ACK_LEN           8   /* 应答帧长度: [AA 55 COLOR MX(2) MY(2) XOR] */
#define CAM_COLOR_FRAME_SOF0        0xAA
#define CAM_COLOR_FRAME_SOF1        0x55

/* 默认超时(ms): 单次检测请求 → 等应答的最大时间 */
#define CAM_COLOR_DEFAULT_TIMEOUT_MS    500

/* ==================== 对外接口 ==================== */

/**
 * @brief 初始化颜色检测模块(创建信号量)
 * @note  必须在 FreeRTOS 调度器启动后调用(例如 MX_FREERTOS_Init)
 */
void camera_color_init(void);

/**
 * @brief 发起一次颜色检测请求(阻塞式, 内部异步)
 *
 * 流程:
 *   1. 通过 USART3 DMA 向摄像头发送 [AA 55 ID]
 *   2. 等待摄像头应答 [AA 55 COLOR MX_L MX_H MY_L MY_H XOR]
 *   3. 校验通过 → 解析颜色编号和像素坐标
 *
 * @param out_color   输出: 检测到的颜色 (0=无, 1=红, 2=绿, 3=蓝, 4=白, 5=黑)
 * @param out_mx      输出: 色块中心 x 像素坐标
 * @param out_my      输出: 色块中心 y 像素坐标
 * @param timeout_ms  等待应答超时(ms), 0 表示用默认值
 * @return true 检测成功(检测到色块); false 失败(超时/校验失败/未检测到)
 *
 * @note 阻塞调用. 调用方任务在等待 DMA 期间被挂起, 不占 CPU.
 */
bool camera_color_detect(uint8_t *out_color,
                         uint16_t *out_mx, uint16_t *out_my,
                         uint32_t timeout_ms);

/**
 * @brief 取最近一次检测的原始数据(调试用)
 * @param color 输出: 最近一次颜色编号
 * @param mx    输出: 最近一次 x 像素坐标
 * @param my    输出: 最近一次 y 像素坐标
 * @note 若未进行过检测, 输出全 0
 */
void camera_color_get_last_raw(uint8_t *color,
                               uint16_t *mx, uint16_t *my);

#endif /* __CAMERA_COLOR_H */