/**
 * @file    cross_detect.h
 * @brief   十字检测纠偏模块
 *
 * 通过 USART2 (DMA+空闲中断) 与 K230 摄像头模块通讯, 在十字交叉线处进行坐标纠偏。
 * 完全非阻塞: DMA 后台搬运数据, 空闲中断触发帧结束, 信号量唤醒 defaultTask。
 *
 * 协议 (USART2, 115200-8-N-1):
 *   STM32 → K230: "DETECT\r\n"    请求检测
 *   STM32 → K230: "OK\r\n"        纠偏完成
 *   STM32 → K230: "RETRY\r\n"     要求重试
 *
 *   K230 → STM32: "CROSS,dx,dy,conf\r\n"  检测到十字 (像素偏移, 置信度)
 *   K230 → STM32: "NOCROSS\r\n"           未检测到十字
 *
 * 坐标系定义:
 *   dx > 0 : 十字在画面中心右侧
 *   dy > 0 : 十字在画面中心下方 (车头方向)
 *
 * 用法:
 *   // freertos.c 中:
 *   SemaphoreHandle_t sem = xSemaphoreCreateBinary();
 *   CrossDetect_Init();
 *   CrossDetect_SetSemaphore(sem);
 *
 *   // defaultTask 中:
 *   bool ok = CrossDetect_Calibrate(1);  // 挂起等待 DMA+IDLE, 不占 CPU
 */
#ifndef __CROSS_DETECT_H
#define __CROSS_DETECT_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* ---- 配置宏 -------------------------------------------------------------- */

/* 像素→物理毫米换算系数 (标定后修改)
 * 公式: k = 2 × H × tan(α/2) / 800
 *       H = 摄像头离地高度 (mm), α = 水平视场角 */
#define CROSS_PX_TO_MM            0.5f

/* 等待 K230 响应超时 (ms) */
#define CROSS_DETECT_TIMEOUT_MS   500

/* 像素偏差合理阈值: |dx|或|dy|超过此值视为异常, 发 RETRY 重试 */
#define CROSS_MAX_OFFSET_PX       100

/* 最大重试次数 (含首次), 超过则返回失败 */
#define CROSS_RETRY_MAX           5

/* USART2 DMA 接收缓冲大小 (字节) */
#define CROSS_UART_RX_BUF_SIZE    64

/* ---- 十字地图结构 --------------------------------------------------------- */

typedef struct {
    uint8_t  id;          /* 十字编号 (1..N) */
    float    x_mm;        /* 世界坐标 X (mm) */
    float    y_mm;        /* 世界坐标 Y (mm) */
} CrossPoint_t;

/* 十字地图数组最大容量 */
#define CROSS_MAP_MAX   16

/* ---- API 接口 ------------------------------------------------------------ */

/**
 * @brief 初始化十字检测模块 (清空地图 + 注册默认十字点)
 */
void CrossDetect_Init(void);

/**
 * @brief 注入 FreeRTOS 信号量 (在 freertos.c 中创建后调用)
 * @param sem 二进制信号量, USART2 空闲中断中 Give, Calibrate 中 Take
 */
void CrossDetect_SetSemaphore(SemaphoreHandle_t sem);

/**
 * @brief 在指定十字点处执行坐标纠偏 (完全非阻塞挂起, DMA+IDLE+信号量)
 * @param cross_id  十字编号 (对应十字地图中的 id)
 * @return true = 纠偏成功, false = 超时/重试耗尽/异常
 * @note  调用前应确保底盘已停稳; 等待期间 defaultTask 挂起, CPU 让给控制环
 */
bool CrossDetect_Calibrate(uint8_t cross_id);

/**
 * @brief 注册一个十字点到地图中
 */
bool CrossDetect_AddPoint(uint8_t id, float x_mm, float y_mm);

/**
 * @brief 清空十字地图
 */
void CrossDetect_ClearMap(void);

/**
 * @brief USART2 中断处理入口 (供 stm32f4xx_it.c 的 USART2_IRQHandler 调用)
 * @note  检测空闲中断(IDLE), 停止 DMA, 发信号量唤醒 defaultTask
 */
void CrossDetect_UART_IDLE_IRQHandler(void);

#endif /* __CROSS_DETECT_H */