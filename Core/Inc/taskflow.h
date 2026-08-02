/**
 * @file    taskflow.h
 * @brief   任务流程模块 —— 将 StartDefaultTask 中的复杂状态机逻辑抽取为独立模块
 * @note    包含舵机测试、摄像头颜色识别测试、底盘运动状态机
 */
#ifndef __TASKFLOW_H
#define __TASKFLOW_H

/**
 * @brief  执行任务流程（阻塞，内部有 for(;;) 循环，永不返回）
 */
void taskflow_run(void);

/**
 * @brief  轻量串口日志打印（USART1 阻塞发送）
 */
void chassis_uart_log(const char *fmt, ...);

#endif /* __TASKFLOW_H */