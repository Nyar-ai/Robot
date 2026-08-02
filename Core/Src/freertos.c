/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis.h"
#include "clamp.h"
#include "stepper.h"
#include "mpu6050.h"
#include "usart.h"
#include "camera_align.h"
#include "camera_color.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "semphr.h"
#include "taskflow.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
    .name = "chassisTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

/* 陀螺仪任务(DMA 异步读取 + yaw 积分 + 注入 chassis).
 * 优先级高于 chassisTask, 但因为用 DMA+信号量异步, 等待期间真正挂起,
 * 不会抢占 chassisTask 的 1ms 控制环. */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
    .name = "gyroTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

/* 夹具控制任务(带轮步进电机梯形加减速 1ms 控制环) */
osThreadId_t clampTaskHandle;
const osThreadAttr_t clampTask_attributes = {
    .name = "clampTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartChassisTask(void *argument);
void StartGyroTask(void *argument);
void StartClampTask(void *argument);

/* I2C DMA 完成信号量(gyroTask 与 DMA 中断同步用) */
static SemaphoreHandle_t g_i2c_dma_sem = NULL;

/* USART2 十字检测信号量 (IDLE 中断唤醒 defaultTask) */
static SemaphoreHandle_t g_uart2_rx_sem = NULL;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  chassis_init();
  clamp_init();        /* 夹具控制层(带轮步进电机梯形曲线规划器) */
  Stepper_Init();      /* 配置 5 个步进定时器(Prescaler/ARR/占空比), 不立即转 */
  camera_align_init(); /* 摄像头地标校准模块(USART2 + DMA + 信号量) */
  camera_color_init(); /* 摄像头颜色检测模块(USART3 + DMA + 信号量) */
  /* MPU6050 初始化放在 gyroTask 里(因为它需要 HAL_Delay, 不能在内核启动前调) */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);
  gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);
  clampTaskHandle = osThreadNew(StartClampTask, NULL, &clampTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  taskflow_run();
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 陀螺仪任务: DMA 异步读 MPU6050 + yaw 积分 + 注入 chassis.
 * 关键: 用 DMA 异步, 读期间任务挂起(osDelayUntil), 不会死占 CPU.
 *       即便优先级高于 chassisTask, 也只在"处理数据"的极短时间内占用,
 *       chassisTask 的 1ms 控制环不受影响. */
void StartGyroTask(void *argument)
{
  (void)argument;

  /* 初始化 MPU6050(此时调度器已启动, 可安全用 HAL_Delay) */
  uint8_t ret = MPU6050_Init();
  if (ret != 0)
  {
    chassis_uart_log("[gyro] MPU6050 init failed, code=%d (run on odom only)\r\n", ret);
    /* 初始化失败不进循环, 但 chassis 会自动退化用里程计 theta */
    for (;;)
    {
      osDelay(1000);
    }
  }
  chassis_uart_log("[gyro] MPU6050 init ok, calibrating yaw bias (keep still)...\r\n");

  /* 上电静止校准零漂: 预热丢弃 + 3sigma 鲁棒均值 + 记录温度基准.
   * 采样数受 MPU6050_CALIB_BUF(256) 限制; 校准期间车必须静止!
   * 预热约 0.4s + 采集约 0.5s, 共约 0.9s. */
  MPU6050_CalibrateYaw(256);
  chassis_uart_log("[gyro] calib done, T=%.1fC; runtime bias-tracking & auto-recalib ON\r\n",
                   MPU6050_GetTempC());

  /* 创建 DMA 完成信号量 */
  g_i2c_dma_sem = xSemaphoreCreateBinary();
  if (g_i2c_dma_sem == NULL)
  {
    chassis_uart_log("[gyro] sem create failed\r\n");
    for (;;)
      osDelay(1000);
  }

  TickType_t last = xTaskGetTickCount();
  const float dt = 0.001f; /* 1ms 节拍, 与 chassis_tick 一致 */

  for (;;)
  {
    /* 1) 启动 DMA 异步读(非阻塞, 立即返回) */
    MPU6050_StartReadDMA();

    /* 2) 信号量等待 DMA 完成(任务真正挂起, CPU 让给 chassisTask).
     *    DMA 在 I2C1_EV 中断完成后, HAL_I2C_MemRxCpltCallback 释放信号量唤醒本任务.
     *    超时 5ms 保护(I2C 异常时不至于永久卡死). */
    if (xSemaphoreTake(g_i2c_dma_sem, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      /* 3) DMA 完成后, 数据已解析, 积分 yaw 并注入 chassis */
      MPU6050_IntegrateYaw(dt);
      chassis_feed_gyro(MPU6050_GetYaw());
    }

    /* 4) 1ms 节拍(vTaskDelayUntil 保证严格 1ms 周期, DMA 通常 0.5ms 内完成) */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

/* HAL I2C 内存读取完成回调(DMA 把 14 字节搬完后, HAL 在中断里调本函数).
 * 这里解析数据 + 释放信号量唤醒 gyroTask. 注意: 运行在中断上下文, 要简短. */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    MPU6050_OnDMAComplete(); /* 解析数据 + 置完成标志 */
    BaseType_t hpw = pdFALSE;
    if (g_i2c_dma_sem != NULL)
    {
      xSemaphoreGiveFromISR(g_i2c_dma_sem, &hpw);
      portYIELD_FROM_ISR(hpw);
    }
  }
}

/* 1ms 控制层: 梯形曲线 + 麦轮逆解 + 里程计.
 * 注意: 本任务严格 1ms 节拍, 绝不做阻塞操作(如串口打印), 否则破坏控制环精度.
 *       打印/指令编排交给低优先级的 defaultTask. */
void StartChassisTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();

  for (;;)
  {
    chassis_tick(); /* 推进底盘状态机(内部 dt=1ms) */

    /* 把 chassis 算出的 4 轮目标线速度(mm/s) 下发给步进电机层 */
    float w[4];
    chassis_get_wheel_speed(w);
    Stepper_SetWheelSpeedAll(w);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(1)); /* 严格 1ms */
  }
}

/* 夹具 1ms 控制层: 梯形曲线 + 高度步进电机 PWM.
 * 对标 chassisTask, 严格 1ms 节拍, 绝不做阻塞操作. */
void StartClampTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();

  for (;;)
  {
    clamp_tick(); /* 推进夹具状态机(内部 dt=1ms) */

    vTaskDelayUntil(&last, pdMS_TO_TICKS(1)); /* 严格 1ms */
  }
}


/* USER CODE END Application */

