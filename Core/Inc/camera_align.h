/**
 * @file    camera_align.h
 * @brief   摄像头黑色十字地标校准模块(USART2 + DMA, 异步请求-应答模式)
 *
 * 设计目标:
 *   机械振动导致麦轮车里程计/陀螺仪积分存在累积漂移. 在地图上预先布置
 *   黑色十字地标, 车行进到 waypoint 停稳后, 通过 USART2 向 K230 摄像头
 *   发起一次校准请求, 摄像头返回"十字像素偏移", 单片机换算为车体系偏移
 *   后, 结合已知的十字世界坐标反推车的真实世界坐标, 调用 chassis_set_pose
 *   校正 x/y(θ 仍信陀螺仪), 消除累积漂移.
 *
 * 通讯模型(仿陀螺仪异步模式):
 *   STM32 → K230:  [0xAA][0x55][ID]                       (3 字节请求)
 *   K230  → STM32: [0xAA][0x55][STATUS][DX_L][DX_H][DY_L][DY_H][XOR]  (8 字节应答)
 *     STATUS: 0 = 未检测到十字, 1 = 检测到
 *     DX/DY : int16 带符号像素偏移 = 十字像素中心 − 图像中心(800x480→400,240)
 *     XOR   : 前 7 字节异或校验
 *
 * 数据流(对照 gyroTask):
 *   camera_align_at(id, xw, yw, timeout)
 *     ├─ HAL_UART_Transmit_DMA(请求)  → 信号量等发送完成
 *     ├─ HAL_UART_Receive_DMA(应答)   → 信号量等接收完成
 *     └─ 像素偏移 → 车体系 mm → R(θ) 旋转 → 反推车位姿 → chassis_set_pose
 *
 * 注意: 本模块所有阻塞操作均通过 FreeRTOS 信号量挂起任务, 不占用 CPU,
 *       不会干扰 1ms 控制环(chassisTask/gyroTask).
 */
/*
调用示例：
case STATE_CALIBRATE3:
      {
        
        bool ok = camera_align_at(0.0f,0.0f,0.0f, 1000);
        int16_t ldx, ldy; uint8_t lstat;
        camera_align_get_last_raw(&ldx, &ldy, &lstat);
        float nx, ny, nth;
        chassis_get_pose(&nx, &ny, &nth);
        chassis_uart_log("[task] calibrate  ok=%d stat=%d dxdy=(%d,%d)px pose(%.1f,%.1f,%.1f)\r\n", ok, lstat, ldx, ldy, nx, ny, nth);
        
        state = STATE_MOVE5;
        break;
      }
*/
#ifndef __CAMERA_ALIGN_H
#define __CAMERA_ALIGN_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 摄像头/几何标定参数(占位, 实测后填) ==================== */

/* K230 摄像头图像分辨率(必须与 position.py 一致) */
#define CAM_IMG_WIDTH       800
#define CAM_IMG_HEIGHT      480
#define CAM_IMG_CX          (CAM_IMG_WIDTH  / 2)   /* 400 */
#define CAM_IMG_CY          (CAM_IMG_HEIGHT / 2)   /* 240 */

/* 像素 → 毫米换算系数(实测: 在摄像头安装高度下, 每毫米对应多少像素)
 * 标定方法: 在摄像头视野地面放一把尺子, 量出画面水平/垂直方向实际覆盖的毫米数,
 *           PIX_PER_MM = 像素数 / 毫米数. 这里给倒数 MM_PER_PIX 便于直接乘. */
#define CAM_MM_PER_PIX_X    0.1f    /* 水平: 每像素对应 0.1mm(占位, 实测填) */
#define CAM_MM_PER_PIX_Y    0.1f    /* 垂直: 每像素对应 0.1mm(占位, 实测填) */

/* 像素方向 → 车体系方向符号
 * 假设: 摄像头光轴朝下, 镜头"图像上"对应车体"前方"(x+), "图像右"对应车体"左"(y+).
 *       (即人站在车后向前看, 图像上=前, 图像右=左)
 * 若实际安装相反, 把对应符号取 -1 即可. */
#define CAM_PIX_X_SIGN     (+1.0f)  /* 图像 y 方向 → 车体 x(前)方向: 图像下↔车体后 = -1 */
#define CAM_PIX_Y_SIGN     (+1.0f)  /* 图像 x 方向 → 车体 y(左)方向: 图像右↔车体左 = +1 */

/* 摄像头视野中心在车体系下的安装偏移(mm)
 * 车体系: x 向前, y 向左, 原点为轮组几何中心(或编码器/里程计原点).
 * 摄像头镜头中心相对该原点的偏移: 前方为正 x, 左方为正 y. */
#define CAM_OFFSET_X_MM     -120.0f
#define CAM_OFFSET_Y_MM     0.0f

/* ==================== 通讯协议宏 ==================== */
#define CAM_REQ_LEN         3                       /* 请求帧长度 */
#define CAM_ACK_LEN         8                       /* 应答帧长度 */
#define CAM_FRAME_SOF0      0xAA
#define CAM_FRAME_SOF1      0x55
#define CAM_ACK_STATUS_OK   1                       /* 检测到十字 */

/* 默认超时(ms): 单次校准请求 → 等应答的最大时间 */
#define CAM_DEFAULT_TIMEOUT_MS   500

/* ==================== 对外接口 ==================== */

/**
 * @brief 初始化摄像头校准模块(创建信号量)
 * @note  必须在 FreeRTOS 调度器启动后调用(例如 MX_FREERTOS_Init)
 */
void camera_align_init(void);

/**
 * @brief 在当前位置发起一次摄像头地标校准(阻塞式, 内部异步)
 *
 * 流程:
 *   1. 通过 USART2 DMA 向摄像头发送 [AA 55 ID]
 *   2. 等待摄像头应答 [AA 55 STATUS DX_L DX_H DY_L DY_H XOR]
 *   3. STATUS=1 且校验通过 → 换算像素偏移 → 反推车位姿 → chassis_set_pose
 *
 * @param id           当前对准的十字地标编号(0~N, 摄像头不区分时可固定 0)
 * @param cross_wx     该十字在世界系下的已知 x 坐标(mm)
 * @param cross_wy     该十字在世界系下的已知 y 坐标(mm)
 * @param timeout_ms   等待应答超时(ms), 0 表示用默认 CAM_DEFAULT_TIMEOUT_MS
 * @return true 校准成功(检测到十字且已校正位姿); false 失败(超时/校验失败/未检测到)
 *
 * @note 阻塞调用. 调用方任务在等待 DMA 期间被挂起, 不占 CPU.
 *       通常由低优先级 defaultTask 在 waypoint 到达、车停稳后调用.
 */
bool camera_align_at(uint8_t id, float cross_wx, float cross_wy, uint32_t timeout_ms);

/**
 * @brief 取最近一次校准的原始像素偏移(调试用)
 * @param dx_px  输出: x 方向像素偏移(十字像素 − 图像中心x)
 * @param dy_px  输出: y 方向像素偏移
 * @param status 输出: 最近一次应答状态(0未检测,1检测到)
 * @note 若未进行过校准, 输出全 0
 */
void camera_align_get_last_raw(int16_t *dx_px, int16_t *dy_px, uint8_t *status);

#endif /* __CAMERA_ALIGN_H */