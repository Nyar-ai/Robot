/**
 * @file    taskflow.c
 * @brief   任务流程实现 —— 从 freertos.c 的 StartDefaultTask 中抽取
 * @note    包含舵机测试、摄像头颜色识别测试、底盘运动状态机
 */

/* Includes ------------------------------------------------------------------*/
#include "taskflow.h"

#include "chassis.h"
#include "clamp.h"
#include "camera_align.h"
#include "camera_color.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------*/
/* 轻量串口日志(USART1, 阻塞发送)                                            */
/* ---------------------------------------------------------------------------*/
void chassis_uart_log(const char *fmt, ...)
{
    static char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof(buf))
            n = (int)sizeof(buf);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 50);
    }
}

/* ---------------------------------------------------------------------------*/
/* 主任务流程                                                                */
/* ---------------------------------------------------------------------------*/
void taskflow_run(void)
{
    /* ---- 舵机测试 ---- */
    chassis_uart_log("[servo] Rotate 135deg (mid)...\r\n");
    clamp_rotate_set(0);
    osDelay(1000);
    clamp_gripper_open();
    osDelay(1000);

    chassis_uart_log("[servo] === Servo Test Done ===\r\n\r\n");

    /* ---- 摄像头颜色识别测试（循环） ---- */
    chassis_uart_log("[color] loop test start - send response frame via USART3...\r\n");
    for (;;) {
        uint8_t color;
        uint16_t mx, my;
        bool ok = camera_color_detect(&color, &mx, &my, 500);
        if (ok) {
            const char *color_name = "???";
            switch (color) {
                case CAM_COLOR_RED:   color_name = "RED";   break;
                case CAM_COLOR_GREEN: color_name = "GREEN"; break;
                case CAM_COLOR_BLUE:  color_name = "BLUE";  break;
                case CAM_COLOR_WHITE: color_name = "WHITE"; break;
                case CAM_COLOR_BLACK: color_name = "BLACK"; break;
            }
            chassis_uart_log("[color] detect=%s center=(%d,%d)\r\n", color_name, mx, my);
        } else {
            chassis_uart_log("[color] no detect (timeout or NONE)\r\n");
        }
        osDelay(500);
    }

    /* ---- 正常任务（状态机，当前因上方 for(;;) 不可达，保留供后续启用） ---- */
    chassis_uart_log("\r\n[task] auto-cycle start: forward 500mm -> return -> calibrate -> loop\r\n");
    chassis_set_pose(-780, 0, 0);
    chassis_uart_log("\r\n[task] set pose to (0, 950, 0)\r\n");

    enum {
        STATE_MOVE1,
        STATE_CALIBRATE1,
        STATE_MOVE2,  /* 第一次去中心点 */
        STATE_TURN1,
        STATE_MOVE3,
        STATE_MOVE4,
        STATE_CALIBRATE2,
        STATE_MOVE5,  /* 去A近点，回到原点 */
        STATE_TURN2,
        STATE_MOVE6,
        STATE_CALIBRATE3,
        STATE_MOVE7,
        STATE_MOVE8,
        STATE_TURN3,
        GO_UP,
        GO_DOWN,
        DS_DONE
    } state = GO_UP;

    for (;;) {
        switch (state) {
        case GO_UP: {
            if (clamp_set_height(80.0f)) {
                chassis_uart_log("[clamp] up done\r\n");
                state = GO_DOWN;
            }
            break;
        }
        case GO_DOWN: {
            if (clamp_set_height(0.0f)) {
                chassis_uart_log("[clamp] down done\r\n");
                state = DS_DONE;
            }
            break;
        }
        case STATE_MOVE1: {
            /* 前进至 (125, 0) */
            bool arrived = move_to_coordinate(125.0f, 0.0f);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_CALIBRATE3;
            }
            break;
        }
        case STATE_CALIBRATE3: {
            bool ok = camera_align_at(0.0f, 0.0f, 0.0f, 1000);
            int16_t ldx, ldy;
            uint8_t lstat;
            camera_align_get_last_raw(&ldx, &ldy, &lstat);
            float nx, ny, nth;
            chassis_get_pose(&nx, &ny, &nth);
            chassis_uart_log("[task] calibrate  ok=%d stat=%d dxdy=(%d,%d)px pose(%.1f,%.1f,%.1f)\r\n",
                             ok, lstat, ldx, ldy, nx, ny, nth);
            state = STATE_MOVE8;
            break;
        }
        case STATE_MOVE8: {
            /* 前进至 (0, 0) */
            bool arrived = move_to_coordinate(0.0f, 0.0f);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_TURN1;
            }
            break;
        }
        case STATE_TURN1: {
            bool done = headturn(90);
            if (done) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] turn1 done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE2;
            }
            break;
        }
        case STATE_MOVE2: {
            bool arrive = move_to_coordinate(0, 390);
            if (arrive) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE3;
            }
            break;
        }
        case STATE_MOVE3: {
            bool arrived = move_to_coordinate(0.0f, 0.0f);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_TURN2;
            }
            break;
        }
        case STATE_TURN2: {
            bool done = headturn(0);
            if (done) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] turn1 done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE4;
            }
            break;
        }
        case STATE_MOVE4: {
            bool arrived = move_to_coordinate(390, 0);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE5;
            }
            break;
        }
        case STATE_MOVE5: {
            bool arrived = move_to_coordinate(0, 0);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_TURN3;
            }
            break;
        }
        case STATE_TURN3: {
            bool done = headturn(-90);
            if (done) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] turn1 done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE6;
            }
            break;
        }
        case STATE_MOVE6: {
            bool arrived = move_to_coordinate(0, -390);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = STATE_MOVE7;
            }
            break;
        }
        case STATE_MOVE7: {
            bool arrived = move_to_coordinate(0, 0);
            if (arrived) {
                float x, y, th;
                chassis_get_pose(&x, &y, &th);
                chassis_uart_log("[task] forward done, pose(%.1f, %.1f, %.1f) -> returning\r\n", x, y, th);
                state = DS_DONE;
            }
            break;
        }
        case DS_DONE:
        default:
            break;
        }
        /* 10ms 轮询节拍 */
        osDelay(10);
    }
}