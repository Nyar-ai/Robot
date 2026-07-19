# cross_detect_uart.py
# K230 立创庐山派 / Lite K230D
# 霍夫直线交点法 识别黑色十字中心
# 算法: 二值化提取黑色 → find_line_segments 霍夫找线段 → 水平/垂直分类 → 两直线交点(亚像素)
#
# 相比旧版(二值化+投影峰值法)改进点:
#   - 交点精度: 投影法取峰值中心(±5px) → 直线方程联立(亚像素,±1-2px)
#   - 抗噪: 霍夫对断点/局部遮挡/光照不均不敏感
#   - 形状校验: 只接受"近似水平+近似垂直"成对线段, 抗非十字干扰
#   - 多帧中值投票: 取最近 N 帧交点中值, 抑制单帧抖动
#
# 通讯模式(请求-应答, 与 STM32 camera_align.c 对接, 协议不变):
#   STM32 → K230:  [0xAA][0x55][ID]                                        (3 字节请求)
#   K230  → STM32: [0xAA][0x55][STATUS][DX_L][DX_H][DY_L][DY_H][XOR]       (8 字节应答)
#     STATUS: 0 = 未检测到十字, 1 =检测到
#     DX/DY : int16 带符号像素偏移 = 十字像素中心 − 图像中心(800x480→400,240), 小端
#     XOR   : 前 7 字节异或校验

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image
from machine import UART

# ==================== 配置参数 ====================

CAM_WIDTH   = 800
CAM_HEIGHT  = 480
CAM_IMG_CX  = CAM_WIDTH  // 2   # 400
CAM_IMG_CY  = CAM_HEIGHT // 2   # 240

# UART
UART_ID     = 2
UART_BAUD   = 115200

# 黑色二值化阈值(灰度): < BLACK_THRESH 视为黑色
BLACK_THRESH = 80

# 线段筛选参数
LINE_MIN_LEN       = 60    # 线段最小长度(像素), 过滤短噪线
LINE_MIN_MAGNITUDE = 300   # 霍夫最小响应强度, 过滤弱响应

# 水平/垂直分类角度阈值(deg)
# K230 line.theta() 返回 0~179, 其中 0/179=水平, 90=垂直
HORIZ_ANGLE_TOL = 18   # |theta|<18 或 |theta-180|<18 视为水平
VERT_ANGLE_TOL  = 18   # |theta-90|<18 视为垂直

# 交点有效性判定: 交点必须在图像范围内(留点余量)
CROSS_MARGIN = 30

# 多帧投票窗口(取最近 VOTE_WINDOW 个有效交点的中值, 抑制单帧抖动)
VOTE_WINDOW = 5

# 协议字节定义(必须与 STM32 camera_align.h 一致)
FRAME_SOF0      = 0xAA
FRAME_SOF1      = 0x55
REQ_LEN         = 3
ACK_LEN         = 8
ACK_STATUS_OK   = 1
ACK_STATUS_NONE = 0

# ==================== 全局变量 ====================
frame_count = 0
fps_start = 0
fps_value = 0

# 多帧交点历史(存 (cx, cy) 元组, None 表示无效)
vote_history = []

# 最近一次用于显示的检测中间结果
last_h_line = None   # (x1,y1,x2,y2)
last_v_line = None
last_cross  = None   # (cx, cy)


# ==================== 摄像头/UART 初始化 ====================

def init_sensor():
    print("[SENSOR] 初始化摄像头 800x480 RGB565...")
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def init_display():
    print("[DISPLAY] 初始化 ST7701 LCD + IDE...")
    Display.init(Display.ST7701, width=CAM_WIDTH, height=CAM_HEIGHT, to_ide=True)


def init_uart():
    print("[UART] 初始化 UART%d @ %d baud..." % (UART_ID, UART_BAUD))
    try:
        uart = UART(UART_ID, UART_BAUD)
        print("[UART] 就绪")
        return uart
    except Exception as e:
        print("[UART] 初始化失败: %s, 将以模拟模式运行" % str(e))
        return None


def update_fps():
    global frame_count, fps_start, fps_value
    frame_count += 1
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, fps_start)
    if elapsed >= 1000:
        fps_value = frame_count * 1000 // elapsed
        frame_count = 0
        fps_start = now
    return fps_value


# ==================== 核心: 霍夫直线交点检测 ====================

def line_endpoints(L):
    """安全获取线段两端点, 兼容不同 API 版本."""
    try:
        return L.x1(), L.y1(), L.x2(), L.y2()
    except:
        # 部分版本用属性
        return L.x1, L.y1, L.x2, L.y2


def line_theta_deg(L):
    """获取线段角度(0~179)."""
    try:
        return L.theta()
    except:
        return L.theta


def line_len(L):
    try:
        return L.length()
    except:
        return L.length


def line_mag(L):
    try:
        return L.magnitude()
    except:
        return getattr(L, "magnitude", 0)


def line_intersection(L1, L2):
    """两线段所在直线的几何交点(亚像素).
    L1=(x1,y1,x2,y2), L2=(x3,y3,x4,y4).
    返回 (px, py) 或 None(平行/数值异常)."""
    x1, y1, x2, y2 = L1
    x3, y3, x4, y4 = L2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return None  # 平行
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    px = x1 + t * (x2 - x1)
    py = y1 + t * (y2 - y1)
    if px != px or py != py:  # NaN 检查
        return None
    return px, py


def detect_cross(img):
    """在当前帧上检测黑色十字中心.
    返回:
      cross (cx, cy) 或 None
      h_line, v_line 用于绘制的线段端点
    """
    # 1. 转灰度 + 提取黑色像素(黑变白, 其他变黑), 便于霍夫在十字上检测
    gs = img.to_grayscale(copy=True)
    # binary: 黑色(灰度<BLACK_THRESH)置为255(白), 其他置0
    gs.binary([(0, BLACK_THRESH)])

    # 2. 霍夫找线段
    try:
        lines = gs.find_line_segments(roi=(0, 0, CAM_WIDTH, CAM_HEIGHT),
                                       merge_distance=8,
                                       max_theta_diff=8)
    except Exception as e:
        print("[Hough] failed: %s" % str(e))
        return None, None, None

    if not lines:
        return None, None, None

    # 3. 分类: 水平 / 垂直
    horiz = []
    vert  = []
    for L in lines:
        th = line_theta_deg(L)
        ln = line_len(L)
        mg = line_mag(L)
        if ln < LINE_MIN_LEN or mg < LINE_MIN_MAGNITUDE:
            continue
        ep = line_endpoints(L)

        # 水平判定
        if th < HORIZ_ANGLE_TOL or th > (180 - HORIZ_ANGLE_TOL):
            horiz.append((ep, th, ln, mg))
        # 垂直判定
        elif abs(th - 90) < VERT_ANGLE_TOL:
            vert.append((ep, th, ln, mg))

    if not horiz or not vert:
        return None, None, None

    # 4. 各取霍夫响应最强的一条(综合 length*magnitude 打分)
    def score(item):
        ep, th, ln, mg = item
        return ln * mg
    horiz.sort(key=score, reverse=True)
    vert.sort(key=score, reverse=True)
    h_ep = horiz[0][0]
    v_ep = vert[0][0]

    # 5. 算交点
    pt = line_intersection(h_ep, v_ep)
    if pt is None:
        return None, h_ep, v_ep
    cx, cy = pt

    # 6. 交点必须在图像范围内
    if not (-CROSS_MARGIN <= cx <= CAM_WIDTH + CROSS_MARGIN and
            -CROSS_MARGIN <= cy <= CAM_HEIGHT + CROSS_MARGIN):
        return None, h_ep, v_ep

    return (cx, cy), h_ep, v_ep


# ==================== 多帧中值投票 ====================

def push_and_vote(cx, cy):
    """把当前帧交点推入历史窗口, 返回窗口内有效交点的中值(cx, cy).
    传入 None 表示本帧无效, 清空窗口(避免半截数据被中值)."""
    global vote_history
    if cx is None:
        vote_history = []
        return None, None

    vote_history.append((cx, cy))
    if len(vote_history) > VOTE_WINDOW:
        vote_history.pop(0)

    if len(vote_history) == 0:
        return None, None

    xs = sorted(p[0] for p in vote_history)
    ys = sorted(p[1] for p in vote_history)
    mid = len(xs) // 2
    return xs[mid], ys[mid]


# ==================== 协议层(不变) ====================

def build_ack(status, dx, dy):
    """组装 8 字节应答帧: AA 55 STATUS DX_L DX_H DY_L DY_H XOR"""
    dx_u16 = dx & 0xFFFF
    dy_u16 = dy & 0xFFFF
    frame = bytes([FRAME_SOF0, FRAME_SOF1, status,
                   dx_u16 & 0xFF, (dx_u16 >> 8) & 0xFF,
                   dy_u16 & 0xFF, (dy_u16 >> 8) & 0xFF])
    xor = 0
    for b in frame:
        xor ^= b
    return frame + bytes([xor])


def try_read_request(uart):
    """读取请求帧 [AA 55 ID], 返回 ID 或 None."""
    if uart is None:
        return None
    if uart.any() < REQ_LEN:
        return None
    req = uart.read(REQ_LEN)
    if req is None or len(req) < REQ_LEN:
        return None
    if req[0] != FRAME_SOF0 or req[1] != FRAME_SOF1:
        return None
    return req[2]


# ==================== 绘制叠加层 ====================

def draw_overlay(img, cx, cy, fps, h_line, v_line, last_status, last_dx, last_dy):
    # 左上: FPS
    img.draw_string_advanced(0, 0, 16, "FPS:%d" % fps, color=(255, 255, 255))
    # 第二行: 最近一次应答
    img.draw_string_advanced(0, 20, 14,
                              "last: stat=%d dxdy=(%d,%d)" % (last_status, last_dx, last_dy),
                              color=(200, 200, 0))

    # 画检测到的线段(蓝=水平, 黄=垂直)
    if h_line is not None:
        x1, y1, x2, y2 = h_line
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(0, 0, 255), thickness=2)
    if v_line is not None:
        x1, y1, x2, y2 = v_line
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(255, 255, 0), thickness=2)

    if cx is not None and cx >= 0:
        # 绿色大十字标记交点
        img.draw_cross(int(cx), int(cy), color=(0, 255, 0), size=24, thickness=2)
        # 红色瞄准框
        img.draw_rectangle(int(cx) - 30, int(cy) - 30, 60, 60,
                           color=(255, 0, 0), thickness=2)
        # 坐标文本
        img.draw_string_advanced(int(cx) + 32, int(cy) - 10, 16,
                                  "(%d,%d)" % (int(cx), int(cy)),
                                  color=(0, 255, 0))
    else:
        img.draw_string_advanced(10, 30, 24, "NO CROSS",
                                  color=(255, 0, 0))


# ==================== 主程序 ====================

def main():
    global frame_count, fps_start
    global last_h_line, last_v_line, last_cross

    print("=" * 50)
    print("  K230 霍夫直线交点法 黑色十字识别")
    print("  立创庐山派 / Lite K230D")
    print("  请求-应答模式 (与 STM32 camera_align.c 对接)")
    print("=" * 50)

    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()

    sensor = init_sensor()
    init_display()
    uart = init_uart()

    sensor.run()
    fps_start = time.ticks_ms()

    last_status = ACK_STATUS_NONE
    last_dx = 0
    last_dy = 0

    print("[MAIN] 开始主循环...")
    print("[MAIN] 黑色阈值: 灰度<%d, 线段最小长度:%d" % (BLACK_THRESH, LINE_MIN_LEN))
    print("[MAIN] 水平容差:%d°, 垂直容差:%d°, 投票窗口:%d" %
          (HORIZ_ANGLE_TOL, VERT_ANGLE_TOL, VOTE_WINDOW))
    print("[MAIN] 协议: 请求 AA 55 ID / 应答 AA 55 STATUS DX_L DX_H DY_L DY_H XOR")

    try:
        while True:
            os.exitpoint()

            # 1. 抓帧
            img = sensor.snapshot()

            # 2. 霍夫直线交点检测(在灰度二值图上)
            cross, h_line, v_line = detect_cross(img)

            # 缓存用于绘制
            last_h_line = h_line
            last_v_line = v_line
            last_cross  = cross

            # 3. 多帧中值投票
            if cross is not None:
                cx_vote, cy_vote = push_and_vote(cross[0], cross[1])
            else:
                cx_vote, cy_vote = push_and_vote(None, None)

            # 4. FPS
            fps = update_fps()

            # 5. 绘制
            draw_overlay(img, cx_vote, cy_vote, fps, h_line, v_line,
                         last_status, last_dx, last_dy)

            # 6. 显示
            Display.show_image(img)

            # 7. 低频调试打印
            if frame_count % 30 == 0:
                if cx_vote is not None:
                    print("[DBG] CROSS=(%d,%d) vote_n=%d FPS=%d" %
                          (int(cx_vote), int(cy_vote), len(vote_history), fps))
                else:
                    print("[DBG] NO CROSS  FPS=%d" % fps)

            # 8. 检查请求 → 应答
            req_id = try_read_request(uart)
            if req_id is not None:
                if cx_vote is not None:
                    status = ACK_STATUS_OK
                    # 注意: 偏移用投票后的中值, 转成 int16
                    dx = int(cx_vote - CAM_IMG_CX)
                    dy = int(cy_vote - CAM_IMG_CY)
                else:
                    status = ACK_STATUS_NONE
                    dx = 0
                    dy = 0

                ack = build_ack(status, dx, dy)
                if uart is not None:
                    try:
                        uart.write(ack)
                    except:
                        pass

                last_status = status
                last_dx = dx
                last_dy = dy
                print("[TX] id=%d stat=%d dxdy=(%d,%d)" % (req_id, status, dx, dy))

    except KeyboardInterrupt:
        print("\n[MAIN] 用户中断")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        print("[CLEANUP] 释放资源...")
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("[CLEANUP] 完成")


if __name__ == "__main__":
    main()