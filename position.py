# cross_detect_uart.py
# K230 立创庐山派 / Lite K230D
# 纯二值化 + 投影法 识别黑色十字中心
# 算法: 全局二值化 → 水平/垂直投影 → 峰值定位 → 十字交叉点
#
# 通讯模式(请求-应答, 与 STM32 camera_align.c 对接):
#   STM32 → K230:  [0xAA][0x55][ID]                                        (3 字节请求)
#   K230  → STM32: [0xAA][0x55][STATUS][DX_L][DX_H][DY_L][DY_H][XOR]       (8 字节应答)
#     STATUS: 0 = 未检测到十字, 1 = 检测到
#     DX/DY : int16 带符号像素偏移 = 十字像素中心 − 图像中心(800x480→400,240), 小端
#     XOR   : 前 7 字节异或校验
#
# 与旧版本区别: 不再定时推送 ASCII, 改为"收到请求才抓帧+应答", 与陀螺仪异步模式一致.

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image
from machine import UART

# ==================== 配置参数 ====================

CAM_WIDTH   = 800
CAM_HEIGHT  = 480

# 图像中心(像素), 用于把绝对像素坐标换算成"相对中心的偏移"
CAM_IMG_CX  = CAM_WIDTH  // 2   # 400
CAM_IMG_CY  = CAM_HEIGHT // 2   # 240

# 二值化灰度阈值: 灰度 < BLACK_THRESH 判定为黑色
BLACK_THRESH = 80

# 投影采样步进 (步进越大越快, 精度越低, 推荐4)
SAMPLE_STEP = 8

# 峰值判定: 行/列黑色像素数需 >= 总行/列数的 MIN_PEAK_RATIO 才算有效
MIN_PEAK_RATIO = 0.08

# 坐标发送
UART_ID     = 2
UART_BAUD   = 115200

# 协议字节定义(必须与 STM32 camera_align.h 一致)
FRAME_SOF0      = 0xAA
FRAME_SOF1      = 0x55
REQ_LEN         = 3
ACK_LEN         = 8
ACK_STATUS_OK   = 1
ACK_STATUS_NONE = 0

# 滑动窗口平滑(用于稳定检测, 仅在请求到来时使用)
SMOOTH_WINDOW = 3

# ==================== 全局变量 ====================
cx_history = []
cy_history = []
frame_count = 0
fps_start = 0
fps_value = 0


def init_sensor():
    """初始化摄像头"""
    print("[SENSOR] 初始化摄像头 800x480 RGB565...")
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def init_display():
    """初始化LCD显示 + IDE帧缓冲"""
    print("[DISPLAY] 初始化 ST7701 LCD + IDE...")
    Display.init(Display.ST7701, width=CAM_WIDTH, height=CAM_HEIGHT, to_ide=True)


def init_uart():
    """初始化UART"""
    print("[UART] 初始化 UART%d @ %d baud..." % (UART_ID, UART_BAUD))
    try:
        uart = UART(UART_ID, UART_BAUD)
        print("[UART] 就绪")
        return uart
    except Exception as e:
        print("[UART] 初始化失败: %s, 将以模拟模式运行" % str(e))
        return None


def smooth_coord(cx, cy):
    """滑动窗口平滑坐标(仅在请求到来时调用, 返回平滑后的中心像素)"""
    global cx_history, cy_history

    if cx < 0:
        cx_history.clear()
        cy_history.clear()
        return -1, -1

    cx_history.append(cx)
    cy_history.append(cy)

    if len(cx_history) > SMOOTH_WINDOW:
        cx_history.pop(0)
        cy_history.pop(0)

    avg_cx = sum(cx_history) // len(cx_history)
    avg_cy = sum(cy_history) // len(cx_history)
    return avg_cx, avg_cy


def update_fps():
    """更新FPS计数"""
    global frame_count, fps_start, fps_value
    frame_count += 1
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, fps_start)
    if elapsed >= 1000:
        fps_value = frame_count * 1000 // elapsed
        frame_count = 0
        fps_start = now
    return fps_value


def binarize_and_project(img):
    """
    bytearray高速二值化 + 投影

    使用 img.bytearray() 一次性获取全帧灰度数据 (img 需先 to_grayscale)
    避免逐像素 get_pixel() 的 Python→C 调用开销
    """
    gs = img.to_grayscale(copy=True)
    data = gs.bytearray()
    total = len(data)     # 800 * 480 = 384000

    # 投影采样
    h_samples = CAM_HEIGHT // SAMPLE_STEP + 1
    v_samples = CAM_WIDTH // SAMPLE_STEP + 1
    h_proj = [0] * h_samples
    v_proj = [0] * v_samples

    stride = CAM_WIDTH  # 每行像素数

    for sy in range(0, CAM_HEIGHT, SAMPLE_STEP):
        row_base = sy * stride
        h_idx = sy // SAMPLE_STEP
        for sx in range(0, CAM_WIDTH, SAMPLE_STEP):
            idx = row_base + sx
            if idx < total:
                g = data[idx]
                if g < BLACK_THRESH:
                    h_proj[h_idx] += 1
                    v_proj[sx // SAMPLE_STEP] += 1

    return h_proj, v_proj


def find_peak_center(h_proj, v_proj):
    """
    从投影数组中找到十字交叉点

    水平臂: h_proj 中连续黑色像素最多的区域中心
    垂直臂: v_proj 中连续黑色像素最多的区域中心

    返回: (cx, cy) 或 (-1, -1)
    """
    h_len = len(h_proj)
    v_len = len(v_proj)

    # --- 水平投影: 找黑色像素最多的连续区域 ---
    h_max_possible = v_len

    h_max_val = max(h_proj) if h_proj else 0
    if h_max_val < h_max_possible * MIN_PEAK_RATIO:
        return -1, -1

    h_threshold = h_max_val * 0.5
    best_h_start = 0
    best_h_end = 0
    best_h_width = 0
    in_region = False
    region_start = 0

    for i in range(h_len):
        if h_proj[i] >= h_threshold:
            if not in_region:
                in_region = True
                region_start = i
        else:
            if in_region:
                in_region = False
                width = i - region_start
                if width > best_h_width:
                    best_h_width = width
                    best_h_start = region_start
                    best_h_end = i

    if in_region:
        width = h_len - region_start
        if width > best_h_width:
            best_h_width = width
            best_h_start = region_start
            best_h_end = h_len

    if best_h_width < 2:
        return -1, -1

    h_center_sample = (best_h_start + best_h_end) // 2
    cy = h_center_sample * SAMPLE_STEP + SAMPLE_STEP // 2
    if cy >= CAM_HEIGHT:
        cy = CAM_HEIGHT - 1

    # --- 垂直投影: 找黑色像素最多的连续区域 ---
    v_max_val = max(v_proj) if v_proj else 0
    if v_max_val < h_max_possible * MIN_PEAK_RATIO:
        return -1, -1

    v_threshold = v_max_val * 0.5
    best_v_start = 0
    best_v_end = 0
    best_v_width = 0
    in_region = False
    region_start = 0

    for i in range(v_len):
        if v_proj[i] >= v_threshold:
            if not in_region:
                in_region = True
                region_start = i
        else:
            if in_region:
                in_region = False
                width = i - region_start
                if width > best_v_width:
                    best_v_width = width
                    best_v_start = region_start
                    best_v_end = i

    if in_region:
        width = v_len - region_start
        if width > best_v_width:
            best_v_width = width
            best_v_start = region_start
            best_v_end = v_len

    if best_v_width < 2:
        return -1, -1

    v_center_sample = (best_v_start + best_v_end) // 2
    cx = v_center_sample * SAMPLE_STEP + SAMPLE_STEP // 2
    if cx >= CAM_WIDTH:
        cx = CAM_WIDTH - 1

    return cx, cy


def draw_overlay(img, cx, cy, fps, last_status, last_dx, last_dy):
    """在图像上绘制检测结果(极简版, 不画直方图以提升帧率)"""

    # 左上角: FPS
    img.draw_string_advanced(0, 0, 16, "FPS:%d" % fps, color=(255, 255, 255))

    # 右上角: 最近一次应答
    img.draw_string_advanced(0, 20, 14,
                              "last: stat=%d dxdy=(%d,%d)" % (last_status, last_dx, last_dy),
                              color=(200, 200, 0))

    if cx >= 0:
        # 绿色大十字标记检测到的中心
        img.draw_cross(cx, cy, color=(0, 255, 0), size=24, thickness=2)
        # 红色瞄准框
        img.draw_rectangle(cx - 30, cy - 30, 60, 60,
                           color=(255, 0, 0), thickness=2)
        # 坐标文本
        img.draw_string_advanced(cx + 32, cy - 10, 16,
                                  "(%d,%d)" % (cx, cy),
                                  color=(0, 255, 0))
    else:
        img.draw_string_advanced(10, 30, 24, "NO CROSS",
                                  color=(255, 0, 0))


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
    """从 UART 读取一个请求帧 [AA 55 ID].
    返回: ID(0~255) 若收到完整合法请求; None 若无数据或不完整."""
    if uart is None:
        return None
    # 至少要有 3 字节才可能凑成一个请求
    if uart.any() < REQ_LEN:
        return None
    req = uart.read(REQ_LEN)
    if req is None or len(req) < REQ_LEN:
        return None
    if req[0] != FRAME_SOF0 or req[1] != FRAME_SOF1:
        # 帧头不对, 丢弃这批字节(让缓冲对齐到下一次)
        return None
    return req[2]


# ==================== 主程序 ====================

def main():
    global frame_count, fps_start

    print("=" * 50)
    print("  K230 二值化 + 投影法 黑色十字识别")
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

    # 最近一次应答内容(显示用)
    last_status = ACK_STATUS_NONE
    last_dx = 0
    last_dy = 0

    print("[MAIN] 开始主循环...")
    print("[MAIN] 阈值: 灰度<%d → 黑色, 步进:%d" % (BLACK_THRESH, SAMPLE_STEP))
    print("[MAIN] 协议: 请求 AA 55 ID / 应答 AA 55 STATUS DX_L DX_H DY_L DY_H XOR")

    try:
        while True:
            os.exitpoint()

            # 1. 获取一帧(始终运行, 保证画面流畅 + FPS 统计)
            img = sensor.snapshot()

            # 2. 二值化 + 投影 + 找十字
            h_proj, v_proj = binarize_and_project(img)
            cx_raw, cy_raw = find_peak_center(h_proj, v_proj)

            # 3. 滑动窗口平滑(检测平滑, 与请求无关)
            cx, cy = smooth_coord(cx_raw, cy_raw)

            # 4. FPS
            fps = update_fps()

            # 5. 绘制叠加层(显示用)
            draw_overlay(img, cx, cy, fps, last_status, last_dx, last_dy)

            # 6. 显示
            Display.show_image(img)

            # 7. 调试打印(低频)
            if frame_count % 30 == 0:
                if cx >= 0:
                    print("[DBG] CROSS=(%d,%d)  FPS=%d" % (cx, cy, fps))
                else:
                    print("[DBG] NO CROSS  FPS=%d" % fps)

            # 8. 检查请求: 收到 AA 55 ID 才应答
            req_id = try_read_request(uart)
            if req_id is not None:
                # 收到请求, 用当前最新检测结果组装应答
                if cx >= 0:
                    status = ACK_STATUS_OK
                    dx = cx - CAM_IMG_CX
                    dy = cy - CAM_IMG_CY
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

                # 更新显示缓存
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