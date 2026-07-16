# cross_detect_uart.py
# K230 立创庐山派 / Lite K230D
# 纯二值化 + 投影法 识别黑色十字中心
# 算法: 全局二值化 → 水平/垂直投影 → 峰值定位 → 十字交叉点

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image
from machine import UART

# ==================== 配置参数 ====================

CAM_WIDTH   = 800
CAM_HEIGHT  = 480

# 二值化灰度阈值: 灰度 < BLACK_THRESH 判定为黑色
BLACK_THRESH = 80

# 投影采样步进 (步进越大越快, 精度越低, 推荐4)
SAMPLE_STEP = 8

# 峰值判定: 行/列黑色像素数需 >= 总行/列数的 MIN_PEAK_RATIO 才算有效
MIN_PEAK_RATIO = 0.08

# 坐标发送
UART_ID     = 2
UART_BAUD   = 115200
SEND_INTERVAL_MS = 80

# 滑动窗口平滑
SMOOTH_WINDOW = 3

# ==================== 全局变量 ====================
cx_history = []
cy_history = []
last_send_time = 0
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
    """滑动窗口平滑坐标"""
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
    avg_cy = sum(cy_history) // len(cy_history)
    return avg_cx, avg_cy


def send_coord(uart, cx, cy):
    """通过UART发送坐标"""
    msg = "X:%d,Y:%d\r\n" % (cx, cy)
    if uart is not None:
        try:
            uart.write(msg)
        except:
            pass
    print("[TX] " + msg.strip())


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
    # 最大可能的黑色像素数 = 图像列数(垂直臂宽度)
    h_max_possible = v_len

    # 找峰值阈值
    h_max_val = max(h_proj) if h_proj else 0
    if h_max_val < h_max_possible * MIN_PEAK_RATIO:
        return -1, -1  # 黑色像素太少, 无十字

    # 找 h_proj 中 >= h_max_val * 0.5 的所有连续区域, 取最宽的那个
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

    # 水平臂中心 (采样坐标, 需还原为像素坐标)
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


def draw_overlay(img, cx, cy, fps):
    """在图像上绘制检测结果 (极简版, 不画直方图以提升帧率)"""

    # 左上角: FPS
    img.draw_string_advanced(0, 0, 16, "FPS:%d" % fps,
                              color=(255, 255, 255))

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


# ==================== 主程序 ====================

def main():
    global last_send_time, frame_count, fps_start

    print("=" * 50)
    print("  K230 二值化 + 投影法 黑色十字识别")
    print("  立创庐山派 / Lite K230D")
    print("=" * 50)

    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()

    sensor = init_sensor()
    init_display()
    uart = init_uart()

    sensor.run()
    fps_start = time.ticks_ms()

    print("[MAIN] 开始主循环...")
    print("[MAIN] 阈值: 灰度<%d → 黑色, 步进:%d" % (BLACK_THRESH, SAMPLE_STEP))
    print("[MAIN] 高速模式: bytearray灰度+步进%d" % SAMPLE_STEP)

    try:
        while True:
            os.exitpoint()

            # 1. 获取一帧
            img = sensor.snapshot()

            # 2. 二值化 + 投影
            h_proj, v_proj = binarize_and_project(img)

            # 3. 从投影找十字交叉点
            cx_raw, cy_raw = find_peak_center(h_proj, v_proj)

            # 4. 滑动窗口平滑
            cx, cy = smooth_coord(cx_raw, cy_raw)

            # 5. FPS
            fps = update_fps()

            # 6. 绘制叠加层
            draw_overlay(img, cx, cy, fps)

            # 7. 显示
            Display.show_image(img)

            # 8. 调试打印
            if frame_count % 20 == 0:
                if cx >= 0:
                    print("[DBG] CROSS=(%d,%d)  FPS=%d" % (cx, cy, fps))
                else:
                    print("[DBG] NO CROSS  FPS=%d" % fps)

            # 9. UART发送
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_time) >= SEND_INTERVAL_MS:
                send_coord(uart, cx, cy)
                last_send_time = now_ms

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