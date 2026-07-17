# cross_detect_uart.py
# K230 立创庐山派 / Lite K230D
# 纯二值化 + 投影法 识别黑色十字中心
# 算法: 全局二值化 → 水平/垂直投影 → 峰值定位 → 十字交叉点
#
# 协议模式: 请求-响应
#   收到 "DETECT\r\n" → 连拍 N 帧 → 中值滤波 → 回复 "CROSS,dx,dy,conf\r\n" 或 "NOCROSS\r\n"

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image
from machine import UART

# ==================== 配置参数 ====================

CAM_WIDTH   = 800
CAM_HEIGHT  = 480

# 画面中心 (偏移原点)
CENTER_X = CAM_WIDTH  // 2   # 400
CENTER_Y = CAM_HEIGHT // 2   # 240

# 二值化灰度阈值: 灰度 < BLACK_THRESH 判定为黑色
BLACK_THRESH = 80

# 投影采样步进 (步进越大越快, 精度越低, 推荐4)
SAMPLE_STEP = 8

# 峰值判定: 行/列黑色像素数需 >= 总行/列数的 MIN_PEAK_RATIO 才算有效
MIN_PEAK_RATIO = 0.08

# 检测帧数 (收到 DETECT 后连续拍多少帧做统计)
DETECT_FRAMES = 10

# 串口
UART_ID     = 2
UART_BAUD   = 115200

# 串口接收超时 (ms)
UART_RX_TIMEOUT = 50

# 请求帧 / 响应帧 分隔符
FRAME_TERM = "\r\n"


# ==================== 全局变量 ====================
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
        uart = UART(UART_ID, UART_BAUD, timeout=UART_RX_TIMEOUT)
        print("[UART] 就绪")
        return uart
    except Exception as e:
        print("[UART] 初始化失败: %s, 将以模拟模式运行" % str(e))
        return None


def uart_readline(uart):
    """非阻塞读一行: 读到 \\r\\n 返回字符串(不含后缀), 否则返回 None"""
    if uart is None:
        return None
    try:
        if uart.any() == 0:
            return None
        # readline 已包含 \\r\\n
        line = uart.readline()
        if line is None:
            return None
        s = line.decode("utf-8").strip()
        return s
    except:
        return None


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


def median_filter(values):
    """中值滤波: 对列表排序后取中间值, 空列表返回 None"""
    if not values:
        return None
    s = sorted(values)
    return s[len(s) // 2]


def do_detect_and_reply(img_buf, uart):
    """
    核心检测函数:
    1. snapshot 连拍 DETECT_FRAMES 帧
    2. 每帧检测十字, 计算相对画面中心的偏移 (dx, dy)
    3. 对 dx, dy 分别做中值滤波
    4. 统计检测成功帧数 → confidence
    5. 通过 UART 回复结果
    """
    print("[DETECT] 开始连拍 %d 帧..." % DETECT_FRAMES)

    dx_list = []
    dy_list = []
    hit_count = 0

    for i in range(DETECT_FRAMES):
        # 获取一帧
        img = sensor.snapshot()

        # 二值化 + 投影
        h_proj, v_proj = binarize_and_project(img)

        # 检测十字
        cx, cy = find_peak_center(h_proj, v_proj)

        if cx >= 0 and cy >= 0:
            # 计算相对画面中心的偏移
            dx = cx - CENTER_X   # >0 十字在中心右侧
            dy = cy - CENTER_Y   # >0 十字在中心下方(车头方向)
            dx_list.append(dx)
            dy_list.append(dy)
            hit_count += 1

        # 显示 (最后一帧用于预览)
        if i == DETECT_FRAMES - 1:
            update_fps()
            if cx >= 0:
                draw_overlay(img, cx, cy, fps_value, dx, dy)
            else:
                draw_overlay(img, -1, -1, fps_value, 0, 0)
            Display.show_image(img)

    # 统计结果
    total = DETECT_FRAMES
    success_rate = hit_count * 100 // total

    if hit_count < total // 2:
        # 大部分帧未检测到十字 → 回复 NOCROSS
        msg = "NOCROSS" + FRAME_TERM
        print("[DETECT] 结果: NOCROSS (hit=%d/%d)" % (hit_count, total))
    else:
        dx_med = median_filter(dx_list)
        dy_med = median_filter(dy_list)
        msg = "CROSS,%d,%d,%d" % (dx_med, dy_med, success_rate) + FRAME_TERM
        print("[DETECT] 结果: dx=%d dy=%d conf=%d%% (hit=%d/%d)" %
              (dx_med, dy_med, success_rate, hit_count, total))

    # 发送响应
    if uart is not None:
        try:
            uart.write(msg)
        except:
            pass
    print("[TX] " + msg.strip())


def draw_preview(img, fps):
    """轻量预览: 只画十字准星和状态栏, 不做投影检测 (省 CPU)"""
    # 左上角: FPS
    img.draw_string_advanced(0, 0, 16, "FPS:%d" % fps,
                              color=(255, 255, 255))
    # 画面中心十字 (蓝色参考线)
    img.draw_cross(CENTER_X, CENTER_Y, color=(0, 0, 255), size=16, thickness=1)
    # 底部状态栏
    img.draw_string_advanced(10, CAM_HEIGHT - 22, 14,
                              "WAIT DETECT...", color=(200, 200, 200))


def draw_overlay(img, cx, cy, fps, dx, dy):
    """在图像上绘制检测结果"""

    # 左上角: FPS
    img.draw_string_advanced(0, 0, 16, "FPS:%d" % fps,
                              color=(255, 255, 255))

    # 画面中心十字 (蓝色参考线)
    img.draw_cross(CENTER_X, CENTER_Y, color=(0, 0, 255), size=16, thickness=1)

    if cx >= 0:
        # 绿色大十字标记检测到的中心
        img.draw_cross(cx, cy, color=(0, 255, 0), size=24, thickness=2)
        # 红色瞄准框
        img.draw_rectangle(cx - 30, cy - 30, 60, 60,
                           color=(255, 0, 0), thickness=2)
        # 偏移量文本
        img.draw_string_advanced(cx + 32, cy - 24, 14,
                                  "off(%d,%d)" % (dx, dy),
                                  color=(0, 255, 0))
        # 坐标文本
        img.draw_string_advanced(cx + 32, cy - 2, 14,
                                  "(%d,%d)" % (cx, cy),
                                  color=(0, 255, 0))
    else:
        img.draw_string_advanced(10, 30, 24, "NO CROSS",
                                  color=(255, 0, 0))

    # 底部状态栏
    img.draw_string_advanced(10, CAM_HEIGHT - 22, 14,
                              "WAIT DETECT..." , color=(200, 200, 200))


# ==================== 主程序 ====================

def main():
    global frame_count, fps_start

    print("=" * 50)
    print("  K230 二值化 + 投影法 黑色十字识别")
    print("  请求-响应模式: 收 DETECT → 检测 → 回复 CROSS/NOCROSS")
    print("  立创庐山派 / Lite K230D")
    print("=" * 50)

    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()

    global sensor
    sensor = init_sensor()
    init_display()
    uart = init_uart()

    sensor.run()
    fps_start = time.ticks_ms()

    print("[MAIN] 开始主循环, 等待 DETECT 请求...")
    print("[MAIN] 参数: 阈值<%d, 步进=%d, 检测帧数=%d" %
          (BLACK_THRESH, SAMPLE_STEP, DETECT_FRAMES))
    print("[MAIN] 画面中心: (%d, %d), 偏移=检测点-中心" % (CENTER_X, CENTER_Y))

    try:
        while True:
            os.exitpoint()

            # 1. 获取一帧用于预览
            img = sensor.snapshot()

            # 2. FPS
            fps = update_fps()

            # 3. 轻量预览: 只画十字准星, 不做投影检测 (省 CPU)
            draw_preview(img, fps)

            # 4. 显示
            Display.show_image(img)

            # 5. 检查 UART 是否有 "DETECT" 请求
            line = uart_readline(uart)
            if line is not None:
                print("[RX] '%s'" % line)
                if line == "DETECT":
                    # 收到请求, 执行完整检测并回复
                    do_detect_and_reply(img, uart)
                elif line == "OK":
                    print("[MAIN] STM32 确认纠偏完成")
                elif line == "RETRY":
                    print("[MAIN] STM32 要求重试")
                else:
                    print("[MAIN] 未知指令: '%s'" % line)

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