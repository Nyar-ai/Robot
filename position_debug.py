# position_debug.py
# K230 立创庐山派 / Lite K230D
# 黑色十字识别 - 纯调试版(无 UART 通讯, 无按键输入, 纯显示)
#
# 用途: 手持 K230 在地图上走动, 实时观察霍夫交点法检测效果 + 标定像素/毫米系数
# 算法与 position.py 完全一致(霍夫直线交点法 + 多帧中值投票)
#
# 屏幕布局(800x480):
#   顶栏: FPS / vote:N/M / [CALIB DEBUG]
#   画面: 蓝色水平线段 + 黄色垂直线段 + 绿色交点 + 红框 + 白色中心参考十字 + 灰色刻度网格
#   底栏: STATUS(超大字) + cx,cy + DX,DY(带符号)
#
# 标定辅助:
#   画面上画刻度网格(每50px浅灰, 每100px深灰+数字标注)
#   拿真尺子量画面水平/垂直覆盖的毫米数, 程序自动算 CAM_MM_PER_PIX

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image

# ==================== 配置参数 ====================

CAM_WIDTH   = 800
CAM_HEIGHT  = 480
CAM_IMG_CX  = CAM_WIDTH  // 2   # 400
CAM_IMG_CY  = CAM_HEIGHT // 2   # 240

# 黑色二值化阈值(灰度): < BLACK_THRESH 视为黑色
BLACK_THRESH = 80

# 线段筛选参数
LINE_MIN_LEN       = 60    # 线段最小长度(像素)
LINE_MIN_MAGNITUDE = 300   # 霍夫最小响应强度

# 水平/垂直分类角度阈值(deg)
HORIZ_ANGLE_TOL = 18
VERT_ANGLE_TOL  = 18

# 交点有效性判定
CROSS_MARGIN = 30

# 多帧投票窗口
VOTE_WINDOW = 5

# 标定辅助: 已知的画面覆盖毫米数(实测后填, 填了程序自动算 CAM_MM_PER_PIX)
# 留 0 表示未标定, 程序会提示你去量
CALIB_HORIZ_MM = 0    # 画面水平方向实际覆盖的毫米数(用尺子量)
CALIB_VERT_MM  = 0    # 画面垂直方向实际覆盖的毫米数

# 刻度网格间距
GRID_MINOR_PX = 50    # 浅灰小网格
GRID_MAJOR_PX = 100   # 深灰大网格(标注数字)

# ==================== 全局变量 ====================
frame_count = 0
fps_start = 0
fps_value = 0
vote_history = []
boot_ms = 0

# 预计算标定系数
CALIB_MM_PER_PIX_X = 0.0
CALIB_MM_PER_PIX_Y = 0.0


# ==================== 摄像头/显示初始化 ====================

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


# ==================== 核心: 霍夫直线交点检测(与 position.py 一致) ====================

def line_endpoints(L):
    try:
        return L.x1(), L.y1(), L.x2(), L.y2()
    except:
        return L.x1, L.y1, L.x2, L.y2


def line_theta_deg(L):
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
    """两线段所在直线的几何交点(亚像素)."""
    x1, y1, x2, y2 = L1
    x3, y3, x4, y4 = L2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    px = x1 + t * (x2 - x1)
    py = y1 + t * (y2 - y1)
    if px != px or py != py:
        return None
    return px, py


def detect_cross(img):
    """检测黑色十字中心, 返回 (cross, h_line, v_line)."""
    gs = img.to_grayscale(copy=True)
    gs.binary([(0, BLACK_THRESH)])

    try:
        lines = gs.find_line_segments(roi=(0, 0, CAM_WIDTH, CAM_HEIGHT),
                                       merge_distance=8,
                                       max_theta_diff=8)
    except Exception as e:
        return None, None, None

    if not lines:
        return None, None, None

    horiz = []
    vert  = []
    for L in lines:
        th = line_theta_deg(L)
        ln = line_len(L)
        mg = line_mag(L)
        if ln < LINE_MIN_LEN or mg < LINE_MIN_MAGNITUDE:
            continue
        ep = line_endpoints(L)
        if th < HORIZ_ANGLE_TOL or th > (180 - HORIZ_ANGLE_TOL):
            horiz.append((ep, th, ln, mg))
        elif abs(th - 90) < VERT_ANGLE_TOL:
            vert.append((ep, th, ln, mg))

    if not horiz or not vert:
        return None, None, None

    def score(item):
        ep, th, ln, mg = item
        return ln * mg
    horiz.sort(key=score, reverse=True)
    vert.sort(key=score, reverse=True)
    h_ep = horiz[0][0]
    v_ep = vert[0][0]

    pt = line_intersection(h_ep, v_ep)
    if pt is None:
        return None, h_ep, v_ep
    cx, cy = pt

    if not (-CROSS_MARGIN <= cx <= CAM_WIDTH + CROSS_MARGIN and
            -CROSS_MARGIN <= cy <= CAM_HEIGHT + CROSS_MARGIN):
        return None, h_ep, v_ep

    return (cx, cy), h_ep, v_ep


def push_and_vote(cx, cy):
    """多帧中值投票, 返回 (cx, cy) 或 (None, None)."""
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


# ==================== 绘制: 刻度网格(标定辅助) ====================

def draw_calibration_grid(img):
    """画刻度网格: 每50px浅灰, 每100px深灰+数字标注."""
    for x in range(0, CAM_WIDTH, GRID_MINOR_PX):
        img.draw_line(x, 0, x, CAM_HEIGHT, color=(40, 40, 40), thickness=1)
    for y in range(0, CAM_HEIGHT, GRID_MINOR_PX):
        img.draw_line(0, y, CAM_WIDTH, y, color=(40, 40, 40), thickness=1)

    for x in range(0, CAM_WIDTH, GRID_MAJOR_PX):
        img.draw_line(x, 0, x, CAM_HEIGHT, color=(80, 80, 80), thickness=1)
        img.draw_string_advanced(x + 2, CAM_HEIGHT - 16, 12, str(x),
                                  color=(120, 120, 120))
    for y in range(0, CAM_HEIGHT, GRID_MAJOR_PX):
        img.draw_line(0, y, CAM_WIDTH, y, color=(80, 80, 80), thickness=1)
        img.draw_string_advanced(2, y + 2, 12, str(y),
                                  color=(120, 120, 120))


def draw_center_reference(img):
    """画白色画面中心参考十字, 判断偏移方向."""
    img.draw_line(CAM_IMG_CX, 0, CAM_IMG_CX, CAM_HEIGHT, color=(255, 255, 255), thickness=1)
    img.draw_line(0, CAM_IMG_CY, CAM_WIDTH, CAM_IMG_CY, color=(255, 255, 255), thickness=1)
    img.draw_string_advanced(CAM_IMG_CX + 4, CAM_IMG_CY + 4, 12,
                              "center(%d,%d)" % (CAM_IMG_CX, CAM_IMG_CY),
                              color=(255, 255, 255))


# ==================== 绘制: 检测结果 + HUD ====================

def draw_overlay(img, cx, cy, fps, h_line, v_line, dx_px, dy_px):
    # 1. 刻度网格(最底层)
    draw_calibration_grid(img)
    # 2. 中心参考十字
    draw_center_reference(img)

    # 3. 检测线段
    if h_line is not None:
        x1, y1, x2, y2 = h_line
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(0, 0, 255), thickness=3)
    if v_line is not None:
        x1, y1, x2, y2 = v_line
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=(255, 255, 0), thickness=3)

    # 4. 交点标记
    if cx is not None and cx >= 0:
        img.draw_cross(int(cx), int(cy), color=(0, 255, 0), size=32, thickness=3)
        img.draw_rectangle(int(cx) - 40, int(cy) - 40, 80, 80,
                           color=(255, 0, 0), thickness=2)
        img.draw_string_advanced(int(cx) + 44, int(cy) - 12, 16,
                                  "(%d,%d)" % (int(cx), int(cy)),
                                  color=(0, 255, 0))

    # 5. 顶栏
    vote_n = len(vote_history)
    img.draw_rectangle(0, 0, CAM_WIDTH, 30, color=(0, 0, 0), thickness=True)
    img.draw_string_advanced(8, 6, 16,
                              "FPS:%d  vote:%d/%d" % (fps, vote_n, VOTE_WINDOW),
                              color=(255, 255, 255))
    img.draw_string_advanced(CAM_WIDTH - 200, 6, 16, "[CALIB DEBUG]",
                              color=(0, 255, 255))

    # 6. 底栏: 超大 STATUS + DX/DY
    img.draw_rectangle(0, CAM_HEIGHT - 90, CAM_WIDTH, 90, color=(0, 0, 0), thickness=True)

    if cx is not None and cx >= 0:
        # 绿色大字
        img.draw_string_advanced(10, CAM_HEIGHT - 84, 40, "OK CROSS",
                                  color=(0, 255, 0))
        sgn_dx = "+" if dx_px >= 0 else ""
        sgn_dy = "+" if dy_px >= 0 else ""
        img.draw_string_advanced(10, CAM_HEIGHT - 40, 20,
                                  "cx,cy=(%d,%d)   DX,DY=(%s%d,%s%d) px" %
                                  (int(cx), int(cy), sgn_dx, dx_px, sgn_dy, dy_px),
                                  color=(255, 255, 255))
        if CALIB_MM_PER_PIX_X > 0:
            mm_dx = dx_px * CALIB_MM_PER_PIX_X
            mm_dy = dy_px * CALIB_MM_PER_PIX_Y
            img.draw_string_advanced(420, CAM_HEIGHT - 40, 20,
                                      "%.1f,%.1f mm" % (mm_dx, mm_dy),
                                      color=(255, 200, 0))
    else:
        # 红色大字
        img.draw_string_advanced(10, CAM_HEIGHT - 84, 40, "NO CROSS",
                                  color=(255, 0, 0))
        img.draw_string_advanced(10, CAM_HEIGHT - 40, 20,
                                  "(move K230 over a black cross)",
                                  color=(200, 200, 200))


# ==================== 标定系数计算 ====================

def compute_calib():
    """根据 CALIB_HORIZ_MM / CALIB_VERT_MM 算 CAM_MM_PER_PIX."""
    global CALIB_MM_PER_PIX_X, CALIB_MM_PER_PIX_Y
    if CALIB_HORIZ_MM > 0:
        CALIB_MM_PER_PIX_X = CALIB_HORIZ_MM / CAM_WIDTH
    if CALIB_VERT_MM > 0:
        CALIB_MM_PER_PIX_Y = CALIB_VERT_MM / CAM_HEIGHT


def print_calib_help():
    print("=" * 60)
    print("  标定辅助说明 (CAM_MM_PER_PIX 标定)")
    print("=" * 60)
    print("步骤:")
    print("  1. 把 K230 放到最终安装高度(与车上一样)")
    print("  2. 在画面里放一把尺子, 水平铺满画面")
    print("  3. 读出尺子覆盖的毫米数(例如画面水平覆盖 320mm)")
    print("  4. 把这个值填到本文件顶部 CALIB_HORIZ_MM = 320")
    print("  5. 垂直方向同理填 CALIB_VERT_MM")
    print("  6. 重启程序, 底栏会显示 DX,DY 的毫米值")
    print("  7. 把算出的 CAM_MM_PER_PIX 填到 STM32 camera_align.h")
    print("-" * 60)
    if CALIB_HORIZ_MM > 0 or CALIB_VERT_MM > 0:
        print("已标定:")
        if CALIB_MM_PER_PIX_X > 0:
            print("  CAM_MM_PER_PIX_X = %.4f  (填进 camera_align.h)" % CALIB_MM_PER_PIX_X)
        if CALIB_MM_PER_PIX_Y > 0:
            print("  CAM_MM_PER_PIX_Y = %.4f  (填进 camera_align.h)" % CALIB_MM_PER_PIX_Y)
    else:
        print("未标定: CALIB_HORIZ_MM / CALIB_VERT_MM 仍为 0")
        print("  画面上每 %dpx 一条深灰线, 用尺子量两条深灰线之间的毫米数" % GRID_MAJOR_PX)
        print("  CAM_MM_PER_PIX = 毫米数 / %d" % GRID_MAJOR_PX)
    print("=" * 60)


# ==================== 主程序 ====================

def main():
    global frame_count, fps_start, boot_ms

    print("=" * 60)
    print("  K230 黑色十字识别 - 纯调试版(无 UART)")
    print("  立创庐山派 / Lite K230D")
    print("  霍夫直线交点法 + 多帧中值投票 + 标定辅助")
    print("=" * 60)

    compute_calib()
    print_calib_help()

    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()

    sensor = init_sensor()
    init_display()
    sensor.run()

    fps_start = time.ticks_ms()
    boot_ms = fps_start

    print("[MAIN] 开始主循环(纯显示, 无 UART, 无按键)...")

    try:
        while True:
            os.exitpoint()

            # 1. 抓帧
            img = sensor.snapshot()

            # 2. 检测
            cross, h_line, v_line = detect_cross(img)

            # 3. 投票
            if cross is not None:
                cx_vote, cy_vote = push_and_vote(cross[0], cross[1])
            else:
                cx_vote, cy_vote = push_and_vote(None, None)

            # 4. FPS
            fps = update_fps()

            # 5. 算 DX/DY(带符号像素偏移)
            if cx_vote is not None:
                dx_px = int(cx_vote - CAM_IMG_CX)
                dy_px = int(cy_vote - CAM_IMG_CY)
            else:
                dx_px = 0
                dy_px = 0

            # 6. 绘制
            draw_overlay(img, cx_vote, cy_vote, fps, h_line, v_line, dx_px, dy_px)

            # 7. 显示
            Display.show_image(img)

            # 8. IDE 串口 CSV(每帧一行, 可复制到 Excel)
            now_ms = time.ticks_ms()
            t_rel = time.ticks_diff(now_ms, boot_ms)
            if cx_vote is not None:
                status_str = "OK"
                cx_str = str(int(cx_vote))
                cy_str = str(int(cy_vote))
            else:
                status_str = "NONE"
                cx_str = "0"
                cy_str = "0"
            print("DBG,%d,%s,%s,%s,%d,%d,%d,%d" %
                  (t_rel, status_str, cx_str, cy_str, dx_px, dy_px,
                   len(vote_history), fps))

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