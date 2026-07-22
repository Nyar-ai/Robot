# cross_detect_uart.py
# K230 projection cross detect (Otsu + vote)
import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image
from machine import UART, FPIOA

CAM_WIDTH = 800
CAM_HEIGHT = 480
CAM_IMG_CX = CAM_WIDTH // 2
CAM_IMG_CY = CAM_HEIGHT // 2
USE_OTSU = False
BLACK_THRESH = 75
SAMPLE_STEP = 8
MIN_PEAK_RATIO = 0.08
VOTE_WINDOW = 5
FRAME_SOF0 = 0xAA
FRAME_SOF1 = 0x55
REQ_LEN = 3
ACK_LEN = 8
ACK_STATUS_OK = 1
ACK_STATUS_NONE = 0

# ROI (Region of Interest) - 排除车体自身区域
# 摄像头装在车尾, 车体出现在画面下部, 裁剪掉底部若干行
ROI_ENABLE = True           # 是否启用 ROI
ROI_Y_START = 0             # ROI 起始 Y(含), 保持 0(顶部不变)
ROI_Y_END   = 360           # ROI 结束 Y(不含), 即排除 Y>=360 的车体区域(可实测调)

frame_count = 0
fps_start = 0
fps_value = 0
vote_history = []


def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def init_display():
    Display.init(Display.ST7701, width=CAM_WIDTH, height=CAM_HEIGHT, to_ide=True)


def init_uart():
    # 配置 UART2 引脚: 丝印T=GPIO11(TX), 丝印R=GPIO12(RX)
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, baudrate=115200,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    print("[UART] UART2 init ok: GPIO11(TX), GPIO12(RX), 115200 8N1")
    return uart


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


def compute_otsu_threshold(gs_img):
    try:
        bins = [0] * 256
        step = 4
        for y in range(0, CAM_HEIGHT, step):
            for x in range(0, CAM_WIDTH, step):
                g = gs_img.get_pixel(x, y)
                if g is not None:
                    bins[g] += 1
        total = sum(bins)
        if total == 0:
            return BLACK_THRESH
        sum_all = sum(i * bins[i] for i in range(256))
        sum_b = 0
        w_b = 0
        max_var = -1.0
        best_t = BLACK_THRESH
        for t in range(256):
            w_b += bins[t]
            if w_b == 0:
                continue
            w_f = total - w_b
            if w_f == 0:
                break
            sum_b += t * bins[t]
            mb = sum_b / w_b
            mf = (sum_all - sum_b) / w_f
            var = w_b * w_f * (mb - mf) * (mb - mf)
            if var > max_var:
                max_var = var
                best_t = t
        return best_t
    except:
        return BLACK_THRESH


def binarize_and_project(gs_img):
    try:
        data = gs_img.bytearray()
    except:
        data = None

    # ROI 范围确定
    if ROI_ENABLE:
        y_start = ROI_Y_START
        y_end   = ROI_Y_END
    else:
        y_start = 0
        y_end   = CAM_HEIGHT

    roi_height = y_end - y_start
    h_samples = roi_height // SAMPLE_STEP + 1
    v_samples = CAM_WIDTH // SAMPLE_STEP + 1
    h_proj = [0] * h_samples
    v_proj = [0] * v_samples
    stride = CAM_WIDTH
    if data is not None:
        total = len(data)
        for sy in range(y_start, y_end, SAMPLE_STEP):
            row_base = sy * stride
            h_idx = (sy - y_start) // SAMPLE_STEP
            for sx in range(0, CAM_WIDTH, SAMPLE_STEP):
                idx = row_base + sx
                if idx < total:
                    if data[idx] != 0:
                        h_proj[h_idx] += 1
                        v_proj[sx // SAMPLE_STEP] += 1
    else:
        for sy in range(y_start, y_end, SAMPLE_STEP):
            h_idx = (sy - y_start) // SAMPLE_STEP
            for sx in range(0, CAM_WIDTH, SAMPLE_STEP):
                g = gs_img.get_pixel(sx, sy)
                if g is not None and g != 0:
                    h_proj[h_idx] += 1
                    v_proj[sx // SAMPLE_STEP] += 1
    return h_proj, v_proj


def find_peak_center(h_proj, v_proj, roi_y_start=0):
    h_len = len(h_proj)
    v_len = len(v_proj)
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
    cy = h_center_sample * SAMPLE_STEP + SAMPLE_STEP // 2 + roi_y_start
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


def detect_cross(img):
    gs = img.to_grayscale(copy=True)
    if USE_OTSU:
        thresh = compute_otsu_threshold(gs)
    else:
        thresh = BLACK_THRESH
    try:
        gs.binary([(0, thresh)])
    except Exception as e:
        print("[binary] failed: " + str(e) + " thresh=" + str(thresh))
        return None
    h_proj, v_proj = binarize_and_project(gs)
    roi_ys = ROI_Y_START if ROI_ENABLE else 0
    cx, cy = find_peak_center(h_proj, v_proj, roi_ys)
    if cx < 0 or cy < 0:
        return None
    return (cx, cy)


def push_and_vote(cx, cy):
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


def build_ack(status, dx, dy):
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
    # 非阻塞读取所有可用数据，查找帧头 AA 55
    data = uart.read()
    if not data or len(data) < REQ_LEN:
        return None
    for i in range(len(data) - REQ_LEN + 1):
        if data[i] == FRAME_SOF0 and data[i + 1] == FRAME_SOF1:
            return data[i + 2]
    return None


def draw_overlay(img, cx, cy, fps, last_status, last_dx, last_dy):
    img.draw_string_advanced(0, 0, 16, "FPS:" + str(fps) + " (proj)", color=(255, 255, 255))
    img.draw_string_advanced(0, 20, 14,
        "last: stat=" + str(last_status) + " dxdy=(" + str(last_dx) + "," + str(last_dy) + ")",
        color=(200, 200, 0))
    if cx is not None and cx >= 0:
        img.draw_cross(int(cx), int(cy), color=(0, 255, 0), size=24, thickness=2)
        img.draw_rectangle(int(cx) - 30, int(cy) - 30, 60, 60, color=(255, 0, 0), thickness=2)
        img.draw_string_advanced(int(cx) + 32, int(cy) - 10, 16,
            "(" + str(int(cx)) + "," + str(int(cy)) + ")", color=(0, 255, 0))
    else:
        img.draw_string_advanced(10, 30, 24, "NO CROSS", color=(255, 0, 0))


def main():
    print("=" * 50)
    print("  K230 projection cross detect (Otsu + vote)")
    print("=" * 50)
    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()
    sensor = init_sensor()
    init_display()
    uart = init_uart()
    sensor.run()
    global fps_start
    fps_start = time.ticks_ms()
    last_status = ACK_STATUS_NONE
    last_dx = 0
    last_dy = 0
    try:
        while True:
            os.exitpoint()
            img = sensor.snapshot()
            cross = detect_cross(img)
            if cross is not None:
                cx_vote, cy_vote = push_and_vote(cross[0], cross[1])
            else:
                cx_vote, cy_vote = push_and_vote(None, None)
            fps = update_fps()
            if cx_vote is not None:
                dx = int(cx_vote - CAM_IMG_CX)
                dy = int(cy_vote - CAM_IMG_CY)
            else:
                dx = 0
                dy = 0
            draw_overlay(img, cx_vote, cy_vote, fps, last_status, last_dx, last_dy)
            Display.show_image(img)
            if frame_count % 30 == 0:
                if cx_vote is not None:
                    print("[DBG] CROSS=(" + str(int(cx_vote)) + "," + str(int(cy_vote)) + ") vote=" + str(len(vote_history)) + " FPS=" + str(fps))
                else:
                    print("[DBG] NO CROSS FPS=" + str(fps))
            req_id = try_read_request(uart)
            if req_id is not None:
                if cx_vote is not None:
                    status = ACK_STATUS_OK
                    dx = int(cx_vote - CAM_IMG_CX)
                    dy = int(cy_vote - CAM_IMG_CY)
                else:
                    status = ACK_STATUS_NONE
                    dx = 0
                    dy = 0
                ack = build_ack(status, dx, dy)
                uart.write(ack)
                last_status = status
                last_dx = dx
                last_dy = dy
                print("[TX] id=" + str(req_id) + " stat=" + str(status) + " dxdy=(" + str(dx) + "," + str(dy) + ")")
    except KeyboardInterrupt:
        print("\n[MAIN] user break")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        sensor.stop()
        Display.deinit()
        uart.deinit()
        print("[INFO] UART 资源已释放")
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
