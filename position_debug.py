# position_debug.py (projection + diagnostic)
# K230 projection cross detect + Otsu + vote + diagnostic panel
import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import image

CAM_WIDTH = 800
CAM_HEIGHT = 480
CAM_IMG_CX = CAM_WIDTH // 2
CAM_IMG_CY = CAM_HEIGHT // 2
USE_OTSU = True
BLACK_THRESH = 80
SAMPLE_STEP = 8
MIN_PEAK_RATIO = 0.08
VOTE_WINDOW = 5

# ROI (Region of Interest) - 排除车体自身区域
# 摄像头装在车尾, 车体出现在画面下部, 裁剪掉底部若干行
ROI_ENABLE = True           # 是否启用 ROI
ROI_Y_START = 0             # ROI 起始 Y(含), 保持 0(顶部不变)
ROI_Y_END   = 360           # ROI 结束 Y(不含), 即排除 Y>=360 的车体区域(可实测调)

SHOW_DIAG_PANE = True
DIAG_PANE_X = 560
DIAG_PANE_W = CAM_WIDTH - DIAG_PANE_X
DIAG_THUMB_W = 240
DIAG_THUMB_H = 144
SAMPLE_SIZE = 40
CALIB_HORIZ_MM = 0
CALIB_VERT_MM = 0
GRID_MINOR_PX = 50
GRID_MAJOR_PX = 100

frame_count = 0
fps_start = 0
fps_value = 0
vote_history = []
boot_ms = 0
CALIB_MM_PER_PIX_X = 0.0
CALIB_MM_PER_PIX_Y = 0.0
last_thresh = 0
last_black_pct = 0.0
last_sample_gray = 0


def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def init_display():
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


def count_black_pct(gs_img, thresh):
    try:
        cnt = 0
        total = 0
        step = 8
        for y in range(0, CAM_HEIGHT, step):
            for x in range(0, CAM_WIDTH, step):
                g = gs_img.get_pixel(x, y)
                if g is not None:
                    total += 1
                    if g < thresh:
                        cnt += 1
        if total == 0:
            return 0.0
        return 100.0 * cnt / total
    except:
        return 0.0


def sample_center_gray(gs_img):
    try:
        x0 = CAM_IMG_CX - SAMPLE_SIZE // 2
        y0 = CAM_IMG_CY - SAMPLE_SIZE // 2
        s = 0
        n = 0
        for y in range(y0, y0 + SAMPLE_SIZE, 2):
            for x in range(x0, x0 + SAMPLE_SIZE, 2):
                g = gs_img.get_pixel(x, y)
                if g is not None:
                    s += g
                    n += 1
        if n == 0:
            return 0
        return s // n
    except:
        return 0


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
    global last_thresh, last_black_pct, last_sample_gray
    gs = img.to_grayscale(copy=True)
    if USE_OTSU:
        thresh = compute_otsu_threshold(gs)
    else:
        thresh = BLACK_THRESH
    black_pct = count_black_pct(gs, thresh)
    sample_gray = sample_center_gray(gs)
    last_thresh = thresh
    last_black_pct = black_pct
    last_sample_gray = sample_gray
    try:
        gs.binary([(0, thresh)])
    except Exception as e:
        return None, (thresh, black_pct, sample_gray, None)
    bin_img = gs
    h_proj, v_proj = binarize_and_project(gs)
    roi_ys = ROI_Y_START if ROI_ENABLE else 0
    cx, cy = find_peak_center(h_proj, v_proj, roi_ys)
    if cx < 0 or cy < 0:
        return None, (thresh, black_pct, sample_gray, bin_img)
    return (cx, cy), (thresh, black_pct, sample_gray, bin_img)


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


def draw_calibration_grid(img):
    right_limit = DIAG_PANE_X if SHOW_DIAG_PANE else CAM_WIDTH
    for x in range(0, right_limit, GRID_MINOR_PX):
        img.draw_line(x, 0, x, CAM_HEIGHT, color=(40, 40, 40), thickness=1)
    for y in range(0, CAM_HEIGHT, GRID_MINOR_PX):
        img.draw_line(0, y, right_limit, y, color=(40, 40, 40), thickness=1)
    for x in range(0, right_limit, GRID_MAJOR_PX):
        img.draw_line(x, 0, x, CAM_HEIGHT, color=(80, 80, 80), thickness=1)
        img.draw_string_advanced(x + 2, CAM_HEIGHT - 110, 12, str(x), color=(120, 120, 120))
    for y in range(0, CAM_HEIGHT, GRID_MAJOR_PX):
        img.draw_line(0, y, right_limit, y, color=(80, 80, 80), thickness=1)
        img.draw_string_advanced(2, y + 2, 12, str(y), color=(120, 120, 120))


def draw_center_reference(img):
    img.draw_line(CAM_IMG_CX, 0, CAM_IMG_CX, CAM_HEIGHT, color=(255, 255, 255), thickness=1)
    img.draw_line(0, CAM_IMG_CY, DIAG_PANE_X, CAM_IMG_CY, color=(255, 255, 255), thickness=1)
    img.draw_string_advanced(CAM_IMG_CX + 4, CAM_IMG_CY + 4, 12, "center(" + str(CAM_IMG_CX) + "," + str(CAM_IMG_CY) + ")", color=(255, 255, 255))


def draw_diagnostic_pane(img, diag):
    if not SHOW_DIAG_PANE:
        return
    thresh, black_pct, sample_gray, bin_img = diag
    img.draw_rectangle(DIAG_PANE_X, 0, DIAG_PANE_W, CAM_HEIGHT, color=(20, 20, 20), thickness=True)
    img.draw_string_advanced(DIAG_PANE_X + 6, 36, 14, "BIN IMAGE", color=(0, 255, 255))
    thumb_x = DIAG_PANE_X + 4
    thumb_y = 56
    if bin_img is not None:
        try:
            img.draw_image(bin_img, thumb_x, thumb_y, x_scale=DIAG_THUMB_W / CAM_WIDTH, y_scale=DIAG_THUMB_H / CAM_HEIGHT)
        except Exception as e:
            img.draw_string_advanced(thumb_x, thumb_y, 12, "draw_image err", color=(255, 0, 0))
    img.draw_rectangle(thumb_x, thumb_y, DIAG_THUMB_W, DIAG_THUMB_H, color=(100, 100, 100), thickness=1)
    y = thumb_y + DIAG_THUMB_H + 10
    img.draw_string_advanced(DIAG_PANE_X + 6, y, 14, "thresh=" + str(thresh) + " (" + ("Otsu" if USE_OTSU else "manual") + ")", color=(255, 255, 0))
    y += 20
    if black_pct > 5:
        bc = (0, 255, 0)
    elif black_pct > 1:
        bc = (255, 255, 0)
    else:
        bc = (255, 0, 0)
    img.draw_string_advanced(DIAG_PANE_X + 6, y, 14, "black=" + str(black_pct) + "%", color=bc)
    y += 20
    img.draw_string_advanced(DIAG_PANE_X + 6, y, 14, "center gray=" + str(sample_gray), color=(255, 200, 0))


def draw_overlay(img, cx, cy, fps, dx_px, dy_px, diag):
    draw_calibration_grid(img)
    draw_center_reference(img)
    if cx is not None and cx >= 0:
        img.draw_cross(int(cx), int(cy), color=(0, 255, 0), size=32, thickness=3)
        img.draw_rectangle(int(cx) - 40, int(cy) - 40, 80, 80, color=(255, 0, 0), thickness=2)
        img.draw_string_advanced(int(cx) + 44, int(cy) - 12, 16, "(" + str(int(cx)) + "," + str(int(cy)) + ")", color=(0, 255, 0))
    draw_diagnostic_pane(img, diag)
    vote_n = len(vote_history)
    img.draw_rectangle(0, 0, CAM_WIDTH, 30, color=(0, 0, 0), thickness=True)
    img.draw_string_advanced(8, 6, 16, "FPS:" + str(fps) + " vote:" + str(vote_n) + "/" + str(VOTE_WINDOW) + " thresh:" + str(last_thresh) + " black:" + str(last_black_pct) + "%", color=(255, 255, 255))
    img.draw_string_advanced(CAM_WIDTH - 130, 6, 16, "[DIAG]", color=(0, 255, 255))
    img.draw_rectangle(0, CAM_HEIGHT - 90, DIAG_PANE_X, 90, color=(0, 0, 0), thickness=True)
    if cx is not None and cx >= 0:
        img.draw_string_advanced(10, CAM_HEIGHT - 84, 40, "OK CROSS", color=(0, 255, 0))
        sgn_dx = "+" if dx_px >= 0 else ""
        sgn_dy = "+" if dy_px >= 0 else ""
        img.draw_string_advanced(10, CAM_HEIGHT - 40, 20, "cx,cy=(" + str(int(cx)) + "," + str(int(cy)) + ")  DX,DY=(" + sgn_dx + str(dx_px) + "," + sgn_dy + str(dy_px) + ") px", color=(255, 255, 255))
    else:
        img.draw_string_advanced(10, CAM_HEIGHT - 84, 40, "NO CROSS", color=(255, 0, 0))
        img.draw_string_advanced(10, CAM_HEIGHT - 40, 18, "black=" + str(last_black_pct) + "% thresh=" + str(last_thresh) + " gray=" + str(last_sample_gray), color=(255, 200, 0))


def compute_calib():
    global CALIB_MM_PER_PIX_X, CALIB_MM_PER_PIX_Y
    if CALIB_HORIZ_MM > 0:
        CALIB_MM_PER_PIX_X = CALIB_HORIZ_MM / CAM_WIDTH
    if CALIB_VERT_MM > 0:
        CALIB_MM_PER_PIX_Y = CALIB_VERT_MM / CAM_HEIGHT


def main():
    global frame_count, fps_start, boot_ms
    compute_calib()
    os.exitpoint(os.EXITPOINT_ENABLE)
    MediaManager.init()
    sensor = init_sensor()
    init_display()
    sensor.run()
    fps_start = time.ticks_ms()
    boot_ms = fps_start
    try:
        while True:
            os.exitpoint()
            img = sensor.snapshot()
            cross, diag = detect_cross(img)
            if cross is not None:
                cx_vote, cy_vote = push_and_vote(cross[0], cross[1])
            else:
                cx_vote, cy_vote = push_and_vote(None, None)
            fps = update_fps()
            if cx_vote is not None:
                dx_px = int(cx_vote - CAM_IMG_CX)
                dy_px = int(cy_vote - CAM_IMG_CY)
            else:
                dx_px = 0
                dy_px = 0
            draw_overlay(img, cx_vote, cy_vote, fps, dx_px, dy_px, diag)
            Display.show_image(img)
            now_ms = time.ticks_ms()
            t_rel = time.ticks_diff(now_ms, boot_ms)
            status_str = "OK" if cx_vote is not None else "NONE"
            cx_str = str(int(cx_vote)) if cx_vote is not None else "0"
            cy_str = str(int(cy_vote)) if cx_vote is not None else "0"
            print("DBG," + str(t_rel) + "," + status_str + "," + cx_str + "," + cy_str + "," + str(dx_px) + "," + str(dy_px) + "," + str(len(vote_history)) + "," + str(fps) + "," + str(last_thresh) + "," + str(last_black_pct) + "," + str(last_sample_gray))
    except KeyboardInterrupt:
        print("\n[MAIN] user break")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
