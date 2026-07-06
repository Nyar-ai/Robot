import sensor, image, time,pyb
from machine import UART
from pyb import LED
import ustruct

# ========== 传感器初始化 ==========
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=4000)
white = LED(4) #照明灯
# 色域阈值 (LAB格式，需根据实际环境微调)
red_threshold    = ((25, 90, 25, 127, -10, 30))     # 红色 (A>25, B>-10)
yellow_threshold = ((45, 100, -20, 20, 30, 80))     # 黄色 (B 30~80)
blue_threshold   = ((15, 80, -20, 15, -80, -25))    # 蓝色 (A<15, B<-25)
MIN_AREA = 80    # 最小面积阈值，过滤噪声

clock = time.clock()

uart = UART(2, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ========== 找最大色块 ==========
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob

# ========== 发送单色数据 ==========
def send_color(color_id, mx, my, mw, mh):
    """color_id: 0=红(2C12), 1=黄(2D13), 2=蓝(2E14)"""
    headers = ((0x2C, 0x12), (0x2D, 0x13), (0x2E, 0x14))
    h0, h1 = headers[color_id]
    FH = bytearray([h0, h1,
                    mx & 0xFF, mx >> 8,
                    my & 0xFF, my >> 8,
                    mw & 0xFF, mw >> 8,
                    mh & 0xFF, mh >> 8, 0x5B])
    uart.write(FH)

# ========== 主循环 ==========
# 帧匹配状态机 (启动: AA BB 5B, 停止: CC DD 5B)
state = 0
active = False

while(True):
    clock.tick()
    white.on()
    # ---- 串口帧匹配状态机 ----
    while uart.any():
        b = uart.readchar()
        if b == -1:
            break

        if state == 0:
            if b == 0xAA:
                state = 1
            elif b == 0xCC:
                state = 3
        elif state == 1:
            if b == 0xBB:
                state = 2
            elif b == 0xAA:
                state = 1
            elif b == 0xCC:
                state = 3
            else:
                state = 0
        elif state == 2:
            if b == 0x5B:
                active = True
                print("START")
                #sensor.skip_frames(time=300)  # 传感器稳定预热
            elif b == 0xAA:
                state = 1
            elif b == 0xCC:
                state = 3
            else:
                state = 0
        elif state == 3:
            if b == 0xDD:
                state = 4
            elif b == 0xCC:
                state = 3
            elif b == 0xAA:
                state = 1
            else:
                state = 0
        elif state == 4:
            if b == 0x5B:
                active = False
                print("STOP")
            elif b == 0xCC:
                state = 3
            elif b == 0xAA:
                state = 1
            else:
                state = 0

    if not active:
        time.sleep_ms(50)
        continue

    try:
        img = sensor.snapshot()

        # ---- 三色识别（找最突出的色块并判色） ----
        candidates = []   # (color_id, blob)
        colors = ((255, 0, 0), (255, 255, 0), (0, 0, 255))
        labels = ("R", "Y", "B")

        for b in img.find_blobs([red_threshold], merge=True):
            if b.pixels() >= MIN_AREA:
                candidates.append((0, b))
        for b in img.find_blobs([yellow_threshold], merge=True):
            if b.pixels() >= MIN_AREA:
                candidates.append((1, b))
        for b in img.find_blobs([blue_threshold], merge=True):
            if b.pixels() >= MIN_AREA:
                candidates.append((2, b))

        if candidates:
            # 选面积最大的色块作为"突出色块"
            best = max(candidates, key=lambda x: x[1].pixels())
            color_id, blob = best

            mx, my = blob[5], blob[6]
            mw, mh = blob[2], blob[3]

            img.draw_rectangle(blob[0:4], color=colors[color_id])
            img.draw_cross(mx, my, color=colors[color_id], size=8)
            img.draw_string(5, 5, "COLOR: " + labels[color_id],
                            color=colors[color_id], scale=2)
            send_color(color_id, mx, my, mw, mh)
            print(labels[color_id], ":", mx, my, mw, mh, "pixels:", blob.pixels())
        else:
            img.draw_string(5, 5, "COLOR: NONE", color=(255, 255, 255), scale=2)
            print("no color detected")

        print("fps:", clock.fps())
    except Exception as e:
        print("ERROR:", e)

    time.sleep_ms(10)
