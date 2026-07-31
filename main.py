import sensor, image, time
from machine import UART
from pyb import LED
import ustruct

# ========== 传感器初始化 ==========
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)          # 320x240
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=4000)
white = LED(4)                             # 照明灯

# 色域阈值 (LAB格式，需根据实际环境微调)
red_threshold    = ((25, 90, 25, 127, -10, 30))      # 红色
yellow_threshold = ((45, 100, -20, 20, 30, 80))      # 黄色
blue_threshold   = ((15, 80, -20, 15, -80, -25))     # 蓝色
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

# ========== 发送应答帧（8字节，与十字检测协议一致）==========
def send_response(status, mx, my):
    """
    status: 0=未检测到, 1=红, 2=黄, 3=蓝
    mx, my: 色块中心像素坐标 (uint16)
    帧格式: [0xAA, 0x55, STATUS, MX_L, MX_H, MY_L, MY_H, XOR]  共8字节
    """
    buf = bytearray(8)
    buf[0] = 0xAA
    buf[1] = 0x55
    buf[2] = status & 0xFF
    buf[3] = mx & 0xFF          # MX 低字节
    buf[4] = (mx >> 8) & 0xFF   # MX 高字节
    buf[5] = my & 0xFF          # MY 低字节
    buf[6] = (my >> 8) & 0xFF   # MY 高字节
    
    # 前7字节 XOR 校验
    xor = 0
    for i in range(7):
        xor ^= buf[i]
    buf[7] = xor
    
    uart.write(buf)

# ========== 主循环：非阻塞请求-应答（仿十字检测通讯逻辑）==========
# 请求帧解析状态机: 等待 STM32 发 [0xAA, 0x55, ID] 三字节
req_state = 0

while(True):
    clock.tick()
    white.on()
    
    # ---- 非阻塞检查串口，逐字节解析请求帧 ----
    request_ready = False
    while uart.any():
        b = uart.readchar()
        if b == -1:
            break
        
        if req_state == 0:
            if b == 0xAA:
                req_state = 1
        
        elif req_state == 1:
            if b == 0x55:
                req_state = 2
            elif b == 0xAA:
                req_state = 1   # 重新匹配帧头
            else:
                req_state = 0
        
        elif req_state == 2:
            # 收到 ID 字节（第三字节），请求帧完整
            req_state = 0       # 复位状态机
            request_ready = True
            break               # 立即处理本次请求
    
    # ---- 收到完整请求 → 执行一次色块检测并应答 ----
    if request_ready:
        try:
            img = sensor.snapshot()
            
            # 三色识别（选面积最大的色块作为"突出色块"）
            candidates = []    # (color_id, blob)，color_id: 1=红, 2=黄, 3=蓝
            colors = ((255, 0, 0), (255, 255, 0), (0, 0, 255))
            labels = ("R", "Y", "B")
            
            for b in img.find_blobs([red_threshold], merge=True):
                if b.pixels() >= MIN_AREA:
                    candidates.append((1, b))     # 1=红
            for b in img.find_blobs([yellow_threshold], merge=True):
                if b.pixels() >= MIN_AREA:
                    candidates.append((2, b))     # 2=黄
            for b in img.find_blobs([blue_threshold], merge=True):
                if b.pixels() >= MIN_AREA:
                    candidates.append((3, b))     # 3=蓝
            
            if candidates:
                best = max(candidates, key=lambda x: x[1].pixels())
                color_id, blob = best
                mx, my = blob[5], blob[6]
                mw, mh = blob[2], blob[3]
                
                img.draw_rectangle(blob[0:4], color=colors[color_id - 1])
                img.draw_cross(mx, my, color=colors[color_id - 1], size=8)
                img.draw_string(5, 5, "COLOR: " + labels[color_id - 1],
                                color=colors[color_id - 1], scale=2)
                
                send_response(color_id, mx, my)
                print(labels[color_id - 1], ":", mx, my, mw, mh, "pixels:", blob.pixels())
            else:
                img.draw_string(5, 5, "COLOR: NONE", color=(255,255,255), scale=2)
                send_response(0, 0, 0)    # 未检测到色块
                print("no color detected")
            
            print("fps:", clock.fps())
            
        except Exception as e:
            print("ERROR:", e)
            send_response(0, 0, 0)    # 异常时也发送空应答
    
    time.sleep_ms(10)