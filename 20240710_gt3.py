# Untitled - By: NAOKO - 水 7 10 2024

import sensor, image, time, utime
from machine import UART
from Maix import GPIO
from fpioa_manager import fm
from modules import ws2812

class_ws2812 = ws2812(8, 1)

fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)

fm.register(18, fm.fpioa.GPIO1)
ButonA = GPIO(GPIO.GPIO1, GPIO.IN, GPIO.PULL_UP)
fm.register(19, fm.fpioa.GPIO2)
ButonB = GPIO(GPIO.GPIO2, GPIO.IN, GPIO.PULL_UP)

uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
#sensor.set_vflip(True) # 上下反転
#sensor.set_hmirror(True) # 左右反転
sensor.set_auto_gain(False)         # オートゲインをオフ
sensor.set_auto_whitebal(0)         # オートホワイトバランスをオフ
sensor.skip_frames(time=2000)       # 2秒間フレームをスキップして安定化
sensor.run(1)

while False:
    uart.write('TEST\n')
    utime.sleep_ms(100)

target_lab_threshold = (30,100,15,127,15,127)
IMAGE_TOTAL_PIXELS = 320 * 240  # 画像の総ピクセル数を定数として定義

while True:
    img = sensor.snapshot()

    if ButonA.value() == 0:
        class_ws2812.set_led(0, (50, 0, 10))
        class_ws2812.display()
        time.sleep(0.5)

    if ButonB.value() == 0:
        class_ws2812.set_led(0, (0, 50, 50))
        class_ws2812.display()
        time.sleep(0.5)

    blobs = img.find_blobs([target_lab_threshold], x_stride=2, y_stride=2, pixels_threshold=100, merge=False, margin=20)
    if blobs:
        max_blob_red = max(blobs, key=lambda b: b.area())  # 面積が最大の領域を取得
        max_per_red = max_blob_red.area() / IMAGE_TOTAL_PIXELS

        img.draw_rectangle(max_blob_red.rect())  # 検出した色を矩形で囲む
        img.draw_cross(max_blob_red.cx(), max_blob_red.cy())  # 検出した色の中心に十字を描く

        sendstr = "R{},{}, {:.2f}".format(max_blob_red.cx(), max_blob_red.cy(), max_per_red)
        uart.write(sendstr + "\n")  # 文字列を一括で送信
        print(max_blob_red.cx(), max_blob_red.cy(), max_per_red)
    else:
        sendstr = "R0,0,0.0O0,0,0.0Y0,0,0.0"
        uart.write(sendstr + "\n")  # 文字列を一括で送信
        print("0,0")

    time.sleep(0.001)  # 短い遅延
