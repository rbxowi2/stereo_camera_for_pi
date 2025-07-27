#!/usr/bin/python3
import time
import numpy as np
import cv2
from picamera2 import Picamera2
import os

os.nice(-20)

# 畫面設定
resolution = (1944, 1944)
frame_width, frame_height = resolution
channels = 3

ScalerCrop_flag = True
crop_x = 200  # 向右偏移
crop_y = 0  # 向上偏移
crop_width = 1944
crop_height = 1944

# 設定幀率
desired_fps = 15

# 設定錄製時間
duration = 20  # 秒
max_frames = duration * desired_fps

# 初始化兩台相機
picam2a = Picamera2(0)
picam2b = Picamera2(1)

# 建立相機配置
config_a = picam2a.create_video_configuration(
    main={"size": resolution, "format": "RGB888"}
)

config_b = picam2b.create_video_configuration(
    main={"size": resolution, "format": "RGB888"}
)

picam2a.configure(config_a)
picam2b.configure(config_b)


picam2a.set_controls({
    "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
})

picam2b.set_controls({
    "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
})

if ScalerCrop_flag:
    picam2a.set_controls({"ScalerCrop": (crop_x, crop_y, crop_width, crop_height)})
    picam2b.set_controls({"ScalerCrop": (crop_x, crop_y, crop_width, crop_height)})


# 預先分配記憶體 (frame_count, 高, 寬, 通道)
frames_in_memory = np.empty((max_frames, frame_height, frame_width*2, channels), dtype=np.uint8)

# 旗標變數，標示有新幀完成
frame_a_ready = False
frame_b_ready = False

def post_a_callback(request):
    global frame_a_ready 
    frame_a_ready = True  # 幀完成，標記為 True

def post_b_callback(request):
    global frame_b_ready
    frame_b_ready = True  # 幀完成，標記為 True

picam2a.post_callback = post_a_callback
picam2b.post_callback = post_b_callback

picam2a.start()
picam2b.start()

time.sleep(1)


frame_count = 0
start_time = time.time()
try:
    while time.time() - start_time < duration:
        if frame_count >= max_frames:
            break  # 避免越界
        # 擷取單幀
        if frame_b_ready and frame_a_ready:
            frame_a_ready = False  # 重設旗標
            frame_b_ready = False  # 重設旗標
            #frame = picam2.capture_array("main")
            # 儲存影像到預分配記憶體中   
            frames_in_memory[frame_count][:, :resolution[0]] = picam2b.capture_array("main")
            frames_in_memory[frame_count][:, resolution[0]:] = picam2a.capture_array("main")
            frame_count += 1
        else:
            # 沒收到新幀，短暫休息避免 CPU 全忙
            time.sleep(0.01)

except KeyboardInterrupt:
    print("錄製中斷")

# def capture_and_combine():
    # picam2b.start()
    # time.sleep(0.5)
    # frame_b = picam2b.capture_array()
    # picam2b.stop()

    # picam2a.start()
    # time.sleep(0.5)
    # frame_a = picam2a.capture_array()
    # picam2a.stop()

    # # 合併左右畫面
    # combined = np.empty((resolution[1], resolution[0] * 2, 3), dtype=frame_a.dtype)
    # combined[:, :resolution[0]] = frame_b
    # combined[:, resolution[0]:] = frame_a

    # return combined

# 停止相機
picam2a.stop()
picam2b.stop()

# 將影像資料儲存到檔案
print(f"錄製完成，共保存 {frame_count} 幀，開始將影像資料儲存到檔案...")

for idx in range(frame_count):
    raw_filename = f"frame_{idx:04d}.raw"
    frames_in_memory[idx].tofile(raw_filename)
    #print(f"保存 {raw_filename}")

# 清理記憶體（可省略）
del frames_in_memory

print("所有影像資料儲存完成。")
