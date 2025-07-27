#!/usr/bin/python3
import os
import time
import numpy as np
import cv2
from picamera2 import Picamera2
from queue import Queue
from threading import Thread

os.nice(-20)

# 畫面設定
resolution = (1944 , 1944)
frame_width, frame_height = resolution
channels = 3

ScalerCrop_flag = True
crop_x = 200  # 向右偏移
crop_y = 0  # 向上偏移
crop_width = 1944
crop_height = 1944

# 設定幀率
desired_fps = 10

# 設定錄製時間
duration = 60*2  # 秒
max_frames = duration * desired_fps


# 緩衝區設定
n = desired_fps+1               # 每個 block 的 frame 數量
b = 7                           # block 數量
# buff = [None] * (n * b)       # 模擬緩衝區（n*b 張 frame）
lock_b = [0] * b                # 每個 block 的鎖，0:空閒, 1:儲存中
n_point = 0                     # 寫入緩衝區的索引
b_point = 0                     # 當前 block
b_point_temp = 0                # 上一個 block
start = True                    # 主迴圈開關（模擬啟動）

save_queue = Queue()            # 儲存任務佇列

save_idx=0


# 背景儲存執行緒，持續從佇列取得任務並儲存
def save_worker():
    global save_idx
    while True:
        b_in = save_queue.get()
        if b_in is None:          # 結束訊號
            break
        lock_b[b_in] = 1          # 標記該 block 正在儲存
        for i in range(n):
            raw_filename = f"frame_{save_idx:04d}.raw"
            frames_in_memory[i +(b_in * n)].tofile(raw_filename)
            save_idx +=1
            if save_idx >= frame_count:
                break
        lock_b[b_in] = 0          # 解除鎖
        save_queue.task_done()
        

# 初始化兩台相機
picam2a = Picamera2(0)
picam2b = Picamera2(1)

# 建立相機配置
config_a = picam2a.create_video_configuration(
    main={"size": resolution, "format": "RGB888"},
    # buffer_count=12  # 與 block 數量一致
)

config_b = picam2b.create_video_configuration(
    main={"size": resolution, "format": "RGB888"},
    # buffer_count=12  # 與 block 數量一致
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
frames_in_memory = np.empty(((n * b), frame_height, frame_width*2, channels), dtype=np.uint8)

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


# 啟動儲存工作執行緒
thread = Thread(target=save_worker)
thread.start()

frame_count = 0

time.sleep(3)

start_time = time.time()

try:
    while time.time() - start_time < duration:
        if frame_count < max_frames:
            # 擷取單幀
            if frame_b_ready and frame_a_ready:
                frame_a_ready = False  # 重設旗標
                frame_b_ready = False  # 重設旗標
                #frame = picam2.capture_array("main")
                # 儲存影像到預分配記憶體中   
                frames_in_memory[n_point][:, :resolution[0]] = picam2b.capture_array("main")
                frames_in_memory[n_point][:, resolution[0]:] = picam2a.capture_array("main")
                
                frame_count += 1
                n_point += 1
                
                #如果超過緩衝區大小 重設指標
                if n_point >= n * b:
                    n_point = 0
                    
                #計算下一個區塊位置
                b_point = n_point // n
                
                #是否切換到新區塊 如果切換新區塊儲存目前區塊 如果新區塊占用中將主線程阻塞
                if b_point_temp != b_point:
                    while lock_b[b_point]:    # 阻塞等待新區塊釋放
                        time.sleep(0.001)
                    save_queue.put(b_point_temp)   # 送到佇列由背景儲存
                    b_point_temp = b_point
                
            else:
                # 沒收到新幀，短暫休息避免 CPU 全忙
                time.sleep(0.001)
    end_time = time.time()
    
    #儲存剩餘畫面
    save_queue.put(b_point_temp)
    
    for i in range(b):
        while lock_b[i]:   # 等待所有塊儲存完
            time.sleep(0.01)
    
    # 結束儲存工作執行緒
    save_queue.put(None)
    thread.join()


except KeyboardInterrupt:
    print("錄製中斷")


# 停止相機
finally:
    picam2a.stop()
    picam2b.stop()
    print("錄製完成，共保存 {} 幀。".format(frame_count))
    del frames_in_memory
    print("所有影像資料儲存完成。")
    print(end_time-start_time)


