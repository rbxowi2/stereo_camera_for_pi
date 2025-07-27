#!/usr/bin/python3
import os, sys
import time
import numpy as np
import cv2
from picamera2 import Picamera2
from queue import Queue
from threading import Thread
import RPi.GPIO as GPIO

# os.nice(-20)

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
desired_fps = 15


# 緩衝區設定
n = desired_fps+1               # 每個 block 的 frame 數量
b = 7                           # block 數量
lock_b = [0] * b                # 每個 block 的鎖，0:空閒, 1:儲存中
n_point = 0                     # 寫入緩衝區的索引
b_point = 0                     # 當前 block
b_point_temp = 0                # 上一個 block


save_queue = Queue()            # 儲存任務佇列

save_idx=0
start_time = 0

dir_idx=0
dirpath = f"r{dir_idx:03d}"

# GPIO
record_flag = 0

def button_callback(channel):
    global record_flag , start_time
    
    if record_flag == 0 :
        record_flag = -1
    elif record_flag == 1 :
        record_flag = 2
    else :
        print("not ready...")


GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

GPIO.add_event_detect(10, GPIO.RISING, callback=button_callback, bouncetime=200) # 200毫秒消除彈跳



# 背景儲存執行緒，持續從佇列取得任務並儲存
def save_worker():
    global save_idx ,dirpath
    while True:
        b_in = save_queue.get()
        if b_in is None:          # 結束訊號
            break
        lock_b[b_in] = 1          # 標記該 block 正在儲存
        for i in range(n):
            raw_filename = f"{dirpath}/frame_{save_idx:04d}.raw"
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

print("ready to record")
try:
    while True:
        if record_flag == 1 :
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
                
        elif record_flag ==2:
            
            end_time = time.time()
            
            print("錄製完成，共保存 {} 幀。".format(frame_count))
            print("錄製時間 {}s" .format(end_time-start_time))
            
            #儲存剩餘畫面
            save_queue.put(b_point_temp)
            
            while not save_queue.empty():
                time.sleep(0.01)
            for i in range(b):
                while lock_b[i]:   # 等待所有塊儲存完
                    time.sleep(0.01)
                    
            record_flag = 0
            ### set led off
            print("ready to record")
            
            # # 結束儲存工作執行緒
            # save_queue.put(None)
            # thread.join()
            # break
            
        elif record_flag == -1: #init 
            
            dir_idx += 1
            dirpath = f"r{dir_idx:03d}"
            os.mkdir( dirpath, 0o777 );
            
            frame_count = 0
            n_point = 0
            b_point = 0
            b_point_temp = 0
            save_idx = 0
            
            print("{} recording...".format(dirpath))
            ### set led on
            start_time = time.time()
            record_flag = 1
            
        else:
            # 沒收到新幀，短暫休息避免 CPU 全忙
            time.sleep(0.001)


except KeyboardInterrupt:
    print("錄製中斷")


# 停止相機
finally:
    picam2a.stop()
    picam2b.stop()
    # print("錄製完成，共保存 {} 幀。".format(frame_count))
    del frames_in_memory
    GPIO.cleanup() # Clean up
    # print("所有影像資料儲存完成。")
    # print(end_time-start_time)


