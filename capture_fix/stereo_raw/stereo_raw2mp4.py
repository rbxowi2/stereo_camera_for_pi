#!/usr/bin/python3
import os
import time
import numpy as np
import cv2
import shutil
from picamera2 import Picamera2
from queue import Queue
from threading import Thread

os.nice(-20)

# 畫面設定
resolution = (1944, 1944)
frame_width, frame_height = resolution
channels = 3

ScalerCrop_flag = True
crop_x = 200
crop_y = 0
crop_width = 1944
crop_height = 1944

# 設定幀率與錄製時間
desired_fps = 10
duration = 60 * 2
max_frames = duration * desired_fps

# 緩衝區設定
n = desired_fps + 1
b = 7
lock_b = [0] * b
n_point = 0
b_point = 0
b_point_temp = 0
start = True
save_queue = Queue()
save_idx = 0
frame_count = 0

# MP4 設定
ramdisk_path = "/dev/shm/output_temp.mp4"  # RAMDisk
ssd_path = "./final_output.mp4"            # 實體磁碟儲存
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fourcc = cv2.VideoWriter_fourcc(*'XVID')

video_writer = cv2.VideoWriter(ramdisk_path, fourcc, desired_fps, (frame_width * 2, frame_height))

# 背景儲存執行緒
def save_worker():
    global save_idx
    while True:
        b_in = save_queue.get()
        if b_in is None:
            break
        lock_b[b_in] = 1
        for i in range(n):
            frame = frames_in_memory[i + (b_in * n)]
            video_writer.write(frame)
            save_idx += 1
            # if save_idx >= frame_count:
                # break
        lock_b[b_in] = 0
        save_queue.task_done()

# 初始化相機
picam2a = Picamera2(0)
picam2b = Picamera2(1)

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

# 預分配記憶體空間
frames_in_memory = np.empty((n * b, frame_height, frame_width * 2, channels), dtype=np.uint8)

# 幀完成旗標
frame_a_ready = False
frame_b_ready = False

def post_a_callback(request):
    global frame_a_ready
    frame_a_ready = True

def post_b_callback(request):
    global frame_b_ready
    frame_b_ready = True

picam2a.post_callback = post_a_callback
picam2b.post_callback = post_b_callback

picam2a.start()
picam2b.start()

# 啟動儲存執行緒
thread = Thread(target=save_worker)
thread.start()

time.sleep(3)
start_time = time.time()

try:
    while time.time() - start_time < duration:
        if frame_count < max_frames:
            if frame_b_ready and frame_a_ready:
                frame_a_ready = False
                frame_b_ready = False

                # 合併左右相機畫面
                frames_in_memory[n_point][:, :resolution[0]] = picam2b.capture_array("main")
                frames_in_memory[n_point][:, resolution[0]:] = picam2a.capture_array("main")

                frame_count += 1
                n_point += 1

                if n_point >= n * b:
                    n_point = 0

                b_point = n_point // n

                if b_point_temp != b_point:
                    while lock_b[b_point]:
                        time.sleep(0.001)
                    save_queue.put(b_point_temp)
                    b_point_temp = b_point
            else:
                time.sleep(0.001)

    end_time = time.time()

    # 儲存最後一個 block
    save_queue.put(b_point_temp)

    for i in range(b):
        while lock_b[i]:
            time.sleep(0.01)

    # 結束儲存執行緒
    save_queue.put(None)
    thread.join()

except KeyboardInterrupt:
    print("錄製中斷")

finally:
    picam2a.stop()
    picam2b.stop()
    print("錄製完成，共保存 {} 幀。".format(frame_count))

    video_writer.release()
    shutil.move(ramdisk_path, ssd_path)
    print(f"影片已儲存至 {ssd_path}")

    del frames_in_memory
    print("所有影像資料儲存完成。")
    print(f"總耗時：{end_time - start_time:.2f} 秒")
# import os
# import time
# import numpy as np
# import cv2
# import shutil
# from picamera2 import Picamera2
# from queue import Queue
# from threading import Thread

# os.nice(-20)

# # 畫面設定
# resolution = (1944 , 1944)
# frame_width, frame_height = resolution
# channels = 3

# ScalerCrop_flag = True
# crop_x = 200
# crop_y = 0
# crop_width = 1944
# crop_height = 1944

# desired_fps = 10
# duration = 60*2
# max_frames = duration * desired_fps

# n = desired_fps + 1
# b = 7

# lock_b = [0] * b
# n_point = 0
# b_point = 0
# b_point_temp = 0

# save_queue = Queue()
# save_idx=0

# # 影片儲存設定，改成寫 MP4 至 RAMDisk，最後存到 SSD
# ramdisk_path = "/dev/shm/output_temp.mp4"
# ssd_path = "./final_output.mp4"
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 可改成 'avc1' 試試 H264
# video_writer = cv2.VideoWriter(ramdisk_path, fourcc, desired_fps, (frame_width*2, frame_height))

# frames_in_memory = np.empty(((n * b), frame_height, frame_width*2, channels), dtype=np.uint8)

# frame_a_ready = False
# frame_b_ready = False

# def post_a_callback(request):
    # global frame_a_ready 
    # frame_a_ready = True

# def post_b_callback(request):
    # global frame_b_ready
    # frame_b_ready = True

# def save_worker():
    # global save_idx
    # while True:
        # b_in = save_queue.get()
        # if b_in is None:
            # break
        # lock_b[b_in] = 1
        # for i in range(n):
            # idx = i + b_in * n
            # frame = frames_in_memory[idx]
            # video_writer.write(frame)
            # save_idx += 1
            # if save_idx >= max_frames:
                # break
        # lock_b[b_in] = 0
        # save_queue.task_done()

# picam2a = Picamera2(0)
# picam2b = Picamera2(1)

# config_a = picam2a.create_video_configuration(
    # main={"size": resolution, "format": "RGB888"},
# )
# config_b = picam2b.create_video_configuration(
    # main={"size": resolution, "format": "RGB888"},
# )

# picam2a.configure(config_a)
# picam2b.configure(config_b)

# picam2a.set_controls({
    # "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
# })
# picam2b.set_controls({
    # "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
# })

# if ScalerCrop_flag:
    # picam2a.set_controls({"ScalerCrop": (crop_x, crop_y, crop_width, crop_height)})
    # picam2b.set_controls({"ScalerCrop": (crop_x, crop_y, crop_width, crop_height)})

# picam2a.post_callback = post_a_callback
# picam2b.post_callback = post_b_callback

# picam2a.start()
# picam2b.start()

# thread = Thread(target=save_worker)
# thread.start()

# frame_count = 0
# time.sleep(3)
# start_time = time.time()

# try:
    # while time.time() - start_time < duration:
        # if frame_count < max_frames:
            # if frame_a_ready and frame_b_ready:
                # frame_a_ready = False
                # frame_b_ready = False

                # frames_in_memory[n_point][:, :frame_width] = picam2b.capture_array("main")
                # frames_in_memory[n_point][:, frame_width:] = picam2a.capture_array("main")

                # frame_count += 1
                # n_point += 1

                # # 到達緩衝區尾端，等待所有區塊寫完再回0
                # if n_point >= n * b:
                    # # 等待所有鎖全部解鎖
                    # for i in range(b):
                        # while lock_b[i]:
                            # time.sleep(0.01)
                    # n_point = 0

                # b_point = n_point // n

                # # 檢查是否切換區塊
                # if b_point_temp != b_point:
                    # # 等待上一區塊完成寫入
                    # while lock_b[b_point_temp]:
                        # time.sleep(0.001)
                    # save_queue.put(b_point_temp)
                    # b_point_temp = b_point
            # else:
                # time.sleep(0.001)
    # # 結束時送最後一個區塊
    # save_queue.put(b_point_temp)

    # for i in range(b):
        # while lock_b[i]:
            # time.sleep(0.01)

    # # 停止寫入執行緒
    # save_queue.put(None)
    # thread.join()

# except KeyboardInterrupt:
    # print("錄製中斷")

# finally:
    # picam2a.stop()
    # picam2b.stop()
    # video_writer.release()

    # # 將 ramdisk 的影片複製到 SSD
    # shutil.copy(ramdisk_path, ssd_path)
    # os.remove(ramdisk_path)
    # del frames_in_memory
    # print(f"錄製完成，共保存 {frame_count} 幀")
    # print("所有影像資料儲存完成")
    # print("總耗時:", time.time() - start_time)
