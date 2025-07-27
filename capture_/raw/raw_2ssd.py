from picamera2 import Picamera2
import time
import numpy as np

picam2 = Picamera2()

# 設定解析度與格式
resolution = (1296, 972)  # (寬, 高)
frame_width, frame_height = resolution
channels = 3

# 建立相機配置
camera_config = picam2.create_video_configuration(
    main={"size": resolution, "format": "RGB888"}
)
picam2.configure(camera_config)

# 設定幀率
desired_fps = 30
picam2.set_controls({
    "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
})

# 設定錄製時間
duration = 3  # 秒
max_frames = duration * desired_fps

# 預先分配記憶體 (frame_count, 高, 寬, 通道)
frames_in_memory = np.empty((max_frames, frame_height, frame_width, channels), dtype=np.uint8)


# 旗標變數，標示有新幀完成
frame_ready = False

def post_callback(request):
    global frame_ready
    frame_ready = True  # 幀完成，標記為 True

picam2.post_callback = post_callback

#time.sleep(3)

# 啟動相機
picam2.start()
print("開始錄製 RAW 格式影片...")

time.sleep(1)

start_time = time.time()
frame_count = 0

try:
    while time.time() - start_time < duration:
        if frame_count >= max_frames:
            break  # 避免越界
        # 擷取單幀
        if frame_ready:
            frame_ready = False  # 重設旗標
            #frame = picam2.capture_array("main")
            # 儲存影像到預分配記憶體中   
            frames_in_memory[frame_count][:] = picam2.capture_array("main")
            frame_count += 1
        else:
            # 沒收到新幀，短暫休息避免 CPU 全忙
            time.sleep(0.01)

except KeyboardInterrupt:
    print("錄製中斷")

# 停止相機
picam2.stop()

# 將影像資料儲存到檔案
print(f"錄製完成，共保存 {frame_count} 幀，開始將影像資料儲存到檔案...")

for idx in range(frame_count):
    raw_filename = f"frame_{idx:04d}.raw"
    frames_in_memory[idx].tofile(raw_filename)
    #print(f"保存 {raw_filename}")

# 清理記憶體（可省略）
del frames_in_memory

print("所有影像資料儲存完成。")
