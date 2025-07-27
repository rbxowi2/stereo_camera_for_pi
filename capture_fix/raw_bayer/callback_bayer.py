from picamera2 import Picamera2
import time
import numpy as np

picam2 = Picamera2()

resolution = (1296, 972)
frame_width, frame_height = resolution

# 配置 raw stream 而非 main
camera_config = picam2.create_video_configuration(
    raw={"size": resolution, "format": "SBGGR10"}  # 放在 raw stream
)
picam2.configure(camera_config)

# 控制幀率（可選）
desired_fps = 30
picam2.set_controls({
    "FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))
})

duration = 3
max_frames = duration * desired_fps

# 注意這裡的資料是 uint16，因為每像素是 10-bit（儲存為 16-bit）
frames_in_memory = np.empty((max_frames, frame_height, frame_width*2), dtype=np.uint8)

#frame_ready = False

def post_callback(request):
    global frame_ready
    frame_ready = True

#picam2.post_callback = post_callback

picam2.start()
print("開始錄製 SBGGR10 RAW 資料...")

time.sleep(1)

start_time = time.time()
frame_count = 0
frame_ready = True
try:
    while time.time() - start_time < duration:
        if frame_count >= max_frames:
            break
        if frame_ready:
            frame_ready = False
            # 擷取 raw stream 中的 Bayer 圖像
            frame = picam2.capture_array("raw")
            frames_in_memory[frame_count][:] = frame
            frame_count += 1
        else:
            time.sleep(0.005)

except KeyboardInterrupt:
    print("錄製中斷")

picam2.stop()

print(f"錄製完成，共保存 {frame_count} 幀")

# 儲存
for idx in range(frame_count):
    frames_in_memory[idx].tofile(f"frame_{idx:04d}_bayer.raw")

del frames_in_memory
print("影像儲存完成")
