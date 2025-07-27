from picamera2 import Picamera2
import time
import socket
import numpy as np

# 影像參數
resolution = (1296, 972)
frame_width, frame_height = resolution
channels = 3

# 預先分配記憶體 ( 高, 寬, 通道)
frames_in_memory = np.empty((frame_height, frame_width*2, channels), dtype=np.uint8)


# 設定幀率
desired_fps = 15

# TCP 伺服器設定
host = "0.0.0.0"
port = 5000

# 旗標變數，標示有新幀完成
frame_a_ready = False
frame_b_ready = False

def post_a_callback(request):
    global frame_a_ready 
    frame_a_ready = True  # 幀完成，標記為 True

def post_b_callback(request):
    global frame_b_ready
    frame_b_ready = True  # 幀完成，標記為 True

# 初始化相機
picam2a = Picamera2(0)
camera_config_a = picam2a.create_video_configuration(main={"size": resolution, "format": "RGB888"})
picam2a.configure(camera_config_a)
picam2a.set_controls({"FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))})
picam2a.post_callback = post_a_callback

picam2b = Picamera2(1)
camera_config = picam2b.create_video_configuration(main={"size": resolution, "format": "RGB888"})
picam2b.configure(camera_config)
picam2b.set_controls({"FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))})
picam2b.post_callback = post_b_callback


picam2a.start()
picam2b.start()
time.sleep(1)



# 建立伺服器 socket
server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind((host, port))
server_sock.listen(1)
print(f"伺服器啟動，等待接收端連線...")

try:
    while True:
        conn, addr = server_sock.accept()
        print(f"接收端已連線：{addr}")

        try:
            while True:
                if frame_b_ready and frame_a_ready:
                    frame_a_ready = False
                    frame_b_ready = False
                    frames_in_memory[:, :resolution[0],:] = picam2b.capture_array("main")
                    frames_in_memory[:, resolution[0]:,:] = picam2a.capture_array("main")
                    data = frames_in_memory.tobytes()
                    length = len(data).to_bytes(4, byteorder='big')
                    conn.sendall(length + data)
                else:
                    time.sleep(0.001)
        except (ConnectionResetError, BrokenPipeError):
            print(f"接收端斷線：{addr}，返回等待狀態...")
            conn.close()
        except Exception as e:
            print(f"未知錯誤：{e}")
            conn.close()
except KeyboardInterrupt:
    print("使用者中斷，關閉伺服器")
finally:
    server_sock.close()
    picam2.stop()
    print("已關閉相機與伺服器")
