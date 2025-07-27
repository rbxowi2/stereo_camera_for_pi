from picamera2 import Picamera2
import time
import socket
import numpy as np

# 影像參數
resolution = (1296, 972)
frame_width, frame_height = resolution
channels = 3

# 設定幀率
desired_fps = 30

# TCP 伺服器設定
host = "0.0.0.0"
port = 5000

# 幀完成旗標
frame_ready = False

def post_callback(request):
    global frame_ready
    frame_ready = True

# 初始化相機
picam2 = Picamera2(1)
camera_config = picam2.create_video_configuration(main={"size": resolution, "format": "RGB888"})
picam2.configure(camera_config)
picam2.set_controls({"FrameDurationLimits": (int(1e6 / desired_fps), int(1e6 / desired_fps))})
picam2.post_callback = post_callback
picam2.start()
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
                if frame_ready:
                    frame_ready = False
                    frame = picam2.capture_array("main")
                    data = frame.tobytes()
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
