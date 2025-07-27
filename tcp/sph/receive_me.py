import socket
import numpy as np
import cv2
import os
import math

cv2.namedWindow("接收畫面", cv2.WINDOW_NORMAL)

# ========= Load Camera Intrinsics =========
param_file = "camera_params.npz"
if not os.path.exists(param_file):
    print(f"❌ Camera parameter file not found: {param_file}")
    exit(1)

data = np.load(param_file)
# K = data["K"]
# D = data["D"]
K = data["K"].astype(np.float64)
D = data["D"].astype(np.float64)

DIM = tuple(data["DIM"])

print("✅ Camera intrinsics loaded")
print("K dtype:", K.dtype)
print("K:\n", K)
print("D dtype:", D.dtype)
print("D:\n", D)

print("📐 Image resolution (DIM):", DIM)

frame_width, frame_height = DIM
channels = 3
frame_size = frame_width * frame_height * channels

# 可選旋轉角度：0, 90, 180, 270
rotate_angle = 0

# TCP 客戶端設定
server_ip = "192.168.200.42"  # <-- 請改為發送端（伺服器）的 IP
server_port = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))
print("已連線到發送端")

cx=K[0,2]
cy=K[1,2]

def recv_all(sock, size):
    buffer = b""
    while len(buffer) < size:
        data = sock.recv(size - len(buffer))
        if not data:
            return None
        buffer += data
    return buffer

def rotate_frame(frame, angle):
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame


def show_mouse_position(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # print(x,y)

        r= math.sqrt(math.pow(cx-x,2)+math.pow(cy-y,2))
        print(r,x,y,cx,cy)
        # pixel = param[y, x]  # param 是 frame
        # text = f"X: {x}, Y: {y}, BGR: {pixel.tolist()}"
        # # 複製一份影像來顯示文字，避免在原圖上重複畫字
        # display_frame = param.copy()
        # cv2.putText(display_frame, text, (10, 30),
                    # cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # cv2.imshow("接收畫面", display_frame)
        
def draw_crosshair(image, x, y, size=100, color=(0, 255, 0), thickness=10):
    """
    在指定影像上畫十字線。
    - image: 要繪製的影像
    - x, y: 十字中心座標
    - size: 十字線長度的一半
    - color: 十字線顏色 (預設綠色)
    - thickness: 線條粗細
    """
    x=int(x)
    y=int(y)
    # 畫水平線
    cv2.line(image, (x - size, y), (x + size, y), color, thickness)
    # 畫垂直線
    cv2.line(image, (x, y - size), (x, y + size), color, thickness)


try:
    while True:
        length_data = recv_all(sock, 4)
        if not length_data:
            break
        length = int.from_bytes(length_data, byteorder='big')
        frame_data = recv_all(sock, length)
        if not frame_data:
            break

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).copy().reshape((frame_height, frame_width, channels))
        rotated_frame = rotate_frame(frame_np, rotate_angle)
        draw_crosshair(frame_np,cx,cy)
        
        cv2.setMouseCallback("接收畫面", show_mouse_position, rotated_frame)
        cv2.imshow("接收畫面", rotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("接收中斷")
finally:
    sock.close()
    cv2.destroyAllWindows()
    print("已關閉接收端")
