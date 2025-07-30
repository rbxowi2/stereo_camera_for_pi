import socket
import numpy as np
import cv2
import  sys

cv2.namedWindow("recv", cv2.WINDOW_NORMAL)

# 影像參數
if sys.argv[1] == '1':
    frame_width, frame_height = 1296, 972
if sys.argv[1] == '2':
    frame_width, frame_height = 2592, 1944
if sys.argv[1] == '3':
    frame_width, frame_height = 1944, 1944
    
channels = 3
frame_size = frame_width * frame_height * channels

# 可選旋轉角度：0, 90, 180, 270
rotate_angle = 0

# TCP 客戶端設定
server_ip = "192.168.200.48" # <-- 請改為發送端（伺服器）的 IP
server_port = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))
print("已連線到發送端")

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

try:
    while True:
        length_data = recv_all(sock, 4)
        if not length_data:
            break
        length = int.from_bytes(length_data, byteorder='big')
        frame_data = recv_all(sock, length)
        if not frame_data:
            break

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width*2, channels))
        rotated_frame = rotate_frame(frame_np, rotate_angle)

        cv2.imshow("recv", rotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("接收中斷")
finally:
    sock.close()
    cv2.destroyAllWindows()
    print("已關閉接收端")
