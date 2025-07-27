import socket
import numpy as np
import cv2
import os
import math

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

# ========= Look at =========
fov_scale_ =1
angle = np.deg2rad(0)
R_look, _ = cv2.Rodrigues(np.array([0,angle, 0]))

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

# ========= Network & Image Reception Settings =========
frame_width, frame_height = DIM
channels = 3


# TCP 客戶端設定
server_ip = "192.168.200.42"  # <-- 請改為發送端（伺服器）的 IP
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




print("📡 Receiving images and showing undistortion effect. Press Q to quit.")

r=900
cx=K[0,2] 
cy=K[1,2] 

mapx = np.zeros((2*r,2*r),np.float32)
mapy = np.zeros((2*r,2*r),np.float32)

def ts(tx,ty):
    c1=(tx/r)*math.pi*0.5
    c2=((r-ty)/r)*math.pi*0.5
    ttx=r*math.cos(c2)*math.cos(c1)
    tty=r*math.sin(c2)
    return(ttx,tty)
    
# def ts(tx,ty):
    # c1=(ty/r)*math.pi*0.5
    # c2=((r-tx)/r)*math.pi*0.5
    # tty=r*math.cos(c2)*math.cos(c1)
    # ttx=r*math.sin(c2)
    # return(ttx,tty)
    
for i in range(2*r):
    for j in range(2*r):
        t=ts(j,i)
        mapx[i,j] = cx-t[0]
        mapy[i,j] = cy-t[1]

try:
    while True:
        # Receive length of frame data
        length_data = recv_all(sock, 4)
        if not length_data:
            break
        length = int.from_bytes(length_data, byteorder='big')

        # Receive frame data
        frame_data = recv_all(sock, length)
        if not frame_data:
            break

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).copy().reshape((frame_height, frame_width, channels))
        
        rst=cv2.remap(frame_np,mapx,mapy,cv2.INTER_LINEAR)
        
        # 加入水平線幫助比對
        for y in range(0, rst.shape[0],50):
            rst[y,:]=(255, 0, 0)
            # cv2.line(frame_np, (0, y), (frame_np.shape[1], y), (0, 255, 0), 1)
            
        # 加入水平線幫助比對
        for x in range(0, rst.shape[1],50):
            rst[:,x]=(0, 0, 255)
            # cv2.line(frame_np, (0, y), (frame_np.shape[1], y), (0, 255, 0), 1)

        # rst=cv2.remap(frame_np,mapx,mapy,cv2.INTER_LINEAR)
        cv2.imshow("Image", rst)
        


        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

except KeyboardInterrupt:
    print("🔌 Reception interrupted")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("✅ Receiver closed")
