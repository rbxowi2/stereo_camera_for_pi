import socket
import numpy as np
import cv2
import os

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


new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K,D,DIM,np.eye(3),balance=0.0,fov_scale=fov_scale_)
print("new_k dtype:", new_k.dtype)
print(new_k)

# # Initialize fisheye undistort rectify map
# map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    # K, D, np.eye(3), new_k, DIM, cv2.CV_16SC2
#)

print("R_look dtype:", R_look.dtype)
print(R_look)

map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, R_look, new_k, DIM, cv2.CV_16SC2
)


print("📡 Receiving images and showing undistortion effect. Press Q to quit.")

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

        cv2.line(frame_np, (972, 0), (972, frame_np.shape[0]),  (255, 0, 0), 3)
        cv2.line(frame_np, (0, 972), (frame_np.shape[1],972),  (255, 0, 0), 3)
        
        undistorted = cv2.remap(frame_np, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        cv2.line(undistorted, (972, 0), (972, undistorted.shape[0]), (0, 0, 255), 3)
        cv2.line(undistorted, (0, 972), (undistorted.shape[1],972), (0, 0, 255), 3)
        
        cv2.imshow("Image", undistorted)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

except KeyboardInterrupt:
    print("🔌 Reception interrupted")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("✅ Receiver closed")
