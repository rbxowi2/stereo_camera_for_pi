import cv2
import numpy as np
import socket

# ========= 載入 stereo_camera_params =========

stereo_param_file = "stereo_camera_params.npz"
data = np.load(stereo_param_file)

K_l = data["K_l"]
D_l = data["D_l"]
K_r = data["K_r"]
D_r = data["D_r"]
R = data["R"]
T = data["T"]
DIM = tuple(data["DIM"])

print("✅ Stereo camera parameters loaded.")
print("📐 Resolution:", DIM)

# ========= 建立 rectify map =========
R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    #flags=0,
    newImageSize=DIM,
    balance=0.0,
    fov_scale=1.0
)

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(
    K_l, D_l, R1, P1[:, :3], DIM, cv2.CV_16SC2
)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(
    K_r, D_r, R2, P2[:, :3], DIM, cv2.CV_16SC2
)

# ========= 連接 TCP 視訊串流 =========
server_ip = "192.168.200.42"  # 更換成伺服器 IP
server_port = 5000
frame_width, frame_height = DIM
channels = 3

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))
print("📡 Connected to stream source.")

def recv_all(sock, size):
    buffer = b""
    while len(buffer) < size:
        data = sock.recv(size - len(buffer))
        if not data:
            return None
        buffer += data
    return buffer

cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)

# ========= 主迴圈 =========
try:
    while True:
        # 讀取影像資料長度
        length_data = recv_all(sock, 4)
        if not length_data:
            print("No length data, exiting.")
            break
        length = int.from_bytes(length_data, byteorder='big')

        # 讀取影像資料
        frame_data = recv_all(sock, length)
        if not frame_data:
            print("No frame data, exiting.")
            break

        # 解碼影像
        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width * 2, channels))
        img_l = frame_np[:, :frame_width]
        img_r = frame_np[:, frame_width:]

        # 做校正
        rect_l = cv2.remap(img_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(img_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        # 合併顯示
        combined = np.hstack((rect_l, rect_r))

        # 畫水平對齊線
        for y in range(0, frame_height, 40):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 255, 0), 1)

        cv2.imshow("Rectified", combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("❎ Quit by user.")
            break

except KeyboardInterrupt:
    print("🔌 Interrupted.")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("🛑 Program terminated.")
