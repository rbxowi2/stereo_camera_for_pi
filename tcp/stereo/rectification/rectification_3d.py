import cv2
import numpy as np
import socket
import os

# ====== 參數與網路設定 ======
stereo_param_file = "stereo_camera_params.npz"
server_ip = "192.168.200.39"
server_port = 5000

channels = 3

# ====== 載入參數 ======
if not os.path.exists(stereo_param_file):
    print(f"❌ Stereo parameter file not found: {stereo_param_file}")
    exit(1)

data = np.load(stereo_param_file)
K_l = data["K_l"]
D_l = data["D_l"]
K_r = data["K_r"]
D_r = data["D_r"]
R = data["R"]
T = data["T"]
DIM = tuple(data["DIM"])
frame_width, frame_height = DIM
DIM2 = (720,720)
print("✅ Stereo parameters loaded.")

# ====== 計算立體校正映射 ======
R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY,newImageSize =DIM2, balance=0.0, fov_scale=0.2
)

#自訂 rectification rotation
angle = np.deg2rad(0)
R_look, _ = cv2.Rodrigues(np.array([0,angle, 0]))
R1 = R_look @ R1
R2 = R_look @ R2

shift_ = 1

if shift_:
    #主點偏移校正
    P1[0,2]=DIM2[0]/2
    P1[1,2]=DIM2[1]/2
    P2[0,2]=DIM2[0]/2
    P2[1,2]=DIM2[1]/2

    Q[0, 3] = -DIM2[0]/2  # 新的主點位置
    Q[1, 3] = -DIM2[1]/2
    
    
map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM2, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM2, cv2.CV_16SC2)

# map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, K_l, DIM2, cv2.CV_16SC2)
# map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, K_r, DIM2, cv2.CV_16SC2)

# ====== 視差計算器設定（SGBM） ======
min_disp = 0
num_disp = 128  # 要是 16 的倍數，越大代表可以觀測更遠的物體，但處理時間會變慢。
block_size = 3 #建議奇數，大一點抗雜訊好，但會模糊邊緣。

stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * 3 * block_size ** 2,
    P2=32 * 3 * block_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

# ====== 顯示視窗 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)

# ====== TCP 接收 ======
def recv_all(sock, size):
    buffer = b""
    while len(buffer) < size:
        data = sock.recv(size - len(buffer))
        if not data:
            return None
        buffer += data
    return buffer

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))
print("📡 已連線到影像伺服器")

try:
    while True:
        length_data = recv_all(sock, 4)
        if not length_data:
            print("連線中斷")
            break
        length = int.from_bytes(length_data, byteorder='big')
        frame_data = recv_all(sock, length)
        if not frame_data:
            print("資料接收不完整")
            break

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).copy().reshape((frame_height, frame_width * 2, channels))

        cv2.line(frame_np, (972, 0), (972, frame_np.shape[0]), (255, 0, 0), 3)
        cv2.line(frame_np, (972+1944, 0), (972+1944, frame_np.shape[0]), (255, 0, 0), 3)
        
        cv2.line(frame_np, (0, 972), (frame_np.shape[1],972), (255, 0, 0), 3)
        cv2.line(frame_np, (0, 972+1944), (frame_np.shape[1],972+1944), (255, 0, 0), 3)
        
        # 分離左右影像
        frame_l = frame_np[:, :frame_width]
        frame_r = frame_np[:, frame_width:]

        # 立體校正
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        cv2.line(rect_l, (int(DIM2[1]/2), 0), (int(DIM2[1]/2), rect_l.shape[0]), (0, 0, 255), 1)
        cv2.line(rect_l, (0, int(DIM2[0]/2)), (rect_l.shape[1],int(DIM2[0]/2)), (0, 0, 255), 1)
        
        # === 視差圖計算 ===
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

        # 視差圖正規化顯示
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        combined = np.hstack((rect_l, disp_color))
        cv2.imshow("Rectified", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("✅ 程式結束")
