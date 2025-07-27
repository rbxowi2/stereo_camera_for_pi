import cv2
import numpy as np
import socket
import os
# ====== 建立視窗 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)

# ====== 自訂 rectification rotation ======
roll = np.deg2rad(0)
yaw = np.deg2rad(0)
pitch = np.deg2rad(0)

fov_scale_ = 0.12

#主點偏移修正
shift_=True
# shift_=False


# ====== 參數與網路設定 ======
stereo_param_file = "stereo_camera_params.npz"
server_ip = "192.168.200.39"
server_port = 5000

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
DIM = (int(DIM[0]), int(DIM[1])) 
DIM2 = DIM
frame_width, frame_height = DIM
channels = 3

print(data)
print("✅ Stereo parameters loaded.")
print("✅ Camera_l intrinsics loaded")
print("🔧 K_l = \n", K_l)
print("🔧 D_l = \n", D_l)
print("✅ Camera_r intrinsics loaded")
print("🔧 K_r = \n", K_r)
print("🔧 D_r = \n", D_r)
print("🔧 R = \n", R)
print("🔧 T = \n", T)
print("📐 Image resolution (DIM):", DIM)


R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY,newImageSize=DIM2,balance=0.0, fov_scale=fov_scale_
)

print("🔧 R1 = \n", R1)
print("🔧 R2 = \n", R2)
print("🔧 P1 = \n", P1)
print("🔧 P2 = \n", P2)
print("🔧 Q = \n", Q)


#rectification rotation
R_look, _ = cv2.Rodrigues(np.array([roll,yaw, pitch]))
R1 = R_look @ R1
R2 = R_look @ R2

if shift_:
    #主點偏移校正
    P1[0,2]=DIM2[0]/2
    P1[1,2]=DIM2[1]/2
    P2[0,2]=DIM2[0]/2
    P2[1,2]=DIM2[1]/2

    # Q[0, 3] = -DIM2[0]/2  # 新的主點位置
    # Q[1, 3] = -DIM2[1]/2

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM2, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM2, cv2.CV_16SC2)

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
print("已連線到影像伺服器")
print("藍線-校正前  紅線-校正後 ")
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

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).copy().reshape((frame_height, frame_width*2, channels))
        
        #加入十字線幫助比對
        cv2.line(frame_np, (int(frame_np.shape[1]/4), 0), (int(frame_np.shape[1]/4), frame_np.shape[0]), (255, 0, 0), 3)
        cv2.line(frame_np, (int(frame_np.shape[1]*(3/4)),0),(int(frame_np.shape[1]*(3/4)), frame_np.shape[0]), (255, 0, 0), 3)
        
        cv2.line(frame_np, (0, int(frame_np.shape[0]/2)), (frame_np.shape[1],int(frame_np.shape[0]/2)), (255, 0, 0), 3)
        
        
        # 分離左右影像
        frame_l = frame_np[:, :frame_width]
        frame_r = frame_np[:, frame_width:]

        # 立體校正
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        # 合併顯示
        combined = np.hstack((rect_l, rect_r))
            
        
        #加入水平線幫助比對
        for y in range(0, combined.shape[0], 50):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 128, 0), 1)
            
        #加入十字線幫助比對
        cv2.line(combined, (int(combined.shape[1]/4), 0), (int(combined.shape[1]/4), combined.shape[0]), (0, 0, 255), 3)
        cv2.line(combined, (int(combined.shape[1]*(3/4)),0),(int(combined.shape[1]*(3/4)), combined.shape[0]), (0, 0, 255), 3)
        
        cv2.line(combined, (0, int(combined.shape[0]/2)), (combined.shape[1],int(combined.shape[0]/2)), (0, 0, 255), 3)
        
        cv2.line(combined, (int(combined.shape[1]/2), 0), (int(combined.shape[1]/2), combined.shape[0]), (0, 255, 255), 3)
        
        
        cv2.imshow("Rectified", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("程式結束")
