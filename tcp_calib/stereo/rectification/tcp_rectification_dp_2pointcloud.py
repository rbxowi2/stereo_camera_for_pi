import cv2
import numpy as np
import socket
import os

# ====== 參數與網路設定 ======
stereo_param_file = "stereo_camera_params.npz"
server_ip = "192.168.200.39"
server_port = 5000
channels = 3

fov_scale_ = 0.2

# ====== 載入參數 ======
if not os.path.exists(stereo_param_file):
    print(f"❌ 立體參數文件未找到: {stereo_param_file}")
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
print("✅ 立體參數載入成功。")

# ====== 計算立體校正映射 ======
R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY, newImageSize=DIM2,balance=0.0, fov_scale=fov_scale_
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

# ====== 視差計算器設定（SGBM） ======
min_disp = 0
num_disp = 32   # Adjusted for better range, must be divisible by 16
block_size = 3

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

# ====== TCP 接收函式 ======
def recv_all(sock, size):
    buffer = b""
    while len(buffer) < size:
        data = sock.recv(size - len(buffer))
        if not data:
            return None
        buffer += data
    return buffer

# ====== 滑鼠座標處理 ======
mouse_x, mouse_y = -1, -1
def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

# ====== 儲存點雲函式 ======
def save_point_cloud(filename, points_3d, colors):
    """
    將 3D 點雲和顏色資料儲存為 .ply 文件。
    """
    # 過濾掉無效的點 (Z 值為 0 或 NaN/inf)
    valid_mask = ~np.isinf(points_3d).any(axis=2) & ~np.isnan(points_3d).any(axis=2) & (points_3d[:,:,2] != 0)
    
    valid_points = points_3d[valid_mask]
    valid_colors = colors[valid_mask]

    # 將顏色從 BGR 轉換為 RGB (如果需要)
    # valid_colors = valid_colors[:, [2, 1, 0]] 

    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(valid_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for i in range(len(valid_points)):
            f.write(f"{valid_points[i, 0]:.4f} {valid_points[i, 1]:.4f} {valid_points[i, 2]:.4f} "
                    f"{int(valid_colors[i, 2])} {int(valid_colors[i, 1])} {int(valid_colors[i, 0])}\n") # 注意顏色順序

    print(f"✅ 點雲已儲存至 {filename}")


# ====== 開啟視窗與連線 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Rectified", on_mouse)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.connect((server_ip, server_port))
    print("📡 已連線到影像伺服器")
except socket.error as e:
    print(f"❌ 無法連線到伺服器: {e}")
    exit(1)


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

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width * 2, channels))
        frame_l = frame_np[:, :frame_width]
        frame_r = frame_np[:, frame_width:]

        # 校正
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        # 灰階與視差
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

        # 建立 3D 點雲
        points_3d = cv2.reprojectImageTo3D(disparity, Q)


        # 顯示視差圖
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        combined = np.hstack((rect_l, disp_color))

        # === 繪製游標位置座標與距離 ===
        if 0 <= mouse_x < disparity.shape[1] and 0 <= mouse_y < disparity.shape[0]:
            d = disparity[mouse_y, mouse_x]
            if d > 0:
                X, Y, Z = points_3d[mouse_y, mouse_x]
                distance = np.sqrt(X**2 + Y**2 + Z**2)
                # 顯示單位為 mm
                text = f"X:{X:.2f}mm Y:{Y:.2f}mm Z:{Z:.2f}mm D:{distance:.2f}mm"
                cv2.putText(combined, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)
                cv2.circle(combined, (mouse_x, mouse_y), 5, (0, 255, 255), -1)

        #加入十字線幫助比對
        cv2.line(combined, (int(combined.shape[1]/4), 0), (int(combined.shape[1]/4), combined.shape[0]), (0, 0, 255), 1)
        cv2.line(combined, (int(combined.shape[1]*(3/4)),0),(int(combined.shape[1]*(3/4)), combined.shape[0]), (0, 0, 255), 1)
        
        cv2.line(combined, (0, int(combined.shape[0]/2)), (combined.shape[1],int(combined.shape[0]/2)), (0, 0, 255), 1)
        
        cv2.line(combined, (int(combined.shape[1]/2), 0), (int(combined.shape[1]/2), combined.shape[0]), (0, 255, 255), 1)
        
        # 顯示畫面
        cv2.imshow("Rectified", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 儲存點雲檔案
            timestamp = int(cv2.getTickCount() / cv2.getTickFrequency())
            filename = f"point_cloud_{timestamp}.ply"
            save_point_cloud(filename, points_3d, rect_l) # 使用左側校正影像的顏色

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("✅ 程式結束")