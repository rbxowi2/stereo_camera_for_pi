import cv2
import numpy as np
import socket
import os
import math
# ====== 建立視窗 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)
cv2.namedWindow("combined_s",cv2.WINDOW_NORMAL)

w_out_ = 3000  # 對應水平解析度360°  垂直解析度=水平解析度/2  <<注意>>是對應水平360度 不是crop的180度
        
#主點偏移修正
shift_=True
# shift_=False

#主點偏移修正 -黑邊剪裁
# crop_ = True
# crop_ = False

# ====== 參數與網路設定 ======
stereo_param_file = "stereo_camera_params.npz"
server_ip = "192.168.200.42"
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
DIM2 = (int(w_out_*1.5),int(w_out_*1.5)) #中介解析度
frame_width, frame_height = DIM
channels = 3

# print(data)
# print("✅ Stereo parameters loaded.")
# print("✅ Camera_l intrinsics loaded")
# print("🔧 K_l = \n", K_l)
# print("🔧 D_l = \n", D_l)
# print("✅ Camera_r intrinsics loaded")
# print("🔧 K_r = \n", K_r)
# print("🔧 D_r = \n", D_r)
# print("🔧 R = \n", R)
# print("🔧 T = \n", T)
# print("📐 Image resolution (DIM):", DIM)


R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY,newImageSize=DIM2,balance=0.0, fov_scale=1
)

# print("🔧 R1 = \n", R1)
# print("🔧 R2 = \n", R2)
# print("🔧 P1 = \n", P1)
# print("🔧 P2 = \n", P2)
# print("🔧 Q = \n", Q)

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

map_ = False
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
        
        DIM3=(int(combined.shape[1]/2),combined.shape[0])

        #加入水平線幫助比對
        for y in range(0, combined.shape[0], 50):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 128, 0), 1)
            
            
        #加入十字線幫助比對
        cv2.line(combined, (int(combined.shape[1]/4), 0), (int(combined.shape[1]/4), combined.shape[0]), (0, 0, 255), 3)
        cv2.line(combined, (int(combined.shape[1]*(3/4)),0),(int(combined.shape[1]*(3/4)), combined.shape[0]), (0, 0, 255), 3)
        
        cv2.line(combined, (0, int(combined.shape[0]/2)), (combined.shape[1],int(combined.shape[0]/2)), (0, 0, 255), 3)
        
        cv2.line(combined, (int(combined.shape[1]/2), 0), (int(combined.shape[1]/2), combined.shape[0]), (0, 255, 255), 3)
        
        
        
        #等距投影變換
        l_image =  combined[:,:DIM3[0]]
        r_image =  combined[:, DIM3[0]:]
        
        if map_ == False:
            map_ = True
            print(P1)
            print(K_l)
            K_ = np.array([
                [P1[0,0], 0, DIM3[0]/2],
                [0, P1[1,1],DIM3[1]/2],
                [0,  0,  1]
            ], dtype=np.float32)
            # K_ = K_l.astype(np.float32)
                        
            h_in, w_in =l_image.shape[:2]
            
            # === 目標 equirectangular 影像大小（可依需求調整） ===
            w_out = w_out_  # 對應水平360°
            h_out = int(w_out/2)   # 對應垂直180°
            output_img = np.zeros((h_out, w_out, 3), dtype=np.uint8)
            
            # === 為每個像素計算其對應的經緯度（lon, lat） ===
            lon = np.linspace(0, 2*np.pi, w_out)        # 方位角 [-180°, 180°]
            lat = np.linspace(np.pi / 2, -np.pi / 2, h_out)  # 仰角 [90°, -90°]
            lon_map, lat_map = np.meshgrid(lon, lat)
            
            # === 將經緯度轉為 3D 單位方向向量 ===
            x = np.cos(lat_map) * np.sin(lon_map)
            y = np.sin(lat_map)
            z = np.cos(lat_map) * np.cos(lon_map)
            
            # 組成 ray 向量並套用相機投影矩陣 K
            rays = np.stack([x, y, z], axis=-1)  # shape: (h_out, w_out, 3)

            proj = rays @ K_.T  # (x, y, z) → 相機像平面
            
            # perspective divide：將 3D 點轉成影像上的 2D 像素座標
            z_safe = np.where(proj[..., 2] == 0, 1e-6, proj[..., 2])
            u = proj[..., 0] / proj[..., 2]
            v = proj[..., 1] / proj[..., 2]
                    
            # 建立 valid mask：只保留落在原圖像範圍內的像素
            valid = (u >= 0) & (u < w_in) & (v >= 0) & (v < h_in)

            # 建立 remap 所需的 map_x, map_y
            map_x = np.where(valid, u, -1).astype(np.float32)
            map_y = np.where(valid, v, -1).astype(np.float32)

        res_l= cv2.remap(
            l_image,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)  # 填黑表示超出原圖範圍
        )
        
        res_r= cv2.remap(
            r_image,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)  # 填黑表示超出原圖範圍
        )
        
        combined_s = np.vstack((res_l, res_r))
        
        cv2.imshow("Rectified", combined)
        
        cv2.imshow("combined_s", combined_s)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("程式結束")
