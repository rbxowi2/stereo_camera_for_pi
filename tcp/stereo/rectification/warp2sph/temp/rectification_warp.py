import cv2
import numpy as np
import socket
import os
import math

# ====== 參數與網路設定 ======
stereo_param_file = "stereo_camera_params.npz"
server_ip = "192.168.200.48"
server_port = 5000
#frame_width, frame_height = 1296, 972
#frame_width, frame_height = DIM
channels = 3

# ====== 滑鼠座標處理 ======
mouse_x, mouse_y = -1, -1
def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

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
# DIM2 = (int(4000), int(4000)) 
DIM2 = DIM
frame_width, frame_height = DIM
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


#D_l = np.array([[ 0.11596874], [-0.0757559], [0.02920274], [0.00794466]], dtype=np.float64)
#D_r = np.array([[ 0.11596874], [-0.0757559], [0.02920274], [0.00794466]], dtype=np.float64)
# K_l[0,2]=(K_l[0,2])-324
# K_r[0,2]=(K_r[0,2])-324
# print("🔧 K_l = \n", K_l)

# angle = np.deg2rad(-45)
# R_look, _ = cv2.Rodrigues(np.array([angle, 0, 0]))
# R_new = R_look @ R


R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY,newImageSize=DIM2,balance=1.0, fov_scale=1.0
)
print("🔧 R1 = \n", R1)
print("🔧 R2 = \n", R2)
print("🔧 P1 = \n", P1)
print("🔧 P2 = \n", P2)
print("🔧 Q = \n", Q)




# angle = np.deg2rad(-45)
# R_look, _ = cv2.Rodrigues(np.array([angle, 0, 0]))

# # 新的 rectification rotation
# R1_new = R_look @ R1
# R2_new = R_look @ R2

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM2, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM2, cv2.CV_16SC2)

# ====== warp ======
# def get_rotation_matrix(rx_deg, ry_deg, rz_deg):
    # # 將角度轉成弧度
    # rx = np.deg2rad(rx_deg)
    # ry = np.deg2rad(ry_deg)
    # rz = np.deg2rad(rz_deg)

    # # 分別建立繞 x, y, z 軸的旋轉矩陣
    # Rx = np.array([
        # [1, 0, 0],
        # [0, np.cos(rx), -np.sin(rx)],
        # [0, np.sin(rx),  np.cos(rx)]
    # ])

    # Ry = np.array([
        # [ np.cos(ry), 0, np.sin(ry)],
        # [ 0, 1, 0],
        # [-np.sin(ry), 0, np.cos(ry)]
    # ])

    # Rz = np.array([
        # [np.cos(rz), -np.sin(rz), 0],
        # [np.sin(rz),  np.cos(rz), 0],
        # [0, 0, 1]
    # ])

    # # 最終旋轉矩陣（按 ZYX 順序）
    # R = Rz @ Ry @ Rx
    # return R


fov_ =60

f = K_l[0,0]
cx = K_l[0,2]
cy = K_l[1,2]

def get_warp_m():
    global mouse_x, mouse_y,fov_,f,cx,cy
    
    dr =(1/math.tan(math.radians(fov_/2)))
    dtt=math.atan(1/math.sqrt((1+math.pow(dr,2))))
    
    
    dx=cx-mouse_x

    ax0=math.atan((dx/f))
    
    print(dx,ax0*(180/3.14159))

    dx1 =cx-( f * math.tan(ax0-math.radians(fov_/2)))
    dx0 =cx-( f * math.tan(ax0+math.radians(fov_/2)))
    
    dx1_=(f/math.cos(ax0-math.radians(fov_/2)))
    dx0_=(f/math.cos(ax0+math.radians(fov_/2)))
    
    # dy=cy-mouse_y
    # ay0=math.atan((dy/f))

    dy0 =cy-( dx0_ * math.tan(dtt))
    dy1 =cy-( dx1_ * math.tan(dtt))
    dy2 =cy-( dx0_ * math.tan(-dtt))
    dy3 =cy-( dx1_ * math.tan(-dtt))
   
    # print(dx0_,dx1_,dy0,dy1)

    
    src_pts = np.array([[dx0,dy0],[dx1,dy1],[dx0,dy2],[dx1,dy3]],dtype=np.float32)
    dst_pts = np.array([[0,0],[frame_width,0],[0,frame_height],[frame_width,frame_height]],dtype=np.float32)
    
    m = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return m

# ====== 建立視窗 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Rectified", on_mouse)
cv2.namedWindow("223", cv2.WINDOW_NORMAL)
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

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width*2, channels))
        
        # 分離左右影像
        frame_l = frame_np[:, :frame_width]
        frame_r = frame_np[:, frame_width:]

        # 立體校正
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)
        
        m =get_warp_m()
        result = cv2.warpPerspective(rect_l,m,(frame_height,frame_width))

        cv2.imshow("223", result)
        
        # 合併顯示
        combined = np.hstack((rect_l, rect_r))

        # 加入水平線幫助比對
        for y in range(0, combined.shape[0], 50):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 255, 0), 1)

        cv2.imshow("Rectified", combined)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    sock.close()
    cv2.destroyAllWindows()
    print("程式結束")
