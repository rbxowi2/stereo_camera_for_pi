import socket
import numpy as np
import cv2
import threading
import os
import time
cv2.namedWindow("view", cv2.WINDOW_NORMAL)
# 全域狀態變數
running = True
save_flag = False
exit_flag = False
clear_flag = False

param_file = "camera_params.npz"
stereo_param_file= "stereo_camera_params.npz"

best_K_l = None
best_D_l = None

best_K_r = None
best_D_r = None

best_rms_stereo = None

best_R = None
best_T = None

DIM = None


objpoints = []
imgpoints = []
imgpoints_l = []
imgpoints_r = []
# ========= Load Camera Intrinsics =========
param_l_file = "camera_params_l.npz"
param_r_file = "camera_params_r.npz"
if not os.path.exists(param_l_file):
    print(f"❌ Camera parameter file not found: {param_l_file}")
    exit(1)
if not os.path.exists(param_r_file):
    print(f"❌ Camera parameter file not found: {param_r_file}")
    exit(1)

data = np.load(param_l_file)
best_K_l = data["K"]
best_D_l = data["D"]
DIM = tuple(data["DIM"])

print("✅ Camera_l intrinsics loaded")
print("🔧 K_l = \n", best_K_l)
print("🔧 D_l = \n", best_D_l)
print("📐 Image resolution (DIM):", DIM)

data = np.load(param_r_file)
best_K_r = data["K"]
best_D_r = data["D"]
DIM = tuple(data["DIM"])

print("✅ Camera_r intrinsics loaded")
print("🔧 K_r = \n", best_K_r)
print("🔧 D_r = \n", best_D_r)
print("📐 Image resolution (DIM):", DIM)

# ======= 棋盤格參數 =======
CHECKERBOARD = (8, 14)
square_size = 25 # mm

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3), np.float64)
objp[:, 0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# ======= 網路與影像參數 =======
frame_width, frame_height = DIM
#frame_width, frame_height = 2596, 1944
channels = 3
#DIM = (frame_height,frame_width)


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

    
def stereo_calibrate_and_return_params():
    global objpoints,imgpoints_l,imgpoints_r,best_K_l,best_D_l,best_K_r,best_D_r,DIM
    flags = cv2.fisheye.CALIB_FIX_INTRINSIC
    R = np.eye(3)
    T = np.zeros((3, 1))
    rms_stereo,K1,D1,K2,D2,R,T = cv2.fisheye.stereoCalibrate(
        objpoints, imgpoints_l, imgpoints_r,
        best_K_l, best_D_l, best_K_r, best_D_r, DIM,
        R, T,
        flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 1e-6))
    
    return R, T,rms_stereo


def key_listener():
    global running, save_flag, exit_flag,clear_flag
    print("Press 's' to save calibration, 'q' to quit.")
    while running:
        key = input()
        if key.lower() == 'q':
            exit_flag = True
            running = False
            print("Exit")
            break
        elif key.lower() == 's':
            save_flag = True
            print("Save requested")
        elif key.lower() == 'c':
            clear_flag = True

thread = threading.Thread(target=key_listener, daemon=True)
thread.start()

try:
    while running:
        
        length_data = recv_all(sock, 4)
        
        if not length_data:
            print("No length data received, exiting")
            break
        
        length = int.from_bytes(length_data, byteorder='big')
        frame_data = recv_all(sock, length)
        
        if not frame_data:
            print("No complete frame data, exiting")
            break
        
        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width*2, channels))
        
        gray=cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
        
        # ret_l, corners_l = cv2.findChessboardCorners(gray[:, :frame_width], CHECKERBOARD, None)
        ret_l, corners2_l = cv2.findChessboardCornersSB(gray[:, :frame_width], CHECKERBOARD, None)
        
        if ret_l:
            # ret_r, corners_r = cv2.findChessboardCorners(gray[:, frame_width:], CHECKERBOARD, None)
            ret_r, corners2_r = cv2.findChessboardCornersSB(gray[:, frame_width:], CHECKERBOARD, None)
            
        if ret_l and ret_r:
            # corners2_l = cv2.cornerSubPix(gray[:, :frame_width], corners_l, (11, 11), (-1, -1), criteria)
            # corners2_r = cv2.cornerSubPix(gray[:, frame_width:], corners_r, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            
            imgpoints_l.append(corners2_l.astype(np.float64))
            imgpoints_r.append(corners2_r.astype(np.float64))

            try:
                R = np.eye(3)
                T = np.zeros((3, 1))
                R,T,rms_stereo=stereo_calibrate_and_return_params()
            except cv2.error as e:
                rms_stereo = None
                objpoints.pop()
                imgpoints_l.pop()
                imgpoints_r.pop()

            if (rms_stereo is not None):
                if (best_rms_stereo is None) or (rms_stereo < best_rms_stereo):
                    best_rms_stereo = rms_stereo
                    best_R=R
                    best_T=T
                    print('Rotation matrix:')
                    print(R)
                    print('Translation:')
                    print(T)
                    print('best_rms_stereo:')
                    print(best_rms_stereo)
                else:
                    objpoints.pop()
                    imgpoints_l.pop()
                    imgpoints_r.pop()

            cv2.drawChessboardCorners(gray[:, :frame_width], CHECKERBOARD, corners2_l, ret_l)
            cv2.drawChessboardCorners(gray[:, frame_width:], CHECKERBOARD, corners2_r, ret_r)
        cv2.imshow("view", gray)
        key = cv2.waitKey(1) & 0xFF
        #time.sleep(0.01)

        if clear_flag:
            objpoints = []
            imgpoints_l = []
            imgpoints_r = []
            best_rms_stereo = None
            clear_flag = False
            
        if save_flag:
            if best_rms_stereo is not None :
                np.savez(stereo_param_file, K_l=best_K_l, D_l=best_D_l,K_r=best_K_r, D_r=best_D_r,R=best_R,T=best_T, DIM=DIM)
                print(f"Saved camera parameters to {stereo_param_file}")
            else:
                print("No calibration data to save yet.")
            save_flag = False

        if exit_flag:
            break

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    running = False
    sock.close()
    cv2.destroyAllWindows()
    print("Program terminated")
