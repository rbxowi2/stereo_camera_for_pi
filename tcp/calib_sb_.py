import socket
import numpy as np
import cv2
import threading

cv2.namedWindow("View", cv2.WINDOW_NORMAL)
# 全域狀態變數
running = True
save_flag = False
exit_flag = False
clear_flag = False
em_flag = False
get_flag = False
calib_flag = False


param_file = "camera_params.npz"

rms = None
K = None
D = None
DIM = None

objpoints = []
imgpoints = []

# ======= 棋盤格參數 =======
CHECKERBOARD = (6, 8)
square_size = 24 # mm

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3), np.float64)
objp[:, 0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# ======= 網路與影像參數 =======
frame_width, frame_height = 1296, 972
#frame_width, frame_height = 2596, 1944
DIM = (frame_width,frame_height)
channels = 3

# TCP 客戶端設定
server_ip = "192.168.200.48"  # <-- 請改為發送端（伺服器）的 IP
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

def calibrate_and_return_params(objpoints, imgpoints, img_size):
    global criteria
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
             cv2.fisheye.CALIB_CHECK_COND +
             cv2.fisheye.CALIB_FIX_SKEW)
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_size, K, D, None, None, flags, criteria)
    return K, D, rms

def calb_():
    global K, D, rms ,objpoints,imgpoints,DIM
    try:
        print("tt:")
        print(len(imgpoints))
        print(len(objpoints))
        K, D, rms = calibrate_and_return_params(objpoints, imgpoints, DIM)
        print(f"Updated best calibration RMS: {rms:.4f}")
    except cv2.error as e:
        print("Calibration failed:", e)
        rms = None
        
def key_listener():
    global running, save_flag, exit_flag,clear_flag,get_flag,calib_flag,em_flag
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
        elif key.lower() == 'g':
            get_flag = True
        elif key.lower() == 'p':
            calib_flag = True
        elif key.lower() == 'e':
            em_flag = True

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

        frame_np = np.frombuffer(frame_data, dtype=np.uint8).reshape((frame_height, frame_width, channels))
        gray = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, None)
        
        if ret and (len(corners) == CHECKERBOARD[0] * CHECKERBOARD[1]) :
            if get_flag:
                get_flag = False
                objpoints.append(objp)
                imgpoints.append(corners.astype(np.float64))
                #imgpoints.append(corners.reshape(-1,1,2).astype(np.float64))


                #DIM = (gray.shape[1], gray.shape[0])
                for i in range(len(objpoints)):
                    if objpoints[i].shape != (CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3):
                        print(f"❌ objpoints[{i}] shape 錯誤: {objpoints[i].shape}")
                    if imgpoints[i].shape != (CHECKERBOARD[0]*CHECKERBOARD[1], 1, 2):
                        print(f"❌ imgpoints[{i}] shape 錯誤: {imgpoints[i].shape}")

                calb_()
                # print("tt:")
                # print(len(imgpoints))
            
            # if calib_flag:
                # calib_flag = False
                # calb_()
                # try:
                    # K, D, rms = calibrate_and_return_params(objpoints, imgpoints, DIM)
                    # print("tt:")
                    # print(len(imgpoints))
                    # print(f"Updated best calibration RMS: {rms:.4f}")
                # except cv2.error as e:
                    # print("Calibration failed:", e)
                    # rms = None
                
            cv2.drawChessboardCorners(gray, CHECKERBOARD, corners, ret)
        
        if rms is not None:
            # 使用最佳參數做校正
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K,D, np.eye(3), K,DIM, cv2.CV_16SC2)
            undistorted = cv2.remap(gray, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            if ret:
                # 校正後畫面標出角點（校正空間角點要轉回像素座標）
                undistort_pts = cv2.fisheye.undistortPoints(corners,K, D, P=K)
                for pt in undistort_pts:
                    x, y = pt[0]
                    cv2.circle(undistorted, (int(x), int(y)), 5, (0, 255, 0), -1)
                # 顯示 RMS
            #cv2.putText(undistorted, f"RMS: {rms:.4f}", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 128), 2)
            cv2.imshow("View", undistorted)
        else:
            cv2.imshow("View", gray)
            
        key = cv2.waitKey(1) & 0xFF
        
        if calib_flag:
            if len(objpoints)>0:
                calb_()
            calib_flag = False
            
        if clear_flag:
            if len(objpoints)>0:
                rms = None
                objpoints.pop()
                imgpoints.pop()
                calb_()
            clear_flag = False
            
        if em_flag:
            objpoints = []
            imgpoints = []
            best_rms = None
            K = None
            D = None
            rms = None
            em_flag = False

        if save_flag:
            if K is not None and D is not None:
                np.savez(param_file,K,D,DIM)
                print(f"Saved camera parameters to {param_file}")
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
