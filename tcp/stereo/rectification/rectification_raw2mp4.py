import cv2
import numpy as np
import socket
import os
import math
import glob
import time
import sys

# ====== 建立視窗 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)

# ====== 影片參數 ======
frame_rate = 10
output_video_path = "rectified_output.mp4"
video_codec = 'mp4v'  # 原本設定
video_quality = 100    # 壓縮品質（0~100，95 為高畫質）

# ====== 自訂 rectification rotation ======
roll = np.deg2rad(0)
yaw = np.deg2rad(0)
pitch = np.deg2rad(0)

fov_scale_ = 0.2
# crop_ = True

# ====== 載入 RAW 檔案列表 ======
raw_files = sorted(glob.glob("frame_*.raw"))
total_frames = len(raw_files)
if total_frames == 0:
    print("❌ 未找到任何 RAW 檔案")
    sys.exit(1)
    
# ====== 載入參數 ======
stereo_param_file = "stereo_camera_params.npz"
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

R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T, flags=cv2.CALIB_ZERO_DISPARITY,newImageSize=DIM2,balance=0.0, fov_scale=fov_scale_
)

#rectification rotation
R_look, _ = cv2.Rodrigues(np.array([roll,yaw, pitch]))
R1 = R_look @ R1
R2 = R_look @ R2

#主點偏移校正
P1[0,2]=DIM2[0]/2
P1[1,2]=DIM2[1]/2
P2[0,2]=DIM2[0]/2
P2[1,2]=DIM2[1]/2

# Q[0, 3] = -K_l[0, 2]  # 新的主點位置
# Q[1, 3] = -K_l[1, 2]

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM2, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM2, cv2.CV_16SC2)

# # ====== 主點偏移校正 保留黑邊======
# def shift_to_original_principal_point(image, K, P, orig_size, new_size):
    # cx_orig = K[0, 2]
    # cy_orig = K[1, 2]
    # cx_rect = P[0, 2]
    # cy_rect = P[1, 2]

    # sx = new_size[0] / orig_size[0]
    # sy = new_size[1] / orig_size[1]

    # dx = (cx_orig - (cx_rect / sx)) * sx
    # dy = (cy_orig - (cy_rect / sy)) * sy

    # M = np.float32([
        # [1, 0, dx],
        # [0, 1, dy]
    # ])
    
    # # Q[0, 3] = -K_l[0, 2]  # 新的主點位置
    # # Q[1, 3] = -K_l[1, 2]

    # return cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))

# # ====== 主點偏移校正  合成新大小 不保留黑邊======
# def shift_to_original_principal_point_and_crop(image, K, P, orig_size, new_size):
    # sx = new_size[0] / orig_size[0]
    # sy = new_size[1] / orig_size[1]
    
    # cx_orig = K[0, 2] * sx
    # cy_orig = K[1, 2] * sy
    
    # cx_rect = P[0, 2]
    # cy_rect = P[1, 2]
    
    
    # dx = int((cx_orig - (cx_rect )))
    # dy = int((cy_orig - (cy_rect )))
    
    # dx_l = int(cx_rect)
    # dx_r = int(new_size[0] - cx_rect)
    # dy_u = int(cy_rect)
    # dy_d = int(new_size[1] - cy_rect)
    
    # crop_w = min(dx_l, dx_r)
    # crop_h = min(dy_u, dy_d)
    
   
    # # 左右影像直接裁切
    # left_img = image[
        # int(cy_rect - crop_h):int(cy_rect + crop_h),
        # int(cx_rect - crop_w):int(cx_rect + crop_w)
    # ]
    # right_img = image[
        # int(cy_rect - crop_h):int(cy_rect + crop_h),
        # int(cx_rect - crop_w + new_size[0]):int(cx_rect + crop_w + new_size[0])
    # ]

    # # 組合後畫面
    # combined = np.hstack((left_img, right_img))
    # return combined

init_VideoWriter = False

try:
    for idx, raw_file in enumerate(raw_files):
        with open(raw_file, "rb") as f:
            raw_data = f.read()
        frame_np = np.frombuffer(raw_data, dtype=np.uint8).copy().reshape((frame_height, frame_width*2, channels))
        
        
        # cv2.line(frame_np, (int(frame_np.shape[1]/4), 0), (int(frame_np.shape[1]/4), frame_np.shape[0]), (255, 0, 0), 3)
        # cv2.line(frame_np, (int(frame_np.shape[1]*(3/4)),0),(int(frame_np.shape[1]*(3/4)), frame_np.shape[0]), (255, 0, 0), 3)
        
        # cv2.line(frame_np, (0, int(frame_np.shape[0]/2)), (frame_np.shape[1],int(frame_np.shape[0]/2)), (255, 0, 0), 3)
        
        
        # 分離左右影像
        frame_l = frame_np[:, :frame_width]
        frame_r = frame_np[:, frame_width:]

        # 立體校正
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)
        
        # 合併顯示
        combined = np.hstack((rect_l, rect_r))
        
        # #主點偏移校正
        # if crop_ :
            # combined=shift_to_original_principal_point_and_crop(combined,K_l,P1,DIM,DIM2)
        # else :
            # combined=shift_to_original_principal_point(combined,K_l,P1,DIM,DIM2)
            
            
        DIM3=(int(combined.shape[1]/2),combined.shape[0])
        
        #加入水平線幫助比對
        for y in range(0, combined.shape[0], 50):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 128, 0), 1)
            
        #加入十字線幫助比對
        cv2.line(combined, (int(combined.shape[1]/4), 0), (int(combined.shape[1]/4), combined.shape[0]), (0, 0, 255), 3)
        cv2.line(combined, (int(combined.shape[1]*(3/4)),0),(int(combined.shape[1]*(3/4)), combined.shape[0]), (0, 0, 255), 3)
        
        cv2.line(combined, (0, int(combined.shape[0]/2)), (combined.shape[1],int(combined.shape[0]/2)), (0, 0, 255), 3)
        cv2.line(combined, (int(combined.shape[1]/2), 0), (int(combined.shape[1]/2), combined.shape[0]), (0, 255, 255), 3)
        
        
        cv2.imshow("Rectified", combined)
        
        if init_VideoWriter == False :
            init_VideoWriter = True
            fourcc = cv2.VideoWriter_fourcc(*video_codec)
            video_out = cv2.VideoWriter(output_video_path, fourcc, frame_rate,  (combined.shape[1],combined.shape[0]))
        else :
            video_out.write(combined)
        
        
        # 顯示進度條
        progress = (idx + 1) / total_frames * 100
        print(f"📊 處理中: [{idx+1}/{total_frames}] {progress:.2f}%", end='\r')
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("使用者中斷")

finally:
    if 'video_out' in locals() and video_out.isOpened():
        video_out.release()
    cv2.destroyAllWindows()
    print("程式結束")
