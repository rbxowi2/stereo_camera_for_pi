# eye_relief_adaptive_vr180.py
import cv2
import numpy as np
import glob
import os
import sys
import time

# ---------- 設定 ----------
raw_pattern = "frame_*.raw"                # 原始 raw 檔路徑格式（左右水平並列）
stereo_param_file = "stereo_camera_params.npz"
output_video_path = "sph_rectified_eye_relief_adapted.mp4"
video_codec = "mp4v"
frame_rate = 12

# 輸出 equirect 大小（水平對應 180°）
w_out_ = 4000   # 建議根據效能調整（越大越慢）
h_out_ = w_out_

# Eye relief（使用者輸入，單位 mm）
# 正值表示模擬「眼睛離鏡片更遠」的效果（畫面看起來被推遠）
# 負值表示「更接近鏡片」
target_eye_relief_mm = 15.0

# 是否對 remapped 結果做 inpaint（洞補）
# hole_inpaint = True
hole_inpaint = False
inpaint_radius = 3

# StereoSGBM 參數（可依情況微調）
minDisparity = 1
numDisparities = 64  # 必須是 16 的倍數
blockSize = 1

# ---------- 檢查檔案 ----------
raw_files = sorted(glob.glob(raw_pattern))
if len(raw_files) == 0:
    print("❌ 未找到任何 RAW 檔案，請確認檔名格式")
    sys.exit(1)

if not os.path.exists(stereo_param_file):
    print(f"❌ 找不到參數檔: {stereo_param_file}")
    sys.exit(1)

data = np.load(stereo_param_file)
K_l = data["K_l"]
D_l = data["D_l"]
K_r = data["K_r"]
D_r = data["D_r"]
R = data["R"]
T = data["T"]
DIM = tuple(data["DIM"])
DIM = (int(DIM[0]), int(DIM[1]))

T_norm = np.linalg.norm(T)
calib_unit = "mm"

print(f"Calibration baseline norm = {T_norm:.6f} ({calib_unit})")

target_eye_relief = target_eye_relief_mm  # mm

# ---------- stereo rectify（fisheye） ----------
# 中介解析度 (rectified 用)
DIM2 = (int(w_out_ * 1.2), int(w_out_ * 1.2))  # 可以調整 speed/accuracy tradeoff

R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    newImageSize=DIM2,
    balance=0.0, fov_scale=1.0
)

# 將 principal point move 到中心（視需要）
P1[0,2] = DIM2[0] / 2.0
P1[1,2] = DIM2[1] / 2.0
P2[0,2] = DIM2[0] / 2.0
P2[1,2] = DIM2[1] / 2.0

Q[0, 3] = -DIM2[0]/2
Q[1, 3] = -DIM2[1]/2

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM2, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM2, cv2.CV_16SC2)

# ---------- Stereo Matcher ----------
stereo = cv2.StereoSGBM_create(
    minDisparity=minDisparity,
    numDisparities=numDisparities,
    blockSize=blockSize,
    P1=8*3*blockSize**2,
    P2=32*3*blockSize**2,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# ---------- 建立 equirect 的 rays 與基本投影矩陣 ----------
# K_ 用於把方向向量 (ray) 投影到 rectified 中介相機平面 (camera intrinsics)
# 用 P1 的焦距與中心來建立
K_center = np.array([
    [P1[0,0], 0, DIM2[0] / 2.0],
    [0, P1[1,1], DIM2[1] / 2.0],
    [0, 0, 1.0]
], dtype=np.float32)

w_out = w_out_
h_out = h_out_

# lon: -90°..+90° (VR180 水平)
lon = np.linspace(-np.pi/2, np.pi/2, w_out)
#lat = np.linspace(np.pi/2, -np.pi/2, h_out)
lat = np.linspace(-np.pi / 2, np.pi / 2, h_out) 
lon_map, lat_map = np.meshgrid(lon, lat)

# 球面方向向量 (camera coord)
x = np.cos(lat_map) * np.sin(lon_map)
y = np.sin(lat_map)
z = np.cos(lat_map) * np.cos(lon_map)
rays = np.stack([x, y, z], axis=-1).astype(np.float32)  # shape (h_out, w_out, 3)

# 投影 rays 到 image plane (不含深度)
proj = rays @ K_center.T
z_safe_proj = np.where(proj[..., 2] == 0, 1e-6, proj[..., 2])
u = proj[..., 0] / z_safe_proj
v = proj[..., 1] / z_safe_proj

# 對於 remap 的基本 valid mask（在 rectified 圖範圍內）
h_in, w_in = DIM2[1], DIM2[0]
valid_basic = (u >= 0) & (u < w_in) & (v >= 0) & (v < h_in)

# 把 map_x/map_y 先存起來（用於把每個 equirect pixel sample depth）
map_x_basic = np.where(valid_basic, u, -1).astype(np.float32)
map_y_basic = np.where(valid_basic, v, -1).astype(np.float32)

# ---------- IO / 顯示準備 ----------
cv2.namedWindow("combined_f", cv2.WINDOW_NORMAL)
init_VideoWriter = False

fourcc = cv2.VideoWriter_fourcc(*video_codec)

print("開始處理影格，共:", len(raw_files))
start_time = time.time()

# ---------- 主迴圈 ----------
for idx, raw_file in enumerate(raw_files):
    # 讀 raw（假設 uint8 三通道，左右水平並列方形）
    with open(raw_file, "rb") as f:
        raw_data = f.read()
    frame_np = np.frombuffer(raw_data, dtype=np.uint8).copy().reshape((DIM[1], DIM[0]*2, 3))
    frame_l_raw = frame_np[:, :DIM[0], :]
    frame_r_raw = frame_np[:, DIM[0]:, :]

    # rectification -> rectified images (DIM2)
    rect_l = cv2.remap(frame_l_raw, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
    rect_r = cv2.remap(frame_r_raw, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

    # disparity (對 rectified 圖)
    gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
    disp = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0  # disparity in pixels

    # optional: 以 bilateral 或 median 去雜訊（可選）
    disp_filtered = cv2.medianBlur(disp, 5)

    # reproject to 3D (使用 Q)
    # Q 來自 stereoRectify，注意 reprojectImageTo3D 的輸入尺度與 Q 的單位需一致
    points_3d = cv2.reprojectImageTo3D(disp_filtered, Q)  # shape (h_in, w_in, 3)
    # 勿將整個 points_3d 複製到高解析度（會爆記憶體），用 map_x_basic/map_y_basic 直接 sample depth (Z)
    depth_map = points_3d[..., 2]  # Z (單位與 T 相同)

    # sample depth 到 equirect（使用 map_x_basic/map_y_basic）
    depth_equi = cv2.remap(depth_map, map_x_basic, map_y_basic, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1.0)

    # invalid depth => nan
    depth_equi = np.where(depth_equi <= 0, np.nan, depth_equi).astype(np.float32)

    # 建立每像素 3D 點 = ray * depth
    # rays shape (h_out, w_out, 3), depth_equi (h_out, w_out)
    depth_safe = depth_equi[..., None]  # (h_out, w_out, 1)
    points3d_equi = rays * depth_safe  # (h_out, w_out, 3) in left rectified cam coords

    # Eye Relief 補償：將每點沿 Z 軸平移 (注意座標系；Z 為前方)
    # 若使用 mm 單位的 calibrations，需要把 target_eye_relief 轉換為相同單位（上方已處理）
    # 把所有3D點在 Z 方向上「減去」 target_eye_relief（簡單模擬眼睛向後移）
    # 實際上：若 target_eye_relief > 0 (眼睛離鏡片更遠)，要把 3D 點相對 camera 向近看起來更遠
    # 這裡採用 points3d_new[..., 2] = points3d[..., 2] - target_eye_relief
    points3d_new = points3d_equi.copy()
    points3d_new[..., 2] = points3d_new[..., 2] - target_eye_relief

    # 投影函數（向量化）：把 3D 點投影回 rectified 相機平面，再 remap 取樣 rectified 圖
    def project_and_sample(points3d, K_mat, source_img):
        # points3d: (h_out, w_out, 3)
        proj = points3d @ K_mat.T  # (h_out,w_out,3)
        z_p = proj[..., 2]
        z_p_safe = np.where(np.abs(z_p) < 1e-6, 1e-6, z_p)
        u_p = proj[..., 0] / z_p_safe
        v_p = proj[..., 1] / z_p_safe

        # out-of-range 或 nan => -1
        invalid = np.isnan(u_p) | np.isnan(v_p) | (u_p < 0) | (u_p >= source_img.shape[1]) | (v_p < 0) | (v_p >= source_img.shape[0]) | (z_p <= 0)
        u_p = np.where(invalid, -1.0, u_p).astype(np.float32)
        v_p = np.where(invalid, -1.0, v_p).astype(np.float32)

        sampled = cv2.remap(source_img, u_p, v_p, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
        # mask of invalid pixels
        mask_invalid = invalid.astype(np.uint8)*255
        return sampled, mask_invalid

    # 因為要產生左右眼的 equirect 畫面（兩張），對左眼採樣 rect_l；對右眼採樣 rect_r
    # 若要左右眼在 Eye Relief 方向再做微調（例如左右偏移IPD），可以在此額外平移 X。
    res_equi_l, mask_l = project_and_sample(points3d_new, K_center, rect_l)
    res_equi_r, mask_r = project_and_sample(points3d_new, K_center, rect_r)

    # 合併（左右並列）
    combined_s = np.hstack((res_equi_l, res_equi_r))

    # 簡單的洞填補：先用 median 去雜訊，再以 inpaint 填洞（若啟用）
    if hole_inpaint:
        # 合併左右 mask 再填補各半張
        mask_combined = np.hstack((mask_l, mask_r))
        # 先用 median 去掉孤立小點
        combined_s_m = cv2.medianBlur(combined_s, 5)
        # inpaint 需要單通道 mask (0/255)
        inpainted = cv2.inpaint(combined_s_m, mask_combined, inpaint_radius, cv2.INPAINT_TELEA)
        display_img = inpainted
    else:
        display_img = combined_s

    # 顯示與輸出
    cv2.imshow("combined_f", display_img)
    if not init_VideoWriter:
        video_out = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (display_img.shape[1], display_img.shape[0]))
        init_VideoWriter = True
    video_out.write(display_img)

    # 進度
    progress = (idx+1)/len(raw_files)*100
    print(f"\r處理中 [{idx+1}/{len(raw_files)}] {progress:.2f}% ", end="")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\n使用者中斷")
        break

# ---------- 結束 ----------
if init_VideoWriter and video_out.isOpened():
    video_out.release()
cv2.destroyAllWindows()
elapsed = time.time() - start_time
print(f"\n完成，總耗時: {elapsed:.1f}s，輸出: {output_video_path}")

