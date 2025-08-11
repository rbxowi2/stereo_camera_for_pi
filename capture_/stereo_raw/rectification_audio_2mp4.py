#ffmpeg
import cv2
import numpy as np
import os
import glob
import sys
from datetime import datetime
from pydub import AudioSegment
import subprocess
import re

# ====== 參數設定 ======
frame_rate = 15

param_file = "stereo_camera_params.npz"

txt_files = glob.glob("recording_*.txt")
txt_file = txt_files[0]

audio_file =glob.glob("recording_*.wav")
audio_file = audio_file[0]

output_video_path = "rectified_output.mp4"

# ====== 讀取 txt 檔，解析 Audio_Start Time 與每幀時間戳 ======
def parse_txt_timestamps(txt_path):
    with open(txt_path, "r") as f:
        lines = f.readlines()

    audio_start_line = lines[0]
    audio_start_time_str = re.search(r'Audio_Start Time: (.+)', audio_start_line).group(1)
    audio_start_time = datetime.fromisoformat(audio_start_time_str)

    # 幀時間戳列表
    frame_times = []
    for line in lines[2:]:  # 從第3行開始為幀時間戳
        parts = line.strip().split()
        if len(parts) == 2:
            # 格式: "00000 2025-08-04T16:04:11.623"
            frame_times.append(datetime.fromisoformat(parts[1]))
    return audio_start_time, frame_times

audio_start_time, frame_times = parse_txt_timestamps(txt_file)
total_frames = len(frame_times)
if total_frames == 0:
    print("❌ 沒有讀取到任何時間戳")
    sys.exit(1)

print(f"🔊 Audio Start Time: {audio_start_time}")
print(f"🎞️ 影片共 {total_frames} 幀")

# ====== 讀取相機參數 ======
if not os.path.exists(param_file):
    print(f"❌ 找不到參數檔 {param_file}")
    sys.exit(1)

data = np.load(param_file)
K_l, D_l = data["K_l"], data["D_l"]
K_r, D_r = data["K_r"], data["D_r"]
R, T = data["R"], data["T"]
DIM = tuple(map(int, data["DIM"]))
frame_width, frame_height = DIM
channels = 3

# ====== fisheye 校正參數 ======
fov_scale_ = 0.2
roll = np.deg2rad(0)
yaw = np.deg2rad(0)
pitch = np.deg2rad(0)

R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K_l, D_l, K_r, D_r, DIM, R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    newImageSize=DIM,
    balance=0.0,
    fov_scale=fov_scale_
)
R_look, _ = cv2.Rodrigues(np.array([roll, yaw, pitch]))
R1, R2 = R_look @ R1, R_look @ R2

P1[0, 2] = DIM[0] / 2
P1[1, 2] = DIM[1] / 2
P2[0, 2] = DIM[0] / 2
P2[1, 2] = DIM[1] / 2

map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K_l, D_l, R1, P1, DIM, cv2.CV_16SC2)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K_r, D_r, R2, P2, DIM, cv2.CV_16SC2)

# ====== 處理音訊，剪裁與影片時間對齊 ======
audio = AudioSegment.from_wav(audio_file)
video_start = frame_times[0]
delta_ms = int((video_start - audio_start_time).total_seconds() * 1000)

if delta_ms > 0:
    trimmed_audio = audio[delta_ms:]
else:
    trimmed_audio = AudioSegment.silent(duration=abs(delta_ms)) + audio

video_duration_ms = int((frame_times[-1]-frame_times[0]).total_seconds() * 1000)
trimmed_audio = trimmed_audio[:video_duration_ms]
trimmed_audio.export("temp_audio.wav", format="wav")

# ====== 開始影像處理 ======
cv2.namedWindow("Rectified", cv2.WINDOW_NORMAL)
video_writer = None

frame_interval = 1 / frame_rate
prev_time = None
prev_frame = None

for idx, current_time in enumerate(frame_times):
    raw_file = f"frame_{idx:04d}.raw"
    # 判斷檔案是否存在
    if os.path.exists(raw_file):
        with open(raw_file, "rb") as f:
            raw_data = f.read()
        frame_np = np.frombuffer(raw_data, dtype=np.uint8).reshape((frame_height, frame_width * 2, channels))
        frame_l, frame_r = frame_np[:, :frame_width], frame_np[:, frame_width:]
        rect_l = cv2.remap(frame_l, map1_l, map2_l, interpolation=cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)
        combined = np.hstack((rect_l, rect_r))
        
    else:
        if prev_frame is not None:
            print(f"⚠️ {raw_file} 不存在，使用上一幀補幀")
            combined = prev_frame.copy()
        else:
            print(f"❌ 無法讀取幀且無前一幀可補，跳過第 {idx} 幀")
            continue

    if video_writer is None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter("temp_video.mp4", fourcc, frame_rate, (combined.shape[1], combined.shape[0]))

    # 判斷掉幀數
    if prev_time is not None:
        delta_t = (current_time - prev_time).total_seconds()
        num_missing = int(delta_t / frame_interval) - 1
        if num_missing > 0:
            for _ in range(num_missing):
                video_writer.write(prev_frame)
            print(f"🔁 掉幀補幀 {num_missing} 幀: {prev_time} -> {current_time}")

    video_writer.write(combined)
    prev_frame = combined.copy()
    prev_time = current_time

    # 畫面輔助線
    for y in range(0, combined.shape[0], 50):
        cv2.line(combined, (0, y), (combined.shape[1], y), (0, 128, 0), 1)
    cv2.line(combined, (combined.shape[1]//4, 0), (combined.shape[1]//4, combined.shape[0]), (0, 0, 255), 3)
    cv2.line(combined, (combined.shape[1]*3//4, 0), (combined.shape[1]*3//4, combined.shape[0]), (0, 0, 255), 3)
    cv2.line(combined, (0, combined.shape[0]//2), (combined.shape[1], combined.shape[0]//2), (0, 0, 255), 3)
    cv2.line(combined, (combined.shape[1]//2, 0), (combined.shape[1]//2, combined.shape[0]), (0, 255, 255), 3)

    cv2.imshow("Rectified", combined)
    print(f"📊 處理中: [{idx + 1}/{total_frames}] {((idx + 1) / total_frames) * 100:.2f}%", end='\r')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if video_writer is not None:
    video_writer.release()
cv2.destroyAllWindows()
print("\n✅ 影片處理完成，開始合併音訊")

# ====== ffmpeg 合併音訊與影片 ======
cmd = [
    "ffmpeg", "-y",
    "-i", "temp_video.mp4",
    "-i", "temp_audio.wav",
    "-c:v", "copy",
    "-c:a", "aac",
    "-strict", "experimental",
    output_video_path
]

subprocess.run(cmd)
print(f"✅ 合併完成，輸出檔案: {output_video_path}")
