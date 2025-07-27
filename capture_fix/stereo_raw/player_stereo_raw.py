import cv2
import numpy as np
import glob
import time


cv2.namedWindow("RAW Player", cv2.WINDOW_NORMAL)

# 配置參數
frame_width = 1944
frame_height = 1944
pixel_format = 3  # RGB888
frame_rate = 15

# ✅ 加入旋轉角度設定（可選 0, 90, 180, 270）
rotate_angle = 0  # <<<<<< 可改成 0 / 90 / 180 / 270

# 加載 RAW 檔案列表
raw_files = sorted(glob.glob("frame_*.raw"))
if not raw_files:
    print("未找到任何 RAW 檔案")
    exit()

print(f"找到 {len(raw_files)} 幀，開始循環播放（按 'q' 鍵退出）...")

# 旋轉函式
def rotate_frame(frame, angle):
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return frame  # 不旋轉

while True:
    for raw_file in raw_files:
        with open(raw_file, "rb") as f:
            raw_data = f.read()

        frame = np.frombuffer(raw_data, dtype=np.uint8)
        try:
            frame = frame.reshape((frame_height, frame_width*2, pixel_format))
        except ValueError:
            print(f"解析錯誤：{raw_file} 的資料大小與解析度不符")
            continue

        # 套用旋轉
        frame = rotate_frame(frame, rotate_angle)

        cv2.imshow("RAW Player", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("使用者退出播放")
            cv2.destroyAllWindows()
            exit()

        time.sleep(1 / frame_rate)
