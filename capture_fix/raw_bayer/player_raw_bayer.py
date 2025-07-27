import cv2
import numpy as np
import glob
import time

frame_width = 1296  # 原始解析度，不用乘2
frame_height = 972
frame_rate = 30
bayer_pattern = cv2.COLOR_BAYER_BG2BGR
rotate_angle = 180

raw_files = sorted(glob.glob("frame_*_bayer.raw"))
if not raw_files:
    print("未找到任何 SBGGR10 RAW 檔案")
    exit()

def rotate_frame(frame, angle):
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame

print(f"找到 {len(raw_files)} 幀，開始播放...（按 q 鍵退出）")

while True:
    for raw_file in raw_files:
        with open(raw_file, "rb") as f:
            raw_data = f.read()
        try:
            bayer = np.frombuffer(raw_data, dtype=np.uint16).reshape((frame_height, frame_width))
        except ValueError:
            print(f"讀取錯誤：{raw_file} 資料大小不符")
            continue

        rgb_frame = cv2.demosaicing(bayer, bayer_pattern)

        # 亮度與對比度調整
        rgb_8bit = cv2.convertScaleAbs(rgb_frame, alpha=255/1023)
        alpha = 1.2  # 對比度
        beta = 20    # 亮度
        adjusted = cv2.convertScaleAbs(rgb_8bit, alpha=alpha, beta=beta)

        # 旋轉
        adjusted = rotate_frame(adjusted, rotate_angle)

        cv2.imshow("SBGGR10 RAW 播放器", adjusted)
        #cv2.imshow(“raw2rgb”, rgb_8bit)

        if cv2.waitKey(int(1000/frame_rate)) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            exit()
