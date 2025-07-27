import cv2
import numpy as np
import glob
import time

cv2.namedWindow("SBGGR10 RAW 1", cv2.WINDOW_NORMAL)
cv2.namedWindow("SBGGR10 RAW 2", cv2.WINDOW_NORMAL)

frame_width = 1296*2  # 原始解析度，不用乘2
frame_height = 972*2
frame_rate = 10
bayer_pattern = cv2.COLOR_BayerGBRG2BGR 
rotate_angle = 0

raw_files = sorted(glob.glob("frame_*.raw"))
if not raw_files:
    print("未找到任何 SBGGR10 RAW 檔案")
    exit()



print(f"找到 {len(raw_files)} 幀，開始播放...（按 q 鍵退出）")

while True:
    for raw_file in raw_files:
        with open(raw_file, "rb") as f:
            raw_data = f.read()
        try:
            bayer = np.frombuffer(raw_data, dtype=np.uint16).reshape((frame_height, frame_width*2))
        except ValueError:
            print(f"讀取錯誤：{raw_file} 資料大小不符")
            continue

        rgb_frame = cv2.demosaicing(bayer, bayer_pattern)

        # 亮度與對比度調整
        rgb_8bit = cv2.convertScaleAbs(rgb_frame, alpha=1023/65535)
        alpha = 1  # 對比度
        beta = 0    # 亮度
        adjusted = cv2.convertScaleAbs(rgb_8bit, alpha=alpha, beta=beta)


        cv2.imshow("SBGGR10 RAW 1", rgb_frame)
        cv2.imshow("SBGGR10 RAW 2", adjusted)
        #cv2.imshow(“raw2rgb”, rgb_8bit)

        if cv2.waitKey(int(1000/frame_rate)) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            exit()
