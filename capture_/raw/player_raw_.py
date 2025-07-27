import cv2
import numpy as np
import glob

# 配置參數
frame_width = 1296
frame_height = 972
pixel_format = 3  # RGB888

# ✅ 加入旋轉角度設定（可選 0, 90, 180, 270）
rotate_angle = 180  # <<<<<< 可改成 0 / 90 / 180 / 270

# 加載 RAW 檔案列表
raw_files = sorted(glob.glob("frame_*.raw"))
if not raw_files:
    print("未找到任何 RAW 檔案")
    exit()

print(f"找到 {len(raw_files)} 幀，按空白鍵播放下一幀（按 'q' 鍵退出）...")

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

index = 0
while index < len(raw_files):
    raw_file = raw_files[index]
    with open(raw_file, "rb") as f:
        raw_data = f.read()

    frame = np.frombuffer(raw_data, dtype=np.uint8)
    try:
        frame = frame.reshape((frame_height, frame_width, pixel_format))
    except ValueError:
        print(f"解析錯誤：{raw_file} 的資料大小與解析度不符")
        index += 1
        continue

    # 套用旋轉
    frame = rotate_frame(frame, rotate_angle)

    cv2.imshow("RAW Player", frame)

    key = cv2.waitKey(0) & 0xFF  # 等待按鍵事件
    if key == ord('q'):
        print("使用者退出播放")
        break
    elif key == 32:  # 空白鍵的 ASCII 是 32
        index += 1
    else:
        print("按下其他鍵，請按空白鍵播放下一幀或按 q 離開")

cv2.destroyAllWindows()
