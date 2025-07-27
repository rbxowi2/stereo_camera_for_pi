#pip install screeninfo opencv-python

import cv2
import numpy as np
import tkinter as tk
from tkinter import simpledialog
from screeninfo import get_monitors

def generate_chessboard(cols, rows, cell_size_mm):
    monitor = get_monitors()[0]
    screen_width_px = monitor.width
    screen_width_mm = monitor.width_mm
    
    px_mm = float(screen_width_px/screen_width_mm)
    
    img_width = int(cols * px_mm*cell_size_mm)
    img_height =int(rows * px_mm*cell_size_mm)
    
    chessboard = np.zeros((img_height, img_width), dtype=np.uint8)
    
    for row in range(rows):
        for col in range(cols):
            y_start = int(row * cell_size_mm*px_mm)
            y_end = int((row+1) * cell_size_mm*px_mm)
            x_start = int(col * cell_size_mm*px_mm)
            x_end = int((col+1) * cell_size_mm*px_mm)
            if (row + col) % 2 == 1:
                chessboard[y_start:y_end, x_start:x_end] = 255
            else:
                chessboard[y_start:y_end, x_start:x_end] = 0
                
    return chessboard

def on_generate():
    try:
        cols = int(entry_cols.get())
        rows = int(entry_rows.get())
        cell_size_mm = float(entry_cell_size.get())

        img = generate_chessboard(cols, rows, cell_size_mm)
        cv2.imshow(f"{cols}x{rows} Chessboard ({cell_size_mm}mm per cell)", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception as e:
        print("發生錯誤：", e)

# GUI 設定
root = tk.Tk()
root.title("棋盤格產生器")

tk.Label(root, text="橫向格數 (cols):").grid(row=0, column=0)
entry_cols = tk.Entry(root)
entry_cols.insert(0, "12")
entry_cols.grid(row=0, column=1)

tk.Label(root, text="縱向格數 (rows):").grid(row=1, column=0)
entry_rows = tk.Entry(root)
entry_rows.insert(0, "9")
entry_rows.grid(row=1, column=1)

tk.Label(root, text="每格大小 (mm):").grid(row=2, column=0)
entry_cell_size = tk.Entry(root)
entry_cell_size.insert(0, "25")
entry_cell_size.grid(row=2, column=1)

btn_generate = tk.Button(root, text="產生棋盤格 (generate)", command=on_generate)
btn_generate.grid(row=3, column=0, columnspan=2)

root.mainloop()
