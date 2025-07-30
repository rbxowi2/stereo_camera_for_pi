<!DOCTYPE html>
<html lang="zh-tw">
<head>
    <meta charset="UTF-8">
</head>
<body>
    <h1>stereo_camera_for_pi 專案說明</h1>
    <ul>
      <li>可以隨身攜帶的VR相機 / A portable VR camera</li>
    </ul>
    <h2>本專案使用 / This project uses :</h2>
    <ul>
        <li>樹梅派 5（8GB） / Raspberry Pi 5 (8GB)</li>
        <li>兩顆 OV5647 魚眼相機模組 / Two OV5647 fisheye camera modules</li>
        <li>PCIe SSD </li>
    </ul>
    <h2>工作流程 / Workflow :</h2>
    <ol>
        <li>透過tcp傳送pi camera畫面. <br>Transmit Pi camera images via TCP</li>
        <li>用pc接收畫面 並校正單鏡頭 與 雙鏡頭立體配對. <br>Receive images on PC and calibrate single camera and stereo camera pairing</li>
        <li>由pi錄製raw畫面 並在pc上轉製. <br>Record raw images on Pi and convert them on PC</li>
    </ol>
    <h2>📁 專案目錄結構 / Project Directory Structure :</h2>

<pre><code>
capture_/
    capture/                        
        ├── stereo_ssd_gpio.py            # [Pi] 使用 GPIO 按鈕啟動錄影 / Record stereo with GPIO button
    raw/                                   
        ├── raw_ram.py                    # [Pi] 使用記憶體緩衝錄製 raw 影像 / Record raw using RAM buffer
        ├── raw_viewer.py                 # [PC] 播放 raw 檔 / Play raw files
    stereo_raw/                            
        ├── rectification_2mp4.py         # [PC] 校正後輸出 mp4 / rectified and output mp4 
        ├── rectification_dp_2mp4.py      # [PC] 顯示深度圖並輸出 mp4 /Convert to depth map and output mp4
        ├── rectification_sph_2mp4.py     # [PC] 轉換為等距柱狀投影並輸出 mp4 /Convert to equirectangular projection and output mp4 
        ├── stereo_raw_viewer.py          # [PC] 播放 stereo raw / Play stereo raw files
        ├── stereo_ram.py                 # [Pi] 使用記憶體緩衝錄製 stereo raw / Record stereo raw using RAM buffer
        ├── stereo_ssd_fixed.py           # [Pi] 使用 PCIe SSD 錄製 stereo raw / Record stereo raw to PCIe SSD
        
tcp_/
    ├── tcp_receive.py                # [PC] 接收並檢視單鏡頭串流畫面 / View single image stream from Pi
    ├── tcp_send_crop.py              # [Pi] 傳送裁切後畫面 / Send cropped single image stream
    ├── tcp_send_stereo_crop.py       # [Pi] 傳送裁切後 stereo 畫面 / Send cropped stereo stream
    ├── tcp_receive_stereo.py         # [PC] 接收 stereo 畫面 / View stereo stream from Pi
    
tcp_calib/
    ├── tcp_calib.py                  # [PC] 單魚眼鏡頭校正 / Calibrate single fisheye lens
    ├── tcp_undistorted_player.py     # [PC] 顯示校正後魚眼影像 / View corrected fisheye image
    stereo/
        ├── tcp_stereo_calib.py           # [PC] 雙魚眼 stereo 校正 / Stereo calibration with two fisheye lenses
        rectification/
            ├── tcp_rectification_dp_2pointcloud.py # [PC] 生成並輸出點雲 / Generate and export point clouds
            ├── tcp_rectification_dp.py       # [PC] 生成並顯示深度圖 / Generate and view depth maps
            ├── tcp_rectification.py          # [PC] 顯示 stereo 校正後影像 / View stereo rectified images
            warp2sph/
                ├── tcp_rectification_sph.py     # [PC] 轉換為等距柱狀投影 / Convert to equirectangular projection

virtual_chessboard.py         # [PC] 顯示虛擬棋盤格供校正使用 / Display on-screen chessboard for calibration

</code>
</pre>
 <h2>範例 / stereo example :</h2>
    <!-- 新增GIF範例 -->
    <li> raw : </li>
    <img src="raw.JPG" alt="raw"width="600" height="300">
    <li> equirectangular projection : </li>
    <img src="sph_rectified_output.gif" alt="sph_rectified_output.gif"width="600" height="300">
    <li> undistorted : </li>
    <img src="rectified_output.gif" alt="rectified_output.gif"width="600" height="300">
    <li> depth map : </li>
    <img src="dp.gif" alt="dp.gif" width="600">
</body>
</html>
