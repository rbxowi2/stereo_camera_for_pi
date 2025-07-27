<!DOCTYPE html>
<html lang="zh-tw">
<head>
    <meta charset="UTF-8">
    <title>stereo_camera_for_pi 專案說明</title>
</head>
<body>
    <h1>stereo_camera_for_pi 專案說明</h1>
    <ol>
        <li>透過tcp傳送pi camera畫面. <br>Transmit Pi camera images via TCP</li>
        <li>用pc接受畫面 並校正單鏡頭 與 雙鏡頭立體配對. <br>Receive images on PC and calibrate single camera and stereo camera pairing</li>
        <li>由pi錄製raw畫面 並在pc上轉製. <br>Record raw images on Pi and convert them on PC</li>
    </ol>
    <h2>目錄結構</h2>
    <ul>
        <li><strong>capture_</strong>
            <ul>
                <li>capture
                    <ul>
                        <li>stetreo_2ssd_gpio.py - 在pi執行，用gpio新增按鈕來啟動錄製</li>
                    </ul>
                </li>
                <li>raw
                    <ul>
                        <li>player_raw_.py - 在pc執行，播放raw檔</li>
                        <li>raw_2ssd.py - 在pi執行，錄製raw檔</li>
                    </ul>
                </li>
                <li>stereo_raw
                    <ul>
                        <li>play_stereo_raw_.py - 在pc執行，播放原始stereo raw檔</li>
                        <li>rectification_2sph_raw2mp4.py - 在pc執行，轉換到等距投影，搭配stereo calib校正參數，輸出mp4檔</li>
                        <li>rectification_raw2mp4.py - 在pc執行，校正配對，輸出mp4檔，搭配stereo calib校正參數</li>
                        <li>stetreo_2ssd_fixed.py - 在pi執行，錄製stereo raw檔，搭配pcie ssd</li>
                        <li>stetreo_by_ram.py - 在pi執行，錄製stereo raw檔</li>
                    </ul>
                </li>
            </ul>
        </li>
        <li><strong>tcp_</strong>
            <ul>
                <li>tcp_receive.py - 在pc執行，檢視pi傳送畫面</li>
                <li>tcp_send_crop.py - 在pi執行，傳送畫面</li>
                <li>tcp_receive_stereo.py - 在pc執行，檢視pi傳送的stereo畫面</li>
                <li>tcp_send_stereo_crop.py - 在pi執行，傳送stereo畫面</li>
            </ul>
        </li>
        <li><strong>tcp_calib</strong>
            <ul>
                <li>tcp_calib.py - 在pc執行，校正魚眼單鏡頭，輸出calib校正參數</li>
                <li>tcp_fisheye_show.py - 在pc執行，檢視校正後魚眼單鏡頭，搭配calib校正參數</li>
                <li>stereo
                    <ul>
                        <li>tcp_calib_stereo.py - 在pc執行，配對雙魚眼單鏡頭，搭配左右calib校正參數，輸出stereo calib校正參數</li>
                        <li>rectification
                            <ul>
                                <li>tcp_rectification.py - 在pc執行，檢視stereo配對，搭配stereo calib校正參數</li>
                                <li>tcp_rectification_3d.py - 在pc執行，檢視深度圖，搭配stereo calib校正參數</li>
                                <li>tcp_rectification_3d_pointcloud.py - 在pc執行，檢視深度圖，搭配stereo calib校正參數，輸出點雲</li>
                                <li>warp2sph
                                    <ul>
                                        <li>tcp_rectification_2sph.py - 在pc執行，檢視轉換等距投影，搭配stereo calib校正參數</li>
                                    </ul>
                                </li>
                            </ul>
                        </li>
                    </ul>
                </li>
            </ul>
        </li>
        <li><strong>virtual_chessboard.py</strong> - 用螢幕顯示棋盤格標定板</li>
    </ul>
</body>
</html>
