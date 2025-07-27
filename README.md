1.透過tcp傳送pi camera畫面 
2.用pc接受畫面 並校正單鏡頭 與 雙鏡頭立體配對
3.由pi錄製raw畫面 並在pc上轉製


capture_
    capture  
        stetreo_2ssd_gpio.py  在pi執行 用gpio新增按鈕 來啟動錄製
    raw
        player_raw_.py  在pc執行 播放raw檔
        raw_2ssd.py  在pi執行 錄製raw檔
    stereo_raw
        play_stereo_raw_.py  在pc執行 播放原始stereo raw檔
        rectification_2sph_raw2mp4.py. 在pc執行 轉換到等距投影 搭配stereo calib校正參數 輸出mp4檔
        rectification_raw2mp4.py  在pc執行 校正配對 輸出mp4檔 搭配stereo calib校正參數 
        stetreo_2ssd_fixed.py  在pi執行 錄製stereo raw檔 搭配pcie ssd
        stetreo_by_ram.py  在pi執行 錄製stereo raw檔 

tcp_
    tcp_receive.py  在pc執行 檢視pi傳送畫面
    tcp_send_crop.py  在pi執行 傳送畫面
    tcp_receive_stereo.py  在pc執行 檢視pi傳送的stereo畫面
    tcp_send_stereo_crop.py  在pi執行 傳送stereo畫面

tcp_calib
    tcp_calib.py  在pc執行 校正魚眼單鏡頭  輸出calib校正參數
    tcp_fisheye_show.py  在pc執行 檢視校正後魚眼單鏡頭 搭配calib校正參數
    stereo
        tcp_calib_stereo.py  在pc執行 配對雙魚眼單鏡頭  搭配左右calib校正參數 輸出stereo calib校正參數
        rectification
            tcp_rectification.py  在pc執行 檢視stereo配對 搭配stereo calib校正參數
            tcp_rectification_3d.py  在pc執行 檢視深度圖 搭配stereo calib校正參數
            tcp_rectification_3d_pointcloud.py  在pc執行 檢視深度圖 搭配stereo calib校正參數 輸出點雲
            warp2sph
                tcp_rectification_2sph.py  在pc執行  檢視轉換等距投影 搭配stereo calib校正參數

virtual_chessboard.py  用螢幕顯示棋盤格標定板.
