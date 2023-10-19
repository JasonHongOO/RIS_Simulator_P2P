# 執行方式
依序開啟 Client A 、 Client B 、 Client C 即可

# 說明

Client C 模擬 UE 移動，回傳 RSRP 給 Client B


Client B 則是 xApp 接收 UE 回傳的 RSRP 做出 RIS 反射角度切換的決策 


Client A 提供 GUI 介面來方便使用者觀察切換的決策是否正確，判斷是否達成 Beam Tracking 的目標，並提供相關參數的 log 資料方便 Debug

# 流程圖

![image](https://github.com/JasonHongOO/RIS_Simulator_P2P/blob/main/Image/Flowchart.png)

# GUI 介面

![image](https://github.com/JasonHongOO/RIS_Simulator_P2P/blob/main/Image/GUI.jpg)
