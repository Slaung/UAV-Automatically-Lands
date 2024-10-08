# 無人機自動降落於靜態與動態平台上

目錄：
- 系統架構
- 視覺偵測
- 控制器設計
- 軟體模擬飛行結果(靜態與動態)
- 真實硬體飛行結果(靜態與動態)

## 1. 系統架構
整體系統架構如下：

![image](https://github.com/Slaung/UAV-Automatically-Lands/tree/main/Figure/Figure1.png)

- 邊緣運算器為Jetson Orin NX，軟體包含ROS系統、攝影機影像擷取、追蹤控制模組、降落控制模組和MAVROS。
- 無人機飛控版週邊包含GPS、IMU、數傳模組和Battery。
- 地面控制站主要遠端控制NX，並觀看影像畫面，以及無人機電池監控和遙控器緊急控制。
