# 無人機自動降落於靜態與動態平台上

目錄：
- 軟體模擬飛行結果(靜態與動態)
- 真實硬體飛行結果(靜態與動態)
- 系統架構
- 視覺偵測
- 控制器設計

## 1. 軟體模擬飛行結果(靜態與動態)

- [軟體模擬-靜態目標降落影片](https://youtu.be/Ru7Ih88LC58)

- [軟體模擬-動態目標降落影片](https://youtu.be/W8m_7iu91D4)

## 2. 真實硬體飛行結果(靜態與動態)

- [真實硬體-靜態目標降落影片-側錄](https://youtu.be/Y8nC2HKw1uc)

- [真實硬體-靜態目標降落影片-NX畫面](https://youtu.be/rj9Mq0KUjhk)

- [真實硬體-動態目標降落影片-側錄](https://youtu.be/AZ4wHHtOg1c)

- [真實硬體-動態目標降落影片-NX畫面](https://youtu.be/JMSZXIL2BW4)

## 3. 系統架構
整體系統架構如下：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure1.png)

- 邊緣運算器為Jetson Orin NX，軟體包含ROS系統、攝影機影像擷取、追蹤控制模組、降落控制模組和MAVROS。
- 無人機飛控版週邊包含GPS、IMU、數傳模組和Battery。
- 地面控制站主要遠端控制NX，並觀看影像畫面，以及無人機電池監控和遙控器緊急控制。

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure2.png)

- 追蹤控制模組：使用YOLOv4-tiny進行降落平台之即時目標檢測，並將目標資訊丟給FNN高度預測器預測高度，最後將中心點誤差和所預測高度輸入進追蹤PD控制器，進行控制。
- 降落控制模組：使用ArUco marker進行檢測，計算中心點誤差和面積大小，最後輸入至降落PD控制器，進行降落控制。

## 4. 視覺偵測

- 使用YOLOv4-tiny檢測架構，將其實現在Jetson Orin NX上，並使用GPU加速推論，FPS達20左右：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure3.png)

- 此為YOLO檢測結果(粉紅色框)，在不同高度下所檢測到的H降落平台：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure4.png)

- 此為ArUco marker在不同高度下所檢測之結果：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure5.png)

## 5. 控制器設計

- 追蹤PD控制器，包含ROS、YOLOv4-tiny和FNN模型之整合，以控制無人機X, Y, Z軸速度控制進行追蹤：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure6.png)

- 降落PD控制器，包含ROS和ArUco marker整合，以控制無人機X, Y軸速度控制進行降落，Z軸速度為等速下降：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure7.png)

- 所使用之 ROS 節點與話題發布圖：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure8.png)

- 所設計之FNN高度預測器，輸入為YOLOv4-tiny所檢測到H降落平台之pixel大小，輸出為所對應到無人機與H降落平台之間高度：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure9.png)

- FNN模型參數和訓練結果，預測出的平均絕對誤差為0.04。

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure10.png)

- 降落平台設計，可靜態也可動態移動：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure11.png)

- 所實驗的無人機設備，包含Jetson Orin NX、C925e webcam、F9P GPS、數傳模組、電池和Pixhawk 6C飛控版：

![image](https://github.com/Slaung/UAV-Automatically-Lands/blob/main/Figure/Figure12.png)




