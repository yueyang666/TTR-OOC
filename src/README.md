# ROS2 Workspace

這裡包含兩個 ROS2 Python 封包，負責產生並解析 UART 封包後發佈車輛狀態，以及示範性地訂閱與分析資料。

## 封包清單
- `py_serial_receive`: 透過 `rf_receiver_node` 讀取（目前以模擬封包產生器替代）UART 資料，依照 `py_serial_receive/config/uart_format.json` 解析後以 JSON 字串發佈到 `/vehicle/status`。啟動時會在專案根目錄寫入 `rf_receiver.log`。
- `py_analyze`: 示範性訂閱 `/vehicle/status` 的節點，解析接收到的 JSON 字串並將結果寫入日誌，可依實際欄位需求調整。

## 建置
```bash
# 1) 載入對應 ROS2 環境
source /opt/ros/<distro>/setup.bash

# 2) 在工作空間根目錄建置
colcon build --symlink-install

# 3) 匯入此工作空間環境
source install/setup.bash
```

## 執行節點
- 發佈模擬 UART 解析結果  
  ```bash
  ros2 run py_serial_receive rf_receiver_node
  ```

- 示範訂閱與分析  
  ```bash
  ros2 run py_analyze analyzer_node
  ```

`rf_receiver_node` 會以 1 Hz 定期發佈資料，欄位與單位由 `py_serial_receive/config/uart_format.json` 定義；若需對接實體 UART，請替換 `py_serial_receive/utils/fake_tty_trans.py` 及相關邏輯以讀取實際串列埠。

## 與前端儀表板整合
前端 Vue 儀表板透過 rosbridge websocket 訂閱 `/vehicle/status`。請在啟動上述節點後，另行啟動 rosbridge：
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
並確保 websocket URL 與 `ros2-dashboard/src/App.vue`（或 `RosConnector.js`）中設定的位址一致。
