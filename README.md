## 專案概要
ROS2 節點把 UART 封包解析成車輛狀態後發佈到 `/vehicle/status`，Vue 儀表板透過 rosbridge websocket 訂閱並即時呈現。此倉庫同時包含 ROS2 工作空間與前端專案。

- ROS2 封包說明與執行方式：參見 [ROS2 README](src/README.md)
- Vue 儀表板開發與部署：參見 [Dashboard README](ros2-dashboard/README.md)

## 快速開始
1. **建置並啟動 ROS2 節點**  
   `source /opt/ros/<distro>/setup.bash && colcon build --symlink-install && source install/setup.bash`  
   執行 `ros2 run py_serial_receive rf_receiver_node`（需要時可再開 `py_analyze` 訂閱端）。
2. **啟動 rosbridge websocket**（前端需連線）  
   `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
3. **啟動前端儀表板**  
   在 `ros2-dashboard` 執行 `npm install && npm run serve`，並確保 `src/App.vue` 內的 websocket URL 與 rosbridge 相符。

## 專案結構
- `src/`：ROS2 Python 封包 (`py_serial_receive`, `py_analyze`) 與設定
- `ros2-dashboard/`：Vue 3 儀表板程式碼
- `rf_receiver.log`：`rf_receiver_node` 產出的運行日誌（啟動後生成）
