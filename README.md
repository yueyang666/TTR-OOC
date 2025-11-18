# Taipei Tech Racing Operations Control Centre (TTROCC)

## OUTLINE
在 ROS2 的框架下，透過 ROS 的數據流完成以下任務(待增加) :
- 將無線電的 UART 封包解析成車輛狀態後發佈到 `/vehicle/status`
- Vue 儀表板透過 rosbridge websocket 訂閱並即時呈現。  
- ROS2 封包說明與執行方式：參見 [ROS2 README](src/README.md)
- Vue 儀表板開發與部署：參見 [Dashboard README](ros2-dashboard/README.md)
- 安裝詳細資訊請：參見 [Installstion README](./installation.md)

此倉庫同時包含 ROS2 工作空間與前端專案。

## Environmental Requirements
> [!NOTE] 
> 請特別注意，由於專案橫跨 ROS, venv, python, cpp, node.js, vue 等等，對於系統位置管理非常要求，請務必注意程式路徑寫法，以免報錯找不到 source 

### OS
- x86_64
- GNU/Linux
- 6.8.0-87-generic
- Ubuntu 24.04.3 LTS
- RAM > 2GB
- ROM > 32GB

### Package
- ROS2-jazzy
- Anaconda (or miniconda)
- python 3.12
- Node.js
- VUE 3.0



## Quick Start
1. **請先下載倉庫**
   ```bash
   git clone https://github.com/yueyang666/TTR-OOC.git
   ```

2. **建置 ROS2 工作區**
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build
   source install/setup.bash
   ```  
3. **啟動 ROS2 節點**
   ```bash
   ros2 run py_serial_receive rf_receiver_node
   ``` 
4. **啟動 rosbridge websocket**（前端需連線）  
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ``` 
5. **啟動前端儀表板**  
   在 `ros2-dashboard` 執行 `npm install && npm run serve`，並確保 `src/App.vue` 內的 websocket URL 與 rosbridge 相符。
6. **編譯網頁(optional)**  
   在 `ros2-dashboard` 執行 `npm run build` 會產生 `/dist` 靜態檔案，可用 `serve -s dist`、Nginx 等靜態伺服器直接部署 `dist/` 內容。

## 專案結構
- `src/`：ROS2 Python 封包 (`py_serial_receive`, `py_analyze`) 與設定
- `ros2-dashboard/`：Vue 3 儀表板程式碼
