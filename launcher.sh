#!/bin/bash

echo "======================================="
echo "    車輛動態行動控制中心 — 啟動中 "
echo "======================================="

PID_FILE="./log/service.pid"

### 1. 載入 ROS2 環境
echo "[1/4] 載入 ROS2 環境..."
source /opt/ros/jazzy/setup.bash
rm -f "$PID_FILE"

### 2. 切換至 ROS2 工作區
echo "[2/4] 切換至 ROS2 工作區..."
cd ~/TTR-OOC || { echo "找不到 TTR-OOC 目錄"; exit 1; }

# 若你每次都有用這個工作區
if [ -d install ]; then
    echo "    載入 install/setup.bash"
    source install/setup.bash
fi

### 3. 啟動 ROSBridge WebSocket
echo "[3/4] 啟動 rosbridge_server..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    > rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
echo "rosbridge=$ROSBRIDGE_PID" >> $PID_FILE

### 4. 啟動 RF Receiver Node
nohup ros2 run py_serial_receive rf_receiver_node \
    > rf_receiver.log 2>&1 &
RF_PID=$!
echo "rf_receiver_node=$RF_PID" >> $PID_FILE

### 5. Vue Dashboard
cd ros2-dashboard
nohup npm run serve > dashboard.log 2>&1 &
DASHBOARD_PID=$!
cd ..
echo "dashboard=$DASHBOARD_PID" >> $PID_FILE

echo ""
echo "======================================="
echo "全部服務啟動成功！"
echo ""
echo "ROSBridge       PID: $ROSBRIDGE_PID"
echo "RF Receiver     PID: $RF_PID"
echo "Vue Dashboard   PID: $DASHBOARD_PID"
echo ""
echo "PID 記錄檔: $PID_FILE"
echo ""
echo "開啟瀏覽器: http://localhost:8080"
echo "======================================="
echo ""
