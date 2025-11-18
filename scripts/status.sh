#!/bin/bash

echo "=============================="
echo " 🚦 系統服務狀態 "
echo "=============================="

echo ""
echo "[Vue / Node]"
pgrep -af "npm|node|vue" || echo "無 Vue / Node 行程"

echo ""
echo "[ROS2 Processes]"
pgrep -af "ros2|rosbridge|rf_receiver" || echo "無 ROS2 行程"

echo ""
echo "[ROS2 Nodes]"
ros2 node list 2>/dev/null || echo "ros2 node list 無回應（可能沒有 ROS 正在運行）"
