#!/bin/bash

PID_FILE="./log/service.pid"

echo "======================================="
echo "             停止所有服務 "
echo "======================================="

if [ ! -f "$PID_FILE" ]; then
    echo "找不到 PID 檔案：$PID_FILE"
    exit 1
fi

while IFS='=' read -r name pid
do
    if kill -0 "$pid" 2>/dev/null; then
        echo "停止 $name (PID: $pid)"
        kill "$pid"
    else
        echo "$name (PID: $pid) 已不在執行"
    fi
done < "$PID_FILE"

rm -f "$PID_FILE"

echo "======================================="
echo " ✔ 全部服務已停止"
echo "======================================="
