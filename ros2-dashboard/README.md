# ROS2 Dashboard（Vue 3）

這個前端以 Vue 3 + roslib.js 建立單頁儀表板，透過 rosbridge websocket 訂閱 `/vehicle/status`，並即時顯示各參數數值與單位。

## 前置需求
- Node.js 16+（建議 18 或更新）與 npm
- 已啟動的 rosbridge websocket 服務（預設程式指向 `ws://192.168.0.107:9090`，請自行調整）
- ROS2 端持續在 `/vehicle/status` 發佈 JSON 字串（例如 `py_serial_receive` 的 `rf_receiver_node`）

## 安裝與運行
```bash
cd ros2-dashboard
npm install

# 開發模式（含熱更新）
npm run serve

# 生產版編譯
npm run build

# 程式碼風格檢查
npm run lint
```

開發伺服器啟動後，瀏覽終端輸出的 URL（通常為 http://localhost:8080）。

## 設定 rosbridge 位址
`src/App.vue` 內的
```js
const ros = new RosConnector('ws://192.168.0.107:9090')
```
改成你的 rosbridge websocket URL 後重新啟動開發伺服器即可。

## 訊息格式
UI 會解析 `RosConnector` 回傳的 JSON，期望每個欄位是：
```json
{
  "欄位名稱": { "raw": 數值, "unit": "單位" }
}
```
若 `raw` 不存在或無法解析，畫面會顯示「無資料」。
