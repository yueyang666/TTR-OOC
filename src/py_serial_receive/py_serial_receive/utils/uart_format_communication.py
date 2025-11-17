import os
import json
from ament_index_python.packages import get_package_share_directory

def decode_packet(packet: bytes, json_path=None) -> dict:
    """根據 JSON 格式自動解碼 UART 封包"""

    # 1️⃣ 若 json_path 沒指定 → 自動找 ROS2 share 目錄
    if json_path is None:
        pkg_share = get_package_share_directory('py_serial_receive')
        json_path = os.path.join(pkg_share, 'config', 'uart_format.json')  # ← 修正重點！

    # 2️⃣ 讀取 JSON
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"找不到 JSON 檔案：{json_path}")

    with open(json_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    # 3️⃣ 協議設定
    proto = cfg["protocol"]
    header = bytes(proto["header"])
    length_index = proto["length_index"]
    data_offset = proto["data_offset"]

    # === 驗證 header ===
    if not packet.startswith(header):
        raise ValueError(f"封包開頭錯誤，預期 {header}, 實際 {packet[:len(header)]}")

    result = {}

    # 4️⃣ 解碼每個欄位
    for field in cfg["fields"]:
        name = field["name"]
        offset = field["offset"]
        length = field["length"]
        order = field.get("byte_order", "big")

        raw_bytes = packet[offset:offset + length]

        if len(raw_bytes) < length:
            result[name] = None
            continue

        value = int.from_bytes(raw_bytes, byteorder=order, signed=False)

        result[name] = {
            "raw": value,
            "unit": field.get("unit", "")
        }

    return result
