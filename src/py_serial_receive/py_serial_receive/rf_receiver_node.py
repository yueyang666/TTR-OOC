# rf_receiver_node.py
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from py_serial_receive.utils.log import Logger_tool
from py_serial_receive.utils.fake_tty_trans import generate_fake_packet
from py_serial_receive.utils.uart_format_communication import decode_packet


class RFReceiverNode(Node):
    def __init__(self):
        super().__init__('rf_receiver_node')
        self.logger = Logger_tool.init_logger(log_file="rf_receiver.log")
        self.logger.info("RF Receiver 節點啟動")

        self.publisher = self.create_publisher(String, '/vehicle/status', 10)

        # JSON 協定檔位置
        pkg_share = get_package_share_directory('py_serial_receive')
        self.json_path = os.path.join(pkg_share, 'config', 'uart_format.json')

        self.timer = self.create_timer(1.0, self.read_and_publish)

    def read_and_publish(self):
        try:
            fake_packet = generate_fake_packet(self.json_path)
            decoded = decode_packet(fake_packet)
            self.publisher.publish(String(data=json.dumps(decoded)))
            self.logger.info(f"已發佈資料: {decoded}")
        except Exception as e:
            self.logger.error(f"解析或發佈失敗: {e}")


def main():
    rclpy.init()
    node = RFReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
