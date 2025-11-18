# rf_receiver_node.py
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from py_bridge_master.utils.fake_tty_trans import generate_fake_packet
from py_bridge_master.utils.uart_format_communication import decode_packet


class RFReceiverNode(Node):
    def __init__(self):
        super().__init__('rf_receiver_node')
        self.logger = self.get_logger()
        #self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.logger.info("RF Receiver 節點啟動")
        self.log_counter = 0
        self.publisher = self.create_publisher(String, '/vehicle/status', 10)

        # JSON 協定檔位置
        pkg_share = get_package_share_directory('py_bridge_master')
        self.json_path = os.path.join(pkg_share, 'config', 'uart_format.json')

        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        try:
            fake_packet = generate_fake_packet(self.json_path)
            decoded = decode_packet(fake_packet)
            self.publisher.publish(String(data=json.dumps(decoded)))
            # 每 10 次（1 秒）記錄一次日誌
            self.log_counter += 1
            if self.log_counter >= 10:  # 0.1 秒 * 10 = 1 秒
                self.logger.info(f"publish heartbeat check")
                self.logger.debug(f"{decoded}")
                self.log_counter = 0

        except Exception as e:
            self.logger.error(f"解析或發佈失敗: {e}")


def main():
    rclpy.init()
    node = RFReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # 使用下面使用開啟 debug 模式
    # python rf_receiver_node.py --ros-args --log-level DEBUG 
    main()