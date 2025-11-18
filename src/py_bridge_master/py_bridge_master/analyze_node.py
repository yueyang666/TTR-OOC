import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AnalyzerNode(Node):
    def __init__(self):
        super().__init__('analyzer_node')
        self.subscription = self.create_subscription(
            String,
            '/vehicle/status',
            self.listener_callback,
            10
        )
        self.get_logger().info("AnalyzerNode 已啟動，訂閱 /vehicle/status")

    def listener_callback(self, msg):
        try:
            # 改用 json.loads() 解析資料
            data = json.loads(msg.data)
            pressure = data.get('PADS_Pressure', '無資料')
            temp = data.get('PADS_Temp', '無資料')
            self.get_logger().info(f"分析結果：壓力 = {pressure}，電量 = {temp}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 解析錯誤：{e}")
        except Exception as e:
            self.get_logger().error(f"未知錯誤：{e}")


def main():
    rclpy.init()
    node = AnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()