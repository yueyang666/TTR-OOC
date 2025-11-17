# analyzer_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast

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
            data = ast.literal_eval(msg.data)
            speed = data.get('speed', '無資料')
            battery = data.get('battery', '無資料')
            self.get_logger().info(f"分析結果：速度 = {speed}，電量 = {battery}")
        except Exception as e:
            self.get_logger().error(f"解析錯誤：{e}")


def main():
    rclpy.init()
    node = AnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
