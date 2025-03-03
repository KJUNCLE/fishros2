import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info("python_node已经运行")
    rclpy.spin(node)
    rclpy.shutdown()

