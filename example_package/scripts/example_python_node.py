#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ExamplePythonNode(Node):
    def __init__(self):
        super().__init__('example_python_node')
        self.get_logger().info("Example Python node has started!")

def main(args=None):
    rclpy.init(args=args)
    node = ExamplePythonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
