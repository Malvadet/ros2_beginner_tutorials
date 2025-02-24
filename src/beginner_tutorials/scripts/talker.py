#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.get_clock().now().to_msg().sec}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
