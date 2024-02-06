#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounterNode(Node): 
    def __init__(self):
        super().__init__("number_counter") 
        self.counter = Int64()
        self.counter.data = 0
        self.subscriber_ = self.create_subscription(Int64,"number",self.callback_number,10)
        self.publishers_ = self.create_publisher(Int64,"number_counter",10)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self,msg):
        self.get_logger().info(str(self.counter.data))
        self.publishers_.publish(self.counter)
        self.counter.data+=msg.data


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()