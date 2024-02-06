#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPubplisherNode(Node): 
    def __init__(self):
        super().__init__("number_publisher") 
        self.publishers_ = self.create_publisher(Int64,"number",10)
        self.timers_ = self.create_timer(1,self.publishNumbers)
        self.get_logger().info("Number Publisher has been started.")

    def publishNumbers(self):
        msg = Int64()
        msg.data = 2
        self.publishers_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPubplisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()