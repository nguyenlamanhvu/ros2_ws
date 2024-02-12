#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPubplisherNode(Node): 
    def __init__(self):
        super().__init__("number_publisher") 
        #default value of publish_number is 2
        self.declare_parameter("publish_number", 2)
        self.number_ = self.get_parameter("publish_number").value
        #default value of frequency is 1.0
        self.declare_parameter("frequency" , 1.0)
        self.frequecy_ = self.get_parameter("frequency").value

        self.publishers_ = self.create_publisher(Int64,"number",10)
        self.timers_ = self.create_timer(1.0 / self.frequecy_,self.publishNumbers)
        self.get_logger().info("Number Publisher has been started.")

    def publishNumbers(self):
        msg = Int64()
        msg.data = self.number_
        self.publishers_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPubplisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()