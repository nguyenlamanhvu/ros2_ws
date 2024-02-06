#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter     #add Node class

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0;
        self.get_logger().info("Hello ROS2")
        self.create_timer(0.5,self.timer_callback)      #after 0.5s, process call timer_callback
    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)       #first command of ros2
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()            #last command of ros2

if __name__ == "__main__":
    main()