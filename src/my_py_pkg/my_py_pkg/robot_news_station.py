#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node): 
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name_ = "C3PO"
        self.publishers_ = self.create_publisher(String,"robot_news",10)        #(msg_type,topic_name,buffer)
        self.timer_ = self.create_timer(0.5,self.publish_news)
        self.get_logger().info("Robot news has been started")

    def publish_news(self):
        msg = String()      #String object
        msg.data = "Hi, this is " + self.robot_name_ + " from the robot news station"
        self.publishers_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()