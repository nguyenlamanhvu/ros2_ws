#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedState
from my_robot_interfaces.srv import SetLed

class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states",[0,0,0])
        self.led_states = self.get_parameter("led_states").value
        
        self.led_panel_publisher_ = self.create_publisher(LedState,"led_panel_state",10)
        self.timer_ = self.create_timer(4,self.led_panel_publish)
        self.server_ = self.create_service(SetLed,"set_led",self.callback_set_led)
        self.get_logger().info("Led panel has been started.")

    def led_panel_publish(self):
        msg = LedState()
        msg.ledstatus = self.led_states
        self.led_panel_publisher_.publish(msg)

    def callback_set_led(self,request,response):
        led_number = request.led_number
        state = request.state
        if(led_number > len(self.led_states) or led_number <= 0):
            response.success = False
            return response
        self.led_states[led_number-1] = 1 if state else 0
        response.success = True
        self.led_panel_publish()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()