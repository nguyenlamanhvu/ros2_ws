#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery") 
        self.battery_state = "full"
        #get init time
        self.last_time_battery_state_changed_ = self.get_current_time()
        self.timer_ = self.create_timer(0.1,self.check_led_state)
        self.get_logger().info("Battery has been started.")
    
    def get_current_time(self):
        secs,nanosecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nanosecs/1000000000.0
    
    def check_led_state(self):
        clock = self.get_current_time()
        if self.battery_state == "full":
            if clock - self.last_time_battery_state_changed_ > 4.0 :
                self.get_logger().info("Battery is empty! Charging now...")
                self.last_time_battery_state_changed_ = clock
                self.battery_state = "empty"    #change battery full to empty state
                self.call_battery_client(3,True)
        else:
            if clock - self.last_time_battery_state_changed_ > 6.0:
                self.get_logger().info("Battery is full! Stop charging now...")
                self.last_time_battery_state_changed_ = clock
                self.battery_state = "full"     #change battery empty to full state
                self.call_battery_client(3,False)

    def call_battery_client(self,led_number,state):
        client_ = self.create_client(SetLed,"set_led")
        while not client_.wait_for_service(1) :
            self.get_logger().warn("Waiting for set led service...")
        request = SetLed.Request()          #init request from service type
        request.led_number = led_number
        request.state = state

        future = client_.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(self.callback_battery_client)   #nho gan gia tri

    def callback_battery_client(self,future):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()