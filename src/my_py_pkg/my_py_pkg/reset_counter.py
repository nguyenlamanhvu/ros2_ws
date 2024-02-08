#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ResetCounterNode(Node): 
    def __init__(self):
        super().__init__("reset_counter") 
        self.call_reset_counter()
    
    def call_reset_counter(self):
        client_ = self.create_client(SetBool,"reset_counter")
        while not client_.wait_for_service(1):
            self.get_logger().warn("Waiting for server number counter.")
        request = SetBool.Request()
        request.data = True

        future = client_.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(self.callback_reset_counter)   #nho gan gia tri
    
    def callback_reset_counter(self,future):
        try:
            response = future.result()
            if(response.success == True):
                self.get_logger().info("Counter has been reset.")
            else:
                self.get_logger().info("Counter hasn't been reset.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()