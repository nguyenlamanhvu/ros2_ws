# No OOP
"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client")

    client = node.create_client(AddTwoInts,"add_two_ints")
    while not client.wait_for_service(1.0):            
        node.get_logger().warn("Waiting for server add two ints...")
    #After server had been stared
    request = AddTwoInts.Request()      #AddTwoInts: class
    request.a = 8
    request.b = 10 
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node,future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + ' + ' + str(request.b) + ' = ' + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r" %(e,))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
"""

#OOP
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial           #allow us to add more arguments to the callback

class AddTwoIntsClientNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_client") 
        self.call_add_two_ints(8,10)

    def call_add_two_ints(self,a,b):
        client = self.create_client(AddTwoInts,"add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server add two ints...")
        request = AddTwoInts.Request()      #init request from service type
        request.a = a
        request.b = b
        future = client.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(partial(self.callback_add_two_ints,a=a,b=b))   #nho gan gia tri
        
    def callback_add_two_ints(self,future,a,b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + ' + ' + str(b) + ' = ' + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
