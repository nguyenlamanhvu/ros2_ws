#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial           #allow us to add more arguments to the callback
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turtle_spawner") 
        self.declare_parameter("spawn_frequency",2)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.declare_parameter("turtle_name_prefix","turtle")
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value

        self.counter_ = 1       #the first turtle is turtle1 => spawn turtle must be turtle2,...
        self.alive_turtle = []    #init array which isn't contain turtle
        self.turtle_array_publisher_ = self.create_publisher(TurtleArray,"alive_turtle",10)
        self.catch_turtle_service_ = self.create_service(CatchTurtle,"catch_turtle"
                                                         ,self.callback_catch_turtle_service)
        
        #spawn turtle every 0.5 seconds
        self.timer_ = self.create_timer(1.0/self.spawn_frequency,self.call_spawn_turtle)
        self.get_logger().info("Turtle spawner has been started.")

    def publish_alive_turtle(self):
        publish_turtle_info = TurtleArray()
        publish_turtle_info.turtles = self.alive_turtle
        self.turtle_array_publisher_.publish(publish_turtle_info)

    def call_spawn_turtle(self):
        self.counter_+=1
        spawn_x_ = random.uniform(0.0,11.0)    #Return a random floating number between two numbers both inclusive
        spawn_y_ = random.uniform(0.0,11.0)
        spawn_theta_ = random.uniform(0.0,2*math.pi)       
        turtle_name_ = self.turtle_name_prefix + str(self.counter_)
        self.call_spawn_server(spawn_x_,spawn_y_,spawn_theta_,turtle_name_)

    def call_spawn_server(self,x,y,theta,name):
        spawn_client_ = self.create_client(Spawn,"spawn")
        while not spawn_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for spawn server...")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = spawn_client_.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(partial(self.callback_spawn_server,x=x,y=y,theta=theta))   #nho gan gia tri

    def callback_spawn_server(self,future,x,y,theta):
        try:
            response = future.result()
            # new turtle was created if response.name != "" (empty)
            # transfer new turtle info to turtle_controller
            if response.name != "":
                self.get_logger().info(response.name)       
                turtle_info = Turtle()
                turtle_info.x = x
                turtle_info.y = y
                turtle_info.theta = theta
                turtle_info.name = response.name
                self.alive_turtle.append(turtle_info)
                self.publish_alive_turtle()
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

    def callback_catch_turtle_service(self,request,response):
        self.call_kill_turtle(request.name)
        response.success = True
        return response
                
    def call_kill_turtle(self,turtle_name):
        kill_turtle_client_ =  self.create_client(Kill,"kill")
        while not kill_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server add two ints...")
        request = Kill.Request()      #init request from service type
        request.name = turtle_name
        future = kill_turtle_client_.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(partial(self.callback_kill_turtle,turtle_name = turtle_name))   #nho gan gia tri
    
    def callback_kill_turtle(self,future,turtle_name):
        try:
            future.result()
            for (i,turtle) in enumerate(self.alive_turtle):
                if turtle.name == turtle_name:
                    del self.alive_turtle[i]
                    self.publish_alive_turtle()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()