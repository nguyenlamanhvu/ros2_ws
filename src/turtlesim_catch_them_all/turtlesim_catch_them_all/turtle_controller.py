#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial           #allow us to add more arguments to the callback


class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.declare_parameter("catch_closet_turtle_first",True)
        self.catch_closet_turtle_first = self.get_parameter("catch_closet_turtle_first").value

        self.pose_ = None
        self.alive_turle = None         #catch turtle
        self.turtlesim_publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.turtlesim_subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.callback_turtle_pose,10)
        self.turtle_spawner_subscriber_ = self.create_subscription(TurtleArray,"alive_turtle"
                                                                   ,self.callback_alive_turtle_subsciber,10)
        self.timer_ = self.create_timer(0.01,self.callback_turtle_velocity)
        
        self.get_logger().info("Turtle controller has been started.")

    def callback_turtle_pose(self,msg):
        self.pose_ = msg

    def callback_turtle_velocity(self):
        if(self.pose_ == None) or self.alive_turle == None:
            return   

        msg = Twist()           
        distance_x = self.alive_turle.x - self.pose_.x
        distance_y = self.alive_turle.y - self.pose_.y
        #eucliden distance
        distance = math.sqrt(distance_x**2 + distance_y**2) 
        
        #use angular.z to modify direction of turtle
        #use linear.x to move
        #angular.z > 0: clockwise
        #angular.z < 0: counter clockwise

        if distance > 0.5:      # distance error <= 0.5
            #angle to goal
            angle_to_goal = math.atan2(distance_y,distance_x)
            #orientation
            diff_angle = angle_to_goal - self.pose_.theta

            if diff_angle > math.pi:
                diff_angle -= 2*math.pi
            elif diff_angle < -math.pi:
                diff_angle += 2*math.pi

            msg.linear.x = 2*distance
            msg.angular.z = 6*diff_angle     
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_client(self.alive_turle.name)
            self.alive_turle = None
        
        self.turtlesim_publisher_.publish(msg)

    def callback_alive_turtle_subsciber(self,msg):
        if(len(msg.turtles) > 0):       #have turtles in array
            if self.catch_closet_turtle_first:       #closet mode
                closet_turtle = None
                closet_distance = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closet_turtle == None or closet_distance > distance:
                        closet_turtle = turtle
                        closet_distance = distance
                self.alive_turle = closet_turtle
            else:                               #normal mode
                self.alive_turle = msg.turtles[0]

    def call_catch_turtle_client(self,turtle_name):
        catch_turtle_client_ =  self.create_client(CatchTurtle,"catch_turtle")
        while not catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server add two ints...")
        request = CatchTurtle.Request()      #init request from service type
        request.name = turtle_name
        future = catch_turtle_client_.call_async(request)
        # add callback_add_two_ints to be executed when client.call_async(request) is done
        future.add_done_callback(partial(self.callback_catch_turtle,turtle_name = turtle_name))   #nho gan gia tri
    
    def callback_catch_turtle(self,future,turtle_name):
        try:
            response = future.result()
            if response.success == True:
                self.get_logger().info("Caught " + turtle_name)
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))        


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()