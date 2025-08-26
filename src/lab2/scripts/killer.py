#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from turtlesim.srv import Kill, Spawn

import math

class KILLER(Node):
    def __init__(self):
        super().__init__('killer')
        self.robot_pose1 = None
        self.robot_pose2 = None

        self.pizza_count = 0
        self.pizza_max = 5

        self.is_kill = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)

        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)

        self.kill_client = self.create_client(Kill, '/remove_turtle')
        self.spawn_client = self.create_client(Spawn, '/spawn_turtle')

        self.spawn_turtle()

        self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("eater start")

    def spawn_turtle(self):
        req = Spawn.Request()
        req.x = 5.44
        req.y = 5.44
        req.theta = 0.0
        req.name = 'turtle2'
        self.spawn_client.call_async(req)
    
    def kill_turtle(self):
        req = Kill.Request()
        req.name = 'turtle1'
        self.kill_client.call_async(req)

    def pose1_callback(self, msg):
        self.robot_pose1 = [msg.x, msg.y, msg.theta]

    def pose2_callback(self, msg):
        self.robot_pose2 = [msg.x, msg.y, msg.theta]
    
    def pizza_count_callback(self, msg):
        self.pizza_count = msg.data
    
    def pub_vel(self, vx, wz):
        data = Twist()
        data.linear.x = vx
        data.angular.z = wz
        self.cmd_vel_pub.publish(data)
    
    def timer_callback(self):
        if self.robot_pose1 is None or self.robot_pose2 is None:
            self.get_logger().info("Wait for pose")
            return
        
        print(self.pizza_count , self.pizza_max)

        if self.pizza_count == self.pizza_max and self.pizza_count >= 0 and self.pizza_max >= 0 and self.is_kill is not True:
            delta_x = self.robot_pose1[0] - self.robot_pose2[0]
            delta_y = self.robot_pose1[1] - self.robot_pose2[1]
            distance = math.sqrt(delta_x**2 + delta_y**2)

            goal_theta = math.atan2(delta_y , delta_x)
            error_theta = goal_theta - self.robot_pose2[2]
            theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            
            linear_gain = 10.00
            angular_gain = 10.0

            max_linear = 3.0
            max_angular = 20.0

            vx = min(max_linear , max(-max_linear, distance * linear_gain))
            wz = min(max_angular , max(-max_angular, theta * angular_gain))

            self.pub_vel(vx, wz)

            if distance < 0.05:
                self.kill_turtle()
                self.pub_vel(0.0,0.0)
                self.is_kill = True
                return
        
        if self.is_kill:
            self.pub_vel(0.0,0.0)
    


def main(args=None):
    rclpy.init(args=args)
    node = KILLER()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
