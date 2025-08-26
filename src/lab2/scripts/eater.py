#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

import math

class EATER(Node):
    def __init__(self):
        super().__init__('eater_node')
        self.robot_pose = None
        self.waypoint = []

        self.pizza_max = 5
        self.pizza_spawn_count = 0
        self.pizza_count = 0

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(Point, '/mouse_position', self.mouse_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_callback, 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback,10)

        self.create_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

        self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("eater start")
    
    def pizza_count_callback(self, msg):
        self.pizza_count = msg.data

    def eat_pizza(self):
        req = Empty.Request()
        self.eat_pizza_client.call_async(req)
    
    def spawn_pizza(self, x, y):
        req = GivePosition.Request()
        req.x = x
        req.y = y
        if self.pizza_spawn_count < self.pizza_max:
            self.create_pizza_client.call_async(req)
            self.pizza_spawn_count += 1

    def mouse_callback(self, msg):
        waypoint = [msg.x, msg.y]
        self.spawn_pizza(waypoint[0], waypoint[1])
        if self.pizza_count == self.pizza_max and self.pizza_spawn_count >= self.pizza_max:
            self.waypoint = [waypoint]
        else:
            self.waypoint.append(waypoint)

    def rviz_callback(self, msg):
        waypoint = [msg.pose.position.x + 5.44, msg.pose.position.y + 5.44]
        self.spawn_pizza(waypoint[0], waypoint[1])
        if self.pizza_count == self.pizza_max and self.pizza_spawn_count >= self.pizza_max:
            self.waypoint = [waypoint]
        else:
            self.waypoint.append(waypoint)

    
    def pose_callback(self, msg):
        self.robot_pose = [msg.x, msg.y, msg.theta]
    
    def pub_vel(self, vx, wz):
        data = Twist()
        data.linear.x = vx
        data.angular.z = wz
        self.cmd_vel_pub.publish(data)
    
    def timer_callback(self):
        if self.robot_pose is None:
            self.get_logger().info("Wait for pose")
            return
        if len(self.waypoint) == 0:
            self.pub_vel(0.0,0.0)
            return
        
        delta_x = self.waypoint[0][0] - self.robot_pose[0]
        delta_y = self.waypoint[0][1] - self.robot_pose[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)

        goal_theta = math.atan2(delta_y , delta_x)
        error_theta = goal_theta - self.robot_pose[2]
        theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

        linear_gain = 10.0
        angular_gain = 20.0

        max_linear = 10.0
        max_angular = 20.0

        vx = min(max_linear , max(-max_linear, distance * linear_gain))
        wz = min(max_angular , max(-max_angular, theta * angular_gain))

        self.pub_vel(vx, wz)

        if distance < 0.1 and self.pizza_count != self.pizza_max:
            self.pub_vel(0.0,0.0)
            self.waypoint.pop(0)
            if self.pizza_spawn_count <= self.pizza_max:
                self.eat_pizza()

        # print(self.pizza_max)
        return

def main(args=None):
    rclpy.init(args=args)
    node = EATER()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
