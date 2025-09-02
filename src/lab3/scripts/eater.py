#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from controller_interfaces.srv import SetMaxPizza, SetParam

import math

class EATER(Node):
    def __init__(self):
        super().__init__('eater_node')
        self.name_space = self.get_namespace()
        # self.name_space = "eater_turtle"

        self.robot_pose = None
        self.waypoint = []

        self.pizza_max = 5
        self.pizza_spawn_count = 0
        self.pizza_count = 0

        self.kp_linear = 10.0
        self.kp_angular = 20.0

        self.max_linear = 10.0
        self.max_angular = 20.0

        self.declare_parameter('sampling_frequency', 100.0)

        self.freq = self.get_parameter('sampling_frequency').value

        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name_space}/cmd_vel', 10)
        self.eat_pub = self.create_publisher(Bool, f'{self.name_space}/eat_status', 10)

        self.create_subscription(Point, '/mouse_position', self.mouse_callback, 10)

        self.create_subscription(Pose, f'{self.name_space}/pose', self.pose_callback, 10)

        self.create_subscription(Int64, f'{self.name_space}/pizza_count', self.pizza_count_callback,10)

        self.create_service(SetMaxPizza, f'{self.name_space}/set_max_pizza', self.set_max_pizza_callback)
        self.create_service(SetParam, f'{self.name_space}/set_controller_param', self.set_controller_param_callback)

        self.create_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, f'{self.name_space}/eat')

        self.create_timer(1.0/self.freq, self.timer_callback)

        self.get_logger().info(f"eater start with ns {self.name_space} and freq at {self.freq}")
    
    def set_controller_param_callback(self, request, response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        self.get_logger().info(f"Current: kp_linear {self.kp_linear} kp_angular {self.kp_angular}")
        return response

    def set_max_pizza_callback(self, request, response):
        if self.pizza_max <= request.max_pizza.data:
            self.pizza_max = request.max_pizza.data
            # response.log.data = f"SetMaxPizza to {self.pizza_max}. You can spawn {self.pizza_max - self.pizza_count} times!"
            response.log.data = "Success"
            self.get_logger().info(f"SetMaxPizza to {self.pizza_max}. You can spawn {self.pizza_max - self.pizza_count} times!")
        else:
            response.log.data = "Failed"
            # response.log.data = f"Failed: MaxPizza is less than current count!"
        return response
    
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
            self.get_logger().info(f"You can spawn {self.pizza_max - self.pizza_spawn_count} left!, Pizza Max {self.pizza_max}")

    def mouse_callback(self, msg):
        waypoint = [msg.x, msg.y]
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
    
    def go_eat(self, state):
        data = Bool()
        data.data = state
        self.eat_pub.publish(data)
    
    def timer_callback(self):
        if self.robot_pose is None:
            self.get_logger().info("Wait for pose")
            return
        if len(self.waypoint) == 0:
            self.pub_vel(0.0,0.0)

            if self.pizza_count == self.pizza_max:
                self.go_eat(True)
            elif self.pizza_count < self.pizza_max:
                self.go_eat(False)

            return
        
        delta_x = self.waypoint[0][0] - self.robot_pose[0]
        delta_y = self.waypoint[0][1] - self.robot_pose[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)

        goal_theta = math.atan2(delta_y , delta_x)
        error_theta = goal_theta - self.robot_pose[2]
        theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

        vx = min(self.max_linear , max(-self.max_linear, distance * self.kp_linear))
        wz = min(self.max_angular , max(-self.max_angular, theta * self.kp_angular))

        self.pub_vel(vx, wz)

        if distance < 0.1:
            self.pub_vel(0.0,0.0)
            if self.pizza_count != self.pizza_max:
                self.waypoint.pop(0)
                if self.pizza_spawn_count <= self.pizza_max:
                    self.eat_pizza()
        
        if self.pizza_count == self.pizza_max:
            self.go_eat(True)
        elif self.pizza_count < self.pizza_max:
            self.go_eat(False)

        return

def main(args=None):
    rclpy.init(args=args)
    node = EATER()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
