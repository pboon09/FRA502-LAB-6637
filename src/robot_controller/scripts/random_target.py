#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import RandomPose
import numpy as np
from tf_transformations import quaternion_from_euler

class RandomPoseNode(Node):
    def __init__(self):
        super().__init__('random_pose_node')

        self.declare_parameter('rho_min', 0.0200)
        self.declare_parameter('rho_max', 0.5304)
        self.declare_parameter('z_center', 0.2000)
        self.declare_parameter('z_radius', 0.5300)

        self.rho_min = self.get_parameter('rho_min').get_parameter_value().double_value
        self.rho_max = self.get_parameter('rho_max').get_parameter_value().double_value
        self.z_center = self.get_parameter('z_center').get_parameter_value().double_value
        self.z_radius = self.get_parameter('z_radius').get_parameter_value().double_value

        self.x_min = -self.rho_max
        self.x_max = self.rho_max
        self.y_min = -self.rho_max
        self.y_max = self.rho_max
        self.z_min = self.z_center - self.z_radius
        self.z_max = self.z_center + self.z_radius

        self.srv = self.create_service(RandomPose, '/random_pose', self.handle_request)

        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)

        self.get_logger().info('Random Pose Node started with workspace equation filter')

    def in_workspace(self, x, y, z):
        rho = np.sqrt(x**2 + y**2)
        
        if rho < self.rho_min:
            return False
        
        rho_norm = (rho - self.rho_min) / (self.rho_max - self.rho_min)
        z_norm = (z - self.z_center) / self.z_radius
        return (rho_norm**2 + z_norm**2) <= 1.0

    def random_point_in_workspace(self):
        while True:
            x = np.random.uniform(self.x_min, self.x_max)
            y = np.random.uniform(self.y_min, self.y_max)
            z = np.random.uniform(self.z_min, self.z_max)
            if self.in_workspace(x, y, z):
                return np.array([x, y, z])

    def handle_request(self, request, response):
        p = self.random_point_in_workspace()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_0'
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])

        q = quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        response.pose = msg
        self.get_logger().info(f"Random pose generated: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")

        self.target_pub.publish(msg)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
