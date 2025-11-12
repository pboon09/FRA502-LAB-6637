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

        self.declare_parameter('r_min', 0.020)
        self.declare_parameter('r_max', 0.251)
        self.declare_parameter('z_min', 0.002)
        self.declare_parameter('z_max', 0.450)

        self.r_min = self.get_parameter('r_min').get_parameter_value().double_value
        self.r_max = self.get_parameter('r_max').get_parameter_value().double_value
        self.z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value


        self.x_min = -self.r_max
        self.x_max = self.r_max
        self.y_min = -self.r_max
        self.y_max = self.r_max

        self.srv = self.create_service(RandomPose, '/random_pose', self.handle_request)
        self.get_logger().info('Random Pose Node started with workspace equation filter')

    def in_workspace(self, x, y, z):
        rho2 = x**2 + y**2
        return (self.r_min**2 <= rho2 <= self.r_max**2) and (self.z_min <= z <= self.z_max)

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
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
