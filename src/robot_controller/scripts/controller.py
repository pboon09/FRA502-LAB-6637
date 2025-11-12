#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ControlMode
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
        L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
        L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
        tool = SE3(0.28, 0, 0)
        self.robot = rtb.DHRobot([L1, L2, L3], name='RRR_robot', tool=tool)

        self.q = np.zeros(self.robot.n)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.mode_srv = self.create_service(ControlMode, '/set_control_mode', self.set_mode_callback)

        self.publish_joints()

        self.get_logger().info("RobotController started")

    def set_mode_callback(self, request, response):
        self.get_logger().info(f"Received service request: {request.x}, {request.y}, {request.z}")

        if request.mode != 0:
            response.success = False
            response.message = "Invalid mode"
            return response

        self.target_pose = SE3([request.x, request.y, request.z])
        self.get_logger().info(f"Received target position: ({request.x}, {request.y}, {request.z})")

        success = False
        for _ in range(5):
            q0 = np.random.uniform(self.robot.qlim[0], self.robot.qlim[1])
            sol = self.robot.ikine_LM(self.target_pose, q0=q0, mask=[1, 1, 1, 0, 0, 0], tol=1e-5)
            if sol.success:
                self.q = sol.q
                success = True
                break

        if success:
            self.get_logger().info(f"IK Solution found: {self.q}")
            response.success = True
            response.message = "IK Solution found"
            response.q_solution = self.q.tolist()
            self.publish_joints()
        else:
            self.get_logger().warn("No valid IK solution found")
            response.success = False
            response.message = "No valid IK solution found"

        return response

    def publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['base_to_link1', 'link1_link2', 'link2_link3']
        js.position = self.q.tolist()
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
