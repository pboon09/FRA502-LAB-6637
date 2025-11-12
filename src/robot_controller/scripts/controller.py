#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ControlMode, RandomPose
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
        self.target_pose = None
        self.control_mode = None

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.mode_srv = self.create_service(ControlMode, '/set_control_mode', self.set_mode_callback)

        self.create_timer(1.0 / 100.0, self.timer_callback)  # 100Hz timer

        self.random_pose_client = self.create_client(RandomPose, '/random_pose')

        self.publish_joints()
        self.get_logger().info("RobotController started")

    def set_mode_callback(self, request, response):
        self.get_logger().info(f"Received service request: {request.mode}")

        if request.mode == 0:
            self.control_mode = 'IPK'
            self.target_pose = SE3([request.x, request.y, request.z])
            self.get_logger().info(f"IPK Mode: Target position set to ({request.x}, {request.y}, {request.z})")

            success = self.compute_ik_solution(self.target_pose)

            if success:
                self.get_logger().info(f"IK Solution found: {self.q}")
                response.success = True
                response.message = "IK Solution found"
                response.q_solution = self.q.tolist()
            else:
                self.get_logger().warn("No valid IK solution found")
                response.success = False
                response.message = "No valid IK solution found"

        elif request.mode == 1:
            self.control_mode = 'AM'
            self.get_logger().info(f"Auto Mode: Requesting random pose")
            self.request_random_pose(response)
            response.success = True
            response.message = "Auto Mode initiated"
            
        else:
            response.success = False
            response.message = "Invalid mode"
            self.get_logger().warn(f"Invalid mode: {request.mode}")

        return response

    def request_random_pose(self, response):
        if not self.random_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("RandomPose service not available")
            return

        request = RandomPose.Request()
        future = self.random_pose_client.call_async(request)

        future.add_done_callback(self.handle_random_pose_response)

    def handle_random_pose_response(self, future):
        try:
            response = future.result()
            if response:
                random_pose = response.pose.pose
                self.target_pose = SE3(random_pose.position.x, random_pose.position.y, random_pose.position.z)
                self.get_logger().info(f"Random pose received: ({random_pose.position.x}, {random_pose.position.y}, {random_pose.position.z})")
                self.control_mode = 'AM'
                success = self.compute_ik_solution(self.target_pose)
                if success:
                    self.get_logger().info(f"IK Solution for random pose: {self.q}")
        except Exception as e:
            self.get_logger().error(f"Failed to get random pose: {e}")

    def compute_ik_solution(self, target_pose):
        success = False
        for _ in range(5):
            q0 = np.random.uniform(self.robot.qlim[0], self.robot.qlim[1])
            sol = self.robot.ikine_LM(target_pose, q0=q0, mask=[1, 1, 1, 0, 0, 0], tol=1e-5)
            if sol.success:
                self.q = sol.q
                success = True
                break
        return success

    def timer_callback(self):
        if self.control_mode is None:
            self.publish_joints()
            return
        
        if self.control_mode == 'IPK' and self.target_pose is not None:
            self.publish_joints()

        elif self.control_mode == 'AM' and self.target_pose is not None:
            self.publish_joints()

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
