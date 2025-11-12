#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ControlMode, RandomPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from tf_transformations import quaternion_from_euler
import time

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
        self.last_target_pose = None 
        self.control_mode = None

        self.start_time = None
        self.waiting_for_new_pose = False

        self.move = False

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.mode_srv = self.create_service(ControlMode, '/set_control_mode', self.set_mode_callback)

        self.create_timer(1.0 / 100.0, self.timer_callback)

        self.random_pose_client = self.create_client(RandomPose, '/random_pose')
        

        self.q = np.radians([0, 90, 90])
        self.publish_joints()
        self.get_logger().info("RobotController started")

    def set_mode_callback(self, request, response):
        self.get_logger().info(f"Received service request: {request.mode}")

        if request.mode == 0:
            self.control_mode = 'IPK'
            self.target_pose = SE3([request.x, request.y, request.z])
            self.publish_pose(request.x, request.y, request.z)
            self.get_logger().info(f"IPK Mode: Target position set to ({request.x}, {request.y}, {request.z})")

            success = self.compute_ik_solution(self.target_pose)
            
            response.current_mode = 0

            if success:
                self.get_logger().info(f"IK Solution found: {self.q}")
                response.success = True
                response.message = "IK Solution found"
                response.q_solution = self.q.tolist()
            else:
                self.get_logger().warn("No valid IK solution found")
                response.success = False
                response.message = "No valid IK solution found"

        elif request.mode == 2 and not self.move:
            self.control_mode = 'AM'
            self.move = True
            if self.last_target_pose is not None:
                self.target_pose = self.last_target_pose
                self.get_logger().info(f"Auto Mode: Resuming last target pose")
                response.message = "Auto Mode Resuming"
            else:
                self.request_random_pose()
                self.get_logger().info(f"Auto Mode: Requesting random pose")
                response.message = "Auto Mode Requesting"
            response.current_mode = 1
            response.success = True
            
        elif request.mode == 2 and self.move:
            self.control_mode = 'AM'
            self.move = False
            self.last_target_pose = self.target_pose
            self.get_logger().info(f"Auto Mode: Stop Moving random pose")
            response.current_mode = 1
            response.success = True
            response.message = "Auto Mode Disabled"

        else:
            self.last_target_pose = None
            self.move = False
            self.publish_pose()
            response.current_mode = request.mode
            response.success = False
            response.message = "Idle mode"
            self.get_logger().warn(f"Idle mode: {request.mode}")

        return response

    def request_random_pose(self):
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

        elif self.control_mode == 'AM' and self.target_pose is not None and self.move:
            fk_pose = self.robot.fkine(self.q)
            current_position = fk_pose.t.flatten()
            target_position = self.target_pose.t.flatten()

            delta_x = target_position - current_position 
            
            J = self.robot.jacob0(self.q)
            J_trans = J[0:3, :]

            delta_q = np.linalg.pinv(J_trans) @ delta_x

            self.q = self.q + delta_q * 0.01

            self.publish_joints()

            position_error = np.linalg.norm(delta_x)
            position_threshold = 0.01

            if position_error < position_threshold:
                if not self.waiting_for_new_pose:
                    self.start_time = time.time()
                    self.get_logger().info(f"Target position reached. Error: {position_error}")
                    self.waiting_for_new_pose = True

            if self.waiting_for_new_pose:
                elapsed_time = time.time() - self.start_time

                if elapsed_time >= 1.0:
                    self.request_random_pose()

                    self.waiting_for_new_pose = False  

    def publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['base_to_link1', 'link1_link2', 'link2_link3']
        js.position = self.q.tolist()
        self.joint_pub.publish(js)
    
    def publish_pose(self, x = 0.0, y = 0.0, z = 0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_0'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        q = quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
