#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ControlMode, RandomPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from tf_transformations import quaternion_from_euler
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('r_min', 0.020)
        self.declare_parameter('r_max', 0.530)
        self.declare_parameter('z_min', -0.330)
        self.declare_parameter('z_max', 0.730)

        self.r_min = self.get_parameter('r_min').get_parameter_value().double_value
        self.r_max = self.get_parameter('r_max').get_parameter_value().double_value
        self.z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value

        L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
        L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
        L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
        tool = SE3(0.28, 0, 0)
        self.robot = rtb.DHRobot([L1, L2, L3], name='RRR_robot', tool=tool)

        self.q = np.zeros(self.robot.n)
        self.target_pose = None
        self.last_target_pose = None 
        self.control_mode = None

        self.delta_q = 0.0

        self.task_space_velocity = np.zeros(3)

        self.start_time = None
        self.waiting_for_new_pose = False

        self.move = False

        self.hz = 100.0

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.velocity_pub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.mode_srv = self.create_service(ControlMode, '/set_control_mode', self.set_mode_callback)
        self.reset_velocity_srv = self.create_client(Trigger, '/reset_velocity')

        self.create_timer(1.0 / self.hz, self.timer_callback)

        self.random_pose_client = self.create_client(RandomPose, '/random_pose')
        

        self.q = np.radians([0, 0, 90])
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

        elif request.mode == 1:
            self.control_mode = 'TO_WF'
            self.publish_pose()
            self.move = True
            self.get_logger().info(f"Teleoperation Mode: World Frame's Velocity Control")
            response.current_mode = 1
            response.success = True
            response.message = "Teleoperation Mode: World Frame's Velocity Control"

        elif request.mode == 2:
            self.control_mode = 'TO_EF'
            self.publish_pose()
            self.move = True
            self.get_logger().info(f"Teleoperation Mode: End Effector Frame's Velocity Control")
            response.current_mode = 2
            response.success = True
            response.message = "Teleoperation Mode: End Effector Frame's Velocity Control"

        elif request.mode == 3 and not self.move:
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
            
        elif request.mode == 3 and self.move:
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
            self.send_reset_velocity_request()
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
    
    def send_reset_velocity_request(self):
        if not self.reset_velocity_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Reset Velocity service not available")
            return

        request = Trigger.Request()

        future = self.reset_velocity_srv.call_async(request)
        future.add_done_callback(self.handle_reset_velocity_response)

    def handle_reset_velocity_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Velocity reset: {response.message}")
            else:
                self.get_logger().warn(f"Failed to reset velocity: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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

            self.delta_q = np.linalg.pinv(J_trans) @ delta_x

            self.q = self.q + self.delta_q * 1.0 / self.hz

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
        
        elif self.control_mode in ['TO_WF', 'TO_EF'] and self.move:
            if self.task_space_velocity.any():
                J = self.robot.jacob0(self.q)
                J_trans = J[0:3, :]
                
                manipulability_index = np.sqrt(np.linalg.det(J_trans.T @ J_trans))
                
                fk_pose = self.robot.fkine(self.q)
                x, y, z = fk_pose.t.flatten()

                self.get_logger().info(f"Current Position: x={x:.3f}, y={y:.3f}, z={z:.3f}, Manipulability: {manipulability_index:.6f} workspace check: {self.in_workspace(x, y, z)}")    

                if manipulability_index < 5e-3 or not self.in_workspace(x, y, z):
                    self.get_logger().warn("Low manipulability detected! Resetting velocity and backtracking.")
                    self.send_reset_velocity_request()
                    self.delta_q = -self.delta_q

                else:
                    if self.control_mode == 'TO_EF':
                        fk_pose = self.robot.fkine(self.q)
                        rotation_matrix = fk_pose.R 
                        task_space_velocity_ef = rotation_matrix.T @ self.task_space_velocity

                        self.delta_q = np.linalg.pinv(J_trans) @ task_space_velocity_ef
                    else:
                        self.delta_q = np.linalg.pinv(J_trans) @ self.task_space_velocity

                self.q = self.q + self.delta_q * 1.0 / self.hz

            self.publish_joints()
        
        elif self.control_mode == 'Idle':
            self.publish_joints()

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
    
    def velocity_callback(self, msg):
        self.task_space_velocity = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    
    def in_workspace(self, x, y, z):
        rho2 = x**2 + y**2 + 0.07
        return (self.r_min**2 <= rho2 <= self.r_max**2) and (self.z_min <= z <= self.z_max)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
