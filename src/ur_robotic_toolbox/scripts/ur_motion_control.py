#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_node')
        self.ur5 = rtb.models.UR5()
        
        self.homing = True
        self.homing_gain = 1.0
        self.homing_threshold = 0.01

        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)  
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def joint_state_callback(self, msg: JointState):
        joint_states = np.array(msg.position)
        if joint_states.shape[0] == self.ur5.n:
            self.ur5.q = joint_states
        else:
            self.get_logger().warn(f"Received joint_states of length {joint_states.shape[0]}, expected {self.ur5.n}")

    def timer_callback(self):
        if self.homing:
            q = self.ur5.q
            vel_cmd = np.zeros(self.ur5.n)
            for i in range(self.ur5.n):
                if abs(q[i]) > self.homing_threshold:
                    vel_cmd[i] = -self.homing_gain * q[i]
            self.velocity_publisher.publish(Float64MultiArray(data=vel_cmd.tolist()))
            if np.all(np.abs(q) < self.homing_threshold):
                self.get_logger().info("All joints homed. Starting main control loop.")
                self.homing = False
            return

        Tep = self.ur5.fkine(self.ur5.q) * sm.SE3.Trans(0.0, 0.0, 0.5)
        v, arrived = rtb.p_servo(self.ur5.fkine(self.ur5.q), Tep, 1.0, threshold=0.01)
        if arrived:
            self.get_logger().info("Arrived at target pose.")
            self.velocity_publisher.publish(Float64MultiArray(data=[0.0]*self.ur5.n))
            return

        J = self.ur5.jacob0(self.ur5.q)
        # cond = np.linalg.cond(J)
        # if cond > 1e4:
        #     self.get_logger().warn(f"Jacobian is near singular! Condition number: {cond:.2e}")
        #     return
        J_pinv = np.linalg.pinv(J)
        self.ur5.qd = J_pinv @ v
        self.velocity_publisher.publish(Float64MultiArray(data=self.ur5.qd.tolist()))

def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
