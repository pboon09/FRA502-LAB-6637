#!/usr/bin/python3

# from lookchoop_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.get_logger().info('DummyNode has been started.')   
        self.q_d = np.array([0.707, 0.0, 0.0])
        self.qd_d = np.array([0.0, 0.0, 0.0])
        self.q = np.array([0.0, 0.0, 0.0])
        self.qd = np.array([0.0, 0.0, 0.0])
        self.kp_q = np.array([30.0, 30.0, 30.0])    # Proportional gains for position
        self.ki_q = np.array([1.0, 1.0, 1.0])    # Integral gains for position
        self.kd_q = np.array([1.0, 1.0, 1.0])       # Derivative gains for position
        self.kp_qd = np.array([10.0, 10.0, 10.0])   # Proportional gains for velocity
        self.ki_qd = np.array([10.0, 10.0, 10.0])   # Integral gains for velocity
        self.kd_qd = np.array([1.0, 1.0, 1.0])      # Derivative gains for velocity

        self.create_timer(0.01, self.control_loop)

        self.l_1 = 0.25
        self.l_2 = 0.5
        self.l_3 = 0.5
        self.l_4 = 0.25

        self.p_e = np.array([0.0, 0.0]) # End-effector position (x, z)

    def joint_states_callback(self, msg):
        self.q = np.array(msg.position[:3])
        self.qd = np.array(msg.velocity[:3])
        print(f"Received joint states: q={self.q}, qd={self.qd}")

        self.p_e[0] = self.l_2 * np.sin(self.q[0]) + self.l_3 * np.sin(self.q[0] + self.q[1]) + self.l_4 * np.sin(self.q[0] + self.q[1] + self.q[2])
        self.p_e[1] = - (self.l_1 + self.l_2 * np.cos(self.q[0]) + self.l_3 * np.cos(self.q[0] + self.q[1]) + self.l_4 * np.cos(self.q[0] + self.q[1] + self.q[2]))

        self.get_logger().info(f'End-effector position: x={self.p_e[0]}, z={self.p_e[1]}')

    def control_loop(self):
        error_q = self.q_d - self.q
        error_qd = self.qd_d - self.qd

        tau = (self.kp_q * error_q) + (self.kd_q * error_qd)
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published control torques: {tau}')


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()