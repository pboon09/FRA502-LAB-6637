#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

class EndEffectorPublisher(Node):
    def __init__(self):
        super().__init__('end_effector_publisher')

        self.declare_parameter('debug', True)
        self.debug = self.get_parameter('debug').value
        
        L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
        L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
        L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
        tool = SE3(0.28, 0, 0)
        self.robot = rtb.DHRobot([L1, L2, L3], name='RRR_robot', tool=tool)

        self.q = np.zeros(self.robot.n)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/end_effector', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.timer = self.create_timer(0.01, self.publish_pose)

        self.get_logger().info('End Effector Publisher Node started')

    def joint_callback(self, msg):
        self.q = np.array(msg.position)
        if self.debug:
            self.get_logger().info(f"Joint States: {self.q}")

    def publish_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('link_0', 'end_effector', rclpy.time.Time())
            msg = PoseStamped()
            msg.header = trans.header
            msg.header.frame_id = 'link_0'
            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.position.z = trans.transform.translation.z
            msg.pose.orientation = trans.transform.rotation
            self.pub.publish(msg)

            fk_pose = self.robot.fkine(self.q)
            fk_position = fk_pose.t
            fk_orientation = fk_pose.R

            if self.debug:
                self.get_logger().info(f"FK Position: {fk_position}")
                self.get_logger().info(f"TF Position: {trans.transform.translation}")
                self.get_logger().info(f"FK Orientation: {fk_orientation}")
                self.get_logger().info(f"TF Orientation: {trans.transform.rotation}")

                fk_position_array = fk_position.flatten()
                tf_position_array = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                error_pos = np.linalg.norm(fk_position_array - tf_position_array)

                fk_rotation = R.from_matrix(fk_orientation)
                fk_quaternion = fk_rotation.as_quat()

                tf_quaternion = np.array([trans.transform.rotation.x, trans.transform.rotation.y,
                                        trans.transform.rotation.z, trans.transform.rotation.w])

                error_orient = np.linalg.norm(fk_quaternion - tf_quaternion)

                self.get_logger().info(f"Position Error: {error_pos:.2f}")
                self.get_logger().info(f"Orientation Error: {error_orient:.2f}")

        except Exception as e:
            return

def main():
    rclpy.init()
    node = EndEffectorPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
