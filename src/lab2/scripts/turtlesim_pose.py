#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class turtlesim_pose(Node):
    def __init__(self):
        super().__init__('odom_pub')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose_2_callback, 10)

        self.pub_odom1 = self.create_publisher(Odometry , '/odom1', 10)
        self.pub_odom2 = self.create_publisher(Odometry , '/odom2', 10)

        self.get_logger().info("turtlesim_pose start")
    
    def pose_1_callback(self, msg):
        if msg is None:
            return
        self.pub_odom(msg, 'turtle1', 'turtle1')

    def pose_2_callback(self, msg):
        if msg is None:
            return
        self.pub_odom(msg, 'turtle2', 'turtle2')

    def pub_odom(self, msg, turtle_name, child_frame_id):
        robot_x = msg.x - 5.44
        robot_y = msg.y - 5.44
        robot_theta = msg.theta
        q = quaternion_from_euler(0.0 ,0.0, robot_theta)

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = child_frame_id
    
        odom.pose.pose.position.x = robot_x
        odom.pose.pose.position.y = robot_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        if turtle_name == 'turtle1':
            self.pub_odom1.publish(odom)
        else:
            self.pub_odom2.publish(odom)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = child_frame_id
        t.transform.translation.x = robot_x
        t.transform.translation.y = robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        return


def main(args=None):
    rclpy.init(args=args)
    node = turtlesim_pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
