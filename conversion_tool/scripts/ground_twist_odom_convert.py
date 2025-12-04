#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cfe_msgs.msg import RoverAppHkTlmt
from cfe_msgs.msg import RoverAppTwistCmdt


class TwistOdomConverter(Node):
    """
    This class receives the Odometry information from topic /groundsystem/rover_app_hk_tlm
    (cfe_msgs) and publishes them to /odom
    This class also subscribes to "/cmd_vel", and publishes the said command
    to /groundsystem/rover_app_cmd (cfe_msgs)
    """
    def __init__(self):
        super().__init__('twist_convert')

        self.declare_parameter('twist_cmd_cfs',  "rover_app_cmd")
        self.twist_cmd_cfs = self.get_parameter('twist_cmd_cfs').value

        self.declare_parameter('odom_tlm_cfs',  "rover_app_hk_tlm")
        self.odom_tlm_cfs = self.get_parameter('odom_tlm_cfs').value


        self.declare_parameter('odom_out',  "/odom")
        self.odom_out = self.get_parameter('odom_out').value

        self.declare_parameter('twist_in',  "/cmd_vel")
        self.twist_in = self.get_parameter('twist_in').value


        self.odom_subscription = self.create_subscription(
            RoverAppHkTlmt,
            '/groundsystem/' + self.odom_tlm_cfs,
            self.cfs_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_out, 10)

        self.twist_subscription = self.create_subscription(
            Twist,
            self.twist_in,
            self.twist_callback,
            10)
        self.twist_publisher = self.create_publisher(RoverAppTwistCmdt, '/groundsystem/' + self.twist_cmd_cfs, 10)


        self.odom_subscription  # prevent unused variable warning
        self.twist_subscription  # prevent unused variable warning


    def cfs_callback(self, msg):
        #self.get_logger().info('Received new cFS odom telemetry', throttle_duration_sec=5.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "rover"
        odom.pose.pose.position.x = msg.payload.state.pose.x
        odom.pose.pose.position.y = msg.payload.state.pose.y
        odom.pose.pose.position.z = msg.payload.state.pose.z
        odom.pose.pose.orientation.x = msg.payload.state.pose.qx
        odom.pose.pose.orientation.y = msg.payload.state.pose.qy
        odom.pose.pose.orientation.z = msg.payload.state.pose.qz
        odom.pose.pose.orientation.w = msg.payload.state.pose.qw                        

        self.odom_publisher.publish(odom)

    def twist_callback(self, msg):
        #self.get_logger().info('Received new twist command to send as cFS RoverAppTwist command', throttle_duration_sec=5.0)

        cmd = RoverAppTwistCmdt()
        cmd.cmd_header.sec.function_code = 1 # CC code. 1 = Command. 0 = Noop
        cmd.twist.linear_x = msg.linear.x
        cmd.twist.linear_y = msg.linear.y
        cmd.twist.linear_z = msg.linear.z
        cmd.twist.angular_x = msg.angular.x
        cmd.twist.angular_y = msg.angular.y
        cmd.twist.angular_z = msg.angular.z

        self.twist_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    toc = TwistOdomConverter()
    rclpy.spin(toc)

    toc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
