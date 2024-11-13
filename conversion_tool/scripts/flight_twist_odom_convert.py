#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cfe_msgs.msg import RoverAppTlmRobotCommandt
from cfe_msgs.msg import RoverAppCmdRobotStatet


class TwistOdomConverter(Node):
    """
    This class receives the Odometry information from topic /groundsystem/rover_app_hk_tlm
    (cfe_msgs) and publishes them to /odom
    This class also subscribes to "/cmd_vel", and publishes the said command
    to /groundsystem/rover_app_cmd (cfe_msgs)
    """
    def __init__(self):
        super().__init__('twist_convert')

        self.declare_parameter('twist_out_cfs',  "rover_app_send_robot_command")
        self.twist_out_cfs = self.get_parameter('twist_out_cfs').value

        self.declare_parameter('odom_in_cfs',  "rover_app_get_robot_odom")
        self.odom_in_cfs = self.get_parameter('odom_in_cfs').value


        self.declare_parameter('odom_in',  "/w200_0000/platform/odom")
        self.odom_in = self.get_parameter('odom_in').value

        self.declare_parameter('twist_out',  "/w200_0000/cmd_vel")
        self.twist_out = self.get_parameter('twist_out').value

        # Subscribe to twist from cfs
        self.twist_subscription = self.create_subscription(
            RoverAppTlmRobotCommandt,
            '/flightsystem/' + self.twist_out_cfs,
            self.cfs_callback,
            10)
        self.twist_publisher = self.create_publisher(Twist, self.twist_out, 10)

        # Subscribe to odom from robot
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_in,
            self.odom_callback,
            10)
        # And publish it to a topic that cFS reads    
        self.odom_publisher = self.create_publisher(RoverAppCmdRobotStatet, '/flightsystem/' + self.odom_in_cfs, 10)


        self.odom_subscription  # prevent unused variable warning
        self.twist_subscription  # prevent unused variable warning


    def cfs_callback(self, msg):
        self.get_logger().info('Received new cFS twist message to send to robot', throttle_duration_sec=1.0)
        # self.get_logger().info(str(msg))
        # print(msg)
        twist = Twist()
        #odom.header.stamp = self.get_clock().now().to_msg()
        twist.linear.x = msg.twist.linear_x
        twist.linear.y = msg.twist.linear_y
        twist.linear.z = msg.twist.linear_z
        twist.angular.x = msg.twist.angular_x
        twist.angular.y = msg.twist.angular_y
        twist.angular.z = msg.twist.angular_z                    
                        
        self.twist_publisher.publish(twist)

    def odom_callback(self, msg):
        self.get_logger().info('Received new odom msg from robot to send to cFS', throttle_duration_sec=1.0)
        # self.get_logger().info(str(msg))

        cmd = RoverAppCmdRobotStatet()
        cmd.cmd_header.sec.function_code = 1 # CC code. 1 = Command. 0 = Noop
        cmd.odom.pose.x = msg.pose.pose.position.x
        cmd.odom.pose.y = msg.pose.pose.position.y
        cmd.odom.pose.z = msg.pose.pose.position.z
        cmd.odom.pose.qx = msg.pose.pose.orientation.x
        cmd.odom.pose.qy = msg.pose.pose.orientation.y
        cmd.odom.pose.qz = msg.pose.pose.orientation.z
        cmd.odom.pose.qw = msg.pose.pose.orientation.w                
        
                
        self.odom_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    toc = TwistOdomConverter()
    rclpy.spin(toc)

    toc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
