#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from cfe_msgs.msg import RobotSimHkTlmt
from cfe_msgs.msg import RobotSimJointCmdt


class JointStateConverter(Node):
    """
    This class receives the joint states from topic /groundsystem/robot_sim/hk_tlm
    (cfe_msgs) and publishes them to /joint_states
    This class also subscribes to "/joint_command", and publishes the said command
    to /ground_system/robot_sim_cmd (cfe_msgs)
    """
    def __init__(self):
        super().__init__('joint_state_convert')


        self.declare_parameter('joint_names',  ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.joint_names = self.get_parameter('joint_names').value
        if len(self.joint_names) != 7:
          self.get_logger().error('Joint names should be of size 7')
          rclpy.shutdown()

        self.topic_name = '/groundsystem/robot_sim_cmd'
        self.js_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.jc_publisher = self.create_publisher(RobotSimJointCmdt, self.topic_name, 10)
        self.js_subscription = self.create_subscription(
            RobotSimHkTlmt,
            '/groundsystem/robot_sim_hk_tlm',
            self.cfs_callback,
            10)

        self.jc_subscription = self.create_subscription(
            JointState,
            'joint_command',
            self.jc_callback,
            10)

        self.js_subscription  # prevent unused variable warning
        self.jc_subscription  # prevent unused variable warning


    def cfs_callback(self, msg):
        self.get_logger().info('Received new cFS joint telemetry', throttle_duration_sec=1.0)
        # self.get_logger().info(str(msg))
        # print(msg)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        # js.header.frame_id = "iss"

        js.name = self.joint_names

        js.position.append(msg.payload.state.joint0)
        js.position.append(msg.payload.state.joint1)
        js.position.append(msg.payload.state.joint2)
        js.position.append(msg.payload.state.joint3)
        js.position.append(msg.payload.state.joint4)
        js.position.append(msg.payload.state.joint5)
        js.position.append(msg.payload.state.joint6)

        # print(js)
        # self.get_logger().info(js)
        self.js_publisher.publish(js)

    def jc_callback(self, msg):
        self.get_logger().info('Received new cFS joint command', throttle_duration_sec=1.0)
        # self.get_logger().info(str(msg))

        cmd = RobotSimJointCmdt()
        cmd.cmd_header.sec.function_code = 1
        cmd.joint0 = msg.position[0]
        cmd.joint1 = msg.position[1]
        cmd.joint2 = msg.position[2]
        cmd.joint3 = msg.position[3]
        cmd.joint4 = msg.position[4]
        cmd.joint5 = msg.position[5]
        cmd.joint6 = msg.position[6]

        self.jc_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    jsc = JointStateConverter()
    rclpy.spin(jsc)

    jsc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
