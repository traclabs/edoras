#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from cfe_msgs.msg import CanadarmAppHkTlmt


class JointStateConverter(Node):
    """
    This class receives the joint states from topic /groundsystem/canadarm_app_hk_tlm
    (cfe_msgs) and publishes them to /joint_states
    """
    def __init__(self):
        super().__init__('joint_state_convert')


        self.declare_parameter('joint_names',  ["Base_Joint", "Shoulder_Roll", "Shoulder_Yaw", "Elbow_Pitch", "Wrist_Pitch", "Wrist_Yaw", "Wrist_Roll"])
        self.joint_names = self.get_parameter('joint_names').value
        if len(self.joint_names) != 7:
          self.get_logger().error('Joint names should be of size 7')
          rclpy.shutdown()

        self.js_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.js_subscription = self.create_subscription(
            CanadarmAppHkTlmt,
            '/groundsystem/canadarm_app_hk_tlm',
            self.tlm_callback,
            10)

        self.js_subscription  # prevent unused variable warning


    def tlm_callback(self, msg):
        self.get_logger().info('Received new Candarm joint telemetry', throttle_duration_sec=5.0)
        # self.get_logger().info(str(msg))
        # print(msg)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        # js.header.frame_id = "iss"

        js.name = self.joint_names

        js.position.append(msg.payload.state.joint_0)
        js.position.append(msg.payload.state.joint_1)
        js.position.append(msg.payload.state.joint_2)
        js.position.append(msg.payload.state.joint_3)
        js.position.append(msg.payload.state.joint_4)
        js.position.append(msg.payload.state.joint_5)
        js.position.append(msg.payload.state.joint_6)

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
