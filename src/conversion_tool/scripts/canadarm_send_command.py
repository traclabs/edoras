#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Duration

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from cfe_msgs.msg import CanadarmAppRobotStatet, CanadarmAppRobotCommandt, CanadarmAppCmdt

class SendMotionCommand(Node):
    """
    Send motion command
    """
    def __init__(self):
        super().__init__('send_motion_command')
        self.declare_parameter('mode', Parameter.Type.INTEGER)
        
        self._publish_state = self.create_publisher(CanadarmAppCmdt, "/groundsystem/canadarm_app_cmd", 10)

    def send_goal(self):

        mode = self.get_parameter('mode')
        msg = CanadarmAppCmdt()
        msg.cmd_header.sec.function_code = 1
        msg.pose_id = mode.value
        self.get_logger().info("Publishing message in topic for cfs")
        self._publish_state.publish(msg)        
        
#########################
def main(args=None):

    rclpy.init(args=args) 
    action_client = SendMotionCommand()
    action_client.send_goal()
    rclpy.spin(action_client)
    
if __name__ == '__main__':
    main()
