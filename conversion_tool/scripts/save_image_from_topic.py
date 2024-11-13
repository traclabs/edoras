#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Image
from std_msgs.msg import String
import threading

import numpy as np
import cv2
from cv_bridge import CvBridge

# Image stuff
# https://stackoverflow.com/questions/72690021/how-to-process-a-image-message-with-opencv-from-ros2

##########################################
# Create a node
# Every t seconds, it takes a picture and stores it in 
# a given location
mutex = threading.Lock()

class CameraFeed(Node):

    def __init__(self):
    
        super().__init__('camera_feed')

        self.declare_parameter('image_topic', "/image_raw")
        self.declare_parameter('image_folder', "cfdp/rosfsw")
        self.declare_parameter('image_name', "rover_image.png")
        self.declare_parameter('timer_dt', 5.0)
        self.declare_parameter('image_size', 100)

        param_image_topic = self.get_parameter('image_topic')        
        param_image_folder = self.get_parameter('image_folder')
        param_image_name = self.get_parameter('image_name')
        param_timer_dt = self.get_parameter('timer_dt')
        param_image_size = self.get_parameter('image_size')

                
        self.image_filename_ = str(param_image_folder.value) + str("/") + str(param_image_name.value)
        self.image_size_ = int(param_image_size.value)
        
        self.timer_write_ = self.create_timer(param_timer_dt.value, self.write_callback)
        
        self.image_subscription = self.create_subscription(
            Image,
            str(param_image_topic.value),
            self.image_callback,
            10)   
        self.br = CvBridge()     
        self.acquired_ = False
        
    def write_callback(self):
        if not self.acquired_:
          return 
          
        mutex.acquire()
        self.frame_scaled_ = cv2.resize(self.frame_, (self.image_size_, self.image_size_))
        mutex.release()
        if not cv2.imwrite(self.image_filename_, self.frame_scaled_):
          self.get_logger().warn("Error saving image!")

    def image_callback(self, msg):
        mutex.acquire()
        self.frame_ = self.br.imgmsg_to_cv2(msg, "bgr8")
        self.acquired_ = True
        mutex.release()       


###################################################
def main(args=None):
    rclpy.init(args=args)

    cfd = CameraFeed()
    rclpy.spin(cfd)
    rclpy.shutdown()


if __name__ == '__main__':
  main()
