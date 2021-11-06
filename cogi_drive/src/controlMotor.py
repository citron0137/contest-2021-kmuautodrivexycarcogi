import rospy
import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray
from matplotlib import pyplot as plt

import sys
import os
import signal
import time


class ControlMotor:

    def __init__(self, rospy):
        self.motorPub = rospy.Publisher(
                'xycar_motor', xycar_motor, queue_size=1)
        print("init")
    def drive(self, angle, speed = 50):
        msg = xycar_motor()
        msg.angle = angle
        msg.speed = speed
        
        self.motorPub.publish(msg)
