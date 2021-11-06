#!/usr/bin/env python

import rospy
import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray

import sys
import os
import signal
import time


image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

ult_data = ""
lid_data = ""
lid_increment = ""

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def ros_ult_callback(data):
    global ult_data
    ult_data = data.data
    print('ult : '+str(ult_data))

def ros_lid_callback(data):
    global lid_data, lid_increment
    lid_data = data.ranges
    lid_increment = data.angle_increment
    print("lid : "+str(lid_data)+"("+str(lid_increment)+")")



rospy.init_node("cogi_drive")

image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
ultrasonic_sub = rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray, ros_ult_callback)
ridar_sub = rospy.Subscriber('/scan',LaserScan,ros_lid_callback)

while True: 
    while not image.size == (640*480*3):
        continue

    #print(lid_increment)
    #print(lid_data)
    #time.sleep(1)
    #print(len(lid_data))
    #for i in range(len(lid_data)):
    #    radian = i * lid_increment
    #    x = lid_data[i] * np.cos(radian) * -1
    #    y = lid_data[i] * np.sin(radian) * 1
    #    #if(x == inf || y == inf):
        #    continue
    #    print(x,y)

    #cv2.imshow('image',image)
    
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
    #print "================================================"
