#!/usr/bin/env python

import rospy
import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray

import sys
import os
import signal
import time




# what drawMap do 
# - find lane
# - find obstacles
# - show bird eye view image with lane and obstacles

class DrawMap:
    def __init__(self):
        self.imageCalibtraion = 0
        self.pltFig, self.pltAx = plt.subplots()
        self.obstaclePoints= []

    def setImageCalibration(self, left, right, bottom, top):
        print("set image calibration")
        self.imageCalibration = [left, right, bottom, top]

    def imagePointToMapPoint(self):
        print("Map Point")

    def setDistancCalibration():
        print("set distanceCalibration")
    
    def putImage(self, image):
        #print("add Image")
        self.image = image

    def putObstaclePoints(self, obstaclePoints):
        #print("add obstacle data")
        self.obstaclePoints = obstaclePoints

    def findLane():
        print("lane data")

    def findObstacles():
        print("obstacle data")

    def showMap(self):
        ax = self.pltAx
        obstaclePoints = self.obstaclePoints

        ax.imshow(self.image, 
                extent=self.imageCalibration, aspect='auto')
        
        ax.scatter(0,0,s=100, c='r') # Car plot
        
        for i in obstaclePoints:
            ax.scatter(i[0], i[1], c='b')
        plt.xlim([-100,+100])
        plt.ylim([-50,+150])
        plt.pause(0.2)
        ax.clear()


if __name__ == "__main__":
    from captureImage import CaptureImage
    from scanDistance import ScanDistance
    
    rospy.init_node("cogi_draw_map")

    capture_image = CaptureImage(rospy)
    scan_distance = ScanDistance(rospy)
    #draw_map = DrawMap()
    #draw_map.setImageCalibration(-120,120,20,96)
     
    while(1):
        image = capture_image.getBirdEyeViewImage()
        #ultData = scan_distance.getUltData()
        
        #leftD = ultData[0]
        #rightD = ultData[4]
        #leftBackD = ultData[7]
        #rightBackD = ultData[5]
        #backD = ultData[6]
        #print(leftD, leftBackD, backD, rightBackD, rightD)
        
        obstaclePoints = scan_distance.getLidObstaclePoints() 
        print(scan_distance.getUltObstaclePoints())
        #draw_map.putImage(image)
        #draw_map.putObstaclePoints(obstaclePoints)
        #draw_map.showMap()


