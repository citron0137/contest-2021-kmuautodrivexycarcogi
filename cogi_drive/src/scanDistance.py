#!/usr/bin/env python

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


class ScanDistance:
    #ultdata
    #lidata

    def __init__(self, rospy):
        #global rospy
        
        self.ult_data = ""
        self.lid_data = ""
        self.lid_increment = ""
        
        self.ultrasonic_sub = rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray, self.ros_ult_callback)
        self.ridar_sub = rospy.Subscriber('/scan',LaserScan,self.ros_lid_callback)

    def ros_ult_callback(self, data):
        self.ult_data = data.data
        #print("ult : "+str(self.ult_data))

    def ros_lid_callback(self, data):
        self.lid_data = data.ranges
        self.lid_increment = data.angle_increment
        #print("lid : "+str(self.lid_data)+"("+str(self.lid_increment)+")")

    
    def getUltData(self):
        while(self.ult_data == ""):
            continue
        return self.ult_data 
    
    def getLidData(self):
        while(self.lid_data == ""):
            continue
        return (self.lid_data, self.lid_increment) 

    def getObstaclePoints(self):
        obstaclePoint = []
        
        ultData = self.getUltData() #ULT PLOT
        leftD = ultData[0]
        rightD = ultData[4]
        leftBackD = ultData[7]
        rightBackD = ultData[5]
        backD = ultData[6]
        obstaclePoint.append((0,0-backD)) 
        #plt.scatter(0,0-backD, c='g')
        obstaclePoint.append((0-leftD,0)) 
        #plt.scatter(0-leftD,0, c='g')
        obstaclePoint.append((0+rightD,0)) 
        #plt.scatter(0+rightD,0, c='g')
        
        lidData,lidIncrement = self.getLidData() #LID Plot
        for i in range(18):
            index = len(lidData)//36 * i
            distance = lidData[index]
            radian = lidIncrement*index
            scale = 100
            x = math.cos(radian) * distance * scale * -1
            y = math.sin(radian) * distance * scale
            if(x != float('inf') and x != float('-inf')
                    and y != float('inf') and y != float('-inf')):
                obstaclePoint.append((x,y))
            #plt.scatter(x,y,c='b')
        return obstaclePoint
    
    def getLidObstaclePoints(self):
        obstaclePoint = []
        lidData,lidIncrement = self.getLidData() #LID Plot

        cnt = -1
        while(True):
            cnt+=1
            radian = cnt * lidIncrement
            if(radian > 3):
                break
            elif(radian <0.14):
                continue
            distance = lidData[cnt]
            radian = lidIncrement*cnt

            scale = 100
            x = math.cos(radian) * distance * scale * -1
            y = math.sin(radian) * distance * scale
            if(x != float('inf') and x != float('-inf')
                    and y != float('inf') and y != float('-inf')):
                obstaclePoint.append((x,y))

        return obstaclePoint
    
    def getUltObstaclePoints(self):
        obstaclePoint = []
        ultData=self.getUltData() 
        leftD = ultData[0]
        rightD = ultData[4]
        leftBackD = ultData[7]
        rightBackD = ultData[5]
        backD = ultData[6]
        obstaclePoint.append((0,0-backD)) 
        #plt.scatter(0,0-backD, c='g')
        obstaclePoint.append((0-leftD,0)) 
        #plt.scatter(0-leftD,0, c='g')
        obstaclePoint.append((0+rightD,0)) 
        #plt.scatter(0+rightD,0, c='g')
        return obstaclePoint

    def showPlots(self):
        fig = plt.figure()
        while(True):
            plt.scatter(0,0,s=100, c='r') # Car plot
            op = self.getObstaclePoints()
            for i in op:
                plt.scatter(i[0], i[1], c='b')
            plt.xlim([-100,+100])
            plt.ylim([-100,+100])
            plt.grid()
            plt.pause(0.2)
            fig.clear()

if(__name__ == "__main__"):
    rospy.init_node("cogi_scan_distance")
    scan_distance = ScanDistance(rospy)
    #scan_distance.showPlots()
    while(1):
        ultData = scan_distance.getUltData()
        leftD = ultData[0]
        rightD = ultData[4]
        leftBackD = ultData[7]
        rightBackD = ultData[5]
        backD = ultData[6]
        print(leftD, leftBackD, backD, rightBackD, rightD)

        time.sleep(1)
    '''
    while(1):
        print("aaa")
        lidData = scan_distance.getLidObstaclePoints()
        print(lidData)
    '''
