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
from ar_track_alvar_msgs.msg import AlvarMarkers 
from tf.transformations import euler_from_quaternion

import sys
import os
import signal
import time


class FindAR:
    
    def __init__(self, rospy):
        #global rospy
        self.ar_pose = ""

        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", 
            AlvarMarkers, self.ros_ar_pose_callback)

    def ros_ar_pose_callback(self, data):
        self.ar_pose = data
        #print("ult : "+str(self.ult_data))

    def getARPoss(self):
        while(self.ar_pose == ""):
            continue
        arDatas = {}
        for i in self.ar_pose.markers:
            arData = {}
            arData["raw"] = i
            arData["DX"] = i.pose.pose.position.x 
            arData["DY"] = i.pose.pose.position.y 
            arData["DZ"] = i.pose.pose.position.z  
            arData["AX"] = i.pose.pose.orientation.x 
            arData["AY"] = i.pose.pose.orientation.y 
            arData["AZ"] = i.pose.pose.orientation.z
            arData["AW"] = i.pose.pose.orientation.w
            (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
            arData["roll"] = roll
            arData["yaw"] = yaw
            arData["pitch"] = pitch
            arDatas[i.id] = arData
        return arDatas


if( __name__ == "__main__" ):
    rospy.init_node("cogi_find_ar")
    find_ar = FindAR(rospy)

    mode = "drive"
    while(True):
        #================================= GET VARIABLES
        # I will write here, those variables will be provided
        arPosess = find_ar.getARPoss()
        arDetected = False
        parkingAr = {} 
        if(1 in arPosess):
            arDetected = True
            parkingAr = arPosess[1]
        else:
            continue
        DX = parkingAr["DX"] #meter 
        DY = parkingAr["DZ"] #meter
        YAW = 0
        YAW = parkingAr["pitch"] #radian
        ROLL = parkingAr["roll"] #radian
        leftBackUlt, backUlt, rightBackUlt = 0000, 0000, 0000 #meter 
        leftUlt, rightUlt = 0000, 0000 #meter 

        distance = math.sqrt(DX**2 + DY**2) # (1) this is an example
        degreeOfArVerticalLine =  0 #(2)
        if(DX == 0):
            degreeOfArVerticalLine = 0 # TODO
        elif(DX > 0 and YAW < 0 ): #(DONE) DX + YAW - # LEFT SIDE ONLY
            degreeOfArVerticalLine = (math.atan(DY/DX) + YAW - math.pi/2) * -1
        elif(DX < 0 and YAW < 0 ): #(DONE) DX - YAW - # AND AT BOTH SIDE
            degreeOfArVerticalLine = (math.atan(DY/DX*-1) - YAW - math.pi/2) * 1
        elif(DX > 0 and YAW > 0 ): #(DONE) DX + YAW + # AND AT BOTH SIDE
            degreeOfArVerticalLine = (math.atan(DY/DX) + YAW - math.pi/2) * -1
        elif(DX < 0 and YAW > 0 ): #(DONE) DX - YAW + # RIGHT SIDE ONLY
            degreeOfArVerticalLine = (math.atan(DY/DX*-1) - YAW - math.pi/2) * 1
        distanceFromArVerticalLine = distance*math.sin(degreeOfArVerticalLine) #DONE (3)
        distanceOnArVerticalLine = distance*math.cos(degreeOfArVerticalLine) #DONE (4)
        #================================= Get and set mode ( drive or reverse )
        #TODO Case to be Done
        if(abs(degreeOfArVerticalLine*180/math.pi) < 0000 and abs(YAW) < 0000 and abs(DX) < 000000 ): 
            mode = "done"
            speed = 0
            angle = 0
            break
        '''
        #TODO Case to change from drive to reverse
        if(mode == "drive" and distanceOnArVerticalLine < 0.45): 
            mode = "reverse"
        if(mode == "drive" and YAW > 0000):
            mode = "reverse"
        #TODO Case to change from reverse to drive 
        if(mode == "reverse" and distance > 000): 
            mode = "drive"
        if(mode == "reverse" and (leftBackUlt < 20 or backUlt < 20 or rightBackUlt < 20)): 
            mode = "drive"
        '''
        #================================= Get angle
        angle = 0
        speed = 0
        referPointRatio = 2
        if(mode == "drive"):
            speed = 3
            if(distanceFromArVerticalLine > 0): # onLeft
                print "GO RIGHT ",
                if(DX > 0 and YAW < 0):
                    print "TURN RIGHT (LITTLE) ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio)
                    print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1 - angleC
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = angleA+angleB
                    print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX > 0 and YAW > 0):
                    print "TURN RIGHT (BIG)    ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) 
                    print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1
                    print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY)
                    print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB + angleA - angleC) 
                    print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX < 0 and YAW < 0):
                    print "TURN RIGHT (MAYBE)  ",
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio)
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleA = math.atan(DX/DY * -1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = angleB - angleA - angleC
                    print "angle: {0}, ".format(angle*180/math.pi),
            else: # onRight
                print "GO LEFT  ",
                if(DX < 0 and YAW > 0):
                    print "TURN LEFT  (LITTLE) ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine + math.pi/2) - angleC
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY*-1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleA+angleB) * -1
                    print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX < 0 and YAW < 0):
                    print "TURN LEFT  (BIG)    ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine + math.pi/2)
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY*-1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB + angleA - angleC) * -1
                    print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX > 0 and YAW > 0):
                    print "TURN LEFT  (MAYBE)  ",
                    angleB = (degreeOfArVerticalLine + math.pi/2) 
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleA = math.atan(DX/DY)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB - angleA - angleC) * -1
                    print "angle: {0}, ".format(angle*180/math.pi),
            #angle = distanceFromArVerticalLine #TODO change
        if(mode == "reverse"):
            speed = -3
            angle = 0 
        
        if(angle > 50):
            angle = 50
        elif(angle < -50):
            angle = -50
        #================================== DEBUGING
        #print "DX: {0:0.2f}, DY: {1:0.2f}, YAW: {2:0.2f}, ".format(DX, DY, YAW*180/math.pi, ROLL*180/math.pi),
        #print "DY/DX: {0:0.2f}, atan(DY/DX): {1:0.2f}, ".format(DY/DX, math.atan(DX/DY)*180/math.pi),
        #print "AOV(2): {0:0.2f}, ".format(degreeOfArVerticalLine),
        #print "AOV(2): {0:0.2f}, ".format(degreeOfArVerticalLine*180/math.pi),
        #print "DOA(4): {1:0.2f}, DFA(3): {0:0.2f}, ".format(distanceFromArVerticalLine, distanceOnArVerticalLine),
        #print "angleC: {0}".format(angleC * 180/math.pi), 
        print "\n",
        time.sleep(1)
        #================================== drive
        #driveAngle = PID(angle, 0000, 0000, 0000)
        #drive(driveAngle, 3) #

     
