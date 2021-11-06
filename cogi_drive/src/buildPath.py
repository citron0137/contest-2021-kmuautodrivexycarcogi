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

from captureImage   import CaptureImage
from scanDistance   import ScanDistance
from controlMotor   import ControlMotor
from findAR         import FindAR

class SingleLaneFinder:
    def setLaneName(self, name):
        self.name=name
    def setRoiPoint(self, roiPointX, roiPointY = 330):
        self.roiPoint = (roiPointX, roiPointY)
    def getRoiPoint(self):
        return self.roiPoint
    def setRoiWidth(self, roiWidth):
        self.roiWidth = roiWidth
    def getRoiWidth(self):
        return self.roiWidth

    def getRoiPoints(self):
        roiPoint = self.getRoiPoint()
        roiWidth = self.getRoiWidth()
        roiHeight = 30
        roiBlockPoints = np.array([
                [roiPoint[0] - roiWidth//2,
                    roiPoint[1] - roiHeight//2],
                [roiPoint[0] - roiWidth//2,
                    roiPoint[1] + roiHeight//2],
                [roiPoint[0] + roiWidth//2,
                    roiPoint[1] + roiHeight//2],
                [roiPoint[0] + roiWidth//2,
                    roiPoint[1] - roiHeight//2],
            ])
        return roiBlockPoints
    
    def drawRoi(self, image):
        roiBlockPoints = self.getRoiPoints()
        cv2.rectangle(image, 
                tuple(roiBlockPoints[0]),
                tuple(roiBlockPoints[2]),
                color=(255,255,255), thickness=2)
        return image

    def getRoiImage(self, image):
        # ROI Block Points
        roiBlockPoints = self.getRoiPoints()
        # substract roi
        blockRoi = np.zeros(
                (image.shape[0],image.shape[1]), dtype=np.uint8)
        cv2.fillPoly(blockRoi, [roiBlockPoints], 1)
        roiImage = (image*blockRoi)
        return roiImage

    def getLanePos(self, lineHighlightedBirdEyeViewImage):
        #get ROI cropped image
        #roiImage = self.drawRoi(
        roiImage = self.getRoiImage(
                lineHighlightedBirdEyeViewImage)
        
        #find Lane Pos in Roi image
        _,contours,_ = cv2.findContours(
                roiImage,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contourCenter = []
        for i in contours:
            i = i[0] 
            #print(i)
            centerPoint = [0,0]
            for j in i:
                centerPoint[0] += j[0]
                centerPoint[1] += j[1]
            centerPoint[0] //= len(i)
            centerPoint[1] //= len(i)
            contourCenter.append(centerPoint)
        #print(contourCenter)
        realPos = [-1,-1]

        prevPos = self.getRoiPoint()
        
        for i in contourCenter:
            if(abs(prevPos[0]-realPos[0]) > abs(prevPos[0]-i[0])):
                realPos[0] = i[0]
                realPos[1] = i[1]
        if(realPos[0] == -1):
            self.lineUndetected = True
            return prevPos

        self.setRoiPoint(realPos[0])
        #DEBUGING
        #print(realPos)
        self.drawRoi(
                lineHighlightedBirdEyeViewImage)
        
        self.lineUndetected = False
        return (realPos[0],realPos[1])        
        


def lanePID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error

    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = input_data
    derror = error - prev_error

    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output


if( __name__ == "__main__" ):
    rospy.init_node("cogi_build_path")
    capture_image = CaptureImage(rospy)
    scan_distance = ScanDistance(rospy)
    control_motor = ControlMotor(rospy)
    find_ar = FindAR(rospy)
    
    left_lane_finder = SingleLaneFinder()
    left_lane_finder.setLaneName("left")
    left_lane_finder.setRoiPoint(180, 300)
    #left_lane_finder.setRoiPoint(250, 450)
    left_lane_finder.setRoiWidth(80)
    
    mid_lane_finder = SingleLaneFinder()
    mid_lane_finder.setLaneName("mid")
    mid_lane_finder.setRoiPoint(315, 300)
    mid_lane_finder.setRoiWidth(70)
    
    right_lane_finder = SingleLaneFinder()
    right_lane_finder.setLaneName("right")
    right_lane_finder.setRoiPoint(450, 300)
    #right_lane_finder.setRoiPoint(400, 450)
    right_lane_finder.setRoiWidth(100)


    
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()

    anStartTime = time.time()
    anEndTime = time.time()

    isRightObstaclePassing = False
    isLeftObstaclePassing = False
    lastObstacleDetectedTime = 0
    isPassing = False

    prevLeftObstacleD = -1
    prevRightObstacleD = -1
    
    arDetectedCount = 1
    lastArDetectedTime = 0
    speed = 5

    obstacleDetecting = False
    while(True):#while(True):
        anStartTime = time.time() # To calc freq
        #print("Get Image")
        image = capture_image.getLaneHighlightedBirdEyeViewImage() 
        
        #TODO
        #=================================================== Find Stopline start
        #print("Find Stopline")
        lineDistance = -1
        lineDistance2 = 0
        lineDetectedCnt = 0
        lines = cv2.HoughLinesP(image, 1, np.pi/180, 30, minLineLength = 150)
        if(type(lines)!= type(None)):
            horizontalLines = []
            verticalLines = []
            for i in lines:
               x1, y1, x2, y2 =i[0]
               if(abs(x1 - x2) <= 13): #10 > 13
                    lineDistance2 += y1
                    verticalLines.append([x1,y1,x2,y2])
            if(len(lines) != 0 ):
                lineDistance2 /= len(lines)
            for i in lines:
               x1, y1, x2, y2 =i[0]
               if(abs(y1 - y2) <= 10):
                    horizontalLines.append([x1,y1,x2,y2])
            for i in horizontalLines:
                lineDistance += i[1]
                lineDetectedCnt += 1
            if(lineDetectedCnt != 0 ):
                lineDistance /=lineDetectedCnt
            if( len(verticalLines) > 10):
                print "D:", lineDistance2," verticalLines :", len(verticalLines), " horizontalLines", len(horizontalLines),
            if(len(verticalLines) > 13 and lineDistance2 <= 110 ):
                print "vertical line ({0})".format(480 - lineDistance),
                obstacleDetecting = False
                time.sleep(0.1) #changed
                control_motor.drive(0, 0)
                time.sleep(7)
        # When it is close than 100, it looks hard to detect the line
        #    
        # if(stopLane):
        #   stop for "?" sec
        #   continue
        #=================================================== Find Stopline End
        
        #=================================================== Calc lap (Find Artag) start
        if((time.time() - lastArDetectedTime) > 60 ):
            arPoss = find_ar.getARPoss()
            arData = ""
            if ( 1 in arPoss):
                print("****************************arDetected**********************")
                obstacleDetecting = True
                if(arDetectedCount == 3):
                    break
                else:
                    lastArDetectedTime = time.time()
                    arDetectedCount += 1

        #  
        # if(lapcount > 3): 
        #   break
        #=================================================== Calc lap (Find Artag) End
        
        
        #=================================================== Find Lane Start
        #print("Find Lane")
        leftLanePoint = left_lane_finder.getLanePos(image)
        leftLaneDetected = not left_lane_finder.lineUndetected

        rightLanePoint = right_lane_finder.getLanePos(image)
        rightLaneDetected = not right_lane_finder.lineUndetected
        
        center = (leftLanePoint[0] + rightLanePoint[0])/2
        if(leftLaneDetected and rightLaneDetected):
            center = (leftLanePoint[0] + rightLanePoint[0])/2
        elif(not leftLaneDetected):
            if(leftLanePoint[0] > 158 and abs(leftLanePoint[0]-rightLanePoint[0]) < 150):
                fixedLeftLanePos = rightLanePoint[0]-180
                left_lane_finder.setRoiPoint(fixedLeftLanePos, 300)
                leftLanePoint = (fixedLeftLanePos, 300)
        elif(not rightLaneDetected):
            if(rightLanePoint[0] < 456 and abs(leftLanePoint[0]-rightLanePoint[0]) < 150):
                fixedRightLanePos = leftLanePoint[0]+180
                right_lane_finder.setRoiPoint(fixedRightLanePos, 300)
                rightLanePoint = (fixedRightLanePos, 300)
        else:
            print "[Both Missed]", 
            print("left =", leftLanePoint[0], leftLaneDetected, 
                "right =", rightLanePoint[0], rightLaneDetected)
        
        center = (leftLanePoint[0] + rightLanePoint[0])/2
        mid_lane_finder.setRoiPoint(center, 300)
        midLanePoint = mid_lane_finder.getLanePos(image)
        center = midLanePoint[0]
        print "[lane: {0} | {1} | {2}] ".format(
                leftLanePoint[0], midLanePoint[0], rightLanePoint[0]), # DBG
        #=================================================== Find Lane End
        # res : Left lane, Right lane, Mid lane, isCurve
        
        #
        #=================================================== Find Obstacle Start
        leftObstacleCnt = 0
        rightObstacleCnt = 0
        centerPlace = (320 - center)* 14/22*11/13 # calibrate
        centerDriveAngle = (leftLanePoint[0] + rightLanePoint[0])/2
        print "[centerDriveAngle: {0}] ".format(centerDriveAngle),
        if(not isLeftObstaclePassing and not isRightObstaclePassing):
            points = scan_distance.getLidObstaclePoints()
            obstacleOnLane = []
            for i in points:
                #if(abs(i[0] - centerPlace) < 10 and i[1] < 40 and i[1] > 20 ):
                if( abs(centerDriveAngle - 320) < 50 and abs(i[0] - centerPlace) < 15 and i[1] < 40 and i[1] > 20 ):
                    obstacleOnLane.append(i)       
            obstacleRightavg = 0
            obstacleLeftavg = 0
            for i in obstacleOnLane:
                if(i[0]-centerPlace < 0):
                    obstacleLeftavg += i[0]
                    leftObstacleCnt += 1
                else:
                    obstacleRightavg += i[0]
                    rightObstacleCnt +=1
            if(rightObstacleCnt):
                obstacleRightavg /= rightObstacleCnt
            if(leftObstacleCnt):
                obstacleLeftavg /= leftObstacleCnt
            if(abs(centerPlace) > 30):
                a = 1
            elif(obstacleDetecting and leftObstacleCnt > 7 and rightObstacleCnt == 0): # 5 -> 7
                isLeftObstaclePassing = True
                lastObstacleDetectedTime = time.time()
                ultPoints = scan_distance.getUltObstaclePoints()
                leftDistance = ultPoints[1][0] *-1
                rightDistance = ultPoints[2][0]
                prevLeftObstacleD = leftDistance
                
            elif(obstacleDetecting and rightObstacleCnt > 7 and leftObstacleCnt == 0):
                isRightObstaclePassing = True
                lastObstacleDetectedTime = time.time()
                ultPoints = scan_distance.getUltObstaclePoints()
                leftDistance = ultPoints[1][0] *-1
                rightDistance = ultPoints[2][0]
                prevRightObstacleD = rightDistance
            print "[obstacle: {0}({3}), {1}({2})] ".format(
                    leftObstacleCnt, rightObstacleCnt, obstacleRightavg, obstacleLeftavg),
        else:
            ultPoints = scan_distance.getUltObstaclePoints()
            leftDistance = ultPoints[1][0] * -1
            rightDistance = ultPoints[2][0]
            obstacelDThreashhold = 21
            if(isLeftObstaclePassing):
                '''
                #if(isPassing and leftDistance < obstacelDThreashhold * -1):
                if(isPassing and (leftDistance - prevLeftObstacleD)  > 25):
                    isPassing = False
                    isLeftObstaclePassing = False
                #elif(leftDistance >obstacelDThreashhold *-1 ):
                elif((leftDistance - prevLeftObstacleD) < -25  and leftDistance < 60):
                    isPassing = True
                '''
                if( time.time() - lastObstacleDetectedTime > 1.5 ): #changed 
                    isLeftObstaclePassing = False
                
                if((leftDistance - prevLeftObstacleD) < -15  and leftDistance < 60):
                    isLeftObstaclePassing = False

            elif(isRightObstaclePassing):
                '''
                #if(isPassing and rightDistance > obstacelDThreashhold):
                if(isPassing and (rightDistance - prevRightObstacleD)  > 25):
                    isPassing = False
                    isRightObstaclePassing = False
                #elif(rightDistance <obstacelDThreashhold  ):
                elif((rightDistance - prevRightObstacleD) < -25 and rightDistance < 60):
                    isPassing = True
                '''
                if( time.time() - lastObstacleDetectedTime > 3 ): #changed 
                    isLeftObstaclePassing = False
                if((rightDistance - prevRightObstacleD) < -15 and rightDistance < 60):
                    isRightObstaclePassing = False

            prevRightObstacleD = rightDistance
            prevLeftObstacleD = leftDistance
            '''
            if(isLeftObstaclePassing):
                print "[obstacleD: {0}] ".format(leftDistance, rightDistance, isPassing),
            elif(isRightObstaclePassing):
                print "[obstacleD: {1}] ".format(leftDistance, rightDistance, isPassing),
            '''
            # left ult be close
            # -> still passing
            # left ult be far
            # -> passed
            #if(right passing)
            # right ult be close
            # -> still passing
            # right ult be far
            # -> passed

        #print "[{0}, {1}] ".format(leftObstacleCnt, rightObstacleCnt),
        #=================================================== Find Obstacle End
        # res : Place of obstacle (front left or front right, rightPass, leftPass, or none)
        
        
        #    
        #=================================================== Control motor    
        #print("Control motor")
        center = (leftLanePoint[0] + rightLanePoint[0])/2
        pidP = 1.2
        if isLeftObstaclePassing:
            center = (midLanePoint[0] + rightLanePoint[0])/2 - 5 #- 15
            speed = 5
            print "[dirve Right: {0}] ".format(center),
            pidP = 1.5
        elif isRightObstaclePassing:
            center = (leftLanePoint[0] + midLanePoint[0])/2 + 5 #+ 15
            speed = 5
            print "[dirve Left : {0}] ".format(center),
            pidP = 1.5
        elif(not obstacleDetecting):
            speed = 5
            print "[dirve CurveZone : {0}] ".format(center),
            pidP = 1.8
        elif(abs(center-320) > 20):
            speed = 5
            print "[dirve Curve : {0}] ".format(center),
        else:
            print "[dirve Center : {0}] ".format(center),
            if(speed < 8):
                speed += 0.1
            

        #print "[dirve center: {0}] ".format(center),

        #   center = center of non-obstacle lane
        #   speed = "slow"
        # elif isCurve:
        #   center = "center of both lane
        #   speed = "slow"
        # else:
        #   center = "center of both lane
        #   speed = "fast"
        #angle = (center - image.shape[1]/2) / 1.2
        angle = lanePID(center, pidP, 0.0007, 0.01)
        #angle = PID(angle, 1.2, 0.0007, 0.01)
        control_motor.drive(angle, int(speed))
        #=================================================== Control motor    
        
        
        #=================================================== DBG code
        #if(abs(center - midLanePoint[0])  > 50):
        #    center = midLanePoint[0]
        #    mid_lane_finder.setRoiPoint(center,300)
        #    midLanePoint = mid_lane_finder.getLanePos(image)
        #    midLaneDetected = not mid_lane_finder.lineUndetected
        #print("mid =", midLanePoint[0],"/",center, midLaneDetected)
        anEndTime = time.time()
        anTermMs = (anEndTime - anStartTime)*1000
        dbgStr = "\t[{0:0.3f}ms]".format(anTermMs)
        #print dbgStr,
        print()
        #=================================================== DBG code
   


    # go along right lane until half of right obstacle
    # go back for "?" time with angle "?"
    # until "?"
    # go back for "?" time with angle "0"
    # while "ult back > 10"

    #==================================================== parking Start
    print("[Parking] Start")
    # After 3rd qr tag detection
    # stage 1 # till the landmark(Right wall)
    ultRight = scan_distance.getUltObstaclePoints()[2][0]
    howManyTimeDetectedInOnce = 0
    isDetectedJustBefore = False
    while(True):
        # Case : When the right obstacle going to be closer, End stage 1
        prevUltRight = ultRight
        ultRight = scan_distance.getUltObstaclePoints()[2][0] 
        if( ultRight < 25):
            howManyTimeDetectedInOnce += 1
            isDetectedJustBefore = True
        elif(isDetectedJustBefore):
            howManyTimeDetectedInOnce = 0
            isDetectedJustBefore = False
        if( howManyTimeDetectedInOnce >= 15):
            break
        # Else : process to drive along right lane
        image = capture_image.getLaneHighlightedBirdEyeViewImage()
        #============================================== Find lane Position
        leftLanePoint = left_lane_finder.getLanePos(image)
        leftLaneDetected = not left_lane_finder.lineUndetected

        rightLanePoint = right_lane_finder.getLanePos(image)
        rightLaneDetected = not right_lane_finder.lineUndetected

        center = (leftLanePoint[0] + rightLanePoint[0])/2
        if(leftLaneDetected and rightLaneDetected):
            center = (leftLanePoint[0] + rightLanePoint[0])/2
        elif(not leftLaneDetected):
            if(leftLanePoint[0] > 158 and abs(leftLanePoint[0]-rightLanePoint[0]) < 150):
                fixedLeftLanePos = rightLanePoint[0]-180
                left_lane_finder.setRoiPoint(fixedLeftLanePos, 300)
                leftLanePoint = (fixedLeftLanePos, 300)
        elif(not rightLaneDetected):
            if(rightLanePoint[0] < 456 and abs(leftLanePoint[0]-rightLanePoint[0]) < 150):
                fixedRightLanePos = leftLanePoint[0]+180
                right_lane_finder.setRoiPoint(fixedRightLanePos, 300)
                rightLanePoint = (fixedRightLanePos, 300)
        else:
            print "[Both Missed]",
            print("left =", leftLanePoint[0], leftLaneDetected,
                "right =", rightLanePoint[0], rightLaneDetected)
        center = (leftLanePoint[0] + rightLanePoint[0])/2
        mid_lane_finder.setRoiPoint(center, 300)
        midLanePoint = mid_lane_finder.getLanePos(image)
        center = midLanePoint[0]
        #============================================== Get angle
        center = (midLanePoint[0] + rightLanePoint[0])/2
        angle = lanePID(center, 1.2, 0.0007, 0.01)
        #============================================== Drive
        speed = 3
        control_motor.drive(angle, speed)
        

    control_motor.drive(0, 0)
    print("[Parking] Stop just before the right obstacle")
    time.sleep(1)

    print("[Parking] Reverse drive with maximum angle")
    #stage 3 # reverse drive with maximum angle
    startTime = time.time()
    while(True): 
        speed = -3
        angle = 50 #right_end
        if( time.time() - startTime > 2.2 ): ##freeze for several mins
            break
        control_motor.drive(angle, speed)

    control_motor.drive(0, 0)
    print("[Parking] Reverse drive with maximum angle done")
    time.sleep(1)


    print("[Parking] reverse drive straightly till the special point")
    #stage 4 # reverse drive, straightly till the special point
    while(True):  
        speed = -3
        angle = -50 # center
        ultData = scan_distance.getUltData()
        rightBackUlt , leftBackUlt, backUlt = ultData[5], ultData[7], ultData[6]
        if(backUlt < 20 or leftBackUlt < 20 or rightBackUlt < 20): # 
            break
        control_motor.drive(angle, speed)
    
    control_motor.drive(0, 0)
    print("[Parking] reverse drive straightly till the special point done")
    time.sleep(1)
    #===============================================================================
    #'''

   
    print("[Parking] start fit in to the parking zone")
    # stage 5 ######## From here, xycar use the ar_tag 
    
    start_time = time.time()
    end_time = time.time()
    prev_error = 0.0
    i_error = 0.0
    
    mode = "drive"
    while(True):
        #================================= GET VARIABLES
        # I will write here, those variables will be provided
        arPosess = find_ar.getARPoss()
        arDetected = False
        parkingAr = {}
        if(2 in arPosess):
            arDetected = True
            parkingAr = arPosess[2]
        else:
            continue
        DX = parkingAr["DX"] #meter
        DY = parkingAr["DZ"] #meter
        YAW = parkingAr["pitch"] #radian
        ROLL = parkingAr["roll"] #radian
        #leftBackUlt, backUlt, rightBackUlt = 0000, 0000, 0000 #meter
        #leftUlt, rightUlt = 0000, 0000 #meter
        ultData = scan_distance.getUltData()
        rightBackUlt , leftBackUlt, backUlt = ultData[5], ultData[7], ultData[6]

        distance = math.sqrt(DX**2 + DY**2) # (1) this is an example
        degreeOfArVerticalLine =  0 #(2)
        if(DX == 0):
            degreeOfArVerticalLine = YAW * -1 #
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
        if( distanceOnArVerticalLine < 0.32  and distanceOnArVerticalLine > 0.28):
            if(abs(degreeOfArVerticalLine*180/math.pi) <= 7 and abs(YAW*180/math.pi) <= 7 and abs(DX) <= 0.05 ):
                mode = "done"
                speed = 0
                angle = 0
                break
        # Case to change from drive to reverse
        if(mode == "drive" and distanceOnArVerticalLine < 0.25):
            mode = "reverse"
        #if(mode == "drive" and YAW > 0000):
        #    mode = "reverse"
        # Case to change from reverse to drive
        '''
        if(mode == "reverse" and distance > 3.5):
            mode = "drive"
        '''
        if(mode == "reverse" and (leftBackUlt < 20 or backUlt < 20 or rightBackUlt < 20)):
            mode = "drive"
        #================================= Get angle
        angle = 0
        speed = 0
        referPointRatio = 4
        if(mode == "drive"):
            speed = 3
            if(distanceFromArVerticalLine > 0): # onLeft
                #print "GO RIGHT ",
                if(DX > 0 and YAW < 0):
                    print "TURN RIGHT (LITTLE) ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio)
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1 - angleC
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = angleA+angleB
                    #print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX > 0 and YAW > 0):
                    print "TURN RIGHT (BIG)    ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio)
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB + angleA - angleC)
                    #print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX < 0 and YAW < 0):
                    print "TURN RIGHT (MAYBE)  ",
                    angleB = (degreeOfArVerticalLine - math.pi/2) * -1
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio)
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleA = math.atan(DX/DY * -1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = angleB - angleA - angleC
                    #print "angle: {0}, ".format(angle*180/math.pi),
            else: # onRight
                #print "GO LEFT  ",
                if(DX < 0 and YAW > 0):
                    print "TURN LEFT  (LITTLE) ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine + math.pi/2) - angleC
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY*-1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleA+angleB) * -1
                    #print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX < 0 and YAW < 0):
                    print "TURN LEFT  (BIG)    ",
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleB = (degreeOfArVerticalLine + math.pi/2)
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleA = math.atan(DX/DY*-1)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB + angleA - angleC) * -1
                    #print "angle: {0}, ".format(angle*180/math.pi),
                elif(DX > 0 and YAW > 0):
                    print "TURN LEFT  (MAYBE)  ",
                    angleB = (degreeOfArVerticalLine + math.pi/2)
                    #print "b: {0}, ".format(angleB*180/math.pi),
                    angleC = math.atan((distanceOnArVerticalLine/distanceFromArVerticalLine)/referPointRatio) * -1
                    #print "c: {0}, ".format(angleC*180/math.pi),
                    angleA = math.atan(DX/DY)
                    #print "a: {0}, ".format(angleA*180/math.pi),
                    angle = (angleB - angleA - angleC) * -1
                    #print "angle: {0}, ".format(angle*180/math.pi),
        if(mode == "reverse"):
            speed = -3
            angle = 0

        angle *= 180/math.pi
        if(angle > 50):
            angle = 50
        elif(angle < -50):
            angle = -50
        #================================== DEBUGING
        print "DX: {0:0.2f}, DY: {1:0.2f}, YAW: {2:0.2f}, ".format(DX, DY, YAW*180/math.pi, ROLL*180/math.pi),
        #print "DY/DX: {0:0.2f}, atan(DY/DX): {1:0.2f}, ".format(DY/DX, math.atan(DX/DY)*180/math.pi),
        #print "AOV(2): {0:0.2f}, ".format(degreeOfArVerticalLine),
        print "AOV(2): {0:0.2f}, ".format(degreeOfArVerticalLine*180/math.pi),
        print "DOA(4): {1:0.2f}, DFA(3): {0:0.2f}, ".format(distanceFromArVerticalLine, distanceOnArVerticalLine),
        print "ULT: {0:0.2f}, {1:0.2f}, {2:0.2f}, ".format(leftBackUlt, backUlt, rightBackUlt),
        print "angle: {0:0.2f}".format(angle),
        print "\n",
        #time.sleep(1)
        #================================== drive
        #driveAngle = PID(angle, 0000, 0000, 0000)
        
        control_motor.drive(angle, speed)
    
    print("[Done] Done Parking")
    control_motor.drive(0, 0)
