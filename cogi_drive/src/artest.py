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


class CaptureImage:
    def __init__(self, rospy):
        #global rospy
        self.rawImageSize=(640,480) #width, hegith
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.calibrationMatrix = np.array(
            [[ 359.09115129,    0.        ,  315.86758878],
            [   0.        ,  359.6797594 ,  233.97157688],
            [   0.        ,    0.        ,    1.        ]])
        self.calibrationDist = np.array([[-0.34825133,  0.13658961,  0.0004364 , -0.00051536, -0.02596215]])
        
        self.birdEyeViewImageSize=(640,480)
        self.birdEyeViewMatrix = np.array(
            [[ -2.46135195e-01,  -1.31272068e+00,   4.02918487e+02],
            [ -2.51655564e-16,  -2.63947718e+00,   8.01790167e+02],
            [ -2.06984553e-19,  -4.10225207e-03,   1.00000000e+00]])
        self.birdEyeViewCalibrationMatrix = np.array(
            [[  1.74048015e+03,   0.00000000e+00,   3.39001966e+02],
            [  0.00000000e+00,   3.97268990e+03,   2.06401897e+02],
            [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
        self.birdEyeViewCalibrationDist = np.array([[  1.17224820e+01,  -7.30422723e+02,  -4.50227100e-01,4.88336487e-01,  -5.44254659e+00]])


        self.calibrationMatrix=np.array( [[ 359.55954828,    0.         , 304.93780631],
         [   0.   ,       358.45714241,  224.95234163],
         [   0.   ,         0.          ,  1.        ]])

        self.calibrationDist=np.array( [[-0.30956134 , 0.09548415 , 0.00166552 , 0.00114415 -0.01386452]])

        self.birdEyeViewMatrix=np.array( [[ -2.48496108e-01,  -1.32531290e+00 ,  3.99251713e+02],
         [  1.67348183e-17 , -2.65601605e+00 ,  8.00662554e+02],
         [  2.92161820e-19 , -4.14160221e-03 ,  1.00000000e+00]])

        self.birdEyeViewCalibrationMatrix=np.array( [[  1.74048015e+03 ,  0.00000000e+00 ,  3.39001966e+02],
         [  0.00000000e+00 ,  3.97268990e+03 ,  2.06401897e+02],
         [  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00]])

        self.birdEyeViewCalibrationDist=np.array( [[  1.17224820e+01,  -7.30422723e+02,  -4.50227100e-01 ,  4.88336487e-01
          , -5.44254659e+00]])

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def getImage(self):
        while not self.image.size == (
                self.rawImageSize[0]*self.rawImageSize[1]*3):
            continue
        return self.image



    def getCalibrationMatrix(self, imageFunc, nx=6, ny=4):
        
        imageCnt = 0

        objpoints = []
        imgpoints = []
        
        objp = np.zeros((nx*ny, 3), np.float32)
        objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        print("Shake chessboard infront of camera") 
        waitCount = 0
        while(True):
           
            
            if(waitCount > 0):
                waitCount-=1
            elif(waitCount == 1):
                print '.',

            rawImage = imageFunc()
            ret, corners = cv2.findChessboardCorners(rawImage, (nx, ny)) 
            if(ret and waitCount == 0):
                print(str(imageCnt))
                imgpoints.append(corners)
                objpoints.append(objp)
                cv2.drawChessboardCorners(rawImage,(nx,ny),corners,ret)
                        #(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
                imageCnt+=1
                waitCount = 30
                #if(imageCnt > 30):
                #    break
            cv2.imshow('image',rawImage)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        shape = (640, 480)
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, shape, None, None) 
        matrix = (mtx,dist)
        print(matrix)
        return matrix

    
    def getCalibratedImage(self):
        rawImage = self.getImage()
        return cv2.undistort(rawImage, self.calibrationMatrix, self.calibrationDist, None, self.calibrationMatrix)
    
    def getBirdEyeViewMatrix(self):
        imageWidth = self.rawImageSize[0]
        imageHeight = self.rawImageSize[1]
        roiPoints = np.array(
                [
                    [imageWidth//2+50, imageHeight*2//3+30],
                    [imageWidth//2-50, imageHeight*2//3+30],
                    [imageWidth//2-100, imageHeight-50],
                    [imageWidth//2+100, imageHeight-50],
                ], dtype=np.int32)
        roi = np.zeros((imageHeight, imageWidth), dtype=np.uint8)
        cv2.fillPoly(roi, [roiPoints], 1)

        vanishingPointAvg = 0
        vanishingPointCnt = 0
        
        while(True):
            #Code for check
            calibratedImage = self.getCalibratedImage()
            hImage = cv2.cvtColor(calibratedImage, cv2.COLOR_RGB2HLS)[:,:,1]
            roiImage = hImage * roi
            cv2.line(roiImage, 
                    (imageWidth//2-100,imageHeight-80), (imageWidth//2+100, imageHeight-80),
                    (255, 255, 255), thickness=1)
            cv2.line(roiImage, 
                    (imageWidth//2,imageHeight-150), (imageWidth//2, imageHeight-80),
                    (255, 255, 255), thickness=1)
            cv2.imshow('roi image', roiImage)
            
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
            

        
        while(True):
            calibratedImage = self.getCalibratedImage()
            hImage = cv2.cvtColor(calibratedImage, cv2.COLOR_RGB2HLS)[:,:,1]
            edges = cv2.Canny(hImage, 20, 70)
            edgesImage = edges*roi
            lines = cv2.HoughLinesP(edgesImage, 1, np.pi/180, 20, lines=np.array([]), minLineLength=30, maxLineGap=7)
            
            verticalLines = []
            horizontalLines = []
            for line in lines:
                for x1, y1, x2, y2 in line:
                    dx = x2-x1
                    dy = y2-y1
                    dgr = math.degrees(math.atan2(dy,dx))
                    if(abs(dgr) < 45):
                        horizontalLines.append((x1, y1, x2, y2))
                    else:
                        verticalLines.append((x1, y1, x2, y2))
            
            #for x1,y1,x2,y2 in horizontalLines:
            #        cv2.line(calibratedImage, (x1,y1), (x2, y2),(255, 0, 0), thickness=1)
                    #cv2.line(calibratedImage, (x1,y1), (x2, y2),(255, 0, 0), thickness=1)
            
            Lhs = np.zeros((2,2), dtype= np.float32)
            Rhs = np.zeros((2,1), dtype= np.float32)
            for x1,y1,x2,y2 in verticalLines:
                normal = np.array([[-(y2-y1)], [x2-x1]], dtype=np.float32)
                normal /=np.linalg.norm(normal)
                point = np.array([[x1],[y1]], dtype=np.float32)
                outer = np.matmul(normal, normal.T)
                Lhs += outer
                Rhs += np.matmul(outer, point)
                cv2.line(calibratedImage, (x1,y1), (x2, y2),(255, 0, 0), thickness=1)
            vanishingPoint = np.matmul(np.linalg.inv(Lhs),Rhs)
            
            if(vanishingPointCnt == 0):
                vanishingPointAvg = vanishingPoint
                vanishingPointCnt = 1
            else:
                vanishingPointAvg = (
                    (vanishingPointAvg[0]*vanishingPointCnt + vanishingPoint[0])/(vanishingPointCnt+1),
                    (vanishingPointAvg[1]*vanishingPointCnt + vanishingPoint[1])/(vanishingPointCnt+1)
                )
                vanishingPointCnt += 1

            
            cv2.circle(calibratedImage, (vanishingPointAvg[0],vanishingPointAvg[1]), 1,(0,0,255), -1)
            cv2.imshow('lines', calibratedImage)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        top = vanishingPointAvg[1] + 60 # random value
        bottom = imageHeight
        width = imageWidth

        def onLine(p1, p2, ycoord):
            return [
                    p1[0]+ 
                    (p2[0]-p1[0])/float(p2[1]-p1[1])*(ycoord-p1[1]),
                    ycoord
            ]
        p1 = [vanishingPointAvg[0] - width/2, top]
        p2 = [vanishingPointAvg[0] + width/2, top]
        p3 = onLine(p2, vanishingPointAvg, bottom)
        p4 = onLine(p1, vanishingPointAvg, bottom)
        originalImagePoints = np.array(
                [p1,p2,p3,p4], dtype=np.float32)
        
        dstPoints = np.array([
            [0,0],
            [self.birdEyeViewImageSize[0],0],
            [self.birdEyeViewImageSize[0],self.birdEyeViewImageSize[1]],
            [0,self.birdEyeViewImageSize[1]]],dtype=np.float32)
        
        self.birdEyeViewMatrix = cv2.getPerspectiveTransform(originalImagePoints, dstPoints)
        return self.birdEyeViewMatrix
    

    def getBirdEyeViewImage(self):
        calibratedImage = self.getCalibratedImage()
        birdEyeViewImage = cv2.warpPerspective(
                calibratedImage, self.birdEyeViewMatrix,
                self.birdEyeViewImageSize)
        return birdEyeViewImage
    
    def getBirdEyeViewCalibrationMatrix(self):
        self.birdEyeViewCalibrationMatrix, self.birdEyeViewCalibrationDist = self.getCalibrationMatrix(self.getBirdEyeViewImage)
        return self.birdEyeViewCalibrationMatrix, self.birdEyeViewCalibrationDist
    
    def getCalibratedBirdEyeViewImage(self):
        birdEyeViewImage = self.getBirdEyeViewImage()
        calibratedBirdEyeViewImage = cv2.warpPerspective(
                birdEyeViewImage, self.birdEyeViewCalibrationMatrix,
                self.birdEyeViewImageSize)
        return calibratedBirdEyeViewImage

    def getLaneHighlightedBirdEyeViewImage(self):
       
        # get calibrated image
        image = self.getCalibratedImage()
        # color filter
        lightImage = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)[:,:,1]
        # remove noise
        blurImage = cv2.GaussianBlur(lightImage,(3,3), 0)
        # highlight lane
        highlightedImage = cv2.Canny(blurImage, 150,200)

        # warp
        birdEyeViewImage = cv2.warpPerspective(
                #blurImage,
                highlightedImage, 
                self.birdEyeViewMatrix,
                self.birdEyeViewImageSize)
        
        return birdEyeViewImage

if( __name__ == "__main__" ):
    rospy.init_node("cogi_capture_image")
    capture_image = CaptureImage(rospy)
    
    while(True):
        cv2.imshow('bird eye view image', capture_image.getImage())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
