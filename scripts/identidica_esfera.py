#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import time
from cv_bridge import CvBridge, CvBridgeError
from main import goal
from main import lista

lower = 0
upper = 1

# cor_menor = None
# cor_maior = None
mask_green = None
mask_blue = None
mask_red = None

#TypeError: Expected Ptr<cv::UMat> for argument '%s'
def acha_esfera(frame):
    global mask_green
    global mask_red
    global mask_blue

   # if frame is not None:
    print("entrou")
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

    cor_menorG = np.array([50, 50, 40], dtype=np.uint8)
    cor_maiorG = np.array([60, 255, 255], dtype=np.uint8)

    cor_menorB = np.array([100,  40,  40], dtype=np.uint8)
    cor_maiorB = np.array([130, 255, 255], dtype=np.uint8)

    cor_menorR = np.array([0, 110, 110], dtype=np.uint8)
    cor_maiorR = np.array([10, 240, 240], dtype=np.uint8)

    mask_green = cv2.inRange(hsv, cor_menorG,cor_maiorG)
    mask_red = cv2.inRange(hsv, cor_menorR,cor_maiorR)
    mask_blue = cv2.inRange(hsv, cor_menorB,cor_maiorB)

    circleG = cv2.HoughCircles(mask_green, cv2.HOUGH_GRADIENT,2,40,param1=75,param2=50,minRadius=1,maxRadius=100)
    circleB = cv2.HoughCircles(mask_blue, cv2.HOUGH_GRADIENT,2,40,param1=75,param2=50,minRadius=1,maxRadius=150)
    circleR = cv2.HoughCircles(mask_red, cv2.HOUGH_GRADIENT,2,40,param1=75,param2=50,minRadius=1,maxRadius=150)

    if circleG is not None:

        lista.append("green_sphere")

        print("Achou círculo verde: {}".format(circleG))
        circleG = np.uint16(np.around(circleG))

        for i in circleG[0,:]:

            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(255,0,0),3)
            #draw a rectangle around the circle
            cv2.rectangle(frame, (i[0] - 2*(i[2]), i[1] - 2*(i[2])), (i[0] + 2*(i[2]), i[1] + 2*(i[2])), (0, 128, 255), 1)

            x1G, y1G = (i[0] - 2*(i[2]), i[1] - 2*(i[2])) 
            x2G, y2G = (i[0] + 2*(i[2]), i[1] + 2*(i[2]))
            
            ROIG = frame[y1G:y2G, x1G:x2G]            
            if ROIG is not None:
                cv2.imshow('ROI', ROIG)

    if circleR is not None:

        lista.append("red_sphere")

        print("Achou círculo vermelho: {}".format(circleR))
        circleR = np.uint16(np.around(circleR))

        for i in circleR[0,:]:

            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(255,0,0),3)            
            #draw a rectangle around the circle
            cv2.rectangle(frame, (i[0] - 2*(i[2]), i[1] - 2*(i[2])), (i[0] + 2*(i[2]), i[1] + 2*(i[2])), (0, 128, 255), 1)
                       
            x1, y1 = (i[0] - 2*(i[2]), i[1] - 2*(i[2])) 
            x2, y2 = (i[0] + 2*(i[2]), i[1] + 2*(i[2]))
            
            ROIR = frame[y1:y2, x1:x2]            
            if ROIR is not None:
                cv2.imshow('ROI', ROIR)            


    if circleB is not None:

        lista.append("blue_sphere")
        print("Achou círculo azul: {}".format(circleB))

        circleB = np.uint16(np.around(circleB))

        for i in circleB[0,:]:

            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(255,0,0),3)            
            #draw a rectangle around the circle
            cv2.rectangle(frame, (i[0] - 2*(i[2]), i[1] - 2*(i[2])), (i[0] + 2*(i[2]), i[1] + 2*(i[2])), (0, 128, 255), 1)
                                   
            x1B, y1B = (i[0] - 2*(i[2]), i[1] - 2*(i[2])) 
            x2B, y2B = (i[0] + 2*(i[2]), i[1] + 2*(i[2]))
            
            ROIB = frame[y1B:y2B, x1B:x2B]            
            if ROIB is not None:
                cv2.imshow('ROI', ROIB)


    else:
        print("nenhum circulo")

    # cv2.imshow("Video", mask)
    # cv2.waitKey(5)


def auto_canny(image, sigma=0.33):

    v = np.median(image)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    return edged




