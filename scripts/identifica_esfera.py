#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import time
from cv_bridge import CvBridge, CvBridgeError


lower = 0
upper = 1

cor_menor = np.array([10, 50, 10], dtype=np.uint8)
cor_maior = np.array([ 60, 255, 60], dtype=np.uint8)
mask = None

#TypeError: Expected Ptr<cv::UMat> for argument '%s'
def acha_esfera(frame):
    global mask
   # if frame is not None:
    print("entrou")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    bordas = cv2.Canny(blur, 50, 150, apertureSize=3)
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
    mask = cv2.inRange(bordas_color, cor_menor,cor_maior)
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT,2,40,param1=200,param2=100,minRadius=1,maxRadius=100)

    if circles is not None:
        print("Achou círculo(s): {}".format(lines))
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:

            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(255,0,0),3)

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




