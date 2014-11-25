#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import matplotlib.pyplot as plot
import logging
import numpy as np
import cv2

cap = cv2.VideoCapture(1)

H_lower = 157
S_lower = 149
V_lower = 67

H_upper = 186
S_upper = 255
V_upper = 255

def get_object(frame):
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower = np.array([H_lower, S_lower, V_lower], dtype=np.uint8)
	upper = np.array([H_upper,S_upper,V_upper], dtype=np.uint8)
	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower, upper)
	kernel_open = np.ones((4,4),np.uint8)
	kernel_close = np.ones((25,25),np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)
	_,contours,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frame, contours, -1, (0,255,0), 3)
	num_objects = len(contours)
	if (num_objects>0):
		cnt = contours[0]
		area = cv2.contourArea(cnt)
		#perimeter = cv2.arcLength(cnt, True)
		moment = cv2.moments(cnt)
		cx = int(moment['m10']/moment['m00'])
		cy = int(moment['m01']/moment['m00'])
	else:
		cnt = 0
		area = 0
		cx = 0
		cy = 0
	cv2.circle(frame,(cx,cy),5,(255,0,0),-1)
	print 'Num Objs: ',repr(num_objects),' Cx: ',repr(cx),' Cy: ',repr(cy),' Area: ',repr(area)
	#image_data = [cx ,cy, area, min_con_x, max_con_x, min_con_y, max_con_y, out_of_bounds, num_objects]


def get_camera_frame():
	global frame
	global new_frame
	_, frame = cap.read()
	new_frame = True

while(1):
	get_camera_frame()
	get_object(frame)
	cv2.imshow('frame',frame)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()


