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
global start_time
start_time= time.time()
time_new = 0
H_lower = 20 #157
S_lower = 114 #149
V_lower = 121 #67

H_upper = 182 # 186
S_upper = 255 # 255
V_upper = 255 # 255


cx_mid = 640/2 # Given the camera resolution
cy_mid = 480/2 # Given the camera resoltuion
guid_adv = 300	
balloon_area_max_thresh = 150000 # Size for estimating we're at the balloon
cx_fov = 1.15192 # Camera azimuth field of view in radians(66 degrees)
#Start of a log
logging.basicConfig(filename='btd.log', level=logging.INFO)

logging_on = True

def img_logger(log_true,cx,cy,num_objects,area_max):
	if log_true:
		msg_type = ' IMG '
		log_time = time.time()-start_time
		logging.info(' Msg_Type: '+msg_type+' Time: '+repr(log_time)+' Cx: '+repr(cx)+' Cy: '+repr(cy)+' Num Objs: '+repr(num_objects)+' Area: '+repr(area_max))

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
	area_max = 0
	cnt_max=0
	if (num_objects>0):
		for i in range(0,num_objects):
			cnt = contours[i]
			area = cv2.contourArea(cnt)
			if area>area_max:
				area_max=area
				cnt_max=cnt
		#perimeter = cv2.arcLength(cnt, True)
		moment = cv2.moments(cnt_max)
		cx = int(moment['m10']/moment['m00'])
		cy = int(moment['m01']/moment['m00'])
	else:
		cnt_max = 0
		area_max = 0
		cx = 0
		cy = 0
	cv2.circle(frame,(cx,cy),5,(255,0,0),-1)
	print 'Num Objs: ',repr(num_objects),' Cx: ',repr(cx),' Cy: ',repr(cy),' Area: ',repr(area_max)
	#image_data = [cx ,cy, area, min_con_x, max_con_x, min_con_y, max_con_y, out_of_bounds, num_objects]
	img_logger(logging_on,cx,cy,num_objects,area_max)

def get_camera_frame():
	global frame
	global new_frame
	_, frame = cap.read()
	frame = cv2.flip(frame,-1,-1)
	new_frame = True

while(1):
	time_prev = time_new
	time_new = time.time()
	get_camera_frame()
	get_object(frame)
	# Grab our current x,y position          
	x_pos=0#vidro.get_position()[0]            
	y_pos=0#vidro.get_position()[1]            
	# Calculate step towards balloon
	img_balloon_ber = (cx_val-cx_mid)*cx_fov/640         
	d_x = guid_adv*math.cos(max_bear_val)    
	d_y = guid_adv*math.sin(max_bear_val)    
	x_com = x_pos+d_x			 
	y_com = y_pos+d_y
	print(' img_balloon_ber: '+repr(img_balloon_ber)+' x_com: '+repr(x_com)+' y_com: '+repr(y_com))
	#cv2.imshow('frame',frame)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()


