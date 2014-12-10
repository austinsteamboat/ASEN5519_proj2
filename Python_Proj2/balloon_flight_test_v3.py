#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import numpy as np
import cv2

###############################
# Image Processing Functions: #
###############################

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
	#print 'Num Objs: ',repr(num_objects),' Cx: ',repr(cx),' Cy: ',repr(cy),' Area: ',repr(area_max)
	#image_data = [cx ,cy, area, min_con_x, max_con_x, min_con_y, max_con_y, out_of_bounds, num_objects]
	image_data = [cx,cy,area_max,num_objects]
	return image_data

def get_camera_frame():
	global frame
	global new_frame
	_, frame = cap.read()
	frame = cv2.flip(frame,-1,-1)
	new_frame = True

#Setup of vidro and controller
vidro = Vidro(False, 1)
flight_ready = vidro.connect()
controller = PositionController(vidro)

start_time = time.time()
previous_time = time.time()

cycle = 0

controller.update_gains()

sequence = 0
seq0_cnt = 0
seq1_cnt = 0
seq2_cnt = 0
seq3_cnt = 0
pos_bound_err = 150
yaw_bound_err = 0.2
yaw_com = 0
alt_com = 1000
reset_val = 1
# Idle Commands
roll_idle = 0
pitch_idle = 1
throttle_idle = 2
yaw_idle = 3

print('Heading to while')
while(1):
##        try:
##                print('RC 5 '+repr(self.msg.chan5_raw)+' RC 6 '+repr(self.msg.chan6_raw))
##        except:
##                print('RC 5 Vidro Update '+repr(vidro.current_rc_channels[4])+' RC 6 Vidro Update '+repr(vidro.current_rc_channels[5]))
	vidro.update_mavlink() # Grab updated rc channel values. This is the right command for it, but it doesn't always seem to update RC channels
	while vidro.current_rc_channels[4] > 1600 and flight_ready == True:
		if(reset_val):
			print 'RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
			controller.vidro.rc_throttle_reset()
			controller.vidro.rc_yaw_reset()
			controller.vidro.rc_pitch_reset()
			controller.vidro.rc_roll_reset()
			reset_val = 0
			
		#print('RC 5 '+repr(vidro.current_rc_channels[4])+' RC 6 '+repr(vidro.current_rc_channels[5]))
		vidro.update_mavlink() # Grab updated rc channel values. This is the right command for it, but it doesn't always seem to update RC channels
		#Reset of errors after each time control loop finishes
		controller.I_error_alt = 0
		controller.I_error_pitch = 0
		controller.I_error_roll = 0
		controller.I_error_yaw = 0
		controller.previous_time_alt = (time.time() - controller.timer) * 10
		controller.previous_time_yaw = (time.time() - controller.timer) * 10
		controller.previous_time_xy = (time.time() - controller.timer) * 10
		vidro.previous_error_alt = 0
		vidro.previous_error_yaw = 0
		vidro.previous_error_roll = 0
		vidro.previous_error_pitch = 0
		desc_alt = 1000
		area_max = 0
		yaw_coarse_step = 0.5236 # 30 degrees to get good, overlapping coverage
		print('outer loop'+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1]))
		#Update of gains before going into control loop
		if vidro.current_rc_channels[5] > 1600:
			controller.update_gains()

		while vidro.current_rc_channels[5] > 1600:
			vidro.update_mavlink() # Grab updated rc channel values
			#On ground -> Takeoff to 1 m
			if sequence == 0:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				error_x_y = controller.rc_xy(0, 0)
				#error_z = 0
				#error_yaw = 0
				err_x = error_x_y[0]
				err_y = error_x_y[1]
				#controller.vidro.set_rc_pitch(1450)
				#controller.vidro.set_rc_roll(1370)
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y)+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1]))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					#print('update')
					if seq0_cnt == 10:
						sequence = 1
						roll_idle = vidro.current_rc_channels[0]
						pitch_idle = vidro.current_rc_channels[1]
						throttle_idle = vidro.current_rc_channels[2]
						yaw_idle = vidro.current_rc_channels[3]					

			if sequence == 1:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				error_x_y = controller.rc_xy(0, 0)
				#error_z = 0
				#error_yaw = 0
				err_x = error_x_y[0]
				err_y = error_x_y[1]
				#controller.vidro.set_rc_pitch(1450)
				#controller.vidro.set_rc_roll(1370)
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y)+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1]))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					seq1_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					print('update')
					if seq1_cnt == 10:
						#
						yaw_pos = vidro.get_yaw_radians() # Grab current yaw val, assuming picture taking could be a while so grab it here son
						#
						roll_idle = vidro.current_rc_channels[0]
						pitch_idle = vidro.current_rc_channels[1]
						throttle_idle = vidro.current_rc_channels[2]
						yaw_idle = vidro.current_rc_channels[3]
						#
						controller.vidro.set_rc_roll(roll_idle)
						controller.vidro.set_rc_pitch(pitch_idle)
						controller.vidro.set_rc_throttle(throttle_idle)						
						controller.vidro.set_rc_yaw(yaw_idle)
						#
						get_camera_frame()
						cx_val,cy_val,area_val,num_objects_val = get_object(frame)
						if(num_objects_val>0):
							balloon_found = True # If we have something, we'll assume we've found the balloon
						if(yaw_pos<0):
							yaw_pos+=(2*math.pi) # Keep our bearing estimate between 0 and 2pi
						if(area_val>area_max): # If our current imag has a bigger red area, update our estimate
							area_max_val = area_val						
							img_balloon_ber = (cx_val-cx_mid)*cx_fov/640 # Basic camera model, somebody check this
							max_bear_val = yaw_pos+img_balloon_ber
						if(max_bear_val<0):
							max_bear_val+=(2*math.pi) # Keep it positive
						seq1_cnt = 0
						yaw_com+=yaw_coarse_step
						if (yaw_com > 2 * math.pi): # Check if we've done a full circle
							yaw_com-=(2*math.pi) # Keep it positive	
							sequence = 2
							yaw_com = max_bear_val
			
			if sequence == 2:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				error_x_y = controller.rc_xy(0, 0)
				#error_z = 0
				#error_yaw = 0
				err_x = error_x_y[0]
				err_y = error_x_y[1]
				#controller.vidro.set_rc_pitch(1450)
				#controller.vidro.set_rc_roll(1370)
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y)+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1]))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					seq2_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					#print('update')
					if seq2_cnt == 10:
						sequence = 3
						roll_idle = vidro.current_rc_channels[0]
						pitch_idle = vidro.current_rc_channels[1]
						throttle_idle = vidro.current_rc_channels[2]
						yaw_idle = vidro.current_rc_channels[3]	
						controller.vidro.set_rc_roll(roll_idle)
						controller.vidro.set_rc_pitch(pitch_idle)
						controller.vidro.set_rc_throttle(throttle_idle)						
						controller.vidro.set_rc_yaw(yaw_idle)
						#
						get_camera_frame()
						cv2.imwrite("bft_pic.jpg",frame) # For now, just take a picture

			#Move to landing position
			if sequence == 3:
				error_z = controller.rc_alt(desc_alt)
				error_yaw = controller.rc_yaw(0)
				error_x_y = controller.rc_xy(0, 0)
				err_x = error_x_y[0]
				err_y = error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					print('update')
					seq2_cnt += 1
					#if seq2_cnt==3: # once we've closed the loop 5 times, advance!
					desc_alt-=10
					if(desc_alt<170):
						sequence = 3

			# Land
			if sequence == 4: 
				controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
				sequence = 4

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




