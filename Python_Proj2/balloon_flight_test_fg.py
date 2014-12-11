#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import numpy as np
import cv2
import logging


#Start of a log
logging.basicConfig(filename='bft_log.log', level=logging.INFO)
logging.info('Starting')
#Setup of vidro and controller
vidro = Vidro(False, 1)
flight_ready = vidro.connect()
controller = PositionController(vidro)
# Start Time for Logging
global start_time
start_time = time.time()

cycle = 0

controller.update_gains()

sequence = 0
seq0_cnt = 0
seq1_cnt = 0
seq2_cnt = 0
seq3_cnt = 0
pos_bound_err = 150
yaw_bound_err = 0.2
logging_on = True
yaw = 0
reset_val = 1
time_begin = time.time()

##########################
# Image Processing Vars: #
##########################

cap = cv2.VideoCapture(1)
# Color Filters
H_lower = 0 #157
S_lower = 149 #149
V_lower = 67 #67

H_upper = 14 # 186
S_upper = 255 # 255
V_upper = 255 # 255

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

def get_pos():
	x_pos = controller.vidro.get_position()[0]
	y_pos = controller.vidro.get_position()[1]
	z_pos = controller.vidro.get_position()[2]
	yaw_pos = vidro.get_yaw_radians()
	pos_data = [x_pos,y_pos,z_pos,yaw_pos]
	return pos_data

def get_pwm():
	roll_pwm = vidro.current_rc_channels[0]
	pitch_pwm = vidro.current_rc_channels[1]
	throttle_pwm = vidro.current_rc_channels[2]
	yaw_pwm = vidro.current_rc_channels[3]
	pwm_data = [roll_pwm,pitch_pwm,throttle_pwm,yaw_pwm]
	return pwm_data

def get_err(x_com,y_com,z_com,yaw_com):
	try:
		error_z = controller.rc_alt(z_com)
		error_yaw = controller.rc_yaw(yaw_com)
		error_x_y = controller.rc_xy(x_com, y_com)
		error_data = [error_x_y[2],error_x_y[3],error_z,error_x_y[0],error_x_y[1],error_yaw]
	except:
		error_z = 0
		error_yaw = 0
		error_x_y=[0,0,0,0]
		error_data = [error_x_y[2],error_x_y[3],error_z,error_x_y[0],error_x_y[1],error_yaw]
	return error_data

def logger(log_true,sequence,pwm_data,pos_data,error_data):
	if log_true:
		msg_type = ' SEQ '
		log_time = time.time()-start_time
		logging.info(repr(log_time)+','+repr(sequence)+','+repr(pwm_data[0])+','+repr(pwm_data[1])+','+repr(pwm_data[2])+','+repr(pwm_data[3])+','+repr(pos_data[0])+','+repr(pos_data[1])+','+repr(pos_data[2])+','+repr(pos_data[3])+','+repr(error_data[0])+','+repr(error_data[1])+','+repr(error_data[2])+','+repr(error_data[3])+','+repr(error_data[4])+','+repr(error_data[5]))

def sequence_logger(log_true,sequence):
	if log_true:
		msg_type = ' SEQ '
		log_time = time.time()-start_time
		logging.info(' Msg_Type: '+msg_type+' Time: '+repr(log_time)+' Seq: '+repr(sequence))


def pwm_logger(log_true,pwm_data):
	if log_true:
		msg_type = ' PWM '
		log_time = time.time()-start_time
		logging.info(' Msg_Type: '+msg_type+' Time: '+repr(log_time)+' Roll: '+repr(pwm_data[0])+' Pitch: '+repr(pwm_data[1])+' Throttle: '+repr(pwm_data[2])+' Yaw: '+repr(pwm_data[3]))

def pos_logger(log_true,pos_data):
	if log_true:
		msg_type = ' POS '
		log_time = time.time()-start_time
		logging.info(' Msg_Type: '+msg_type+' Time: '+repr(log_time)+' X: '+repr(pos_data[0])+' Y: '+repr(pos_data[1])+' Z: '+repr(pos_data[2])+' Yaw: '+repr(pos_data[3]))

def err_logger(log_true,error_data):
	if log_true:
		msg_type = ' ERR '
		log_time = time.time()-start_time
		logging.info(' Msg_Type: '+msg_type+' Time: '+repr(log_time)+' X_err: '+repr(error_data[0])+' Y_err: '+repr(error_data[1])+' Z_err: '+repr(error_data[2])+' Roll_err: '+repr(error_data[3])+' Pitch_err: '+repr(error_data[4])+' Yaw_err: '+repr(error_data[5]))

def err_check(error_data,pos_bound_err,yaw_bound_err):
	error_x = error_data[0]
	error_y = error_data[1]
	error_z = error_data[2]
	error_yaw = error_data[5]
	try:
		if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(error_y) < pos_bound_err) and (abs(error_x) < pos_bound_err)):
			x_out = True
		else:
			x_out = False
	except:
		x_out = False
	return x_out

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
		print('outer loop'+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1])+' Throttle: '+repr(vidro.current_rc_channels[2])+' Yaw: '+repr(vidro.current_rc_channels[3]))
		alt_com = 0		
		yaw_com = 0		
		x_com = 0
		y_com = 0
		balloon_found = False
		area_max_val = 0
		cx_mid = 640/2 # Given the camera resolution
		cy_mid = 480/2 # Given the camera resoltuion
		guid_adv = 300	
		balloon_area_max_thresh = 150000 # Size for estimating we're at the balloon
		cx_fov = 1.15192 # Camera azimuth field of view in radians(66 degrees)
		#Update of gains before going into control loop
		if vidro.current_rc_channels[5] > 1600:
			controller.update_gains()

		while vidro.current_rc_channels[5] > 1600:
			# Update Commands
			err_log_data = get_err(x_com,y_com,alt_com,yaw_com)
			# Grab updated rc channel values
			vidro.update_mavlink()
			# Get PWM commands and Pos
			pwm_log_data = get_pwm()
			pos_log_data = get_pos()
			# Log data
                        logger(logging_on,sequence,pwm_log_data,pos_log_data,err_log_data)
			#pwm_logger(logging_on,pwm_log_data)
			#pos_logger(logging_on,pos_log_data)
			#err_logger(logging_on,err_log_data)
			print(' X Pos: '+repr(pos_log_data[0])+' X_err: '+repr(err_log_data[0])+' Roll_pwm: '+repr(pwm_log_data[0])+' Y Pos: '+repr(pos_log_data[1])+' Y_err: '+repr(err_log_data[1])+' Pitch_pwm: '+repr(pwm_log_data[1]))
			#On ground -> Takeoff to 1 m
			if sequence == 0:
				alt_com = 1000
				yaw_com = 0
				x_com = 0
				y_com = 0
				#sequence_logger(logging_on,sequence)
				check_val = err_check(err_log_data,pos_bound_err,yaw_bound_err)
				if (check_val):# Closes Error 
					yaw_com +=0.3	
					if yaw_com > 2 * math.pi:
						yaw_com -= (2 * math.pi)				
					seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					if seq0_cnt == 10:
						sequence = 1

			# Coarse Search
			if sequence == 1:
				alt_com = 1000
				#sequence_logger(logging_on,sequence)
				check_val = err_check(err_log_data,pos_bound_err,yaw_bound_err)
				if (check_val):# Closes Error 
					yaw_com +=0.4						
					if ((yaw_com > (2 * math.pi)) and balloon_found):########################################################
						sequence = 2						
						yaw_com = max_bear_val
					else:	
						yaw_com -= (2 * math.pi)			
					seq1_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					if seq1_cnt == 5:
						seq1_cnt = 0
						pos_log_data = get_pos()
						get_camera_frame()
						cx_val,cy_val,area_val,num_objects_val = get_object(frame)
						if(num_objects_val>0):
							balloon_found = True # If we have something, we'll assume we've found the balloon
						if(area_val>area_max_val): # If our current imag has a bigger red area, update our estimate
							area_max_val = area_val
							img_balloon_ber = -1*(cx_val-cx_mid)*cx_fov/640 # Basic camera model, somebody check this
							max_bear_val = pos_log_data[3]+img_balloon_ber

			# Guide
			if sequence == 2:
				alt_com = 1000
				x_com = 0
				y_com = 0
				#sequence_logger(logging_on,sequence)
				check_val = err_check(err_log_data,pos_bound_err,yaw_bound_err)
				if (check_val):# Closes Error 			
					seq2_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					if seq2_cnt == 10:
						seq2_cnt = 0
						pos_log_data = get_pos()
						get_camera_frame()
						cx_val,cy_val,area_val,num_objects_val = get_object(frame)
						if(num_objects_val>0):					# If we've seen something, adjust our pointing
							img_balloon_ber = -1*(cx_val-cx_mid)*cx_fov/640    # Basic camera model, somebody check this
							max_bear_val = pos_log_data[3]+img_balloon_ber
							sequence = 3	
					else:
						balloon_found = False 				# If we don't see anything go back to coarse search
						sequence = 1
			
			#Seq. 4: Terminal
			if sequence == 3:
				cv2.imwrite("terminal_pic.jpg",frame) # For now, just take a picture
				sequence=4			
						
			if sequence == 4:
				yaw_com = 0
				x_com = 0
				y_com = 0
				#sequence_logger(logging_on,sequence)
				check_val = err_check(err_log_data,pos_bound_err,yaw_bound_err)
				if (check_val):# Closes Error 
					alt_com-=10
					if alt_com<170:
						sequence = 5

			# Land
			if sequence == 5: 
				controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
				logging_on = False
				sequence = 5

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




