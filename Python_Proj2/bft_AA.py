#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import numpy as np
import cv2

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

###############################
#Setup of vidro and controller#
###############################

vidro = Vidro(False, 1)
flight_ready = vidro.connect()
controller = PositionController(vidro)
start_time = time.time()
# Load gains
controller.update_gains() 

####################
# Controller Values#
####################

take_off_alt = 1000
# Pos Commands
x_com = 0
y_com = 0
alt_com = 0
yaw_com = 0
# Pos Values
x_pos = 0
y_pos = 0
alt_pos = 0
yaw_pos = 0
guid_adv = 500 # Guidance advancing value, ie move n mm towards estimated bearing and re-check

##################
# Img Proc Values#
##################

cx_val = 0
cy_val = 0
num_objects_val = 0
cx_mid = 640/2 # Given the camera resolution
cy_mid = 480/2 # Given the camera resoltuion
cx_fov = 1.15192 # Camera azimuth field of view in radians(66 degrees)
cy_fov = 0.69813 # Camera elivation field of view in radians(40 degrees)
area_val = 0
area_max_val = 0
max_bear_val = 0 # bearing of the largest return from search
img_balloon_ber = 0; # balloon bearing in the image
f_val = 1 # Focal scalar
yaw_coarse_step = 0.5236 # 30 degrees to get good, overlapping coverage
balloon_found = False
balloon_area_max_thresh = 250000 # Size for estimating we're at the balloon
#################### Add defs in balloon_tracker_lib for img processing shit

#############
# FSM Values#
#############
sequence = 0
seq0_cnt = 0
seq1_cnt = 0
seq2_cnt = 0
seq3_cnt = 0

# Err Bounds
pos_bound_err = 100
yaw_bound_err = 0.2
# RC Over-ride reset initialization
reset_val = 1

print('Heading to main loop')

#####################
# Main Software Loop#
#####################

while(1):
	vidro.update_mavlink() # Grab updated rc channel values. This is the right command for it, but it doesn't always seem to update RC channels
	# Armed Loop
	# Pilot still has overrides	
	while vidro.current_rc_channels[4] > 1600 and flight_ready == True:
		if(reset_val):
			print 'Reset Over-rides'
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
		
		
		print('Armed Loop')
		#Update of gains before going into control loop
		if vidro.current_rc_channels[5] > 1600:
			controller.update_gains() # Potentially move this into the main loop

		# Auto Loop
		while vidro.current_rc_channels[5] > 1600:
			vidro.update_mavlink() # Grab updated rc channel values
			print('Auto Loop')

			# Seq. 0: Takeoff to 1 m and origin ###############=> Separate these commands maybe....maybe in all of them, just make a positioning thread maybe			
			if sequence == 0:
				# Assign Commands
				alt_com = take_off_alt
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				#error_x_y = controller.rc_xy(x_com, y_com)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(round(error_z))+' Err Yaw: '+repr(round(error_yaw))+' Err X: '+repr(round(err_x))+' Err y: '+repr(round(err_y)))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error 
					seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					if seq0_cnt == 10:
						sequence = 1

			#Seq. 1: Coarse Search
			if sequence == 1:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				#error_x_y = controller.rc_xy(x_com, y_com)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(round(error_z))+' Err Yaw: '+repr(round(error_yaw))+' Err X: '+repr(round(err_x))+' Err y: '+repr(round(err_y)))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error 
					# Hold her steady while we img proc					
					vidro.set_rc_throttle(vidro.base_rc_throttle)
					vidro.set_rc_roll(vidro.base_rc_roll)
					vidro.set_rc_pitch(vidro.base_rc_pitch)
					vidro.set_rc_yaw(vidro.base_rc_yaw)					
					# Run img proc					
					yaw_com +=yaw_coarse_step
					yaw_pos = vidro.get_yaw_radians() # Grab current yaw val, assuming picture taking could be a while so grab it here son
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
					if (yaw_com > 2 * math.pi): # Check if we've done a full circle
						yaw_com-=(2*math.pi) # Keep it positive
						if(balloon_found): # Advance if we've found the balloon in our rotation
							sequence = 2
							yaw_com = max_bear_val # Set our next yaw command to where we saw the biggest return
						
			#Seq. 2: Fine Search ######################## Could just kill this sequence....
			if sequence == 2:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				#error_x_y = controller.rc_xy(x_com, y_com)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(round(error_z))+' Err Yaw: '+repr(round(error_yaw))+' Err X: '+repr(round(err_x))+' Err y: '+repr(round(err_y)))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error 
					# Hold her steady while we img proc					
					vidro.set_rc_throttle(vidro.base_rc_throttle)
					vidro.set_rc_roll(vidro.base_rc_roll)
					vidro.set_rc_pitch(vidro.base_rc_pitch)
					vidro.set_rc_yaw(vidro.base_rc_yaw)					
					# Run img proc						
					# Check to see if we still see the Balloon					
					yaw_pos = vidro.get_yaw_radians() # Update our radians					
					get_camera_frame()
					cx_val,cy_val,area_val,num_objects_val = get_object(frame)
					if(num_objects_val>0):					# If we've seen something, adjust our pointing
						img_balloon_ber = (cx_val-cx_mid)*cx_fov/640    # Basic camera model, somebody check this
						if(abs(img_balloon_ber)<yaw_bound_err):		# If we're in the error, advance and update bear est.							
							sequence=3
							max_bear_val = yaw_pos+img_balloon_ber
							if(max_bear_val<0):
								max_bear_val+=(2*math.pi) 	# Keep it positive
						else:						# Else, adjust our yaw_com
							max_bear_val = yaw_pos+img_balloon_ber
							if(max_bear_val<0):
								max_bear_val+=(2*math.pi)
							yaw_com = max_bear_val
					else:
						balloon_found = False 				# If we don't see anything go back to coarse search
						sequence = 1
						area_max_val = 0
			
			#Seq. 3: Guidance
			if sequence == 3:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(yaw_com)
				#error_x_y = controller.rc_xy(x_com, y_com)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(round(error_z))+' Err Yaw: '+repr(round(error_yaw))+' Err X: '+repr(round(err_x))+' Err y: '+repr(round(err_y)))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error 
					# Hold her steady while we img proc					
					vidro.set_rc_throttle(vidro.base_rc_throttle)
					vidro.set_rc_roll(vidro.base_rc_roll)
					vidro.set_rc_pitch(vidro.base_rc_pitch)
					vidro.set_rc_yaw(vidro.base_rc_yaw)					
					# Run img proc						
					# Check to see if we still see the Balloon					
					yaw_pos = vidro.get_yaw_radians() # Update our radians					
					get_camera_frame()
					cx_val,cy_val,area_val,num_objects_val = get_object(frame)
					if(num_objects_val>0):					# If we've seen something, adjust our pointing
						img_balloon_ber = (cx_val-cx_mid)*cx_fov/640   # Basic camera model, somebody check this
						max_bear_val = yaw_pos+img_balloon_ber
						if(max_bear_val<0):
							max_bear_val+=(2*math.pi) 		# Keep it positive
						yaw_com = max_bear_val
						if(area_val>balloon_area_max_thresh): 		# If the balloon is taking up most of our FOV, advance
							sequence = 4 
						else:						# Else, update x,y com
							# Grab our current x,y position          
							x_pos=vidro.get_position()[0]            
							y_pos=vidro.get_position()[1]            
							# Calculate step towards balloon         
							d_x = guid_adv*math.cos(max_bear_val)    
							d_y = guid_adv*math.sin(max_bear_val)    
							x_com = x_pos+d_x			 
							y_com = y_pos+d_y	
					else:
						balloon_found = False 				# If we don't see anything go back to coarse search
						sequence = 1
						area_max_val = 0						
			
			#Seq. 4: Terminal
			if sequence == 4:
				cv2.imwrite("terminal_pic.jpg",frame) # For now, just take a picture
				sequence=5

			#Move to landing position
			if sequence == 5:
				error_z = controller.rc_alt(alt_com)
				error_yaw = controller.rc_yaw(0)
				#error_x_y = controller.rc_xy(x_com, y_com)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(round(error_z))+' Err Yaw: '+repr(round(error_yaw))+' Err X: '+repr(round(err_x))+' Err y: '+repr(round(err_y)))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error 
					alt_com-=1
					if(alt_com<170):
						sequence = 6

			# Land
			if sequence == 6: 
				controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
				controller.vidro.rc_throttle_reset()
				controller.vidro.rc_yaw_reset()
				controller.vidro.rc_pitch_reset()
				controller.vidro.rc_roll_reset()
				reset_val = 0
				vidro.close()
				break	# this break won't break all the loops, just the auto loop			

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1



