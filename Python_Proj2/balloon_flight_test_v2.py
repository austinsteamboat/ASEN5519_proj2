#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import numpy as np
import cv2


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
pos_bound_err = 300
yaw_bound_err = 0.2
yaw_com = 0
alt_com = 1350
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
					if seq1_cnt == 3:
						roll_idle = vidro.current_rc_channels[0]
						pitch_idle = vidro.current_rc_channels[1]
						throttle_idle = vidro.current_rc_channels[2]
						yaw_idle = vidro.current_rc_channels[3]	
						seq1_cnt = 0
						yaw_com+=yaw_coarse_step
						print "Yaw Com: "+repr(yaw_com)
						if (yaw_com > 2 * math.pi): # Check if we've done a full circle
							yaw_com-=(2*math.pi) # Keep it positive	
							sequence = 2
							#yaw_com = 0			
						
			#Move to landing position
			if sequence == 2:
				error_z = controller.rc_alt(desc_alt)
				error_yaw = controller.rc_yaw(yaw_com)
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
			if sequence == 3: 
				controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
				controller.vidro.rc_throttle_reset()
				controller.vidro.rc_yaw_reset()
				controller.vidro.rc_pitch_reset()
				controller.vidro.rc_roll_reset()
				sequence = 3

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




