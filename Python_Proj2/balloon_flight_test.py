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
pos_bound_err = 100
yaw_bound_err = 0.2
yaw = 0
reset_val = 1
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
		print('outer loop')
		#Update of gains before going into control loop
		if vidro.current_rc_channels[5] > 1600:
			controller.update_gains()

		while vidro.current_rc_channels[5] > 1600:
			vidro.update_mavlink() # Grab updated rc channel values
			#On ground -> Takeoff to 1 m
			if sequence == 0:
				error_z = controller.rc_alt(1000)
				error_yaw = controller.rc_yaw(0)
				#error_x_y = controller.rc_xy(0, 0)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					print('update')
					if seq0_cnt == 10:
						sequence = 2

			#Turn in center of space
			if sequence == 1:
				error_z = controller.rc_alt(1000)
				error_yaw = controller.rc_yaw(yaw)
				#error_x_y = controller.rc_xy(0,0)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					yaw +=1
					seq1_cnt += 1 # again just update the sequence once we've closed the loop
					print('update')
					if yaw > 2 * math.pi:
						yaw -= (2 * math.pi)					
					if seq1_cnt==20: # once we've closed the loop for 20 steps, we're done
						sequence = 2
						
			#Move to landing position
			if sequence == 2:
				error_z = controller.rc_alt(desc_alt)
				error_yaw = controller.rc_yaw(0)
				#error_x_y = controller.rc_xy(0, 0)
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					print('update')
					seq2_cnt += 1
					#if seq2_cnt==3: # once we've closed the loop 5 times, advance!
					desc_alt-=1
					if(desc_alt<170):
						sequence = 3

			# Land
			if sequence == 3: 
				controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
				#print(' RC 1 '+repr(vidro.current_rc_channels[0])+' RC 2 '+repr(vidro.current_rc_channels[1])+' RC 3 '+repr(vidro.current_rc_channels[2])+' RC 4 '+repr(vidro.current_rc_channels[3])+' RC 5 '+repr(vidro.current_rc_channels[4])+' RC 6 '+repr(vidro.current_rc_channels[5]))
				sequence = 3

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




