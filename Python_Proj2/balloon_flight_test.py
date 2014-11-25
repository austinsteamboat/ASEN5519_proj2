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
yaw = 0
print('Heading to while')
while vidro.current_rc_channels[4] > 1600 and flight_ready == True:
	update_mavlink(controller) # Grab updated rc channel values
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

	#Update of gains before going into control loop
	if vidro.current_rc_channels[5] > 1600:
		controller.update_gains()

	while vidro.current_rc_channels[5] > 1600:
		update_mavlink(controller) # grab updated value for rc channels I think
		#On ground -> Takeoff to 1 m
		if sequence == 0:
			error_z = controller.rc_alt(1000)
			error_yaw = controller.rc_yaw(0)
			error_pitch, error_roll = controller.rc_xy(0, 0)
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:# Closes Error for takeoff
				seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
				if seq0_cnt == 3:
					sequence = 1

		#Turn in center of space
		if sequence == 1:
			error_z = controller.rc_alt(1000)
			error_yaw = controller.rc_yaw(yaw)
			error_x_y = controller.rc_xy(0,0)
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50: # if we've closed the error, update
				yaw +=1
				seq1_cnt += 1 # again just update the sequence once we've closed the loop
				print(repr(yaw))
				if yaw > 2 * math.pi:
					yaw -= (2 * math.pi)					
				if seq1_cnt==20: # once we've closed the loop for 20 steps, we're done
					sequence = 2
					
		#Move to landing position
		if sequence == 2:
			error_z = controller.rc_alt(170)
			error_yaw = controller.rc_yaw(0)
			error_x_y = controller.rc_xy(0, 0)
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50: # if we've closed the error, update
				seq2_cnt += 1
				if seq2_cnt==5 # once we've closed the loop 5 times, advance!
					sequence = 3
		
		if sequence == 3: land
			controller.vidro.set_rc_throttle(controller,0)#not 100% on this
			sequence = 3






