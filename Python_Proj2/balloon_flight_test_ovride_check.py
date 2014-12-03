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
yaw_bound_err = 0.5
yaw = 0
reset_val = 1
print('Heading to while')
while(1):
	vidro.update_mavlink() # Grab updated rc channel values. This is the right command for it, but it doesn't always seem to update RC channels
	print('RC 5 Vidro Update '+repr(vidro.current_rc_channels[4])+' RC 6 Vidro Update '+repr(vidro.current_rc_channels[5])+' RC Throttle '+repr(vidro.current_rc_channels[2]))
	while vidro.current_rc_channels[4] > 1600 and flight_ready == True:
		if(reset_val):
			print 'RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
			controller.vidro.rc_throttle_reset()
			reset_val = 0

		print('RC 5 Vidro Update '+repr(vidro.current_rc_channels[4])+' RC 6 Vidro Update '+repr(vidro.current_rc_channels[5])+' RC Throttle '+repr(vidro.current_rc_channels[2]))
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
		print('outer loop')
		#Update of gains before going into control loop
		if vidro.current_rc_channels[5] > 1600:
			controller.update_gains()

		while vidro.current_rc_channels[5] > 1600:
			vidro.update_mavlink() # Grab updated rc channel values
			# Set throttle command
			controller.vidro.set_rc_throttle(0)# it'll round it to minimum which is like 1100
			print('RC 5 Vidro Update '+repr(vidro.current_rc_channels[4])+' RC 6 Vidro Update '+repr(vidro.current_rc_channels[5])+' RC Throttle '+repr(vidro.current_rc_channels[2]))
			print('inner loop')
			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			






