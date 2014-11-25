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
seq_time = 0
prev_seq_time = 0
time_diff = 0
yaw = 0
print('Heading to while')
while flight_ready == True:#1600 vidro.current_rc_channels[4] > 0 and 
	print('RC 5 '+repr(vidro.current_rc_channels[4])+' RC 6 '+repr(vidro.current_rc_channels[5]))
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
	if True: # vidro.current_rc_channels[5] > 0
		controller.update_gains()

	while True: # vidro.current_rc_channels[5] > 0
		#On ground
		if sequence == 0:
			error_z = controller.rc_alt(1000)
			error_yaw = controller.rc_yaw(0)
			error_pitch, error_roll = controller.rc_xy(0, 0)
			alt_cur_val = controller.vidro.get_position()[2]
			seq_time = time.time()
			print(' error z '+repr(error_z)+' error yaw '+repr(error_yaw)+' error Pitch '+repr(error_pitch)+' cur alt '+repr(alt_cur_val))
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
				print('debug2')
				
				prev_seq_time = seq_time
				debug_val = (time.time() % 1)
				print(repr(debug_val))
				if () == 0:
					print('seq0')
					seq0_cnt += 1
					if seq0_cnt == 3:
						sequence = 1

		#Hover in center of space
		if sequence == 1:
			error_z = controller.rc_alt(1000)
			error_yaw = controller.rc_yaw(0)
			error_x_y = controller.rc_xy(0,0)
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
				if (time.time() % 1) == 0:
					seq1_cnt += 1
					print('seq1')
					if seq1_cnt == 3:
						sequence = 2

		#Hover in center of space
		if sequence == 2:
			error_z = controller.rc_alt(1000)
			error_yaw = controller.rc_yaw(yaw)
			error_x_y = controller.rc_xy(0,0)
			if error_z < 50 and error_yaw < 1 and error_pitch < 50 and error_roll < 50:
				if (time.time() % 1) == 0:
					seq2_cnt += 1
					print('seq2')
					yaw +=1
					print(repr(yaw))
					if yaw > 2 * math.pi:
						yaw -= (2 * math.pi)
					if seq2_cnt==20:
						sequence = 3
					

	       
		#Land
		if sequence == 3:
			error_z = controller.rc_alt(150)
			error_yaw = controller.rc_yaw(0)
			error_x_y = controller.rc_xy(0, 0)

			current_time = time.time()
			cycle += 1
			previous_time = current_time

