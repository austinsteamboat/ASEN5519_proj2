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
pos_bound_err = 100
yaw_bound_err = 0.2
logging_on = True
yaw = 0
reset_val = 1
time_begin = time.time()


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
	error_z = controller.rc_alt(z_com)
	error_yaw = controller.rc_yaw(yaw_com)
	error_x_y = controller.rc_xy(x_com, y_com)
	error_data = [error_x_y[2],error_x_y[3],error_z,error_x_y[0],error_x_y[1],error_yaw]
	return error_data

def logger(log_true,sequence,pwm_data,pos_data,error_data):
	if log_true:
		msg_type = ' SEQ '
		log_time = time.time()-start_time
		logging.info(repr(log_time)+','+repr(sequence)+','+repr(pwm_data[0])+','+repr(pwm_data[1])+','+repr(pwm_data[2])+','repr(pwm_data[3])+','+repr(pos_data[0])+','+repr(pos_data[1])+','+repr(pos_data[2])+','+repr(pos_data[3])+','+repr(error_data[0])+','+repr(error_data[1])+','+repr(error_data[2]+','+repr(error_data[3])+','+repr(error_data[4])+','+repr(error_data[5]))

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
	if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(error_y) < pos_bound_err) and (abs(error_x) < pos_bound_err)):
		x_out = True
	else:
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
		print('outer loop'+' Roll: '+repr(vidro.current_rc_channels[0])+' Pitch: '+repr(vidro.current_rc_channels[1])+repr(logging_on))
		alt_com = 0		
		yaw_com = 0		
		x_com = 0
		y_com = 0
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
                        logger(loggin_on,pwm_log_data,pos_log_data,err_log_data)
			#pwm_logger(logging_on,pwm_log_data)
			#pos_logger(logging_on,pos_log_data)
			#err_logger(logging_on,err_log_data)
			print(' X_err: '+repr(err_log_data[0])+' Y_err: '+repr(err_log_data[1])+' Z_err: '+repr(err_log_data[2])+' Roll_err: '+repr(err_log_data[3])+' Pitch_err: '+repr(err_log_data[4])+' Yaw_err: '+repr(err_log_data[5]))

			#On ground -> Takeoff to 1 m
			if sequence == 0:
				alt_com = 1000
				yaw_com = 0
				x_com = 0
				y_com = 0
				sequence_logger(logging_on,sequence)
				check_val = err_check(err_log_data,pos_bound_err,yaw_bound_err)
				if (check_val):# Closes Error for takeoff
					seq0_cnt += 1 # just update the sequence if the loop is closed for 3 software loops
					print('update')
					if seq0_cnt == 10:
						sequence = 0

			#Turn in center of space
			if sequence == 1:
				# Hold her steady while we img proc					
				vidro.set_rc_throttle(vidro.base_rc_throttle)
				vidro.set_rc_roll(vidro.base_rc_roll)
				vidro.set_rc_pitch(vidro.base_rc_pitch)
				vidro.set_rc_yaw(vidro.base_rc_yaw)
				#error_z = controller.rc_alt(1000)
				#error_yaw = controller.rc_yaw(yaw)
				#error_x_y = controller.rc_xy(0,0)
				error_z = 0
				error_yaw = 0
				err_x = 0#error_x_y[0]
				err_y = 0#error_x_y[1]
				print('Seq: '+repr(sequence)+' Err Z: '+repr(error_z)+' Err Yaw: '+repr(error_yaw)+' Err X: '+repr(err_x)+' Err y: '+repr(err_y))
				if ((abs(error_z) < pos_bound_err) and (abs(error_yaw) < yaw_bound_err) and (abs(err_y) < pos_bound_err) and (abs(err_x) < pos_bound_err)):# Closes Error for takeoff
					yaw +=1
					seq1_cnt += 1 # again just update the sequence once we've closed the loop
					print('update')
					if yaw > 2 * math.pi:
						yaw -= (2 * math.pi)					
					if seq1_cnt==5: # once we've closed the loop for 20 steps, we're done
						sequence = 2
						time2 = time.time()
						print 'Duration: '+repr(time2-time1)
						
			#Move to landing position
			if sequence == 2:
				error_z = controller.rc_alt(desc_alt)
				error_yaw = controller.rc_yaw(1.57079633)
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
				logging_on = False
				#print(' RC 1 '+repr(vidro.current_rc_channels[0])+' RC 2 '+repr(vidro.current_rc_channels[1])+' RC 3 '+repr(vidro.current_rc_channels[2])+' RC 4 '+repr(vidro.current_rc_channels[3])+' RC 5 '+repr(vidro.current_rc_channels[4])+' RC 6 '+repr(vidro.current_rc_channels[5]))
				sequence = 3

			if vidro.current_rc_channels[5] < 1600:
				reset_val = 1
				print('Gonna Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')




