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
print('Heading to while')
while(1):
	#print("Position: X: " + repr(vidro.get_position()[0]) + " Y: " + repr(vidro.get_position()[1]) + " Z: " + repr(vidro.get_position()[2])+ " Yaw: " + repr(vidro.get_yaw_radians())+ " Pitch: " + repr(vidro.get_pitch())+ " Roll: " + repr(vidro.get_roll()))
	print repr(vidro.get_yaw_radians())

