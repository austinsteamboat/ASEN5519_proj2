"""
Simple test for connecting to the vidro class. Some things like time may not be working
"""

from vidro import Vidro, ViconStreamer
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

vidro = Vidro(True, 115200,"127.0.0.1:14551")
vidro.connect()
cycles_message = 0
cycles_change = 0
previous_time = 0
previous_rc = 0
while vidro.current_rc_channels[4] > 1500:
	while vidro.current_rc_channels[2] < 1800:
		print str(vidro.current_rc_channels[2]) + "   " + str(vidro.rc_msg_time) + "   " + str(cycles_message) + "   " + str(cycles_change)
		vidro.set_rc_throttle(vidro.current_rc_channels[2]+10)
		vidro.get_mavlink()
		time.sleep(.01)

		if vidro.rc_msg_time < previous_time:
			cycles_message = 0

		if vidro.current_rc_channels[2] != previous_rc:
			cycles_change = 0

		cycles_change += 1
		cycles_message += 1
		previous_time = vidro.rc_msg_time
		previous_rc = vidro.current_rc_channels[2]

	while vidro.current_rc_channels[2] > 1300:
		print str(vidro.current_rc_channels[2]) + " " + str(vidro.rc_msg_time) + "   " + str(cycles_message) + "   " + str(cycles_change)
		vidro.set_rc_throttle(vidro.current_rc_channels[2]-10)
		vidro.get_mavlink()
		time.sleep(.01)

		if vidro.rc_msg_time < previous_time:
			cycles_message = 0

		if vidro.current_rc_channels[2] != previous_rc:
			cycles_change = 0

		cycles_change += 1
		cycles_message += 1
		previous_time = vidro.rc_msg_time
		previous_rc = vidro.current_rc_channels[2]

	vidro.get_mavlink()
	time.sleep(.01)
vidro.close()
