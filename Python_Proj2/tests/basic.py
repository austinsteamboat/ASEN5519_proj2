"""
Simple pymavlink example to test getting and sending RC Values
"""

from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import matplotlib.pyplot as plot

baud = 115200
device = "127.0.0.1:14551"

channel_1 = None

master = mavutil.mavlink_connection(device, baud)
msg = master.recv_match(type='HEARTBEAT', blocking=True)
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_system))
while (channel_1 == None):
	msg = master.recv_match(type='RC_CHANNELS_RAW', blocking=True)
	channel_1 = msg.chan1_raw
print("Got RC Channels")

while True:
	msg = master.recv_match(blocking=False)
	if msg:
		if msg.get_type() == "BAD_DATA":
			if mavutil.all_printable(msg.data):
				print "Whoops, got bad data", msg.data
		if msg.get_type() == "RC_CHANNELS_RAW":
			channel_1 = msg.chan1_raw
			print channel_1

		master.mav.rc_channels_override_send(master.target_system, master.target_component, channel_1+1.0, 0, 0, 0, 0, 0, 0, 0)


	time.sleep(.01)
